// SPDX-License-Identifier: BSD-3-Clause

use std::{f32::consts::PI, io, ops::Range};

use crate::{
    airspy::Complex,
    config::{Mode, invalid},
};

const MAX_TAPS: usize = 4_097;
const AGC_RATE: f32 = 0.0003;
const AGC_MAX: f32 = 1_000_000.0;

pub struct Demod {
    decim: Fir,
    side: Fir,
    factor: usize,
    phase: usize,
    osc: Complex,
    rotation: Complex,
    ticks: usize,
    agc: Agc,
}

impl Demod {
    pub fn new(
        input_rate: u32,
        output_rate: u32,
        band: Range<f32>,
        mode: Mode,
        agc: bool,
        gain: f32,
    ) -> io::Result<Self> {
        if output_rate == 0 || input_rate < output_rate || !input_rate.is_multiple_of(output_rate) {
            return Err(invalid(format!(
                "device rate {input_rate} must be an integer multiple of output rate {output_rate}"
            )));
        }
        let low_cut = band.start;
        let high_cut = band.end;
        if !low_cut.is_finite()
            || !high_cut.is_finite()
            || low_cut <= 0.0
            || high_cut <= low_cut
            || high_cut >= output_rate as f32 * 0.5
        {
            return Err(invalid("invalid audio passband"));
        }

        let factor = (input_rate / output_rate) as usize;
        let decim = if factor == 1 {
            Fir::new(vec![1.0])
        } else {
            let transition = output_rate as f32 - 2.0 * high_cut;
            let taps = ntaps(8.0 * input_rate as f32 / transition, "lower AJD_HIGH_CUT")?;
            Fir::new(lowpass(taps, output_rate as f32 * 0.5, input_rate as f32))
        };

        // Mix to DC w/ 2*low_cut transition between that and the sideband
        let side_taps = ntaps(4.0 * output_rate as f32 / low_cut, "raise AJD_LOW_CUT")?;
        let center = (low_cut + high_cut) * 0.5;
        let side = Fir::new(lowpass(side_taps, center, output_rate as f32));
        let direction = match mode {
            Mode::Usb => 1.0,
            Mode::Lsb => -1.0,
        };
        let angle = direction * 2.0 * PI * center / output_rate as f32;

        Ok(Self {
            decim,
            side,
            factor,
            phase: 0,
            osc: Complex { re: 1.0, im: 0.0 },
            rotation: Complex {
                re: angle.cos(),
                im: angle.sin(),
            },
            ticks: 0,
            agc: Agc::new(agc, gain),
        })
    }

    pub fn process(&mut self, input: &[Complex], mut output: Option<&mut Vec<i16>>) -> usize {
        let mut frames = 0;
        for &sample in input {
            self.decim.push(sample);
            if self.phase == 0 {
                let sample = self.decim.output();
                let mixed = mulc(sample, self.osc);
                self.side.push(mixed);
                let filtered = mul(self.side.output(), self.osc);
                let audio = self.agc.process(filtered).clamp(-1.0, 1.0);

                if let Some(buf) = output.as_deref_mut() {
                    buf.push((audio * i16::MAX as f32).round() as i16);
                }
                frames += 1;
                self.advance();
            }
            self.phase += 1;
            if self.phase == self.factor {
                self.phase = 0;
            }
        }
        frames
    }

    pub fn skip(&mut self, input_frames: u64) -> u64 {
        if input_frames == 0 {
            return 0;
        }

        let factor = self.factor as u64;
        let phase = self.phase as u64;
        let first = (factor - phase) % factor;
        let output_frames = if first >= input_frames {
            0
        } else {
            1 + (input_frames - first - 1) / factor
        };
        self.phase = ((phase + input_frames % factor) % factor) as usize;
        self.decim.clear();
        self.side.clear();
        for _ in 0..output_frames {
            self.advance();
        }
        output_frames
    }

    fn advance(&mut self) {
        self.osc = mul(self.osc, self.rotation);
        self.ticks += 1;
        if self.ticks & 1023 == 0 {
            let norm = (self.osc.re * self.osc.re + self.osc.im * self.osc.im).sqrt();
            if norm.is_finite() && norm > 0.0 {
                self.osc.re /= norm;
                self.osc.im /= norm;
            } else {
                self.osc = Complex { re: 1.0, im: 0.0 };
            }
        }
    }
}

struct Fir {
    taps: Box<[f32]>,
    ring: Box<[Complex]>,
    pos: usize,
}

impl Fir {
    fn new(taps: Vec<f32>) -> Self {
        let len = taps.len();
        Self {
            taps: taps.into_boxed_slice(),
            ring: vec![Complex::default(); len * 2].into_boxed_slice(),
            pos: 0,
        }
    }

    fn push(&mut self, sample: Complex) {
        let len = self.taps.len();
        self.ring[self.pos] = sample;
        self.ring[self.pos + len] = sample;
        self.pos += 1;
        if self.pos == len {
            self.pos = 0;
        }
    }

    fn clear(&mut self) {
        self.ring.fill(Complex::default());
        self.pos = 0;
    }

    fn output(&self) -> Complex {
        let len = self.taps.len();
        let half = len / 2;
        let window = &self.ring[self.pos..self.pos + len];
        let mut re = window[half].re * self.taps[half];
        let mut im = window[half].im * self.taps[half];

        for i in 0..half {
            let pair_re = window[i].re + window[len - 1 - i].re;
            let pair_im = window[i].im + window[len - 1 - i].im;
            re += pair_re * self.taps[i];
            im += pair_im * self.taps[i];
        }
        Complex { re, im }
    }
}

struct Agc {
    enabled: bool,
    gain: f32,
    level: f32,
}

impl Agc {
    const fn new(enabled: bool, level: f32) -> Self {
        Self {
            enabled,
            gain: 1.0,
            level,
        }
    }

    fn process(&mut self, sample: Complex) -> f32 {
        if !self.enabled {
            return sample.re * self.level;
        }

        let re = sample.re * self.gain;
        let im = sample.im * self.gain;
        let change = 1.0 + AGC_RATE * (1.0 - (re * re + im * im));
        self.gain = (self.gain * change).clamp(f32::MIN_POSITIVE, AGC_MAX);
        if !self.gain.is_finite() {
            self.gain = 1.0;
        }
        re * self.level
    }
}

fn ntaps(estimate: f32, hint: &str) -> io::Result<usize> {
    if !estimate.is_finite() || estimate <= 0.0 {
        return Err(invalid("invalid FIR transition width"));
    }
    let mut taps = estimate.ceil() as usize;
    taps = taps.max(3) | 1;
    if taps > MAX_TAPS {
        return Err(invalid(format!("filter requires {taps} taps; {hint}")));
    }
    Ok(taps)
}

fn lowpass(taps: usize, cutoff: f32, rate: f32) -> Vec<f32> {
    debug_assert!(taps > 0 && taps & 1 == 1);
    debug_assert!(cutoff > 0.0 && cutoff < rate * 0.5);

    let middle = (taps / 2) as isize;
    let scale = 2.0 * cutoff / rate;
    let mut coeff = Vec::with_capacity(taps);
    for index in 0..taps {
        let offset = index as isize - middle;
        let sinc = if offset == 0 {
            scale
        } else {
            let x = PI * offset as f32;
            (scale * x).sin() / x
        };
        let phase = 2.0 * PI * index as f32 / (taps - 1) as f32;
        let window = 0.42 - 0.5 * phase.cos() + 0.08 * (2.0 * phase).cos();
        coeff.push(sinc * window);
    }
    let sum: f32 = coeff.iter().sum();
    for value in &mut coeff {
        *value /= sum;
    }
    coeff
}

const fn mul(lhs: Complex, rhs: Complex) -> Complex {
    Complex {
        re: lhs.re * rhs.re - lhs.im * rhs.im,
        im: lhs.re * rhs.im + lhs.im * rhs.re,
    }
}

const fn mulc(lhs: Complex, rhs: Complex) -> Complex {
    Complex {
        re: lhs.re * rhs.re + lhs.im * rhs.im,
        im: lhs.im * rhs.re - lhs.re * rhs.im,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(freq: f32, rate: u32, len: usize) -> Vec<Complex> {
        (0..len)
            .map(|index| {
                let phase = 2.0 * PI * freq * index as f32 / rate as f32;
                Complex {
                    re: phase.cos(),
                    im: phase.sin(),
                }
            })
            .collect()
    }

    fn rms(mode: Mode, freq: f32) -> f32 {
        const RATE: u32 = 12_000;
        let input = tone(freq, RATE, RATE as usize);
        let mut decoder = Demod::new(RATE, RATE, 500.0..2_500.0, mode, false, 1.0).unwrap();
        let mut samples = Vec::new();
        decoder.process(&input, Some(&mut samples));
        let samples: Vec<_> = samples
            .iter()
            .map(|&sample| f32::from(sample) / f32::from(i16::MAX))
            .skip(RATE as usize / 4)
            .collect();
        (samples.iter().map(|sample| sample * sample).sum::<f32>() / samples.len() as f32).sqrt()
    }

    #[test]
    fn fir_gain() {
        let coeff = lowpass(101, 1_000.0, 12_000.0);
        assert!((coeff.iter().sum::<f32>() - 1.0).abs() < 1e-5);
        for index in 0..coeff.len() / 2 {
            assert!((coeff[index] - coeff[coeff.len() - 1 - index]).abs() < f32::EPSILON);
        }
    }

    #[test]
    fn decim_phase() {
        let mut decoder =
            Demod::new(48_000, 12_000, 500.0..2_500.0, Mode::Usb, false, 1.0).unwrap();
        assert_eq!(decoder.process(&[Complex::default(); 7], None), 2);
        assert_eq!(decoder.process(&[Complex::default(); 9], None), 2);
    }

    #[test]
    fn skip_reset() {
        let mut decoder =
            Demod::new(48_000, 12_000, 500.0..2_500.0, Mode::Usb, false, 1.0).unwrap();
        assert_eq!(decoder.process(&[Complex { re: 1.0, im: 1.0 }; 7], None), 2);
        assert_eq!(decoder.skip(2), 1);
        assert!(
            decoder
                .decim
                .ring
                .iter()
                .all(|sample| *sample == Complex::default())
        );
        assert!(
            decoder
                .side
                .ring
                .iter()
                .all(|sample| *sample == Complex::default())
        );
        assert_eq!(decoder.process(&[Complex::default(); 3], None), 0);
    }

    #[test]
    fn sideband() {
        let usb = rms(Mode::Usb, 1_500.0);
        let usb_mirror = rms(Mode::Usb, -1_500.0);
        let lsb = rms(Mode::Lsb, -1_500.0);
        let lsb_mirror = rms(Mode::Lsb, 1_500.0);

        assert!(usb > 0.6, "USB level was {usb}");
        assert!(lsb > 0.6, "LSB level was {lsb}");
        assert!(usb_mirror < 0.01, "USB mirror level was {usb_mirror}");
        assert!(lsb_mirror < 0.01, "LSB mirror level was {lsb_mirror}");
    }
}
