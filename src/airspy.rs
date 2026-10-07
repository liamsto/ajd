// SPDX-License-Identifier: BSD-3-Clause
// USB protocol and sample correction ported from airspyhf/libairspyhf/src.
/*
Copyright (c) 2013-2023, Youssef Touil <youssef@airspy.com>
Copyright (c) 2013-2017, Ian Gilmour <ian@sdrsharp.com>
Copyright (c) 2013-2017, Benjamin Vernoux <bvernoux@airspy.com>
Copyright (c) 2013, Michael Ossmann <mike@ossmann.com>
Copyright (c) 2012, Jared Boone <jared@sharebrained.com>
Copyright (c) 2018, Leif Asbrink <leif@sm5bsz.com>

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of Airspy HF+ nor the names of its contributors may be used
   to endorse or promote products derived from this software without specific
   prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

use std::{
    f64::consts::TAU,
    io,
    ops::ControlFlow,
    panic::{AssertUnwindSafe, catch_unwind, resume_unwind},
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    thread,
    time::{Duration, Instant},
};

use nusb::{
    Interface, MaybeFuture,
    transfer::{Bulk, ControlIn, ControlOut, ControlType, In, Recipient, TransferError},
};

use crate::config::{Config, RATE, invalid};

pub const BLOCK: usize = 4096;
const BYTES: usize = BLOCK * 4;
const TRANSFERS: usize = 16;
const BUFFERS: usize = 8;
const TIMEOUT: Duration = Duration::from_millis(500);

const RX_MODE: u8 = 1;
const SET_FREQ: u8 = 2;
const GET_RATES: u8 = 3;
const SET_RATE: u8 = 4;
const READ_CONFIG: u8 = 5;
const SET_AGC: u8 = 10;
const SET_AGC_THRESHOLD: u8 = 11;
const SET_ATT: u8 = 12;
const GET_ARCH: u8 = 14;
const GET_GAIN: u8 = 15;
const GET_DELTA: u8 = 16;
const SET_VCTCXO: u8 = 17;
const SET_FRONTEND: u8 = 18;
const GET_ATT: u8 = 19;
const SET_BIAS: u8 = 22;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Complex {
    pub re: f32,
    pub im: f32,
}

pub struct Device {
    iface: Interface,
    pub rate: u32,
    mixer: Mixer,
}

impl Device {
    pub fn open(config: &Config) -> io::Result<Self> {
        let info = nusb::list_devices()
            .wait()?
            .find(|dev| {
                dev.vendor_id() == 0x03eb
                    && dev.product_id() == 0x800c
                    && config
                        .serial
                        .is_none_or(|sn| dev.serial_number().and_then(serial) == Some(sn))
            })
            .ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::NotFound,
                    "Airspy HF+ not found (check AJD_SERIAL)",
                )
            })?;
        let usb = info.open().wait()?;
        if !matches!(usb.active_configuration(), Ok(cfg) if cfg.configuration_value() == 1) {
            usb.set_configuration(1).wait()?;
        }
        let iface = usb.claim_interface(0).wait()?;
        iface.set_alt_setting(1).wait()?;
        write(&iface, RX_MODE, 0, 0, &[])?;

        let rates = words(&iface, GET_RATES)?.unwrap_or_else(|| vec![768_000]);
        let rate = rate(&rates, config.device_rate)?;
        let index = rates.iter().position(|&r| r == rate).unwrap() as u16;
        let low_if = read(&iface, GET_ARCH, rates.len() as u16, rates.len() as u16)?
            .is_some_and(|arch| arch[index as usize] != 0);
        // valid zero-IF LO before switching sample rates?
        if !low_if {
            write(&iface, SET_FREQ, 0, 0, &180_u32.to_be_bytes())?;
        }
        write(&iface, SET_RATE, 0, index, &[])?;
        let gain =
            read(&iface, GET_GAIN, 0, 1)?.map_or(1.0, |v| 10_f32.powf(-0.05 * f32::from(v[0])));

        let mut ppb = 0;
        if let Some(flash) = read(&iface, READ_CONFIG, 0, 256)?
            && u32::from_le_bytes(flash[..4].try_into().unwrap()) == 0xa5ca71b0
        {
            ppb = i32::from_le_bytes(flash[4..8].try_into().unwrap());
            let vc = u16::from_le_bytes(flash[8..10].try_into().unwrap());
            let flags = u32::from_le_bytes(flash[12..16].try_into().unwrap());
            for (cmd, value, index) in [
                (SET_VCTCXO, vc, 0),
                (SET_FRONTEND, flags as u16, (flags >> 16) as u16),
            ] {
                match write(&iface, cmd, value, index, &[]) {
                    Ok(()) | Err(TransferError::Stall) => {} // old
                    Err(error) => return Err(error.into()),
                }
            }
        }
        let (khz, mut shift) = tune(config.frequency, config.calibration.unwrap_or(ppb), low_if)?;
        write(&iface, SET_FREQ, 0, 0, &khz.to_be_bytes())?;
        if let Some(data) = read(&iface, GET_DELTA, 0, 4)? {
            shift += delta(data.try_into().unwrap())?;
        }

        write(
            &iface,
            SET_AGC,
            u16::from(config.attenuation.is_none() && config.hf_agc),
            0,
            &[],
        )?;
        let att = config.attenuation.unwrap_or(0.0);
        let att_index = if att == 0.0 {
            0
        } else {
            let steps: Vec<_> = words(&iface, GET_ATT)?.map_or_else(
                || (0..9).map(|i| i as f32 * 6.0).collect(),
                |words| words.into_iter().map(f32::from_bits).collect(),
            );
            attidx(&steps, att)?
        };
        write(&iface, SET_ATT, att_index, 0, &[])?;
        if let Some(high) = config.hf_agc_high {
            write(&iface, SET_AGC_THRESHOLD, u16::from(high), 0, &[])?;
        }
        if let Some(bias) = config.bias_tee {
            write(&iface, SET_BIAS, bias as u16, 0, &[])?;
        }

        Ok(Self {
            iface,
            rate,
            mixer: Mixer::new(rate, shift, gain, !low_if),
        })
    }

    pub fn recv(
        mut self,
        stop: &AtomicBool,
        mut callback: impl FnMut(&[Complex], u64) -> ControlFlow<()>,
    ) -> io::Result<()> {
        if stop.load(Ordering::Acquire) {
            return Ok(());
        }
        let mut ep = self.iface.endpoint::<Bulk, In>(0x81)?;
        ep.clear_halt().wait()?;
        let (free_tx, free_rx) = mpsc::sync_channel(BUFFERS);
        let (tx, rx) = mpsc::sync_channel(BUFFERS);
        for _ in 0..BUFFERS {
            free_tx.send(ep.allocate(BYTES)).unwrap();
        }
        for _ in 0..TRANSFERS {
            let buf = ep.allocate(BYTES);
            ep.submit(buf);
        }
        write(&self.iface, RX_MODE, 1, 0, &[])?;

        let result = thread::scope(|scope| {
            // don't alloc here
            let usb = thread::Builder::new()
                .name("airspy-usb".into())
                .spawn_scoped(scope, move || -> io::Result<()> {
                    let mut dropped = 0;
                    let mut last = Instant::now();
                    while !stop.load(Ordering::Acquire) {
                        let Some(done) = ep.wait_next_complete(Duration::from_millis(100)) else {
                            if last.elapsed() >= Duration::from_secs(2) {
                                return Err(io::Error::new(
                                    io::ErrorKind::TimedOut,
                                    "Airspy USB stream stalled",
                                ));
                            }
                            continue;
                        };
                        let buf = done.into_result()?;
                        if buf.len() != BYTES {
                            return Err(io::Error::new(
                                io::ErrorKind::InvalidData,
                                "short Airspy I/Q transfer",
                            ));
                        }
                        last = Instant::now();
                        if let Ok(spare) = free_rx.try_recv() {
                            // extra?
                            if tx.send((buf, dropped)).is_err() {
                                break;
                            }
                            dropped = 0;
                            ep.submit(spare);
                        } else {
                            dropped += BLOCK as u64;
                            ep.submit(buf);
                        }
                    }
                    Ok(()) // Cancels outgoing
                })?;

            let consumed = catch_unwind(AssertUnwindSafe(|| {
                let mut samples = [Complex::default(); BLOCK];
                while !stop.load(Ordering::Acquire) {
                    let Ok((buf, dropped)) = rx.recv() else { break };
                    self.mixer.convert(&buf, &mut samples, dropped);
                    let _ = free_tx.try_send(buf);
                    if callback(&samples, dropped).is_break() {
                        break;
                    }
                }
            }));
            // Make sure we release the thread
            stop.store(true, Ordering::Release);
            let result = usb
                .join()
                .map_err(|_| io::Error::other("Airspy USB thread panicked"));
            if let Err(panic) = consumed {
                resume_unwind(panic);
            }
            result?
        });
        let off = write(&self.iface, RX_MODE, 0, 0, &[]);
        result.and(off.map_err(Into::into))
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        // Also stop if starting capture fails or
        let _ = write(&self.iface, RX_MODE, 0, 0, &[]);
    }
}

fn serial(text: &str) -> Option<u64> {
    let hex = text.strip_prefix("AIRSPYHF SN:").unwrap_or(text);
    (hex.len() == 16)
        .then(|| u64::from_str_radix(hex, 16).ok())
        .flatten()
}

fn write(
    iface: &Interface,
    request: u8,
    value: u16,
    index: u16,
    data: &[u8],
) -> Result<(), TransferError> {
    iface
        .control_out(
            ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request,
                value,
                index,
                data,
            },
            TIMEOUT,
        )
        .wait()
}

fn read(iface: &Interface, request: u8, index: u16, length: u16) -> io::Result<Option<Vec<u8>>> {
    match iface
        .control_in(
            ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request,
                value: 0,
                index,
                length,
            },
            TIMEOUT,
        )
        .wait()
    {
        Ok(data) if data.len() == usize::from(length) => Ok(Some(data)),
        Ok(_) => Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("short Airspy control reply for request {request}"),
        )),
        Err(TransferError::Stall) => Ok(None), // Older  firmware
        Err(error) => Err(error.into()),
    }
}

fn words(iface: &Interface, request: u8) -> io::Result<Option<Vec<u32>>> {
    let Some(count) = read(iface, request, 0, 4)? else {
        return Ok(None);
    };
    let count = u32::from_le_bytes(count.try_into().unwrap());
    if count == 0 || count > u32::from(u16::MAX) / 4 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid Airspy table size",
        ));
    }
    Ok(
        read(iface, request, count as u16, (count * 4) as u16)?.map(|data| {
            data.as_chunks::<4>()
                .0
                .iter()
                .map(|&v| u32::from_le_bytes(v))
                .collect()
        }),
    )
}

fn rate(rates: &[u32], requested: Option<u32>) -> io::Result<u32> {
    if let Some(rate) = requested {
        if !rates.contains(&rate) {
            return Err(invalid(format!(
                "AJD_DEVICE_RATE {rate} is unavailable; device reports {rates:?}"
            )));
        }
        if rate < RATE || !rate.is_multiple_of(RATE) {
            return Err(invalid(format!(
                "AJD_DEVICE_RATE {rate} must be an integer multiple of {RATE} Hz"
            )));
        }
        return Ok(rate);
    }
    rates
        .iter()
        .copied()
        .filter(|rate| *rate >= RATE && rate.is_multiple_of(RATE))
        .min()
        .ok_or_else(|| {
            invalid(format!(
                "no device rate is an integer multiple of {RATE} Hz; device reports {rates:?}"
            ))
        })
}

fn attidx(steps: &[f32], att: f32) -> io::Result<u16> {
    if steps.iter().any(|v| !v.is_finite() || *v < 0.0) || steps.windows(2).any(|v| v[0] > v[1]) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "invalid Airspy attenuation table",
        ));
    }
    steps
        .iter()
        .position(|&v| v >= att)
        .map(|i| i as u16)
        .ok_or_else(|| {
            invalid(format!(
                "AJD_ATTENUATION {att} exceeds device steps {steps:?}"
            ))
        })
}

fn tune(hz: f64, ppb: i32, low_if: bool) -> io::Result<(u32, f64)> {
    let adjusted = hz * (1.0 + f64::from(ppb) * 1e-9);
    let lo = ((adjusted + if low_if { 0.0 } else { 5000.0 }) * 1e-3)
        .round()
        .max(if low_if { 84.0 } else { 180.0 });
    if !adjusted.is_finite() || adjusted <= 0.0 || lo > f64::from(u32::MAX) {
        return Err(invalid("Airspy frequency or calibration is out of range"));
    }
    Ok((lo as u32, adjusted - lo * 1000.0))
}

fn delta(data: [u8; 4]) -> io::Result<f64> {
    let scale = 1_u32.checked_shl(u32::from(data[0])).ok_or_else(|| {
        io::Error::new(io::ErrorKind::InvalidData, "invalid Airspy frequency delta")
    })?;
    let value = i32::from_le_bytes([0, data[1], data[2], data[3]]) >> 8;
    Ok(f64::from(value) * 1000.0 / f64::from(scale))
}

struct Mixer {
    step: f64,
    phase: f64,
    gain: f32,
    zero_if: bool,
    dc: Complex,
}

impl Mixer {
    fn new(rate: u32, shift: f64, gain: f32, zero_if: bool) -> Self {
        Self {
            step: -TAU * shift / f64::from(rate),
            phase: 0.0,
            gain: gain / 32768.0,
            zero_if,
            dc: Complex::default(),
        }
    }

    fn convert(&mut self, raw: &[u8], out: &mut [Complex], dropped: u64) {
        debug_assert_eq!(raw.len(), out.len() * 4);
        self.phase = (self.phase + self.step * dropped as f64).rem_euclid(TAU);
        if dropped != 0 {
            self.dc = Complex::default();
        }
        let (sin, cos) = self.step.sin_cos();
        let (mut im, mut re) = (self.phase + self.step).sin_cos();
        for (bytes, dst) in raw.as_chunks::<4>().0.iter().zip(out.iter_mut()) {
            // NOTE: little-endian Q, I, not I, Q!
            let mut i = f32::from(i16::from_le_bytes([bytes[2], bytes[3]])) * self.gain;
            let mut q = f32::from(i16::from_le_bytes([bytes[0], bytes[1]])) * self.gain;
            if self.zero_if {
                self.dc.re += 1e-4 * (i - self.dc.re);
                self.dc.im += 1e-4 * (q - self.dc.im);
                i -= self.dc.re;
                q -= self.dc.im;
                (i, q) = ((i + 0.00006 * q) * 0.9955, (q + 0.00006 * i) * 1.0045);
            }
            *dst = Complex {
                re: i * re as f32 - q * im as f32,
                im: i * im as f32 + q * re as f32,
            };
            (re, im) = (re * cos - im * sin, re * sin + im * cos);
        }
        self.phase = (self.phase + self.step * out.len() as f64).rem_euclid(TAU);
    }
}

#[cfg(test)]
mod tests {
    use crate::{config::Mode, dsp::Demod};

    use super::*;

    #[test]
    fn dev_cfg() {
        assert_eq!(
            serial("AIRSPYHF SN:0123456789ABCDEF"),
            Some(0x0123_4567_89ab_cdef)
        );
        assert_eq!(serial("0123456789abcdef"), Some(0x0123_4567_89ab_cdef));
        assert_eq!(serial("AIRSPYHF SN:bad"), None);
        assert_eq!(rate(&[768_000, 384_000], None).unwrap(), 384_000);
        assert_eq!(rate(&[768_000, 384_000], Some(768_000)).unwrap(), 768_000);
        assert_eq!(rate(&[456_000], None).unwrap(), 456_000);
        assert!(rate(&[44_100], None).is_err());
        assert!(rate(&[768_000], Some(384_000)).is_err());
        assert!(rate(&[0], Some(0)).is_err());
        assert_eq!(attidx(&[0.0, 6.0, 12.0], 7.0).unwrap(), 2);
        assert!(attidx(&[0.0, 48.0], 49.0).is_err());
        assert!(attidx(&[f32::NAN], 1.0).is_err());
    }

    #[test]
    fn tune_delta() {
        assert_eq!(tune(7_078_000.0, 0, false).unwrap(), (7083, -5000.0));
        let (lo, shift) = tune(7_078_123.0, 1000, true).unwrap();
        assert_eq!(lo, 7078);
        assert!((shift - 130.078123).abs() < 1e-6);
        assert_eq!(tune(10_000.0, 0, true).unwrap(), (84, -74000.0));
        assert_eq!(delta([10, 0, 1, 0]).unwrap(), 250.0);
        assert_eq!(delta([10, 0, 255, 255]).unwrap(), -250.0);
        assert!(delta([32, 0, 0, 0]).is_err());
        assert!(tune(f64::NAN, 0, false).is_err());
        assert!(tune(7_078_000.0, -1_000_000_000, false).is_err());
        assert!(tune(1e20, 0, true).is_err());
    }

    #[test]
    fn iq_gap() {
        let mut sample = [Complex::default()];
        Mixer::new(48_000, 0.0, 0.5, false).convert(&[0, 64, 0, 128], &mut sample, 0);
        assert_eq!(sample[0], Complex { re: -0.5, im: 0.25 });

        let mut mixer = Mixer::new(48_000, -12_000.0, 1.0, false);
        mixer.convert(&[0, 0, 0, 64], &mut sample, 0);
        assert!(sample[0].re.abs() < 1e-6 && (sample[0].im - 0.5).abs() < 1e-6);
        mixer.convert(&[0, 0, 0, 64], &mut sample, 2);
        assert!((sample[0].re - 0.5).abs() < 1e-6 && sample[0].im.abs() < 1e-6);

        let raw = [0, 0, 0, 64].repeat(BLOCK * 16);
        let mut out = vec![Complex::default(); raw.len() / 4];
        Mixer::new(768_000, -5000.0, 1.0, true).convert(&raw, &mut out, 0);
        assert!(out.last().unwrap().re.hypot(out.last().unwrap().im) < 0.001);
    }

    #[test]
    fn iq_audio() {
        const RATE_IN: u32 = 384_000;
        for low_if in [false, true] {
            let (_, shift) = tune(7_078_123.0, 1000, low_if).unwrap();
            for (mode, hz) in [(Mode::Usb, 1500.0), (Mode::Lsb, -1500.0)] {
                let mut mixer = Mixer::new(RATE_IN, shift, 1.0, !low_if);
                let mut demod = Demod::new(RATE_IN, RATE, 200.0..4000.0, mode, false, 1.0).unwrap();
                let mut raw = [0; BYTES];
                let mut iq = [Complex::default(); BLOCK];
                let mut audio = Vec::new();
                for block in 0..RATE_IN as usize / BLOCK {
                    for (n, bytes) in raw.as_chunks_mut::<4>().0.iter_mut().enumerate() {
                        let phase =
                            TAU * (hz + shift) * (block * BLOCK + n) as f64 / f64::from(RATE_IN);
                        let (q, i) = phase.sin_cos();
                        bytes[..2].copy_from_slice(&((q * 16384.0).round() as i16).to_le_bytes());
                        bytes[2..].copy_from_slice(&((i * 16384.0).round() as i16).to_le_bytes());
                    }
                    mixer.convert(&raw, &mut iq, 0);
                    demod.process(&iq, Some(&mut audio));
                }
                let audio = &audio[2400..]; // Ignore filter startup.
                let (mut i, mut q) = (0.0, 0.0);
                for (n, &sample) in audio.iter().enumerate() {
                    let phase = TAU * 1500.0 * n as f64 / f64::from(RATE);
                    let sample = f64::from(sample) / f64::from(i16::MAX);
                    i += sample * phase.cos();
                    q += sample * phase.sin();
                }
                let level = 2.0 * i.hypot(q) / audio.len() as f64;
                assert!(
                    (0.49..0.51).contains(&level),
                    "{mode:?}, low_if={low_if}: {level}"
                );
            }
        }
    }
}
