// SPDX-License-Identifier: BSD-3-Clause

use crate::{
    config::{DecodeSettings, RATE},
    message::Assembler,
    output,
};
use js8rs::{
    protocol::{Submode, code_time},
    rx::{DecodeConfig, DecodeScheduler, Decoded, Decoder, Event, SAMPLE_BUFFER_SIZE},
};
use std::{
    io::{self, Write},
    sync::mpsc::{Receiver, SyncSender},
};

const MODES: [Submode; 5] = [
    Submode::Normal,
    Submode::Fast,
    Submode::Turbo,
    Submode::Slow,
    Submode::Ultra,
];

pub struct Packet {
    pub data: Vec<i16>,
    pub start: u64,
    pub discontinuity: bool,
}

#[derive(Default, Debug)]
pub struct Stats {
    pub passes: u64,
    pub frames: u64,
    pub messages: u64,
    pub discontinuities: u64,
}

struct Rx {
    decoder: Decoder,
    scheduler: DecodeScheduler,
    samples: Vec<i16>,
    pos: usize,
}

impl Rx {
    fn new(settings: DecodeSettings) -> Self {
        Self {
            decoder: Decoder::with_modes(settings.modes),
            scheduler: DecodeScheduler::new(),
            samples: vec![0; SAMPLE_BUFFER_SIZE],
            pos: 0,
        }
    }

    fn push(
        &mut self,
        packet: &Packet,
        settings: DecodeSettings,
        frames: &mut Vec<Decoded>,
    ) -> u64 {
        let received_ms = (packet.start + packet.data.len() as u64) * 1_000 / u64::from(RATE);
        let config = DecodeConfig::default()
            .with_modes(settings.modes)
            .with_utc(utc(received_ms))
            .with_nominal_frequency(settings.nominal_frequency)
            .with_frequency_range(settings.min_frequency, settings.max_frequency);
        // UTC uses a minute offset; untimed counts samples since the last gap.
        if !cfg!(feature = "experimental-time") {
            self.pos = (packet.start % SAMPLE_BUFFER_SIZE as u64) as usize;
        }
        let mut input = packet.data.as_slice();
        let mut passes = 0;
        while !input.is_empty() {
            let start = self.pos;
            let off = start % self.samples.len();
            if !cfg!(feature = "experimental-time") && off == 0 {
                self.samples.fill(0);
                self.scheduler = DecodeScheduler::new();
            }
            let len = input.len().min(self.samples.len() - off);
            let end = start + len;
            self.samples[off..off + len].copy_from_slice(&input[..len]);
            for mode in MODES {
                if settings.modes.contains(mode.into())
                    && let Some(window) = self.scheduler.next_window(mode, end, start)
                {
                    let (data, n) = if cfg!(feature = "experimental-time") {
                        // wrapped windows from the whole ring
                        (self.samples.as_slice(), window.start + window.size)
                    } else {
                        let data = &self.samples[window.start..window.start + window.size];
                        (data, data.len())
                    };
                    self.decoder
                        .decode(data, n, &config.with_modes(mode.into()), |event| {
                            if let Event::Decoded(frame) = event {
                                if cfg!(feature = "experimental-time") {
                                    frames.push(frame); // overlap is suppressed by js8ers
                                } else {
                                    best(frames, frame);
                                }
                            }
                        });
                    passes += 1;
                }
            }
            input = &input[len..];
            self.pos = if cfg!(feature = "experimental-time") {
                end
            } else {
                end % self.samples.len()
            };
        }
        passes
    }
}

pub fn run(
    settings: DecodeSettings,
    mut output: impl Write,
    packets: Receiver<Packet>,
    free: SyncSender<Vec<i16>>,
) -> io::Result<Stats> {
    let mut rx = Rx::new(settings);
    let mut assembler = Assembler::new();
    let mut frames = Vec::new();
    let mut encoded = Vec::new();
    let mut stats = Stats::default();
    let mut end = None;

    while let Ok(packet) = packets.recv() {
        if packet.discontinuity || end.is_some_and(|end| end != packet.start) {
            rx = Rx::new(settings);
            assembler.clear();
            stats.discontinuities += 1;
        }
        end = Some(packet.start + packet.data.len() as u64);
        let received_ms = end.unwrap() * 1_000 / u64::from(RATE);
        frames.clear();
        stats.passes += rx.push(&packet, settings, &mut frames);
        // Return storage before notification I/O so a slow reader can use the
        // full bounded input queue. A closed pool means capture has stopped.
        let _ = free.try_send(packet.data);
        stats.frames += frames.len() as u64;
        for frame in &frames {
            if let Some(message) = assembler.push(frame, received_ms) {
                output::send(&mut output, &message, &mut encoded)?;
                stats.messages += 1;
            }
        }
    }
    Ok(stats)
}

// Same frame can get reported more than once so only keep the best
fn best(frames: &mut Vec<Decoded>, decoded: Decoded) {
    let duplicate = frames.iter_mut().find(|frame| {
        frame.frame.submode == decoded.frame.submode
            && frame.frame.flags == decoded.frame.flags
            && frame.frame.encoded == decoded.frame.encoded
            && frame.frequency_hz.max(0.0) as u32 == decoded.frequency_hz.max(0.0) as u32
    });
    if let Some(previous) = duplicate {
        if (decoded.quality, decoded.snr) > (previous.quality, previous.snr) {
            *previous = decoded;
        }
    } else {
        frames.push(decoded);
    }
}

const fn utc(unix_ms: u64) -> u32 {
    let seconds = (unix_ms / 1_000) % 86_400;
    code_time(
        (seconds / 3_600) as u8,
        ((seconds % 3_600) / 60) as u8,
        (seconds % 60) as u8,
    )
}

#[cfg(test)]
mod tests {
    use std::{sync::mpsc, time::Duration};

    use ajd::protocol::Notification;
    use js8rs::{
        codec::{BuildFramesOptions, EncodedFrame, Frame, build_frames},
        protocol::FrameFlags,
        tx::{Channel, Modulator},
    };

    use crate::{airspy::Complex, config::Mode, dsp::Demod};

    use super::*;

    fn waveform(frame: &EncodedFrame, lead: usize, hz: f64) -> Vec<i16> {
        let mut modulator = Modulator::new();
        let mut demod = Demod::new(48_000, RATE, 200.0..4_000.0, Mode::Usb, false, 0.25).unwrap();
        let mut pcm = [0; 4_096];
        let mut iq = [Complex::default(); 2_048];
        let mut samples = vec![0; lead];
        modulator.start(frame, 0, hz, Duration::ZERO, Channel::Mono);
        while !modulator.is_idle() {
            let len = modulator.render_stereo(&mut pcm);
            for (index, sample) in iq[..len].iter_mut().enumerate() {
                sample.re = f32::from(pcm[index * 2]) / f32::from(i16::MAX);
            }
            demod.process(&iq[..len], Some(&mut samples));
        }
        samples.resize(
            if cfg!(feature = "experimental-time") {
                samples.len() + frame.submode.samples_per_period()
            } else {
                frame.submode.samples_per_period()
            },
            0,
        );
        samples
    }

    #[test]
    fn iq_json() {
        let leads: &[usize] = if cfg!(feature = "experimental-time") {
            // Off-slot, off-stride, and across the actual sample ring boundary.
            &[16_044, SAMPLE_BUFFER_SIZE - 20_000]
        } else {
            &[0]
        };
        for mode in MODES {
            for &lead in leads {
                let encoded = build_frames(&BuildFramesOptions::new("HELLO", mode))
                    .encode()
                    .unwrap();
                assert_eq!(encoded.len(), 1);
                let samples = waveform(&encoded[0], lead, 1_500.0);
                let (tx, rx) = mpsc::channel();
                let (free, _pool) = mpsc::sync_channel(1);
                // Last slot of a minute: trailing silence also tests minute wrap.
                let start = (SAMPLE_BUFFER_SIZE - mode.samples_per_period()) as u64;
                for (index, data) in samples.chunks(997).enumerate() {
                    tx.send(Packet {
                        data: data.to_vec(),
                        start: start + (index * 997) as u64,
                        discontinuity: false,
                    })
                    .unwrap();
                }
                if !cfg!(feature = "experimental-time") {
                    tx.send(Packet {
                        data: vec![0; 997],
                        start: start + samples.len() as u64,
                        discontinuity: false,
                    })
                    .unwrap();
                }
                drop(tx);
                let settings = DecodeSettings {
                    modes: mode.into(),
                    min_frequency: 200,
                    max_frequency: 4_000,
                    nominal_frequency: 1_500,
                };
                let mut json = Vec::new();
                let stats = run(settings, &mut json, rx, free).unwrap();
                assert_eq!(stats.messages, 1, "{mode}, lead={lead}: {stats:?}");
                assert_eq!(json.iter().filter(|&&byte| byte == b'\n').count(), 1);
                let message: Notification = serde_json::from_slice(&json).unwrap();
                assert_eq!(message.text, "HELLO");
                assert_eq!(message.submode, mode as u8);
                assert!((message.frequency_hz - 1_500.0).abs() < 2.0);
                assert_eq!(
                    message.sample_position.is_some(),
                    cfg!(feature = "experimental-time")
                );
                if let Some(position) = message.sample_position {
                    assert!(position.abs_diff(lead as u64) < mode.samples_for_one_symbol());
                }
                assert!(message.received_unix_ms >= start * 1_000 / u64::from(RATE));
                assert!(
                    message.received_unix_ms
                        <= (start + samples.len() as u64) * 1_000 / u64::from(RATE)
                );
                assert_eq!(message.utc, utc(message.received_unix_ms));
                assert_eq!(stats.discontinuities, 0);
            }
        }
    }

    #[test]
    fn wire_json() {
        // actual js8call data!
        for mode in MODES {
            let mut raw = build_frames(&BuildFramesOptions::new("HELLO", mode));
            raw.frames = [
                ("9-S5oVx5ukfO", 1),
                ("IUnN3JQHO+KO", 0),
                if mode == Submode::Normal {
                    ("tYuMzod0HA7+", 2)
                } else {
                    ("UBXRtAS14eV+", 6)
                },
                ("Vk64SVGGOq00", 3),
                ("2URtg4DOPkfO", 3),
                ("2URtg4DOOkfO", 3),
            ]
            .into_iter()
            .map(|(txt, bits)| Frame::new(txt, FrameFlags::from_bits_truncate(bits)))
            .collect();
            let mut samples = Vec::new();
            for (i, fr) in raw.encode().unwrap().iter().enumerate() {
                samples.extend(waveform(fr, 0, 1_500.0 + i as f64 * 0.4));
            }
            let (tx, rx) = mpsc::channel();
            let (free, _pool) = mpsc::sync_channel(1);
            for (i, data) in samples.chunks(997).enumerate() {
                tx.send(Packet {
                    data: data.to_vec(),
                    start: (i * 997) as u64,
                    discontinuity: false,
                })
                .unwrap();
            }
            drop(tx);
            let mut json = Vec::new();
            let stats = run(
                DecodeSettings {
                    modes: mode.into(),
                    min_frequency: 1_400,
                    max_frequency: 1_600,
                    nominal_frequency: 1_500,
                },
                &mut json,
                rx,
                free,
            )
            .unwrap();
            let msgs = serde_json::Deserializer::from_slice(&json)
                .into_iter::<Notification>()
                .collect::<Result<Vec<_>, _>>()
                .unwrap();
            assert_eq!(msgs.len(), 4, "{mode}: {stats:?} {msgs:?}");
            assert_eq!(msgs[0].from.as_deref(), Some("F/K1ABC"), "{mode}");
            assert_eq!(msgs[0].to.as_deref(), Some("K2XYZ"));
            assert_eq!(msgs[0].command.as_deref(), Some("MSG"));
            assert_eq!(msgs[0].payload.as_deref(), Some("HELLO WORLD"));
            assert_eq!(msgs[0].checksum.as_ref().unwrap().value, "5GA");
            assert_eq!(msgs[0].frame_count, 3);
            for (msg, to, cmd) in [
                (&msgs[1], "@ALLCALL", "SNR?"),
                (&msgs[2], "@ALLCALL", "CQ"),
                (&msgs[3], "@HB", "HEARTBEAT"),
            ] {
                assert_eq!(msg.from.as_deref(), Some("K1ABC"));
                assert_eq!(msg.to.as_deref(), Some(to));
                assert_eq!(msg.command.as_deref(), Some(cmd));
                assert!(msg.checksum.is_none());
            }
        }
    }

    #[test]
    fn gap_reset() {
        let mode = Submode::Ultra;
        let encoded = build_frames(&BuildFramesOptions::new("HELLO", mode))
            .encode()
            .unwrap();
        let lead = if cfg!(feature = "experimental-time") {
            16_044
        } else {
            0
        };
        let samples = waveform(&encoded[0], lead, 1_500.0);
        let split = mode.samples_for_symbols() as usize / 2 + lead;
        for timestamp_gap in [false, true] {
            let (tx, rx) = mpsc::channel();
            let (free, _pool) = mpsc::sync_channel(1);
            let mut start = 0;
            for (part, data) in [&samples[..split], &samples[split..], &samples]
                .into_iter()
                .enumerate()
            {
                if part == 1 && timestamp_gap {
                    start += mode.samples_per_period() as u64;
                }
                for (index, chunk) in data.chunks(997).enumerate() {
                    tx.send(Packet {
                        data: chunk.to_vec(),
                        start,
                        discontinuity: part == 1 && index == 0 && !timestamp_gap,
                    })
                    .unwrap();
                    start += chunk.len() as u64;
                }
            }
            drop(tx);
            let mut json = Vec::new();
            let stats = run(
                DecodeSettings {
                    modes: mode.into(),
                    min_frequency: 200,
                    max_frequency: 4_000,
                    nominal_frequency: 1_500,
                },
                &mut json,
                rx,
                free,
            )
            .unwrap();
            assert_eq!(stats.discontinuities, 1);
            assert_eq!(
                stats.messages, 1,
                "timestamp_gap={timestamp_gap}: {stats:?}"
            );
            let message: Notification = serde_json::from_slice(&json).unwrap();
            assert_eq!(message.text, "HELLO");
            if let Some(position) = message.sample_position {
                let expected = (samples.len() - split + lead) as u64;
                assert!(position.abs_diff(expected) < mode.samples_for_one_symbol());
            }
        }
    }
}
