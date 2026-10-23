// SPDX-License-Identifier: BSD-3-Clause

use std::collections::{HashMap, VecDeque};

use ajd::protocol::{Checksum, Notification};
use js8rs::{
    protocol::{FrameFlags, Submode},
    rx::{BufferedCommandResult, Decoded, MessageBufferAssembler, ReassemblyEvent},
};

pub struct Assembler {
    frames: HashMap<Key, FrameState>,
}

#[derive(Clone, Copy, Eq, Hash, PartialEq)]
struct Key(Submode, u32);

struct FrameState {
    asm: MessageBufferAssembler,
    cmd: Option<BufferedCommandResult>,
    calls: VecDeque<String>,
    hdr: Option<Decoded>,
    buf: bool,
    late: bool,
    utc: u32,
    submode: u8,
    frame_type: u8,
    flags: FrameFlags,
    frame_count: usize,
    text: String,
    frequency_hz: f32,
    snr_db: i32,
    time_offset_seconds: f32,
    quality: f32,
    sample_position: Option<u64>,
}

impl Assembler {
    pub fn new() -> Self {
        Self {
            frames: HashMap::new(),
        }
    }

    pub fn clear(&mut self) {
        self.frames.clear();
    }

    pub fn push(&mut self, decoded: &Decoded, received_unix_ms: u64) -> Option<Notification> {
        let key = key(decoded);
        let prev = self.near(key);
        let mut state = if decoded.frame.flags.contains(FrameFlags::FIRST) {
            if let Some(prev) = prev {
                self.frames.remove(&prev);
            }
            FrameState::new(decoded)
        } else {
            self.frames.remove(&prev?)?
        };
        if !state.push(decoded) {
            return None;
        }

        if state.flags.contains(FrameFlags::LAST)
            && state.hdr.is_none()
            && !state.buf
            && state
                .cmd
                .as_ref()
                .is_none_or(|cmd| cmd.from != "<....>" && cmd.to != "<....>")
        {
            let cmd = state.cmd.take();
            Some(notif(state, cmd, received_unix_ms))
        } else {
            self.frames.insert(key, state);
            None
        }
    }

    fn near(&self, key: Key) -> Option<Key> {
        if self.frames.contains_key(&key) {
            return Some(key);
        }
        let range = key.0.rx_threshold() as u32;
        let mut near = None;
        let mut dist = range + 1;
        for &prev in self.frames.keys().filter(|prev| prev.0 == key.0) {
            let delta = prev.1.abs_diff(key.1);
            if delta < dist {
                near = Some(prev);
                dist = delta;
            } else if delta == dist {
                near = None;
            }
        }
        near
    }
}

impl FrameState {
    fn new(decoded: &Decoded) -> Self {
        Self {
            asm: MessageBufferAssembler::new(),
            cmd: None,
            calls: VecDeque::new(),
            hdr: None,
            buf: false,
            late: false,
            utc: decoded.utc,
            submode: decoded.frame.submode as u8,
            frame_type: decoded.frame.frame_type as u8,
            flags: decoded.frame.flags,
            frame_count: 0,
            text: String::new(),
            frequency_hz: decoded.frequency_hz,
            snr_db: decoded.snr,
            time_offset_seconds: decoded.time_offset_seconds,
            quality: decoded.quality,
            sample_position: decoded.sample_position,
        }
    }

    fn push(&mut self, decoded: &Decoded) -> bool {
        self.flags |= decoded.frame.flags;
        self.frame_count = self.frame_count.saturating_add(1);
        self.text.push_str(&decoded.frame.message);
        self.quality = self.quality.min(decoded.quality);

        let mut frame = decoded.clone();
        frame.frequency_hz = 0.0;
        if frame.frame.is_heartbeat {
            let (to, cmd) = if frame.frame.is_alt {
                ("@ALLCALL", " CQ")
            } else {
                ("@HB", " HEARTBEAT")
            };
            frame.frame.directed = vec![frame.frame.compound.clone(), to.into(), cmd.into()];
        } else if frame.frame.is_compound() && !frame.frame.is_directed() {
            self.late |= self.cmd.is_some() || self.buf || self.hdr.is_some();
            self.calls.push_back(frame.frame.compound.clone());
            if let Some(hdr) = self.hdr.take() {
                frame = hdr;
            }
        }
        for call in frame.frame.directed.iter_mut().take(2) {
            fillcall(call, &mut self.calls);
        }
        if frame.frame.flags.contains(FrameFlags::LAST)
            && frame
                .frame
                .directed
                .iter()
                .take(2)
                .any(|call| call == "<....>")
        {
            self.hdr = Some(frame);
            return true;
        }

        match self.asm.push_decoded(&frame) {
            Some(ReassemblyEvent::Completed(cmd)) => {
                self.cmd = Some(cmd);
                self.buf = false;
            }
            Some(ReassemblyEvent::BufferedStarted(_)) => {
                self.cmd = None;
                self.buf = true;
            }
            Some(ReassemblyEvent::DroppedInvalidChecksum(_)) => return false,
            _ => {}
        }
        if let Some(cmd) = &mut self.cmd {
            fillcall(&mut cmd.from, &mut self.calls);
            fillcall(&mut cmd.to, &mut self.calls);
        }
        true
    }
}

fn fillcall(call: &mut String, calls: &mut VecDeque<String>) {
    if call == "<....>"
        && let Some(full) = calls.pop_front()
    {
        *call = full;
    }
}

fn notif(
    mut message: FrameState,
    command: Option<BufferedCommandResult>,
    received_unix_ms: u64,
) -> Notification {
    let text_len = message.text.trim_end_matches(' ').len();
    message.text.truncate(text_len);
    let (from, to, command_name, payload, checksum) = match command {
        Some(command) => {
            if message.late {
                message.text = format!(
                    "{}: {}{} {}",
                    command.from, command.to, command.command, command.payload,
                )
                .trim_end_matches(' ')
                .to_owned();
            }
            let checksum = command.checksum.map(|checksum| {
                if !message.late {
                    stripck(&mut message.text, &checksum.value);
                }
                Checksum {
                    size_bits: checksum.size_bits,
                    value: checksum.value,
                }
            });
            (
                Some(command.from),
                Some(command.to),
                Some(command.command.trim().to_owned()),
                Some(command.payload),
                checksum,
            )
        }
        None => (None, None, None, None, None),
    };
    let frame_count = message.frame_count.min(u16::MAX as usize) as u16;

    Notification {
        received_unix_ms,
        utc: message.utc,
        submode: message.submode,
        frame_type: message.frame_type,
        flags: message.flags.bits(),
        frame_count,
        text: message.text,
        from,
        to,
        command: command_name,
        payload,
        checksum,
        frequency_hz: message.frequency_hz,
        snr_db: message.snr_db,
        time_offset_seconds: message.time_offset_seconds,
        quality: message.quality,
        sample_position: message.sample_position,
    }
}

fn stripck(text: &mut String, checksum: &str) {
    if checksum.is_empty() || !text.ends_with(checksum) {
        return;
    }
    let start = text.len() - checksum.len();
    if start != 0 && text.as_bytes()[start - 1] == b' ' {
        text.truncate(start - 1);
    }
}

fn key(decoded: &Decoded) -> Key {
    Key(decoded.frame.submode, decoded.frequency_hz.max(0.0) as u32)
}

#[cfg(test)]
mod tests {
    use js8rs::{
        codec::{BuildFramesOptions, build_frames, parse_frame},
        protocol::Submode,
    };

    use super::*;

    #[test]
    fn free_text() {
        let text = "THIS IS A LONG FREE TEXT MESSAGE FOR REASSEMBLY";
        let built = build_frames(
            &BuildFramesOptions::new(text, Submode::Fast)
                .with_station("K1ABC", "EM73")
                .with_data(true),
        );
        assert!(built.frames.len() > 1);
        let last = built.frames.len() - 1;
        let mut assembler = Assembler::new();

        for (index, frame) in built.frames.iter().enumerate() {
            let decoded = decoded(frame, Submode::Fast);
            let message = assembler.push(&decoded, 123);
            if index == last {
                let message = message.expect("last frame should complete the message");
                assert_eq!(message.text.trim(), text);
                assert_eq!(message.frame_count as usize, built.frames.len());
            } else {
                assert!(message.is_none());
            }
        }
    }

    #[test]
    fn cmd_cksum() {
        let payload = "HELLO THIS IS A LONG BUFFERED MESSAGE";
        let built = build_frames(
            &BuildFramesOptions::new(format!("MSG {payload}"), Submode::Normal)
                .with_station("K1ABC", "EM73")
                .with_selected_call("K2XYZ"),
        );
        let mut assembler = Assembler::new();
        let mut message = None;
        for frame in &built.frames {
            message = assembler.push(&decoded(frame, Submode::Normal), 456);
        }

        let message = message.expect("buffered command should complete");
        assert_eq!(message.from.as_deref(), Some("K1ABC"));
        assert_eq!(message.to.as_deref(), Some("K2XYZ"));
        assert_eq!(message.command.as_deref(), Some("MSG"));
        assert_eq!(message.payload.as_deref(), Some(payload));
        let checksum = &message.checksum.as_ref().unwrap().value;
        assert!(!message.text.ends_with(checksum));
    }

    #[test]
    fn compound() {
        let mut assembler = Assembler::new();
        let mut message = None;
        // Packed by JS8Call-improved; /P alone fits in a standard header.
        for (text, bits) in [
            ("AURtg4xlckfO", 1),
            ("IUnN3K8ec+KO", 0),
            ("UBXUuCjvshV+", 4),
            ("Uv7YCosF++++", 6),
        ] {
            message = assembler.push(&wire(text, bits, Submode::Fast, 1_500.0), 789);
        }

        let message = message.expect("compound directed message should complete");
        assert_eq!(message.from.as_deref(), Some("K1ABC/QRP"));
        assert_eq!(message.to.as_deref(), Some("K2XYZ/QRP"));
        assert_eq!(message.command.as_deref(), Some("MSG"));
        assert_eq!(
            message.text,
            "K1ABC/QRP: K2XYZ/QRP MSG HELLO FROM A COMPOUND CALL"
        );
        assert_eq!(
            message.payload.as_deref(),
            Some("HELLO FROM A COMPOUND CALL")
        );
        assert!(message.checksum.is_some());
    }

    #[test]
    fn wire_meta() {
        for (text, to, cmd, payload) in [
            ("Vk64SUugOzcL", "K2XYZ", "SNR", "-10"),
            ("2URtg4DOPkfO", "@ALLCALL", "CQ", ""),
            ("2URtg4DOOkfO", "@HB", "HEARTBEAT", ""),
        ] {
            let mut asm = Assembler::new();
            let msg = asm
                .push(&wire(text, 3, Submode::Normal, 1_500.0), 0)
                .unwrap();
            assert_eq!(
                msg.from.as_deref(),
                Some(if cmd == "SNR" { "K1ABC/P" } else { "K1ABC" })
            );
            assert_eq!(msg.to.as_deref(), Some(to));
            assert_eq!(msg.command.as_deref(), Some(cmd));
            assert_eq!(msg.payload.as_deref(), Some(payload));
            assert!(msg.checksum.is_none());
        }
    }

    #[test]
    fn late_call() {
        let mode = Submode::Fast;
        let mut asm = Assembler::new();
        for (text, bits) in [
            ("IUnN3K8ec+KO", 1),
            ("UBXUuCjvshV+", 4),
            ("Uv7YCosF++++", 6),
        ] {
            assert!(asm.push(&wire(text, bits, mode, 1_500.0), 0).is_none());
        }
        let msg = asm
            .push(&wire("AURtg4xlckfO", 0, mode, 1_500.0), 0)
            .unwrap();
        assert_eq!(msg.from.as_deref(), Some("K1ABC/QRP"));
        assert_eq!(msg.to.as_deref(), Some("K2XYZ/QRP"));
        assert_eq!(
            msg.text,
            "K1ABC/QRP: K2XYZ/QRP MSG HELLO FROM A COMPOUND CALL"
        );
        assert_eq!(msg.checksum.unwrap().value, "6RW");

        assert!(
            asm.push(&wire("IUnN3JQHO+bu", 3, mode, 1_500.0), 0)
                .is_none()
        );
        let msg = asm
            .push(&wire("AURtg4xlckfO", 0, mode, 1_500.0), 0)
            .unwrap();
        assert_eq!(msg.from.as_deref(), Some("K1ABC/QRP"));
        assert_eq!(msg.command.as_deref(), Some("SNR"));
        assert_eq!(msg.payload.as_deref(), Some("-10"));
        assert_eq!(msg.text, "K1ABC/QRP: K2XYZ SNR -10");
    }

    #[test]
    fn drift_mix_reset() {
        let mut asm = Assembler::new();
        let mode = Submode::Normal;
        for (freq, mode) in [(1_500.0, mode), (2_500.0, mode), (1_500.0, Submode::Slow)] {
            assert!(asm.push(&wire("Vk64SUugOya0", 1, mode, freq), 0).is_none());
        }
        let fresh = build_frames(&BuildFramesOptions::new("HELLO", mode).with_data(true));
        let mut reset = decoded(&fresh.frames[0], mode);
        reset.frequency_hz = 1_499.9;
        let msg = asm.push(&reset, 0).unwrap();
        assert_eq!(msg.text, "HELLO");
        assert!(msg.command.is_none());

        for (i, text) in ["tYuNzBGqh+ft", "td8ylGsJE9Z+"].into_iter().enumerate() {
            for (freq, mode) in [(2_500.0, mode), (1_500.0, Submode::Slow)] {
                let msg = asm.push(
                    &wire(
                        text,
                        if i == 1 { 2 } else { 0 },
                        mode,
                        freq + 9.0 * (i + 1) as f32,
                    ),
                    0,
                );
                if i == 0 {
                    assert!(msg.is_none());
                } else {
                    let msg = msg.unwrap();
                    assert_eq!(msg.frequency_hz, freq);
                    assert_eq!(
                        msg.payload.as_deref(),
                        Some("HELLO THIS IS A LONG BUFFERED MESSAGE")
                    );
                    assert_eq!(msg.checksum.unwrap().value, "-RS");
                }
            }
        }
        assert!(asm.frames.is_empty());
    }

    #[test]
    fn freq_tie() {
        let mut asm = Assembler::new();
        let mode = Submode::Normal;
        for freq in [1_490.0, 1_510.0] {
            assert!(asm.push(&wire("Vk64SUugOya0", 1, mode, freq), 0).is_none());
        }
        assert!(
            asm.push(&wire("tYuNzBGqh+ft", 0, mode, 1_500.0), 0)
                .is_none()
        );
        assert!(asm.frames.values().all(|state| state.frame_count == 1));
        assert!(
            asm.push(&wire("tYuNzBGqh+ft", 0, mode, 1_501.0), 0)
                .is_none()
        );
        let msg = asm
            .push(&wire("td8ylGsJE9Z+", 2, mode, 1_502.0), 0)
            .unwrap();
        assert_eq!(msg.frequency_hz, 1_510.0);
        assert_eq!(msg.frame_count, 3);
    }

    #[test]
    fn bad_cksum_reset() {
        let mode = Submode::Ultra;
        let built = build_frames(
            &BuildFramesOptions::new("MSG HELLO THIS IS A LONG BUFFERED MESSAGE", mode)
                .with_station("K1ABC", "EM73")
                .with_selected_call("K2XYZ"),
        );
        assert!(built.frames.len() > 2);
        let mut assembler = Assembler::new();
        for (index, frame) in built.frames.iter().enumerate() {
            let mut frame = decoded(frame, mode);
            if index == built.frames.len() - 1 {
                frame.frame.message.push('X');
            }
            assert!(assembler.push(&frame, 0).is_none());
        }
        for (index, frame) in built.frames.iter().enumerate() {
            if index == 1 {
                assembler.clear();
            }
            assert!(assembler.push(&decoded(frame, mode), 0).is_none());
        }
        let mut message = None;
        for frame in &built.frames {
            message = assembler.push(&decoded(frame, mode), 1);
        }
        assert!(message.is_some());
    }

    fn decoded(frame: &js8rs::codec::Frame, submode: Submode) -> Decoded {
        Decoded::new(
            parse_frame(&frame.encoded, frame.flags, submode),
            123_456,
            -10,
            0.1,
            1_500.0,
            0.9,
        )
    }

    fn wire(text: &str, bits: u8, mode: Submode, freq: f32) -> Decoded {
        Decoded::new(
            parse_frame(text, FrameFlags::from_bits_truncate(bits), mode),
            123_456,
            -10,
            0.1,
            freq,
            0.9,
        )
    }
}
