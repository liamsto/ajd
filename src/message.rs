// SPDX-License-Identifier: BSD-3-Clause

use std::collections::HashMap;

use ajd::protocol::{Checksum, Notification};
use js8rs::{
    protocol::{FrameFlags, Submode},
    rx::{BufferedCommandResult, Decoded, MessageBufferAssembler, ReassemblyEvent},
};

pub struct Assembler {
    frames: HashMap<Key, FrameState>,
    commands: MessageBufferAssembler,
    pending: HashMap<Key, BufferedCommandResult>,
}

#[derive(Clone, Copy, Eq, Hash, PartialEq)]
struct Key(Submode, u32);

struct FrameState {
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
            commands: MessageBufferAssembler::new(),
            pending: HashMap::new(),
        }
    }

    pub fn clear(&mut self) {
        self.frames.clear();
        self.commands.clear();
        self.pending.clear();
    }

    pub fn push(&mut self, decoded: &Decoded, received_unix_ms: u64) -> Option<Notification> {
        let key = key(decoded);
        if decoded.frame.flags.contains(FrameFlags::FIRST) {
            self.pending.remove(&key);
        }
        let complete = self.frame(key, decoded);

        match self.commands.push_decoded(decoded) {
            Some(ReassemblyEvent::Completed(command)) => {
                if let Some(message) = complete {
                    Some(notif(message, Some(command), received_unix_ms))
                } else {
                    self.pending.insert(key, command);
                    None
                }
            }
            Some(ReassemblyEvent::DroppedInvalidChecksum(_)) => {
                self.frames.remove(&key);
                self.pending.remove(&key);
                None
            }
            Some(ReassemblyEvent::BufferedStarted(_))
            | Some(ReassemblyEvent::BufferedAppended(_)) => None,
            Some(_) => None,
            None => complete.map(|message| {
                let command = self.pending.remove(&key);
                notif(message, command, received_unix_ms)
            }),
        }
    }

    fn frame(&mut self, key: Key, decoded: &Decoded) -> Option<FrameState> {
        let first = decoded.frame.flags.contains(FrameFlags::FIRST);
        let last = decoded.frame.flags.contains(FrameFlags::LAST);

        if first {
            self.frames.insert(key, FrameState::new(decoded));
        } else {
            let state = self.frames.get_mut(&key)?;
            state.push(decoded);
        }

        last.then(|| self.frames.remove(&key)).flatten()
    }
}

impl FrameState {
    fn new(decoded: &Decoded) -> Self {
        Self {
            utc: decoded.utc,
            submode: decoded.frame.submode as u8,
            frame_type: decoded.frame.frame_type as u8,
            flags: decoded.frame.flags,
            frame_count: 1,
            text: decoded.frame.message.clone(),
            frequency_hz: decoded.frequency_hz,
            snr_db: decoded.snr,
            time_offset_seconds: decoded.time_offset_seconds,
            quality: decoded.quality,
            sample_position: decoded.sample_position,
        }
    }

    fn push(&mut self, decoded: &Decoded) {
        self.flags |= decoded.frame.flags;
        self.frame_count = self.frame_count.saturating_add(1);
        self.text.push_str(&decoded.frame.message);
        self.quality = self.quality.min(decoded.quality);
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
            let checksum = command.checksum.map(|checksum| {
                stripck(&mut message.text, &checksum.value);
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
        let built = build_frames(
            &BuildFramesOptions::new("K2XYZ/P MSG HELLO FROM A COMPOUND CALL", Submode::Fast)
                .with_station("K1ABC/P", "EM73"),
        );
        let mut assembler = Assembler::new();
        let mut message = None;
        for frame in &built.frames {
            message = assembler.push(&decoded(frame, Submode::Fast), 789);
        }

        let message = message.expect("compound directed message should complete");
        assert!(message.text.contains("HELLO FROM A COMPOUND CALL"));
        assert_eq!(
            message.payload.as_deref(),
            Some("HELLO FROM A COMPOUND CALL")
        );
        assert!(message.checksum.is_some());
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
}
