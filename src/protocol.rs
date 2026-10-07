// SPDX-License-Identifier: BSD-3-Clause

//! Newline-delimited JSON notification schema.

use serde::{Deserialize, Serialize};

/// A successfully decoded, complete JS8 message
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Notification {
    /// Time at the end of the block in unix ms
    pub received_unix_ms: u64,
    /// HHMMSS
    pub utc: u32,
    /// JS8 submode (`0` normal, `1` fast, `2` turbo, `4` slow, `8` ultra, these consts come from JS8Call).
    pub submode: u8,
    /// Type of the first frame
    pub frame_type: u8,
    /// Combined flag bits
    pub flags: u8,
    /// Total frames
    pub frame_count: u16,
    /// Human-readable message text.
    pub text: String,
    /// Directed-message source if it exists.
    pub from: Option<String>,
    /// Directed-message destination if it exists.
    pub to: Option<String>,
    /// Directed command without surrounding spaces, if present.
    pub command: Option<String>,
    /// Directed payload with its checksum removed, if present
    pub payload: Option<String>,
    /// Checksum when applicable.
    pub checksum: Option<Checksum>,
    /// Frequency offset of the first frame.
    pub frequency_hz: f32,
    /// SNR in dB of the first frame.
    pub snr_db: i32,
    /// First frame's timing offset.
    pub time_offset_seconds: f32,
    /// Worst decoder quality of all the frames.
    pub quality: f32,
    /// First frame's position in 12 kHz samples since the last stream reset.
    /// Doesn't matter for aligned decodes.
    pub sample_position: Option<u64>,
}

/// Checksum carried by a buffered directed JS8 message.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Checksum {
    /// Checksum width in bits.
    pub size_bits: u8,
    /// Encoded checksum text received over the air.
    pub value: String,
}
