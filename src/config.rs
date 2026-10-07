// SPDX-License-Identifier: BSD-3-Clause

use js8rs::protocol::DecodeModes;
use std::{env, fmt::Display, io, str::FromStr};

pub const RATE: u32 = 12_000;
const LOW_CUT: f32 = 200.0;
const HIGH_CUT: f32 = 4_000.0;
const GAIN: f32 = 0.25;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Mode {
    Usb,
    Lsb,
}

impl Mode {
    fn parse(value: &str) -> io::Result<Self> {
        match value {
            "usb" => Ok(Self::Usb),
            "lsb" => Ok(Self::Lsb),
            _ => Err(invalid(format!("unsupported AJD_MODE: {value}"))),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct DecodeSettings {
    pub modes: DecodeModes,
    pub min_frequency: u32,
    pub max_frequency: u32,
    pub nominal_frequency: u32,
}

#[derive(Debug)]
pub struct Config {
    pub frequency: f64,
    pub device_rate: Option<u32>,
    pub serial: Option<u64>,
    pub calibration: Option<i32>,
    pub mode: Mode,
    pub low_cut: f32,
    pub high_cut: f32,
    pub sock: Option<String>,
    pub decode: DecodeSettings,
    pub hf_agc: bool,
    pub hf_agc_high: Option<bool>,
    pub attenuation: Option<f32>,
    pub bias_tee: Option<i8>,
    pub agc: bool,
    pub gain: f32,
}

impl Config {
    pub fn read() -> io::Result<Self> {
        let frequency: f64 =
            value("AJD_FREQUENCY")?.ok_or_else(|| invalid("AJD_FREQUENCY is required"))?;
        let device_rate: Option<u32> = value("AJD_DEVICE_RATE")?;
        let serial = text("AJD_SERIAL")?
            .map(|value| serial(&value))
            .transpose()?;
        let calibration: Option<i32> = value("AJD_CALIBRATION")?;
        let mode = text("AJD_MODE")?
            .as_deref()
            .map(Mode::parse)
            .transpose()?
            .unwrap_or(Mode::Usb);
        let low_cut: f32 = value("AJD_LOW_CUT")?.unwrap_or(LOW_CUT);
        let high_cut: f32 = value("AJD_HIGH_CUT")?.unwrap_or(HIGH_CUT);
        let hf_agc = value("AJD_HF_AGC")?.unwrap_or(true);
        let hf_agc_high = text("AJD_HF_AGC_THRESHOLD")?
            .map(|value| match value.as_str() {
                "low" => Ok(false),
                "high" => Ok(true),
                _ => Err(invalid(format!(
                    "unsupported AJD_HF_AGC_THRESHOLD: {value}"
                ))),
            })
            .transpose()?;
        let attenuation: Option<f32> = value("AJD_ATTENUATION")?;
        let bias_tee: Option<i8> = value("AJD_BIAS_TEE")?;
        let agc = value("AJD_AGC")?.unwrap_or(true);
        let gain: f32 = value("AJD_GAIN")?.unwrap_or(GAIN);
        let sock = text("AJD_SOCK")?;
        let decode = DecodeSettings {
            modes: text("AJD_MODES")?
                .as_deref()
                .map(modes)
                .transpose()?
                .unwrap_or(DecodeModes::ALL),
            min_frequency: value("AJD_MIN_FREQUENCY")?.unwrap_or(200),
            max_frequency: value("AJD_MAX_FREQUENCY")?.unwrap_or(4_000),
            nominal_frequency: value("AJD_NOMINAL_FREQUENCY")?.unwrap_or(1_500),
        };
        decode.validate()?;

        if !frequency.is_finite() || frequency <= 0.0 {
            return Err(invalid("AJD_FREQUENCY must be finite and > 0"));
        }
        if !low_cut.is_finite() || low_cut <= 0.0 {
            return Err(invalid("AJD_LOW_CUT must be finite and > 0"));
        }
        if !high_cut.is_finite() || high_cut <= low_cut || high_cut >= RATE as f32 * 0.5 {
            return Err(invalid(
                "AJD_HIGH_CUT must be finite, above AJD_LOW_CUT, and below Nyquist",
            ));
        }
        if attenuation.is_some_and(|value| !value.is_finite() || value < 0.0) {
            return Err(invalid("AJD_ATTENUATION must be finite and >= 0"));
        }
        if !gain.is_finite() || gain <= 0.0 {
            return Err(invalid("AJD_GAIN must be finite and > 0"));
        }

        Ok(Self {
            frequency,
            device_rate,
            serial,
            calibration,
            mode,
            low_cut,
            high_cut,
            sock,
            decode,
            hf_agc,
            hf_agc_high,
            attenuation,
            bias_tee,
            agc,
            gain,
        })
    }
}

impl DecodeSettings {
    fn validate(&self) -> io::Result<()> {
        if self.min_frequency >= self.max_frequency || self.max_frequency > 5_000 {
            return Err(invalid(
                "AJD decode frequency range must be increasing and no higher than 5000 Hz",
            ));
        }
        if !(self.min_frequency..=self.max_frequency).contains(&self.nominal_frequency) {
            return Err(invalid(
                "AJD_NOMINAL_FREQUENCY must be inside the decode range",
            ));
        }
        Ok(())
    }
}

fn modes(value: &str) -> io::Result<DecodeModes> {
    if value == "all" {
        return Ok(DecodeModes::ALL);
    }
    let mut modes = DecodeModes::NONE;
    for name in value.split(',') {
        modes |= match name.trim() {
            "normal" => DecodeModes::NORMAL,
            "fast" => DecodeModes::FAST,
            "turbo" => DecodeModes::TURBO,
            "slow" => DecodeModes::SLOW,
            "ultra" => DecodeModes::ULTRA,
            name => return Err(invalid(format!("unsupported AJD_MODES entry: {name}"))),
        };
    }
    Ok(modes)
}

fn text(name: &str) -> io::Result<Option<String>> {
    match env::var(name) {
        Ok(value) if value.is_empty() => Err(invalid(format!("{name} cannot be empty"))),
        Ok(value) => Ok(Some(value)),
        Err(env::VarError::NotPresent) => Ok(None),
        Err(env::VarError::NotUnicode(_)) => Err(invalid(format!("{name} is not valid UTF-8"))),
    }
}

fn value<T>(name: &str) -> io::Result<Option<T>>
where
    T: FromStr,
    T::Err: Display,
{
    text(name)?
        .map(|value| {
            value
                .parse()
                .map_err(|error| invalid(format!("invalid {name}: {error}")))
        })
        .transpose()
}

fn serial(value: &str) -> io::Result<u64> {
    let value = value
        .strip_prefix("0x")
        .or_else(|| value.strip_prefix("0X"))
        .unwrap_or(value);
    u64::from_str_radix(value, 16).map_err(|error| invalid(format!("invalid AJD_SERIAL: {error}")))
}

pub fn invalid(message: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, message.into())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode_parse() {
        assert_eq!(Mode::parse("usb").unwrap(), Mode::Usb);
        assert_eq!(Mode::parse("lsb").unwrap(), Mode::Lsb);
        assert!(Mode::parse("am").is_err());
        assert_eq!(modes("all").unwrap(), DecodeModes::ALL);
        assert_eq!(
            modes("normal, turbo").unwrap(),
            DecodeModes::NORMAL | DecodeModes::TURBO
        );
        assert!(modes("").is_err());
        assert!(modes("normal,bogus").is_err());
    }

    #[test]
    fn cfg_check() {
        let mut settings = DecodeSettings {
            modes: DecodeModes::ALL,
            min_frequency: 200,
            max_frequency: 4_000,
            nominal_frequency: 1_500,
        };
        assert!(settings.validate().is_ok());
        settings.nominal_frequency = 100;
        assert!(settings.validate().is_err());
        settings.nominal_frequency = 1_500;
        settings.max_frequency = 6_000;
        assert!(settings.validate().is_err());
        settings.max_frequency = settings.min_frequency;
        assert!(settings.validate().is_err());
    }

    #[test]
    fn serial_parse() {
        assert_eq!(serial("0123456789abcdef").unwrap(), 0x0123_4567_89ab_cdef);
        assert_eq!(serial("0x10").unwrap(), 0x10);
        assert!(serial("not-hex").is_err());
    }
}
