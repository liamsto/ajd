Mostly airspy env vars:
| Variable | Default | Meaning |
| --- | --- | --- |
| `AJD_FREQUENCY` | required | Dial/suppressed-carrier frequency in Hz |
| `AJD_SOCK` | unset | Unix stream socket for notifications otherwise stdout |
| `AJD_MODES` | `all` | `all` or a comma-separated `normal,fast,turbo,slow,ultra` |
| `AJD_MIN_FREQUENCY` | `200` | Lower audio-frequency search bound in Hz |
| `AJD_MAX_FREQUENCY` | `4000` | Upper audio-frequency search bound in Hz <= 5000 |
| `AJD_NOMINAL_FREQUENCY` | `1500` | Audio frequency prioritized by the decoder inside the search range |
| `AJD_SERIAL` | first device | AirSpy serial in hexadecimal, optionally prefixed with `0x` |
| `AJD_DEVICE_RATE` | auto | Hardware sample rate, an integer multiple of 12000 Hz |
| `AJD_MODE` | `usb` | Receive sideband: `usb` or `lsb` |
| `AJD_LOW_CUT` | `200` | Demodulator audio passband lower edge in Hz |
| `AJD_HIGH_CUT` | `4000` | Demodulator audio passband upper edge in Hz, below 6000 |
| `AJD_HF_AGC` | `true` | Enable AirSpy HF hardware AGC |
| `AJD_HF_AGC_THRESHOLD` | device default | Hardware AGC threshold: `low` or `high` |
| `AJD_ATTENUATION` | unset | Manual attenuation in dB this will disable hardware AGC |
| `AJD_AGC` | `true` | Enable software IF AGC |
| `AJD_GAIN` | `0.25` | Demodulator gain, nominal peak level with software AGC |
| `AJD_CALIBRATION` | device flash | Runtime frequency correction in parts per billion |
| `AJD_BIAS_TEE` | unchanged | AirSpy bias-tee selection/value |

cargo build --release --features experimental-time
AJD_FREQUENCY=7078000 AJD_MODES=fast,ultra \
    ./target/release/ajd
