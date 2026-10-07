mod airspy;
mod config;
mod decode;
mod dsp;
mod message;
mod output;

use std::{
    error::Error,
    io::{self, Write},
    ops::ControlFlow,
    sync::{
        Arc,
        atomic::AtomicBool,
        mpsc::{self, TrySendError},
    },
    thread,
};

use airspy::{BLOCK, Device};
use config::{Config, RATE};
use decode::Packet;
use dsp::Demod;
use js8rs::timing::unix_time_ms;
use output::Sock;
use signal_hook::{
    consts::{SIGINT, SIGTERM},
    flag,
};

const QUEUE_SECONDS: usize = 2;

type Result<T> = std::result::Result<T, Box<dyn Error + Send + Sync>>;

#[derive(Default)]
struct Stats {
    device_dropped: u64,
    audio_dropped: u64,
}

fn main() -> Result<()> {
    let config = Config::read()?;
    let stopping = Arc::new(AtomicBool::new(false));
    flag::register_conditional_shutdown(SIGINT, 130, Arc::clone(&stopping))?;
    flag::register_conditional_shutdown(SIGTERM, 143, Arc::clone(&stopping))?;
    flag::register(SIGINT, Arc::clone(&stopping))?;
    flag::register(SIGTERM, Arc::clone(&stopping))?;

    let sock = config.sock.as_deref().map(Sock::bind).transpose()?;
    let output: Box<dyn Write + Send> = if let Some(sock) = &sock {
        eprintln!("waiting for reader: {}", config.sock.as_deref().unwrap());
        let Some(stream) = sock.accept(&stopping)? else {
            return Ok(());
        };
        Box::new(stream)
    } else {
        Box::new(io::stdout())
    };
    let radio = Device::open(&config)?;
    let rate = radio.rate;

    let mut demod = Demod::new(
        rate,
        RATE,
        config.low_cut..config.high_cut,
        config.mode,
        config.agc,
        config.gain,
    )?;
    let factor = (rate / RATE) as usize;
    let capacity = BLOCK.div_ceil(factor);
    let buffers = (rate as usize * QUEUE_SECONDS).div_ceil(BLOCK).max(2);
    let (free_tx, free_rx) = mpsc::sync_channel(buffers);
    for _ in 0..buffers {
        free_tx.send(Vec::with_capacity(capacity))?;
    }
    let (packet_tx, packet_rx) = mpsc::sync_channel(buffers - 1);
    let mut stats = Stats::default();
    let callback_stats = &mut stats;
    let callback_free = free_tx.clone();
    let mut discontinuity = false;
    let mut next = None;

    eprintln!(
        "receiving: {:.0} Hz | {:?}, {rate} -> {RATE} Hz | {:.0}-{:.0} Hz",
        config.frequency, config.mode, config.low_cut, config.high_cut,
    );
    eprintln!(
        "decode: {:?}, {} | {}-{} Hz, nominal {} Hz",
        config.decode.modes,
        if cfg!(feature = "experimental-time") {
            "untimed"
        } else {
            "UTC slots"
        },
        config.decode.min_frequency,
        config.decode.max_frequency,
        config.decode.nominal_frequency,
    );
    eprintln!(
        "output: {} | newline-delimited JSON",
        config.sock.as_deref().unwrap_or("stdout"),
    );

    let worker = thread::spawn(move || decode::run(config.decode, output, packet_rx, free_tx));
    let decoder = &worker;
    let capture = radio.recv(&stopping, move |samples, dropped_samples| {
        if decoder.is_finished() {
            return ControlFlow::Break(());
        }
        callback_stats.device_dropped += dropped_samples;

        let skipped = demod.skip(dropped_samples);
        discontinuity |= dropped_samples != 0;
        // ADvance by sample count, backlog can't move across slots
        let start = next.map_or_else(
            || {
                (unix_time_ms() * u64::from(RATE) / 1_000)
                    .saturating_sub((samples.len() / factor) as u64)
            },
            |end| end + skipped,
        );

        let Ok(mut packet) = free_rx.try_recv() else {
            let dropped = demod.process(samples, None);
            next = Some(start + dropped as u64);
            callback_stats.audio_dropped += dropped as u64;
            discontinuity |= dropped != 0;
            return ControlFlow::Continue(());
        };

        packet.clear();
        let frames = demod.process(samples, Some(&mut packet));
        next = Some(start + frames as u64);
        if frames == 0 {
            let _ = callback_free.try_send(packet);
            return ControlFlow::Continue(());
        }
        let packet = Packet {
            data: packet,
            start,
            discontinuity,
        };
        match packet_tx.try_send(packet) {
            Ok(()) => {
                discontinuity = false;
                ControlFlow::Continue(())
            }
            Err(TrySendError::Full(mut packet)) => {
                callback_stats.audio_dropped += frames as u64;
                packet.data.clear();
                let _ = callback_free.try_send(packet.data);
                discontinuity = true;
                ControlFlow::Continue(())
            }
            Err(TrySendError::Disconnected(_)) => ControlFlow::Break(()),
        }
    });
    let decoded = worker
        .join()
        .map_err(|_| io::Error::other("decoder thread panicked"))??;
    capture?;

    let device_dropped = stats.device_dropped;
    let audio_dropped = stats.audio_dropped;
    eprintln!(
        "stopped: {} passes, {} frames, {} messages, {} discontinuities; {device_dropped} device samples dropped, {audio_dropped} audio samples dropped",
        decoded.passes, decoded.frames, decoded.messages, decoded.discontinuities,
    );
    Ok(())
}
