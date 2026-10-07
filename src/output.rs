// SPDX-License-Identifier: BSD-3-Clause

use std::{
    fs,
    io::{self, Write},
    os::unix::net::{UnixListener, UnixStream},
    path::Path,
    sync::atomic::{AtomicBool, Ordering},
    thread,
    time::Duration,
};

use ajd::protocol::Notification;

// Own the pathname as well as the listener, so early returns will unlink it too
pub struct Sock(UnixListener);

impl Sock {
    pub fn bind(path: impl AsRef<Path>) -> io::Result<Self> {
        let sock = Self(UnixListener::bind(path)?);
        sock.0.set_nonblocking(true)?;
        Ok(sock)
    }

    pub fn accept(&self, stop: &AtomicBool) -> io::Result<Option<UnixStream>> {
        while !stop.load(Ordering::Acquire) {
            match self.0.accept() {
                Ok((stream, _)) => {
                    stream.set_nonblocking(false)?;
                    stream.set_write_timeout(Some(Duration::from_secs(5)))?;
                    return Ok(Some(stream));
                }
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                    // ponytail: 100 ms startup polling; use poll if latency matters.
                    thread::sleep(Duration::from_millis(100));
                }
                Err(error) if error.kind() == io::ErrorKind::Interrupted => {}
                Err(error) => return Err(error),
            }
        }
        Ok(None)
    }
}

impl Drop for Sock {
    fn drop(&mut self) {
        if let Ok(addr) = self.0.local_addr()
            && let Some(path) = addr.as_pathname()
        {
            let _ = fs::remove_file(path);
        }
    }
}

pub fn send(
    output: &mut impl Write,
    message: &Notification,
    buffer: &mut Vec<u8>,
) -> io::Result<()> {
    buffer.clear();
    serde_json::to_writer(&mut *buffer, message)?;
    buffer.push(b'\n');
    output.write_all(buffer)?;
    output.flush()
}

#[cfg(test)]
mod tests {
    use std::{
        io::{BufRead, BufReader, Read},
        os::unix::fs::FileTypeExt,
        sync::mpsc,
    };

    use crate::message::Assembler;
    use js8rs::{
        codec::{BuildFramesOptions, build_frames, parse_frame},
        protocol::Submode,
        rx::Decoded,
    };

    use super::*;

    #[test]
    fn sock_io() {
        let built = build_frames(&BuildFramesOptions::new("HELLO", Submode::Ultra));
        let frame = &built.frames[0];
        let decoded = Decoded::new(
            parse_frame(&frame.encoded, frame.flags, built.submode),
            123_456,
            -10,
            0.1,
            1_500.0,
            0.9,
        );
        let mut message = Assembler::new().push(&decoded, 1_234).unwrap();
        message.text = "quotes: \" slash: \\ newline:\nUTF-8: æ".into();
        let mut buffer = Vec::new();
        let mut expected = Vec::new();
        send(&mut expected, &message, &mut buffer).unwrap();
        send(&mut expected, &message, &mut buffer).unwrap();
        assert_eq!(expected.iter().filter(|&&byte| byte == b'\n').count(), 2);

        let dir = std::env::temp_dir().join(format!("ajd-sock-{}", std::process::id()));
        fs::create_dir(&dir).unwrap();
        let path = dir.join("out.sock");
        let sock = Sock::bind(&path).unwrap();
        assert!(fs::metadata(&path).unwrap().file_type().is_socket());
        assert!(Sock::bind(&path).is_err());
        let mut stream = UnixStream::connect(&path).unwrap();
        let mut output = sock.accept(&AtomicBool::new(false)).unwrap().unwrap();
        assert_eq!(
            output.write_timeout().unwrap(),
            Some(Duration::from_secs(5))
        );
        send(&mut output, &message, &mut buffer).unwrap();
        send(&mut output, &message, &mut buffer).unwrap();
        drop(output);
        stream
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let mut bytes = Vec::new();
        stream.read_to_end(&mut bytes).unwrap();
        assert_eq!(bytes, expected);
        for line in BufReader::new(bytes.as_slice()).lines() {
            assert_eq!(
                serde_json::from_str::<Notification>(&line.unwrap()).unwrap(),
                message
            );
        }
        drop(sock);
        assert!(!path.exists());

        // Rebind, then stop while waiting for a reader.
        let sock = Sock::bind(&path).unwrap();
        let stop = AtomicBool::new(false);
        let (tx, rx) = mpsc::channel();
        thread::scope(|scope| {
            scope.spawn(|| {
                let result = sock.accept(&stop).map(|stream| stream.is_none());
                tx.send(result).unwrap();
            });
            assert!(rx.recv_timeout(Duration::from_millis(150)).is_err());
            stop.store(true, Ordering::Release);
            assert!(rx.recv_timeout(Duration::from_secs(2)).unwrap().unwrap());
        });
        // A disconnected reader must fail the write.
        stop.store(false, Ordering::Release);
        let stream = UnixStream::connect(&path).unwrap();
        let mut output = sock.accept(&stop).unwrap().unwrap();
        send(&mut output, &message, &mut buffer).unwrap();
        drop(stream);
        assert!(send(&mut output, &message, &mut buffer).is_err());
        drop(output);
        drop(sock);
        assert!(!path.exists());

        // Never replace an existing file or socket.
        fs::write(&path, b"keep me").unwrap();
        assert!(Sock::bind(&path).is_err());
        assert_eq!(fs::read(&path).unwrap(), b"keep me");
        fs::remove_file(path).unwrap();
        fs::remove_dir(dir).unwrap();
        assert!(send(&mut [].as_mut_slice(), &message, &mut buffer).is_err());
    }
}
