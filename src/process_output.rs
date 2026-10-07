//! Drain child diagnostics without blocking a child or growing GUI history.
use std::{
    collections::VecDeque,
    io::Read,
    os::fd::AsRawFd,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
};
const CAPACITY: usize = 32 * 1024;

pub(crate) struct Tail {
    bytes: Arc<Mutex<VecDeque<u8>>>,
    truncated: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    reader: Option<std::thread::JoinHandle<Result<(), String>>>,
    error: Option<String>,
}

impl Tail {
    pub(crate) fn drain(mut input: impl Read + AsRawFd + Send + 'static) -> Result<Self, String> {
        let descriptor = input.as_raw_fd();
        let flags = unsafe { libc::fcntl(descriptor, libc::F_GETFL) };
        if flags < 0
            || unsafe { libc::fcntl(descriptor, libc::F_SETFL, flags | libc::O_NONBLOCK) } < 0
        {
            return Err(format!(
                "Cannot make diagnostic pipe nonblocking: {}",
                std::io::Error::last_os_error()
            ));
        }
        let bytes = Arc::new(Mutex::new(VecDeque::new()));
        let truncated = Arc::new(AtomicBool::new(false));
        let stop = Arc::new(AtomicBool::new(false));
        let destination = bytes.clone();
        let lost = truncated.clone();
        let stopped = stop.clone();
        let reader = std::thread::Builder::new()
            .name("child-stderr".into())
            .spawn(move || {
                let mut buffer = [0; 8192];
                let mut final_bytes = 0;
                loop {
                    match input.read(&mut buffer) {
                        Ok(0) => return Ok(()),
                        Ok(count) => {
                            let Ok(mut tail) = destination.lock() else {
                                return Err("Diagnostic tail lock poisoned".into());
                            };
                            for byte in &buffer[..count] {
                                if tail.len() == CAPACITY {
                                    tail.pop_front();
                                    lost.store(true, Ordering::Release);
                                }
                                tail.push_back(*byte);
                            }
                            if stopped.load(Ordering::Acquire) {
                                final_bytes += count;
                                if final_bytes >= 256 * 1024 {
                                    return Err("Diagnostic stream exceeded the bounded final drain".into());
                                }
                            }
                        }
                        Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {
                            if stopped.load(Ordering::Acquire) {
                                return Err("Diagnostic pipe remained open after child cleanup; final tail may be incomplete".into());
                            }
                            std::thread::park_timeout(std::time::Duration::from_millis(10));
                        }
                        Err(error) if error.kind() == std::io::ErrorKind::Interrupted => (),
                        Err(error) => return Err(format!("Child stderr read failed: {error}")),
                    }
                }
            })
            .map_err(|e| e.to_string())?;
        Ok(Self {
            bytes,
            truncated,
            stop,
            reader: Some(reader),
            error: None,
        })
    }

    pub(crate) fn finish(&mut self) -> Result<(), String> {
        self.stop.store(true, Ordering::Release);
        if let Some(reader) = self.reader.take() {
            reader.thread().unpark();
            self.error = reader
                .join()
                .map_err(|_| "Diagnostic reader panicked".to_owned())
                .and_then(|result| result)
                .err();
        }
        self.error.clone().map_or(Ok(()), Err)
    }

    pub(crate) fn text(&self) -> String {
        let bytes = self
            .bytes
            .lock()
            .map(|bytes| bytes.iter().copied().collect::<Vec<_>>())
            .unwrap_or_default();
        let prefix = if self.truncated.load(Ordering::Acquire) {
            "[earlier stderr omitted]\n"
        } else {
            ""
        };
        format!("{prefix}{}", String::from_utf8_lossy(&bytes))
    }
}

impl Drop for Tail {
    fn drop(&mut self) {
        let _ = self.finish();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn drains_more_than_a_pipe_and_retains_only_bounded_tail() {
        let (reader, mut writer) = std::os::unix::net::UnixStream::pair().unwrap();
        let mut tail = Tail::drain(reader).unwrap();
        use std::io::Write as _;
        std::thread::scope(|scope| {
            scope.spawn(move || {
                writer.write_all(&vec![b'x'; 256 * 1024]).unwrap();
                writer.write_all(b"last cause").unwrap();
            });
        });
        tail.finish().unwrap();
        let text = tail.text();
        assert!(text.starts_with("[earlier stderr omitted]"));
        assert!(text.ends_with("last cause"));
        assert!(text.len() <= CAPACITY + 25);
    }

    #[test]
    fn held_pipe_is_bounded_and_read_failure_is_not_eof() {
        let (reader, _held_writer) = std::os::unix::net::UnixStream::pair().unwrap();
        let mut tail = Tail::drain(reader).unwrap();
        assert!(tail.finish().unwrap_err().contains("remained open"));
        assert!(tail.reader.is_none());
        let directory = tempfile::tempdir().unwrap();
        let file = std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(directory.path().join("write-only"))
            .unwrap();
        let mut tail = Tail::drain(file).unwrap();
        assert!(tail.finish().unwrap_err().contains("read failed"));
        assert!(tail.reader.is_none());
    }
}
