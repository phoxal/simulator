//! Setup-owned terminal feedback; final readiness remains on stdout.
use super::terminal::Terminal;
use std::{
    sync::{Arc, Condvar, Mutex},
    thread::JoinHandle,
    time::{Duration, Instant},
};

const HEARTBEAT: Duration = Duration::from_secs(5);
const FRAME: Duration = Duration::from_millis(200);
struct State {
    phase: &'static str,
    since: Instant,
    data_since: Instant,
    generation: u64,
    bytes: Option<(u64, Option<u64>)>,
    finished: bool,
    cancelled: bool,
}
pub(super) struct Progress {
    shared: Arc<(Mutex<State>, Condvar)>,
    owner: Option<JoinHandle<()>>,
    output: Arc<Mutex<Terminal>>,
}
impl Progress {
    pub(super) fn start() -> Self {
        Self::with_terminal(Terminal::new())
    }
    fn with_terminal(mut terminal: Terminal) -> Self {
        let now = Instant::now();
        let shared = Arc::new((
            Mutex::new(State {
                phase: "Preparing MuJoCo 3.12.0",
                since: now,
                data_since: now,
                generation: 0,
                bytes: None,
                finished: false,
                cancelled: false,
            }),
            Condvar::new(),
        ));
        terminal.line("Setup: Preparing MuJoCo 3.12.0.");
        let output = Arc::new(Mutex::new(terminal));
        let writer = output.clone();
        let worker = shared.clone();
        let owner = std::thread::spawn(move || {
            let (mutex, changed) = &*worker;
            let mut state = mutex.lock().unwrap_or_else(|e| e.into_inner());
            let mut generation = state.generation;
            let mut next_plain = state.since + HEARTBEAT;
            loop {
                if state.finished {
                    break;
                }
                drop(state);
                let interval = if writer.lock().unwrap_or_else(|e| e.into_inner()).live() {
                    FRAME
                } else {
                    HEARTBEAT
                };
                state = mutex.lock().unwrap_or_else(|e| e.into_inner());
                if state.finished {
                    break;
                }
                let (current, _) = changed
                    .wait_timeout(state, interval)
                    .unwrap_or_else(|e| e.into_inner());
                state = current;
                if state.finished {
                    break;
                }
                drop(state);
                let mut terminal = writer.lock().unwrap_or_else(|e| e.into_inner());
                state = mutex.lock().unwrap_or_else(|e| e.into_inner());
                if state.finished {
                    break;
                }
                if generation != state.generation {
                    generation = state.generation;
                    next_plain = state.since + HEARTBEAT;
                }
                let stalled =
                    state.phase == "Downloading" && state.data_since.elapsed() >= HEARTBEAT;
                let message = if terminal.live() {
                    Some(live_line(
                        state.phase,
                        state.bytes,
                        state.since.elapsed(),
                        stalled,
                        terminal.frame(),
                    ))
                } else if Instant::now() >= next_plain {
                    next_plain = Instant::now() + HEARTBEAT;
                    Some(line(
                        state.phase,
                        state.bytes,
                        state.since.elapsed(),
                        stalled,
                    ))
                } else {
                    None
                };
                drop(state);
                if let Some(message) = message
                    && !terminal.update(&message)
                {
                    terminal.line(&message);
                }
                drop(terminal);
                state = mutex.lock().unwrap_or_else(|e| e.into_inner());
            }
        });
        Self {
            shared,
            owner: Some(owner),
            output,
        }
    }
    pub(super) fn phase(&self, phase: &'static str) {
        let mut terminal = self.output.lock().unwrap_or_else(|e| e.into_inner());
        {
            let mut state = self.shared.0.lock().unwrap_or_else(|e| e.into_inner());
            if state.cancelled {
                return;
            }
            state.phase = phase;
            state.since = Instant::now();
            state.data_since = state.since;
            state.generation += 1;
            state.bytes = None;
        }
        let text = format!("Setup: {phase}.");
        terminal.line(&text);
        self.shared.1.notify_all();
    }
    pub(super) fn cancel(&self) {
        let mut terminal = self.output.lock().unwrap_or_else(|e| e.into_inner());
        {
            let mut state = self.shared.0.lock().unwrap_or_else(|e| e.into_inner());
            state.cancelled = true;
            state.phase = "Cancellation requested. Waiting for cleanup";
            state.since = Instant::now();
            state.generation += 1;
            state.bytes = None;
        }
        terminal.line("Setup: Cancellation requested. Waiting for cleanup.");
        self.shared.1.notify_all();
    }
    pub(super) fn verified_reuse(&self) {
        let mut terminal = self.output.lock().unwrap_or_else(|e| e.into_inner());
        {
            let mut state = self.shared.0.lock().unwrap_or_else(|e| e.into_inner());
            if state.cancelled {
                return;
            }
            state.finished = true;
        }
        terminal.line("Setup: Existing installation verified. No download needed.");
        self.shared.1.notify_all();
    }
    pub(super) fn report_bytes(&self) {
        let mut terminal = self.output.lock().unwrap_or_else(|e| e.into_inner());
        let state = self.shared.0.lock().unwrap_or_else(|e| e.into_inner());
        if state.cancelled {
            return;
        }
        let text = if terminal.live() {
            live_line(state.phase, state.bytes, state.since.elapsed(), false, ' ')
        } else {
            line(state.phase, state.bytes, state.since.elapsed(), false)
        };
        drop(state);
        terminal.line(&text);
    }
    pub(super) fn bytes(&self, received: u64, total: Option<u64>) {
        let mut state = self.shared.0.lock().unwrap_or_else(|e| e.into_inner());
        if !state.cancelled {
            if state.bytes.is_none_or(|(previous, _)| previous != received) {
                state.data_since = Instant::now();
            }
            state.bytes = Some((received, total.filter(|total| *total > 0)));
        }
    }
}
impl Drop for Progress {
    fn drop(&mut self) {
        self.shared
            .0
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .finished = true;
        self.shared.1.notify_all();
        if let Some(owner) = self.owner.take() {
            let _ = owner.join();
        }
        self.output
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clear();
    }
}
fn line(
    phase: &str,
    bytes: Option<(u64, Option<u64>)>,
    elapsed: Duration,
    stalled: bool,
) -> String {
    let phase = if stalled {
        "Waiting for download data"
    } else {
        phase
    };
    let amount = match bytes {
        Some((received, Some(total))) => format!(
            " - {:.1} MiB of {:.1} MiB,",
            received as f64 / 1048576.0,
            total as f64 / 1048576.0
        ),
        Some((received, None)) => format!(" - {:.1} MiB,", received as f64 / 1048576.0),
        None => " -".into(),
    };
    format!("Setup: {phase}{amount} {}s elapsed.", elapsed.as_secs())
}
fn live_line(
    phase: &str,
    bytes: Option<(u64, Option<u64>)>,
    elapsed: Duration,
    stalled: bool,
    frame: char,
) -> String {
    if phase == "Downloading" {
        let phase = if stalled {
            "Waiting for data"
        } else {
            "Downloading"
        };
        if let Some((received, Some(total))) = bytes
            && total > 0
            && received <= total
        {
            let filled = ((received as u128 * 8) / u128::from(total)) as usize;
            let percent = (received as u128 * 100) / u128::from(total);
            let bar = format!("{}{}", "=".repeat(filled), " ".repeat(8 - filled));
            return format!(
                "Setup: {frame} {phase} [{bar}] {percent:3}% {:.1}/{:.1} MiB, {}s",
                received as f64 / 1048576.0,
                total as f64 / 1048576.0,
                elapsed.as_secs()
            );
        }
        let amount = bytes.map_or_else(
            || "waiting for response".into(),
            |(received, _)| format!("{:.1} MiB received", received as f64 / 1048576.0),
        );
        return format!(
            "Setup: {frame} {phase} - {amount}, {}s elapsed.",
            elapsed.as_secs()
        );
    }
    format!("Setup: {frame} {phase} - {}s elapsed.", elapsed.as_secs())
}
#[cfg(test)]
mod tests {
    use super::*;
    use std::{fs, io::Write, net::TcpListener};
    #[allow(dead_code)]
    mod pty {
        include!(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/support/pty.rs"));
    }

    #[test]
    fn verified_reuse_is_permanent_without_working_frames_or_elapsed_time() {
        let (mut terminal, slave) = pty::Pty::pair(100);
        let _endpoint = slave.try_clone().unwrap();
        let progress = Progress::with_terminal(Terminal::with_output(Some(slave), true));
        progress.phase("Checking existing installation");
        progress.verified_reuse();
        terminal.until("Existing installation verified. No download needed.", 2);
        let offset = terminal.bytes.len();
        std::thread::park_timeout(Duration::from_millis(400));
        terminal.drain();
        assert_eq!(terminal.bytes.len(), offset);
        let text = String::from_utf8_lossy(&terminal.bytes);
        assert!(
            text.contains("\r\nSetup: Existing installation verified. No download needed.\r\n")
        );
        assert!(!text.contains("No download needed -"));
        drop(progress);
    }

    #[test]
    fn transfer_percentage_requires_a_positive_total_and_never_rounds_up() {
        let render = |phase, bytes| live_line(phase, Some(bytes), Duration::ZERO, false, '|');
        assert!(render("Downloading", (999, Some(1000))).contains("99%"));
        assert!(render("Downloading", (1000, Some(1000))).contains("100%"));
        for bytes in [(1, None), (0, Some(0)), (1001, Some(1000))] {
            assert!(!render("Downloading", bytes).contains('%'));
        }
        assert!(!render("Checking MuJoCo compatibility", (1000, Some(1000))).contains('%'));
    }

    #[test]
    fn reporter_unwind_joins_and_clears_before_permanent_error_output() {
        let (mut terminal, slave) = pty::Pty::pair(100);
        let mut permanent = slave.try_clone().unwrap();
        let result = std::panic::catch_unwind(|| {
            let progress = Progress::with_terminal(Terminal::with_output(Some(slave), true));
            progress.phase("Downloading");
            panic!("explicit fixture failure");
        });
        assert!(result.is_err());
        writeln!(permanent, "PRIMARY_ERROR").unwrap();
        permanent.flush().unwrap();
        terminal.until("PRIMARY_ERROR", 2);
        let offset = terminal.bytes.len();
        std::thread::park_timeout(Duration::from_millis(300));
        terminal.drain();
        assert_eq!(terminal.bytes.len(), offset);
        assert!(
            !String::from_utf8_lossy(&terminal.bytes)
                .split("PRIMARY_ERROR")
                .nth(1)
                .unwrap()
                .contains('\u{1b}')
        );
    }

    #[test]
    fn actual_pty_reporter_known_unknown_stall_resize_cancel_and_final_clear() {
        for known in [true, false] {
            let (mut terminal, slave) = pty::Pty::pair(100);
            // Keep the PTY endpoint open while inspecting final output: macOS
            // can discard unread terminal data when the last slave closes.
            let _endpoint = slave.try_clone().unwrap();
            let progress = Progress::with_terminal(Terminal::with_output(Some(slave), true));
            progress.phase("Downloading");
            progress.bytes(4194304, known.then_some(8388608));
            terminal.until(
                if known {
                    "50% 4.0/8.0 MiB"
                } else {
                    "4.0 MiB received"
                },
                3,
            );
            if !known {
                assert!(!terminal.bytes.contains(&b'%'));
            }
            terminal.until("Waiting for data", 7);
            progress.bytes(8388608, known.then_some(8388608));
            progress.report_bytes();
            progress.phase("Checking MuJoCo compatibility");
            terminal.drain();
            assert!(String::from_utf8_lossy(&terminal.bytes).contains("8.0"));
            terminal.resize(60);
            std::thread::park_timeout(Duration::from_millis(400));
            terminal.drain();
            let offset = terminal.bytes.len();
            progress.cancel();
            progress.phase("Extracting runtime");
            drop(progress);
            terminal.until("Cancellation requested. Waiting for cleanup", 2);
            assert!(!terminal.bytes[offset..].contains(&27));
            let text = String::from_utf8_lossy(&terminal.bytes[offset..]);
            assert!(text.contains("Cancellation requested. Waiting for cleanup"));
            assert!(!text.contains("Extracting runtime"));
            let final_len = terminal.bytes.len();
            std::thread::park_timeout(Duration::from_millis(250));
            terminal.drain();
            assert_eq!(
                terminal.bytes.len(),
                final_len,
                "joined reporter wrote after completion"
            );
        }
    }

    #[test]
    fn short_http_download_retains_measured_known_unknown_and_empty_body_counts() {
        for total in [Some(6), None, Some(0)] {
            let listener = TcpListener::bind("127.0.0.1:0").unwrap();
            let address = listener.local_addr().unwrap();
            let server = std::thread::spawn(move || {
                use std::io::Read;
                let (mut stream, _) = listener.accept().unwrap();
                let mut request = [0; 4096];
                assert!(stream.read(&mut request).unwrap() > 0);
                let header = total
                    .map(|n| format!("Content-Length: {n}\r\n"))
                    .unwrap_or_default();
                write!(
                    stream,
                    "HTTP/1.1 200 OK\r\n{header}Connection: close\r\n\r\n"
                )
                .unwrap();
                if total != Some(0) {
                    stream.write_all(b"abcdef").unwrap();
                }
            });
            let directory = tempfile::tempdir().unwrap();
            let destination = directory.path().join("archive");
            let progress = Progress::start();
            progress.phase("Downloading");
            let runtime = tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap();
            runtime.block_on(async {
                let response = reqwest::Client::builder()
                    .no_proxy()
                    .build()
                    .unwrap()
                    .get(format!("http://{address}"))
                    .send()
                    .await
                    .unwrap();
                super::super::receive(response, &destination, &progress)
                    .await
                    .unwrap();
            });
            let received = if total == Some(0) { 0 } else { 6 };
            assert_eq!(fs::metadata(&destination).unwrap().len(), received);
            assert_eq!(
                progress.shared.0.lock().unwrap().bytes,
                Some((received, total.filter(|n| *n > 0)))
            );
            server.join().unwrap();
        }
    }

    #[test]
    fn cancelled_feedback_cannot_resume_download_or_installation_progress() {
        let progress = Progress::start();
        progress.phase("Downloading");
        progress.bytes(1024, Some(2048));
        progress.cancel();
        progress.phase("Extracting runtime");
        progress.bytes(2048, Some(2048));
        progress.report_bytes();
        let state = progress.shared.0.lock().unwrap();
        assert_eq!(state.phase, "Cancellation requested. Waiting for cleanup");
        assert_eq!(state.bytes, None);
    }

    #[test]
    fn measured_lines_are_plain_and_truthful_for_narrow_redirected_and_stalled_output() {
        assert_eq!(
            line(
                "Downloading",
                Some((4194304, Some(8388608))),
                Duration::from_secs(5),
                false
            ),
            "Setup: Downloading - 4.0 MiB of 8.0 MiB, 5s elapsed."
        );
        assert_eq!(
            line(
                "Downloading",
                Some((4194304, None)),
                Duration::from_secs(10),
                true
            ),
            "Setup: Waiting for download data - 4.0 MiB, 10s elapsed."
        );
        assert_eq!(
            line(
                "Waiting for setup lock",
                None,
                Duration::from_secs(5),
                false
            ),
            "Setup: Waiting for setup lock - 5s elapsed."
        );
    }
}
