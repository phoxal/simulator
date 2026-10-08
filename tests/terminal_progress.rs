//! Real PTY presentation and cancellation at explicit owned boundaries.
#[path = "support/owned_child.rs"]
#[allow(dead_code)]
mod owned_child;
#[path = "support/pty.rs"]
#[allow(dead_code)]
mod pty;
use owned_child::OwnedChild;
use pty::Pty;
use std::{
    fs,
    process::Command,
    time::{Duration, Instant},
};

fn await_exit(owner: &mut OwnedChild) -> std::process::ExitStatus {
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        if let Some(status) = owner.child().try_wait().unwrap() {
            return status;
        }
        assert!(Instant::now() < deadline, "terminal owner did not finish");
        std::thread::park_timeout(Duration::from_millis(10));
    }
}
#[test]
fn actual_setup_pty_lock_wait_cancel_and_plain_terminal_fallbacks() {
    use fs2::FileExt;
    for (columns, term, ci, closed) in [
        (100, "xterm-256color", false, false),
        (60, "xterm-256color", false, false),
        (100, "dumb", false, false),
        (100, "unknown-terminal", false, false),
        (100, "vt52", false, false),
        (100, "xterm-256color", true, false),
        (100, "xterm-256color", false, true),
    ] {
        let directory = tempfile::tempdir().unwrap();
        let root = directory.path().join("runtime");
        let parent = root.join("mujoco/3.12.0");
        fs::create_dir_all(&parent).unwrap();
        let lock = fs::OpenOptions::new()
            .create(true)
            .truncate(false)
            .write(true)
            .open(parent.join(format!(
                "{}.lock",
                phoxal::artifact::application::HOST_EXECUTION_TARGET
            )))
            .unwrap();
        lock.lock_exclusive().unwrap();
        let mut command = Command::new(env!("CARGO_BIN_EXE_phoxal-simulator"));
        command.arg("--runtime-root").arg(&root).arg("setup");
        let mut terminal = Pty::attach(&mut command, columns);
        command.env("TERM", term);
        if ci {
            command.env("CI", "true");
        }
        let mut owner = OwnedChild::new(command.spawn().unwrap(), Duration::from_secs(2));
        let started = Instant::now();
        // Bound process admission separately from the short cancellation cut.
        // First launch of a newly linked native GUI binary can be slower on macOS.
        terminal.until("Waiting for setup lock", 15);
        println!(
            "Actual setup initial lock-wait output after {:?}",
            started.elapsed()
        );
        if columns < 72 || term != "xterm-256color" || ci {
            assert!(!terminal.bytes.contains(&27));
        } else {
            terminal.until("\u{1b}[2KSetup:", 3);
        }
        if closed {
            drop(terminal);
            assert_eq!(
                unsafe { libc::kill(owner.child().id() as i32, libc::SIGINT) },
                0
            );
            assert_eq!(await_exit(&mut owner).code(), Some(1));
        } else {
            assert_eq!(
                unsafe { libc::kill(owner.child().id() as i32, libc::SIGINT) },
                0
            );
            assert_eq!(await_exit(&mut owner).code(), Some(1));
            terminal.drain();
            let text = String::from_utf8_lossy(&terminal.bytes);
            assert!(text.contains("Cancellation requested. Waiting for cleanup"));
            assert!(text.contains("Operation cancelled"));
            assert!(
                !text
                    .split("Operation cancelled")
                    .nth(1)
                    .unwrap()
                    .contains('\u{1b}')
            );
            println!("Actual setup PTY columns={columns}, TERM={term}, CI={ci}: {text:?}");
        }
        assert_eq!(
            fs::read_dir(parent).unwrap().count(),
            1,
            "cancel must not publish an installation"
        );
    }
}
