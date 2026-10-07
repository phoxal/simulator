//! Explicit readiness hold fixture. It implements no supervisor/control protocol.
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

static STOP: AtomicBool = AtomicBool::new(false);
extern "C" fn stop(_: i32) {
    STOP.store(true, Ordering::Release);
}
unsafe extern "C" {
    fn signal(number: i32, handler: extern "C" fn(i32)) -> usize;
}

fn main() {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let state = args
        .windows(2)
        .find(|pair| pair[0] == "--state-dir")
        .expect("explicit state directory");
    assert_ne!(unsafe { signal(15, stop) }, usize::MAX);
    std::fs::write(
        std::path::Path::new(&state[1]).join("fixture-started"),
        std::process::id().to_string(),
    )
    .unwrap();
    // The real launcher supplies the private build root first. This fixture-owned
    // receipt lets qualification observe handler readiness without sleeping.
    if !args[0].to_string_lossy().starts_with("--") {
        std::fs::write(
            std::path::Path::new(&args[0])
                .parent()
                .unwrap()
                .join("held-fixture.started"),
            std::process::id().to_string(),
        )
        .unwrap();
    }
    eprintln!(
        "EXPLICIT TEST FIXTURE: supervisor readiness deliberately held; no control implementation"
    );
    let deadline = Instant::now() + Duration::from_secs(60);
    while !STOP.load(Ordering::Acquire) && Instant::now() < deadline {
        std::thread::park_timeout(Duration::from_millis(25));
    }
    if !STOP.load(Ordering::Acquire) {
        std::process::exit(8);
    }
}
