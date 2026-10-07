//! Explicit early-exit diagnostic fixture, not supervisor behavior evidence.
use std::io::Write;
fn main() {
    let mut stderr = std::io::stderr().lock();
    stderr.write_all(&vec![b'x'; 256 * 1024]).unwrap();
    writeln!(
        stderr,
        "TEST_SUPERVISOR_FAILURE: explicit early exit after bounded diagnostic flood"
    )
    .unwrap();
    std::process::exit(7);
}
