//! Test-owned external image-tool process, not a native engine implementation.
use std::io::Write;

fn main() {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    match args[0].to_str().unwrap() {
        "fail" => {
            let mut stderr = std::io::stderr().lock();
            stderr.write_all(&vec![b'x'; 256 * 1024]).unwrap();
            writeln!(stderr, "FINAL_IMAGE_TOOL_CAUSE").unwrap();
            std::process::exit(7);
        }
        "hold" => {
            std::fs::write(&args[1], std::process::id().to_string()).unwrap();
            loop {
                std::thread::park();
            }
        }
        _ => panic!("unknown fixture operation"),
    }
}
