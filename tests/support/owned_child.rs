//! Local subprocess ownership for this CLI test target, including assertion unwind.
use std::process::{Child, ExitStatus};
use std::time::{Duration, Instant};

pub(super) struct OwnedChild {
    child: Option<Child>,
    grace: Duration,
}

impl OwnedChild {
    pub(super) fn new(child: Child, grace: Duration) -> Self {
        Self {
            child: Some(child),
            grace,
        }
    }
    pub(super) fn child(&mut self) -> &mut Child {
        self.child.as_mut().expect("owned child")
    }
    pub(super) fn stop(&mut self) -> Result<ExitStatus, String> {
        let child = self.child.as_mut().ok_or("child already reaped")?;
        if let Some(status) = child.try_wait().map_err(|e| e.to_string())? {
            self.child = None;
            return Ok(status);
        }
        let term = unsafe { libc::kill(child.id() as i32, libc::SIGTERM) };
        let signal_error = (term != 0).then(std::io::Error::last_os_error);
        let deadline = Instant::now() + self.grace;
        loop {
            if let Some(status) = child.try_wait().map_err(|e| e.to_string())? {
                self.child = None;
                return Ok(status);
            }
            if Instant::now() >= deadline {
                break;
            }
            std::thread::park_timeout(Duration::from_millis(10));
        }
        child
            .kill()
            .map_err(|e| format!("owned kill failed after TERM {signal_error:?}: {e}"))?;
        let deadline = Instant::now() + Duration::from_secs(2);
        loop {
            if let Some(status) = child.try_wait().map_err(|e| e.to_string())? {
                self.child = None;
                return Ok(status);
            }
            if Instant::now() >= deadline {
                return Err("owned child was not reaped within the kill deadline".into());
            }
            std::thread::park_timeout(Duration::from_millis(10));
        }
    }
}

impl Drop for OwnedChild {
    fn drop(&mut self) {
        if self.child.is_some()
            && let Err(error) = self.stop()
        {
            eprintln!("TEST PROCESS CLEANUP FAILED: {error}");
        }
    }
}

pub(super) fn no_children(command: &mut std::process::Command) -> Result<(), String> {
    let output = command
        .output()
        .map_err(|e| format!("child inventory command failed: {e}"))?;
    match output.status.code() {
        Some(1) if output.stdout.is_empty() => Ok(()),
        Some(0) => Err(format!(
            "child inventory found children: {}",
            String::from_utf8_lossy(&output.stdout)
        )),
        _ => Err(format!(
            "child inventory failed {}: {}",
            output.status,
            String::from_utf8_lossy(&output.stderr)
        )),
    }
}
