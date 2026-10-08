//! One stderr line, only while this setup owner has exclusive terminal use.
use std::fs::File;
use std::io::{IsTerminal, Write};
use std::os::fd::{AsRawFd, FromRawFd};

pub(super) struct Terminal {
    columns: Option<u16>,
    drawn: bool,
    frame: usize,
    output: Option<File>,
    interactive: bool,
}
impl Terminal {
    pub(super) fn new() -> Self {
        // Own the output descriptor without leaking it into setup subprocesses.
        let fd = unsafe { libc::fcntl(libc::STDERR_FILENO, libc::F_DUPFD_CLOEXEC, 0) };
        let output = (fd >= 0).then(|| unsafe { File::from_raw_fd(fd) });
        let interactive = std::env::var_os("CI").is_none()
            && std::env::var("TERM").ok().is_some_and(|term| {
                matches!(
                    term.split('-').next(),
                    Some(
                        "xterm"
                            | "screen"
                            | "tmux"
                            | "rxvt"
                            | "linux"
                            | "ansi"
                            | "alacritty"
                            | "foot"
                            | "wezterm"
                            | "kitty"
                    )
                )
            });
        Self::with_output(output, interactive)
    }
    pub(super) fn with_output(output: Option<File>, interactive: bool) -> Self {
        Self {
            columns: columns(output.as_ref(), interactive),
            drawn: false,
            frame: 0,
            output,
            interactive,
        }
    }
    pub(super) fn live(&self) -> bool {
        self.columns.is_some()
    }
    pub(super) fn frame(&mut self) -> char {
        let frame = ['|', '/', '-', '\\'][self.frame % 4];
        self.frame = self.frame.wrapping_add(1);
        frame
    }
    pub(super) fn update(&mut self, text: &str) -> bool {
        let Some(width) = self.columns else {
            return false;
        };
        if columns(self.output.as_ref(), self.interactive) != Some(width) {
            // Resize can reflow the old line. Append rather than moving over
            // wrapped content, then permanently use ordinary lines.
            if self.drawn {
                self.write_best_effort(b"\n");
            }
            self.columns = None;
            self.drawn = false;
            return false;
        }
        let text: String = text
            .chars()
            .filter(|c| !c.is_control())
            .take(usize::from(width - 1))
            .collect();
        if !self.write_best_effort(format!("\r\x1b[2K{text}").as_bytes()) {
            self.columns = None;
            self.drawn = false;
            return false;
        }
        self.drawn = true;
        true
    }
    pub(super) fn line(&mut self, text: &str) {
        self.clear();
        self.write_best_effort(format!("{text}\n").as_bytes());
    }
    pub(super) fn clear(&mut self) {
        if self.drawn {
            if columns(self.output.as_ref(), self.interactive) == self.columns {
                self.write_best_effort(b"\r\x1b[2K");
            } else {
                self.write_best_effort(b"\n");
                self.columns = None;
            }
            self.drawn = false;
        }
    }
    fn write_best_effort(&mut self, bytes: &[u8]) -> bool {
        let Some(output) = self.output.as_mut() else {
            return false;
        };
        output
            .write_all(bytes)
            .and_then(|()| output.flush())
            .is_ok()
    }
}
fn columns(output: Option<&File>, interactive: bool) -> Option<u16> {
    let output = output?;
    if !output.is_terminal() || !interactive {
        return None;
    }
    let mut size: libc::winsize = unsafe { std::mem::zeroed() };
    // TIOCGWINSZ writes only this valid winsize allocation.
    if unsafe { libc::ioctl(output.as_raw_fd(), libc::TIOCGWINSZ, &mut size) } != 0
        || size.ws_col < 72
    {
        return None;
    }
    Some(size.ws_col)
}
