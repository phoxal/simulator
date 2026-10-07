//! Owned, bounded disk-image tool lifetime and mount cleanup.
use super::*;

#[derive(Debug)]
#[cfg(any(target_os = "macos", test))]
pub(super) struct ImageToolFailure {
    cause: String,
    cleanup_complete: bool,
}
#[cfg(any(target_os = "macos", test))]
impl From<String> for ImageToolFailure {
    fn from(cause: String) -> Self {
        Self {
            cause,
            cleanup_complete: true,
        }
    }
}
#[cfg(any(target_os = "macos", test))]
impl From<&str> for ImageToolFailure {
    fn from(cause: &str) -> Self {
        cause.to_owned().into()
    }
}
#[cfg(any(target_os = "macos", test))]
impl std::fmt::Display for ImageToolFailure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.cause.fmt(f)
    }
}

pub(super) struct ImageExtraction {
    pub(super) result: Result<(), String>,
    pub(super) cleanup_complete: bool,
}

#[cfg(any(target_os = "macos", test))]
pub(super) enum ImageOperation<'a> {
    Attach { archive: &'a Path, mount: &'a Path },
    Detach { mount: &'a Path },
}

#[cfg(any(target_os = "macos", test))]
pub(super) fn stage_image(
    archive: &Path,
    output: &Path,
    temporary: &Path,
    cancel: &Cancellation,
    mut tool: impl FnMut(ImageOperation<'_>, &Cancellation) -> Result<(), ImageToolFailure>,
) -> ImageExtraction {
    let mount = temporary.join("mount");
    if let Err(error) = fs::create_dir(&mount) {
        return ImageExtraction {
            result: Err(error.to_string()),
            cleanup_complete: true,
        };
    }
    let primary: Result<(), ImageToolFailure> =
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            tool(
                ImageOperation::Attach {
                    archive,
                    mount: &mount,
                },
                cancel,
            )?;
            cancel.check()?;
            let mut budget = CopyBudget::default();
            for name in ["mujoco.framework", "LICENSE", "THIRD_PARTY_NOTICES"] {
                copy_tree(
                    &mount,
                    &mount.join(name),
                    &output.join(name),
                    &mut budget,
                    cancel,
                )?;
            }
            Ok(())
        }))
        .unwrap_or_else(|_| Err("MuJoCo image staging panicked".into()));
    // Attach may have mounted before cancellation or a nonzero exit. Cleanup
    // owns this exact private mount even when the primary operation failed.
    let cleanup = tool(
        ImageOperation::Detach { mount: &mount },
        &Cancellation::default(),
    );
    let cleanup_complete = cleanup.is_ok()
        && primary
            .as_ref()
            .err()
            .is_none_or(|error| error.cleanup_complete);
    let result = match (primary, cleanup) {
        (Ok(()), Ok(())) => Ok(()),
        (Err(error), Ok(())) => Err(error.to_string()),
        (Ok(()), Err(error)) => Err(format!("MuJoCo mount cleanup failed: {error}")),
        (Err(primary), Err(cleanup)) => {
            Err(format!("{primary}; MuJoCo mount cleanup failed: {cleanup}"))
        }
    };
    ImageExtraction {
        result,
        cleanup_complete,
    }
}

#[cfg(target_os = "macos")]
pub(super) fn extract_dmg(
    archive: &Path,
    output: &Path,
    temporary: &Path,
    cancel: &Cancellation,
) -> ImageExtraction {
    stage_image(archive, output, temporary, cancel, image_command)
}

#[cfg(target_os = "macos")]
pub(super) fn image_command(
    operation: ImageOperation<'_>,
    cancel: &Cancellation,
) -> Result<(), ImageToolFailure> {
    let mut command = std::process::Command::new("hdiutil");
    match operation {
        ImageOperation::Attach { archive, mount } => {
            command
                .args(["attach", "-readonly", "-nobrowse", "-quiet", "-mountpoint"])
                .arg(mount)
                .arg(archive);
        }
        ImageOperation::Detach { mount } => {
            use std::os::unix::fs::MetadataExt as _;
            let parent = mount.parent().ok_or("Owned mount has no parent")?;
            if fs::metadata(mount).map_err(|e| e.to_string())?.dev()
                == fs::metadata(parent).map_err(|e| e.to_string())?.dev()
            {
                return Ok(());
            }
            command.args(["detach", "-quiet"]).arg(mount);
        }
    }
    run_image_process(&mut command, cancel, Duration::from_secs(45))
}

#[cfg(any(target_os = "macos", test))]
pub(super) fn run_image_process(
    command: &mut std::process::Command,
    cancel: &Cancellation,
    timeout: Duration,
) -> Result<(), ImageToolFailure> {
    use std::process::Stdio;
    cancel.check()?;
    let mut child = command
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| ImageToolFailure::from(e.to_string()))?;
    let stderr = match child.stderr.take() {
        Some(stderr) => stderr,
        None => {
            return Err(reap_image_tool(
                &mut child,
                "missing image-tool stderr".into(),
                None,
            ));
        }
    };
    let mut tail = match crate::process_output::Tail::drain(stderr) {
        Ok(tail) => tail,
        Err(error) => {
            return Err(reap_image_tool(&mut child, error, None));
        }
    };
    let deadline = Instant::now() + timeout;
    loop {
        let status = match child.try_wait() {
            Ok(status) => status,
            Err(error) => {
                return Err(reap_image_tool(
                    &mut child,
                    format!("Image tool status failed: {error}"),
                    Some(&mut tail),
                ));
            }
        };
        if let Some(status) = status {
            let diagnostic_cleanup = tail.finish();
            return if status.success() {
                diagnostic_cleanup.map_err(Into::into)
            } else {
                Err(format!(
                    "Image tool failed: {status}. {}{}",
                    tail.text(),
                    diagnostic_cleanup
                        .err()
                        .map(|error| format!("; diagnostic cleanup failed: {error}"))
                        .unwrap_or_default()
                )
                .into())
            };
        }
        if cancel.is_cancelled() || Instant::now() >= deadline {
            let cause = if cancel.is_cancelled() {
                "Operation cancelled"
            } else {
                "Image tool timed out"
            };
            return Err(reap_image_tool(&mut child, cause.into(), Some(&mut tail)));
        }
        std::thread::sleep(Duration::from_millis(25));
    }
}

#[cfg(any(target_os = "macos", test))]
fn reap_image_tool(
    child: &mut std::process::Child,
    mut cause: String,
    tail: Option<&mut crate::process_output::Tail>,
) -> ImageToolFailure {
    if let Err(error) = child.kill() {
        cause.push_str(&format!("; image-tool termination failed: {error}"));
    }
    let deadline = Instant::now() + Duration::from_secs(5);
    let cleanup_complete = loop {
        match child.try_wait() {
            Ok(Some(_)) => break true,
            Ok(None) if Instant::now() < deadline => std::thread::sleep(Duration::from_millis(25)),
            Ok(None) => {
                cause.push_str("; image-tool reap timed out");
                break false;
            }
            Err(error) => {
                cause.push_str(&format!("; image-tool reap failed: {error}"));
                break false;
            }
        }
    };
    if cleanup_complete {
        cause.push_str("; image tool exited and was reaped");
    }
    if let Some(tail) = tail {
        if let Err(error) = tail.finish() {
            cause.push_str(&format!("; diagnostic cleanup failed: {error}"));
        }
        let text = tail.text();
        if !text.is_empty() {
            cause.push_str(&format!("\n{text}"));
        }
    }
    ImageToolFailure {
        cause,
        cleanup_complete,
    }
}

#[cfg(not(target_os = "macos"))]
pub(super) fn extract_dmg(_: &Path, _: &Path, _: &Path, _: &Cancellation) -> ImageExtraction {
    ImageExtraction {
        result: Err("DMG distributions are only supported on macOS".into()),
        cleanup_complete: true,
    }
}
