//! Explicit, checksum-pinned native setup owned by the simulator.
use crate::cancellation::Cancellation;
use fs2::FileExt;
use sha2::{Digest, Sha256};
use std::{
    fs::{self, File, OpenOptions},
    io::{Read, Write},
    path::{Component, Path, PathBuf},
    sync::OnceLock,
    time::{Duration, Instant},
};

mod archive;
mod image;
use archive::extract_tar;
#[cfg(any(target_os = "macos", test))]
use archive::{CopyBudget, copy_tree};
#[cfg(test)]
use archive::{copy_bounded, safe_link, safe_relative};
use image::extract_dmg;
#[cfg(test)]
use image::{ImageOperation, stage_image};

const VERSION: &str = "3.12.0";
const MAX_DOWNLOAD: u64 = 64 * 1024 * 1024;
const MAX_EXPANDED: u64 = 1024 * 1024 * 1024;
const MAX_FILES: usize = 50_000;
static ROOT: OnceLock<PathBuf> = OnceLock::new();

pub(crate) fn configure_root(root: Option<PathBuf>) -> Result<(), String> {
    if let Some(root) = root {
        if root.as_os_str().is_empty() {
            return Err("runtime root must not be empty".into());
        }
        let root = if root.is_absolute() {
            root
        } else {
            std::env::current_dir()
                .map_err(|e| e.to_string())?
                .join(root)
        };
        ROOT.set(root)
            .map_err(|_| "runtime root was already configured")?;
    }
    Ok(())
}

fn root() -> Result<PathBuf, String> {
    ROOT.get().cloned().map(Ok).unwrap_or_else(|| {
        std::env::var_os("HOME")
            .map(|home| PathBuf::from(home).join(".phoxal/simulator"))
            .ok_or_else(|| "Home directory unavailable; select --runtime-root explicitly".into())
    })
}

pub(crate) fn command() -> Result<String, String> {
    command_for(ROOT.get().map(PathBuf::as_path))
}

fn command_for(selected: Option<&Path>) -> Result<String, String> {
    match selected {
        None if std::env::var_os("HOME").is_none() => Err("Home directory unavailable; choose a writable directory and run phoxal-simulator --runtime-root '<directory>' setup, then retry with the same --runtime-root".into()),
        None => Ok("phoxal-simulator setup".into()),
        Some(root) => {
            let text = root.to_str().ok_or("The selected runtime root is not UTF-8; pass that exact directory to phoxal-simulator --runtime-root manually")?;
            // POSIX single quotes preserve whitespace, newlines and shell metacharacters.
            let quoted = text.replace('\'', "'\\''");
            Ok(format!("phoxal-simulator --runtime-root '{quoted}' setup"))
        }
    }
}

pub(crate) fn recovery_instruction() -> String {
    if let Some(path) = std::env::var_os("PHOXAL_MUJOCO_LIBRARY") {
        format!(
            "repair or remove PHOXAL_MUJOCO_LIBRARY={} and retry. This strict override takes precedence over managed setup.",
            Path::new(&path).display()
        )
    } else {
        match command() {
            Ok(command) => format!("run {command}, then retry. Startup never downloads a runtime."),
            Err(error) => error,
        }
    }
}

struct Distribution {
    target: &'static str,
    archive: &'static str,
    checksum: &'static str,
    library: &'static str,
    dmg: bool,
}

fn distribution() -> Result<Distribution, String> {
    distribution_for(phoxal::artifact::application::HOST_EXECUTION_TARGET)
}

fn distribution_for(target: &str) -> Result<Distribution, String> {
    let (target, archive, checksum, library, dmg) = match target {
        "aarch64-apple-darwin" => (
            "aarch64-apple-darwin",
            "mujoco-3.12.0-macos-universal2.dmg",
            "8410882d724c3637b935dc0482b0de90efed44b17bbcfcb4a165ee46285a4865",
            "mujoco.framework/Versions/A/libmujoco.3.12.0.dylib",
            true,
        ),
        "x86_64-apple-darwin" => (
            "x86_64-apple-darwin",
            "mujoco-3.12.0-macos-universal2.dmg",
            "8410882d724c3637b935dc0482b0de90efed44b17bbcfcb4a165ee46285a4865",
            "mujoco.framework/Versions/A/libmujoco.3.12.0.dylib",
            true,
        ),
        "aarch64-unknown-linux-gnu" => (
            "aarch64-unknown-linux-gnu",
            "mujoco-3.12.0-linux-aarch64.tar.gz",
            "08fd5627a2ef7d5a42580c40e014ab2c1a644f082010c584ca361a3ed8cad838",
            "lib/libmujoco.so.3.12.0",
            false,
        ),
        "x86_64-unknown-linux-gnu" => (
            "x86_64-unknown-linux-gnu",
            "mujoco-3.12.0-linux-x86_64.tar.gz",
            "a9367911e6d5eaeade17c2197304687421c1fc932cdf7bcd4cb8cfaf0374dcb2",
            "lib/libmujoco.so.3.12.0",
            false,
        ),
        _ => {
            return Err(format!(
                "No supported prebuilt MuJoCo distribution for target {target}"
            ));
        }
    };
    Ok(Distribution {
        target,
        archive,
        checksum,
        library,
        dmg,
    })
}

pub(crate) fn managed_library() -> Result<Option<PathBuf>, String> {
    managed_library_for(phoxal::artifact::application::HOST_EXECUTION_TARGET)
}

fn managed_library_for(target: &str) -> Result<Option<PathBuf>, String> {
    let Ok(spec) = distribution_for(target) else {
        return Ok(None);
    };
    Ok(Some(
        root()?
            .join("mujoco")
            .join(VERSION)
            .join(spec.target)
            .join(spec.library),
    ))
}

pub(crate) fn validate_managed_if_selected(
    path: &Path,
    cancel: &Cancellation,
) -> Result<(), String> {
    validate_managed_for(
        path,
        cancel,
        phoxal::artifact::application::HOST_EXECUTION_TARGET,
    )
}

fn validate_managed_for(path: &Path, cancel: &Cancellation, target: &str) -> Result<(), String> {
    let Ok(spec) = distribution_for(target) else {
        // Unsupported managed distributions do not restrict external ABI admission.
        return Ok(());
    };
    let Ok(root) = root() else {
        return Ok(());
    };
    let directory = root.join("mujoco").join(VERSION).join(spec.target);
    if path == directory.join(spec.library) && directory.exists() {
        validate_install(&directory, &spec, cancel)?;
    }
    Ok(())
}

pub(crate) fn run(cancel: &Cancellation) -> Result<PathBuf, String> {
    let root = root()?;
    let spec = distribution()?;
    install(&root, &spec, cancel, |destination, cancel| {
        download(&spec, destination, cancel)
    })
}

pub(crate) fn run_cli() -> Result<PathBuf, String> {
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .map_err(|e| e.to_string())?;
    runtime.block_on(async {
        let cancel = Cancellation::default();
        let owner_cancel = cancel.clone();
        let mut owner = tokio::task::spawn_blocking(move || run(&owner_cancel));
        tokio::select! {
            result = &mut owner => result.map_err(|e| e.to_string())?,
            signal = tokio::signal::ctrl_c() => {
                signal.map_err(|e| e.to_string())?;
                cancel.cancel();
                owner.await.map_err(|e| e.to_string())?
            }
        }
    })
}

fn install(
    root: &Path,
    spec: &Distribution,
    cancel: &Cancellation,
    obtain: impl FnOnce(&Path, &Cancellation) -> Result<(), String>,
) -> Result<PathBuf, String> {
    cancel.check()?;
    let parent = root.join("mujoco").join(VERSION);
    fs::create_dir_all(&parent)
        .map_err(|e| setup_write_error("creating runtime directory", &parent, e))?;
    let lock = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .open(parent.join(format!("{}.lock", spec.target)))
        .map_err(|e| {
            setup_write_error(
                "opening setup lock",
                &parent.join(format!("{}.lock", spec.target)),
                e,
            )
        })?;
    let deadline = Instant::now() + Duration::from_secs(180);
    loop {
        cancel.check()?;
        match lock.try_lock_exclusive() {
            Ok(()) => break,
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {
                if Instant::now() >= deadline {
                    return Err("Another setup is still running; retry when it completes".into());
                }
                std::thread::sleep(Duration::from_millis(25));
            }
            Err(error) => return Err(setup_write_error("locking runtime setup", &parent, error)),
        }
    }
    let destination = parent.join(spec.target);
    let library = destination.join(spec.library);
    // An installed directory is immutable. Never replace a possibly live API.
    if destination.exists() {
        validate_install(&destination, spec, cancel)?;
        crate::native_binding::probe_library_with_cancel(&library, cancel)?;
        return Ok(library);
    }
    let temporary = tempfile::Builder::new()
        .prefix(".setup-")
        .tempdir_in(&parent)
        .map_err(|e| setup_write_error("creating setup transaction", &parent, e))?;
    let archive = temporary.path().join(spec.archive);
    obtain(&archive, cancel)?;
    verify_digest(&archive, spec.checksum, cancel)?;
    let staged = temporary.path().join("runtime");
    fs::create_dir(&staged)
        .map_err(|e| setup_write_error("creating staged runtime", &staged, e))?;
    if spec.dmg {
        let extraction = extract_dmg(&archive, &staged, temporary.path(), cancel);
        if !extraction.cleanup_complete {
            let retained = temporary.keep();
            return Err(format!(
                "{}; owned mount cleanup unresolved, temporary directory preserved at {}",
                extraction.result.unwrap_err(),
                retained.display()
            ));
        }
        extraction.result?;
    } else {
        extract_tar(&archive, &staged, temporary.path(), cancel)?;
    }
    cancel.check()?;
    crate::native_binding::probe_library_with_cancel(&staged.join(spec.library), cancel)
        .map_err(|e| format!("Downloaded runtime failed complete API admission: {e}"))?;
    let provenance = serde_json::json!({
        "version": VERSION, "target": spec.target, "archive": spec.archive,
        "url": format!("https://github.com/google-deepmind/mujoco/releases/download/{VERSION}/{}", spec.archive),
        "sha256": spec.checksum, "library": spec.library,
        "library_sha256": file_digest(&staged.join(spec.library), cancel)?,
    });
    fs::write(
        staged.join("provenance.json"),
        serde_json::to_vec_pretty(&provenance).map_err(|e| e.to_string())?,
    )
    .map_err(|e| {
        setup_write_error(
            "writing runtime provenance",
            &staged.join("provenance.json"),
            e,
        )
    })?;
    validate_install(&staged, spec, cancel)?;
    cancel.check()?;
    fs::rename(&staged, &destination)
        .map_err(|e| setup_write_error("atomically publishing runtime", &destination, e))?;
    Ok(library)
}

fn setup_write_error(phase: &str, path: &Path, cause: std::io::Error) -> String {
    format!(
        "Setup failed while {phase}: {cause}\nSelected path: {}\nNext: choose a writable directory with --runtime-root DIRECTORY, or repair this directory's permissions and available space, then run setup again.",
        path.display()
    )
}

fn validate_install(
    directory: &Path,
    spec: &Distribution,
    cancel: &Cancellation,
) -> Result<(), String> {
    validate_install_contents(directory, spec, cancel).map_err(|error| {
        if error == "Operation cancelled" { error } else {
            format!("Installed runtime verification failed: {error}\nSelected installation: {}\nNext: preserve and repair this installation explicitly, or choose another writable --runtime-root DIRECTORY. Setup never replaces an existing installation.", directory.display())
        }
    })
}

fn validate_install_contents(
    directory: &Path,
    spec: &Distribution,
    cancel: &Cancellation,
) -> Result<(), String> {
    cancel.check()?;
    if fs::symlink_metadata(directory)
        .map_err(|e| e.to_string())?
        .file_type()
        .is_symlink()
    {
        return Err("Managed runtime directory must not be a symlink".into());
    }
    for name in [spec.library, "LICENSE", "THIRD_PARTY_NOTICES"] {
        let path = directory.join(name);
        if !path.is_file() {
            return Err(format!(
                "Runtime is incomplete: {}. Preserve it and choose another --runtime-root or repair this directory explicitly.",
                path.display()
            ));
        }
        let resolved = path.canonicalize().map_err(|e| e.to_string())?;
        if !resolved.starts_with(directory.canonicalize().map_err(|e| e.to_string())?) {
            return Err(format!(
                "Runtime resource escapes installation: {}",
                path.display()
            ));
        }
    }
    let bytes = fs::read(directory.join("provenance.json"))
        .map_err(|e| format!("Runtime provenance unavailable: {e}"))?;
    let value: serde_json::Value = serde_json::from_slice(&bytes).map_err(|e| e.to_string())?;
    if value["sha256"] != spec.checksum
        || value["target"] != spec.target
        || value["version"] != VERSION
        || value["library"] != spec.library
    {
        return Err("Runtime provenance does not match the supported distribution".into());
    }
    if value["library_sha256"] != file_digest(&directory.join(spec.library), cancel)? {
        return Err("Managed runtime library differs from its installation receipt; repair it explicitly or choose another --runtime-root".into());
    }
    Ok(())
}

fn file_digest(path: &Path, cancel: &Cancellation) -> Result<String, String> {
    let mut source = File::open(path).map_err(|e| e.to_string())?;
    let mut digest = Sha256::new();
    let mut total = 0u64;
    let mut bytes = [0; 64 * 1024];
    loop {
        cancel.check()?;
        let count = source.read(&mut bytes).map_err(|e| e.to_string())?;
        if count == 0 {
            return Ok(format!("{:x}", digest.finalize()));
        }
        total += count as u64;
        if total > MAX_EXPANDED {
            return Err("Managed runtime file exceeds size limit".into());
        }
        digest.update(&bytes[..count]);
    }
}

fn download(spec: &Distribution, destination: &Path, cancel: &Cancellation) -> Result<(), String> {
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .map_err(|e| e.to_string())?;
    runtime.block_on(async {
        tokio::select! {
            biased;
            _ = cancel.wait() => Err("Operation cancelled".into()),
            result = transfer(spec, destination) => result,
        }
    })
}

async fn transfer(spec: &Distribution, destination: &Path) -> Result<(), String> {
    let client = reqwest::Client::builder()
        .connect_timeout(Duration::from_secs(10))
        .timeout(Duration::from_secs(120))
        .redirect(reqwest::redirect::Policy::limited(5))
        .build()
        .map_err(|e| e.to_string())?;
    let mut response = client
        .get(format!(
            "https://github.com/google-deepmind/mujoco/releases/download/{VERSION}/{}",
            spec.archive
        ))
        .send()
        .await
        .and_then(reqwest::Response::error_for_status)
        .map_err(|e| {
            format!("Cannot download MuJoCo: {e}. Retry setup when network access is available.")
        })?;
    if response
        .content_length()
        .is_some_and(|bytes| bytes > MAX_DOWNLOAD)
    {
        return Err("Runtime download exceeds size limit".into());
    }
    let mut file = File::create(destination)
        .map_err(|e| setup_write_error("creating downloaded archive", destination, e))?;
    let mut total = 0u64;
    while let Some(bytes) = response.chunk().await.map_err(|error| format!("MuJoCo archive transfer failed: {error}; cause: {}\nArchive: {}\nNext: check network/proxy access and rerun {}. The incomplete transaction will not be published.", std::error::Error::source(&error).map(ToString::to_string).unwrap_or_else(|| "no additional loader cause".into()), spec.archive, command().unwrap_or_else(|_| "phoxal-simulator setup with the selected runtime root".into())))? {
        total += bytes.len() as u64;
        if total > MAX_DOWNLOAD {
            return Err("Runtime download exceeds size limit".into());
        }
        file.write_all(&bytes).map_err(|e| setup_write_error("writing downloaded archive", destination, e))?;
    }
    file.sync_all()
        .map_err(|e| setup_write_error("syncing downloaded archive", destination, e))
}

fn verify_digest(path: &Path, expected: &str, cancel: &Cancellation) -> Result<(), String> {
    let mut input = File::open(path).map_err(|e| e.to_string())?;
    let mut digest = Sha256::new();
    let mut bytes = [0; 64 * 1024];
    let mut total = 0;
    loop {
        cancel.check()?;
        let count = input.read(&mut bytes).map_err(|e| e.to_string())?;
        if count == 0 {
            break;
        }
        total += count as u64;
        if total > MAX_DOWNLOAD {
            return Err("Runtime archive exceeds size limit".into());
        }
        digest.update(&bytes[..count]);
    }
    if format!("{:x}", digest.finalize()) != expected {
        return Err(
            "Runtime checksum mismatch; nothing was extracted or installed. Retry setup.".into(),
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests;
