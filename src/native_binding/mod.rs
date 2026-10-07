//! Internal adaptation of mujoco-rs 6.0.1 (MuJoCo 3.12.0), MIT/Apache-2.0.
//! The original ABI file is unchanged; build.rs generates typed runtime dispatch.
//! This module never links MuJoCo, downloads it, or installs callbacks before validation.
#![allow(dead_code, unused_imports, non_snake_case, clippy::all)]

use crate::notice::Notice;
use std::path::{Path, PathBuf};
use std::sync::{Mutex, OnceLock};

pub mod error;
pub mod prelude;
pub mod util;
pub mod wrappers;
#[allow(warnings, clippy::approx_constant)]
pub mod mujoco_c {
    include!(concat!(env!("OUT_DIR"), "/mujoco_dispatch.rs"));
}

struct Admission {
    api: OnceLock<mujoco_c::Api>,
    loading: Mutex<()>,
}

impl Admission {
    const fn new() -> Self {
        Self {
            api: OnceLock::new(),
            loading: Mutex::new(()),
        }
    }

    fn initialize(&self, candidates: &[PathBuf]) -> Result<(), String> {
        self.initialize_cancellable(candidates, &crate::cancellation::Cancellation::default())
            .map_err(|error| error.to_string())
    }

    fn initialize_cancellable(
        &self,
        candidates: &[PathBuf],
        cancel: &crate::cancellation::Cancellation,
    ) -> Result<(), Notice> {
        if self.api.get().is_some() {
            return Ok(());
        }
        let _loading = loop {
            cancel
                .check()
                .map_err(|_| Notice::new("Operation cancelled", "", ""))?;
            match self.loading.try_lock() {
                Ok(lock) => break lock,
                Err(std::sync::TryLockError::WouldBlock) => {
                    std::thread::sleep(std::time::Duration::from_millis(25))
                }
                Err(_) => return Err("MuJoCo admission lock poisoned".into()),
            }
        };
        if self.api.get().is_some() {
            return Ok(());
        }
        let admitted = load_candidates(candidates, cancel)?;
        self.api
            .set(admitted)
            .map_err(|_| "MuJoCo was already admitted")?;
        Ok(())
    }
}

static API: Admission = Admission::new();

/// Admission happens before any model/data layout access or process launch.
pub fn initialize() -> Result<(), String> {
    initialize_with_cancel(&crate::cancellation::Cancellation::default())
        .map_err(|error| error.to_string())
}

pub(crate) fn initialize_with_cancel(
    cancel: &crate::cancellation::Cancellation,
) -> Result<(), Notice> {
    if API.api.get().is_some() {
        return Ok(());
    }
    API.initialize_cancellable(&candidates()?, cancel)
}

fn candidates() -> Result<Vec<PathBuf>, String> {
    let explicit = std::env::var_os("PHOXAL_MUJOCO_LIBRARY");
    let candidates: Vec<PathBuf> = if let Some(path) = explicit {
        return Ok(vec![path.into()]);
    } else if cfg!(target_os = "macos") {
        [
            "libmujoco.3.12.0.dylib",
            "libmujoco.dylib",
            "/opt/homebrew/lib/libmujoco.dylib",
            "/usr/local/lib/libmujoco.dylib",
            "/Library/Frameworks/mujoco.framework/Versions/A/libmujoco.dylib",
            "/Library/Frameworks/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib",
            "/Applications/MuJoCo.app/Contents/Frameworks/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib",
            "/Applications/MuJoCoStudio.app/Contents/Frameworks/mujoco.framework/Versions/A/libmujoco.3.12.0.dylib",
        ]
        .into_iter()
        .map(Into::into)
        .collect()
    } else {
        [
            "libmujoco.so.3.12.0",
            "libmujoco.so",
            "/usr/local/lib/libmujoco.so",
            "/usr/lib/libmujoco.so",
        ]
        .into_iter()
        .map(Into::into)
        .collect()
    };
    // Managed setup is optional. Missing HOME must not suppress ordinary loader paths.
    let mut paths: Vec<_> = crate::setup::managed_library()
        .ok()
        .flatten()
        .into_iter()
        .collect();
    paths.extend(candidates);
    Ok(paths)
}

/// A complete API probe does not mutate the process-wide admitted API.
pub(crate) fn probe_library(path: &Path) -> Result<(), String> {
    probe_library_with_cancel(path, &crate::cancellation::Cancellation::default())
}

pub(crate) fn probe_library_with_cancel(
    path: &Path,
    cancel: &crate::cancellation::Cancellation,
) -> Result<(), String> {
    admit_library(path, cancel)
        .map(|_| ())
        .map_err(|error| error.details(path))
}

enum AdmissionError {
    Load(String),
    Missing(String),
    Integrity(String),
    Cancelled,
    VersionFunction(String),
    Version(i32),
    Symbols(String),
}

impl AdmissionError {
    fn summary(&self) -> String {
        match self {
            Self::Cancelled => "Operation cancelled".into(),
            Self::Missing(_) => "The selected native library file was not found.".into(),
            Self::Load(cause) if cause.contains("no such file") || cause.contains("No such file") || cause.contains("image not found") => "The native library file or one of its dependencies was not found.".into(),
            Self::Load(cause) if cause.contains("incompatible architecture") || cause.contains("wrong ELF class") => "The native library architecture is incompatible with this executable.".into(),
            Self::Load(cause) if cause.contains("not a mach-o") || cause.contains("invalid ELF") || cause.contains("file too short") => "The selected file is not a valid native shared library.".into(),
            Self::Load(cause) if cause.contains("Permission denied") || cause.contains("permission denied") => "The operating system denied access to the native library.".into(),
            Self::Load(_) => "The operating system rejected loading the native library; inspect the loader cause below.".into(),
            Self::Integrity(cause) if cause.contains("incomplete") || cause.contains("unavailable") => "The managed runtime is missing a required library, license or provenance receipt.".into(),
            Self::Integrity(cause) if cause.contains("differs") || cause.contains("checksum") => "The managed native library no longer matches its verified installation receipt.".into(),
            Self::Integrity(cause) if cause.contains("symlink") || cause.contains("escapes") => "The managed runtime contains an unsafe resource path or symlink.".into(),
            Self::Integrity(_) => "The managed native library failed integrity verification.".into(),
            Self::VersionFunction(_) => "MuJoCo is incompatible: the library does not expose its version function.".into(),
            Self::Version(code) => {
                // VERSIONING.md specifies this encoding from MuJoCo 3.5 onward.
                // Older concatenated codes and invalid values have no inferred version here.
                let found = if *code >= 3_005_000 {
                    format!("MuJoCo {}.{}.{}", code / 1_000_000, code % 1_000_000 / 1_000, code % 1_000)
                } else {
                    "a library with an unrecognized version".into()
                };
                format!("MuJoCo is incompatible: found {found}; this simulator requires MuJoCo 3.12.0.")
            },
            Self::Symbols(_) => "The native library is missing required MuJoCo API symbols.".into(),
        }
    }
    fn phase(&self) -> &'static str {
        match self {
            Self::Cancelled => "cancelled",
            Self::Load(_) | Self::Missing(_) => "library discovery",
            Self::Integrity(_) => "managed integrity",
            Self::Version(_) | Self::VersionFunction(_) => "ABI version",
            Self::Symbols(_) => "API symbols",
        }
    }
    fn details(&self, path: &Path) -> String {
        let cause = match self {
            Self::Cancelled => return "Operation cancelled".into(),
            Self::Load(error)
            | Self::Missing(error)
            | Self::Integrity(error)
            | Self::VersionFunction(error)
            | Self::Symbols(error) => error,
            Self::Version(code) => {
                return format!(
                    "ABI version at {}: unsupported MuJoCo version {code}; required native code 3012000",
                    path.display()
                );
            }
        };
        format!("{} at {}: {cause}", self.phase(), path.display())
    }
}

fn admit_library(
    path: &Path,
    cancel: &crate::cancellation::Cancellation,
) -> Result<mujoco_c::Api, AdmissionError> {
    crate::setup::validate_managed_if_selected(path, cancel).map_err(|error| {
        if error == "Operation cancelled" && cancel.is_cancelled() {
            AdmissionError::Cancelled
        } else {
            AdmissionError::Integrity(error)
        }
    })?;
    // Loading user-selected code is the explicit native operation boundary.
    let library = unsafe { libloading::Library::new(path) }.map_err(|error| {
        // A basename may resolve through the platform loader search path.
        // Only a failed absolute selection has an unambiguous filesystem absence.
        if path.is_absolute() && matches!(path.try_exists(), Ok(false)) {
            AdmissionError::Missing(error.to_string())
        } else {
            AdmissionError::Load(error.to_string())
        }
    })?;
    // Only the version function is queried before the ABI is admitted.
    let version: libloading::Symbol<'_, unsafe extern "C" fn() -> std::os::raw::c_int> = unsafe {
        library
            .get(b"mj_version\0")
            .map_err(|error| AdmissionError::VersionFunction(error.to_string()))?
    };
    let version = unsafe { version() };
    if version != mujoco_c::mjVERSION_HEADER as i32 {
        return Err(AdmissionError::Version(version));
    }
    unsafe { mujoco_c::Api::load(library) }.map_err(AdmissionError::Symbols)
}

fn load_candidates(
    candidates: &[PathBuf],
    cancel: &crate::cancellation::Cancellation,
) -> Result<mujoco_c::Api, Notice> {
    let mut failures = Vec::new();
    for path in candidates {
        cancel
            .check()
            .map_err(|_| Notice::new("Operation cancelled", "", ""))?;
        match admit_library(path, cancel) {
            Ok(api) => return Ok(api),
            Err(AdmissionError::Cancelled) => {
                return Err(Notice::new("Operation cancelled", "", ""));
            }
            Err(error) => failures.push((path, error)),
        }
    }
    let primary = failures
        .iter()
        .find(|(_, error)| !matches!(error, AdmissionError::Load(_) | AdmissionError::Missing(_)))
        .or_else(|| failures.first());
    let primary = primary
        .map(|(_, error)| error.summary())
        .unwrap_or_else(|| "No supported library candidates".into());
    let details = failures
        .iter()
        .map(|(path, error)| error.details(path))
        .collect::<Vec<_>>()
        .join("\n");
    Err(Notice::new(
        format!("MuJoCo 3.12.0 unavailable or incompatible: {primary}"),
        crate::setup::recovery_instruction(),
        format!("Candidates:\n{details}"),
    ))
}

fn api() -> &'static mujoco_c::Api {
    initialize().unwrap_or_else(|error| {
        panic!("native operation without successful MuJoCo admission: {error}")
    });
    API.api.get().expect("successful MuJoCo admission")
}

pub fn mujoco_version() -> &'static str {
    // The admitted library has a process lifetime and exports this static string.
    unsafe { std::ffi::CStr::from_ptr(mujoco_c::mj_versionString()) }
        .to_str()
        .expect("MuJoCo version is ASCII")
}

#[cfg(test)]
pub(crate) mod admission_tests;
