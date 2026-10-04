//! Internal adaptation of mujoco-rs 6.0.1 (MuJoCo 3.12.0), MIT/Apache-2.0.
//! The original ABI file is unchanged; build.rs generates typed runtime dispatch.
//! This module never links MuJoCo, downloads it, or installs callbacks before validation.
#![allow(dead_code, unused_imports, non_snake_case, clippy::all)]

use std::sync::OnceLock;

pub mod error;
pub mod prelude;
pub mod util;
pub mod wrappers;
#[allow(warnings, clippy::approx_constant)]
pub mod mujoco_c {
    include!(concat!(env!("OUT_DIR"), "/mujoco_dispatch.rs"));
}

static API: OnceLock<Result<mujoco_c::Api, String>> = OnceLock::new();

/// Admission happens before any model/data layout access or process launch.
pub fn initialize() -> Result<(), String> {
    API.get_or_init(load)
        .as_ref()
        .map(|_| ())
        .map_err(Clone::clone)
}

fn load() -> Result<mujoco_c::Api, String> {
    let explicit = std::env::var_os("PHOXAL_MUJOCO_LIBRARY");
    let candidates: Vec<std::ffi::OsString> = if let Some(path) = explicit {
        vec![path]
    } else if cfg!(target_os = "macos") {
        [
            "libmujoco.3.12.0.dylib",
            "libmujoco.dylib",
            "/opt/homebrew/lib/libmujoco.dylib",
            "/usr/local/lib/libmujoco.dylib",
            "/Library/Frameworks/mujoco.framework/Versions/A/libmujoco.dylib",
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
    let mut failures = Vec::new();
    for path in candidates {
        // Loading user-selected code is the explicit native operation boundary.
        let library = match unsafe { libloading::Library::new(&path) } {
            Ok(library) => library,
            Err(error) => {
                failures.push(format!("{}: {error}", path.to_string_lossy()));
                continue;
            }
        };
        // Only the version function is queried before the ABI is admitted.
        let version: libloading::Symbol<'_, unsafe extern "C" fn() -> std::os::raw::c_int> = unsafe {
            library
                .get(b"mj_version\0")
                .map_err(|error| format!("MuJoCo version symbol: {error}"))?
        };
        let version = unsafe { version() };
        if version != mujoco_c::mjVERSION_HEADER as i32 {
            return Err(format!(
                "unsupported MuJoCo version {version}; phoxal-simulator requires 3.12.0 (3012000), from {}",
                path.to_string_lossy()
            ));
        }
        return unsafe { mujoco_c::Api::load(library) };
    }
    Err(format!(
        "MuJoCo 3.12.0 is not available. Install it with your system package manager or provide your own distribution, then set PHOXAL_MUJOCO_LIBRARY to its shared library file. No native library is downloaded. Tried: {}",
        failures.join("; ")
    ))
}

fn api() -> &'static mujoco_c::Api {
    match API.get_or_init(load) {
        Ok(api) => api,
        Err(error) => panic!("native operation without successful MuJoCo admission: {error}"),
    }
}

pub fn mujoco_version() -> &'static str {
    // The admitted library has a process lifetime and exports this static string.
    unsafe { std::ffi::CStr::from_ptr(mujoco_c::mj_versionString()) }
        .to_str()
        .expect("MuJoCo version is ASCII")
}
