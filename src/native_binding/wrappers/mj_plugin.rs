//! MuJoCo plugin library loading.
use crate::native_binding::error::MjPluginError;
use crate::native_binding::mujoco_c::{mj_loadAllPluginLibraries, mj_loadPluginLibrary};
use std::ffi::{CString, c_char, c_int};
use std::path::Path;
/// Callback invoked by [`load_all_plugin_libraries`] for each loaded library.
///
/// Parameters: `filename`, `first` plugin index (`-1` when `count` is 0), and `count` of
/// plugins that the library registered.
pub type MjPluginLibraryLoadCallback = Option<unsafe extern "C" fn(*const c_char, c_int, c_int)>;
/// Loads a single MuJoCo plugin shared library. Wraps [`mj_loadPluginLibrary`].
///
/// # Note
/// A load failure is not reported here: on Unix MuJoCo calls `mju_error`, whose default handler
/// prints the message and ends the process; on Windows MuJoCo discards the result.
///
/// # Errors
/// Returns [`MjPluginError`] if `path` is not valid UTF-8 or contains a null byte.
///
/// # Examples
///
/// Load the PID actuator plugin before using PID-based actuators in a model:
///
/// ```no_run
/// use mujoco_rs::prelude::*;
///
/// load_plugin_library("path/to/mujoco/bin/mujoco_plugin/libactuator.so")
///     .expect("invalid plugin path");
///
/// let model = MjModel::from_xml("model.xml").expect("could not load the model");
/// ```
pub fn load_plugin_library<P: AsRef<Path>>(path: P) -> Result<(), MjPluginError> {
    let s = path
        .as_ref()
        .to_str()
        .ok_or(MjPluginError::InvalidUtf8Path)?;
    let c = CString::new(s).map_err(|_| MjPluginError::NullBytePath)?;
    unsafe { mj_loadPluginLibrary(c.as_ptr()) };
    Ok(())
}
/// Loads all MuJoCo plugin shared libraries found in `directory`.
/// Wraps [`mj_loadAllPluginLibraries`].
///
/// Pass `None` for `callback` to omit per-library notification.
///
/// # Note
/// A library in `directory` that the loader refuses is not reported here: on Unix MuJoCo calls
/// `mju_error`, whose default handler prints the message and ends the process; on Windows MuJoCo
/// discards the result.
///
/// # Errors
/// Returns [`MjPluginError`] if `directory` is not valid UTF-8 or contains a null byte.
///
/// # Examples
///
/// Load all MuJoCo plugins from the plugin directory (e.g. to enable PID actuators,
/// cable elasticity simulation, SDF collision shapes, or custom sensors):
///
/// ```no_run
/// use mujoco_rs::prelude::*;
///
/// // Load all MuJoCo plugins from the plugin directory.
/// // Adjust the path to match your MuJoCo installation.
/// load_all_plugin_libraries("path/to/mujoco/bin/mujoco_plugin", None)
///     .expect("failed to load plugin libraries");
///
/// let model = MjModel::from_xml("model.xml").expect("could not load the model");
/// ```
pub fn load_all_plugin_libraries<P: AsRef<Path>>(
    directory: P,
    callback: MjPluginLibraryLoadCallback,
) -> Result<(), MjPluginError> {
    let s = directory
        .as_ref()
        .to_str()
        .ok_or(MjPluginError::InvalidUtf8Path)?;
    let c = CString::new(s).map_err(|_| MjPluginError::NullBytePath)?;
    unsafe { mj_loadAllPluginLibraries(c.as_ptr(), callback) };
    Ok(())
}
