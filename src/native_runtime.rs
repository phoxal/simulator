//! Exact ABI admission before accessing linked native model/data layouts.
use crate::{cancellation::Cancellation, notice::Notice};

pub(crate) fn initialize() -> Result<(), String> {
    initialize_with_cancel(&Cancellation::default())
        .map(|_| ())
        .map_err(|error| error.to_string())
}

pub(crate) fn initialize_with_cancel(cancel: &Cancellation) -> Result<Initialization, Notice> {
    if cancel.is_cancelled() {
        return Ok(Initialization::Cancelled);
    }
    // The linker owns the library's process lifetime. Query only this scalar
    // version function before any binding accesses ABI-dependent structures.
    let version = unsafe { mujoco_rs::mujoco_c::mj_version() };
    if version != mujoco_rs::mujoco_c::mjVERSION_HEADER as i32 {
        return Err(Notice::new(
            "The application engine is incompatible with this executable",
            "reinstall the complete Phoxal Simulator application.",
            format!("MuJoCo native code {version}; required 3012000 (3.12.0)"),
        ));
    }
    Ok(Initialization::Ready)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum Initialization {
    Ready,
    Cancelled,
}
