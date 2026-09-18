//! Unit tests for the private `mujoco` module.
//!
//! Each submodule is feature-gated so the file is empty (and therefore
//! produces no `dead_code` warnings under the workspace `-D warnings`
//! policy) when its feature is not enabled.

#[cfg(feature = "native")]
mod component_models;

#[cfg(feature = "rendering")]
mod rendering;
