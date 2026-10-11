//! Explicit native qualification of the actual application owners.
//! Run with RUSTFLAGS="--cfg native_qualification" and the declared engine/host fixtures.
#[cfg(not(native_qualification))]
compile_error!(
    "native qualification requires RUSTFLAGS=--cfg native_qualification; see README prerequisites"
);
include!("../src/main.rs");
