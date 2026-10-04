# Runtime MuJoCo binding

This simulator-private module adapts mujoco-rs 6.0.1+mj-3.12.0 under its MIT and Apache-2.0 licenses, retained beside the source.
`upstream_ffi.rs` is the original generated MuJoCo 3.12.0 ABI declaration from that published crate and is not edited by hand.
The wrappers retain upstream typed model, data, rendering, and error behavior.
Unused upstream viewer and unit-test code is omitted; simulator-owned native acceptance exercises the operations actually consumed here.

The root build script reads the ABI declaration with syn and generates typed function-pointer dispatch into Cargo's OUT_DIR.
It preserves ABI types, constants, calling conventions, and variadic function signatures, replacing direct external symbol calls with a library-owned dispatch table.
A normal build needs no C compiler, MuJoCo headers, or native library.
The runtime resolves the version function first, admits exact 3.12.0, then resolves the required function set and retains the library for the process lifetime.
Model and data destruction use that same validated instance.

To update the ABI, obtain the independently generated declaration from the qualified upstream binding release, retain its provenance and licenses, then regenerate through the normal Cargo build.
An ABI update requires native qualification and independent missing-version, incompatible-version, and symbol-refusal tests before changing the accepted version.
Do not hand-edit the generated dispatch file or substitute a dummy native library for product installation.
