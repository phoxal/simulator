//! Source builds resolve their private engine through the selected link
//! directory, mirroring the packaged executable-relative layout.
fn main() {
    // The rpath follows the engine the linker actually selected, so runtime
    // discovery stays coherent when a deliberate environment override (CI
    // provisioning, a relocated provisioned root) changes the link directory.
    // Cargo's repository [.cargo/config.toml] supplies the default value;
    // see README "Contributor source builds".
    println!("cargo:rerun-if-env-changed=MUJOCO_DYNAMIC_LINK_DIR");
    let engine_lib = std::env::var("MUJOCO_DYNAMIC_LINK_DIR").ok();
    match std::env::var("CARGO_CFG_TARGET_OS").as_deref() {
        Ok("macos") => {
            println!("cargo:rustc-link-arg=-Wl,-rpath,@executable_path/../Frameworks");
            if let Some(dir) = engine_lib {
                println!("cargo:rustc-link-arg=-Wl,-rpath,{dir}");
            }
        }
        Ok("linux") => {
            println!("cargo:rustc-link-arg=-Wl,-rpath,$ORIGIN/../lib");
            if let Some(dir) = engine_lib {
                println!("cargo:rustc-link-arg=-Wl,-rpath,{dir}");
            }
        }
        _ => {}
    }
}
