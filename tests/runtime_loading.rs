//! Process acceptance of the native-free executable and lazy ABI boundary.
use std::fs;
use std::path::Path;
use std::process::Command;

fn simulator() -> Command {
    let mut command = Command::new(env!("CARGO_BIN_EXE_phoxal-simulator"));
    command
        .env_remove("DYLD_LIBRARY_PATH")
        .env_remove("LD_LIBRARY_PATH")
        .env_remove("MUJOCO_DYNAMIC_LINK_DIR");
    command
}

#[test]
fn help_version_and_scene_staging_do_not_load_mujoco() {
    for argument in ["--help", "--version"] {
        let result = simulator()
            .env("PHOXAL_MUJOCO_LIBRARY", "/missing/mujoco")
            .arg(argument)
            .output()
            .unwrap();
        assert!(
            result.status.success(),
            "{}",
            String::from_utf8_lossy(&result.stderr)
        );
    }
    let directory = tempfile::tempdir().unwrap();
    let scene = directory.path().join("scene.xml");
    fs::write(&scene, "<mujoco><worldbody/></mujoco>").unwrap();
    let output = directory.path().join("closure");
    let result = simulator()
        .env("PHOXAL_MUJOCO_LIBRARY", "/missing/mujoco")
        .arg("stage-scene")
        .arg("--scene")
        .arg(&scene)
        .arg("--output")
        .arg(&output)
        .output()
        .unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    assert_eq!(
        fs::read(output.join("scene.xml")).unwrap(),
        fs::read(scene).unwrap()
    );
}

#[test]
fn missing_library_is_an_actionable_native_error_before_bundle_access() {
    let result = simulator()
        .env("PHOXAL_MUJOCO_LIBRARY", "/missing/mujoco")
        .args([
            "run",
            "--scene",
            "/missing/scene",
            "--bundle",
            "/missing/bundle",
            "--headless",
            "--steps",
            "1",
        ])
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(1));
    let error = String::from_utf8_lossy(&result.stderr);
    assert!(error.contains("MuJoCo 3.12.0 is not available"), "{error}");
    assert!(error.contains("PHOXAL_MUJOCO_LIBRARY"), "{error}");
    assert!(
        !error.contains("canonicalize bundle"),
        "native admission must precede bundle use: {error}"
    );
}

fn fake_library(directory: &Path, version: i32) -> std::path::PathBuf {
    let source = directory.join("fake.c");
    fs::write(
        &source,
        format!("int mj_version(void) {{ return {version}; }}\n"),
    )
    .unwrap();
    let library = directory.join(if cfg!(target_os = "macos") {
        "fake.dylib"
    } else {
        "fake.so"
    });
    let flag = if cfg!(target_os = "macos") {
        "-dynamiclib"
    } else {
        "-shared"
    };
    let status = Command::new("cc")
        .args([flag, "-fPIC"])
        .arg(&source)
        .arg("-o")
        .arg(&library)
        .status()
        .unwrap();
    assert!(status.success());
    library
}

#[test]
fn unsupported_version_is_rejected_before_resolving_model_symbols() {
    let directory = tempfile::tempdir().unwrap();
    let library = fake_library(directory.path(), 3011000);
    let result = simulator()
        .env("PHOXAL_MUJOCO_LIBRARY", library)
        .args([
            "run",
            "--scene",
            "scene",
            "--bundle",
            "bundle",
            "--headless",
            "--steps",
            "1",
        ])
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(1));
    let error = String::from_utf8_lossy(&result.stderr);
    assert!(
        error.contains("unsupported MuJoCo version 3011000"),
        "{error}"
    );
    assert!(
        !error.contains("symbol mj_"),
        "version admission must precede symbol resolution: {error}"
    );
}

#[test]
fn accepted_version_still_requires_the_complete_native_symbol_set() {
    let directory = tempfile::tempdir().unwrap();
    let library = fake_library(directory.path(), 3012000);
    let result = simulator()
        .env("PHOXAL_MUJOCO_LIBRARY", library)
        .args([
            "run",
            "--scene",
            "scene",
            "--bundle",
            "bundle",
            "--headless",
            "--steps",
            "1",
        ])
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(1));
    let error = String::from_utf8_lossy(&result.stderr);
    assert!(error.contains("MuJoCo symbol"), "{error}");
}
