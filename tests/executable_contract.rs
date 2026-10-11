//! Characterization of the contract-bearing executable and closed scene staging.
use phoxal::artifact::application::{
    APPLICATION_RECORD_BYTES, BUNDLE_CONTRACT, HOST_EXECUTION_TARGET, SIMULATION_PROTOCOL_CONTRACT,
    SIMULATOR_LAUNCH_CONTRACT, decode_application_contract,
};
use std::{fs, process::Command};
#[path = "support/owned_child.rs"]
mod owned_child;
use owned_child::OwnedChild;

fn simulator() -> Command {
    Command::new(env!("CARGO_BIN_EXE_phoxal-simulator"))
}

#[test]
fn executable_retains_the_portable_application_contract() {
    let bytes = fs::read(env!("CARGO_BIN_EXE_phoxal-simulator")).unwrap();
    let records: Vec<_> = bytes
        .windows(APPLICATION_RECORD_BYTES)
        .filter_map(decode_application_contract)
        .collect();
    assert_eq!(records.len(), 1, "one authoritative embedded record");
    let contract = &records[0];
    assert_eq!(contract.bundle, Some(BUNDLE_CONTRACT));
    assert_eq!(contract.launch, SIMULATOR_LAUNCH_CONTRACT);
    assert_eq!(contract.simulation, Some(SIMULATION_PROTOCOL_CONTRACT));
    assert_eq!(contract.execution, None);
    assert_eq!(contract.target, HOST_EXECUTION_TARGET);
}

#[test]
fn help_version_and_scene_staging_preserve_output_and_exact_resource_bytes() {
    for argument in ["--help", "--version"] {
        let result = simulator().arg(argument).output().unwrap();
        assert!(result.status.success(), "{:?}", result);
        assert!(!result.stdout.is_empty());
        assert!(result.stderr.is_empty());
    }
    let directory = tempfile::tempdir().unwrap();
    let scene = directory.path().join("scene with spaces.xml");
    let mesh = directory.path().join("triangle.obj");
    fs::write(&scene, b"<mujoco><asset><mesh name=\"triangle\" file=\"triangle.obj\"/></asset><worldbody/></mujoco>").unwrap();
    fs::write(&mesh, b"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n").unwrap();
    let output = directory.path().join("staged closure");
    let result = simulator()
        .arg("stage-scene")
        .arg("--scene")
        .arg(&scene)
        .arg("--output")
        .arg(&output)
        .output()
        .unwrap();
    assert!(result.status.success(), "{:?}", result);
    assert!(result.stdout.is_empty(), "staging is a filesystem contract");
    assert!(result.stderr.is_empty());
    assert_eq!(
        fs::read(output.join("scene with spaces.xml")).unwrap(),
        fs::read(scene).unwrap()
    );
    assert_eq!(
        fs::read(output.join("triangle.obj")).unwrap(),
        fs::read(mesh).unwrap()
    );
}

#[test]
fn argument_failure_and_scene_admission_failure_have_distinct_exits() {
    let argument = simulator()
        .args(["run", "scene.xml", "--build", "build", "--headless"])
        .output()
        .unwrap();
    assert_eq!(argument.status.code(), Some(2));
    assert!(argument.stdout.is_empty());
    assert!(String::from_utf8_lossy(&argument.stderr).contains("--duration"));
    let directory = tempfile::tempdir().unwrap();
    let scene = directory.path().join("scene.xml");
    fs::write(
        &scene,
        b"<mujoco><include file=\"../outside.xml\"/></mujoco>",
    )
    .unwrap();
    let result = simulator()
        .arg("stage-scene")
        .arg("--scene")
        .arg(scene)
        .arg("--output")
        .arg(directory.path().join("out"))
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(1));
    assert!(result.stdout.is_empty());
    assert!(String::from_utf8_lossy(&result.stderr).contains("phoxal-simulator:"));
    assert!(!directory.path().join("out").exists());
}
#[test]
fn owned_cli_processes_are_reaped_on_success_error_unwind_and_forced_fallback() {
    use std::time::{Duration, Instant};
    fn failed_operation(_owner: OwnedChild) -> Result<(), String> {
        Err("induced fallible operation".into())
    }
    for case in ["normal", "error", "unwind", "ignore-term"] {
        let directory = tempfile::tempdir().unwrap();
        let ready = directory.path().join("ready");
        let mode = if case == "ignore-term" { ":" } else { "exit 0" };
        let child = Command::new("sh")
            .arg(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/tests/fixtures/process/shutdown.sh"
            ))
            .arg(&ready)
            .arg(mode)
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .spawn()
            .unwrap();
        let mut owner = OwnedChild::new(
            child,
            if case == "ignore-term" {
                Duration::from_millis(50)
            } else {
                Duration::from_secs(2)
            },
        );
        let pid = owner.child().id();
        let deadline = Instant::now() + Duration::from_secs(5);
        while !ready.exists() {
            assert!(owner.child().try_wait().unwrap().is_none());
            assert!(Instant::now() < deadline, "fixture never became ready");
            std::thread::park_timeout(Duration::from_millis(10));
        }
        owned_child::no_children(Command::new("pgrep").args(["-P", &pid.to_string()])).unwrap();
        match case {
            "unwind" => {
                assert!(
                    std::panic::catch_unwind(std::panic::AssertUnwindSafe(move || {
                        let _owner = owner;
                        panic!("induced assertion with owned subprocess");
                    }))
                    .is_err()
                );
            }
            "error" => {
                let result = failed_operation(owner);
                assert!(result.is_err());
            }
            "ignore-term" => {
                use std::os::unix::process::ExitStatusExt;
                assert_eq!(owner.stop().unwrap().signal(), Some(libc::SIGKILL));
            }
            _ => assert!(owner.stop().unwrap().success()),
        }
        assert_eq!(
            unsafe { libc::kill(pid as i32, 0) },
            -1,
            "owned pid remains live after {case}"
        );
        assert_eq!(
            std::io::Error::last_os_error().raw_os_error(),
            Some(libc::ESRCH)
        );
    }
}

#[test]
fn child_inventory_command_errors_are_not_absence_evidence() {
    assert!(
        owned_child::no_children(&mut Command::new("/no/such/inventory-command"))
            .unwrap_err()
            .contains("command failed")
    );
    assert!(
        owned_child::no_children(Command::new("pgrep").arg("--invalid-test-option"))
            .unwrap_err()
            .contains("inventory failed")
    );
}
