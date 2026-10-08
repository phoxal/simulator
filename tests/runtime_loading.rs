//! Process acceptance of the native-free executable and lazy ABI boundary.
use std::fs;
use std::path::Path;
use std::process::Command;
#[path = "support/owned_child.rs"]
mod owned_child;
use owned_child::OwnedChild;

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
            "/missing/scene",
            "--build",
            "/missing/bundle",
            "--headless",
            "--steps",
            "1",
        ])
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(1));
    let error = String::from_utf8_lossy(&result.stderr);
    assert!(
        error
            .lines()
            .next()
            .unwrap()
            .contains("native library file was not found"),
        "{error}"
    );
    assert!(
        error.contains("library discovery at /missing/mujoco"),
        "{error}"
    );
    assert!(
        error.contains("repair or remove PHOXAL_MUJOCO_LIBRARY"),
        "{error}"
    );
    assert!(!error.contains("run phoxal-simulator setup"), "{error}");
    assert!(error.contains("PHOXAL_MUJOCO_LIBRARY"), "{error}");
    assert!(
        !error.contains("canonicalize bundle"),
        "native admission must precede bundle use: {error}"
    );
}

fn fake_library(directory: &Path, version: i32) -> std::path::PathBuf {
    let source = directory.join("fake.rs");
    fs::write(
        &source,
        format!("#[unsafe(no_mangle)] pub extern \"C\" fn mj_version() -> i32 {{ {version} }}\n"),
    )
    .unwrap();
    let library = directory.join(if cfg!(target_os = "macos") {
        "fake.dylib"
    } else {
        "fake.so"
    });
    let status = Command::new("rustc")
        .args(["--crate-type", "cdylib", "--edition", "2024"])
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
            "scene",
            "--build",
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
            "scene",
            "--build",
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

#[test]
fn setup_help_and_invalid_runtime_root_do_not_need_an_existing_native_api() {
    let result = simulator()
        .env("PHOXAL_MUJOCO_LIBRARY", "/missing/mujoco")
        .args(["setup", "--help"])
        .output()
        .unwrap();
    assert!(result.status.success());
    let directory = tempfile::tempdir().unwrap();
    let root = directory.path().join("not-a-directory");
    fs::write(&root, b"preserve").unwrap();
    let result = simulator()
        .env("PHOXAL_MUJOCO_LIBRARY", "/missing/mujoco")
        .arg("--runtime-root")
        .arg(&root)
        .arg("setup")
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(1));
    let error = String::from_utf8_lossy(&result.stderr);
    assert!(
        error.contains("Setup failed while creating runtime directory"),
        "{error}"
    );
    assert!(error.contains(&root.display().to_string()), "{error}");
    assert!(
        error.contains("Next: choose a writable directory with --runtime-root DIRECTORY"),
        "{error}"
    );
    assert!(!error.contains("library discovery"), "{error}");
    assert_eq!(fs::read(root).unwrap(), b"preserve");
}

#[test]
#[ignore = "requires an admitted native library and retained real rover build"]
fn invalid_native_scene_reports_selected_paths_and_model_repair() {
    let library = std::env::var_os("PHOXAL_MUJOCO_LIBRARY").expect("explicit native library");
    let build = std::env::var_os("PHOXAL_QUALIFICATION_BUILD").expect("retained real build");
    let directory = tempfile::tempdir().unwrap();
    let scene = directory.path().join("invalid native scene.xml");
    fs::write(
        &scene,
        r#"<mujoco><worldbody><site name="robot_mount"/><geom name="broken" type="sphere" size="-1"/></worldbody></mujoco>"#,
    )
    .unwrap();
    let result = simulator()
        .env("PHOXAL_MUJOCO_LIBRARY", library)
        .arg("run")
        .arg(&scene)
        .arg("--build")
        .arg(&build)
        .args(["--headless", "--steps", "1"])
        .output()
        .unwrap();
    assert_eq!(result.status.code(), Some(1));
    let error = String::from_utf8_lossy(&result.stderr);
    assert!(error.contains("Scene preparation failed"), "{error}");
    assert!(error.contains(scene.to_str().unwrap()), "{error}");
    assert!(
        error.contains(Path::new(&build).to_str().unwrap()),
        "{error}"
    );
    assert!(error.contains("size 0 must be positive"), "{error}");
    assert!(error.contains("Next: repair the selected scene"), "{error}");
    assert!(!error.contains("phoxal-simulator setup"), "{error}");
}

#[test]
#[ignore = "requires an actual supported host native library for loader search"]
fn no_home_still_admits_an_external_loader_search_candidate() {
    let library = std::env::var_os("PHOXAL_MUJOCO_LIBRARY").expect("explicit native library");
    let directory = tempfile::tempdir().unwrap();
    let name = if cfg!(target_os = "macos") {
        "libmujoco.3.12.0.dylib"
    } else {
        "libmujoco.so.3.12.0"
    };
    #[cfg(unix)]
    std::os::unix::fs::symlink(&library, directory.path().join(name)).unwrap();
    let result = simulator()
        .env_remove("HOME")
        .env_remove("PHOXAL_MUJOCO_LIBRARY")
        .env(
            if cfg!(target_os = "macos") {
                "DYLD_LIBRARY_PATH"
            } else {
                "LD_LIBRARY_PATH"
            },
            directory.path(),
        )
        .args([
            "run",
            "/missing/scene",
            "--build",
            "/missing/build",
            "--headless",
            "--steps",
            "1",
        ])
        .output()
        .unwrap();
    let error = String::from_utf8_lossy(&result.stderr);
    assert_eq!(result.status.code(), Some(1));
    assert!(error.contains("canonicalize bundle"), "{error}");
    assert!(!error.contains("Home directory unavailable"), "{error}");
    let setup = simulator()
        .env_remove("HOME")
        .arg("setup")
        .output()
        .unwrap();
    assert_eq!(setup.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&setup.stderr).contains("--runtime-root"));
}

#[cfg(target_os = "macos")]
#[test]
#[ignore = "requires an actual macOS graphical session; bounded terminal launch, not GUI acceptance"]
fn bare_idle_reports_missing_runtime_without_preparation_or_children() {
    let directory = tempfile::tempdir().unwrap();
    let runtime = directory.path().join("private runtime");
    let stderr_path = directory.path().join("stderr");
    let child = simulator()
        .env(
            "PHOXAL_MUJOCO_LIBRARY",
            directory.path().join("missing.dylib"),
        )
        .arg("--runtime-root")
        .arg(&runtime)
        .stdout(std::process::Stdio::null())
        .stderr(fs::File::create(&stderr_path).unwrap())
        .spawn()
        .unwrap();
    let mut child = OwnedChild::new(child, std::time::Duration::from_secs(2));
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
    let result = (|| -> Result<(), String> {
        loop {
            let error = fs::read_to_string(&stderr_path).unwrap();
            if error.contains("Next: repair or remove PHOXAL_MUJOCO_LIBRARY") {
                assert!(error.contains("MuJoCo 3.12.0 unavailable or incompatible:"));
                assert!(
                    child.child().try_wait().unwrap().is_none(),
                    "bare shell must remain open"
                );
                owned_child::no_children(
                    Command::new("pgrep").args(["-P", &child.child().id().to_string()]),
                )?;
                assert!(
                    !runtime.exists(),
                    "availability must not download or prepare"
                );
                return Ok(());
            }
            if child.child().try_wait().unwrap().is_some() {
                return Err(format!("bare shell exited: {error}"));
            }
            if std::time::Instant::now() >= deadline {
                return Err(format!("no availability diagnostic: {error}"));
            }
            std::thread::sleep(std::time::Duration::from_millis(20));
        }
    })();
    // Bounded terminal qualification, not observed GUI-close acceptance.
    child.stop().unwrap();
    result.unwrap();
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

#[test]
fn setup_reports_and_flushes_while_lock_is_held_then_cancels() {
    use fs2::FileExt;
    use std::time::{Duration, Instant};
    let directory = tempfile::tempdir().unwrap();
    let root = directory.path().join("runtime");
    let parent = root.join("mujoco/3.12.0");
    fs::create_dir_all(&parent).unwrap();
    let lock = fs::OpenOptions::new()
        .create(true)
        .truncate(false)
        .write(true)
        .open(parent.join(format!(
            "{}.lock",
            phoxal::artifact::application::HOST_EXECUTION_TARGET
        )))
        .unwrap();
    lock.lock_exclusive().unwrap();
    let stderr = directory.path().join("stderr");
    let stdout = directory.path().join("stdout");
    let child = simulator()
        .arg("--runtime-root")
        .arg(&root)
        .arg("setup")
        .stderr(fs::File::create(&stderr).unwrap())
        .stdout(fs::File::create(&stdout).unwrap())
        .spawn()
        .unwrap();
    let mut owner = OwnedChild::new(child, Duration::from_secs(2));
    let deadline = Instant::now() + Duration::from_secs(12);
    loop {
        let text = fs::read_to_string(&stderr).unwrap();
        if text.lines().any(|line| {
            line.starts_with("Setup: Waiting for setup lock - ") && line.ends_with("s elapsed.")
        }) {
            assert!(
                text.starts_with("Setup: Preparing MuJoCo 3.12.0."),
                "{text}"
            );
            assert!(owner.child().try_wait().unwrap().is_none());
            assert_eq!(fs::metadata(&stdout).unwrap().len(), 0);
            assert!(!text.contains('\u{1b}'));
            break;
        }
        assert!(Instant::now() < deadline, "no incremental output: {text}");
        std::thread::sleep(Duration::from_millis(10));
    }
    assert_eq!(
        unsafe { libc::kill(owner.child().id() as i32, libc::SIGINT) },
        0
    );
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        if let Some(status) = owner.child().try_wait().unwrap() {
            assert!(!status.success());
            break;
        }
        assert!(Instant::now() < deadline, "cancel did not finish");
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(
        fs::read_to_string(stderr)
            .unwrap()
            .contains("Operation cancelled")
    );
    assert_eq!(
        fs::read_dir(parent).unwrap().count(),
        1,
        "cancel published transaction"
    );
}

#[test]
fn setup_reports_while_proxy_connection_is_blocked_without_exposing_proxy_secrets() {
    use std::{
        net::TcpListener,
        time::{Duration, Instant},
    };
    let directory = tempfile::tempdir().unwrap();
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    listener.set_nonblocking(true).unwrap();
    let address = listener.local_addr().unwrap();
    let stderr = directory.path().join("stderr");
    let child = simulator()
        .arg("--runtime-root")
        .arg(directory.path().join("runtime"))
        .arg("setup")
        .env(
            "HTTPS_PROXY",
            format!("http://private-user:private-secret@{address}"),
        )
        .env("NO_PROXY", "")
        .env("no_proxy", "")
        .stderr(fs::File::create(&stderr).unwrap())
        .stdout(std::process::Stdio::null())
        .spawn()
        .unwrap();
    let mut owner = OwnedChild::new(child, Duration::from_secs(2));
    let deadline = Instant::now() + Duration::from_secs(12);
    let connection = loop {
        match listener.accept() {
            Ok((connection, _)) => break connection,
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(error) => panic!("{error}"),
        }
        assert!(Instant::now() < deadline, "proxy never contacted");
        std::thread::sleep(Duration::from_millis(10));
    };
    loop {
        let text = fs::read_to_string(&stderr).unwrap();
        if text.lines().any(|line| {
            line.starts_with("Setup: Waiting for download data - ") && line.ends_with("s elapsed.")
        }) {
            assert!(owner.child().try_wait().unwrap().is_none());
            assert!(!text.contains("private-secret"));
            assert!(!text.contains('\u{1b}'));
            break;
        }
        assert!(Instant::now() < deadline, "no network liveness: {text}");
        std::thread::sleep(Duration::from_millis(10));
    }
    assert_eq!(
        unsafe { libc::kill(owner.child().id() as i32, libc::SIGINT) },
        0
    );
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        if let Some(status) = owner.child().try_wait().unwrap() {
            assert!(!status.success());
            break;
        }
        assert!(Instant::now() < deadline, "network cancel did not finish");
        std::thread::sleep(Duration::from_millis(10));
    }
    drop(connection);
    assert!(
        fs::read_to_string(stderr)
            .unwrap()
            .contains("Operation cancelled")
    );
    assert_eq!(
        fs::read_dir(directory.path().join("runtime/mujoco/3.12.0"))
            .unwrap()
            .count(),
        1
    );
}

#[test]
fn setup_cancel_with_closed_stderr_exits_failure_without_panic_or_publication() {
    use fs2::FileExt;
    use std::{
        io::Read,
        os::fd::AsRawFd,
        time::{Duration, Instant},
    };
    let directory = tempfile::tempdir().unwrap();
    let root = directory.path().join("runtime");
    let parent = root.join("mujoco/3.12.0");
    fs::create_dir_all(&parent).unwrap();
    let lock = fs::OpenOptions::new()
        .create(true)
        .truncate(false)
        .write(true)
        .open(parent.join(format!(
            "{}.lock",
            phoxal::artifact::application::HOST_EXECUTION_TARGET
        )))
        .unwrap();
    lock.lock_exclusive().unwrap();
    let child = simulator()
        .arg("--runtime-root")
        .arg(&root)
        .arg("setup")
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .unwrap();
    let mut owner = OwnedChild::new(child, Duration::from_secs(2));
    let mut reader = owner.child().stderr.take().unwrap();
    let flags = unsafe { libc::fcntl(reader.as_raw_fd(), libc::F_GETFL) };
    assert!(flags >= 0);
    assert_eq!(
        unsafe { libc::fcntl(reader.as_raw_fd(), libc::F_SETFL, flags | libc::O_NONBLOCK) },
        0
    );
    let deadline = Instant::now() + Duration::from_secs(3);
    let mut received = Vec::new();
    loop {
        let mut bytes = [0; 1024];
        match reader.read(&mut bytes) {
            Ok(0) => panic!("setup exited before waiting"),
            Ok(count) => received.extend_from_slice(&bytes[..count]),
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(error) => panic!("{error}"),
        }
        if String::from_utf8_lossy(&received).contains("Setup: Waiting for setup lock.") {
            break;
        }
        assert!(Instant::now() < deadline, "no flushed lock wait");
        std::thread::park_timeout(Duration::from_millis(10));
    }
    drop(reader);
    assert_eq!(
        unsafe { libc::kill(owner.child().id() as i32, libc::SIGINT) },
        0
    );
    let deadline = Instant::now() + Duration::from_secs(3);
    loop {
        if let Some(status) = owner.child().try_wait().unwrap() {
            assert_eq!(
                status.code(),
                Some(1),
                "cancellation must not panic or succeed"
            );
            break;
        }
        assert!(
            Instant::now() < deadline,
            "closed stderr prevented cancellation"
        );
        std::thread::park_timeout(Duration::from_millis(10));
    }
    assert_eq!(
        fs::read_dir(parent).unwrap().count(),
        1,
        "cancel published a transaction"
    );
}
