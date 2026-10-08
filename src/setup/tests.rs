//! Real archive/filesystem/API boundaries with explicit fixture inputs.
use super::*;
use std::sync::{
    Arc,
    atomic::{AtomicUsize, Ordering},
};

#[test]
fn distribution_requires_an_exact_supported_execution_target() {
    for (target, archive, dmg) in [
        (
            "aarch64-apple-darwin",
            "mujoco-3.12.0-macos-universal2.dmg",
            true,
        ),
        (
            "x86_64-apple-darwin",
            "mujoco-3.12.0-macos-universal2.dmg",
            true,
        ),
        (
            "aarch64-unknown-linux-gnu",
            "mujoco-3.12.0-linux-aarch64.tar.gz",
            false,
        ),
        (
            "x86_64-unknown-linux-gnu",
            "mujoco-3.12.0-linux-x86_64.tar.gz",
            false,
        ),
    ] {
        let spec = distribution_for(target).unwrap();
        assert_eq!(spec.target, target);
        assert_eq!(spec.archive, archive);
        assert_eq!(spec.dmg, dmg);
    }
    for target in [
        "aarch64-unknown-linux-musl",
        "x86_64-unknown-linux-musl",
        "armv7-unknown-linux-gnueabihf",
        "x86_64-pc-windows-msvc",
    ] {
        let error = distribution_for(target).err().unwrap();
        assert!(error.contains(target));
        assert!(managed_library_for(target).unwrap().is_none());
        // External files bypass managed integrity checks, not the loader's ABI checks.
        validate_managed_for(
            Path::new("external-library"),
            &Cancellation::default(),
            target,
        )
        .unwrap();
    }
    assert_eq!(
        distribution().unwrap().target,
        phoxal::artifact::application::HOST_EXECUTION_TARGET
    );
}

fn archive(root: &Path) -> (PathBuf, String) {
    let library =
        crate::native_binding::admission_tests::library(root, "runtime", Some(3012000), true);
    let path = root.join("fixture.tar.gz");
    let gzip =
        flate2::write::GzEncoder::new(File::create(&path).unwrap(), flate2::Compression::default());
    let mut builder = tar::Builder::new(gzip);
    for (name, bytes) in [
        ("lib/runtime", fs::read(library).unwrap()),
        ("LICENSE", b"fixture license".to_vec()),
        ("THIRD_PARTY_NOTICES", b"fixture notices".to_vec()),
    ] {
        let mut header = tar::Header::new_gnu();
        header.set_size(bytes.len() as u64);
        header.set_mode(0o644);
        header.set_cksum();
        builder
            .append_data(
                &mut header,
                format!("mujoco-{VERSION}/{name}"),
                bytes.as_slice(),
            )
            .unwrap();
    }
    let mut directory = tar::Header::new_gnu();
    directory.set_entry_type(tar::EntryType::Directory);
    directory.set_size(0);
    directory.set_mode(0o755);
    directory.set_cksum();
    builder
        .append_data(
            &mut directory,
            format!("mujoco-{VERSION}/include"),
            std::io::empty(),
        )
        .unwrap();
    builder.into_inner().unwrap().finish().unwrap();
    let digest = format!("{:x}", Sha256::digest(fs::read(&path).unwrap()));
    (path, digest)
}

fn spec(checksum: &str) -> Distribution {
    Distribution {
        target: "fixture",
        archive: "fixture.tar.gz",
        checksum: Box::leak(checksum.to_owned().into_boxed_str()),
        library: "lib/runtime",
        dmg: false,
    }
}

#[test]
fn checksum_failure_cancellation_offline_retry_and_atomic_reuse() {
    let temporary = tempfile::tempdir().unwrap();
    let (archive, checksum) = archive(temporary.path());
    let root = temporary.path().join("managed");
    let bad = spec("wrong");
    let error = install(&root, &bad, &Cancellation::default(), |path, _| {
        fs::copy(&archive, path)
            .map(|_| ())
            .map_err(|e| e.to_string())
    })
    .unwrap_err();
    assert!(error.contains("checksum mismatch"), "{error}");
    assert!(!root.join("mujoco/3.12.0/fixture").exists());
    assert_eq!(
        fs::read_dir(root.join("mujoco/3.12.0")).unwrap().count(),
        1,
        "only serialization lock remains"
    );
    let cancel = Cancellation::default();
    let cancelled = install(&root, &spec(&checksum), &cancel, |path, cancel| {
        fs::copy(&archive, path).unwrap();
        cancel.cancel();
        Ok(())
    })
    .unwrap_err();
    assert!(cancelled.contains("cancelled"));
    let offline = install(&root, &spec(&checksum), &Cancellation::default(), |_, _| {
        Err("offline".into())
    })
    .unwrap_err();
    assert_eq!(offline, "offline");
    let installed = install(
        &root,
        &spec(&checksum),
        &Cancellation::default(),
        |path, _| {
            fs::copy(&archive, path)
                .map(|_| ())
                .map_err(|e| e.to_string())
        },
    )
    .unwrap();
    let before = fs::read(&installed).unwrap();
    let reused = install(&root, &spec(&checksum), &Cancellation::default(), |_, _| {
        panic!("valid offline installation must not download")
    })
    .unwrap();
    assert_eq!(reused, installed);
    assert_eq!(fs::read(&installed).unwrap(), before);
    fs::write(
        installed
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .join("provenance.json"),
        b"{}",
    )
    .unwrap();
    assert!(
        install(
            &root,
            &spec(&checksum),
            &Cancellation::default(),
            |_, _| panic!("never overwrite an existing installation")
        )
        .is_err()
    );
    assert_eq!(fs::read(installed).unwrap(), before);
}

#[test]
fn concurrent_setup_obtains_once_and_admits_every_returned_install() {
    let temporary = tempfile::tempdir().unwrap();
    let (archive, checksum) = archive(temporary.path());
    let root = temporary.path().join("managed");
    let obtains = Arc::new(AtomicUsize::new(0));
    std::thread::scope(|scope| {
        for _ in 0..4 {
            let obtains = obtains.clone();
            let root = &root;
            let archive = &archive;
            let checksum = &checksum;
            scope.spawn(move || {
                let installed = install(
                    root,
                    &spec(checksum),
                    &Cancellation::default(),
                    |destination, _| {
                        obtains.fetch_add(1, Ordering::SeqCst);
                        fs::copy(archive, destination)
                            .map(|_| ())
                            .map_err(|e| e.to_string())
                    },
                )
                .unwrap();
                crate::native_binding::probe_library(&installed).unwrap();
            });
        }
    });
    assert_eq!(obtains.load(Ordering::SeqCst), 1);
}

#[test]
fn escaping_paths_links_and_resource_budgets_fail_before_copy() {
    for path in ["../escape", "/absolute", "lib/../../escape"] {
        assert!(safe_relative(Path::new(path)).is_err());
    }
    assert!(safe_link(Path::new("lib"), Path::new("../../escape")).is_err());
    assert!(safe_link(Path::new("lib"), Path::new("/absolute")).is_err());
    safe_link(Path::new("mujoco.framework"), Path::new("Versions/A")).unwrap();
    let temporary = tempfile::tempdir().unwrap();
    fs::write(temporary.path().join("big"), b"1234").unwrap();
    let mut budget = CopyBudget {
        bytes: MAX_EXPANDED - 2,
        files: 0,
    };
    assert!(
        copy_tree(
            temporary.path(),
            &temporary.path().join("big"),
            &temporary.path().join("copy"),
            &mut budget,
            &Cancellation::default()
        )
        .unwrap_err()
        .contains("size")
    );
    assert!(!temporary.path().join("copy").exists());
    let mut output = Vec::new();
    assert!(
        copy_bounded(
            &mut b"1234".as_slice(),
            &mut output,
            3,
            &Cancellation::default()
        )
        .is_err()
    );
    assert!(output.is_empty());
}

#[test]
fn pre_cancelled_setup_and_non_directory_root_do_not_publish() {
    let temporary = tempfile::tempdir().unwrap();
    let cancel = Cancellation::default();
    cancel.cancel();
    assert!(
        install(temporary.path(), &spec("unused"), &cancel, |_, _| panic!(
            "cancelled before download"
        ))
        .is_err()
    );
    assert!(!temporary.path().join("mujoco").exists());
    let root = temporary.path().join("file");
    fs::write(&root, b"preserve").unwrap();
    assert!(
        install(
            &root,
            &spec("unused"),
            &Cancellation::default(),
            |_, _| panic!("invalid root before download")
        )
        .is_err()
    );
    assert_eq!(fs::read(root).unwrap(), b"preserve");
}

#[test]
fn image_attach_failure_and_cancel_always_report_owned_detach() {
    for cancellation in [false, true] {
        let temporary = tempfile::tempdir().unwrap();
        let output = temporary.path().join("runtime");
        fs::create_dir(&output).unwrap();
        let cancel = Cancellation::default();
        let mut calls = Vec::new();
        let outcome = stage_image(
            Path::new("verified.dmg"),
            &output,
            temporary.path(),
            &cancel,
            |operation, latch| match operation {
                ImageOperation::Attach { archive, mount } => {
                    assert_eq!(archive, Path::new("verified.dmg"));
                    assert_eq!(mount, temporary.path().join("mount"));
                    calls.push("attach");
                    if cancellation {
                        latch.cancel();
                        Ok(())
                    } else {
                        Err("attach failed after mount".into())
                    }
                }
                ImageOperation::Detach { mount } => {
                    assert_eq!(mount, temporary.path().join("mount"));
                    calls.push("detach");
                    assert!(
                        !latch.is_cancelled(),
                        "cleanup must not inherit cancellation"
                    );
                    Ok(())
                }
            },
        );
        assert!(outcome.cleanup_complete);
        let error = outcome.result.unwrap_err();
        assert!(
            error.contains(if cancellation {
                "cancelled"
            } else {
                "attach failed"
            }),
            "{error}"
        );
        assert_eq!(calls, ["attach", "detach"]);
    }
    let temporary = tempfile::tempdir().unwrap();
    let outcome = stage_image(
        Path::new("verified.dmg"),
        temporary.path(),
        temporary.path(),
        &Cancellation::default(),
        |operation, _| match operation {
            ImageOperation::Attach { .. } => Err("primary mount failure".into()),
            ImageOperation::Detach { .. } => Err("detach refused".into()),
        },
    );
    assert!(!outcome.cleanup_complete);
    let error = outcome.result.unwrap_err();
    assert!(
        error.contains("primary mount failure") && error.contains("detach refused"),
        "{error}"
    );
}

#[test]
fn existing_install_validation_respects_caller_cancellation() {
    let temporary = tempfile::tempdir().unwrap();
    let cancel = Cancellation::default();
    cancel.cancel();
    let error = validate_install(temporary.path(), &spec("unused"), &cancel).unwrap_err();
    assert_eq!(error, "Operation cancelled");
}

#[test]
fn actual_image_tool_process_retains_final_diagnostics_and_reaps_cancelled_child() {
    let temporary = tempfile::tempdir().unwrap();
    let executable = temporary.path().join("image-tool");
    let compiled = std::process::Command::new("rustc")
        .args(["--edition=2024", "-o"])
        .arg(&executable)
        .arg(Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/process/image_tool.rs"))
        .output()
        .unwrap();
    assert!(
        compiled.status.success(),
        "{}",
        String::from_utf8_lossy(&compiled.stderr)
    );
    let mut command = std::process::Command::new(&executable);
    command.arg("fail");
    let failure = image::run_image_process(
        &mut command,
        &Cancellation::default(),
        Duration::from_secs(5),
    )
    .unwrap_err();
    assert!(failure.to_string().contains("FINAL_IMAGE_TOOL_CAUSE"));
    assert!(failure.to_string().contains("exit status: 7"));
    assert!(!failure.to_string().contains("Ok(())"));
    let ready = temporary.path().join("ready");
    let cancel = Cancellation::default();
    let signal = cancel.clone();
    let observed = ready.clone();
    let controller = std::thread::spawn(move || {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            if let Ok(pid) = fs::read_to_string(&observed) {
                signal.cancel();
                return pid.parse::<i32>().unwrap();
            }
            assert!(
                Instant::now() < deadline,
                "image fixture did not become ready"
            );
            std::thread::sleep(Duration::from_millis(10));
        }
    });
    let mut command = std::process::Command::new(executable);
    command.arg("hold").arg(ready);
    let failure =
        image::run_image_process(&mut command, &cancel, Duration::from_secs(10)).unwrap_err();
    let pid = controller.join().unwrap();
    assert!(failure.to_string().contains("Operation cancelled"));
    assert!(failure.to_string().contains("exited and was reaped"));
    assert_eq!(unsafe { libc::kill(pid, 0) }, -1);
    assert_eq!(
        std::io::Error::last_os_error().raw_os_error(),
        Some(libc::ESRCH)
    );
}

fn raw_archive(path: &Path, entries: &[(&str, tar::EntryType, u64, Option<&str>)]) {
    let mut gzip =
        flate2::write::GzEncoder::new(File::create(path).unwrap(), flate2::Compression::default());
    for (name, kind, size, link) in entries {
        let mut header = tar::Header::new_gnu();
        header.set_mode(0o755);
        header.set_entry_type(*kind);
        header.set_size(*size);
        // Raw header bytes deliberately bypass the builder's safe path checks:
        // these cases must exercise our actual archive admission boundary.
        assert!(name.len() < 100);
        header.as_mut_bytes()[..name.len()].copy_from_slice(name.as_bytes());
        if let Some(link) = link {
            header.set_link_name(link).unwrap();
        }
        header.set_cksum();
        gzip.write_all(header.as_bytes()).unwrap();
        // Oversized entries intentionally omit their payload. Admission must
        // reject the declared size before attempting any payload write.
    }
    gzip.write_all(&[0; 1024]).unwrap();
    gzip.finish().unwrap();
}

#[test]
fn actual_archive_rejects_escape_links_entry_budgets_without_publication_or_temp_leaks() {
    let temporary = tempfile::tempdir().unwrap();
    let outside = temporary.path().join("outside");
    let absolute = outside.to_str().unwrap();
    let cases = [
        vec![("../outside", tar::EntryType::Regular, 0, None)],
        vec![(absolute, tar::EntryType::Regular, 0, None)],
        vec![
            (
                "mujoco-3.12.0/lib/link",
                tar::EntryType::Symlink,
                0,
                Some("../../../outside"),
            ),
            (
                "mujoco-3.12.0/lib/link/write",
                tar::EntryType::Regular,
                0,
                None,
            ),
        ],
        vec![(
            "mujoco-3.12.0/large",
            tar::EntryType::Regular,
            MAX_EXPANDED + 1,
            None,
        )],
        vec![("mujoco-3.12.0/lib", tar::EntryType::Directory, 0, None); MAX_FILES + 1],
    ];
    for (index, entries) in cases.iter().enumerate() {
        let archive = temporary.path().join(format!("malicious-{index}.tar.gz"));
        raw_archive(&archive, entries);
        let checksum = file_digest(&archive, &Cancellation::default()).unwrap();
        let root = temporary.path().join(format!("managed-{index}"));
        let failure = install(
            &root,
            &spec(&checksum),
            &Cancellation::default(),
            |path, _| {
                fs::copy(&archive, path)
                    .map(|_| ())
                    .map_err(|e| e.to_string())
            },
        )
        .unwrap_err();
        assert!(
            failure.contains(if index == 2 {
                "symlink escapes"
            } else if index == 3 {
                "expanded size"
            } else if index == 4 {
                "too many entries"
            } else {
                "Unsafe runtime archive path"
            }),
            "case {index}: {failure}"
        );
        assert!(!outside.exists());
        assert!(!root.join("mujoco/3.12.0/fixture").exists());
        let remaining: Vec<_> = fs::read_dir(root.join("mujoco/3.12.0"))
            .unwrap()
            .map(|entry| entry.unwrap().file_name())
            .collect();
        assert_eq!(
            remaining,
            [std::ffi::OsString::from("fixture.lock")],
            "no extraction temp may remain"
        );
    }
}

#[test]
fn actual_archive_preserves_contained_version_links() {
    let temporary = tempfile::tempdir().unwrap();
    let archive = temporary.path().join("contained.tar.gz");
    raw_archive(
        &archive,
        &[
            ("mujoco-3.12.0/lib/A", tar::EntryType::Directory, 0, None),
            (
                "mujoco-3.12.0/lib/Current",
                tar::EntryType::Symlink,
                0,
                Some("A"),
            ),
            ("mujoco-3.12.0/include", tar::EntryType::Directory, 0, None),
            ("mujoco-3.12.0/LICENSE", tar::EntryType::Regular, 0, None),
            (
                "mujoco-3.12.0/THIRD_PARTY_NOTICES",
                tar::EntryType::Regular,
                0,
                None,
            ),
        ],
    );
    let extraction = temporary.path().join("transaction");
    let output = temporary.path().join("output");
    fs::create_dir(&extraction).unwrap();
    fs::create_dir(&output).unwrap();
    extract_tar(&archive, &output, &extraction, &Cancellation::default()).unwrap();
    assert_eq!(
        fs::read_link(output.join("lib/Current")).unwrap(),
        Path::new("A")
    );
    assert_eq!(
        output.join("lib/Current").canonicalize().unwrap(),
        output.join("lib/A").canonicalize().unwrap()
    );
}

#[test]
fn contextual_setup_command_preserves_the_selected_root() {
    assert_eq!(command_for(None).unwrap(), "phoxal-simulator setup");
    for text in [
        "/tmp/contextual runtime",
        "/tmp/it's selected",
        "/tmp/line\nroot",
        r"/tmp/scene\root",
    ] {
        let command = command_for(Some(Path::new(text))).unwrap();
        // Execute only the argument parser, never the actual setup command.
        let script = command.replacen("phoxal-simulator", "printf '%s\n'", 1);
        let output = std::process::Command::new("/bin/sh")
            .args(["-c", &script])
            .output()
            .unwrap();
        assert!(output.status.success());
        assert_eq!(
            String::from_utf8(output.stdout).unwrap(),
            format!("--runtime-root\n{text}\nsetup\n")
        );
    }
}

#[test]
fn terminal_diagnostics_preserve_recovery_lines_without_terminal_controls() {
    assert_eq!(
        terminal_diagnostic("failed\u{1b}[2J\r\nNext: repair path\u{7}".into()),
        "failed\\u{1b}[2J\\r\nNext: repair path\\u{7}"
    );
}
