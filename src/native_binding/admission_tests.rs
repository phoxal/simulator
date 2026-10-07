//! Explicit test-owned shared libraries exercise the real complete-symbol loader.
use super::*;
use std::{fs, process::Command};

#[test]
fn collapsed_admission_cause_is_independent_of_long_selected_paths() {
    let long_path = PathBuf::from(format!(
        "/selected/{}/libmujoco",
        "long directory ".repeat(30)
    ));
    for (error, expected) in [
        (AdmissionError::Version(3011000), "incompatible"),
        (
            AdmissionError::Symbols("missing mj_step".into()),
            "missing required",
        ),
        (
            AdmissionError::Integrity("checksum mismatch".into()),
            "no longer matches",
        ),
        (
            AdmissionError::Load("dlopen failed".into()),
            "rejected loading",
        ),
    ] {
        assert!(error.summary().contains(expected));
        assert!(!error.summary().contains("long directory"));
        assert!(
            error
                .details(&long_path)
                .contains(&long_path.display().to_string())
        );
    }
}

pub(crate) fn library(
    directory: &Path,
    name: &str,
    version: Option<i32>,
    complete: bool,
) -> PathBuf {
    let mut source = String::new();
    if let Some(version) = version {
        source.push_str(&format!(
            "#[unsafe(no_mangle)] pub extern \"C\" fn mj_version() -> i32 {{ {version} }}\n"
        ));
    }
    if complete {
        for line in include_str!("upstream_ffi.rs").lines() {
            if let Some(function) = line.trim().strip_prefix("pub fn ") {
                let name = function.split('(').next().unwrap();
                if name != "mj_version" {
                    // Only mj_version is invoked. Other exports prove symbol closure,
                    // never pretend to implement native physics or these signatures.
                    source.push_str(&format!(
                        "#[unsafe(no_mangle)] pub extern \"C\" fn {name}() {{}}\n"
                    ));
                }
            }
        }
    }
    let input = directory.join(format!("{name}.rs"));
    fs::write(&input, source).unwrap();
    let output = directory.join(format!("{name}.{}", std::env::consts::DLL_EXTENSION));
    let result = Command::new("rustc")
        .args(["--crate-type", "cdylib", "--edition", "2024"])
        .arg(&input)
        .arg("-o")
        .arg(&output)
        .output()
        .unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    output
}

#[test]
fn rejected_candidates_retry_and_never_replace_an_admitted_api() {
    let temporary = tempfile::tempdir().unwrap();
    let valid = library(temporary.path(), "valid", Some(3012000), true);
    let wrong = library(temporary.path(), "wrong", Some(3011000), false);
    let absent = library(temporary.path(), "absent", None, false);
    let partial = library(temporary.path(), "partial", Some(3012000), false);
    for rejected in [&wrong, &absent, &partial] {
        assert!(probe_library(rejected).is_err());
        let admission = Admission::new();
        assert!(
            admission
                .initialize(std::slice::from_ref(rejected))
                .is_err()
        );
        assert!(admission.api.get().is_none());
        admission
            .initialize(&[rejected.clone(), valid.clone()])
            .unwrap();
        let before = admission.api.get().unwrap() as *const _;
        admission
            .initialize(std::slice::from_ref(rejected))
            .unwrap();
        assert_eq!(admission.api.get().unwrap() as *const _, before);
    }
    let admission = std::sync::Arc::new(Admission::new());
    let missing = temporary.path().join("repair");
    assert!(
        admission
            .initialize(std::slice::from_ref(&missing))
            .is_err()
    );
    fs::copy(&valid, &missing).unwrap();
    std::thread::scope(|scope| {
        for _ in 0..8 {
            let admission = admission.clone();
            let path = missing.clone();
            scope.spawn(move || admission.initialize(&[path]).unwrap());
        }
    });
    assert!(admission.api.get().is_some());
}

#[test]
fn waiting_admission_cancels_without_caching_a_failure() {
    let admission = std::sync::Arc::new(Admission::new());
    let held = admission.loading.lock().unwrap();
    let cancel = crate::cancellation::Cancellation::default();
    let owner_cancel = cancel.clone();
    let owner = admission.clone();
    let thread = std::thread::spawn(move || owner.initialize_cancellable(&[], &owner_cancel));
    cancel.cancel();
    assert_eq!(
        thread.join().unwrap().unwrap_err().to_string(),
        "Operation cancelled"
    );
    assert!(admission.api.get().is_none());
    drop(held);
    let directory = tempfile::tempdir().unwrap();
    let valid = library(directory.path(), "after_cancel", Some(3012000), true);
    admission.initialize(&[valid]).unwrap();
    assert!(admission.api.get().is_some());
}

#[test]
fn found_version_headlines_use_only_documented_modern_encoding() {
    assert!(
        AdmissionError::Version(3011000)
            .summary()
            .contains("3.11.0")
    );
    assert!(AdmissionError::Version(4002001).summary().contains("4.2.1"));
    for code in [-1, 0, 327, 3004000] {
        let error = AdmissionError::Version(code);
        assert!(error.summary().contains("unrecognized version"));
        assert!(!error.summary().contains(&format!("version {code}")));
        assert!(
            error
                .details(Path::new("/selected/library"))
                .contains(&format!("version {code}"))
        );
    }
    assert!(
        !AdmissionError::Version(3011000)
            .summary()
            .contains("3011000")
    );
    assert!(
        !AdmissionError::Version(3011000)
            .summary()
            .contains("3012000")
    );
}

#[test]
fn actual_missing_path_details_cannot_change_the_native_recovery_action() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("missing\nNext: SECONDARY_PATH_TEXT");
    let failure = load_candidates(&[path], &crate::cancellation::Cancellation::default())
        .err()
        .unwrap();
    assert_eq!(failure.action, crate::setup::recovery_instruction());
    assert!(
        failure
            .primary
            .contains("native library file was not found")
    );
    assert!(failure.details.contains("Next: SECONDARY_PATH_TEXT"));
    assert!(
        failure
            .to_string()
            .contains(&format!("Next: {}", failure.action))
    );
}
