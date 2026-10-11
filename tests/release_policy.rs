use std::{path::Path, process::Command};
#[test]
fn release_guard_refuses_published_different_source_and_unavailable_api() {
    let dir = tempfile::tempdir().unwrap();
    git(dir.path(), &["init", "--bare", "remote.git"]);
    git(dir.path(), &["init"]);
    git(dir.path(), &["remote", "add", "origin", "remote.git"]);
    let gh = dir.path().join("gh");
    assert!(
        Command::new("rustc")
            .args([
                "--edition=2024",
                "tests/fixtures/process/release_api.rs",
                "-o"
            ])
            .arg(&gh)
            .status()
            .unwrap()
            .success()
    );
    let sha = "a".repeat(40);
    for (record, unavailable, pass) in [
        (String::new(), false, true),
        (format!("true\t{sha}\n"), false, true),
        (format!("false\t{sha}\n"), false, false),
        (format!("true\t{}\n", "b".repeat(40)), false, false),
        (String::new(), true, false),
    ] {
        let mut command = Command::new("bash");
        command
            .arg(Path::new(env!("CARGO_MANIFEST_DIR")).join(".github/scripts/release-policy.sh"))
            .args(["guard", "0.2.5", &sha])
            .current_dir(dir.path())
            .env("GITHUB_REPOSITORY", "fixture/repository")
            .env(
                "PATH",
                format!(
                    "{}:{}",
                    dir.path().display(),
                    std::env::var("PATH").unwrap()
                ),
            )
            .env("PHOXAL_RELEASE_RECORD", record);
        if unavailable {
            command.env("PHOXAL_RELEASE_API_FAILURE", "1");
        }
        assert_eq!(command.status().unwrap().success(), pass);
    }
}
fn checksum(path: &Path) -> String {
    let bytes = std::fs::read(path).unwrap();
    use sha2::Digest as _;
    format!(
        "{:x}  {}\n",
        sha2::Sha256::digest(bytes),
        path.file_name().unwrap().to_str().unwrap()
    )
}
#[test]
fn final_download_checksums_are_flat_exact_and_reject_changed_assets() {
    let dir = tempfile::tempdir().unwrap();
    let input = dir.path().join("input");
    let names = [
        "phoxal-simulator-0.2.5-aarch64-apple-darwin.dmg",
        "phoxal-simulator-0.2.5-x86_64-linux.tar.gz",
        "phoxal-simulator-0.2.5-aarch64-linux.tar.gz",
    ];
    for (i, name) in names.iter().enumerate() {
        let slot = input.join(i.to_string());
        std::fs::create_dir_all(&slot).unwrap();
        let file = slot.join(name);
        std::fs::write(&file, format!("final artifact {i}")).unwrap();
        std::fs::write(slot.join("final-checksum.txt"), checksum(&file)).unwrap();
    }
    let seal = |output: &Path| {
        Command::new("bash")
            .args([".github/scripts/release-policy.sh", "seal", "0.2.5"])
            .arg(&input)
            .arg(output)
            .status()
            .unwrap()
            .success()
    };
    let output = dir.path().join("sealed");
    assert!(seal(&output));
    let sums = std::fs::read_to_string(output.join("SHA256SUMS.txt")).unwrap();
    assert_eq!(sums.lines().count(), 3);
    for name in names {
        assert!(sums.contains(&format!("  {name}\n")));
    }
    assert!(
        Command::new("shasum")
            .args(["-a", "256", "-c", "SHA256SUMS.txt"])
            .current_dir(&output)
            .status()
            .unwrap()
            .success()
    );
    std::fs::write(input.join("0").join(names[0]), "tampered").unwrap();
    assert!(!seal(&dir.path().join("tampered")));
}

fn git(root: &Path, args: &[&str]) -> String {
    let output = Command::new("git")
        .args(args)
        .current_dir(root)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{args:?}: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    String::from_utf8(output.stdout).unwrap().trim().to_owned()
}
#[test]
fn shallow_guard_checks_remote_lightweight_and_peeled_tags_and_transport_failure() {
    let dir = tempfile::tempdir().unwrap();
    git(dir.path(), &["init", "--bare", "remote.git"]);
    git(dir.path(), &["init", "author"]);
    let author = dir.path().join("author");
    git(&author, &["config", "user.name", "Fixture"]);
    git(
        &author,
        &["config", "user.email", "fixture@example.invalid"],
    );
    git(
        &author,
        &[
            "-c",
            "commit.gpgsign=false",
            "commit",
            "--allow-empty",
            "-m",
            "selected",
        ],
    );
    let selected = git(&author, &["rev-parse", "HEAD"]);
    git(
        &author,
        &[
            "-c",
            "commit.gpgsign=false",
            "commit",
            "--allow-empty",
            "-m",
            "other",
        ],
    );
    let other = git(&author, &["rev-parse", "HEAD"]);
    git(&author, &["remote", "add", "origin", "../remote.git"]);
    git(&author, &["push", "origin", "HEAD:refs/heads/fixture"]);
    let remote = format!("file://{}", dir.path().join("remote.git").display());
    git(
        dir.path(),
        &[
            "clone",
            "--depth=1",
            "--no-tags",
            "--branch",
            "fixture",
            &remote,
            "shallow",
        ],
    );
    let shallow = dir.path().join("shallow");
    assert_eq!(
        git(&shallow, &["rev-parse", "--is-shallow-repository"]),
        "true"
    );
    let gh = dir.path().join("gh");
    assert!(
        Command::new("rustc")
            .args([
                "--edition=2024",
                "tests/fixtures/process/release_api.rs",
                "-o"
            ])
            .arg(&gh)
            .status()
            .unwrap()
            .success()
    );
    let guard = || {
        Command::new("bash")
            .arg(Path::new(env!("CARGO_MANIFEST_DIR")).join(".github/scripts/release-policy.sh"))
            .args(["guard", "0.2.5", &selected])
            .current_dir(&shallow)
            .env("GITHUB_REPOSITORY", "fixture/repository")
            .env("PHOXAL_RELEASE_RECORD", "")
            .env(
                "PATH",
                format!(
                    "{}:{}",
                    dir.path().display(),
                    std::env::var("PATH").unwrap()
                ),
            )
            .status()
            .unwrap()
            .success()
    };
    assert!(guard(), "absent remote tag");
    for annotated in [false, true] {
        for (sha, pass) in [(&selected, true), (&other, false)] {
            if annotated {
                git(
                    &author,
                    &[
                        "-c",
                        "tag.gpgsign=false",
                        "tag",
                        "-a",
                        "phoxal-simulator-v0.2.5",
                        sha,
                        "-m",
                        "fixture",
                    ],
                );
            } else {
                git(
                    &author,
                    &[
                        "-c",
                        "tag.gpgsign=false",
                        "tag",
                        "phoxal-simulator-v0.2.5",
                        sha,
                    ],
                );
            }
            git(
                &author,
                &["push", "origin", "refs/tags/phoxal-simulator-v0.2.5"],
            );
            assert!(git(&shallow, &["tag", "--list"]).is_empty());
            assert_eq!(guard(), pass, "annotated={annotated} source={sha}");
            git(
                &author,
                &["push", "origin", ":refs/tags/phoxal-simulator-v0.2.5"],
            );
            git(&author, &["tag", "-d", "phoxal-simulator-v0.2.5"]);
        }
    }
    git(
        &shallow,
        &["remote", "set-url", "origin", "../unavailable.git"],
    );
    assert!(!guard(), "transport failure must not mean absent tag");
}
#[test]
fn configured_provision_step_keeps_logs_out_of_environment_and_propagates_failure() {
    let workflow = std::fs::read_to_string(".github/workflows/release.yml").unwrap();
    let block = workflow
        .split("- name: Provision the checksum-pinned private engine")
        .nth(1)
        .unwrap()
        .split("- name: Build the release executable")
        .next()
        .unwrap();
    let run = block
        .split("run: |\n")
        .nth(1)
        .unwrap()
        .lines()
        .map(str::trim)
        .collect::<Vec<_>>()
        .join("\n")
        .replace("${{ matrix.target }}", "aarch64-apple-darwin");
    let dir = tempfile::tempdir().unwrap();
    std::fs::create_dir_all(dir.path().join(".github/scripts")).unwrap();
    std::fs::write(dir.path().join(".github/scripts/provision-mujoco.sh"), "printf 'archive: OK\n/dev/disk fixture\nMUJOCO_DYNAMIC_LINK_DIR=%s/lib\n' \"$2\"\nexit \"${FIXTURE_EXIT:-0}\"\n").unwrap();
    let envfile = dir.path().join("env");
    for (exit, pass) in [("0", true), ("7", false)] {
        std::fs::write(&envfile, "").unwrap();
        let out = Command::new("bash")
            .args(["-e", "-c", &run])
            .current_dir(dir.path())
            .env("RUNNER_TEMP", dir.path())
            .env("GITHUB_ENV", &envfile)
            .env("FIXTURE_EXIT", exit)
            .output()
            .unwrap();
        assert_eq!(out.status.success(), pass);
        assert!(
            String::from_utf8(out.stdout)
                .unwrap()
                .contains("archive: OK")
        );
        let content = std::fs::read_to_string(&envfile).unwrap();
        if pass {
            assert_eq!(
                content,
                format!(
                    "MUJOCO_DYNAMIC_LINK_DIR={}/mujoco/lib\n",
                    dir.path().display()
                )
            );
        } else {
            assert!(content.is_empty());
        }
    }
}
