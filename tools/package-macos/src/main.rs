//! Builds a self-contained development macOS application from a simulator binary.

use std::collections::BTreeMap;
use std::env;
use std::ffi::{OsStr, OsString};
use std::fs::{self, File, OpenOptions};
use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

use sha2::{Digest, Sha256};

const NATIVE_NAME: &str = "libmujoco.3.12.0.dylib";

struct Options {
    binary: PathBuf,
    distribution: PathBuf,
    output: PathBuf,
    version: String,
    short_version: String,
    build_version: String,
}

fn main() -> ExitCode {
    match run() {
        Ok(output) => {
            println!("{}", output.display());
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("phoxal-package-macos: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<PathBuf, String> {
    let options = parse(env::args_os().skip(1))?;
    let binary = options
        .binary
        .canonicalize()
        .map_err(|error| format!("cannot resolve binary: {error}"))?;
    let distribution = options
        .distribution
        .canonicalize()
        .map_err(|error| format!("cannot resolve MuJoCo distribution: {error}"))?;
    let output = absolute(options.output)?;
    let receipt = receipt_path(&output);
    if output.extension() != Some(OsStr::new("app"))
        || output.exists()
        || output.is_symlink()
        || receipt.exists()
        || receipt.is_symlink()
    {
        return Err("--output must name a new .app directory".into());
    }

    let native = distribution
        .join("mujoco.framework/Versions/A")
        .join(NATIVE_NAME);
    let dependency = format!("@rpath/mujoco.framework/Versions/A/{NATIVE_NAME}");
    if !command_output("otool", [OsStr::new("-L"), binary.as_os_str()])?.contains(&dependency) {
        return Err("binary does not link the supported MuJoCo 3.12.0 framework".into());
    }
    for required in [
        native.as_path(),
        &distribution.join("LICENSE"),
        &distribution.join("THIRD_PARTY_NOTICES"),
    ] {
        if !required.is_file() {
            return Err(format!(
                "native distribution is incomplete: {}",
                required.display()
            ));
        }
    }

    let parent = output.parent().unwrap_or_else(|| Path::new("."));
    fs::create_dir_all(parent).map_err(display_io("create output parent", parent))?;
    let stage = tempfile::Builder::new()
        .prefix(".phoxal-package-")
        .tempdir_in(parent)
        .map_err(|error| format!("cannot create package staging directory: {error}"))?;
    let app = stage.path().join(
        output
            .file_name()
            .ok_or_else(|| "output has no application name".to_owned())?,
    );
    let contents = app.join("Contents");
    let executable = contents.join("MacOS/phoxal-simulator-mujoco");
    let resources = contents.join("Resources");
    let frameworks = contents.join("Frameworks");
    fs::create_dir_all(executable.parent().expect("executable parent"))
        .and_then(|()| fs::create_dir(&resources))
        .and_then(|()| fs::create_dir(&frameworks))
        .map_err(display_io("create application layout", &app))?;
    fs::copy(&binary, &executable).map_err(display_io("copy simulator", &executable))?;
    let bundled_native = frameworks.join(NATIVE_NAME);
    fs::copy(&native, &bundled_native)
        .map_err(display_io("copy MuJoCo library", &bundled_native))?;
    fs::copy(
        distribution.join("LICENSE"),
        resources.join("MUJOCO_LICENSE"),
    )
    .map_err(display_io("copy MuJoCo license", &resources))?;
    fs::copy(
        distribution.join("THIRD_PARTY_NOTICES"),
        resources.join("MUJOCO_THIRD_PARTY_NOTICES"),
    )
    .map_err(display_io("copy MuJoCo notices", &resources))?;
    fs::copy(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../LICENSE"),
        resources.join("LICENSE"),
    )
    .map_err(display_io("copy simulator license", &resources))?;
    write_info_plist(
        &contents.join("Info.plist"),
        &options.short_version,
        &options.build_version,
    )?;

    command(
        "install_name_tool",
        [
            OsStr::new("-add_rpath"),
            OsStr::new("@executable_path/../Frameworks"),
            executable.as_os_str(),
        ],
    )?;
    command(
        "install_name_tool",
        [
            OsStr::new("-change"),
            OsStr::new(&dependency),
            OsStr::new(&format!("@rpath/{NATIVE_NAME}")),
            executable.as_os_str(),
        ],
    )?;
    command(
        "codesign",
        [
            OsStr::new("--force"),
            OsStr::new("--sign"),
            OsStr::new("-"),
            executable.as_os_str(),
        ],
    )?;
    if digest(&native)? != digest(&bundled_native)? {
        return Err("native library changed during packaging".into());
    }

    let mut provenance = BTreeMap::from([
        (
            "application_input_sha256",
            serde_json::Value::String(digest(&binary)?),
        ),
        (
            "application_version",
            serde_json::Value::String(options.version),
        ),
        (
            "application_short_version",
            serde_json::Value::String(options.short_version),
        ),
        (
            "application_build_version",
            serde_json::Value::String(options.build_version),
        ),
        (
            "mujoco_library_sha256",
            serde_json::Value::String(digest(&bundled_native)?),
        ),
        ("mujoco_version", serde_json::Value::String("3.12.0".into())),
        ("notarized", serde_json::Value::Bool(false)),
        ("signing", serde_json::Value::String("ad-hoc".into())),
    ]);
    write_json(&resources.join("provenance.json"), &provenance, false)?;
    command(
        "codesign",
        [
            OsStr::new("--force"),
            OsStr::new("--sign"),
            OsStr::new("-"),
            app.as_os_str(),
        ],
    )?;
    command(
        "codesign",
        [
            OsStr::new("--verify"),
            OsStr::new("--deep"),
            OsStr::new("--strict"),
            app.as_os_str(),
        ],
    )?;
    command(&executable, [OsStr::new("--help")])?;
    provenance.insert(
        "application_packaged_sha256",
        serde_json::Value::String(digest(&executable)?),
    );
    fs::rename(&app, &output).map_err(display_io("publish application", &output))?;
    write_json(&receipt, &provenance, true)?;
    Ok(output)
}

fn parse(arguments: impl IntoIterator<Item = OsString>) -> Result<Options, String> {
    let mut values = BTreeMap::new();
    let mut arguments = arguments.into_iter();
    while let Some(flag) = arguments.next() {
        let flag = flag
            .into_string()
            .map_err(|_| "argument names must be UTF-8".to_owned())?;
        if !matches!(
            flag.as_str(),
            "--binary" | "--mujoco-distribution" | "--output" | "--version"
        ) {
            return Err(format!("unknown argument `{flag}`"));
        }
        let value = arguments
            .next()
            .ok_or_else(|| format!("{flag} requires a value"))?;
        if values.insert(flag.clone(), value).is_some() {
            return Err(format!("{flag} was provided more than once"));
        }
    }
    let take = |flag: &str| {
        values
            .get(flag)
            .cloned()
            .ok_or_else(|| format!("missing required {flag}"))
    };
    let version = take("--version")?
        .into_string()
        .map_err(|_| "--version must be UTF-8".to_owned())?;
    let (short_version, build_version) = application_versions(&version)?;
    Ok(Options {
        binary: take("--binary")?.into(),
        distribution: take("--mujoco-distribution")?.into(),
        output: take("--output")?.into(),
        version,
        short_version,
        build_version,
    })
}

fn application_versions(version: &str) -> Result<(String, String), String> {
    let (short, build) = version
        .split_once("-dev.")
        .map_or((version, version), |(short, build)| (short, build));
    if short.split('.').count() != 3
        || !short
            .split('.')
            .all(|part| !part.is_empty() && part.bytes().all(|byte| byte.is_ascii_digit()))
    {
        return Err(
            "--version must be a three-part numeric version or a 0.0.0-dev.N development version"
                .into(),
        );
    }
    if version.contains("-dev.")
        && (short != "0.0.0"
            || build.is_empty()
            || build.starts_with('0')
            || !build.bytes().all(|byte| byte.is_ascii_digit()))
    {
        return Err("development --version must use 0.0.0-dev.N with a positive numeric N".into());
    }
    Ok((short.to_owned(), build.to_owned()))
}

fn absolute(path: PathBuf) -> Result<PathBuf, String> {
    if path.is_absolute() {
        Ok(path)
    } else {
        env::current_dir()
            .map(|current| current.join(path))
            .map_err(|error| format!("cannot resolve output directory: {error}"))
    }
}

fn receipt_path(output: &Path) -> PathBuf {
    let mut receipt = output.to_owned();
    receipt.set_extension("app.provenance.json");
    receipt
}

fn command(
    program: impl AsRef<OsStr>,
    arguments: impl IntoIterator<Item = impl AsRef<OsStr>>,
) -> Result<(), String> {
    let program = program.as_ref();
    let output = Command::new(program)
        .args(arguments)
        .output()
        .map_err(|error| format!("cannot run {}: {error}", program.to_string_lossy()))?;
    if output.status.success() {
        Ok(())
    } else {
        let stdout = String::from_utf8_lossy(&output.stdout);
        let stderr = String::from_utf8_lossy(&output.stderr);
        Err(format!(
            "{} failed ({}): stdout: {}; stderr: {}",
            program.to_string_lossy(),
            output.status,
            stdout.trim(),
            stderr.trim()
        ))
    }
}

fn command_output(
    program: impl AsRef<OsStr>,
    arguments: impl IntoIterator<Item = impl AsRef<OsStr>>,
) -> Result<String, String> {
    let program = program.as_ref();
    let output = Command::new(program)
        .args(arguments)
        .output()
        .map_err(|error| format!("cannot run {}: {error}", program.to_string_lossy()))?;
    if output.status.success() {
        Ok(String::from_utf8_lossy(&output.stdout).into_owned())
    } else {
        let stdout = String::from_utf8_lossy(&output.stdout);
        let stderr = String::from_utf8_lossy(&output.stderr);
        Err(format!(
            "{} failed ({}): stdout: {}; stderr: {}",
            program.to_string_lossy(),
            output.status,
            stdout.trim(),
            stderr.trim()
        ))
    }
}

fn digest(path: &Path) -> Result<String, String> {
    let mut source = File::open(path).map_err(display_io("open for hashing", path))?;
    let mut hasher = Sha256::new();
    let mut buffer = [0_u8; 1024 * 1024];
    loop {
        let read = source
            .read(&mut buffer)
            .map_err(display_io("read for hashing", path))?;
        if read == 0 {
            break;
        }
        hasher.update(&buffer[..read]);
    }
    Ok(format!("{:x}", hasher.finalize()))
}

fn write_info_plist(path: &Path, short_version: &str, build_version: &str) -> Result<(), String> {
    let contents = format!(
        r#"<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleExecutable</key><string>phoxal-simulator-mujoco</string>
  <key>CFBundleIdentifier</key><string>com.phoxal.simulator</string>
  <key>CFBundleName</key><string>Phoxal Simulator</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>{short_version}</string>
  <key>CFBundleVersion</key><string>{build_version}</string>
  <key>NSHighResolutionCapable</key><true/>
</dict>
</plist>
"#
    );
    fs::write(path, contents).map_err(display_io("write Info.plist", path))
}

fn write_json(
    path: &Path,
    value: &BTreeMap<&str, serde_json::Value>,
    create_new: bool,
) -> Result<(), String> {
    let mut options = OpenOptions::new();
    options
        .write(true)
        .create(true)
        .truncate(!create_new)
        .create_new(create_new);
    let mut output = options
        .open(path)
        .map_err(display_io("create provenance", path))?;
    serde_json::to_writer_pretty(&mut output, value)
        .map_err(|error| format!("cannot encode provenance: {error}"))?;
    output
        .write_all(b"\n")
        .map_err(display_io("write provenance", path))
}

fn display_io<'a>(action: &'static str, path: &'a Path) -> impl FnOnce(io::Error) -> String + 'a {
    move |error| format!("{action} `{}`: {error}", path.display())
}

#[cfg(test)]
mod tests {
    use super::application_versions;

    #[test]
    fn stable_and_development_versions_map_to_valid_bundle_versions() {
        assert_eq!(
            application_versions("1.2.3").expect("stable version"),
            ("1.2.3".to_owned(), "1.2.3".to_owned())
        );
        assert_eq!(
            application_versions("0.0.0-dev.12").expect("development version"),
            ("0.0.0".to_owned(), "12".to_owned())
        );
    }

    #[test]
    fn invalid_or_non_monotonic_development_versions_are_rejected() {
        for version in ["0.0", "1.2.3-alpha.1", "0.0.0-dev.0", "0.0.0-dev.01"] {
            assert!(
                application_versions(version).is_err(),
                "{version} must be rejected"
            );
        }
    }
}
