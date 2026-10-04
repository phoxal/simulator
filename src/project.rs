//! Project convenience orchestration through the developer command's public boundary.
use crate::config::{Bound, Options, Presentation};
use clap::Parser;
use std::path::PathBuf;
use std::process::{Command, ExitCode};

#[derive(Parser)]
#[command(
    name = "phoxal-simulator project",
    about = "Prepare the current robot project and open its simulation"
)]
struct Project {
    #[arg(long, default_value = "simulation/scene.xml")]
    scene: PathBuf,
    #[arg(long, default_value_t = 10000, value_parser = clap::value_parser!(u64).range(1..))]
    steps: u64,
    #[arg(long)]
    headless: bool,
    /// Open the desktop execution paused.
    #[arg(long, conflicts_with = "headless")]
    paused: bool,
}

pub(super) fn run() -> ExitCode {
    let options = Project::parse_from(
        std::iter::once(std::ffi::OsString::from("project")).chain(std::env::args_os().skip(2)),
    );
    match prepare_and_run(options) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("phoxal-simulator: {error}");
            ExitCode::FAILURE
        }
    }
}

fn prepare_and_run(project: Project) -> Result<(), String> {
    // Refuse missing/unsupported native libraries before any source acquisition or launch.
    crate::native_binding::initialize()?;
    let staging = tempfile::Builder::new()
        .prefix("phoxal-project-")
        .tempdir()
        .map_err(|e| e.to_string())?;
    let bundle = staging.path().join("bundle");
    let scene_name = project
        .scene
        .file_name()
        .ok_or("scene must name a file")?
        .to_owned();
    let simulator = std::env::current_exe().map_err(|e| e.to_string())?;
    let status = Command::new("cargo").args(["phoxal", "build", "--simulation-scene"])
        .arg(&project.scene).arg("--output").arg(&bundle)
        .env("PHOXAL_SIMULATOR", simulator).status()
        .map_err(|e| format!("cannot invoke cargo phoxal build: {e}; install cargo-phoxal with cargo install cargo-phoxal"))?;
    if !status.success() {
        return Err(format!("robot preparation failed: {status}"));
    }
    crate::lifecycle::run(Options {
        simulation_run: None,
        probe: false,
        scene: bundle.join("scene").join(scene_name),
        bundle,
        json: false,
        presentation: if project.headless {
            Presentation::Headless
        } else {
            Presentation::Desktop
        },
        scope: Some("local".into()),
        connect: None,
        supervisor_id: Some("local".into()),
        run_id: Some("desktop".into()),
        bound: Some(Bound::Steps(project.steps)),
        auto_run: !project.paused,
    })
}
