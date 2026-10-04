#[cfg(not(any(target_os = "linux", target_os = "macos")))]
compile_error!("Phoxal supports Linux and macOS only");

mod application_contract;
mod contract;
mod lifecycle;
mod native_binding;
mod project;

mod authority;
mod bindings;
mod bundle;
mod cadence;
mod composition;
mod config;
mod desktop;
mod execution;
mod georeference;
pub mod mujoco;
mod native_provider;
mod observations;
mod remote;
mod runtime;

use std::process::ExitCode;

fn main() -> ExitCode {
    if std::env::args_os()
        .nth(1)
        .is_some_and(|argument| argument == "stage-scene")
    {
        return stage_scene();
    }
    if std::env::args_os()
        .nth(1)
        .is_some_and(|argument| argument == "project")
    {
        return project::run();
    }
    let result = config::Options::from_env().and_then(lifecycle::run);
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) if error.is_empty() => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("phoxal-simulator: {error}");
            ExitCode::FAILURE
        }
    }
}

/// Resource staging belongs to the simulator's existing closed-model owner.
fn stage_scene() -> ExitCode {
    use clap::Parser as _;
    #[derive(clap::Parser)]
    struct StageScene {
        #[arg(long)]
        scene: std::path::PathBuf,
        #[arg(long)]
        output: std::path::PathBuf,
    }
    let options = StageScene::parse_from(
        std::iter::once(std::ffi::OsString::from("stage-scene")).chain(std::env::args_os().skip(2)),
    );
    let result = (|| -> Result<(), String> {
        let closed = mujoco::ClosedModel::from_referenced_file(&options.scene)
            .map_err(|error| error.to_string())?;
        for resource in closed.resources() {
            let destination = options.output.join(resource.name());
            std::fs::create_dir_all(destination.parent().ok_or("resource has no directory")?)
                .map_err(|error| error.to_string())?;
            std::fs::write(destination, resource.bytes()).map_err(|error| error.to_string())?;
        }
        Ok(())
    })();
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("phoxal-simulator: {error}");
            ExitCode::FAILURE
        }
    }
}
