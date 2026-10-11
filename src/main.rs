#[cfg(not(any(target_os = "linux", target_os = "macos")))]
compile_error!("Phoxal supports Linux and macOS only");

mod app;
mod application_contract;
mod contract;
mod lifecycle;
mod native_runtime;

mod authority;
mod bindings;
mod bundle;
mod cadence;
mod cancellation;
mod composition;
mod config;
mod desktop;
mod document;
mod execution;
mod georeference;
pub mod mujoco;
mod native_provider;
mod notice;
mod observations;
mod process_output;
mod remote;
mod runtime;

use std::process::ExitCode;

fn main() -> ExitCode {
    use clap::Parser as _;
    let cli = config::Cli::parse();
    let result = match cli.command {
        Some(config::AppCommand::Smoke) => app::world::smoke(),
        Some(config::AppCommand::Run(args)) => {
            let options = args.into_options();
            if options.presentation == config::Presentation::Desktop {
                desktop::run(options).map(|_| ())
            } else {
                lifecycle::run(options)
            }
        }
        Some(config::AppCommand::Probe(args)) => lifecycle::run(args.into_options()),
        Some(config::AppCommand::StageScene(args)) => return stage_scene(args),
        None => desktop::idle().map(|_| ()),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            write_error(&error);
            ExitCode::FAILURE
        }
    }
}

// Diagnostics must not turn a closed consumer into a panic or delay cancellation.
fn write_error(error: &str) {
    use std::io::Write as _;
    let mut stderr = std::io::stderr().lock();
    let _ = writeln!(stderr, "phoxal-simulator: {error}");
    let _ = stderr.flush();
}

/// Resource staging belongs to the simulator's existing closed-model owner.
fn stage_scene(options: config::StageSceneArgs) -> ExitCode {
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
