#[cfg(not(any(target_os = "linux", target_os = "macos")))]
compile_error!("Phoxal supports Linux and macOS only");

mod application_contract;
mod contract;
mod lifecycle;
mod native_binding;

mod authority;
mod bindings;
mod bundle;
mod cadence;
mod cancellation;
mod composition;
mod config;
mod desktop;
mod execution;
mod georeference;
pub mod mujoco;
mod native_provider;
mod notice;
mod observations;
mod process_output;
mod remote;
mod runtime;
mod setup;

use std::process::ExitCode;

fn main() -> ExitCode {
    use clap::Parser as _;
    let cli = config::Cli::parse();
    if let Err(error) = setup::configure_root(cli.runtime_root) {
        write_error(&error);
        return ExitCode::FAILURE;
    }
    let result = match cli.command {
        Some(config::AppCommand::Run(args)) => lifecycle::run(args.into_options()),
        Some(config::AppCommand::Probe(args)) => lifecycle::run(args.into_options()),
        Some(config::AppCommand::StageScene(args)) => return stage_scene(args),
        Some(config::AppCommand::Setup) => {
            setup::run_cli().map(|path| println!("MuJoCo 3.12.0 is ready at {}", path.display()))
        }
        None => desktop::idle().map(|_| ()),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) if error.is_empty() => ExitCode::SUCCESS,
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
