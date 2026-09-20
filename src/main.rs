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

#[cfg(test)]
mod tests;

use std::process::ExitCode;

fn main() -> ExitCode {
    let result = config::Options::parse(std::env::args_os().skip(1)).and_then(|options| {
        if options.presentation == config::Presentation::Desktop && !options.probe {
            desktop::run(options)
        } else {
            tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .map_err(|e| format!("cannot create async runtime: {e}"))?
                .block_on(runtime::run(options, None))
        }
    });
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) if error.is_empty() => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("phoxal-simulator-mujoco: {error}");
            ExitCode::FAILURE
        }
    }
}
