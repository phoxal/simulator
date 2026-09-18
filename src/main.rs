#[cfg(feature = "rendering")]
#[cfg(feature = "rendering")]
mod authority;
mod bindings;
mod bundle;
mod cadence;
mod composition;
mod config;
#[cfg(feature = "rendering")]
mod desktop;
#[cfg(feature = "rendering")]
mod execution;
#[cfg(feature = "rendering")]
mod georeference;
mod mujoco;
mod native_provider;
mod observations;
mod remote;
mod runtime;

#[cfg(test)]
mod tests;

use std::process::ExitCode;

fn main() -> ExitCode {
    let result: Result<(), String> = (|| -> Result<(), String> {
        let options = config::Options::parse(std::env::args_os().skip(1))?;
        if options.probe {
            let runtime = tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .map_err(|e| format!("cannot create async runtime: {e}"))?;
            return runtime.block_on(runtime::probe(options));
        }
        #[cfg(feature = "rendering")]
        {
            if options.presentation == config::Presentation::Desktop {
                return desktop::run(options);
            }
            let runtime = tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .map_err(|e| format!("cannot create async runtime: {e}"))?;
            return runtime.block_on(runtime::run(options, None));
        }
        #[cfg(not(feature = "rendering"))]
        {
            let _ = options;
            Err(
                "phoxal-simulator-mujoco requires the `rendering` feature to drive a simulation; \
                 rebuild with `--features native,rendering` to exercise the binary. \
                 `--probe` is the only operation available with `--features native`."
                    .to_owned(),
            )
        }
    })();
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) if error.is_empty() => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("phoxal-simulator-mujoco: {error}");
            ExitCode::FAILURE
        }
    }
}
