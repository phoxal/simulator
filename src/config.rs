use std::path::PathBuf;

use clap::{ArgGroup, Parser};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Presentation {
    Headless,
    Desktop,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum Bound {
    Steps(u64),
    Duration(std::time::Duration),
}

#[derive(Debug, Clone)]
pub(super) struct Options {
    pub(super) simulation_run: Option<PathBuf>,
    pub(super) probe: bool,
    pub(super) scene: PathBuf,
    pub(super) bundle: PathBuf,
    pub(super) json: bool,
    pub(super) presentation: Presentation,
    pub(super) scope: Option<String>,
    pub(super) connect: Option<String>,
    pub(super) supervisor_id: Option<String>,
    pub(super) run_id: Option<String>,
    pub(super) bound: Option<Bound>,
    pub(super) auto_run: bool,
}

/// Application commands. Help and argument errors are resolved before native loading.
#[derive(Debug, Parser)]
#[command(
    name = "phoxal-simulator",
    version,
    about = "Open or run a Phoxal simulation"
)]
pub(super) struct Cli {
    #[command(subcommand)]
    pub(super) command: Option<AppCommand>,
}

#[derive(Debug, clap::Subcommand)]
pub(super) enum AppCommand {
    /// Run an existing robot build with an explicitly selected scene.
    Run(RunArgs),
    /// Inspect native model facts for robot preparation without launching participants.
    Probe(ProbeArgs),
    /// Copy a scene's closed resource set for robot preparation, without native execution.
    StageScene(StageSceneArgs),
}

#[derive(Debug, clap::Args)]
#[command(group(ArgGroup::new("bound").args(["duration", "steps"])))]
pub(super) struct RunArgs {
    /// Scene MJCF or MJZ archive.
    pub(super) scene: PathBuf,
    /// Existing runnable robot directory.
    #[arg(long)]
    pub(super) build: PathBuf,
    /// Execute without a window, with an explicit finite bound.
    #[arg(long, requires = "bound")]
    headless: bool,
    /// Simulated time to advance, for example 10s or 250ms.
    #[arg(long, value_parser = positive_finite)]
    duration: Option<std::time::Duration>,
    /// Native boundaries requested by a prepared scenario.
    #[arg(long, hide = true, value_parser = clap::value_parser!(u64).range(1..))]
    steps: Option<u64>,
    /// Open paused rather than immediately advancing.
    #[arg(long, conflicts_with = "headless")]
    paused: bool,
    /// Prepared scenario execution request supplied by the scenario command.
    #[arg(long, hide = true)]
    simulation_run: Option<PathBuf>,
    /// Existing supervisor router, for externally owned executions.
    #[arg(long)]
    connect: Option<String>,
    #[arg(long, default_value = "local", value_parser = identity)]
    scope: String,
    #[arg(long, default_value = "local", value_parser = identity)]
    supervisor_id: String,
    #[arg(long, default_value = "local-simulation", value_parser = identity)]
    run_id: String,
}

#[derive(Debug, clap::Args)]
pub(super) struct ProbeArgs {
    /// Explicit scene whose native facts are inspected.
    scene: PathBuf,
    #[arg(long)]
    build: PathBuf,
    /// Standard machine-readable native facts for the developer tool.
    #[arg(long)]
    json: bool,
}

#[derive(Debug, clap::Args)]
pub(super) struct StageSceneArgs {
    #[arg(long)]
    pub(super) scene: PathBuf,
    #[arg(long)]
    pub(super) output: PathBuf,
}

impl RunArgs {
    pub(super) fn into_options(self) -> Options {
        Options {
            simulation_run: self.simulation_run,
            probe: false,
            scene: self.scene,
            bundle: self.build,
            json: false,
            presentation: if self.headless {
                Presentation::Headless
            } else {
                Presentation::Desktop
            },
            scope: Some(self.scope),
            connect: self.connect,
            supervisor_id: Some(self.supervisor_id),
            run_id: Some(self.run_id),
            bound: self
                .duration
                .map(Bound::Duration)
                .or(self.steps.map(Bound::Steps)),
            auto_run: !self.paused,
        }
    }
}

impl ProbeArgs {
    pub(super) fn into_options(self) -> Options {
        Options {
            simulation_run: None,
            probe: true,
            scene: self.scene,
            bundle: self.build,
            json: self.json,
            presentation: Presentation::Headless,
            scope: None,
            connect: None,
            supervisor_id: None,
            run_id: None,
            bound: None,
            auto_run: false,
        }
    }
}

fn identity(value: &str) -> Result<String, String> {
    validate_identity(value, "identity")?;
    Ok(value.to_owned())
}

impl Bound {
    pub(super) fn steps(self, quantum_ns: u64) -> Result<u64, String> {
        match self {
            Self::Steps(steps) => Ok(steps),
            Self::Duration(duration) => {
                if quantum_ns == 0 {
                    return Err("simulation quantum must be positive".into());
                }
                let nanos = duration.as_nanos();
                if nanos == 0 || nanos % u128::from(quantum_ns) != 0 {
                    return Err(format!(
                        "duration {duration:?} is not an integral number of {quantum_ns}ns simulation quanta"
                    ));
                }
                u64::try_from(nanos / u128::from(quantum_ns))
                    .map_err(|_| "duration produces too many simulation steps".into())
            }
        }
    }
}

impl Presentation {
    pub(super) const fn label(self) -> &'static str {
        match self {
            Self::Headless => "headless",
            Self::Desktop => "desktop",
        }
    }
}

fn positive_finite(value: &str) -> Result<std::time::Duration, String> {
    let duration = humantime::parse_duration(value).map_err(|error| error.to_string())?;
    if duration.is_zero() {
        return Err("duration must be positive (for example 10s or 250ms)".into());
    }
    Ok(duration)
}

fn validate_identity(value: &str, field: &str) -> Result<(), String> {
    if value.is_empty()
        || value.len() > 128
        || !value.bytes().all(|byte| {
            byte.is_ascii_lowercase() || byte.is_ascii_digit() || matches!(byte, b'-' | b'_')
        })
    {
        return Err(format!(
            "{field} must be 1-128 lowercase ASCII letters, digits, '-' or '_'"
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(args: &[&str]) -> Result<Options, String> {
        let cli =
            Cli::try_parse_from(std::iter::once("phoxal-simulator").chain(args.iter().copied()))
                .map_err(|error| error.to_string())?;
        match cli.command {
            Some(AppCommand::Run(args)) => Ok(args.into_options()),
            _ => Err("expected run".into()),
        }
    }

    #[test]
    fn desktop_has_no_default_bound_and_can_start_paused() {
        let options = run(&["run", "scene.xml", "--build", "build"]).unwrap();
        assert_eq!(options.presentation, Presentation::Desktop);
        assert!(options.bound.is_none());
        assert!(options.auto_run);
        assert!(
            !run(&["run", "scene.xml", "--build", "build", "--paused"])
                .unwrap()
                .auto_run
        );
    }

    #[test]
    fn headless_has_a_finite_time_bound() {
        let options = run(&[
            "run",
            "scene.xml",
            "--build",
            "build",
            "--headless",
            "--duration",
            "300ms",
        ])
        .unwrap();
        assert_eq!(
            options.bound,
            Some(Bound::Duration(std::time::Duration::from_millis(300)))
        );
        assert!(run(&["run", "scene.xml", "--build", "build", "--headless"]).is_err());
    }

    #[test]
    fn no_request_selects_idle_without_a_run() {
        assert!(
            Cli::try_parse_from(["phoxal-simulator"])
                .unwrap()
                .command
                .is_none()
        );
    }

    #[test]
    fn time_bounds_reject_zero_quantum_and_step_counter_overflow() {
        assert!(
            Bound::Duration(std::time::Duration::from_secs(1))
                .steps(0)
                .is_err()
        );
        assert!(Bound::Duration(std::time::Duration::MAX).steps(1).is_err());
    }

    #[test]
    fn duration_must_be_an_exact_quantum_multiple() {
        assert_eq!(
            Bound::Duration(std::time::Duration::from_millis(20))
                .steps(10_000_000)
                .unwrap(),
            2
        );
        assert!(
            Bound::Duration(std::time::Duration::from_millis(15))
                .steps(10_000_000)
                .is_err()
        );
    }
}
