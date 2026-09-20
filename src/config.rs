use std::path::PathBuf;

#[cfg(test)]
use std::ffi::OsString;

use clap::{ArgGroup, Parser};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Presentation {
    Headless,
    Desktop,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum Bound {
    Steps(u64),
    Duration(f64),
}

#[derive(Debug)]
pub(super) struct Options {
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

#[derive(Debug, Parser)]
#[command(
    name = "phoxal-simulator",
    version = env!("CARGO_PKG_VERSION"),
    about = "Run the native Phoxal MuJoCo simulator",
    group(ArgGroup::new("presentation").required(true).args(["headless", "desktop"]))
)]
struct Cli {
    /// Inspect the composed model without connecting to a supervisor.
    #[arg(long)]
    probe: bool,
    /// Scene MJCF or MJZ archive.
    #[arg(long)]
    scene: PathBuf,
    /// Prepared Phoxal bundle directory.
    #[arg(long)]
    bundle: PathBuf,
    /// Emit machine-readable probe output.
    #[arg(long, requires = "probe")]
    json: bool,
    /// Run without a presentation window.
    #[arg(long, conflicts_with = "desktop")]
    headless: bool,
    /// Open the interactive simulator window.
    #[arg(long, conflicts_with = "headless")]
    desktop: bool,
    /// Supervisor router endpoint.
    #[arg(long)]
    connect: Option<String>,
    /// Deployment namespace.
    #[arg(long)]
    scope: Option<String>,
    /// Supervisor identity within the namespace.
    #[arg(long)]
    supervisor_id: Option<String>,
    /// Finite simulation run identity.
    #[arg(long)]
    run_id: Option<String>,
    /// Advance exactly this many native quanta.
    #[arg(long, conflicts_with = "duration", value_parser = clap::value_parser!(u64).range(1..))]
    steps: Option<u64>,
    /// Advance exactly this many seconds.
    #[arg(long, conflicts_with = "steps", value_parser = positive_finite)]
    duration: Option<f64>,
    /// Start a desktop run immediately instead of paused.
    #[arg(long)]
    auto_run: bool,
}

impl Bound {
    pub(super) fn steps(self, quantum_ns: u64) -> Result<u64, String> {
        match self {
            Self::Steps(steps) => Ok(steps),
            Self::Duration(seconds) => {
                let duration_ns = seconds * 1_000_000_000.0;
                let quanta = duration_ns / quantum_ns as f64;
                let nearest = quanta.round();
                let tolerance = f64::EPSILON * quanta.abs().max(1.0) * 16.0;
                if !quanta.is_finite() || nearest < 1.0 || (quanta - nearest).abs() > tolerance {
                    return Err(format!(
                        "duration {seconds} seconds is not an integral number of {quantum_ns}ns simulation quanta"
                    ));
                }
                if nearest > u64::MAX as f64 {
                    return Err("duration produces too many simulation steps".to_owned());
                }
                Ok(nearest as u64)
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

impl Options {
    pub(super) fn from_env() -> Result<Self, String> {
        Self::from_cli(Cli::parse())
    }

    #[cfg(test)]
    pub(super) fn parse(args: impl IntoIterator<Item = OsString>) -> Result<Self, String> {
        let arguments = std::iter::once(OsString::from("phoxal-simulator")).chain(args);
        let cli = Cli::try_parse_from(arguments).map_err(|error| error.to_string())?;
        Self::from_cli(cli)
    }

    fn from_cli(cli: Cli) -> Result<Self, String> {
        let presentation = if cli.headless {
            Presentation::Headless
        } else {
            Presentation::Desktop
        };
        let bound = match (cli.steps, cli.duration) {
            (Some(steps), None) => Some(Bound::Steps(steps)),
            (None, Some(duration)) => Some(Bound::Duration(duration)),
            (None, None) => None,
            (Some(_), Some(_)) => unreachable!("clap rejects conflicting bounds"),
        };
        if cli.probe {
            if cli.connect.is_some()
                || cli.scope.is_some()
                || cli.supervisor_id.is_some()
                || cli.run_id.is_some()
                || bound.is_some()
                || cli.auto_run
            {
                return Err(
                    "probe accepts model facts only; remove run identity and finite-bound options"
                        .to_owned(),
                );
            }
        } else {
            if cli.connect.is_none()
                || cli.scope.is_none()
                || cli.supervisor_id.is_none()
                || cli.run_id.is_none()
            {
                return Err(
                    "a run requires --connect, --scope, --supervisor-id, and --run-id".to_owned(),
                );
            }
            if bound.is_none() {
                return Err("a run requires exactly one of --steps or --duration".to_owned());
            }
        }
        for (field, value) in [
            ("scope", cli.scope.as_deref()),
            ("supervisor id", cli.supervisor_id.as_deref()),
            ("run id", cli.run_id.as_deref()),
        ] {
            if let Some(value) = value {
                validate_identity(value, field)?;
            }
        }
        Ok(Self {
            probe: cli.probe,
            scene: cli.scene,
            bundle: cli.bundle,
            json: cli.json,
            presentation,
            scope: cli.scope,
            connect: cli.connect,
            supervisor_id: cli.supervisor_id,
            run_id: cli.run_id,
            bound,
            auto_run: cli.auto_run,
        })
    }
}

fn positive_finite(value: &str) -> Result<f64, String> {
    let value = value
        .parse::<f64>()
        .map_err(|_| "duration must be a positive finite number".to_owned())?;
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err("duration must be a positive finite number".to_owned())
    }
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
