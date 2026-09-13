use std::ffi::OsString;
use std::path::PathBuf;

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
    pub(super) fn parse(args: impl Iterator<Item = OsString>) -> Result<Self, String> {
        let mut probe = false;
        let mut scene = None;
        let mut bundle = None;
        let mut json = false;
        let mut presentation = None;
        let mut scope = None;
        let mut connect = None;
        let mut supervisor_id = None;
        let mut run_id = None;
        let mut bound = None;
        let mut args = args.peekable();
        while let Some(argument) = args.next() {
            let argument = argument
                .into_string()
                .map_err(|_| "arguments must be valid UTF-8".to_owned())?;
            match argument.as_str() {
                "--help" | "-h" => {
                    println!(
                        "usage: phoxal-simulator-mujoco --probe --scene PATH --bundle PATH --json [--headless|--desktop]\n       phoxal-simulator-mujoco --scene PATH --bundle PATH --headless|--desktop --connect ENDPOINT --scope S --supervisor-id ID --run-id ID --steps N|--duration SEC"
                    );
                    return Err(String::new());
                }
                "--probe" => {
                    if probe {
                        return Err("--probe may be specified once".to_owned());
                    }
                    probe = true;
                }
                "--json" => {
                    if json {
                        return Err("--json may be specified once".to_owned());
                    }
                    json = true;
                }
                "--headless" => {
                    if presentation
                        .replace(Presentation::Headless)
                        .is_some_and(|value| value != Presentation::Headless)
                    {
                        return Err("choose either --headless or --desktop".to_owned());
                    }
                }
                "--desktop" => {
                    if presentation
                        .replace(Presentation::Desktop)
                        .is_some_and(|value| value != Presentation::Desktop)
                    {
                        return Err("choose either --headless or --desktop".to_owned());
                    }
                }
                "--scene" => {
                    let value = next_value(&mut args, "--scene")?;
                    if scene.replace(PathBuf::from(value)).is_some() {
                        return Err("--scene may be specified once".to_owned());
                    }
                }
                "--bundle" => {
                    let value = next_value(&mut args, "--bundle")?;
                    if bundle.replace(PathBuf::from(value)).is_some() {
                        return Err("--bundle may be specified once".to_owned());
                    }
                }
                "--connect" => {
                    let value = next_value(&mut args, "--connect")?;
                    if connect.replace(value).is_some() {
                        return Err("--connect may be specified once".into());
                    }
                }
                "--scope" => {
                    let value = next_value(&mut args, "--scope")?;
                    if scope.replace(value).is_some() {
                        return Err("--scope may be specified once".to_owned());
                    }
                }
                "--supervisor-id" => {
                    let value = next_value(&mut args, "--supervisor-id")?;
                    if supervisor_id.replace(value).is_some() {
                        return Err("--supervisor-id may be specified once".to_owned());
                    }
                }
                "--run-id" => {
                    let value = next_value(&mut args, "--run-id")?;
                    if run_id.replace(value).is_some() {
                        return Err("--run-id may be specified once".to_owned());
                    }
                }
                "--steps" => {
                    let value = next_value(&mut args, "--steps")?;
                    let steps = value
                        .parse::<u64>()
                        .map_err(|_| "--steps requires a positive integer".to_owned())?;
                    if steps == 0 {
                        return Err("--steps requires a positive integer".to_owned());
                    }
                    if bound.replace(Bound::Steps(steps)).is_some() {
                        return Err("choose exactly one of --steps or --duration".to_owned());
                    }
                }
                "--duration" => {
                    let value = next_value(&mut args, "--duration")?;
                    let seconds = value
                        .parse::<f64>()
                        .map_err(|_| "--duration requires a positive finite number".to_owned())?;
                    if !seconds.is_finite() || seconds <= 0.0 {
                        return Err("--duration requires a positive finite number".to_owned());
                    }
                    if bound.replace(Bound::Duration(seconds)).is_some() {
                        return Err("choose exactly one of --steps or --duration".to_owned());
                    }
                }
                value if value.starts_with('-') => {
                    return Err(format!("unknown option {value}"));
                }
                value => return Err(format!("unexpected positional argument {value:?}")),
            }
        }
        let scene = scene.ok_or_else(|| "--scene requires a path".to_owned())?;
        let bundle = bundle.ok_or_else(|| "--bundle requires a path".to_owned())?;
        let presentation = presentation.ok_or_else(|| {
            if probe {
                "a probe requires either --headless or --desktop".to_owned()
            } else {
                "a run requires either --headless or --desktop".to_owned()
            }
        })?;
        if probe {
            if connect.is_some()
                || scope.is_some()
                || supervisor_id.is_some()
                || run_id.is_some()
                || bound.is_some()
            {
                return Err(
                    "probe accepts model facts only; remove run identity and finite-bound options"
                        .to_owned(),
                );
            }
        } else {
            if json {
                return Err("--json is only valid with --probe".to_owned());
            }
            if connect.is_none() || scope.is_none() || supervisor_id.is_none() || run_id.is_none() {
                return Err(
                    "a run requires --connect, --scope, --supervisor-id, and --run-id".to_owned(),
                );
            }
            if bound.is_none() {
                return Err("a run requires exactly one of --steps or --duration".to_owned());
            }
        }
        for (field, value) in [
            ("scope", scope.as_deref()),
            ("supervisor id", supervisor_id.as_deref()),
            ("run id", run_id.as_deref()),
        ] {
            if let Some(value) = value {
                validate_identity(value, field)?;
            }
        }
        Ok(Self {
            probe,
            scene,
            bundle,
            json,
            presentation,
            scope,
            connect,
            supervisor_id,
            run_id,
            bound,
        })
    }
}

pub(super) fn next_value(
    args: &mut impl Iterator<Item = OsString>,
    option: &str,
) -> Result<String, String> {
    args.next()
        .ok_or_else(|| format!("{option} requires a value"))?
        .into_string()
        .map_err(|_| format!("{option} value must be valid UTF-8"))
}

pub(super) fn validate_identity(value: &str, field: &str) -> Result<(), String> {
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
