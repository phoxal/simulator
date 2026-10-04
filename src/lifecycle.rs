//! Simulator-owned supervisor lifetime and terminal reporting.
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::time::{Duration, Instant};

use crate::runtime::TerminalEvidence as SimulatorTerminalEvidence;
use phoxal::artifact::simulation::ScenarioExecutionReport;
use serde::{Deserialize, Serialize};

use crate::config::{Options, Presentation};

#[derive(Serialize)]
struct Cleanup {
    supervisor_stop_requested: bool,
    supervisor_exited: bool,
    supervisor_killed: bool,
    error: Option<String>,
}

struct Supervisor {
    child: Child,
    _directory: tempfile::TempDir,
}

// Leave time for the supervisor's bounded participant shutdown and its own teardown.
const SUPERVISOR_SHUTDOWN_GRACE: Duration = Duration::from_secs(15);

fn successful_exit(status: ExitStatus) -> Result<(), String> {
    if status.success() {
        Ok(())
    } else {
        Err(format!("supervisor exited unsuccessfully: {status}"))
    }
}

impl Supervisor {
    fn stop(&mut self) -> Cleanup {
        self.stop_with_timeout(SUPERVISOR_SHUTDOWN_GRACE)
    }

    fn stop_with_timeout(&mut self, timeout: Duration) -> Cleanup {
        let mut result = Cleanup {
            supervisor_stop_requested: true,
            supervisor_exited: false,
            supervisor_killed: false,
            error: None,
        };
        let operation = (|| -> Result<(), String> {
            if let Some(status) = self.child.try_wait().map_err(|e| e.to_string())? {
                result.supervisor_exited = true;
                return successful_exit(status);
            }
            // The supervisor's SIGTERM handler performs orderly graph shutdown.
            if unsafe { libc::kill(self.child.id() as libc::pid_t, libc::SIGTERM) } != 0 {
                return Err(std::io::Error::last_os_error().to_string());
            }
            let deadline = Instant::now() + timeout;
            loop {
                if let Some(status) = self.child.try_wait().map_err(|e| e.to_string())? {
                    result.supervisor_exited = true;
                    return successful_exit(status);
                }
                if Instant::now() >= deadline {
                    self.child.kill().map_err(|e| e.to_string())?;
                    result.supervisor_killed = true;
                    let status = self.child.wait().map_err(|e| e.to_string())?;
                    result.supervisor_exited = true;
                    return Err(format!(
                        "supervisor shutdown timed out after {timeout:?}; forced kill: {status}"
                    ));
                }
                std::thread::sleep(Duration::from_millis(25));
            }
        })();
        result.error = operation.err();
        result
    }
}

impl Drop for Supervisor {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

#[derive(Deserialize)]
#[serde(tag = "schema")]
enum Readiness {
    #[serde(rename = "phoxal/supervisor-ready/v0")]
    V0 { execution: String },
}

fn wait_ready(supervisor: &mut Supervisor, file: &Path) -> Result<(), String> {
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        if file.is_file() {
            let readiness: Readiness =
                serde_json::from_slice(&fs::read(file).map_err(|e| e.to_string())?)
                    .map_err(|e| format!("invalid supervisor readiness: {e}"))?;
            let Readiness::V0 { execution } = readiness;
            if execution.is_empty() {
                return Err("supervisor readiness has no execution identity".into());
            }
            return Ok(());
        }
        if let Some(status) = supervisor.child.try_wait().map_err(|e| e.to_string())? {
            return Err(format!("supervisor exited before readiness: {status}"));
        }
        if Instant::now() >= deadline {
            return Err("supervisor readiness timed out after 30 seconds".into());
        }
        std::thread::sleep(Duration::from_millis(25));
    }
}

/// Native admission precedes every supervisor launch; cleanup also runs on failures.
pub(super) fn run(options: Options) -> Result<(), String> {
    crate::native_binding::initialize()?;
    if options.presentation == Presentation::Desktop && !options.probe {
        return crate::desktop::run(options).map(|_| ());
    }
    run_owned(options, None).map(|_| ())
}

pub(super) fn run_owned(
    mut options: Options,
    worker: Option<crate::desktop::Worker>,
) -> Result<Option<SimulatorTerminalEvidence>, String> {
    let facts = crate::bundle::BundleFacts::load(&options.bundle)?;
    if facts.admitted.target != phoxal::artifact::application::HOST_EXECUTION_TARGET {
        return Err(format!(
            "bundle target {} cannot run on {}",
            facts.admitted.target,
            phoxal::artifact::application::HOST_EXECUTION_TARGET
        ));
    }
    if options.probe {
        return engine(options, worker);
    }
    let mut supervisor = None;
    let mut result_path: Option<PathBuf> = None;
    if options.connect.is_none() {
        let directory = tempfile::Builder::new()
            .prefix("phoxal-sim-")
            .tempdir_in("/tmp")
            .map_err(|e| e.to_string())?;
        let ready = directory.path().join("ready.json");
        let result = directory.path().join("scenario-result.json");
        let endpoint = format!(
            "unixsock-stream/{}",
            directory.path().join("router.sock").display()
        );
        let executable = facts.root.join(&facts.admitted.supervisor.path);
        let canonical = executable
            .canonicalize()
            .map_err(|e| format!("supervisor {}: {e}", executable.display()))?;
        if !canonical.starts_with(&facts.root) || !canonical.is_file() {
            return Err("supervisor executable escapes the bundle or is not a file".into());
        }
        let mut command = Command::new(canonical);
        command
            .arg(&facts.root)
            .arg("--state-dir")
            .arg(directory.path())
            .arg("--ready-file")
            .arg(&ready)
            .arg("--listen")
            .arg(&endpoint)
            .arg("--scope")
            .arg(options.scope.as_deref().unwrap_or("local"))
            .arg("--supervisor-id")
            .arg(options.supervisor_id.as_deref().unwrap_or("local"))
            .arg("--owner-pid")
            .arg(std::process::id().to_string())
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::inherit());
        if let Some(specification) = &options.simulation_run {
            command
                .arg("--simulation-run")
                .arg(specification)
                .arg("--scenario-result")
                .arg(&result);
            result_path = Some(result);
        }
        let child = command
            .spawn()
            .map_err(|e| format!("cannot start supervisor: {e}"))?;
        let mut owner = Supervisor {
            child,
            _directory: directory,
        };
        wait_ready(&mut owner, &ready)?;
        options.connect = Some(endpoint);
        supervisor = Some(owner);
    }
    let presentation = options.presentation;
    let scene = options.scene.clone();
    let bundle = options.bundle.clone();
    let scope = options.scope.clone();
    let supervisor_id = options.supervisor_id.clone();
    let run_id = options.run_id.clone();
    let started = Instant::now();
    let mut outcome = engine(options, worker);
    let cleanup = supervisor
        .as_mut()
        .map(Supervisor::stop)
        .unwrap_or(Cleanup {
            supervisor_stop_requested: false,
            supervisor_exited: true,
            supervisor_killed: false,
            error: None,
        });
    let scenario_result: Result<Option<ScenarioExecutionReport>, String> = result_path
        .map(|path| {
            let bytes =
                fs::read(&path).map_err(|e| format!("scenario result {}: {e}", path.display()))?;
            serde_json::from_slice(&bytes).map_err(|e| format!("invalid scenario result: {e}"))
        })
        .transpose();
    let scenario = match scenario_result {
        Ok(report) => report,
        Err(error) => {
            let message = outcome
                .as_ref()
                .err()
                .map_or_else(|| error.clone(), |cause| format!("{cause}; {error}"));
            outcome = Err(message);
            None
        }
    };
    let terminal = outcome.as_ref().ok().and_then(|e| e.as_ref());
    let provider_contract_verified = terminal.is_some_and(|e| {
        let SimulatorTerminalEvidence::V0 {
            provider_contract_verified,
            outcome,
            requested_steps,
            completed_steps,
            ..
        } = e;
        *provider_contract_verified
            && *requested_steps > 0
            && (outcome == "success" && completed_steps == requested_steps
                || presentation == Presentation::Desktop
                    && outcome == "stopped"
                    && completed_steps <= requested_steps)
    });
    let completion = completion_result(&outcome, &cleanup, provider_contract_verified);
    let executable = std::env::current_exe().map_err(|e| e.to_string())?;
    let report = serde_json::json!({
        "schema":"phoxal/simulation-run/v0", "scene":scene, "bundle":bundle,
        "simulator":{"package":env!("CARGO_PKG_NAME"),"version":env!("CARGO_PKG_VERSION"),"binary":"phoxal-simulator","source":"installed-executable","executable":executable},
        "scope":scope,"supervisor_id":supervisor_id,"run_id":run_id,"supervisor_ready":true,
        "provider_contract_verified":provider_contract_verified,"simulator_exit_code":if completion.is_ok(){0}else{1},
        "simulator_wall_time_ns":u64::try_from(started.elapsed().as_nanos()).unwrap_or(u64::MAX),
        "simulator_stdout":terminal.map(serde_json::to_string).transpose().map_err(|e|e.to_string())?.unwrap_or_default(),
        "simulator_stderr":completion.as_ref().err().cloned().unwrap_or_default(),
        "cleanup":cleanup,"scenario":scenario,"terminal":terminal
    });
    println!(
        "{}",
        serde_json::to_string(&report).map_err(|e| e.to_string())?
    );
    completion?;
    outcome
}

fn completion_result(
    outcome: &Result<Option<SimulatorTerminalEvidence>, String>,
    cleanup: &Cleanup,
    provider_contract_verified: bool,
) -> Result<(), String> {
    let mut errors = Vec::new();
    if let Err(error) = outcome {
        errors.push(error.clone());
    }
    if let Some(error) = &cleanup.error {
        errors.push(format!("supervisor cleanup failed: {error}"));
    }
    if !provider_contract_verified {
        errors.push("native simulation did not produce valid terminal evidence".into());
    }
    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors.join("; "))
    }
}

fn engine(
    options: Options,
    worker: Option<crate::desktop::Worker>,
) -> Result<Option<SimulatorTerminalEvidence>, String> {
    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .map_err(|e| e.to_string())?
        .block_on(crate::runtime::run(options, worker))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn idle_supervisor() -> Supervisor {
        Supervisor {
            child: Command::new("/bin/sleep").arg("60").spawn().unwrap(),
            _directory: tempfile::tempdir().unwrap(),
        }
    }

    #[test]
    fn malformed_readiness_releases_the_owned_process_and_state() {
        let mut supervisor = idle_supervisor();
        let pid = supervisor.child.id();
        let state = supervisor._directory.path().to_path_buf();
        let ready = state.join("ready.json");
        fs::write(
            &ready,
            br#"{"schema":"phoxal/supervisor-ready/v0","execution":""}"#,
        )
        .unwrap();
        assert!(
            wait_ready(&mut supervisor, &ready)
                .unwrap_err()
                .contains("execution identity")
        );
        drop(supervisor);
        assert!(!state.exists());
        assert_eq!(unsafe { libc::kill(pid as libc::pid_t, 0) }, -1);
    }

    #[test]
    fn ready_process_is_reaped_by_orderly_cleanup() {
        let mut supervisor = shutdown_fixture("exit 0");
        let ready = supervisor._directory.path().join("ready.json");
        wait_ready(&mut supervisor, &ready).unwrap();
        let cleanup = supervisor.stop();
        assert!(cleanup.supervisor_exited);
        assert!(!cleanup.supervisor_killed);
        assert!(cleanup.error.is_none());
        assert!(supervisor.child.try_wait().unwrap().is_some());
    }

    fn shutdown_fixture(on_term: &str) -> Supervisor {
        let directory = tempfile::tempdir().unwrap();
        let ready = directory.path().join("ready.json");
        let script = format!(
            "trap '{on_term}' TERM; printf '%s' '{{\"schema\":\"phoxal/supervisor-ready/v0\",\"execution\":\"execution-1\"}}' > \"$1\"; while :; do :; done"
        );
        let child = Command::new("/bin/sh")
            .args(["-c", &script, "fixture"])
            .arg(&ready)
            .spawn()
            .unwrap();
        let mut supervisor = Supervisor {
            child,
            _directory: directory,
        };
        wait_ready(&mut supervisor, &ready).unwrap();
        supervisor
    }

    #[test]
    fn nonzero_shutdown_is_reaped_and_reported_in_the_final_outcome() {
        let mut supervisor = shutdown_fixture("exit 7");
        let cleanup = supervisor.stop();
        assert!(cleanup.supervisor_exited);
        assert!(!cleanup.supervisor_killed);
        assert!(cleanup.error.as_ref().unwrap().contains('7'));
        let failure = completion_result(&Ok(None), &cleanup, true).unwrap_err();
        assert!(failure.contains("supervisor cleanup failed"));
        assert!(failure.contains('7'));
    }

    #[test]
    fn an_already_exited_nonzero_supervisor_is_not_success() {
        let mut supervisor = shutdown_fixture("exit 7");
        assert_eq!(
            unsafe { libc::kill(supervisor.child.id() as libc::pid_t, libc::SIGTERM) },
            0
        );
        assert_eq!(supervisor.child.wait().unwrap().code(), Some(7));
        let cleanup = supervisor.stop();
        assert!(cleanup.supervisor_exited);
        assert!(cleanup.error.unwrap().contains('7'));
    }

    #[test]
    fn forced_shutdown_is_reaped_and_reported_as_failure() {
        let mut supervisor = shutdown_fixture(":");
        let pid = supervisor.child.id();
        let cleanup = supervisor.stop_with_timeout(Duration::from_millis(50));
        assert!(cleanup.supervisor_exited);
        assert!(cleanup.supervisor_killed);
        assert!(cleanup.error.as_ref().unwrap().contains("forced kill"));
        assert_eq!(unsafe { libc::kill(pid as libc::pid_t, 0) }, -1);
        assert!(completion_result(&Ok(None), &cleanup, true).is_err());
    }

    #[test]
    fn final_outcome_retains_engine_and_cleanup_failures() {
        let cleanup = Cleanup {
            supervisor_stop_requested: true,
            supervisor_exited: true,
            supervisor_killed: true,
            error: Some("forced kill".into()),
        };
        let failure = completion_result(&Err("engine failed".into()), &cleanup, false).unwrap_err();
        assert!(failure.contains("engine failed"));
        assert!(failure.contains("forced kill"));
        assert!(failure.contains("terminal evidence"));
    }

    #[test]
    fn an_early_supervisor_failure_is_reported_before_engine_start() {
        let directory = tempfile::tempdir().unwrap();
        let ready = directory.path().join("ready.json");
        let child = Command::new("/bin/sh")
            .args(["-c", "exit 7"])
            .spawn()
            .unwrap();
        let mut supervisor = Supervisor {
            child,
            _directory: directory,
        };
        let error = wait_ready(&mut supervisor, &ready).unwrap_err();
        assert!(error.contains("exited before readiness"), "{error}");
        assert!(error.contains('7'), "{error}");
        assert!(supervisor.stop().supervisor_exited);
    }
}
