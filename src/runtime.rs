use crate::bindings::ProbeFacts;
use crate::bindings::build_provider;
use crate::bindings::probe_contract;
use crate::bundle::BundleFacts;
use crate::composition::load_composed_model;
use crate::config::Options;
use crate::config::Presentation;
use crate::mujoco::Model;
use crate::mujoco::Scene;
use crate::native_provider::ComponentProvider;
use crate::remote::ProvenanceInput;
use crate::remote::RemoteSceneRun;
use crate::remote::SIMULATION_PROTOCOL;
use serde::Serialize;

pub(super) const CONTROL_PRINCIPAL: &str = "simulator";

pub(super) struct PreparedNative {
    bundle: BundleFacts,
    model: Model,
    quantum_ns: u64,
    provider: ComponentProvider,
    pub(super) context: phoxal::artifact::simulation_context::SimulationContext,
}

pub(super) fn prepare(options: &Options) -> Result<PreparedNative, String> {
    let mut bundle = BundleFacts::load(&options.bundle)?;
    let model = load_composed_model(&options.scene, &bundle)?;
    let simulation = crate::bindings::simulation_definition(&bundle, &model)?;
    let quantum_ns = simulation.quantum_ns;
    let manifest =
        crate::bundle::read_bounded_regular(&bundle.root.join("manifest.json"), 16 * 1024 * 1024)?;
    let context =
        phoxal::artifact::simulation_context::SimulationContext::new(&manifest, simulation.clone());
    context.clone().admit(&manifest, &mut bundle.admitted)?;
    bundle.simulation = Some(simulation);
    // Resolve native inputs/outputs before any supervisor or participant starts.
    let provider = build_provider(
        &bundle,
        &model,
        bundle.simulation.as_ref().ok_or("missing native facts")?,
    )?;
    Ok(PreparedNative {
        bundle,
        model,
        quantum_ns,
        provider,
        context,
    })
}

pub(super) async fn run(
    options: Options,
    desktop: Option<crate::desktop::Worker>,
    prepared: PreparedNative,
) -> Result<Option<crate::runtime::TerminalEvidence>, String> {
    let cancel = desktop
        .as_ref()
        .map(|owner| owner.cancel.clone())
        .unwrap_or_default();
    let display = desktop.as_ref().map(|owner| owner.display.clone());
    let PreparedNative {
        bundle,
        model,
        quantum_ns,
        provider,
        ..
    } = prepared;

    if options.probe {
        if !options.json {
            return Err(
                "a probe requires --json so its facts have one stable machine-readable shape"
                    .to_owned(),
            );
        }
        let contract = probe_contract(&bundle, &model)?;
        let facts = ProbeFacts {
            model_identity: model.identity().to_hex(),
            quantum_ns,
            providers: contract.providers,
            actuation_bindings: contract.actuation_bindings,
        };
        println!(
            "{}",
            serde_json::to_string(&facts).map_err(|error| error.to_string())?
        );
        return Ok(None);
    }

    let simulation = bundle
        .simulation
        .as_ref()
        .ok_or_else(|| "native context has no phoxal.simulation.v1 contract".to_owned())?;
    if simulation.protocol != SIMULATION_PROTOCOL || simulation.mode != "controlled" {
        return Err(format!(
            "native simulation contract is {} / {}, expected {} / controlled",
            simulation.protocol, simulation.mode, SIMULATION_PROTOCOL
        ));
    }
    if simulation.model_identity != model.identity().to_hex() {
        return Err(format!(
            "native context model identity {} does not match composed native model {}",
            simulation.model_identity,
            model.identity().to_hex()
        ));
    }
    if simulation.quantum_ns != quantum_ns {
        return Err(format!(
            "native context quantum {}ns does not match composed native quantum {}ns",
            simulation.quantum_ns, quantum_ns
        ));
    }

    let requested_steps = match options.bound {
        Some(bound) => bound.steps(quantum_ns)?,
        // Interactive execution has no user-imposed bound; only the native
        // boundary counter's representable range limits it.
        None if options.presentation == crate::config::Presentation::Desktop => u64::MAX,
        None => return Err("headless simulation requires a finite duration".into()),
    };
    let scope = options
        .scope
        .as_deref()
        .ok_or_else(|| "a run requires --scope".to_owned())?;
    let supervisor_id = options
        .supervisor_id
        .as_deref()
        .ok_or_else(|| "a run requires --supervisor-id".to_owned())?;
    let run_id = options
        .run_id
        .as_deref()
        .ok_or_else(|| "a run requires --run-id".to_owned())?;

    let endpoint = options
        .connect
        .as_deref()
        .ok_or("a run requires --connect")?;
    let config = phoxal::session::ConnectionConfig::new(
        endpoint,
        scope.to_owned(),
        CONTROL_PRINCIPAL.to_owned(),
    )
    .map_err(|error| error.to_string())?;
    let connection = tokio::select! {
        biased;
        result = phoxal::session::connect(config) => result.map_err(|error| error.to_string())?,
        _ = cancel.wait() => return Ok(None),
    };
    let selected = tokio::select! {
        biased;
        result = connection.supervisor(supervisor_id) => result.map(Some),
        _ = cancel.wait() => Ok(None),
    };
    let supervisor = match selected {
        Ok(None) => {
            stopping(&display);
            return cleanup_outcome(
                Ok(None),
                connection.close().await.map_err(|e| e.to_string()),
                &display,
                "public session",
            );
        }
        Ok(Some(supervisor)) => supervisor,
        Err(error) => {
            stopping(&display);
            return cleanup_outcome(
                Err(error.to_string()),
                connection.close().await.map_err(|e| e.to_string()),
                &display,
                "public session",
            );
        }
    };
    let outcome = execute_remote_run(
        &supervisor,
        &bundle,
        model,
        provider,
        RunRequest {
            run_id,
            requested_steps,
            presentation: options.presentation,
            auto_run: options.auto_run,
            collect_every_boundary: options.simulation_run.is_some(),
        },
        desktop,
    )
    .await;
    stopping(&display);
    drop(supervisor);
    let close_result = connection.close().await;
    cleanup_outcome(
        outcome,
        close_result.map_err(|e| e.to_string()),
        &display,
        "public session",
    )
}

type Display = Option<std::sync::Arc<std::sync::Mutex<crate::desktop::DisplayState>>>;

fn stopping(display: &Display) {
    if let Some(display) = display
        && let Ok(mut state) = display.lock()
    {
        state.phase = "Stopping";
        state.stopping = true;
    }
}

fn cleanup_outcome<T>(
    outcome: Result<T, String>,
    cleanup: Result<(), String>,
    display: &Display,
    owner: &str,
) -> Result<T, String> {
    match cleanup {
        Ok(()) => outcome,
        Err(cause) => {
            if let Some(display) = display
                && let Ok(mut state) = display.lock()
            {
                state.cleanup_failed = true;
            }
            let cleanup = format!("{owner} cleanup unconfirmed: {cause}");
            Err(match outcome {
                Ok(_) => cleanup,
                Err(primary) => format!("{primary}; {cleanup}"),
            })
        }
    }
}

struct RunRequest<'a> {
    run_id: &'a str,
    requested_steps: u64,
    presentation: Presentation,
    auto_run: bool,
    collect_every_boundary: bool,
}

async fn execute_remote_run(
    supervisor: &phoxal::session::Supervisor,
    bundle: &BundleFacts,
    model: Model,
    provider: ComponentProvider,
    request: RunRequest<'_>,
    desktop: Option<crate::desktop::Worker>,
) -> Result<Option<TerminalEvidence>, String> {
    let display = desktop.as_ref().map(|owner| owner.display.clone());
    let cancel = desktop
        .as_ref()
        .map(|owner| owner.cancel.clone())
        .unwrap_or_default();
    let RunRequest {
        run_id,
        requested_steps,
        presentation,
        auto_run,
        collect_every_boundary,
    } = request;
    let executions = supervisor
        .management()
        .executions()
        .await
        .map_err(|error| error.to_string())?;
    if cancel.is_cancelled() {
        return Ok(None);
    }
    let execution = match executions.as_slice() {
        [execution] => execution,
        [] => return Err("supervisor advertised no execution".to_owned()),
        _ => {
            return Err(format!(
                "supervisor advertised {} executions; run selection would be ambiguous",
                executions.len()
            ));
        }
    };
    if execution.execution_id.is_empty() || execution.timeline_id.is_empty() {
        return Err("supervisor advertised an execution without a fenced identity".to_owned());
    }
    let viewport_model = model.clone();
    let scene = Scene::new(model).map_err(|error| error.to_string())?;
    let robot_bundle_identity = bundle.robot_id.clone();
    let provenance = ProvenanceInput::new(robot_bundle_identity, run_id.to_owned())
        .map_err(|error| error.to_string())?;
    let native_bindings = provider.binding_evidence();
    let acquired = RemoteSceneRun::acquire(
        scene,
        supervisor.simulation(),
        provider,
        execution.execution_id.clone(),
        provenance,
    )
    .await;
    let mut run = match acquired {
        Ok(run) => run,
        Err(error) => {
            if matches!(
                &error,
                crate::remote::RemoteSceneError::AcquisitionUnconfirmed(_)
                    | crate::remote::RemoteSceneError::CleanupUnconfirmed { .. }
            ) {
                stopping(&display);
                if let Some(display) = &display
                    && let Ok(mut state) = display.lock()
                {
                    state.cleanup_failed = true;
                }
            }
            return Err(error.to_string());
        }
    };
    // Do not abandon an in-flight authority acquisition. Once acknowledged,
    // release it explicitly before classifying cancellation as complete.
    let outcome = async {
        if cancel.is_cancelled() {
            return Ok(None);
        }
        let snapshots = match crate::execution::drive(
            &mut run,
            &viewport_model,
            requested_steps,
            desktop,
            auto_run,
            collect_every_boundary,
        )
        .await
        {
            Ok(snapshots) => snapshots,
            Err(error) => {
                return Err(error);
            }
        };
        if let Some(display) = &display
            && let Ok(mut state) = display.lock()
        {
            state.phase = "Stopping";
            state.stopping = true;
        }
        let joint_name = format!("{}__base_freejoint", bundle.robot_id);
        let joint = viewport_model
            .joint(&joint_name)
            .map_err(|error| error.to_string())?
            .ok_or_else(|| format!("composed robot has no root joint `{joint_name}`"))?;
        let joint = viewport_model
            .joint_info(joint)
            .map_err(|error| error.to_string())?;
        let body = viewport_model.inner_arc().jnt_bodyid()[joint.handle.index()] as usize;
        let native_body = snapshots
            .iter()
            .map(|snapshot| native_body_sample(snapshot, body))
            .collect::<Result<Vec<_>, _>>()?;
        let provenance = run.provenance().clone();
        let completed_steps = run.boundary();
        let timeline_id = run
            .timeline_id()
            .ok_or_else(|| "native run has no current timeline identity".to_owned())?
            .to_owned();
        let model_identity = run.provenance().model_identity.clone();
        let quantum_ns = run.provenance().quantum_ns;
        Ok(Some(TerminalEvidence::V0 {
            native_bindings,
            provenance,
            provider_contract_verified: true,
            outcome: if completed_steps == requested_steps {
                "success"
            } else {
                "stopped"
            }
            .to_owned(),
            completed_steps,
            requested_steps,
            presentation: presentation.label().to_owned(),
            model_identity,
            quantum_ns,
            execution_id: execution.execution_id.clone(),
            timeline_id,
            native_body,
        }))
    }
    .await;
    // Every path after acknowledged acquisition attempts remote release, including
    // cancellation, drive failure and terminal evidence construction failure.
    stopping(&display);
    let release = run
        .release()
        .await
        .map(|_| ())
        .map_err(|error| error.to_string());
    if release.is_err() {
        run.mark_application_lost();
    }
    cleanup_outcome(outcome, release, &display, "native authority")
}

#[derive(Debug, Serialize)]
pub(super) struct NativeBodySample {
    pub(super) boundary: u64,
    pub(super) position_m: [f64; 3],
    pub(super) orientation_wxyz: [f64; 4],
    pub(super) linear_velocity_mps: [f64; 3],
    pub(super) angular_velocity_radps: [f64; 3],
}

fn native_body_sample(
    snapshot: &crate::mujoco::StateSnapshot,
    body: usize,
) -> Result<NativeBodySample, String> {
    let position_m = *snapshot
        .body_positions()
        .get(body)
        .ok_or_else(|| "root body position is outside native snapshot".to_owned())?;
    let orientation_wxyz = *snapshot
        .body_orientations()
        .get(body)
        .ok_or_else(|| "root body orientation is outside native snapshot".to_owned())?;
    let velocity = snapshot
        .body_velocities()
        .get(body)
        .ok_or_else(|| "root body velocity is outside native snapshot".to_owned())?;
    let angular_velocity_radps = [velocity[0], velocity[1], velocity[2]];
    let linear_velocity_mps = [velocity[3], velocity[4], velocity[5]];
    Ok(NativeBodySample {
        boundary: snapshot.boundary(),
        position_m,
        orientation_wxyz,
        linear_velocity_mps,
        angular_velocity_radps,
    })
}

#[derive(Debug, Serialize)]
#[serde(tag = "schema")]
pub(super) enum TerminalEvidence {
    /// The first simulator terminal evidence generation.
    ///
    /// Carries the artifact [`crate::remote::SimulationProvenance`] and the
    /// selected presentation label as runtime diagnostics alongside the
    /// schema-tagged terminal record. Downstream readers may deserialize a
    /// subset of these fields; serde will ignore unknown members by default.
    #[serde(rename = "phoxal/simulation-run/v0")]
    V0 {
        /// Native binding closure recorded for the run.
        native_bindings: serde_json::Value,
        /// Simulation provenance recorded by the runner.
        provenance: crate::remote::SimulationProvenance,
        /// Whether the simulator's provider contract was independently verified.
        provider_contract_verified: bool,
        /// `success` or `stopped`.
        outcome: String,
        /// Completed native transitions.
        completed_steps: u64,
        /// Requested native transitions.
        requested_steps: u64,
        /// Selected presentation (`headless` or `desktop`).
        presentation: String,
        /// Native model identity.
        model_identity: String,
        /// Native quantum in nanoseconds.
        quantum_ns: u64,
        /// Supervisor execution identity.
        execution_id: String,
        /// Controlled timeline identity.
        timeline_id: String,
        /// Every boundary for headless/scenario evidence, otherwise only the current state.
        native_body: Vec<NativeBodySample>,
    },
}

#[cfg(test)]
mod body_evidence_tests {
    #[test]
    fn cleanup_preserves_primary_failure_and_fences_restart_even_after_cancellation() {
        let display = Some(std::sync::Arc::new(std::sync::Mutex::new(
            crate::desktop::DisplayState::default(),
        )));
        super::stopping(&display);
        let result = super::cleanup_outcome::<()>(
            Err("selection refused".into()),
            Err("transport closed".into()),
            &display,
            "public session",
        );
        assert_eq!(
            result,
            Err("selection refused; public session cleanup unconfirmed: transport closed".into())
        );
        let state = display.as_ref().unwrap().lock().unwrap();
        assert!(state.stopping);
        assert!(state.cleanup_failed);
        drop(state);
        let result = super::cleanup_outcome(
            Ok(None::<()>),
            Err("release identity rejected".into()),
            &display,
            "native authority",
        );
        assert_eq!(
            result,
            Err("native authority cleanup unconfirmed: release identity rejected".into())
        );
        assert!(display.as_ref().unwrap().lock().unwrap().cleanup_failed);
        assert_eq!(
            super::cleanup_outcome(Ok(None::<()>), Ok(()), &None, "public session"),
            Ok(None)
        );
    }

    #[test]
    fn native_body_origin_velocities_use_world_axes_and_not_inertial_center() {
        let model = crate::mujoco::Model::from_xml(r#"<mujoco><worldbody><body name="root"><freejoint name="free"/><inertial pos="0.3 0.2 0.1" mass="2" diaginertia="1 1 1"/><geom size="0.1"/></body></worldbody></mujoco>"#).unwrap();
        let mut world = crate::mujoco::Workspace::new(&model).unwrap();
        let h = std::f64::consts::FRAC_1_SQRT_2;
        world.set_qpos(&[1.0, 2.0, 3.0, h, h, 0.0, 0.0]).unwrap();
        world.set_qvel(&[0.4, 0.5, 0.6, 0.0, 2.0, 0.0]).unwrap();
        world.forward().unwrap();
        let state = world.snapshot().unwrap();
        let sample = super::native_body_sample(&state, 1).unwrap();
        assert_eq!(sample.position_m, [1.0, 2.0, 3.0]);
        for (actual, expected) in sample.angular_velocity_radps.iter().zip([0.0, 0.0, 2.0]) {
            assert!((actual - expected).abs() < 1e-12);
        }
        for (actual, expected) in sample.linear_velocity_mps.iter().zip([0.4, 0.5, 0.6]) {
            assert!((actual - expected).abs() < 1e-12);
        }
        assert_ne!(sample.angular_velocity_radps, state.qvel()[3..6]);
        // A COM-centered query would include omega cross the nonzero inertial offset.
        assert!((sample.linear_velocity_mps[0] - 0.4).abs() < 1e-12);
    }
}
