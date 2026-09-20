use crate::bindings::ProbeFacts;
use crate::bindings::build_provider;
use crate::bindings::probe_contract;
use crate::bundle::BundleFacts;
use crate::composition::load_composed_model;
use crate::config::Options;
use crate::config::Presentation;
use crate::mujoco::Model;
use crate::mujoco::PhysicsQuantum;
use crate::mujoco::Scene;
use crate::native_provider::ComponentProvider;
use crate::remote::ProvenanceInput;
use crate::remote::RemoteSceneRun;
use crate::remote::SIMULATION_PROTOCOL;
use crate::remote::quantum_nanoseconds;
use serde::Serialize;

pub(super) const CONTROL_PRINCIPAL: &str = "simulator";

pub(super) async fn run(
    options: Options,
    desktop: Option<crate::desktop::Worker>,
) -> Result<(), String> {
    let bundle = BundleFacts::load(&options.bundle)?;
    let model = load_composed_model(&options.scene, &bundle)?;
    let quantum_ns = quantum_nanoseconds(
        PhysicsQuantum::from_seconds(model.timestep()).map_err(|error| error.to_string())?,
    )
    .map_err(|error| error.to_string())?;

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
        return Ok(());
    }

    let simulation = bundle
        .simulation
        .as_ref()
        .ok_or_else(|| "run bundle has no phoxal.simulation.v1 contract".to_owned())?;
    if simulation.protocol != SIMULATION_PROTOCOL || simulation.mode != "controlled" {
        return Err(format!(
            "bundle simulation contract is {} / {}, expected {} / controlled",
            simulation.protocol, simulation.mode, SIMULATION_PROTOCOL
        ));
    }
    if simulation.model_identity != model.identity().to_hex() {
        return Err(format!(
            "bundle model identity {} does not match composed native model {}",
            simulation.model_identity,
            model.identity().to_hex()
        ));
    }
    if simulation.quantum_ns != quantum_ns {
        return Err(format!(
            "bundle quantum {}ns does not match composed native quantum {}ns",
            simulation.quantum_ns, quantum_ns
        ));
    }

    let requested_steps = options
        .bound
        .ok_or_else(|| "a run requires exactly one of --steps or --duration".to_owned())?
        .steps(quantum_ns)?;
    let provider = build_provider(&bundle, &model, simulation)?;
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
    let connection = phoxal::session::connect(config)
        .await
        .map_err(|error| error.to_string())?;
    let supervisor = match connection.supervisor(supervisor_id).await {
        Ok(supervisor) => supervisor,
        Err(error) => {
            let _ = connection.close().await;
            return Err(error.to_string());
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
        },
        desktop,
    )
    .await;
    drop(supervisor);
    let close_result = connection.close().await;
    match (outcome, close_result) {
        (Ok(evidence), Ok(())) => {
            println!(
                "{}",
                serde_json::to_string(&evidence).map_err(|error| error.to_string())?
            );
            Ok(())
        }
        (Ok(_), Err(error)) => Err(format!("public session cleanup failed: {error}")),
        (Err(error), Ok(())) => Err(error),
        (Err(error), Err(close_error)) => Err(format!(
            "{error}; public session cleanup failed: {close_error}"
        )),
    }
}

struct RunRequest<'a> {
    run_id: &'a str,
    requested_steps: u64,
    presentation: Presentation,
    auto_run: bool,
}

async fn execute_remote_run(
    supervisor: &phoxal::session::Supervisor,
    bundle: &BundleFacts,
    model: Model,
    provider: ComponentProvider,
    request: RunRequest<'_>,
    desktop: Option<crate::desktop::Worker>,
) -> Result<TerminalEvidence, String> {
    let RunRequest {
        run_id,
        requested_steps,
        presentation,
        auto_run,
    } = request;
    let executions = supervisor
        .management()
        .executions()
        .await
        .map_err(|error| error.to_string())?;
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
    let robot_bundle_identity = bundle.provenance.source_closure_sha256.clone();
    let provenance = ProvenanceInput::new(robot_bundle_identity, run_id.to_owned())
        .map_err(|error| error.to_string())?;
    let native_bindings = provider.binding_evidence();
    let mut run = RemoteSceneRun::acquire(
        scene,
        supervisor.simulation(),
        provider,
        execution.execution_id.clone(),
        provenance,
    )
    .await
    .map_err(|error| error.to_string())?;
    let snapshots = match crate::execution::drive(
        &mut run,
        &viewport_model,
        requested_steps,
        desktop,
        auto_run,
    )
    .await
    {
        Ok(snapshots) => snapshots,
        Err(error) => {
            run.mark_application_lost();
            return Err(error);
        }
    };
    let joint_name = format!("{}__base_freejoint", bundle.manifest.robot_id);
    let joint = viewport_model
        .joint(&joint_name)
        .map_err(|error| error.to_string())?
        .ok_or_else(|| format!("composed robot has no root joint `{joint_name}`"))?;
    let joint = viewport_model
        .joint_info(joint)
        .map_err(|error| error.to_string())?;
    let native_body = snapshots
        .iter()
        .map(|snapshot| native_body_sample(snapshot, joint))
        .collect::<Result<Vec<_>, _>>()?;
    let provenance = run.provenance().clone();
    let completed_steps = run.boundary();
    let timeline_id = run
        .timeline_id()
        .ok_or_else(|| "native run has no current timeline identity".to_owned())?
        .to_owned();
    let model_identity = run.provenance().model_identity.clone();
    let quantum_ns = run.provenance().quantum_ns;
    run.release().await.map_err(|error| error.to_string())?;
    Ok(TerminalEvidence {
        schema: "phoxal/simulation-run/v0".to_owned(),
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
    })
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
    joint: crate::mujoco::JointInfo,
) -> Result<NativeBodySample, String> {
    let position_m = snapshot
        .qpos()
        .get(joint.qpos_offset..joint.qpos_offset + 3)
        .and_then(|values| values.try_into().ok())
        .ok_or_else(|| "root free joint position is outside qpos".to_owned())?;
    let orientation_wxyz = snapshot
        .qpos()
        .get(joint.qpos_offset + 3..joint.qpos_offset + 7)
        .and_then(|values| values.try_into().ok())
        .ok_or_else(|| "root free joint orientation is outside qpos".to_owned())?;
    let linear_velocity_mps = snapshot
        .qvel()
        .get(joint.dof_offset..joint.dof_offset + 3)
        .and_then(|values| values.try_into().ok())
        .ok_or_else(|| "root free joint linear velocity is outside qvel".to_owned())?;
    let angular_velocity_radps = snapshot
        .qvel()
        .get(joint.dof_offset + 3..joint.dof_offset + 6)
        .and_then(|values| values.try_into().ok())
        .ok_or_else(|| "root free joint angular velocity is outside qvel".to_owned())?;
    Ok(NativeBodySample {
        boundary: snapshot.boundary(),
        position_m,
        orientation_wxyz,
        linear_velocity_mps,
        angular_velocity_radps,
    })
}

#[derive(Debug, Serialize)]
pub(super) struct TerminalEvidence {
    pub(super) native_bindings: serde_json::Value,
    pub(super) provenance: crate::remote::SimulationProvenance,
    pub(super) schema: String,
    pub(super) provider_contract_verified: bool,
    pub(super) outcome: String,
    pub(super) completed_steps: u64,
    pub(super) requested_steps: u64,
    pub(super) presentation: String,
    pub(super) model_identity: String,
    pub(super) quantum_ns: u64,
    pub(super) execution_id: String,
    pub(super) timeline_id: String,
    pub(super) native_body: Vec<NativeBodySample>,
}
