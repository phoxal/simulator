use crate::bundle::BundleFacts;
use crate::composition::native_component_prefix;
#[cfg(feature = "rendering")]
use crate::georeference::georeference;
use crate::native_provider::Cadence;
#[cfg(feature = "rendering")]
use crate::remote::ProviderSet;
#[cfg(feature = "rendering")]
use crate::remote::SIMULATION_PROTOCOL;
#[cfg(feature = "rendering")]
use crate::remote::quantum_nanoseconds;
#[cfg(feature = "rendering")]
use phoxal::communication::simulation::ProviderRequirement;
use crate::mujoco::Model;
#[cfg(feature = "rendering")]
use crate::mujoco::PhysicsQuantum;
use phoxal_artifact_format::artifact::OutputKind;
use phoxal_artifact_format::artifact::PortKind;
#[cfg(feature = "rendering")]
use phoxal_artifact_format::bundle::BundleActuationBinding;
#[cfg(feature = "rendering")]
use phoxal_artifact_format::bundle::BundleSimulation;
#[cfg(feature = "rendering")]
use phoxal_artifact_format::bundle::BundleSimulationProvider;
use phoxal_artifact_format::document::CapabilityDeclaration;
use phoxal_artifact_format::document::NativeTargetKind;
use prost::Name;
use serde::Serialize;
#[cfg(feature = "rendering")]
use std::collections::BTreeMap;
use std::collections::BTreeSet;

pub(super) struct ProbeContract {
    pub(super) providers: Vec<ProbeProvider>,
    pub(super) actuation_bindings: Vec<ProbeActuation>,
}

#[derive(Debug, Clone, Serialize)]
pub(super) struct ProbeProvider {
    pub(super) rate_microhertz: u64,
    pub(super) service_instance: String,
    pub(super) port: String,
    pub(super) kind: PortKind,
    pub(super) input_fqn: String,
    pub(super) payload_fqn: String,
}

#[derive(Debug, Clone, Serialize)]
pub(super) struct ProbeActuation {
    pub(super) service_instance: String,
    pub(super) port: String,
    pub(super) payload_fqn: String,
    pub(super) actuator_ids: Vec<String>,
}

#[derive(Debug, Clone, Serialize)]
pub(super) struct ProbeFacts {
    pub(super) model_identity: String,
    pub(super) quantum_ns: u64,
    pub(super) providers: Vec<ProbeProvider>,
    pub(super) actuation_bindings: Vec<ProbeActuation>,
}

pub(super) fn output_observation_kind(kind: OutputKind) -> Option<PortKind> {
    Some(match kind {
        OutputKind::State => PortKind::State,
        OutputKind::Sample => PortKind::Sample,
        OutputKind::Event => PortKind::Event,
        OutputKind::Stream => PortKind::Stream,
        _ => return None,
    })
}

pub(super) fn probe_contract(bundle: &BundleFacts, model: &Model) -> Result<ProbeContract, String> {
    let providers = generated_provider_facts(bundle)?;
    let actuation_bindings = generated_actuation_facts(bundle, model)?;
    #[cfg(feature = "rendering")]
    {
        let simulation = BundleSimulation {
            protocol: SIMULATION_PROTOCOL.to_owned(),
            mode: "controlled".to_owned(),
            model_identity: model.identity().to_hex(),
            quantum_ns: quantum_nanoseconds(
                PhysicsQuantum::from_seconds(model.timestep()).map_err(|error| error.to_string())?,
            )
            .map_err(|error| error.to_string())?,
            providers: providers
                .iter()
                .map(|provider| BundleSimulationProvider {
                    rate_microhertz: provider.rate_microhertz,
                    service_instance: provider.service_instance.clone(),
                    port: provider.port.clone(),
                    kind: provider.kind,
                    input_fqn: provider.input_fqn.clone(),
                    payload_fqn: provider.payload_fqn.clone(),
                    service_fqn: String::new(),
                    method: String::new(),
                    max_message_bytes: 0,
                    max_buffered_items: 0,
                })
                .collect(),
            actuation_bindings: actuation_bindings
                .iter()
                .map(|binding| BundleActuationBinding {
                    service_instance: binding.service_instance.clone(),
                    port: binding.port.clone(),
                    payload_fqn: binding.payload_fqn.clone(),
                    actuator_ids: binding.actuator_ids.clone(),
                })
                .collect(),
        };
        let _provider = build_provider(bundle, model, &simulation)?;
    }
    Ok(ProbeContract {
        providers,
        actuation_bindings,
    })
}

pub(super) fn generated_provider_facts(bundle: &BundleFacts) -> Result<Vec<ProbeProvider>, String> {
    let driver_instances = bundle
        .manifest
        .document
        .robot
        .components
        .iter()
        .filter_map(|(instance, selection)| selection.driver.as_ref().map(|_| instance.as_str()))
        .collect::<BTreeSet<_>>();
    let mut providers = Vec::new();
    for executable in &bundle.manifest.executables {
        if executable.role != "driver" || !driver_instances.contains(executable.instance.as_str()) {
            continue;
        }
        let artifact = executable.artifact.as_ref().ok_or_else(|| {
            format!(
                "selected driver {} has no retained generated artifact contract",
                executable.instance
            )
        })?;
        for output in artifact
            .runtime
            .transient_outputs
            .iter()
            .chain(artifact.runtime.service_outputs.iter())
        {
            let Some(kind) = output_observation_kind(output.kind) else {
                continue;
            };
            let port = output.port.as_ref().ok_or_else(|| {
                format!(
                    "observation output on {} has no generated port",
                    executable.instance
                )
            })?;
            let signature = output.signature.as_ref().ok_or_else(|| {
                format!(
                    "observation output {}/{} has no generated signature",
                    executable.instance, port
                )
            })?;
            let component = bundle
                .manifest
                .components
                .iter()
                .find(|component| component.instance == executable.instance)
                .ok_or_else(|| format!("provider {} has no component", executable.instance))?;
            let capability = component.definition.capabilities.get(port).ok_or_else(|| {
                format!(
                    "provider {}/{} has no declared capability",
                    executable.instance, port
                )
            })?;
            let rate = semantic_number(capability, "publish_rate_hz", &executable.instance)?;
            let cadence = Cadence::new(rate, 1).map_err(|error| error.to_string())?;
            providers.push(ProbeProvider {
                rate_microhertz: cadence.rate_microhertz(),
                service_instance: executable.instance.clone(),
                port: port.clone(),
                kind,
                input_fqn: signature.request.clone(),
                payload_fqn: signature.response.clone(),
            });
        }
    }
    let mut provider_keys = BTreeSet::new();
    for provider in &providers {
        if provider.input_fqn.is_empty() {
            return Err(format!(
                "generated observation {}/{} has no request message identity",
                provider.service_instance, provider.port
            ));
        }
        if !provider_keys.insert((provider.service_instance.as_str(), provider.port.as_str())) {
            return Err(format!(
                "generated observation {}/{} is duplicated",
                provider.service_instance, provider.port
            ));
        }
    }
    providers.sort_by(|left, right| {
        left.service_instance
            .cmp(&right.service_instance)
            .then_with(|| left.port.cmp(&right.port))
    });
    if providers.is_empty() {
        return Err(
            "simulation requires at least one selected physical driver provider".to_owned(),
        );
    }
    Ok(providers)
}

pub(super) fn generated_actuation_facts(
    bundle: &BundleFacts,
    model: &Model,
) -> Result<Vec<ProbeActuation>, String> {
    let mut targets = Vec::new();
    for (instance, selection) in &bundle.manifest.document.robot.components {
        if selection.driver.is_none() {
            continue;
        }
        let component = bundle
            .manifest
            .components
            .iter()
            .find(|component| component.instance == *instance)
            .ok_or_else(|| format!("component {instance} has no resolved bundle record"))?;
        for (capability_name, capability) in &component.definition.capabilities {
            if capability.kind != "motor" {
                continue;
            }
            if capability.target.kind != NativeTargetKind::Actuator {
                return Err(format!(
                    "component {instance} motor target must be an actuator, got {:?}",
                    capability.target.kind
                ));
            }
            let native_name = format!(
                "{}{capability_target}",
                native_component_prefix(bundle, instance),
                capability_target = capability.target.id
            );
            model
                .bind_actuator(phoxal_service_motion::ports::ACTUATORS, &native_name)
                .map_err(|error| format!("actuator binding {native_name}: {error}"))?;
            targets.push(format!("{instance}.{capability_name}"));
        }
    }
    targets.sort();
    targets.dedup();
    if targets.is_empty() {
        return Err(
            "the native probe cannot invent actuator membership: no selected motor capability has an explicit composed actuator name"
                .to_owned(),
        );
    }
    let mut outputs = Vec::new();
    for executable in &bundle.manifest.executables {
        let Some(artifact) = &executable.artifact else {
            continue;
        };
        for output in artifact
            .runtime
            .transient_outputs
            .iter()
            .chain(artifact.runtime.service_outputs.iter())
        {
            if output.kind != OutputKind::Setpoint {
                continue;
            }
            let port = output.port.as_deref().ok_or_else(|| {
                format!(
                    "setpoint output on {} has no generated port",
                    executable.instance
                )
            })?;
            let signature = output.signature.as_ref().ok_or_else(|| {
                format!(
                    "setpoint output {}/{} has no generated signature",
                    executable.instance, port
                )
            })?;
            if port != phoxal_service_motion::ports::ACTUATORS.name()
                || signature.response != phoxal_service_motion::ActuatorSetpoint::full_name()
            {
                // Intermediate service intents are ordinary graph traffic.
                // Only native actuator products belong in the physics input cut.
                continue;
            }
            outputs.push(ProbeActuation {
                service_instance: executable.instance.clone(),
                port: port.to_owned(),
                payload_fqn: signature.response.clone(),
                actuator_ids: targets.clone(),
            });
        }
    }
    if outputs.is_empty() {
        return Err(
            "the native probe found no generated motion actuator setpoint output; refusing an invented control route"
                .to_owned(),
        );
    }
    if outputs.len() > 1 {
        return Err(
            "multiple motion actuator outputs would make native membership ambiguous".to_owned(),
        );
    }
    Ok(outputs)
}

#[cfg(feature = "rendering")]
pub(super) fn build_provider(
    bundle: &BundleFacts,
    model: &Model,
    simulation: &BundleSimulation,
) -> Result<ComponentProvider, String> {
    let requirements = simulation
        .providers
        .iter()
        .map(|provider| {
            Ok(ProviderRequirement {
                rate_microhertz: provider.rate_microhertz,
                service_instance: provider.service_instance.clone(),
                port: provider.port.clone(),
                payload_fqn: provider.payload_fqn.clone(),
                kind: match provider.kind {
                    PortKind::State => phoxal::communication::session::PortKind::State,
                    PortKind::Sample => phoxal::communication::session::PortKind::Sample,
                    PortKind::Event => phoxal::communication::session::PortKind::Event,
                    PortKind::Stream => phoxal::communication::session::PortKind::Stream,
                    PortKind::Setpoint => phoxal::communication::session::PortKind::Setpoint,
                    PortKind::Read => phoxal::communication::session::PortKind::Read,
                    PortKind::Commands => phoxal::communication::session::PortKind::Commands,
                } as i32,
                input_fqn: provider.input_fqn.clone(),
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    let providers = ProviderSet::new(requirements).map_err(|error| error.to_string())?;
    let observations = observation_bindings(bundle, model, &providers)?;
    let actuations = simulation
        .actuation_bindings
        .iter()
        .map(|binding| {
            if binding.port != phoxal_service_motion::ports::ACTUATORS.name()
                || binding.payload_fqn != phoxal_service_motion::ActuatorSetpoint::full_name()
            {
                return Err(format!(
                    "simulation actuation {}/{} does not use generated motion constants",
                    binding.service_instance, binding.port
                ));
            }
            let targets = binding
                .actuator_ids
                .iter()
                .map(|actuator_id| {
                    let (instance, capability_name) =
                        actuator_id.split_once('.').ok_or_else(|| {
                            format!("actuator {actuator_id} must identify component.capability")
                        })?;
                    let component = bundle
                        .manifest
                        .components
                        .iter()
                        .find(|c| c.instance == instance)
                        .ok_or_else(|| format!("actuator {actuator_id} has no component"))?;
                    let capability = component
                        .definition
                        .capabilities
                        .get(capability_name)
                        .filter(|c| {
                            c.kind == "motor" && c.target.kind == NativeTargetKind::Actuator
                        })
                        .ok_or_else(|| format!("actuator {actuator_id} has no motor capability"))?;
                    let native_name = format!(
                        "{}{}",
                        native_component_prefix(bundle, instance),
                        capability.target.id
                    );
                    let native = model
                        .bind_actuator(phoxal_service_motion::ports::ACTUATORS, &native_name)
                        .map_err(|error| format!("actuator binding {actuator_id}: {error}"))?;
                    let mode = match native.info.mode {
                        crate::mujoco::ActuatorMode::Torque => NativeControlMode::Torque,
                        crate::mujoco::ActuatorMode::Velocity => NativeControlMode::Velocity,
                        crate::mujoco::ActuatorMode::Unsupported => {
                            return Err(format!(
                                "actuator {actuator_id} has unsupported native control semantics"
                            ));
                        }
                    };
                    Ok(ActuatorTarget::new(actuator_id.clone(), native_name, mode))
                })
                .collect::<Result<Vec<_>, String>>()?;
            Ok(ActuationDeclaration::motion(
                binding.service_instance.clone(),
                targets,
            ))
        })
        .collect::<Result<Vec<_>, String>>()?;
    let cadence = providers
        .requirements()
        .iter()
        .map(|requirement| {
            let component = bundle
                .manifest
                .components
                .iter()
                .find(|component| component.instance == requirement.service_instance)
                .ok_or_else(|| {
                    format!("provider {} has no component", requirement.service_instance)
                })?;
            let capability = component
                .definition
                .capabilities
                .get(&requirement.port)
                .ok_or_else(|| {
                    format!("provider {} has no cadence capability", requirement.port)
                })?;
            let rate =
                semantic_number(capability, "publish_rate_hz", &requirement.service_instance)?;
            let cadence = Cadence::new(rate, simulation.quantum_ns).map_err(|e| e.to_string())?;
            if cadence.rate_microhertz() != requirement.rate_microhertz {
                return Err(format!(
                    "provider {}.{} rate differs from its admitted cadence",
                    requirement.service_instance, requirement.port
                ));
            }
            Ok((
                (
                    requirement.service_instance.clone(),
                    requirement.port.clone(),
                ),
                cadence,
            ))
        })
        .collect::<Result<BTreeMap<_, _>, String>>()?;
    ComponentProvider::new(model, providers, observations, actuations, cadence)
        .map_err(|error| error.to_string())
}

#[cfg(feature = "rendering")]
pub(super) fn observation_bindings(
    bundle: &BundleFacts,
    model: &Model,
    providers: &ProviderSet,
) -> Result<Vec<ObservationBinding>, String> {
    let mut bindings = Vec::new();
    for (instance, component) in &bundle.manifest.document.robot.components {
        if component.driver.is_none() {
            continue;
        }
        let resolved = bundle
            .manifest
            .components
            .iter()
            .find(|candidate| candidate.instance == *instance)
            .ok_or_else(|| format!("component {instance} has no resolved bundle record"))?;
        let routes = providers
            .requirements()
            .iter()
            .filter(|requirement| requirement.service_instance == *instance)
            .map(|requirement| requirement.port.as_str())
            .collect::<BTreeSet<_>>();
        if routes.is_empty() {
            return Err(format!(
                "selected driver {instance} has no explicit provider routes"
            ));
        }
        let prefix = native_component_prefix(bundle, instance);
        let mut handled = BTreeSet::new();
        let mut requirements = providers
            .requirements()
            .iter()
            .filter(|requirement| requirement.service_instance == *instance)
            .collect::<Vec<_>>();
        // A fused IMU binds the authored raw signals too. Resolve it before
        // validating the optional raw output routes, independent of map ordering.
        requirements.sort_by_key(|requirement| {
            resolved
                .definition
                .capabilities
                .get(&requirement.port)
                .is_none_or(|capability| capability.kind != "imu")
        });
        for requirement in requirements {
            if handled.contains(requirement.port.as_str()) {
                continue;
            }
            let capability = resolved
                .definition
                .capabilities
                .get(&requirement.port)
                .ok_or_else(|| {
                    format!(
                        "component {instance} capability {:?} is missing from authored definition",
                        requirement.port
                    )
                })?;
            let native_target = format!("{prefix}{}", capability.target.id);
            match capability.kind.as_str() {
                "imu" => {
                    let acceleration = resolved
                        .definition
                        .capabilities
                        .get("accelerometer")
                        .ok_or_else(|| {
                            format!("component {instance} IMU has no accelerometer capability")
                        })?;
                    let gyroscope = resolved
                        .definition
                        .capabilities
                        .get("gyroscope")
                        .ok_or_else(|| {
                            format!("component {instance} IMU has no gyroscope capability")
                        })?;
                    let orientation_sensor =
                        capability_signal(capability, "orientation", instance)?;
                    let accelerometer_sensor =
                        capability_signal(acceleration, "acceleration", instance)?;
                    let gyroscope_sensor =
                        capability_signal(gyroscope, "angular_velocity", instance)?;
                    let sensor_frame = native_target.clone();
                    if capability.target.kind != NativeTargetKind::Site
                        || acceleration.target.kind != NativeTargetKind::Site
                        || gyroscope.target.kind != NativeTargetKind::Site
                    {
                        return Err(format!(
                            "component {instance} inertial capabilities must target sites"
                        ));
                    }
                    if acceleration.target.id != capability.target.id
                        || gyroscope.target.id != capability.target.id
                    {
                        return Err(format!(
                            "component {instance} inertial capabilities must share one target site"
                        ));
                    }
                    let binding = ObservationBinding::semantic_imu(
                        instance,
                        native_target,
                        sensor_frame,
                        format!("{prefix}{orientation_sensor}"),
                        format!("{prefix}{accelerometer_sensor}"),
                        format!("{prefix}{gyroscope_sensor}"),
                        &requirement.payload_fqn,
                    )
                    .map_err(|error| error.to_string())?;
                    bindings.push(binding);
                    handled.extend(["imu", "accelerometer", "gyroscope"]);
                }
                "encoder" => {
                    if capability.target.kind != NativeTargetKind::Joint {
                        return Err(format!(
                            "component {instance} encoder target must be a joint"
                        ));
                    }
                    let binding = ObservationBinding::semantic_encoder(
                        instance,
                        format!("{prefix}{}", capability.target.id),
                        &requirement.payload_fqn,
                    )
                    .map_err(|error| error.to_string())?;
                    bindings.push(binding);
                    handled.insert(requirement.port.as_str());
                }
                "camera" => {
                    validate_camera_capability(capability, model, &native_target, instance)?;
                    if capability.target.kind != NativeTargetKind::Camera {
                        return Err(format!(
                            "component {instance} camera target must be a camera"
                        ));
                    }
                    let mode = capability
                        .semantics
                        .get("mode")
                        .and_then(serde_json::Value::as_str)
                        .ok_or_else(|| {
                            format!("component {instance} camera has no authored mode")
                        })?;
                    let binding = ObservationBinding::semantic_camera(
                        instance,
                        &requirement.port,
                        mode,
                        native_target,
                        &requirement.payload_fqn,
                    )
                    .map_err(|error| error.to_string())?;
                    bindings.push(binding);
                    handled.insert(requirement.port.as_str());
                }
                "depth" => {
                    validate_camera_capability(capability, model, &native_target, instance)?;
                    if capability.target.kind != NativeTargetKind::Camera {
                        return Err(format!(
                            "component {instance} depth capability must target a camera"
                        ));
                    }
                    bindings.push(
                        ObservationBinding::semantic_depth(
                            instance,
                            native_target,
                            &requirement.payload_fqn,
                            [
                                semantic_number(capability, "min_range_m", instance)?,
                                semantic_number(capability, "max_range_m", instance)?,
                            ],
                        )
                        .map_err(|error| error.to_string())?,
                    );
                    handled.insert(requirement.port.as_str());
                }
                "range" => {
                    if capability.target.kind != NativeTargetKind::Site {
                        return Err(format!(
                            "component {instance} range capability must target a site"
                        ));
                    }
                    bindings.push(
                        ObservationBinding::semantic_range(
                            instance,
                            native_target,
                            semantic_number(capability, "min_range_m", instance)?,
                            semantic_number(capability, "max_range_m", instance)?,
                            semantic_number(capability, "field_of_view_rad", instance)?,
                            &requirement.payload_fqn,
                        )
                        .map_err(|error| error.to_string())?,
                    );
                    handled.insert(requirement.port.as_str());
                }
                "gnss" => {
                    if capability.target.kind != NativeTargetKind::Site {
                        return Err(format!(
                            "component {instance} GNSS capability must target a site"
                        ));
                    }
                    let georeference = georeference(model, instance)?;
                    bindings.push(
                        ObservationBinding::semantic_gnss(
                            instance,
                            native_target,
                            georeference,
                            &requirement.payload_fqn,
                        )
                        .map_err(|error| error.to_string())?,
                    );
                    handled.insert(requirement.port.as_str());
                }
                other => {
                    return Err(format!(
                        "component {instance} capability {} uses unsupported semantic kind {other:?}",
                        requirement.port
                    ));
                }
            }
        }
    }
    Ok(bindings)
}

#[cfg(feature = "rendering")]
pub(super) fn capability_signal(
    capability: &CapabilityDeclaration,
    role: &str,
    instance: &str,
) -> Result<String, String> {
    capability
        .signals
        .get(role)
        .cloned()
        .ok_or_else(|| format!("component {instance} capability has no {role} signal"))
}

pub(super) fn semantic_number(
    capability: &CapabilityDeclaration,
    key: &str,
    instance: &str,
) -> Result<f64, String> {
    let value = capability
        .semantics
        .get(key)
        .and_then(serde_json::Value::as_f64)
        .ok_or_else(|| format!("component {instance} capability is missing numeric {key}"))?;
    if !value.is_finite() {
        return Err(format!(
            "component {instance} capability {key} must be finite"
        ));
    }
    Ok(value)
}

#[cfg(feature = "rendering")]
fn validate_camera_capability(
    capability: &CapabilityDeclaration,
    model: &Model,
    native_target: &str,
    instance: &str,
) -> Result<(), String> {
    let camera = model
        .camera(native_target)
        .map_err(|e| e.to_string())?
        .ok_or_else(|| format!("component {instance} camera {native_target} is absent"))?;
    let camera = model.camera_info(camera).map_err(|e| e.to_string())?;
    let mut resolution = [0usize; 2];
    for (index, key) in ["width_px", "height_px"].iter().enumerate() {
        let value = capability
            .semantics
            .get(*key)
            .and_then(serde_json::Value::as_u64)
            .filter(|value| *value > 0)
            .ok_or_else(|| {
                format!("component {instance} camera {key} must be a positive integer")
            })?;
        resolution[index] = usize::try_from(value)
            .map_err(|_| format!("camera {key} exceeds addressable memory"))?;
    }
    let fov = semantic_number(capability, "field_of_view_rad", instance)?;
    // Existing authored MJCF uses hundredths of a degree. This agreement
    // tolerance is below half a pixel at the maintained 640-pixel resolution.
    if resolution != camera.resolution || (fov - camera.fovy_degrees.to_radians()).abs() > 0.001 {
        return Err(format!(
            "component {instance} camera resolution or vertical FOV disagrees with its compiled native camera"
        ));
    }
    let bytes_per_pixel = if capability.kind == "depth" {
        5
    } else if capability
        .semantics
        .get("mode")
        .and_then(serde_json::Value::as_str)
        == Some("mono")
    {
        1
    } else {
        3
    };
    let maximum_encoded_bytes = resolution[0]
        .checked_mul(resolution[1])
        .and_then(|pixels| pixels.checked_mul(bytes_per_pixel))
        .and_then(|bytes| bytes.checked_add(64));
    if maximum_encoded_bytes.is_none_or(|bytes| bytes > 4 * 1024 * 1024) {
        return Err(format!(
            "component {instance} camera exceeds the 4 MiB encoded product budget"
        ));
    }
    Ok(())
}
