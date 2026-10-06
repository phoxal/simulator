use crate::bundle::{BundleFacts, component_definition};
use crate::composition::native_component_prefix;
use crate::georeference::georeference;
use crate::mujoco::Model;
use crate::mujoco::PhysicsQuantum;
use crate::native_provider::ActuationDeclaration;
use crate::native_provider::ActuatorTarget;
use crate::native_provider::Cadence;
use crate::native_provider::ComponentProvider;
use crate::native_provider::NativeControlMode;
use crate::native_provider::ObservationBinding;
use crate::remote::ProviderSet;
use crate::remote::SIMULATION_PROTOCOL;
use crate::remote::quantum_nanoseconds;
use phoxal::artifact::bundle::InstanceRole;
use phoxal::artifact::bundle::{
    BundleActuationBinding, BundleSimulation, BundleSimulationProvider,
};
use phoxal::artifact::document::{CapabilityDeclaration, NativeTargetKind};
use phoxal::artifact::{MethodShape, OutputRecord, RuntimeRecord};
use phoxal::communication::simulation::ProviderRequirement;
use prost::Name;
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};

pub(super) struct ProbeContract {
    pub(super) providers: Vec<ProbeProvider>,
    pub(super) actuation_bindings: Vec<ProbeActuation>,
}

#[derive(Debug, Clone, Serialize)]
pub(super) struct ProbeProvider {
    pub(super) rate_microhertz: u64,
    pub(super) service_instance: String,
    pub(super) port: String,
    pub(super) shape: MethodShape,
    pub(super) service_fqn: String,
    pub(super) method: String,
    pub(super) retained_latest: bool,
    pub(super) lease_valid_for_ms: Option<u64>,
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

fn runtime_outputs(runtime: &RuntimeRecord) -> impl Iterator<Item = &OutputRecord> {
    let RuntimeRecord::V0 { outputs, .. } = runtime;
    outputs.iter()
}

pub(super) fn probe_contract(bundle: &BundleFacts, model: &Model) -> Result<ProbeContract, String> {
    let providers = generated_provider_facts(bundle)?;
    let actuation_bindings = generated_actuation_facts(bundle, model)?;
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
                shape: provider.shape,
                retained_latest: provider.retained_latest,
                lease_valid_for_ms: provider.lease_valid_for_ms,
                input_fqn: provider.input_fqn.clone(),
                payload_fqn: provider.payload_fqn.clone(),
                service_fqn: provider.service_fqn.clone(),
                method: provider.method.clone(),
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
    Ok(ProbeContract {
        providers,
        actuation_bindings,
    })
}

pub(super) fn simulation_definition(
    bundle: &BundleFacts,
    model: &Model,
) -> Result<BundleSimulation, String> {
    let contract = probe_contract(bundle, model)?;
    let providers = contract
        .providers
        .into_iter()
        .map(|provider| {
            let RuntimeRecord::V0 { outputs, .. } = bundle
                .runtime_record(&provider.service_instance)
                .ok_or_else(|| "native provider has no compiled runtime".to_owned())?;
            let output = outputs
                .iter()
                .find(|output| output.port.as_deref() == Some(&provider.port))
                .ok_or_else(|| "native provider has no compiled output".to_owned())?;
            let bytes = output
                .max_bytes
                .ok_or_else(|| "native provider has no byte bound".to_owned())?;
            let items = output
                .max_items
                .or_else(|| provider.retained_latest.then_some(1))
                .ok_or_else(|| "native provider has no item bound".to_owned())?;
            Ok(BundleSimulationProvider {
                rate_microhertz: provider.rate_microhertz,
                service_instance: provider.service_instance,
                port: provider.port,
                shape: provider.shape,
                retained_latest: provider.retained_latest,
                lease_valid_for_ms: provider.lease_valid_for_ms,
                input_fqn: provider.input_fqn,
                payload_fqn: provider.payload_fqn,
                service_fqn: provider.service_fqn,
                method: provider.method,
                max_message_bytes: u32::try_from(bytes)
                    .map_err(|_| "native byte bound exceeds u32")?,
                max_buffered_items: u32::try_from(items)
                    .map_err(|_| "native item bound exceeds u32")?,
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    Ok(BundleSimulation {
        protocol: SIMULATION_PROTOCOL.to_owned(),
        mode: "controlled".to_owned(),
        model_identity: model.identity().to_hex(),
        quantum_ns: quantum_nanoseconds(
            PhysicsQuantum::from_seconds(model.timestep()).map_err(|e| e.to_string())?,
        )
        .map_err(|e| e.to_string())?,
        providers,
        actuation_bindings: contract
            .actuation_bindings
            .into_iter()
            .map(|binding| BundleActuationBinding {
                service_instance: binding.service_instance,
                port: binding.port,
                payload_fqn: binding.payload_fqn,
                actuator_ids: binding.actuator_ids,
            })
            .collect(),
    })
}

pub(super) fn generated_provider_facts(bundle: &BundleFacts) -> Result<Vec<ProbeProvider>, String> {
    let mut providers = Vec::new();
    for (instance_id, instance) in &bundle.instances {
        let Some(record) = bundle.runtime_record(instance_id) else {
            continue;
        };
        if instance.role != InstanceRole::Driver
            || !bundle
                .components
                .get(instance_id)
                .is_some_and(|component| component.driver)
        {
            continue;
        }
        for output in runtime_outputs(record) {
            let Some(signature) = output
                .signature
                .as_ref()
                .filter(|signature| signature.shape == MethodShape::Observation)
            else {
                continue;
            };
            let port = output.port.as_ref().ok_or_else(|| {
                format!("observation output on {instance_id} has no generated port")
            })?;
            let component = bundle
                .components
                .get(instance_id)
                .ok_or_else(|| format!("provider {instance_id} has no component"))?;
            let capability = component_definition(&component.definition)
                .1
                .get(port.as_str())
                .ok_or_else(|| {
                    format!("provider {instance_id}/{port} has no declared capability")
                })?;
            let rate = semantic_number(capability, "publish_rate_hz", instance_id)?;
            let cadence = Cadence::new(rate, 1).map_err(|error| error.to_string())?;
            providers.push(ProbeProvider {
                rate_microhertz: cadence.rate_microhertz(),
                service_instance: instance_id.clone(),
                port: port.clone(),
                shape: signature.shape,
                service_fqn: signature.service.clone(),
                method: signature.method.clone(),
                retained_latest: signature.retained_latest,
                lease_valid_for_ms: signature.lease_valid_for_ms,
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
    let routes = phoxal::artifact::simulation_context::actuator_routes(&bundle.admitted)?;
    let mut outputs = Vec::new();
    for (source, targets) in routes {
        let RuntimeRecord::V0 {
            outputs: served, ..
        } = bundle
            .runtime_record(&source.instance)
            .ok_or_else(|| format!("native actuator source {source} is not authored"))?;
        let output = served
            .iter()
            .find(|output| output.port.as_deref() == Some(&source.endpoint))
            .ok_or_else(|| format!("native actuator source {source} has no compiled output"))?;
        let signature = output
            .signature
            .as_ref()
            .ok_or("native actuator output has no compiled signature")?;
        if signature.shape != MethodShape::Observation
            || !signature.lease_valid_for_ms.is_some_and(|lease| lease > 0)
            || signature.response
                != phoxal::contracts::component::actuator::ActuatorCommand::full_name()
        {
            return Err(format!(
                "native actuator source {source} is not a canonical actuator projection"
            ));
        }
        for target in &targets {
            let (instance, name) = target
                .split_once('.')
                .ok_or("invalid native actuator identity")?;
            let component = bundle
                .components
                .get(instance)
                .ok_or("native motor has no component")?;
            let capability = component_definition(&component.definition)
                .1
                .get(name)
                .ok_or("native motor capability absent")?;
            let native_name = format!(
                "{}{}",
                native_component_prefix(bundle, instance),
                capability.target.id
            );
            model
                .bind_actuator(crate::contract::simulator_api::ACTUATORS, &native_name)
                .map_err(|error| format!("actuator binding {native_name}: {error}"))?;
        }
        outputs.push(ProbeActuation {
            service_instance: source.instance,
            port: source.endpoint,
            payload_fqn: signature.response.clone(),
            actuator_ids: targets.into_iter().collect(),
        });
    }
    if outputs.is_empty() {
        return Err("native execution has no authored actuator input edges".into());
    }
    Ok(outputs)
}

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
                shape: phoxal::communication::session::MethodShape::Observation as i32,
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
            if binding.payload_fqn
                != phoxal::contracts::component::actuator::ActuatorCommand::full_name()
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
                        .components
                        .get(instance)
                        .ok_or_else(|| format!("actuator {actuator_id} has no component"))?;
                    let capability = component_definition(&component.definition)
                        .1
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
                        .bind_actuator(crate::contract::simulator_api::ACTUATORS, &native_name)
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
            Ok(ActuationDeclaration {
                service_instance: binding.service_instance.clone(),
                port: binding.port.clone(),
                payload_fqn: binding.payload_fqn.clone(),
                targets,
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    let cadence = providers
        .requirements()
        .iter()
        .map(|requirement| {
            let component = bundle
                .components
                .get(&requirement.service_instance)
                .ok_or_else(|| {
                    format!("provider {} has no component", requirement.service_instance)
                })?;
            let capability = component_definition(&component.definition)
                .1
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

pub(super) fn observation_bindings(
    bundle: &BundleFacts,
    model: &Model,
    providers: &ProviderSet,
) -> Result<Vec<ObservationBinding>, String> {
    let mut bindings = Vec::new();
    for (instance, component) in &bundle.components {
        if !component.driver {
            continue;
        }
        let capabilities = component_definition(&component.definition).1;
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
            capabilities
                .get(&requirement.port)
                .is_none_or(|capability| capability.kind != "imu")
        });
        for requirement in requirements {
            if handled.contains(requirement.port.as_str()) {
                continue;
            }
            let capability = capabilities.get(&requirement.port).ok_or_else(|| {
                format!(
                    "component {instance} capability {:?} is missing from authored definition",
                    requirement.port
                )
            })?;
            let native_target = format!("{prefix}{}", capability.target.id);
            match capability.kind.as_str() {
                "imu" => {
                    let acceleration = capabilities.get("accelerometer").ok_or_else(|| {
                        format!("component {instance} IMU has no accelerometer capability")
                    })?;
                    let gyroscope = capabilities.get("gyroscope").ok_or_else(|| {
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
