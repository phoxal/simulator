use super::config::ActuationDeclaration;
use super::config::NativeControlMode;
use super::config::ObservationBinding;
use super::observations::BoundObservation;
use crate::mujoco::ActuatorBinding;
use crate::mujoco::ActuatorMode;
use crate::mujoco::Model;
use crate::mujoco::SensorBinding;
use crate::mujoco::SensorKind;
use crate::remote::ActuationBinding;
use crate::remote::NativeProviderError;
use crate::remote::ProviderSet;
use phoxal::contracts::MethodSignature;
use prost::Name;
use std::collections::BTreeSet;

pub(super) const MAX_IDENTIFIER_BYTES: usize = 64;

/// Exact provider-to-native facts exposed by the reference provider.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NativeObservationBindingFact {
    /// Provider service instance.
    pub service_instance: String,
    /// Generated port name.
    pub port: String,
    /// Generated response payload FQN.
    pub payload_fqn: String,
    /// Native model object names used by this output.
    pub native_names: Vec<String>,
}

/// Exact actuator provenance exposed by the reference provider.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NativeActuatorBindingFact {
    /// Wire actuator identity.
    pub actuator_id: String,
    /// Native model actuator name.
    pub native_name: String,
    /// Explicit authored control interpretation.
    pub mode: NativeControlMode,
    /// Native control family observed in the compiled MuJoCo model.
    pub native_mode: ActuatorMode,
}

/// Exact typed motion binding and its native actuator mappings.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NativeActuationBindingFact {
    /// Provider service instance.
    pub service_instance: String,
    /// Generated output port.
    pub port: String,
    /// Generated output payload FQN.
    pub payload_fqn: String,
    /// Complete native mappings.
    pub actuators: Vec<NativeActuatorBindingFact>,
}

#[derive(Clone, Debug)]
pub(super) struct BoundActuation {
    pub(super) binding: ActuationBinding,
    pub(super) targets: Vec<BoundActuatorTarget>,
}

#[derive(Clone, Debug)]
pub(super) struct BoundActuatorTarget {
    pub(super) wire_id: String,
    pub(super) native_name: String,
    pub(super) binding: ActuatorBinding,
    pub(super) mode: NativeControlMode,
}

pub(super) fn bind_observation(
    model: &Model,
    providers: &ProviderSet,
    config: ObservationBinding,
) -> Result<(BoundObservation, Vec<NativeObservationBindingFact>), NativeProviderError> {
    match config {
        ObservationBinding::Bno085Imu {
            service_instance,
            sensor_frame_id,
            orientation_sensor,
            accelerometer_sensor,
            gyroscope_sensor,
            sensor_site,
        } => {
            validate_text(&service_instance, "BNO085 service instance")?;
            validate_text(&sensor_frame_id, "BNO085 sensor frame")?;
            let orientation =
                model.bind_sensor(crate::contract::simulator_api::IMU, &orientation_sensor)?;
            let accelerometer = model.bind_sensor(
                crate::contract::simulator_api::ACCELEROMETER,
                &accelerometer_sensor,
            )?;
            let gyroscope =
                model.bind_sensor(crate::contract::simulator_api::GYROSCOPE, &gyroscope_sensor)?;
            let _sensor_site = sensor_site
                .map(|site| model.bind_site(crate::contract::simulator_api::IMU, &site))
                .transpose()?;
            require_sensor_kind(
                &orientation,
                SensorKind::FrameQuaternion,
                "BNO085 orientation",
            )?;
            require_sensor_kind(
                &accelerometer,
                SensorKind::Accelerometer,
                "BNO085 acceleration",
            )?;
            require_sensor_kind(&gyroscope, SensorKind::Gyroscope, "BNO085 angular velocity")?;
            let facts: Vec<NativeObservationBindingFact> = [
                optional_port(
                    providers,
                    &service_instance,
                    crate::contract::simulator_api::IMU.signature(),
                    vec![orientation_sensor],
                )?,
                optional_port(
                    providers,
                    &service_instance,
                    crate::contract::simulator_api::ACCELEROMETER.signature(),
                    vec![accelerometer_sensor],
                )?,
                optional_port(
                    providers,
                    &service_instance,
                    crate::contract::simulator_api::GYROSCOPE.signature(),
                    vec![gyroscope_sensor],
                )?,
            ]
            .into_iter()
            .flatten()
            .collect();
            if facts.is_empty() {
                return Err(NativeProviderError::InvalidPayload(
                    "BNO085 binding does not correspond to any selected generated provider port"
                        .to_owned(),
                ));
            }
            Ok((
                BoundObservation::Bno085 {
                    service_instance,
                    frame_id: sensor_frame_id,
                    orientation,
                    accelerometer,
                    gyroscope,
                },
                facts,
            ))
        }
        ObservationBinding::Ddsm115EncoderJoint {
            service_instance,
            joint_id,
        } => {
            validate_text(&service_instance, "DDSM115 service instance")?;
            validate_text(&joint_id, "DDSM115 encoder joint")?;
            let joint = model.joint(&joint_id)?.ok_or_else(|| {
                NativeProviderError::InvalidPayload(format!(
                    "model has no DDSM115 encoder joint {joint_id:?}"
                ))
            })?;
            let info = model.joint_info(joint)?;
            if info.kind != crate::mujoco::JointKind::Hinge {
                return Err(NativeProviderError::Unsupported(format!(
                    "DDSM115 encoder joint {joint_id:?} is not a scalar hinge"
                )));
            }
            let fact = require_port(
                providers,
                &service_instance,
                crate::contract::simulator_api::ENCODER.signature(),
                vec![joint_id.clone()],
            )?;
            Ok((
                BoundObservation::Ddsm115EncoderJoint {
                    service_instance,
                    joint_id,
                },
                vec![fact],
            ))
        }
        ObservationBinding::OakImu {
            service_instance,
            sensor_frame_id,
            orientation_sensor,
            accelerometer_sensor,
            gyroscope_sensor,
            sensor_site,
        } => {
            validate_text(&service_instance, "OAK-D Lite service instance")?;
            validate_text(&sensor_frame_id, "OAK-D Lite sensor frame")?;
            let orientation =
                model.bind_sensor(crate::contract::simulator_api::IMU, &orientation_sensor)?;
            let accelerometer = model.bind_sensor(
                crate::contract::simulator_api::ACCELEROMETER,
                &accelerometer_sensor,
            )?;
            let gyroscope =
                model.bind_sensor(crate::contract::simulator_api::GYROSCOPE, &gyroscope_sensor)?;
            let _sensor_site = sensor_site
                .map(|site| model.bind_site(crate::contract::simulator_api::IMU, &site))
                .transpose()?;
            require_sensor_kind(
                &orientation,
                SensorKind::FrameQuaternion,
                "OAK-D Lite orientation",
            )?;
            require_sensor_kind(
                &accelerometer,
                SensorKind::Accelerometer,
                "OAK-D Lite acceleration",
            )?;
            require_sensor_kind(
                &gyroscope,
                SensorKind::Gyroscope,
                "OAK-D Lite angular velocity",
            )?;
            let facts: Vec<NativeObservationBindingFact> = [
                optional_port(
                    providers,
                    &service_instance,
                    crate::contract::simulator_api::IMU.signature(),
                    vec![orientation_sensor],
                )?,
                optional_port(
                    providers,
                    &service_instance,
                    crate::contract::simulator_api::ACCELEROMETER.signature(),
                    vec![accelerometer_sensor],
                )?,
                optional_port(
                    providers,
                    &service_instance,
                    crate::contract::simulator_api::GYROSCOPE.signature(),
                    vec![gyroscope_sensor],
                )?,
            ]
            .into_iter()
            .flatten()
            .collect();
            if facts.is_empty() {
                return Err(NativeProviderError::InvalidPayload(
                    "OAK-D Lite IMU binding does not correspond to any selected generated provider port"
                        .to_owned(),
                ));
            }
            Ok((
                BoundObservation::OakImu {
                    service_instance,
                    frame_id: sensor_frame_id,
                    orientation,
                    accelerometer,
                    gyroscope,
                },
                facts,
            ))
        }
        ObservationBinding::ZedF9pGnss {
            service_instance,
            antenna_site,
            georeference,
        } => {
            validate_text(&service_instance, "ZED-F9P service instance")?;
            validate_text(&antenna_site, "ZED-F9P antenna site")?;
            georeference.validate()?;
            let antenna = model.bind_site(crate::contract::simulator_api::GNSS, &antenna_site)?;
            let fact = require_port(
                providers,
                &service_instance,
                crate::contract::simulator_api::GNSS.signature(),
                vec![antenna_site],
            )?;
            Ok((
                BoundObservation::ZedF9pGnss {
                    service_instance,
                    antenna,
                    georeference,
                },
                vec![fact],
            ))
        }
        ObservationBinding::OakLeftMonoCamera {
            service_instance,
            native_camera,
        } => {
            validate_text(&service_instance, "OAK-D Lite left mono service instance")?;
            let fact = require_port(
                providers,
                &service_instance,
                crate::contract::simulator_api::LEFT_MONO.signature(),
                vec![native_camera.clone()],
            )?;
            let camera =
                model.bind_camera(crate::contract::simulator_api::LEFT_MONO, &native_camera)?;
            Ok((
                BoundObservation::OakLeftMonoCamera {
                    service_instance,
                    camera,
                },
                vec![fact],
            ))
        }
        ObservationBinding::OakRgbCamera {
            service_instance,
            native_camera,
        } => {
            validate_text(&service_instance, "OAK-D Lite RGB service instance")?;
            let fact = require_port(
                providers,
                &service_instance,
                crate::contract::simulator_api::RGB.signature(),
                vec![native_camera.clone()],
            )?;
            let camera = model.bind_camera(crate::contract::simulator_api::RGB, &native_camera)?;
            Ok((
                BoundObservation::OakRgbCamera {
                    service_instance,
                    camera,
                },
                vec![fact],
            ))
        }
        ObservationBinding::OakRightMonoCamera {
            service_instance,
            native_camera,
        } => {
            validate_text(&service_instance, "OAK-D Lite right mono service instance")?;
            let fact = require_port(
                providers,
                &service_instance,
                crate::contract::simulator_api::RIGHT_MONO.signature(),
                vec![native_camera.clone()],
            )?;
            let camera =
                model.bind_camera(crate::contract::simulator_api::RIGHT_MONO, &native_camera)?;
            Ok((
                BoundObservation::OakRightMonoCamera {
                    service_instance,
                    camera,
                },
                vec![fact],
            ))
        }
        ObservationBinding::OakDepth {
            service_instance,
            native_camera,
            range_m,
        } => {
            if !range_m.iter().all(|value| value.is_finite())
                || range_m[0] <= 0.0
                || range_m[0] >= range_m[1]
            {
                return Err(NativeProviderError::InvalidPayload(
                    "depth range must be finite, positive, and ordered".into(),
                ));
            }
            validate_text(&service_instance, "OAK-D Lite depth service instance")?;
            let fact = require_port(
                providers,
                &service_instance,
                crate::contract::simulator_api::DEPTH.signature(),
                vec![native_camera.clone()],
            )?;
            let camera =
                model.bind_camera(crate::contract::simulator_api::DEPTH, &native_camera)?;
            Ok((
                BoundObservation::OakDepth {
                    service_instance,
                    camera,
                    range_m,
                },
                vec![fact],
            ))
        }
        ObservationBinding::Vl53l1xRange {
            service_instance,
            native_site,
            min_range_m,
            max_range_m,
            fov_rad,
        } => {
            validate_text(&service_instance, "VL53L1X service instance")?;
            let _ = require_port(
                providers,
                &service_instance,
                crate::contract::simulator_api::RANGE.signature(),
                vec![native_site.clone()],
            )?;
            let site = model.bind_site(crate::contract::simulator_api::RANGE, &native_site)?;
            if !min_range_m.is_finite()
                || !max_range_m.is_finite()
                || !fov_rad.is_finite()
                || min_range_m < 0.0
                || max_range_m <= min_range_m
                || fov_rad <= 0.0
            {
                return Err(NativeProviderError::InvalidPayload(
                    "VL53L1X range limits and field of view must be finite and ordered".to_owned(),
                ));
            }
            let fact = require_port(
                providers,
                &service_instance,
                crate::contract::simulator_api::RANGE.signature(),
                vec![native_site],
            )?;
            Ok((
                BoundObservation::Vl53l1xRange {
                    service_instance,
                    site,
                    min_range_m,
                    max_range_m,
                    fov_rad,
                },
                vec![fact],
            ))
        }
    }
}

pub(super) fn bind_actuation(
    model: &Model,
    config: &ActuationDeclaration,
) -> Result<(BoundActuation, NativeActuationBindingFact), NativeProviderError> {
    let port = crate::contract::simulator_api::ACTUATORS;
    if config.port != port.signature().endpoint
        || config.payload_fqn
            != phoxal::contracts::component::actuator::ActuatorSetpoint::full_name()
    {
        return Err(NativeProviderError::InvalidActuation(format!(
            "actuation binding {}/{} must use generated motion port {} with payload {}",
            config.service_instance,
            config.port,
            port.signature().endpoint,
            phoxal::contracts::component::actuator::ActuatorSetpoint::full_name()
        )));
    }
    validate_text(&config.service_instance, "motion service instance")?;
    if config.targets.is_empty() {
        return Err(NativeProviderError::InvalidActuation(format!(
            "motion binding {}/{} has no actuator targets",
            config.service_instance, config.port
        )));
    }
    let mut wire_ids = BTreeSet::new();
    let mut native_names = BTreeSet::new();
    let mut targets = Vec::new();
    let mut facts = Vec::new();
    for target in &config.targets {
        validate_text(&target.actuator_id, "wire actuator id")?;
        validate_text(&target.native_name, "native actuator name")?;
        if !wire_ids.insert(target.actuator_id.as_str()) {
            return Err(NativeProviderError::InvalidActuation(format!(
                "motion binding repeats wire actuator {}",
                target.actuator_id
            )));
        }
        if !native_names.insert(target.native_name.as_str()) {
            return Err(NativeProviderError::InvalidActuation(format!(
                "motion binding repeats native actuator {}",
                target.native_name
            )));
        }
        let binding = model.bind_actuator(port, &target.native_name)?;
        let native_mode = binding.info.mode;
        let mode_matches = matches!(
            (target.mode, native_mode),
            (NativeControlMode::Torque, ActuatorMode::Torque)
                | (NativeControlMode::Velocity, ActuatorMode::Velocity)
        );
        if !mode_matches {
            return Err(NativeProviderError::InvalidActuation(format!(
                "actuator {} is configured as {:?}, but native actuator {} has mode {:?}",
                target.actuator_id, target.mode, target.native_name, native_mode
            )));
        }
        targets.push(BoundActuatorTarget {
            wire_id: target.actuator_id.clone(),
            native_name: target.native_name.clone(),
            binding,
            mode: target.mode,
        });
        facts.push(NativeActuatorBindingFact {
            actuator_id: target.actuator_id.clone(),
            native_name: target.native_name.clone(),
            mode: target.mode,
            native_mode,
        });
    }
    let native_ids = targets
        .iter()
        .map(|target| target.native_name.clone())
        .collect::<Vec<_>>();
    let binding = ActuationBinding::new(
        config.service_instance.clone(),
        config.port.clone(),
        config.payload_fqn.clone(),
        native_ids,
    )
    .map_err(|error| NativeProviderError::InvalidActuation(error.to_string()))?;
    facts.sort_by(|left, right| left.native_name.cmp(&right.native_name));
    Ok((
        BoundActuation {
            binding: binding.clone(),
            targets,
        },
        NativeActuationBindingFact {
            service_instance: config.service_instance.clone(),
            port: config.port.clone(),
            payload_fqn: config.payload_fqn.clone(),
            actuators: facts,
        },
    ))
}

pub(super) fn require_port(
    providers: &ProviderSet,
    service_instance: &str,
    signature: MethodSignature,
    native_names: Vec<String>,
) -> Result<NativeObservationBindingFact, NativeProviderError> {
    let requirement = providers
        .get(service_instance, signature.endpoint)
        .ok_or_else(|| {
            NativeProviderError::InvalidPayload(format!(
                "no provider requirement for {service_instance}/{}",
                signature.endpoint
            ))
        })?;
    if requirement.shape != phoxal::communication::session::MethodShape::Observation as i32
        || signature.shape != phoxal::contracts::MethodShape::Observation
        || requirement.payload_fqn != signature.response
        || requirement.input_fqn != signature.request
    {
        return Err(NativeProviderError::InvalidPayload(format!(
            "provider {service_instance}/{} does not match generated signature {}",
            signature.endpoint, signature.response
        )));
    }
    Ok(NativeObservationBindingFact {
        service_instance: service_instance.to_owned(),
        port: signature.endpoint.to_owned(),
        payload_fqn: signature.response.to_owned(),
        native_names,
    })
}

pub(super) fn optional_port(
    providers: &ProviderSet,
    service_instance: &str,
    signature: MethodSignature,
    native_names: Vec<String>,
) -> Result<Option<NativeObservationBindingFact>, NativeProviderError> {
    if providers
        .get(service_instance, signature.endpoint)
        .is_none()
    {
        return Ok(None);
    }
    require_port(providers, service_instance, signature, native_names).map(Some)
}

pub(super) fn require_sensor_kind(
    binding: &SensorBinding,
    expected: SensorKind,
    capability: &str,
) -> Result<(), NativeProviderError> {
    if binding.info.kind != expected {
        return Err(NativeProviderError::Unsupported(format!(
            "{capability} is bound to native sensor kind {:?}, expected {:?}",
            binding.info.kind, expected
        )));
    }
    Ok(())
}

pub(super) fn validate_text(value: &str, field: &str) -> Result<(), NativeProviderError> {
    if value.is_empty()
        || value.len() > MAX_IDENTIFIER_BYTES
        || !value.is_ascii()
        || value
            .bytes()
            .any(|byte| byte.is_ascii_whitespace() || byte == 0)
    {
        return Err(NativeProviderError::InvalidPayload(format!(
            "{field} {value:?} is invalid"
        )));
    }
    Ok(())
}
