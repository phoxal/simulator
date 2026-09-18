use super::binding::require_port;
use super::camera::CameraEncoding;
use super::camera::encode_camera_observation;
use super::camera::encode_depth_observation;
use super::geodesy::Georeference;
use crate::remote::NativeProviderError;
use crate::remote::ProviderSet;
use phoxal::communication::simulation::Observation;
use phoxal_component_bno085 as bno085_contract;
use phoxal_component_ddsm115 as ddsm115_contract;
use phoxal_component_oak_d_lite as oak_contract;
use phoxal_component_vl53l1x as vl53l1x_contract;
use phoxal_component_zed_f9p as zed_contract;
use crate::mujoco::CameraBinding;
use crate::mujoco::Model;
use crate::mujoco::SensorBinding;
use crate::mujoco::SiteBinding;
use crate::mujoco::StateSnapshot;
use crate::mujoco::Workspace;
use phoxal::port::PortSignature;
use prost::Message;

#[derive(Clone, Debug)]
pub(super) enum BoundObservation {
    Bno085 {
        service_instance: String,
        frame_id: String,
        orientation: SensorBinding,
        accelerometer: SensorBinding,
        gyroscope: SensorBinding,
    },

    Ddsm115EncoderJoint {
        service_instance: String,
        joint_id: String,
    },
    OakImu {
        service_instance: String,
        frame_id: String,
        orientation: SensorBinding,
        accelerometer: SensorBinding,
        gyroscope: SensorBinding,
    },

    ZedF9pGnss {
        service_instance: String,
        antenna: SiteBinding,
        georeference: Georeference,
    },
    OakLeftMonoCamera {
        service_instance: String,
        camera: CameraBinding,
    },
    OakRgbCamera {
        service_instance: String,
        camera: CameraBinding,
    },
    OakRightMonoCamera {
        service_instance: String,
        camera: CameraBinding,
    },
    OakDepth {
        service_instance: String,
        camera: CameraBinding,
        range_m: [f64; 2],
    },
    Vl53l1xRange {
        service_instance: String,
        site: SiteBinding,
        min_range_m: f64,
        max_range_m: f64,
        fov_rad: f64,
    },
}

impl BoundObservation {
    pub(super) fn requires_workspace(&self) -> bool {
        matches!(
            self,
            Self::OakLeftMonoCamera { .. }
                | Self::OakRgbCamera { .. }
                | Self::OakRightMonoCamera { .. }
                | Self::OakDepth { .. }
                | Self::Vl53l1xRange { .. }
        )
    }

    pub(super) fn encode(
        &self,
        providers: &ProviderSet,
        model: &Model,
        state: &StateSnapshot,
        quantum_ns: u64,
        render_workspace: Option<&mut Workspace>,
    ) -> Result<Vec<Observation>, NativeProviderError> {
        match self {
            Self::Bno085 {
                service_instance,
                frame_id,
                orientation,
                accelerometer,
                gyroscope,
            } => {
                let orientation = exact_quaternion(orientation, state, "BNO085 orientation")?;
                let accelerometer = exact_values(accelerometer, state, 3, "BNO085 acceleration")?;
                let gyroscope = exact_values(gyroscope, state, 3, "BNO085 angular velocity")?;
                let imu = bno085_contract::ImuSample {
                    orientation: Some(bno085_contract::Quaternion {
                        w: orientation[0],
                        x: orientation[1],
                        y: orientation[2],
                        z: orientation[3],
                    }),
                    angular_velocity_radps: Some(bno085_contract::Vector3 {
                        x: gyroscope[0],
                        y: gyroscope[1],
                        z: gyroscope[2],
                    }),
                    linear_acceleration_mps2: Some(bno085_contract::Vector3 {
                        x: accelerometer[0],
                        y: accelerometer[1],
                        z: accelerometer[2],
                    }),
                    sensor_frame_id: frame_id.clone(),
                };
                let accel = bno085_contract::AccelerometerSample {
                    linear_acceleration_mps2: Some(bno085_contract::Vector3 {
                        x: accelerometer[0],
                        y: accelerometer[1],
                        z: accelerometer[2],
                    }),
                };
                let gyro = bno085_contract::GyroscopeSample {
                    angular_velocity_radps: Some(bno085_contract::Vector3 {
                        x: gyroscope[0],
                        y: gyroscope[1],
                        z: gyroscope[2],
                    }),
                };
                let mut observations = Vec::new();
                if providers
                    .get(service_instance, bno085_contract::ports::IMU.name())
                    .is_some()
                {
                    observations.push(encode_observation(
                        providers,
                        service_instance,
                        bno085_contract::ports::IMU.signature(),
                        state,
                        quantum_ns,
                        imu.encode_to_vec(),
                    )?);
                }
                if providers
                    .get(
                        service_instance,
                        bno085_contract::ports::ACCELEROMETER.name(),
                    )
                    .is_some()
                {
                    observations.push(encode_observation(
                        providers,
                        service_instance,
                        bno085_contract::ports::ACCELEROMETER.signature(),
                        state,
                        quantum_ns,
                        accel.encode_to_vec(),
                    )?);
                }
                if providers
                    .get(service_instance, bno085_contract::ports::GYROSCOPE.name())
                    .is_some()
                {
                    observations.push(encode_observation(
                        providers,
                        service_instance,
                        bno085_contract::ports::GYROSCOPE.signature(),
                        state,
                        quantum_ns,
                        gyro.encode_to_vec(),
                    )?);
                }
                Ok(observations)
            }
            Self::Ddsm115EncoderJoint {
                service_instance,
                joint_id,
            } => Ok(vec![crate::observations::encode_encoder_observation(
                providers,
                service_instance,
                ddsm115_contract::ports::ENCODER.name(),
                joint_id,
                model,
                state,
                quantum_ns,
            )?]),
            Self::OakImu {
                service_instance,
                frame_id,
                orientation,
                accelerometer,
                gyroscope,
            } => {
                let orientation = exact_quaternion(orientation, state, "OAK-D Lite orientation")?;
                let accelerometer =
                    exact_values(accelerometer, state, 3, "OAK-D Lite acceleration")?;
                let gyroscope = exact_values(gyroscope, state, 3, "OAK-D Lite angular velocity")?;
                let imu = oak_contract::ImuSample {
                    orientation: Some(oak_contract::Quaternion {
                        w: orientation[0],
                        x: orientation[1],
                        y: orientation[2],
                        z: orientation[3],
                    }),
                    angular_velocity_radps: Some(oak_contract::Vector3 {
                        x: gyroscope[0],
                        y: gyroscope[1],
                        z: gyroscope[2],
                    }),
                    linear_acceleration_mps2: Some(oak_contract::Vector3 {
                        x: accelerometer[0],
                        y: accelerometer[1],
                        z: accelerometer[2],
                    }),
                    sensor_frame_id: frame_id.clone(),
                };
                let accel = oak_contract::AccelerometerSample {
                    linear_acceleration_mps2: Some(oak_contract::Vector3 {
                        x: accelerometer[0],
                        y: accelerometer[1],
                        z: accelerometer[2],
                    }),
                };
                let gyro = oak_contract::GyroscopeSample {
                    angular_velocity_radps: Some(oak_contract::Vector3 {
                        x: gyroscope[0],
                        y: gyroscope[1],
                        z: gyroscope[2],
                    }),
                };
                let mut observations = Vec::new();
                if providers
                    .get(service_instance, oak_contract::ports::IMU.name())
                    .is_some()
                {
                    observations.push(encode_observation(
                        providers,
                        service_instance,
                        oak_contract::ports::IMU.signature(),
                        state,
                        quantum_ns,
                        imu.encode_to_vec(),
                    )?);
                }
                if providers
                    .get(service_instance, oak_contract::ports::ACCELEROMETER.name())
                    .is_some()
                {
                    observations.push(encode_observation(
                        providers,
                        service_instance,
                        oak_contract::ports::ACCELEROMETER.signature(),
                        state,
                        quantum_ns,
                        accel.encode_to_vec(),
                    )?);
                }
                if providers
                    .get(service_instance, oak_contract::ports::GYROSCOPE.name())
                    .is_some()
                {
                    observations.push(encode_observation(
                        providers,
                        service_instance,
                        oak_contract::ports::GYROSCOPE.signature(),
                        state,
                        quantum_ns,
                        gyro.encode_to_vec(),
                    )?);
                }
                Ok(observations)
            }
            Self::ZedF9pGnss {
                service_instance,
                antenna,
                georeference,
            } => {
                let position = antenna.position(state)?;
                let [latitude_deg, longitude_deg, altitude_m] = georeference.project(position)?;
                let sample = zed_contract::GnssSample {
                    latitude_deg,
                    longitude_deg,
                    altitude_m,
                    position_covariance: Vec::new(),
                };
                Ok(vec![encode_observation(
                    providers,
                    service_instance,
                    zed_contract::ports::GNSS.signature(),
                    state,
                    quantum_ns,
                    sample.encode_to_vec(),
                )?])
            }
            Self::OakLeftMonoCamera {
                service_instance,
                camera,
            } => Ok(vec![encode_camera_observation(
                providers,
                service_instance,
                oak_contract::ports::LEFT_MONO.signature(),
                camera,
                render_workspace,
                state,
                quantum_ns,
                CameraEncoding::Mono8,
            )?]),
            Self::OakRgbCamera {
                service_instance,
                camera,
            } => Ok(vec![encode_camera_observation(
                providers,
                service_instance,
                oak_contract::ports::RGB.signature(),
                camera,
                render_workspace,
                state,
                quantum_ns,
                CameraEncoding::Rgb8,
            )?]),
            Self::OakRightMonoCamera {
                service_instance,
                camera,
            } => Ok(vec![encode_camera_observation(
                providers,
                service_instance,
                oak_contract::ports::RIGHT_MONO.signature(),
                camera,
                render_workspace,
                state,
                quantum_ns,
                CameraEncoding::Mono8,
            )?]),
            Self::OakDepth {
                service_instance,
                camera,
                range_m,
            } => Ok(vec![encode_depth_observation(
                providers,
                service_instance,
                camera,
                render_workspace,
                state,
                quantum_ns,
                *range_m,
            )?]),
            Self::Vl53l1xRange {
                service_instance,
                site,
                min_range_m,
                max_range_m,
                fov_rad,
            } => encode_range_observation(
                providers,
                service_instance,
                site,
                *min_range_m,
                *max_range_m,
                *fov_rad,
                render_workspace,
                state,
                quantum_ns,
            ),
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub(super) fn encode_range_observation(
    providers: &ProviderSet,
    service_instance: &str,
    site: &SiteBinding,
    min_range_m: f64,
    max_range_m: f64,
    fov_rad: f64,
    observation_workspace: Option<&mut Workspace>,
    state: &StateSnapshot,
    quantum_ns: u64,
) -> Result<Vec<Observation>, NativeProviderError> {
    let workspace = observation_workspace.ok_or_else(|| {
        NativeProviderError::Unsupported(format!(
            "provider {service_instance}/{} has no native observation workspace",
            vl53l1x_contract::ports::RANGE.name()
        ))
    })?;
    let distance = workspace
        .finite_fov_range(site.native, min_range_m, max_range_m, fov_rad)
        .map_err(|error| NativeProviderError::Unsupported(error.to_string()))?;
    let sample = vl53l1x_contract::RangeSample {
        distance_m: distance.unwrap_or(0.0),
        min_range_m,
        max_range_m,
        valid: distance.is_some(),
    };
    Ok(vec![encode_observation(
        providers,
        service_instance,
        vl53l1x_contract::ports::RANGE.signature(),
        state,
        quantum_ns,
        sample.encode_to_vec(),
    )?])
}

pub(super) fn encode_observation(
    providers: &ProviderSet,
    service_instance: &str,
    signature: PortSignature,
    state: &StateSnapshot,
    quantum_ns: u64,
    payload: Vec<u8>,
) -> Result<Observation, NativeProviderError> {
    require_port(providers, service_instance, signature, Vec::new())?;
    crate::observations::packet(service_instance, signature.name, state, quantum_ns, payload)
}

pub(super) fn exact_values<'a>(
    binding: &SensorBinding,
    state: &'a StateSnapshot,
    dimension: usize,
    name: &str,
) -> Result<&'a [f64], NativeProviderError> {
    let values = binding.values(state)?;
    if values.len() != dimension {
        return Err(NativeProviderError::InvalidPayload(format!(
            "{name} has native dimension {}, expected {dimension}",
            values.len()
        )));
    }
    if values.iter().any(|value| !value.is_finite()) {
        return Err(NativeProviderError::InvalidPayload(format!(
            "{name} contains a non-finite native value"
        )));
    }
    Ok(values)
}

pub(super) fn exact_quaternion<'a>(
    binding: &SensorBinding,
    state: &'a StateSnapshot,
    name: &str,
) -> Result<&'a [f64], NativeProviderError> {
    let values = exact_values(binding, state, 4, name)?;
    let norm_squared = values.iter().map(|value| value * value).sum::<f64>();
    if !norm_squared.is_finite() || (norm_squared - 1.0).abs() > 1.0e-6 {
        return Err(NativeProviderError::InvalidPayload(format!(
            "{name} is not a unit quaternion in native [w, x, y, z] order"
        )));
    }
    Ok(values)
}
