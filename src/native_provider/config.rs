use super::geodesy::Georeference;
use crate::remote::NativeProviderError;
use phoxal_component_bno085 as bno085_contract;
use phoxal_component_ddsm115 as ddsm115_contract;
use phoxal_component_oak_d_lite as oak_contract;
use phoxal_component_vl53l1x as vl53l1x_contract;
use phoxal_component_zed_f9p as zed_contract;

/// Explicit native control semantics for a configured actuator.
///
/// MuJoCo's scalar `ctrl` value has no universal interpretation.  The
/// selection is therefore part of the fixed simulator configuration and is
/// never guessed from a generic motion payload.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NativeControlMode {
    /// The authored actuator consumes torque in N m.
    Torque,
    /// The authored actuator is a velocity servo consuming rad/s.
    Velocity,
}

/// One explicit wire-to-native actuator identity mapping.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActuatorTarget {
    /// Actuator identity carried by the motion contract.
    pub actuator_id: String,
    /// Model-local native actuator name passed to `Model::bind_actuator`.
    pub native_name: String,
    /// Authored native interpretation of the scalar control.
    pub mode: NativeControlMode,
}

impl ActuatorTarget {
    /// Creates one explicit wire/native actuator mapping.
    #[must_use]
    pub fn new(
        actuator_id: impl Into<String>,
        native_name: impl Into<String>,
        mode: NativeControlMode,
    ) -> Self {
        Self {
            actuator_id: actuator_id.into(),
            native_name: native_name.into(),
            mode,
        }
    }
}

/// One explicit typed motion output admitted by the fixed reference provider.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActuationDeclaration {
    /// Service instance owning the output.
    pub service_instance: String,
    /// Generated motion output port.  The convenience constructor fills this
    /// from `phoxal_service_motion::ports::ACTUATORS`.
    pub port: String,
    /// Generated motion payload FQN.
    pub payload_fqn: String,
    /// Complete wire-to-native actuator mappings.
    pub targets: Vec<ActuatorTarget>,
}

impl ActuationDeclaration {
    /// Creates the official motion actuator output binding.
    #[must_use]
    pub fn motion(
        service_instance: impl Into<String>,
        targets: impl IntoIterator<Item = ActuatorTarget>,
    ) -> Self {
        Self {
            service_instance: service_instance.into(),
            port: phoxal_service_motion::ports::ACTUATORS.name().to_owned(),
            payload_fqn: phoxal_service_motion::ports::ACTUATORS
                .signature()
                .response
                .to_owned(),
            targets: targets.into_iter().collect(),
        }
    }
}

/// One official component capability selected for native observation.
///
/// Every native object name is caller-supplied.  The generated port identity
/// is fixed by the variant and cannot be replaced with a similarly named
/// handwritten port.
#[derive(Clone, Debug, PartialEq)]
pub enum ObservationBinding {
    /// BNO085 fused IMU plus its explicit accelerometer and gyroscope signals.
    Bno085Imu {
        /// Provider service instance.
        service_instance: String,
        /// Native frame identity carried in all three payloads.
        sensor_frame_id: String,
        /// Native frame-quaternion sensor name.
        orientation_sensor: String,
        /// Native accelerometer sensor name.
        accelerometer_sensor: String,
        /// Native gyroscope sensor name.
        gyroscope_sensor: String,
        /// Optional authored sensor site used to validate frame ownership.
        sensor_site: Option<String>,
    },

    /// DDSM115 encoder output derived from the authored component joint.
    Ddsm115EncoderJoint {
        /// Provider service instance.
        service_instance: String,
        /// Native joint selected by the component capability.
        joint_id: String,
    },
    /// OAK-D Lite fused IMU plus its explicit accelerometer and gyroscope signals.
    OakImu {
        /// Provider service instance.
        service_instance: String,
        /// Native frame identity carried in all three payloads.
        sensor_frame_id: String,
        /// Native frame-quaternion sensor name.
        orientation_sensor: String,
        /// Native accelerometer sensor name.
        accelerometer_sensor: String,
        /// Native gyroscope sensor name.
        gyroscope_sensor: String,
        /// Optional authored sensor site used to validate frame ownership.
        sensor_site: Option<String>,
    },

    /// ZED-F9P local site projected through an explicit WGS84 reference.
    ZedF9pGnss {
        /// Provider service instance.
        service_instance: String,
        /// Native antenna site name.
        antenna_site: String,
        /// Fixed WGS84/ENU projection.
        georeference: Georeference,
    },
    /// OAK-D Lite camera output rendered through the MuJoCo offscreen context.
    OakLeftMonoCamera {
        /// Provider service instance.
        service_instance: String,
        /// Native camera name.
        native_camera: String,
    },
    /// OAK-D Lite RGB camera output rendered through the MuJoCo offscreen context.
    OakRgbCamera {
        /// Provider service instance.
        service_instance: String,
        /// Native camera name.
        native_camera: String,
    },
    /// OAK-D Lite right monochrome camera output rendered through the MuJoCo offscreen context.
    OakRightMonoCamera {
        /// Provider service instance.
        service_instance: String,
        /// Native camera name.
        native_camera: String,
    },
    /// OAK-D Lite geometric depth output rendered through the MuJoCo offscreen context.
    OakDepth {
        /// Provider service instance.
        service_instance: String,
        /// Native depth camera name.
        native_camera: String,
        /// Inclusive valid geometric depth interval in meters.
        range_m: [f64; 2],
    },
    /// VL53L1X nearest-valid finite-FOV ray output.
    Vl53l1xRange {
        /// Provider service instance.
        service_instance: String,
        /// Native site at the range sensor origin.
        native_site: String,
        /// Declared minimum range in meters.
        min_range_m: f64,
        /// Declared maximum range in meters.
        max_range_m: f64,
        /// Declared full field of view in radians.
        fov_rad: f64,
    },
}

impl ObservationBinding {
    /// Convenience constructor for a BNO085 signal set with an authored site.
    #[must_use]
    pub fn bno085_at_site(
        service_instance: impl Into<String>,
        sensor_site: impl Into<String>,
        sensor_frame_id: impl Into<String>,
        orientation_sensor: impl Into<String>,
        accelerometer_sensor: impl Into<String>,
        gyroscope_sensor: impl Into<String>,
    ) -> Self {
        Self::Bno085Imu {
            service_instance: service_instance.into(),
            sensor_frame_id: sensor_frame_id.into(),
            orientation_sensor: orientation_sensor.into(),
            accelerometer_sensor: accelerometer_sensor.into(),
            gyroscope_sensor: gyroscope_sensor.into(),
            sensor_site: Some(sensor_site.into()),
        }
    }

    /// Convenience constructor for an encoder derived from an authored joint.
    #[must_use]
    pub fn ddsm115_encoder_joint(
        service_instance: impl Into<String>,
        joint_id: impl Into<String>,
    ) -> Self {
        Self::Ddsm115EncoderJoint {
            service_instance: service_instance.into(),
            joint_id: joint_id.into(),
        }
    }

    /// Convenience constructor for an OAK-D Lite signal set with an authored site.
    #[must_use]
    pub fn oak_imu_at_site(
        service_instance: impl Into<String>,
        sensor_site: impl Into<String>,
        sensor_frame_id: impl Into<String>,
        orientation_sensor: impl Into<String>,
        accelerometer_sensor: impl Into<String>,
        gyroscope_sensor: impl Into<String>,
    ) -> Self {
        Self::OakImu {
            service_instance: service_instance.into(),
            sensor_frame_id: sensor_frame_id.into(),
            orientation_sensor: orientation_sensor.into(),
            accelerometer_sensor: accelerometer_sensor.into(),
            gyroscope_sensor: gyroscope_sensor.into(),
            sensor_site: Some(sensor_site.into()),
        }
    }

    /// Convenience constructor for an explicit ZED-F9P geodetic binding.
    #[must_use]
    pub fn zed_f9p(
        service_instance: impl Into<String>,
        antenna_site: impl Into<String>,
        georeference: Georeference,
    ) -> Self {
        Self::ZedF9pGnss {
            service_instance: service_instance.into(),
            antenna_site: antenna_site.into(),
            georeference,
        }
    }

    /// Selects the owner-contract IMU encoder for one semantic `imu`
    /// capability.  Product payload identities are resolved here, inside the
    /// contract encoder owner, rather than in bundle composition code.
    pub fn semantic_imu(
        service_instance: impl Into<String>,
        sensor_site: impl Into<String>,
        sensor_frame_id: impl Into<String>,
        orientation_sensor: impl Into<String>,
        accelerometer_sensor: impl Into<String>,
        gyroscope_sensor: impl Into<String>,
        payload_fqn: &str,
    ) -> Result<Self, NativeProviderError> {
        let service_instance = service_instance.into();
        let sensor_site = sensor_site.into();
        let sensor_frame_id = sensor_frame_id.into();
        let orientation_sensor = orientation_sensor.into();
        let accelerometer_sensor = accelerometer_sensor.into();
        let gyroscope_sensor = gyroscope_sensor.into();
        if payload_fqn == bno085_contract::ports::IMU.signature().response {
            Ok(Self::bno085_at_site(
                service_instance,
                sensor_site,
                sensor_frame_id,
                orientation_sensor,
                accelerometer_sensor,
                gyroscope_sensor,
            ))
        } else if payload_fqn == oak_contract::ports::IMU.signature().response {
            Ok(Self::oak_imu_at_site(
                service_instance,
                sensor_site,
                sensor_frame_id,
                orientation_sensor,
                accelerometer_sensor,
                gyroscope_sensor,
            ))
        } else {
            Err(NativeProviderError::Unsupported(format!(
                "semantic IMU route has unsupported generated payload {payload_fqn}"
            )))
        }
    }

    /// Selects the owner-contract encoder for one semantic `encoder`
    /// capability.
    pub fn semantic_encoder(
        service_instance: impl Into<String>,
        native_joint: impl Into<String>,
        payload_fqn: &str,
    ) -> Result<Self, NativeProviderError> {
        let service_instance = service_instance.into();
        let native_joint = native_joint.into();
        if payload_fqn == ddsm115_contract::ports::ENCODER.signature().response {
            Ok(Self::ddsm115_encoder_joint(service_instance, native_joint))
        } else {
            Err(NativeProviderError::Unsupported(format!(
                "semantic encoder route has unsupported generated payload {payload_fqn}"
            )))
        }
    }

    /// Selects the owner-contract camera encoder for one semantic `camera`
    /// capability.
    pub fn semantic_camera(
        service_instance: impl Into<String>,
        port: &str,
        mode: &str,
        native_camera: impl Into<String>,
        payload_fqn: &str,
    ) -> Result<Self, NativeProviderError> {
        let service_instance = service_instance.into();
        let native_camera = native_camera.into();
        if payload_fqn != oak_contract::ports::LEFT_MONO.signature().response
            && payload_fqn != oak_contract::ports::RIGHT_MONO.signature().response
            && payload_fqn != oak_contract::ports::RGB.signature().response
        {
            return Err(NativeProviderError::Unsupported(format!(
                "semantic camera route has unsupported generated payload {payload_fqn}"
            )));
        }
        match (port, mode) {
            ("left_mono", "mono") => Ok(Self::OakLeftMonoCamera {
                service_instance,
                native_camera,
            }),
            ("rgb", "rgb") => Ok(Self::OakRgbCamera {
                service_instance,
                native_camera,
            }),
            ("right_mono", "mono") => Ok(Self::OakRightMonoCamera {
                service_instance,
                native_camera,
            }),
            _ => Err(NativeProviderError::Unsupported(format!(
                "semantic camera capability {port} has unsupported mode {mode}"
            ))),
        }
    }

    /// Selects the owner-contract depth encoder for one semantic `depth`
    /// capability.
    pub fn semantic_depth(
        service_instance: impl Into<String>,
        native_camera: impl Into<String>,
        payload_fqn: &str,
        range_m: [f64; 2],
    ) -> Result<Self, NativeProviderError> {
        if payload_fqn != oak_contract::ports::DEPTH.signature().response {
            return Err(NativeProviderError::Unsupported(format!(
                "semantic depth route has unsupported generated payload {payload_fqn}"
            )));
        }
        Ok(Self::OakDepth {
            service_instance: service_instance.into(),
            native_camera: native_camera.into(),
            range_m,
        })
    }

    /// Selects the owner-contract range encoder for one semantic `range`
    /// capability.
    pub fn semantic_range(
        service_instance: impl Into<String>,
        native_site: impl Into<String>,
        min_range_m: f64,
        max_range_m: f64,
        fov_rad: f64,
        payload_fqn: &str,
    ) -> Result<Self, NativeProviderError> {
        if payload_fqn != vl53l1x_contract::ports::RANGE.signature().response {
            return Err(NativeProviderError::Unsupported(format!(
                "semantic range route has unsupported generated payload {payload_fqn}"
            )));
        }
        Ok(Self::Vl53l1xRange {
            service_instance: service_instance.into(),
            native_site: native_site.into(),
            min_range_m,
            max_range_m,
            fov_rad,
        })
    }

    /// Selects the owner-contract GNSS encoder for one semantic `gnss`
    /// capability.
    pub fn semantic_gnss(
        service_instance: impl Into<String>,
        antenna_site: impl Into<String>,
        georeference: Georeference,
        payload_fqn: &str,
    ) -> Result<Self, NativeProviderError> {
        if payload_fqn != zed_contract::ports::GNSS.signature().response {
            return Err(NativeProviderError::Unsupported(format!(
                "semantic GNSS route has unsupported generated payload {payload_fqn}"
            )));
        }
        Ok(Self::zed_f9p(service_instance, antenna_site, georeference))
    }
}
