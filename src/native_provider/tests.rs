use super::camera::rgb_to_mono8;
use super::config::ActuationDeclaration;
use super::config::ActuatorTarget;
use super::config::NativeControlMode;
use super::config::ObservationBinding;
use super::geodesy::Georeference;
use super::geodesy::WGS84_FIRST_ECCENTRICITY_SQUARED;
use super::geodesy::WGS84_SEMI_MAJOR_AXIS_METERS;
use super::runtime::ComponentProvider;
use crate::mujoco::Model;
use crate::remote::ProviderSet;
use phoxal::communication::simulation::ProductMembership;

use std::path::Path;

use crate::remote::NativeProvider;
use phoxal::communication::simulation::{Actuation, ProviderRequirement};
use prost::Message;

fn ddsm115_model() -> Model {
    Model::from_file(Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/motor.xml"))
        .expect("native encoder fixture")
}

fn provider_requirement<P: phoxal::contracts::MethodDescriptor>(
    service_instance: &str,
    port: P,
) -> ProviderRequirement {
    let signature = port.signature();
    ProviderRequirement {
        rate_microhertz: 500_000_000,
        service_instance: service_instance.to_owned(),
        port: signature.endpoint.to_owned(),
        payload_fqn: signature.response.to_owned(),
        shape: phoxal::communication::session::MethodShape::Observation as i32,
        input_fqn: signature.request.to_owned(),
    }
}

#[test]
fn component_encoder_reads_native_joint_and_applies_velocity() {
    let model = ddsm115_model();
    let providers = ProviderSet::new(vec![provider_requirement(
        "wheel",
        crate::contract::simulator_api::ENCODER,
    )])
    .expect("provider requirements");
    let mut provider = ComponentProvider::new(
        &model,
        providers,
        [ObservationBinding::ddsm115_encoder_joint(
            "wheel",
            "motor_joint",
        )],
        [ActuationDeclaration::motion(
            "motion",
            [ActuatorTarget::new(
                "wheel__motor",
                "motor",
                NativeControlMode::Velocity,
            )],
        )],
        std::collections::BTreeMap::from([(
            ("wheel".into(), "encoder".into()),
            crate::cadence::Cadence::new(500.0, 2_000_000).unwrap(),
        )]),
    )
    .expect("official DDSM115 provider binding");
    assert_eq!(provider.observation_facts.len(), 1);
    assert_eq!(provider.actuation_facts.len(), 1);

    let scene = crate::mujoco::Scene::new(model.clone()).expect("native scene");
    let state = scene.snapshot().expect("initial native state");
    let observations = provider
        .observations(&model, &state, 2_000_000)
        .expect("encoder observation");
    assert_eq!(observations.len(), 1);
    let sample = phoxal::contracts::component::encoder::EncoderSample::decode(
        observations[0].payload.as_slice(),
    )
    .expect("encoder payload");
    assert_eq!(sample.position_rad, Some(0.0));
    assert_eq!(sample.velocity_radps, Some(0.0));

    let setpoint = phoxal::contracts::component::actuator::ActuatorSetpoint {
        targets: vec![phoxal::contracts::component::actuator::ActuatorTarget {
            actuator_id: "wheel__motor".to_owned(),
            control: Some(phoxal::contracts::component::actuator::Control::VelocityRadps(3.0)),
        }],
    };
    let controls = provider
        .controls(
            &model,
            &[Actuation {
                membership: Some(ProductMembership {
                    producer: "motion".to_owned(),
                    port: crate::contract::simulator_api::ACTUATORS
                        .signature()
                        .endpoint
                        .to_owned(),
                    ..ProductMembership::default()
                }),
                valid_until_ns: 2_000_000,
                payload: setpoint.encode_to_vec(),
            }],
        )
        .expect("velocity actuation");
    assert_eq!(controls, vec![3.0]);
}

#[test]
fn mono_conversion_uses_fixed_luminance_coefficients() {
    assert_eq!(rgb_to_mono8(&[0, 0, 0]).expect("black pixel"), vec![0]);
    assert_eq!(
        rgb_to_mono8(&[255, 255, 255]).expect("white pixel"),
        vec![255]
    );
    assert_eq!(
        rgb_to_mono8(&[255, 0, 0, 0, 255, 0, 0, 0, 255]).expect("primary pixels"),
        vec![77, 149, 29]
    );
    assert!(rgb_to_mono8(&[0, 0]).is_err());
}

#[test]
fn georeference_uses_wgs84_ellipsoid_with_enu_yaw() {
    let latitude_deg = 52.0;
    let longitude_deg = 5.0;
    let altitude_m = 14.0;
    let georeference = Georeference::with_yaw(
        latitude_deg,
        longitude_deg,
        altitude_m,
        [3.0, -4.0, 2.5],
        std::f64::consts::FRAC_PI_2,
    )
    .expect("valid WGS84 reference");
    let origin = georeference
        .project([3.0, -4.0, 2.5])
        .expect("origin projection");
    assert!((origin[0] - latitude_deg).abs() < 1.0e-12);
    assert!((origin[1] - longitude_deg).abs() < 1.0e-12);
    assert!((origin[2] - altitude_m).abs() < 1.0e-8);

    let north = georeference
        .project([13.0, -4.0, 2.5])
        .expect("north projection");
    let latitude = latitude_deg.to_radians();
    let sin_latitude = latitude.sin();
    let meridian_radius = WGS84_SEMI_MAJOR_AXIS_METERS * (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED)
        / (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude).powf(1.5);
    let expected_latitude = latitude_deg + (10.0 / meridian_radius).to_degrees();
    // The tangent-plane expectation is first-order.  Compare it with an
    // explicit one-centimetre geodetic bound because the ECEF round trip
    // includes the ellipsoid's second-order curvature.
    let latitude_one_cm = (0.01 / meridian_radius).to_degrees();
    assert!((north[0] - expected_latitude).abs() < latitude_one_cm);
    assert!((north[1] - longitude_deg).abs() < 0.01e-6);
    assert!((north[2] - altitude_m).abs() < 0.01);

    let east = Georeference::with_yaw(latitude_deg, longitude_deg, altitude_m, [0.0; 3], 0.0)
        .expect("zero-yaw reference")
        .project([10.0, 0.0, 0.0])
        .expect("east projection");
    let prime_vertical_radius = WGS84_SEMI_MAJOR_AXIS_METERS
        / (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude).sqrt();
    let expected_longitude =
        longitude_deg + (10.0 / (prime_vertical_radius * latitude.cos())).to_degrees();
    let longitude_one_cm = (0.01 / (prime_vertical_radius * latitude.cos())).to_degrees();
    assert!((east[0] - latitude_deg).abs() < 0.01e-6);
    assert!((east[1] - expected_longitude).abs() < longitude_one_cm);
    assert!((east[2] - altitude_m).abs() < 0.01);
}

mod numerical;
