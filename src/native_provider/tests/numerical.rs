use super::*;
use phoxal_component_bno085 as imu;
use phoxal_mujoco::Workspace;
use std::collections::BTreeMap;

fn model(xml: &str) -> Model {
    Model::from_xml(xml).unwrap()
}

fn provider(
    model: &Model,
    requirements: Vec<ProviderRequirement>,
    binding: ObservationBinding,
    mode: Option<NativeControlMode>,
) -> ComponentProvider {
    let cadence = requirements
        .iter()
        .map(|requirement| {
            (
                (
                    requirement.service_instance.clone(),
                    requirement.port.clone(),
                ),
                crate::cadence::Cadence::new(500.0, 2_000_000).unwrap(),
            )
        })
        .collect::<BTreeMap<_, _>>();
    ComponentProvider::new(
        model,
        ProviderSet::new(requirements).unwrap(),
        [binding],
        mode.map(|mode| {
            ActuationDeclaration::motion("motion", [ActuatorTarget::new("drive", "drive", mode)])
        }),
        cadence,
    )
    .unwrap()
}

#[test]
fn encoder_si_and_bounded_servo_force_survive_direction_and_mount_rotation() {
    for quaternion in ["1 0 0 0", "0.7071067811865476 0.7071067811865476 0 0"] {
        let model = model(&format!(
            r#"<mujoco>
          <option timestep="0.002" gravity="0 0 0"/>
          <worldbody><body quat="{quaternion}"><joint name="rotor" axis="0 0 1"/>
            <geom type="sphere" size="0.1" mass="1"/>
          </body></worldbody>
          <actuator><velocity name="drive" joint="rotor" kv="1" forcerange="-2 2"/></actuator>
          <sensor><actuatorfrc name="force" actuator="drive"/></sensor>
        </mujoco>"#
        ));
        let mut provider = provider(
            &model,
            vec![provider_requirement(
                "wheel",
                ddsm115_contract::ports::ENCODER,
            )],
            ObservationBinding::ddsm115_encoder_joint("wheel", "rotor"),
            Some(NativeControlMode::Velocity),
        );
        let force = model
            .bind_sensor(ddsm115_contract::ports::ENCODER, "force")
            .unwrap();
        let mut workspace = Workspace::new(&model).unwrap();
        for direction in [-1.0, 1.0] {
            workspace
                .set_qpos(&[direction * std::f64::consts::FRAC_PI_2])
                .unwrap();
            workspace.set_qvel(&[direction * 2.0]).unwrap();
            workspace.forward().unwrap();
            let observations = provider
                .observations(&model, &workspace.snapshot().unwrap(), 2_000_000)
                .unwrap();
            let sample =
                ddsm115_contract::EncoderSample::decode(observations[0].payload.as_slice())
                    .unwrap();
            assert!(
                (sample.position_rad.unwrap() - direction * std::f64::consts::FRAC_PI_2).abs()
                    < 1e-9
            );
            assert!((sample.velocity_radps.unwrap() - direction * 2.0).abs() < 1e-9);
            // The public contract reports continuous SI ground truth. Count
            // quantization is not applied a second time by the simulator.
            assert!(
                (sample.position_rad.unwrap() / std::f64::consts::TAU * 4096.0
                    - direction * 1024.0)
                    .abs()
                    < 1e-9
            );
            workspace.set_controls(&[direction * 2.0]).unwrap();
            for (velocity, expected_force) in [(0.0, 2.0), (1.0, 1.0), (5.0, -2.0)] {
                workspace.set_qvel(&[direction * velocity]).unwrap();
                workspace.forward().unwrap();
                let state = workspace.snapshot().unwrap();
                assert!(
                    (force.values(&state).unwrap()[0] - direction * expected_force).abs() < 1e-6
                );
            }
        }
    }
}

#[test]
fn imu_mount_and_post_step_specific_force_are_encoded_in_sensor_coordinates() {
    for (quaternion, sign) in [("1 0 0 0", 1.0), ("0 1 0 0", -1.0)] {
        let model = model(&format!(
            r#"<mujoco>
          <option timestep="0.002" gravity="0 0 -9.81" integrator="Euler"/>
          <worldbody><body><joint name="slide" type="slide" axis="1 0 0"/>
            <geom type="sphere" size="0.1" mass="1"/>
            <site name="imu" quat="{quaternion}"/>
          </body></worldbody>
          <actuator><motor name="drive" joint="slide"/></actuator>
          <sensor><framequat name="orientation" objtype="site" objname="imu"/>
            <accelerometer name="acceleration" site="imu"/><gyro name="gyro" site="imu"/>
          </sensor>
        </mujoco>"#
        ));
        let mut provider = provider(
            &model,
            vec![provider_requirement("sensor", imu::ports::IMU)],
            ObservationBinding::bno085_at_site(
                "sensor",
                "imu",
                "sensor-frame",
                "orientation",
                "acceleration",
                "gyro",
            ),
            Some(NativeControlMode::Torque),
        );
        let mut workspace = Workspace::new(&model).unwrap();
        for accelerating in [false, true] {
            if accelerating {
                workspace.set_controls(&[1.0]).unwrap();
                workspace.step().unwrap();
                workspace.forward().unwrap();
            }
            let observations = provider
                .observations(&model, &workspace.snapshot().unwrap(), 2_000_000)
                .unwrap();
            let sample = imu::ImuSample::decode(observations[0].payload.as_slice()).unwrap();
            let q = sample.orientation.unwrap();
            let expected = if sign > 0.0 {
                [1.0, 0.0, 0.0, 0.0]
            } else {
                [0.0, 1.0, 0.0, 0.0]
            };
            let dot = [q.w, q.x, q.y, q.z]
                .iter()
                .zip(expected)
                .map(|(a, b)| a * b)
                .sum::<f64>();
            assert!((dot.abs() - 1.0).abs() < 1e-6);
            let acceleration = sample.linear_acceleration_mps2.unwrap();
            assert!((acceleration.x - if accelerating { 1.0 } else { 0.0 }).abs() < 1e-6);
            assert!(acceleration.y.abs() < 1e-6);
            assert!((acceleration.z - sign * 9.81).abs() < 1e-6);
            let gyro = sample.angular_velocity_radps.unwrap();
            assert!([gyro.x, gyro.y, gyro.z].iter().all(|v| v.abs() < 1e-6));
            assert_eq!(sample.sensor_frame_id, "sensor-frame");
        }
    }
}

#[test]
fn finite_fov_selects_off_axis_nearest_return_and_reports_no_hit() {
    use phoxal_component_vl53l1x as range;
    for (angle, wall, expected) in [
        (0.3_f64, true, Some(1.0)),
        (0.8, true, Some(2.0)),
        (0.8, false, None),
    ] {
        let x = 1.05 * angle.sin();
        let z = 1.05 * angle.cos();
        let wall = if wall {
            r#"<geom type="box" pos="0 0 2.1" size="3 3 0.1"/>"#
        } else {
            ""
        };
        let model = model(&format!(
            r#"<mujoco><option timestep="0.002"/>
          <worldbody><body name="sensor"><site name="range"/></body>
            <geom type="sphere" pos="{x} 0 {z}" size="0.05"/>{wall}
          </worldbody></mujoco>"#
        ));
        let mut provider = provider(
            &model,
            vec![provider_requirement("sensor", range::ports::RANGE)],
            ObservationBinding::Vl53l1xRange {
                service_instance: "sensor".into(),
                native_site: "range".into(),
                min_range_m: 0.01,
                max_range_m: 4.0,
                fov_rad: 0.6,
            },
            None,
        );
        let workspace = Workspace::new(&model).unwrap();
        let observations = provider
            .observations(&model, &workspace.snapshot().unwrap(), 2_000_000)
            .unwrap();
        let sample = range::RangeSample::decode(observations[0].payload.as_slice()).unwrap();
        assert_eq!(sample.valid, expected.is_some());
        assert!(
            (sample.distance_m - expected.unwrap_or(0.0)).abs() < 0.001,
            "{sample:?}"
        );
        assert_eq!(sample.min_range_m, 0.01);
        assert_eq!(sample.max_range_m, 4.0);
    }
}

// Independent forward ellipsoid conversion checks the inverse used by the
// provider. Its constants use WGS84's defining semi-axis and inverse flattening.
fn ecef([latitude, longitude, height]: [f64; 3]) -> [f64; 3] {
    let flattening = 1.0 / 298.257_223_563;
    let eccentricity = flattening * (2.0 - flattening);
    let latitude = latitude.to_radians();
    let longitude = longitude.to_radians();
    let radius = 6_378_137.0 / (1.0 - eccentricity * latitude.sin().powi(2)).sqrt();
    [
        (radius + height) * latitude.cos() * longitude.cos(),
        (radius + height) * latitude.cos() * longitude.sin(),
        (radius * (1.0 - eccentricity) + height) * latitude.sin(),
    ]
}

#[test]
fn gnss_uses_the_antenna_mount_and_preserves_centimetre_ecef_accuracy() {
    use phoxal_component_zed_f9p as gnss;
    let model = model(
        r#"<mujoco><option timestep="0.002"/>
        <worldbody><body name="sensor"><site name="antenna" pos="0 0 1"/></body></worldbody></mujoco>"#,
    );
    let reference = Georeference::with_yaw(0.0, 0.0, 0.0, [0.0; 3], 0.0).unwrap();
    assert!(reference.project([100_000.001, 0.0, 0.0]).is_err());
    assert!(reference.project([f64::MAX, f64::MAX, f64::MAX]).is_err());
    let mut provider = provider(
        &model,
        vec![provider_requirement("sensor", gnss::ports::GNSS)],
        ObservationBinding::zed_f9p("sensor", "antenna", reference),
        None,
    );
    let observations = provider
        .observations(
            &model,
            &Workspace::new(&model).unwrap().snapshot().unwrap(),
            2_000_000,
        )
        .unwrap();
    let sample = gnss::GnssSample::decode(observations[0].payload.as_slice()).unwrap();
    assert!(sample.latitude_deg.abs() < 1e-8);
    assert!(sample.longitude_deg.abs() < 1e-8);
    assert!((sample.altitude_m - 1.0).abs() < 0.001);
    assert!(sample.position_covariance.is_empty());
    for latitude in [-89.999_f64, -52.0, 0.0, 52.0, 89.999] {
        for longitude in [-180.0_f64, 5.0, 180.0] {
            for yaw in [0.0_f64, 0.7, std::f64::consts::FRAC_PI_2] {
                let reference =
                    Georeference::with_yaw(latitude, longitude, 14.0, [3.0, -4.0, 2.0], yaw)
                        .unwrap();
                let origin = ecef([latitude, longitude, 14.0]);
                let lat = latitude.to_radians();
                let lon = longitude.to_radians();
                for displacement in [
                    [0.0_f64, 0.0, 1.0],
                    [10.0, 0.0, 0.0],
                    [0.0, 10.0, 0.0],
                    [60_000.0, -60_000.0, 50_000.0],
                ] {
                    let actual = ecef(
                        reference
                            .project([
                                3.0 + displacement[0],
                                -4.0 + displacement[1],
                                2.0 + displacement[2],
                            ])
                            .unwrap(),
                    );
                    let east = yaw.cos() * displacement[0] - yaw.sin() * displacement[1];
                    let north = yaw.sin() * displacement[0] + yaw.cos() * displacement[1];
                    let up = displacement[2];
                    let expected = [
                        origin[0] - east * lon.sin() - north * lat.sin() * lon.cos()
                            + up * lat.cos() * lon.cos(),
                        origin[1] + east * lon.cos() - north * lat.sin() * lon.sin()
                            + up * lat.cos() * lon.sin(),
                        origin[2] + north * lat.cos() + up * lat.sin(),
                    ];
                    let error = actual
                        .iter()
                        .zip(expected)
                        .map(|(a, b)| (a - b).powi(2))
                        .sum::<f64>()
                        .sqrt();
                    assert!(
                        error < 0.01,
                        "ECEF error {error} at {latitude},{longitude}, yaw {yaw}, {displacement:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn camera_payloads_preserve_calibrated_projection_row_order_and_metric_depth() {
    use phoxal_component_oak_d_lite as camera;
    let model = model(
        r#"<mujoco>
      <option timestep="0.002"/>
      <visual><global offwidth="96" offheight="72"/><quality shadowsize="128" offsamples="0"/></visual>
      <asset><texture name="checker" type="2d" builtin="checker" width="32" height="32" rgb1="0.2 0.2 0.2" rgb2="0.6 0.6 0.6"/>
        <material name="plane" texture="checker" texrepeat="4 4" emission="1"/>
        <material name="red" rgba="1 0 0 1" emission="1"/><material name="blue" rgba="0 0 1 1" emission="1"/>
      </asset>
      <worldbody><geom type="plane" size="10 10 0.01" material="plane"/>
        <geom type="box" pos="0.5 0.5 0.0005" size="0.1 0.1 0.0005" material="red"/>
        <geom type="box" pos="-0.5 -0.5 0.0005" size="0.1 0.1 0.0005" material="blue"/>
        <camera name="camera" pos="0 0 2" fovy="60" resolution="96 72"/>
      </worldbody></mujoco>"#,
    );
    let requirements = vec![
        provider_requirement("camera", camera::ports::RGB),
        provider_requirement("camera", camera::ports::LEFT_MONO),
        provider_requirement("camera", camera::ports::DEPTH),
    ];
    let cadence = requirements
        .iter()
        .map(|r| {
            (
                (r.service_instance.clone(), r.port.clone()),
                crate::cadence::Cadence::new(500.0, 2_000_000).unwrap(),
            )
        })
        .collect();
    let mut provider = ComponentProvider::new(
        &model,
        ProviderSet::new(requirements).unwrap(),
        [
            ObservationBinding::OakRgbCamera {
                service_instance: "camera".into(),
                native_camera: "camera".into(),
            },
            ObservationBinding::OakLeftMonoCamera {
                service_instance: "camera".into(),
                native_camera: "camera".into(),
            },
            ObservationBinding::OakDepth {
                service_instance: "camera".into(),
                native_camera: "camera".into(),
                range_m: [0.1, 4.0],
            },
        ],
        [],
        cadence,
    )
    .unwrap();
    let observations = provider
        .observations(
            &model,
            &Workspace::new(&model).unwrap().snapshot().unwrap(),
            2_000_000,
        )
        .unwrap();
    let payload = |port: &str| {
        observations
            .iter()
            .find(|o| o.membership.as_ref().unwrap().port == port)
            .unwrap()
            .payload
            .as_slice()
    };
    let rgb = camera::CameraFrame::decode(payload("rgb")).unwrap();
    let mono = camera::CameraFrame::decode(payload("left_mono")).unwrap();
    let depth = camera::DepthFrame::decode(payload("depth")).unwrap();
    assert_eq!(
        (rgb.width_px, rgb.height_px, rgb.encoding),
        (96, 72, camera::ImageEncoding::Rgb8 as i32)
    );
    assert_eq!(
        (mono.width_px, mono.height_px, mono.encoding),
        (96, 72, camera::ImageEncoding::Mono8 as i32)
    );
    assert_eq!(mono.data, rgb_to_mono8(&rgb.data).unwrap());
    assert_eq!((depth.width_px, depth.height_px), (96, 72));
    assert!(depth.depth_mm.iter().all(|d| d.abs_diff(2000) <= 2));
    let focal = 72.0 / (2.0 * 30.0_f64.to_radians().tan());
    for (channel, x, y) in [(0, 0.5, 0.5), (2, -0.5, -0.5)] {
        let pixels = rgb
            .data
            .chunks_exact(3)
            .enumerate()
            .filter(|(_, p)| u16::from(p[channel]) > 2 * u16::from(p[1]) && p[channel] > 80)
            .map(|(i, _)| ((i % 96) as f64, (i / 96) as f64))
            .collect::<Vec<_>>();
        assert!(!pixels.is_empty());
        let centroid = [
            pixels.iter().map(|p| p.0).sum::<f64>() / pixels.len() as f64,
            pixels.iter().map(|p| p.1).sum::<f64>() / pixels.len() as f64,
        ];
        let expected = [47.5 + focal * x / 1.999, 35.5 - focal * y / 1.999];
        assert!(
            (centroid[0] - expected[0]).abs() <= 0.5 && (centroid[1] - expected[1]).abs() <= 0.5,
            "calibration {centroid:?}, expected {expected:?}"
        );
    }
    if let Some(directory) = std::env::var_os("PHOXAL_NUMERICAL_EVIDENCE_DIR") {
        let directory = std::path::PathBuf::from(directory);
        std::fs::create_dir_all(&directory).unwrap();
        let mut ppm = b"P6\n96 72\n255\n".to_vec();
        ppm.extend_from_slice(&rgb.data);
        std::fs::write(directory.join("camera-calibration.ppm"), ppm).unwrap();
        let rows = depth
            .depth_mm
            .chunks(96)
            .map(|row| row.iter().map(u32::to_string).collect::<Vec<_>>().join(","))
            .collect::<Vec<_>>()
            .join("\n");
        std::fs::write(directory.join("camera-depth-mm.csv"), rows).unwrap();
    }
}
