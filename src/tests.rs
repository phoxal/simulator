use crate::config::{Bound, Options, Presentation};
use crate::georeference::georeference;
use crate::mujoco::Model;
use std::ffi::OsString;

fn parse(arguments: &[&str]) -> Result<Options, String> {
    Options::parse(arguments.iter().copied().map(OsString::from))
}

#[test]
fn probe_requires_closed_scene_facts_and_json() {
    let options = parse(&[
        "--probe",
        "--scene",
        "scene.xml",
        "--bundle",
        "bundle",
        "--json",
        "--headless",
    ])
    .expect("probe options");
    assert!(options.probe);
    assert!(options.json);
    assert_eq!(options.presentation, Presentation::Headless);
    assert!(options.bound.is_none());
}

#[test]
fn run_requires_exact_public_identity_and_finite_bound() {
    let options = parse(&[
        "--scene",
        "scene.xml",
        "--bundle",
        "bundle",
        "--headless",
        "--connect",
        "unixsock-stream//tmp/test.sock",
        "--scope",
        "local",
        "--supervisor-id",
        "sim",
        "--run-id",
        "run",
        "--steps",
        "12",
    ])
    .expect("run options");
    assert!(!options.probe);
    assert_eq!(options.bound, Some(Bound::Steps(12)));
    assert_eq!(options.scope.as_deref(), Some("local"));
}

#[test]
fn options_reject_mixed_probe_and_run_modes() {
    assert!(
        parse(&[
            "--probe",
            "--scene",
            "scene.xml",
            "--bundle",
            "bundle",
            "--json",
            "--steps",
            "1",
        ])
        .is_err()
    );
    assert!(
        parse(&[
            "--scene",
            "scene.xml",
            "--bundle",
            "bundle",
            "--headless",
            "--connect",
            "unixsock-stream//tmp/test.sock",
            "--scope",
            "local",
            "--supervisor-id",
            "sim",
            "--run-id",
            "run",
        ])
        .is_err()
    );
    assert!(
        parse(&[
            "--scene",
            "scene.xml",
            "--bundle",
            "bundle",
            "--headless",
            "--desktop",
            "--connect",
            "unixsock-stream//tmp/test.sock",
            "--scope",
            "local",
            "--supervisor-id",
            "sim",
            "--run-id",
            "run",
            "--steps",
            "1",
        ])
        .is_err()
    );
}

#[test]
fn options_require_a_presentation_and_keep_json_probe_only() {
    assert!(
        parse(&[
            "--probe",
            "--scene",
            "scene.xml",
            "--bundle",
            "bundle",
            "--json"
        ])
        .is_err()
    );
    assert!(
        parse(&[
            "--scene",
            "scene.xml",
            "--bundle",
            "bundle",
            "--connect",
            "unixsock-stream//tmp/test.sock",
            "--scope",
            "local",
            "--supervisor-id",
            "sim",
            "--run-id",
            "run",
            "--steps",
            "1",
        ])
        .is_err()
    );
    assert!(
        parse(&[
            "--scene",
            "scene.xml",
            "--bundle",
            "bundle",
            "--headless",
            "--json",
            "--connect",
            "unixsock-stream//tmp/test.sock",
            "--scope",
            "local",
            "--supervisor-id",
            "sim",
            "--run-id",
            "run",
            "--steps",
            "1",
        ])
        .is_err()
    );
}

#[test]
fn duration_must_be_an_exact_quantum_multiple() {
    assert_eq!(Bound::Duration(0.02).steps(10_000_000).unwrap(), 2);
    assert!(Bound::Duration(0.015).steps(10_000_000).is_err());
}

#[test]
fn zed_georeference_reads_scene_owned_mjcf_metadata() {
    let model = Model::from_xml(
        r#"<mujoco model="scene">
                <custom>
                    <numeric name="phoxal_georeference" data="52 5 10 1 2 3 0.25"/>
                    <text name="phoxal_georeference_axes" data="ENU"/>
                    <text name="phoxal_georeference_datum" data="WGS84_ELLIPSOIDAL"/>
                </custom>
                <worldbody/>
            </mujoco>"#,
    )
    .expect("scene model");
    let georeference = georeference(&model, "gnss").expect("scene georeference");
    assert_eq!(georeference.latitude_deg, 52.0);
    assert_eq!(georeference.longitude_deg, 5.0);
    assert_eq!(georeference.altitude_m, 10.0);
    assert_eq!(georeference.origin_m, [1.0, 2.0, 3.0]);
    assert_eq!(georeference.yaw_rad, 0.25);
}

#[test]
fn zed_georeference_rejects_missing_scene_metadata() {
    let model =
        Model::from_xml(r#"<mujoco model="scene"><worldbody/></mujoco>"#).expect("scene model");
    let error = georeference(&model, "gnss").expect_err("metadata is required");
    assert!(error.contains("phoxal_georeference"));
}
