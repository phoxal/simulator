#![cfg(feature = "rendering")]

use std::path::Path;

use crate::mujoco::{Model, Workspace};

#[test]
fn native_camera_renders_rgb_and_metric_depth_from_a_private_workspace() {
    let model = Model::from_file(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("src/mujoco/tests/fixtures/rendering/scene.xml"),
    )
    .expect("rendering fixture model");
    let camera = model
        .camera("camera")
        .expect("camera lookup")
        .expect("rendering fixture camera");
    let mut workspace = Workspace::new(&model).expect("private rendering workspace");
    let rendered = workspace
        .render_camera(camera)
        .expect("native camera render");

    assert_eq!(rendered.resolution(), [4, 3]);
    assert_eq!(rendered.rgb().len(), 4 * 3 * 3);
    assert_eq!(rendered.depth_m().len(), 4 * 3);
    assert!(rendered.depth_m().iter().all(|depth| depth.is_finite()));
    assert!(
        rendered
            .depth_m()
            .iter()
            .all(|depth| (*depth - 1.0).abs() < 0.001),
        "plane depth: {:?}",
        rendered.depth_m()
    );
    assert!(
        rendered
            .rgb()
            .as_chunks::<3>()
            .0
            .iter()
            .all(|pixel| pixel[0] > pixel[1] && pixel[0] > pixel[2]),
        "red plane RGB: {:?}",
        rendered.rgb()
    );
    let mut second = Workspace::new(&model).expect("second independent renderer");
    let other = second.render_camera(camera).expect("second render");
    assert_eq!(rendered, other);
    drop(second);
    assert_eq!(
        rendered,
        workspace
            .render_camera(camera)
            .expect("first renderer survives second context destruction")
    );
}

#[test]
fn oversized_framebuffer_is_rejected_before_native_allocation() {
    let model = Model::from_xml(r#"<mujoco><visual><global offwidth="8192" offheight="8192"/></visual><worldbody><camera name="camera" resolution="4 3"/></worldbody></mujoco>"#).unwrap();
    let camera = model.camera("camera").unwrap().unwrap();
    let mut workspace = Workspace::new(&model).unwrap();
    assert!(
        workspace
            .render_camera(camera)
            .unwrap_err()
            .to_string()
            .contains("pixel budget")
    );
}

#[test]
fn oversized_capture_is_rejected_before_native_allocation() {
    let model = Model::from_xml(r#"<mujoco><worldbody/></mujoco>"#).unwrap();
    let mut workspace = Workspace::new(&model).unwrap();
    let view = workspace.default_view_camera();
    assert!(workspace.render_viewport(view, [usize::MAX, 2]).is_err());
    assert!(workspace.render_viewport(view, [0, 1]).is_err());
}

#[test]
fn no_hit_depth_is_invalid_instead_of_a_fabricated_far_plane_measurement() {
    let model = Model::from_xml(
        r#"<mujoco><worldbody><camera name="camera" resolution="4 3"/></worldbody></mujoco>"#,
    )
    .unwrap();
    let camera = model.camera("camera").unwrap().unwrap();
    let mut workspace = Workspace::new(&model).unwrap();
    let rendered = workspace.render_camera(camera).unwrap();
    assert!(
        rendered.depth_m().iter().all(|depth| *depth == 0.0),
        "{:?}",
        rendered.depth_m()
    );
}

#[test]
fn cameras_with_different_resolutions_share_a_workspace_without_stale_pixels() {
    let model = Model::from_xml(
        r#"<mujoco><worldbody>
        <geom type="plane" size="10 10 0.1" rgba="1 0 0 1"/>
        <camera name="large" pos="0 0 2" resolution="64 48"/>
        <camera name="small" pos="0 0 1" resolution="8 6"/>
        </worldbody></mujoco>"#,
    )
    .unwrap();
    let mut workspace = Workspace::new(&model).unwrap();
    for _ in 0..3 {
        for (name, resolution, expected_depth) in [("large", [64, 48], 2.0), ("small", [8, 6], 1.0)]
        {
            let image = workspace
                .render_camera(model.camera(name).unwrap().unwrap())
                .unwrap();
            assert_eq!(image.resolution(), resolution);
            assert!(
                image
                    .depth_m()
                    .iter()
                    .all(|depth| (*depth - expected_depth).abs() < 0.001)
            );
        }
    }
}

#[test]
fn native_root_body_highlights_descendants_without_changing_materials() {
    let model = Model::from_xml(r#"<mujoco><visual><global offwidth="64" offheight="48"/></visual>
        <worldbody><body name="root"><freejoint/><body name="child"><geom type="sphere" size="0.4" rgba="0.2 0.4 0.8 1"/></body></body></worldbody></mujoco>"#).unwrap();
    let mut workspace = Workspace::new(&model).unwrap();
    let view = crate::mujoco::ViewCamera {
        look_at: [0.0; 3],
        distance: 3.0,
        azimuth: 90.0,
        elevation: 0.0,
    };
    let state = workspace.snapshot().unwrap();
    // Compare warm native readbacks. The first pass differs slightly even
    // between two unselected renders on this driver, independently of highlight.
    workspace.render_viewport(view, [64, 48]).unwrap();
    let ordinary = workspace.render_viewport(view, [64, 48]).unwrap();
    let selected = workspace
        .render_viewport_selected(view, [64, 48], Some(1))
        .unwrap();
    assert_ne!(ordinary.rgb(), selected.rgb());
    assert_eq!(state, workspace.snapshot().unwrap());
    let restored = workspace.render_viewport(view, [64, 48]).unwrap();
    assert_eq!(ordinary, restored);
}

/// The bundled default must open legible: a readable checker ground with
/// balanced lighting at the authored default framing, occupying a real
/// share of the view instead of a flat over-lit or missed surface.
/// This is an explicit native qualification check: the default non-graphical
/// contract does not render pixels.
/// Bounded same-code checker trial comparison: renders the frozen
/// baseline (texrepeat 4 4, fixture copy) and the trial default
/// (texrepeat 3 3, bundled resource) at the identical authored default
/// camera and resolution, writes both frames as PPM files under
/// target/checker-trial/ and prints luminance statistics for QA/design
/// comparison against matched native captures. Offscreen metrics do not
/// establish native brightness or fidelity acceptance.
#[test]
#[cfg(native_qualification)]
fn checker_trial_renders_baseline_and_trial_for_matched_comparison() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("target/checker-trial");
    std::fs::create_dir_all(&dir).unwrap();
    let write_ppm = |name: &str, rendered: &crate::mujoco::RenderedCamera| {
        let [w, h] = rendered.resolution();
        let mut data = format!(
            "P6
{w} {h}
255
"
        )
        .into_bytes();
        data.extend_from_slice(rendered.rgb());
        std::fs::write(dir.join(name), data).unwrap();
    };
    let stats = |rendered: &crate::mujoco::RenderedCamera| {
        let rgb = rendered.rgb();
        let luma = |i: usize| {
            (f64::from(rgb[3 * i]) + f64::from(rgb[3 * i + 1]) + f64::from(rgb[3 * i + 2])) / 3.0
        };
        let pixels: Vec<f64> = (0..rgb.len() / 3).map(luma).collect();
        let mean = pixels.iter().sum::<f64>() / pixels.len() as f64;
        let variance =
            pixels.iter().map(|p| (p - mean) * (p - mean)).sum::<f64>() / pixels.len() as f64;
        (mean, variance)
    };
    let baseline_model = Model::from_file(
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("src/mujoco/tests/fixtures/rendering/checker-baseline.xml"),
    )
    .expect("baseline checker fixture");
    let mut baseline_ws = Workspace::new(&baseline_model).unwrap();
    let baseline_view = baseline_ws.default_view_camera();
    let baseline_render = baseline_ws
        .render_viewport(baseline_view, [640, 480])
        .unwrap();
    let (baseline_mean, baseline_var) = stats(&baseline_render);
    eprintln!("CHECKER_TRIAL baseline mean={baseline_mean:.1} variance={baseline_var:.1}");
    write_ppm("baseline.ppm", &baseline_render);

    let trial_model = Model::from_xml(crate::document::DEFAULT_SCENE).unwrap();
    let mut trial_ws = Workspace::new(&trial_model).unwrap();
    let trial_view = trial_ws.default_view_camera();
    let trial_render = trial_ws.render_viewport(trial_view, [640, 480]).unwrap();
    let (trial_mean, trial_var) = stats(&trial_render);
    eprintln!("CHECKER_TRIAL trial mean={trial_mean:.1} variance={trial_var:.1}");
    write_ppm("trial.ppm", &trial_render);

    // Both definitions remain legible checker grounds offscreen.
    assert!(trial_var > 800.0, "trial lost checker contrast");
    assert!(baseline_var > 800.0, "baseline lost checker contrast");
}

#[test]
#[cfg(native_qualification)]
fn bundled_default_scene_renders_a_legible_checker_ground() {
    let model = Model::from_xml(crate::document::DEFAULT_SCENE).unwrap();
    let mut workspace = Workspace::new(&model).unwrap();
    let view = workspace.default_view_camera();
    let rendered = workspace.render_viewport(view, [640, 480]).unwrap();
    let rgb = rendered.rgb();
    let luma = |i: usize| {
        (f64::from(rgb[3 * i]) + f64::from(rgb[3 * i + 1]) + f64::from(rgb[3 * i + 2])) / 3.0
    };
    let pixels: Vec<f64> = (0..640 * 480).map(luma).collect();
    let mean = pixels.iter().sum::<f64>() / pixels.len() as f64;
    let variance =
        pixels.iter().map(|p| (p - mean) * (p - mean)).sum::<f64>() / pixels.len() as f64;
    let depth = rendered.depth_m();
    let occupied = depth[..depth.len() / 2]
        .iter()
        .filter(|depth| **depth > 0.0 && **depth < 10.0)
        .count();
    assert!(
        pixels.iter().copied().fold(f64::INFINITY, f64::min) < 90.0,
        "expected dark checker cells, mean {mean:.1}, variance {variance:.1}"
    );
    assert!(
        pixels.iter().copied().fold(f64::NEG_INFINITY, f64::max) > 150.0,
        "expected light checker cells, mean {mean:.1}, variance {variance:.1}"
    );
    assert!(
        variance > 800.0,
        "expected checker contrast, mean {mean:.1}, variance {variance:.1}"
    );
    assert!(
        occupied * 4 > depth.len() / 2,
        "expected ground depth occupancy, occupied {occupied} of {}",
        depth.len() / 2
    );
    assert!(mean < 220.0, "expected balanced lighting, mean {mean:.1}");
}
