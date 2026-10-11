//! Camera-only tracking tied to a presented native identity, never execution input.
use super::*;
pub(super) struct Target {
    pub epoch: scene::SceneEpoch,
    pub body: usize,
    pub label: String,
}
pub(super) fn is_descendant(bodies: &[NativeBody], mut body: usize, root: usize) -> bool {
    for _ in 0..bodies.len() {
        if body == root {
            return true;
        }
        let Some(parent) = bodies.get(body).and_then(|b| b.parent) else {
            return false;
        };
        body = parent;
    }
    false
}
pub(super) fn candidate(state: &RunState) -> Option<(usize, String)> {
    let body = state.selection.as_ref()?.body;
    let selected = state.bodies.get(body)?;
    if let Some(root) = state.robot_root
        && is_descendant(&state.bodies, body, root)
    {
        return Some((
            root,
            state
                .document
                .robot
                .clone()
                .unwrap_or_else(|| state.robot_name.clone()),
        ));
    }
    if selected.mobility == BodyMobility::Fixed || body == 0 {
        return None;
    }
    Some((
        body,
        if selected.name.is_empty() {
            format!("Body {body}")
        } else {
            selected.name.clone()
        },
    ))
}
impl Target {
    pub fn track(&mut self, view: &scene::PresentedView, camera: &mut ViewCamera) -> bool {
        if self.epoch.model != view.identity.epoch.model
            || self.epoch.execution != view.identity.epoch.execution
        {
            return false;
        }
        let Some(position) = view.snapshot.body_positions().get(self.body) else {
            return false;
        };
        if !position.iter().all(|x| x.is_finite()) {
            return false;
        }
        self.epoch.generation = view.identity.epoch.generation;
        camera.look_at = *position;
        true
    }
}

#[cfg(all(test, native_qualification))]
mod tests {
    use super::*;
    #[test]
    fn verified_rover_root_follow_is_camera_only_and_identity_fenced() {
        let build = std::env::var_os("PHOXAL_QUALIFICATION_BUILD").expect("actual prepared rover");
        let bundle = crate::bundle::BundleFacts::load(std::path::Path::new(&build)).unwrap();
        let closed = crate::document::Document::default_scene()
            .unwrap()
            .definition
            .scene
            .close()
            .unwrap();
        let model = crate::composition::compose_closed(
            closed,
            &bundle,
            "robot_mount",
            Some(([0.; 3], [1., 0., 0., 0.])),
        )
        .unwrap();
        let root = model
            .robot_root()
            .expect("verified native composition root");
        let workspace = crate::mujoco::Workspace::new(&model).unwrap();
        let mut state = RunState {
            snapshot: crate::app::RunSnapshot {
                bodies: workspace.scene_bodies().into(),
                robot_root: Some(root),
                document: crate::app::world::DocumentView {
                    robot: Some(bundle.robot_id.clone()),
                    ..Default::default()
                },
                ..Default::default()
            },
            requests: Default::default(),
        };
        let child = state
            .bodies
            .iter()
            .find(|b| b.parent == Some(root))
            .unwrap()
            .id;
        state.selection = Some(crate::mujoco::NativeSelection {
            body: child,
            geom: None,
            point: [0.; 3],
        });
        assert_eq!(candidate(&state).unwrap().0, root);
        let mut tracked = crate::mujoco::Scene::new(model.clone()).unwrap();
        let mut control = crate::mujoco::Scene::new(model.clone()).unwrap();
        let epoch = scene::SceneEpoch {
            execution: "qualified".into(),
            model: model.identity(),
            generation: 0,
        };
        let mut target = Target {
            epoch: epoch.clone(),
            body: root,
            label: bundle.robot_id.clone(),
        };
        let mut camera = workspace.default_view_camera();
        let original = camera;
        for serial in 1..=20 {
            tracked.step().unwrap();
            control.step().unwrap();
            let snapshot = tracked.snapshot().unwrap();
            let view = scene::PresentedView {
                identity: scene::FrameIdentity {
                    epoch: epoch.clone(),
                    serial,
                },
                camera,
                resolution: [320, 240],
                snapshot: snapshot.clone(),
            };
            assert!(target.track(&view, &mut camera));
            assert_eq!(camera.look_at, snapshot.body_positions()[root]);
            assert_eq!(
                snapshot,
                control.snapshot().unwrap(),
                "camera tracking changed native execution"
            );
        }
        assert_eq!(camera.distance, original.distance);
        assert_eq!(camera.azimuth, original.azimuth);
        assert_eq!(camera.elevation, original.elevation);
        state.selection = Some(crate::mujoco::NativeSelection {
            body: 0,
            geom: None,
            point: [0.; 3],
        });
        assert!(candidate(&state).is_none());
        assert_eq!(target.body, root);
        let mut view = scene::PresentedView {
            identity: scene::FrameIdentity {
                epoch: epoch.clone(),
                serial: 21,
            },
            camera,
            resolution: [320, 240],
            snapshot: tracked.snapshot().unwrap(),
        };
        camera.azimuth += 30.;
        camera.distance *= 0.5;
        assert!(target.track(&view, &mut camera));
        assert_eq!(tracked.snapshot().unwrap(), control.snapshot().unwrap());
        tracked.reset().unwrap();
        view.snapshot = tracked.snapshot().unwrap();
        view.identity.epoch.generation = 1;
        assert!(target.track(&view, &mut camera));
        assert_eq!(target.epoch.generation, 1);
        let last = camera;
        view.identity.epoch.execution = "replacement".into();
        assert!(!target.track(&view, &mut camera));
        assert_eq!(camera, last);
        view.identity.epoch.execution = epoch.execution;
        view.identity.epoch.model = crate::mujoco::ModelIdentity([9; 32]);
        assert!(!target.track(&view, &mut camera));
        assert_eq!(camera, last);
        eprintln!(
            "FOLLOW actual rover root={root} child={child} boundaries20 physics-identical; pause/orbit/zoom/reset/replacement fences qualified"
        );
    }
}
