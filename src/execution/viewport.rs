//! Worker-owned native view, selection and rendering on copied state only.
use crate::desktop::{
    DisplayState,
    scene::{FrameIdentity, PresentedView, SceneAction, SceneEpoch, SceneResult, ViewportFrame},
};
use crate::mujoco::{Model, NativeSelection, StateSnapshot, ViewCamera, Workspace};
use std::sync::{Arc, Mutex};

pub(super) struct Viewport {
    workspace: Workspace,
    pub camera: ViewCamera,
    epoch: SceneEpoch,
    serial: u64,
    selected: Option<NativeSelection>,
}
impl Viewport {
    pub fn new(model: &Model, execution: &str, generation: u64) -> Result<Self, String> {
        let workspace = Workspace::new(model).map_err(|e| e.to_string())?;
        Ok(Self {
            camera: workspace.default_view_camera(),
            workspace,
            epoch: SceneEpoch {
                execution: execution.into(),
                model: model.identity(),
                generation,
            },
            serial: 0,
            selected: None,
        })
    }
    pub fn reset(
        &mut self,
        generation: u64,
        display: &Arc<Mutex<DisplayState>>,
    ) -> Result<(), String> {
        self.epoch.generation = generation;
        self.selected = None;
        let mut state = display.lock().map_err(|_| "desktop state lock poisoned")?;
        state.frame = None;
        state.presented = None;
        state.pending_camera = None;
        state.pending_scene = None;
        state.selection = None;
        state.scene_result = None;
        Ok(())
    }
    pub fn input(&mut self, display: &Arc<Mutex<DisplayState>>) -> Result<bool, String> {
        let (camera, request, presented) = {
            let mut state = display.lock().map_err(|_| "desktop state lock poisoned")?;
            (
                state.pending_camera.take(),
                state.pending_scene.take(),
                state.presented.clone(),
            )
        };
        let mut changed = false;
        if let Some(request) = camera
            && request.epoch == self.epoch
            && crate::desktop::viewport::valid_camera(request.camera)
        {
            self.camera = request.camera;
            changed = true;
        }
        if let Some(request) = request {
            let valid = presented
                .as_ref()
                .is_some_and(|view| request.matches(&self.epoch, view, self.camera));
            if valid {
                let view = presented.ok_or("presented view missing")?;
                self.sync(&view.snapshot)?;
                match request.action {
                    SceneAction::Pick(xy) => {
                        self.selected = self
                            .workspace
                            .pick_viewport(view.camera, view.resolution, xy)
                            .map_err(|e| e.to_string())?
                    }
                    SceneAction::SelectBody(body) => {
                        if let Some(point) = view.snapshot.body_positions().get(body) {
                            self.selected = Some(NativeSelection {
                                body,
                                geom: None,
                                point: *point,
                            });
                        }
                    }
                    SceneAction::Clear => self.selected = None,
                    SceneAction::Focus => {
                        if let Some(selected) = &self.selected {
                            self.camera = self
                                .workspace
                                .body_view(selected.body, self.camera)
                                .map_err(|e| e.to_string())?;
                        }
                    }
                    SceneAction::DefaultCamera => {
                        self.camera = self.workspace.default_view_camera()
                    }
                }
                changed = true;
            }
            display
                .lock()
                .map_err(|_| "desktop state lock poisoned")?
                .scene_result = Some(SceneResult {
                frame: request.frame,
                stale: !valid,
                camera: (valid
                    && matches!(
                        request.action,
                        SceneAction::Focus | SceneAction::DefaultCamera
                    ))
                .then_some(self.camera),
            });
        }
        Ok(changed)
    }
    pub fn drag_anchor(
        &mut self,
        begin: &crate::desktop::scene::DragBegin,
        presented: &PresentedView,
    ) -> Result<(usize, [f64; 3], ViewCamera), String> {
        if begin.frame.epoch != self.epoch
            || begin.frame != presented.identity
            || presented.camera != self.camera
        {
            return Err("drag begins from a stale presented view".into());
        }
        self.sync(&presented.snapshot)?;
        let hit = self
            .workspace
            .pick_viewport(presented.camera, presented.resolution, begin.xy)
            .map_err(|e| e.to_string())?
            .ok_or("drag missed native surface")?;
        // An explicitly selected ancestor can be manipulated by grabbing its
        // visible descendant, with that ancestor still named in the inspector.
        // A picked attached body is never silently promoted to a free ancestor.
        let body = self
            .selected
            .as_ref()
            .filter(|selected| self.workspace.descendant_of(hit.body, selected.body))
            .map_or(hit.body, |selected| selected.body);
        let anchor = self.workspace.local_anchor(body, hit.point)?;
        self.selected = Some(NativeSelection { body, ..hit });
        Ok((body, anchor, presented.camera))
    }
    fn sync(&mut self, snapshot: &StateSnapshot) -> Result<(), String> {
        if snapshot.model_identity() != self.epoch.model {
            return Err("viewport snapshot belongs to another model".into());
        }
        self.workspace
            .set_qpos(snapshot.qpos())
            .map_err(|e| e.to_string())?;
        self.workspace
            .set_qvel(snapshot.qvel())
            .map_err(|e| e.to_string())?;
        self.workspace.forward().map_err(|e| e.to_string())
    }
    pub fn render(&mut self, snapshot: &StateSnapshot) -> Result<ViewportFrame, String> {
        self.sync(snapshot)?;
        let [width, height] = self.workspace.framebuffer_resolution();
        let ratio = (1024.0 / width as f64).min(768.0 / height as f64).min(1.0);
        let resolution = [
            (width as f64 * ratio) as usize,
            (height as f64 * ratio) as usize,
        ];
        let image = self
            .workspace
            .render_viewport_selected(
                self.camera,
                resolution,
                self.selected.as_ref().map(|hit| hit.body),
            )
            .map_err(|e| e.to_string())?;
        self.serial = self
            .serial
            .checked_add(1)
            .ok_or("viewport frame identity exhausted")?;
        Ok(ViewportFrame {
            image,
            view: Arc::new(PresentedView {
                identity: FrameIdentity {
                    epoch: self.epoch.clone(),
                    serial: self.serial,
                },
                camera: self.camera,
                resolution,
                snapshot: snapshot.clone(),
            }),
        })
    }
    pub fn publish(&self, state: &mut DisplayState) {
        if state.bodies.is_empty() {
            state.bodies = self.workspace.scene_bodies().into();
        }
        state.camera = Some(self.camera);
        state.selection = self.selected.clone();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::desktop::scene::{CameraRequest, SceneRequest};
    const XML: &str = r#"<mujoco><visual><global offwidth="320" offheight="240"/></visual><worldbody>
        <body name="fixed" pos="3 0 0"><geom type="sphere" size="0.2"/></body>
        <body name="ball"><freejoint/><geom type="sphere" size="0.4" rgba="0.2 0.6 0.9 1"/></body>
        </worldbody></mujoco>"#;
    fn prepared() -> (Viewport, StateSnapshot, Arc<Mutex<DisplayState>>) {
        let model = Model::from_xml(XML).unwrap();
        let snapshot = Workspace::new(&model).unwrap().snapshot().unwrap();
        let mut owner = Viewport::new(&model, "execution-a", 0).unwrap();
        owner.camera = ViewCamera {
            look_at: [0.0; 3],
            distance: 3.0,
            azimuth: 90.0,
            elevation: 0.0,
        };
        let frame = owner.render(&snapshot).unwrap();
        let state = Arc::new(Mutex::new(DisplayState {
            presented: Some(frame.view),
            ..Default::default()
        }));
        (owner, snapshot, state)
    }
    fn request(display: &Arc<Mutex<DisplayState>>, action: SceneAction) {
        let mut state = display.lock().unwrap();
        state.pending_scene = Some(SceneRequest {
            frame: state.presented.as_ref().unwrap().identity.clone(),
            action,
        });
    }
    #[test]
    fn native_drag_begin_uses_exact_frame_and_selected_identity() {
        let (mut owner, _, display) = prepared();
        request(&display, SceneAction::SelectBody(2));
        owner.input(&display).unwrap();
        let view = display.lock().unwrap().presented.clone().unwrap();
        let begin = crate::desktop::scene::DragBegin {
            id: 1,
            frame: view.identity.clone(),
            xy: [0.5; 2],
            received: std::time::Instant::now(),
        };
        let (body, anchor, camera) = owner.drag_anchor(&begin, &view).unwrap();
        assert_eq!(body, 2);
        assert_eq!(camera, view.camera);
        assert!(anchor.iter().all(|v| v.is_finite()));
        for alter in 0..5 {
            let mut stale = begin.clone();
            match alter {
                0 => stale.frame.epoch.execution = "old".into(),
                1 => stale.frame.epoch.generation += 1,
                2 => stale.frame.epoch.model = crate::mujoco::ModelIdentity([8; 32]),
                3 => stale.frame.serial += 1,
                _ => stale.xy = [f64::NAN, 0.5],
            }
            assert!(owner.drag_anchor(&stale, &view).is_err());
        }
        owner.camera.azimuth += 1.0;
        assert!(owner.drag_anchor(&begin, &view).is_err());
    }
    #[test]
    fn native_viewport_selects_highlights_and_clears_without_mutating_state() {
        let (mut owner, snapshot, display) = prepared();
        let baseline = owner.render(&snapshot).unwrap();
        request(&display, SceneAction::Pick([0.5, 0.5]));
        assert!(owner.input(&display).unwrap());
        let selected = owner.selected.as_ref().unwrap();
        assert_eq!(selected.body, 2);
        assert_eq!(selected.geom, Some(1));
        assert!(selected.point.iter().all(|v| v.is_finite()));
        let highlight = owner.render(&snapshot).unwrap();
        assert_ne!(baseline.image.rgb(), highlight.image.rgb());
        assert_eq!(snapshot, owner.workspace.snapshot().unwrap());
        let bodies = owner.workspace.scene_bodies();
        assert_eq!(bodies[1].mobility.label(), "Fixed");
        assert_eq!(bodies[2].mobility.label(), "Free joint");
        request(&display, SceneAction::Focus);
        owner.input(&display).unwrap();
        assert_eq!(owner.camera.look_at, [0.0; 3]);
        assert!(owner.camera.distance < 3.0);
        // A camera-changed old frame is rejected, not silently used for a pick.
        request(&display, SceneAction::Pick([0.0, 0.0]));
        assert!(!owner.input(&display).unwrap());
        assert!(display.lock().unwrap().scene_result.as_ref().unwrap().stale);
        display.lock().unwrap().presented = Some(owner.render(&snapshot).unwrap().view);
        request(&display, SceneAction::Pick([0.0, 0.0]));
        owner.input(&display).unwrap();
        assert!(owner.selected.is_none());
        owner.camera = baseline.view.camera;
        assert_eq!(baseline.image, owner.render(&snapshot).unwrap().image);
        // Selection only changes abstract renderer geoms, never the authored model.
        let independent = Workspace::new(owner.workspace.model())
            .unwrap()
            .render_viewport(owner.camera, [320, 240])
            .unwrap();
        assert_eq!(independent, baseline.image);
    }
    #[test]
    fn native_pick_uses_the_presented_snapshot_even_after_newer_rendering() {
        let (mut owner, _snapshot, display) = prepared();
        let mut authoritative = Workspace::new(owner.workspace.model()).unwrap();
        let mut qpos = authoritative.snapshot().unwrap().qpos().to_vec();
        qpos[0] = 10.0;
        authoritative.set_qpos(&qpos).unwrap();
        authoritative.forward().unwrap();
        let later = authoritative.snapshot().unwrap();
        owner.render(&later).unwrap();
        // The UI has not presented the newer image. The original click must
        // restore and query the exact earlier copy, not this later world state.
        request(&display, SceneAction::Pick([0.5, 0.5]));
        owner.input(&display).unwrap();
        assert_eq!(owner.selected.as_ref().unwrap().body, 2);
        assert_eq!(authoritative.snapshot().unwrap(), later);
        assert_eq!(later.qpos()[0], 10.0);
    }

    #[test]
    fn frame_epoch_and_camera_fences_reject_stale_and_coalesce_input() {
        let (mut owner, snapshot, display) = prepared();
        for alter in 0..4 {
            request(&display, SceneAction::Pick([0.5, 0.5]));
            let mut state = display.lock().unwrap();
            let identity = &mut state.pending_scene.as_mut().unwrap().frame;
            match alter {
                0 => identity.epoch.generation += 1,
                1 => identity.epoch.execution = "old-execution".into(),
                2 => identity.serial += 1,
                _ => identity.epoch.model = Model::from_xml("<mujoco/>").unwrap().identity(),
            }
            drop(state);
            assert!(!owner.input(&display).unwrap());
            assert!(owner.selected.is_none());
        }
        let mut camera = owner.camera;
        for index in 0..10000 {
            camera.azimuth = f64::from(index % 360);
            display.lock().unwrap().pending_camera = Some(CameraRequest {
                epoch: owner.epoch.clone(),
                camera,
            });
        }
        owner.input(&display).unwrap();
        assert_eq!(owner.camera, camera);
        assert!(display.lock().unwrap().pending_camera.is_none());
        display.lock().unwrap().presented = Some(owner.render(&snapshot).unwrap().view);
        request(&display, SceneAction::SelectBody(1));
        owner.input(&display).unwrap();
        assert_eq!(owner.selected.as_ref().unwrap().body, 1);
        request(&display, SceneAction::Clear);
        owner.input(&display).unwrap();
        assert!(owner.selected.is_none());
        let old = display.lock().unwrap().presented.clone().unwrap();
        owner.reset(1, &display).unwrap();
        assert!(display.lock().unwrap().presented.is_none());
        // Even explicitly reintroduced old UI data cannot cross reset.
        display.lock().unwrap().presented = Some(old.clone());
        request(&display, SceneAction::SelectBody(2));
        assert!(!owner.input(&display).unwrap());
        let replacement = Viewport::new(owner.workspace.model(), "execution-b", 0).unwrap();
        assert!(display.lock().unwrap().pending_scene.is_none());
        let stale = SceneRequest {
            frame: old.identity.clone(),
            action: SceneAction::Clear,
        };
        assert!(!stale.matches(&replacement.epoch, &old, replacement.camera));
    }
}
