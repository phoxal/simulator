//! Simulator-owned translation perturbation, never a second authoritative world.
use super::{Scene, Workspace};
use crate::mujoco::ViewCamera;
use crate::native_binding::{
    mujoco_c,
    wrappers::mj_visualization::{MjtPertBit, MjvCamera, MjvOption, MjvPerturb, MjvScene},
};

pub(crate) struct NativeDrag {
    perturb: MjvPerturb,
    origin: [f64; 3],
    pub deadline: std::time::Instant,
    pub force: bool,
    pub scale: f64,
    pub right: [f64; 3],
    pub up: [f64; 3],
}
impl Workspace {
    pub(crate) fn descendant_of(&self, body: usize, ancestor: usize) -> bool {
        let parents = self.data.model().body_parentid();
        let mut current = body;
        while current > 0 && current < parents.len() {
            if current == ancestor {
                return true;
            }
            current = parents[current] as usize;
        }
        false
    }
    pub(crate) fn local_anchor(&self, body: usize, point: [f64; 3]) -> Result<[f64; 3], String> {
        if body >= self.data.xpos().len() || !point.iter().all(|x| x.is_finite()) {
            return Err("invalid native anchor".into());
        }
        let mut perturb = MjvPerturb {
            select: body as i32,
            ..Default::default()
        };
        perturb.update_local_pos(&point, &self.data);
        Ok(perturb.localpos)
    }
    fn drag(
        &mut self,
        body: usize,
        anchor: [f64; 3],
        view: ViewCamera,
    ) -> Result<NativeDrag, String> {
        let model = self.data.model();
        if body == 0 || body >= model.nbody() as usize || model.body_weldid()[body] == 0 {
            return Err("selected native body is fixed".into());
        }
        if !anchor.iter().all(|x| x.is_finite() && x.abs() < 1e6) {
            return Err("invalid native anchor".into());
        }
        let mut scene = MjvScene::new(model, model.ngeom() as usize + 100);
        let mut camera = MjvCamera::new_free(model);
        camera.lookat = view.look_at;
        camera.distance = view.distance;
        camera.azimuth = view.azimuth;
        camera.elevation = view.elevation;
        scene.update(
            &mut self.data,
            &MjvOption::default(),
            &MjvPerturb::default(),
            &mut camera,
        );
        let mut perturb = MjvPerturb {
            select: body as i32,
            localpos: anchor,
            ..Default::default()
        };
        perturb.start(MjtPertBit::mjPERT_TRANSLATE, &mut self.data, &scene);
        let mut forward = [0.0; 3];
        let mut up = [0.0; 3];
        // SAFETY: initialized native abstract scene, bounded three-element outputs.
        unsafe {
            mujoco_c::mjv_cameraInModel(std::ptr::null_mut(), &mut forward, &mut up, scene.ffi());
        }
        let right = [
            forward[1] * up[2] - forward[2] * up[1],
            forward[2] * up[0] - forward[0] * up[2],
            forward[0] * up[1] - forward[1] * up[0],
        ];
        if !perturb.scale.is_finite()
            || perturb.scale <= 0.0
            || !perturb.localmass.is_finite()
            || perturb.localmass <= 0.0
        {
            return Err("invalid native perturbation scale or effective mass".into());
        }
        Ok(NativeDrag {
            force: false,
            deadline: std::time::Instant::now() + super::DRAG_LIVENESS,
            origin: perturb.refselpos,
            scale: perturb.scale,
            right,
            up,
            perturb,
        })
    }
}
impl Scene {
    pub(crate) fn begin_drag(
        &mut self,
        body: usize,
        anchor: [f64; 3],
        camera: ViewCamera,
        paused: bool,
    ) -> Result<NativeDrag, String> {
        if self.phase == super::ScenePhase::Failed {
            return Err("native scene failed".into());
        }
        if paused {
            self.free_joint(body)?;
        }
        let mut drag = self.workspace.drag(body, anchor, camera)?;
        drag.force = !paused;
        Ok(drag)
    }
    fn free_joint(&self, body: usize) -> Result<usize, String> {
        let model = self.workspace.data.model();
        if body >= model.nbody() as usize || model.body_jntnum()[body] != 1 {
            return Err("paused translation requires the selected body's own free joint".into());
        }
        let joint = model.body_jntadr()[body] as usize;
        if model.jnt_type()[joint] != mujoco_c::mjtJoint::mjJNT_FREE {
            return Err("paused translation requires the selected body's own free joint".into());
        }
        for (index, kind) in model.eq_type().iter().enumerate() {
            if !self.workspace.data.eq_active()[index]
                || !matches!(
                    kind,
                    mujoco_c::mjtEq::mjEQ_WELD | mujoco_c::mjtEq::mjEQ_CONNECT
                )
            {
                continue;
            }
            for object in [model.eq_obj1id()[index], model.eq_obj2id()[index]] {
                if object < 0 {
                    continue;
                }
                let linked = match model.eq_objtype()[index] {
                    mujoco_c::mjtObj::mjOBJ_BODY => object as usize,
                    mujoco_c::mjtObj::mjOBJ_SITE => model.site_bodyid()[object as usize] as usize,
                    _ => continue,
                };
                if self.workspace.descendant_of(linked, body) {
                    return Err("paused translation refuses an active native weld/connect constraint on the selected subtree".into());
                }
            }
        }
        Ok(joint)
    }
    pub(crate) fn move_drag(
        &mut self,
        drag: &mut NativeDrag,
        displacement: [f64; 2],
        paused: bool,
    ) -> Result<(), String> {
        if !displacement.iter().all(|x| x.is_finite() && x.abs() <= 2.0) {
            return Err("invalid drag displacement".into());
        }
        let offset: [f64; 3] = std::array::from_fn(|i| {
            (drag.scale * (drag.right[i] * displacement[0] + drag.up[i] * displacement[1]))
                .clamp(-100.0, 100.0)
        });
        drag.perturb.refselpos = std::array::from_fn(|i| drag.origin[i] + offset[i]);
        if paused {
            let body = drag.perturb.select as usize;
            let joint = self.free_joint(body)?;
            let model = self.workspace.data.model();
            let q = model.jnt_qposadr()[joint] as usize;
            let v = model.jnt_dofadr()[joint] as usize;
            let current = self.workspace.data.xpos()[body];
            let rotation = self.workspace.data.xmat()[body];
            let anchor: [f64; 3] = std::array::from_fn(|i| {
                current[i]
                    + (0..3)
                        .map(|j| rotation[3 * i + j] * drag.perturb.localpos[j])
                        .sum::<f64>()
            });
            for (i, value) in anchor.iter().enumerate() {
                self.workspace.data.qpos_mut()[q + i] += drag.perturb.refselpos[i] - value;
            }
            self.workspace.data.qvel_mut()[v..v + 6].fill(0.0);
            if let Err(error) = self.workspace.forward() {
                self.phase = super::ScenePhase::Failed;
                return Err(error.to_string());
            }
            if let Err(error) = self
                .observation_workspace
                .copy_observation_from(&self.workspace)
            {
                self.phase = super::ScenePhase::Failed;
                return Err(error.to_string());
            }
        }
        Ok(())
    }
    // Native force API overwrites one body's six slots. Save baseline, calculate
    // the owned contribution, compose it for integration, restore even on error.
    pub(crate) fn integrate_drag(
        &mut self,
        controls: &[f64],
        drag: &NativeDrag,
    ) -> Result<super::SceneStep, crate::mujoco::SceneError> {
        let body = drag.perturb.select as usize;
        let baseline = self.workspace.data.xfrc_applied()[body];
        self.workspace.data.xfrc_applied_mut()[body] = [0.0; 6];
        // SAFETY: body/model validated at begin, thread-affine data owned here.
        unsafe {
            mujoco_c::mjv_applyPerturbForce(
                self.workspace.data.model().ffi(),
                self.workspace.data.ffi_mut(),
                &drag.perturb,
            );
        }
        for (slot, value) in self.workspace.data.xfrc_applied_mut()[body]
            .iter_mut()
            .zip(baseline)
        {
            *slot += value;
        }
        if let Some((index, value)) = self.workspace.data.xfrc_applied()[body]
            .iter()
            .copied()
            .enumerate()
            .find(|(_, x)| !x.is_finite())
        {
            self.workspace.data.xfrc_applied_mut()[body] = baseline;
            return Err(crate::mujoco::WorkspaceError::NonFinite {
                name: "drag force",
                index,
                value,
            }
            .into());
        }
        let result = self.integrate_plain(controls);
        self.workspace.data.xfrc_applied_mut()[body] = baseline;
        result
    }
}

impl Scene {
    pub(crate) fn drag_begin(
        &mut self,
        body: usize,
        anchor: [f64; 3],
        camera: ViewCamera,
        paused: bool,
    ) -> Result<(), String> {
        self.drag = None;
        self.drag = Some(self.begin_drag(body, anchor, camera, paused)?);
        Ok(())
    }
    pub(crate) fn drag_update(&mut self, delta: [f64; 2], paused: bool) -> Result<(), String> {
        let mut drag = self.drag.take().ok_or("no active native drag")?;
        let result = self.move_drag(&mut drag, delta, paused);
        if result.is_ok() {
            self.drag = Some(drag);
        }
        result
    }
    pub(crate) fn drag_renew(&mut self, received: std::time::Instant) {
        if let Some(drag) = &mut self.drag {
            drag.deadline = received + super::DRAG_LIVENESS;
        }
    }
    pub(crate) fn drag_cancel(&mut self) {
        self.drag = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mujoco::Model;
    const XML: &str = r#"<mujoco><option gravity="0 0 0" timestep="0.01"/><worldbody>
      <body name="fixed" pos="3 0 0"><geom size=".2"/></body>
      <body name="ball"><freejoint/><geom size=".2" mass="1"/>
        <body name="child" pos="0 0 .5"><joint type="hinge"/><geom size=".1" mass=".1"/></body>
      </body><body name="other" pos="4 0 0"><freejoint/><geom size=".2"/></body>
      </worldbody></mujoco>"#;
    fn scene() -> Scene {
        Scene::new(Model::from_xml(XML).unwrap()).unwrap()
    }
    fn camera() -> ViewCamera {
        ViewCamera {
            look_at: [0.0; 3],
            distance: 3.0,
            azimuth: 90.0,
            elevation: -20.0,
        }
    }
    #[test]
    fn running_force_integrates_without_teleport_and_preserves_external_force() {
        let mut scene = scene();
        scene.workspace.data.xfrc_applied_mut()[2] = [0.0, 0.1, 0.0, 0.0, 0.0, 0.0];
        scene.workspace.data.xfrc_applied_mut()[4] = [0.3; 6];
        let external = scene.workspace.data.xfrc_applied().to_vec();
        let before = scene.snapshot().unwrap();
        scene
            .drag_begin(2, [0.0, 0.0, 0.2], camera(), false)
            .unwrap();
        scene.drag_update([0.1, 0.0], false).unwrap();
        assert_eq!(before, scene.snapshot().unwrap());
        let after = scene.step().unwrap().state;
        assert_eq!(after.boundary(), 1);
        assert!(after.qvel()[0].abs() > 0.0);
        assert!(
            after.qpos()[0].abs() < 0.1,
            "one physical quantum, not target teleport"
        );
        assert_eq!(external, scene.workspace.data.xfrc_applied());
        scene.drag_cancel();
        scene.step().unwrap();
        assert_eq!(external, scene.workspace.data.xfrc_applied());
        // Articulated children support force, never paused ancestor teleport.
        scene.drag_begin(3, [0.0; 3], camera(), false).unwrap();
        scene.drag_update([0.05, 0.0], false).unwrap();
        scene.step().unwrap();
        assert!(scene.drag_begin(3, [0.0; 3], camera(), true).is_err());
        assert!(scene.drag_begin(1, [0.0; 3], camera(), false).is_err());
    }
    #[test]
    fn paused_translation_is_authoritative_local_velocity_reset_and_exact_reset() {
        let mut scene = scene();
        let original = scene.snapshot().unwrap();
        scene.workspace.data.qvel_mut().fill(0.5);
        scene.workspace.forward().unwrap();
        let other = scene.workspace.data.qvel()[6..].to_vec();
        scene.drag_begin(2, [0.0; 3], camera(), true).unwrap();
        scene.drag_update([0.1, 0.0], true).unwrap();
        let moved = scene.snapshot().unwrap();
        assert_eq!(moved.boundary(), 0);
        assert_eq!(moved.time_seconds(), 0.0);
        assert_ne!(moved.qpos(), original.qpos());
        assert_eq!(&moved.qvel()[..6], &[0.0; 6]);
        assert_eq!(&moved.qvel()[6..], other);
        assert_eq!(&moved.qpos()[3..7], &original.qpos()[3..7]);
        assert_eq!(moved.body_positions()[2][0], moved.qpos()[0]);
        scene.drag_cancel();
        assert_eq!(scene.snapshot().unwrap(), moved);
        assert_eq!(scene.reset().unwrap(), original);
        assert!(scene.drag.is_none());
    }
    #[test]
    fn moving_body_anchor_is_local_and_invalid_update_cancels() {
        let mut scene = scene();
        let anchor = scene.workspace.local_anchor(2, [0.1, 0.0, 0.0]).unwrap();
        scene.workspace.data.qpos_mut()[0] = 2.0;
        scene.workspace.forward().unwrap();
        scene.drag_begin(2, anchor, camera(), false).unwrap();
        assert_eq!(scene.drag.as_ref().unwrap().origin, [2.1, 0.0, 0.0]);
        assert!(scene.drag_update([f64::NAN, 0.0], false).is_err());
        assert!(scene.drag.is_none());
    }
}

#[cfg(test)]
mod deadline_tests {
    use super::*;
    use crate::mujoco::Model;
    #[test]
    fn expired_force_and_paused_pose_gesture_do_not_apply_force_at_step() {
        let model = Model::from_xml(r#"<mujoco><option gravity="0 0 0"/><worldbody><body><freejoint/><geom size=".2"/></body></worldbody></mujoco>"#).unwrap();
        let camera = ViewCamera {
            look_at: [0.0; 3],
            distance: 3.0,
            azimuth: 90.0,
            elevation: 0.0,
        };
        let mut expired = Scene::new(model.clone()).unwrap();
        let mut baseline = Scene::new(model.clone()).unwrap();
        expired.drag_begin(1, [0.0; 3], camera, false).unwrap();
        expired.drag_update([0.2, 0.0], false).unwrap();
        expired.drag.as_mut().unwrap().deadline =
            std::time::Instant::now() - std::time::Duration::from_secs(1);
        assert_eq!(
            expired.step().unwrap().state,
            baseline.step().unwrap().state
        );
        assert!(expired.drag.is_none());
        let mut paused = Scene::new(model).unwrap();
        paused.drag_begin(1, [0.0; 3], camera, true).unwrap();
        paused.drag_update([0.2, 0.0], true).unwrap();
        let edited = paused.snapshot().unwrap();
        assert_eq!(paused.step().unwrap().state.qpos(), edited.qpos());
    }
}

#[cfg(test)]
mod constraint_tests {
    use super::*;
    #[test]
    fn active_weld_refuses_paused_free_body_translation_but_allows_running_force() {
        let model = crate::mujoco::Model::from_xml(r#"<mujoco><worldbody><body name="free"><freejoint/><geom size=".2"/></body></worldbody><equality><weld body1="free"/></equality></mujoco>"#).unwrap();
        let mut scene = Scene::new(model).unwrap();
        let camera = ViewCamera {
            look_at: [0.0; 3],
            distance: 3.0,
            azimuth: 90.0,
            elevation: 0.0,
        };
        assert!(
            scene
                .drag_begin(1, [0.0; 3], camera, true)
                .unwrap_err()
                .contains("constraint")
        );
        assert!(scene.drag_begin(1, [0.0; 3], camera, false).is_ok());
    }
}
