//! Definitions related to visualization.
use super::mj_data::MjData;
use super::mj_model::traits::ModelType;
use super::mj_model::{MjModel, MjtGeom, MjtObj};
use super::mj_primitive::{MjtByte, MjtNum, MjtSize};
use super::mj_rendering::{MjrContext, MjrRectangle};
use crate::getter_setter;
use crate::native_binding::error::MjSceneError;
use crate::native_binding::mujoco_c::*;
use crate::native_binding::util::checked_c_len;
use crate::{array_slice_dyn, c_str_as_str_method};
use std::default::Default;
use std::mem::MaybeUninit;
use std::ptr;
/// Result of a mouse-based selection query via [`MjvScene::find_selection`].
#[derive(Debug, Clone, PartialEq)]
pub struct SceneSelection {
    /// Selected body id, or `None` if nothing was selected.
    pub body_id: Option<usize>,
    /// Selected geom id, or `None` if nothing was selected.
    pub geom_id: Option<usize>,
    /// Selected flex id, or `None` if nothing was selected.
    pub flex_id: Option<usize>,
    /// Selected skin id, or `None` if nothing was selected.
    pub skin_id: Option<usize>,
    /// 3D world coordinates of the selection point; `[0.0; 3]` when nothing was selected.
    pub point: [MjtNum; 3],
}
impl Default for SceneSelection {
    fn default() -> Self {
        Self {
            body_id: None,
            geom_id: None,
            flex_id: None,
            skin_id: None,
            point: [0.0; 3],
        }
    }
}
/// These are the available categories of geoms in the abstract visualizer. The bitmask selects the
/// categories that [`MjvScene::update_with_catmask`] adds to the scene.
pub type MjtCatBit = mjtCatBit;
/// These are the mouse actions that the abstract visualizer recognizes. It is up to the user to intercept mouse events
/// and translate them into these actions, as illustrated in MuJoCo's `simulate` application.
pub type MjtMouse = mjtMouse;
/// These bitmasks enable the translational and rotational components of the mouse perturbation. For the regular mouse,
/// only one can be enabled at a time. For the 3D mouse (SpaceNavigator) both can be enabled simultaneously. They are used
/// in `mjvPerturb.active`.
pub type MjtPertBit = mjtPertBit;
/// These are the possible camera types, used in `mjvCamera.type`.
pub type MjtCamera = mjtCamera;
const _: () = {
    assert!(MjtCamera::mjCAMERA_FREE as i32 == 0);
    assert!(MjtCamera::mjCAMERA_TRACKING as i32 == 1);
    assert!(MjtCamera::mjCAMERA_FIXED as i32 == 2);
    assert!(MjtCamera::mjCAMERA_USER as i32 == 3);
};
impl TryFrom<i32> for MjtCamera {
    type Error = MjSceneError;
    fn try_from(value: i32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::mjCAMERA_FREE),
            1 => Ok(Self::mjCAMERA_TRACKING),
            2 => Ok(Self::mjCAMERA_FIXED),
            3 => Ok(Self::mjCAMERA_USER),
            _ => Err(MjSceneError::InvalidCameraType(value)),
        }
    }
}
/// These are the abstract visualization elements that can have text labels. Used in `mjvOption.label`.
pub type MjtLabel = mjtLabel;
/// These are the MuJoCo objects whose spatial frames can be rendered. Used in `mjvOption.frame`.
pub type MjtFrame = mjtFrame;
/// These are indices in the array `mjvOption.flags`, whose elements enable/disable the visualization of the
/// corresponding model or decoration element.
pub type MjtVisFlag = mjtVisFlag;
/// These are indices in the array `mjvScene.flags`, whose elements enable/disable OpenGL rendering effects.
pub type MjtRndFlag = mjtRndFlag;
/// These are the possible stereo rendering types. They are used in `mjvScene.stereo`.
pub type MjtStereo = mjtStereo;
/// Mouse perturbation state (selected body/flex/skin, interaction mode, reference position/orientation, local position).
pub type MjvPerturb = mjvPerturb;
impl Default for MjvPerturb {
    fn default() -> Self {
        unsafe {
            let mut pert = MaybeUninit::uninit();
            mjv_defaultPerturb(pert.as_mut_ptr());
            pert.assume_init()
        }
    }
}
impl MjvPerturb {
    /// Initializes the perturbation state for mouse interaction of the given `type_`.
    /// Must be called before [`MjvPerturb::move_`].
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process when `self.select` names a body
    /// (`0 < select < nbody`) and the cameras of `scene` were never filled by
    /// [`MjvScene::update`], which leaves `frustum_near` at zero.
    ///
    /// # Panics
    /// Panics if `self.flexselect` is greater than or equal to the number of flexes in the model
    /// of `data`.
    pub fn start<M: ModelType>(
        &mut self,
        type_: MjtPertBit,
        data: &mut MjData<M>,
        scene: &MjvScene,
    ) {
        let nflex = data.model().nflex();
        assert!(
            (self.flexselect as MjtSize) < nflex,
            "selected flex id {} is out of range for a model with {nflex} flexes",
            self.flexselect
        );
        let model_ffi = data.model().ffi();
        unsafe {
            mjv_initPerturb(model_ffi, data.ffi_mut(), scene.ffi(), self);
        }
        self.active = type_ as i32;
    }
    /// Move an object with mouse. Wraps [`mjv_movePerturb`].
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process when `action` is not one of
    /// [`MjtMouse::mjMOUSE_MOVE_V`], [`MjtMouse::mjMOUSE_MOVE_H`],
    /// [`MjtMouse::mjMOUSE_MOVE_V_REL`], [`MjtMouse::mjMOUSE_MOVE_H_REL`],
    /// [`MjtMouse::mjMOUSE_ROTATE_V`], [`MjtMouse::mjMOUSE_ROTATE_H`] or
    /// [`MjtMouse::mjMOUSE_ZOOM`], and when the cameras of `scene` were never filled by
    /// [`MjvScene::update`], which leaves `frustum_near` at zero.
    ///
    /// # Panics
    /// Panics if `self.select` is out of range for the model in `data` (i.e. negative or
    /// `>= nbody`).
    pub fn move_<M: ModelType>(
        &mut self,
        data: &MjData<M>,
        action: MjtMouse,
        dx: MjtNum,
        dy: MjtNum,
        scene: &MjvScene,
    ) {
        let nbody = data.model().nbody();
        assert!(
            self.select >= 0 && (self.select as MjtSize) < nbody,
            "selected perturbation body id {} is out of range for a model with {} bodies",
            self.select,
            nbody
        );
        unsafe {
            mjv_movePerturb(
                data.model().ffi(),
                data.ffi(),
                action as i32,
                dx,
                dy,
                scene.ffi(),
                self,
            );
        }
    }
    /// Apply perturbation pose and force.
    ///
    /// # Note
    /// This method **zeroes `xfrc_applied`** for all bodies before applying the perturbation
    /// force. Any external forces set on `data` before calling this method will be cleared.
    /// If you need to preserve external forces, apply them *after* calling this method.
    pub fn apply<M: ModelType>(&mut self, data: &mut MjData<M>) {
        data.xfrc_applied_mut().fill([0.0; 6]);
        let model_ffi = data.model().ffi();
        unsafe {
            mjv_applyPerturbPose(model_ffi, data.ffi_mut(), self, 0);
        }
        let model_ffi = data.model().ffi();
        unsafe {
            mjv_applyPerturbForce(model_ffi, data.ffi_mut(), self);
        }
    }
    /// Updates the body-local position of the selection point.
    ///
    /// # Panics
    /// Panics if `self.select` is out of range for the `xpos`/`xmat` arrays (i.e., negative or
    /// `>= nbody`). In debug builds, a dedicated assertion fires first for the negative case.
    pub fn update_local_pos<M: ModelType>(
        &mut self,
        selection_xyz: &[MjtNum; 3],
        data: &MjData<M>,
    ) {
        debug_assert!(
            self.select >= 0,
            "invalid selecting when calling update_local_pos"
        );
        let select = self.select as usize;
        let body_xpos = &data.xpos()[select];
        let body_xmat = &data.xmat()[select];
        let tmp = [
            selection_xyz[0] - body_xpos[0],
            selection_xyz[1] - body_xpos[1],
            selection_xyz[2] - body_xpos[2],
        ];
        self.localpos = [
            body_xmat[0] * tmp[0] + body_xmat[3] * tmp[1] + body_xmat[6] * tmp[2],
            body_xmat[1] * tmp[0] + body_xmat[4] * tmp[1] + body_xmat[7] * tmp[2],
            body_xmat[2] * tmp[0] + body_xmat[5] * tmp[1] + body_xmat[8] * tmp[2],
        ];
    }
}
/// Abstract camera parameters (type, fixed/tracking ids, lookat, distance, azimuth and elevation
/// in degrees, orthographic mode).
pub type MjvCamera = mjvCamera;
impl MjvCamera {
    /// Creates a new free camera.
    /// By default, the camera will look at the center of the model.
    pub fn new_free(model: &MjModel) -> Self {
        let mut camera: mjvCamera_ = Self::default();
        unsafe {
            mjv_defaultFreeCamera(model.ffi(), &mut camera);
        }
        camera
    }
    /// Creates a new fixed camera.
    ///
    /// # Panics
    /// In debug builds, panics if `camera_id` exceeds `i32::MAX`.
    pub fn new_fixed(camera_id: usize) -> Self {
        debug_assert!(camera_id <= i32::MAX as usize, "camera_id exceeds i32::MAX");
        mjvCamera_ {
            type_: MjtCamera::mjCAMERA_FIXED as i32,
            fixedcamid: camera_id as i32,
            ..Self::default()
        }
    }
    /// Creates a new tracking camera to track a body with the given `tracking_id`.
    ///
    /// # Panics
    /// In debug builds, panics if `tracking_id` exceeds `i32::MAX`.
    pub fn new_tracking(tracking_id: usize) -> Self {
        debug_assert!(
            tracking_id <= i32::MAX as usize,
            "tracking_id exceeds i32::MAX"
        );
        mjvCamera_ {
            type_: MjtCamera::mjCAMERA_TRACKING as i32,
            trackbodyid: tracking_id as i32,
            ..Self::default()
        }
    }
    /// Creates a new camera of user type.
    pub fn new_user() -> Self {
        mjvCamera_ {
            type_: MjtCamera::mjCAMERA_USER as i32,
            ..Self::default()
        }
    }
    /// Sets the camera into tracking mode.
    pub fn track(&mut self, tracking_id: usize) {
        self.type_ = MjtCamera::mjCAMERA_TRACKING as i32;
        self.fixedcamid = -1;
        self.trackbodyid = tracking_id as i32;
    }
    /// Sets the camera free from tracking.
    pub fn free(&mut self) {
        self.trackbodyid = -1;
        self.type_ = MjtCamera::mjCAMERA_FREE as i32;
    }
    /// Sets the camera to a fixed `camera_id`.
    pub fn fix(&mut self, camera_id: usize) {
        self.type_ = MjtCamera::mjCAMERA_FIXED as i32;
        self.fixedcamid = camera_id as i32;
        self.trackbodyid = -1;
    }
    /// Move camera with mouse.
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process when `action` is not one of
    /// [`MjtMouse::mjMOUSE_ROTATE_V`], [`MjtMouse::mjMOUSE_ROTATE_H`],
    /// [`MjtMouse::mjMOUSE_MOVE_V`], [`MjtMouse::mjMOUSE_MOVE_H`], [`MjtMouse::mjMOUSE_TURN_V`],
    /// [`MjtMouse::mjMOUSE_TURN_H`], [`MjtMouse::mjMOUSE_ZOOM`], [`MjtMouse::mjMOUSE_MOVE_V_REL`]
    /// or [`MjtMouse::mjMOUSE_MOVE_H_REL`]. A camera of type [`MjtCamera::mjCAMERA_FIXED`] returns
    /// early, before that point.
    pub fn move_(&mut self, action: MjtMouse, model: &MjModel, dx: MjtNum, dy: MjtNum) {
        unsafe {
            mjv_moveCamera(model.ffi(), action as i32, dx, dy, self);
        };
    }
    /// Get the camera coordinate frame (pos, forward, up, right).
    ///
    /// # Note
    /// MuJoCo raises a fatal error, which ends the process, when `self.type_` is not
    /// [`MjtCamera::mjCAMERA_FREE`], [`MjtCamera::mjCAMERA_TRACKING`] or
    /// [`MjtCamera::mjCAMERA_FIXED`].
    ///
    /// # Panics
    /// Panics if this is a fixed camera (`MjtCamera::mjCAMERA_FIXED`) whose `fixedcamid` is out of
    /// range, or a tracking camera (`MjtCamera::mjCAMERA_TRACKING`) whose `trackbodyid` is greater
    /// than or equal to the number of bodies.
    pub fn frame<M: ModelType>(
        &self,
        data: &MjData<M>,
    ) -> ([MjtNum; 3], [MjtNum; 3], [MjtNum; 3], [MjtNum; 3]) {
        if self.type_ == MjtCamera::mjCAMERA_FIXED as i32 {
            let ncam = data.model().ncam();
            assert!(
                self.fixedcamid >= 0 && (self.fixedcamid as i64) < ncam,
                "fixed camera id {} is out of range for a model with {} cameras",
                self.fixedcamid,
                ncam
            );
        } else if self.type_ == MjtCamera::mjCAMERA_TRACKING as i32 && self.trackbodyid >= 0 {
            let nbody = data.model().nbody();
            assert!(
                (self.trackbodyid as i64) < nbody,
                "tracked body id {} is out of range for a model with {} bodies",
                self.trackbodyid,
                nbody
            );
        }
        let mut headpos = [0.0; 3];
        let mut forward = [0.0; 3];
        let mut up = [0.0; 3];
        let mut right = [0.0; 3];
        unsafe {
            mjv_cameraFrame(
                &mut headpos,
                &mut forward,
                &mut up,
                &mut right,
                data.ffi(),
                self,
            );
        }
        (headpos, forward, up, right)
    }
    /// Compute the `frustum` (zver, zhor, zclip) suitable for rendering.
    ///
    /// # Note
    /// MuJoCo raises a fatal error, which ends the process, when `self.type_` is not
    /// [`MjtCamera::mjCAMERA_FREE`], [`MjtCamera::mjCAMERA_TRACKING`] or
    /// [`MjtCamera::mjCAMERA_FIXED`], or when `self.fixedcamid` is out of range for `model`.
    pub fn frustum(&self, model: &MjModel) -> ([f32; 2], [f32; 2], [f32; 2]) {
        let mut zver = [0.0; 2];
        let mut zhor = [0.0; 2];
        let mut zclip = [0.0; 2];
        unsafe {
            mjv_cameraFrustum(&mut zver, &mut zhor, &mut zclip, model.ffi(), self);
        }
        (zver, zhor, zclip)
    }
}
impl Default for MjvCamera {
    fn default() -> Self {
        unsafe {
            let mut c = MaybeUninit::uninit();
            mjv_defaultCamera(c.as_mut_ptr());
            c.assume_init()
        }
    }
}
/// OpenGL camera parameters (position, forward/up vectors, frustum planes).
pub type MjvGLCamera = mjvGLCamera;
/// OpenGL camera parameters (position, forward/up vectors, frustum planes). Alias of
/// [`MjvGLCamera`].
pub type MjrCamera = mjrCamera;
impl MjvGLCamera {
    /// Average the current MjvGLCamera with the `other` MjvGLCamera.
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process when `self.orthographic` and
    /// `other.orthographic` differ.
    pub fn average_camera(&self, other: &Self) -> Self {
        unsafe { mjv_averageCamera(self, other) }
    }
}
/// Visual geometry element (type, size, material, RGBA, label, etc.) used in a scene.
pub type MjvGeom = mjvGeom;
impl MjvGeom {
    /// Sets the geom so that it acts as a connector (line, arrow, etc.) between
    /// two 3D points. `width` is the connector radius in length units, or its width in pixels for
    /// `mjGEOM_LINE`.
    ///
    /// Wraps [`mjv_connector`]. The connector type
    /// is taken from the geom's current [`type_`](MjvGeom::type_) field, so
    /// set it to the desired connector type (e.g. `mjGEOM_LINE`, `mjGEOM_ARROW`)
    /// **before** calling this method, or initialize the geom
    /// with that type via [`MjvScene::create_geom`].
    ///
    /// # Note
    /// MuJoCo raises a fatal error, which ends the process, for any type other than
    /// `mjGEOM_CAPSULE`, `mjGEOM_CYLINDER`, `mjGEOM_ARROW`, `mjGEOM_ARROW1`, `mjGEOM_ARROW2` or
    /// `mjGEOM_LINE`.
    pub fn connect(&mut self, width: MjtNum, from: [MjtNum; 3], to: [MjtNum; 3]) {
        unsafe {
            mjv_connector(self, self.type_, width, &from, &to);
        }
    }
    /// Compatibility method to convert the `label` attribute into a `String`.
    pub fn label(&self) -> String {
        let len = self
            .label
            .iter()
            .position(|&c| c == 0)
            .unwrap_or(self.label.len());
        let bytes: &[u8] = bytemuck::cast_slice(&self.label[..len]);
        String::from_utf8_lossy(bytes).to_string()
    }
    /// Writes `s` into the fixed-size label buffer, NUL-terminating it.
    /// # Errors
    /// Returns [`MjSceneError::NonAsciiLabel`] when `s` contains non-ASCII characters.
    /// Returns [`MjSceneError::LabelTooLong`] when `s` exceeds the buffer capacity
    /// (`self.label.len() - 1` bytes).
    pub fn set_label(&mut self, s: &str) -> Result<(), MjSceneError> {
        if !s.is_ascii() {
            return Err(MjSceneError::NonAsciiLabel);
        }
        let capacity = self.label.len() - 1;
        if s.len() > capacity {
            return Err(MjSceneError::LabelTooLong {
                len: s.len(),
                capacity,
            });
        }
        let target: &mut [u8] = bytemuck::cast_slice_mut(&mut self.label[..s.len()]);
        target.copy_from_slice(s.as_bytes());
        self.label[s.len()] = 0;
        Ok(())
    }
}
/// Visual light source parameters (position, direction, ambient/diffuse/specular RGB, etc.).
pub type MjvLight = mjvLight;
/// Visualization rendering options (flags, label types, frame display, etc.).
pub type MjvOption = mjvOption;
impl Default for MjvOption {
    fn default() -> Self {
        let mut opt = MaybeUninit::uninit();
        unsafe {
            mjv_defaultOption(opt.as_mut_ptr());
            opt.assume_init()
        }
    }
}
/// Abstraction for plotting figures.
pub type MjvFigure = mjvFigure;
impl Default for MjvFigure {
    fn default() -> Self {
        *Self::new_boxed()
    }
}
impl MjvFigure {
    /// Instantiates a new figure with default values, allocated on the heap.
    ///
    /// `MjvFigure` is ~800 KB; this constructor avoids placing it on the stack.
    pub fn new_boxed() -> Box<Self> {
        let mut opt = Box::new(MaybeUninit::uninit());
        unsafe {
            mjv_defaultFigure(opt.as_mut_ptr());
            opt.assume_init()
        }
    }
    /// Draws the 2D figure to the `viewport` on screen.
    ///
    /// Wraps [`mjr_figure`].
    ///
    /// # Safety
    /// Every `linepnt` entry must lie within `0..=mjMAXLINEPNT`, the capacity of the matching
    /// `linedata` row. Automated checks from Rust side are too expensive.
    pub unsafe fn draw(&mut self, viewport: MjrRectangle, context: &MjrContext) {
        unsafe { mjr_figure(viewport, self, context.ffi()) };
    }
}
/// Figure options.
impl MjvFigure {
    getter_setter! {
        with, get, set, [flg_legend : bool; "whether to show legend."; flg_extend : bool;
        "whether to automatically extend axis ranges to fit data."; flg_barplot : bool;
        "whether to isolate line segments."; flg_selection : bool;
        "whether to show vertical selection line."; flg_symmetric : bool;
        "whether to make y-axis symmetric.";]
    }
    getter_setter! {
        with, [gridsize : [i32; 2]; "number of grid points in (x, y)."; gridrgb : [f32;
        3]; "grid line RGB color."; figurergba : [f32; 4]; "figure RGBA color."; panergba
        : [f32; 4]; "pane RGBA color."; legendrgba : [f32; 4]; "legend RGBA color.";
        textrgb : [f32; 3]; "text RGB color."; linergb : [[f32; 3]; mjMAXLINE as usize];
        "line colors."; range : [[f32; 2]; 2];
        "axis ranges (min >= max means automatic).";]
    }
    c_str_as_str_method! {
        with, get, set { xlabel; "the x-axis label."; title; "the title."; xformat;
        "the x-axis C's printf format (e.g., `%.1f`)."; yformat;
        "the y-axis C's printf format (e.g., `%.1f`)."; linename[plot_index : usize];
        "the line name of plot with `plot_index`."; }
    }
}
/// Plot data manipulation
impl MjvFigure {
    /// Checks if the buffer is full for plot with `plot_index`.
    ///
    /// # Panics
    /// Panics if `plot_index >= mjMAXLINE`.
    ///
    /// Use [`MjvFigure::try_full`] for a fallible alternative.
    pub fn full(&self, plot_index: usize) -> bool {
        self.try_full(plot_index).unwrap()
    }
    /// Checks if the buffer is full for plot with `plot_index`.
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    ///
    /// Use [`MjvFigure::full`] for a panicking alternative.
    pub fn try_full(&self, plot_index: usize) -> Result<bool, MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex {
                plot_index,
                max_plots: mjMAXLINE as usize,
            });
        }
        Ok(self.linepnt[plot_index] >= (self.linedata[plot_index].len() / 2) as i32)
    }
    /// Checks if the buffer is empty for plot with `plot_index`.
    ///
    /// # Panics
    /// Panics if `plot_index >= mjMAXLINE`.
    ///
    /// Use [`MjvFigure::try_empty`] for a fallible alternative.
    pub fn empty(&self, plot_index: usize) -> bool {
        self.try_empty(plot_index).unwrap()
    }
    /// Checks if the buffer is empty for plot with `plot_index`.
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    ///
    /// Use [`MjvFigure::empty`] for a panicking alternative.
    pub fn try_empty(&self, plot_index: usize) -> Result<bool, MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex {
                plot_index,
                max_plots: mjMAXLINE as usize,
            });
        }
        Ok(self.linepnt[plot_index] == 0)
    }
    /// Pushes a new data point to buffer for the specific plot with `plot_index`.
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    /// Returns [`MjSceneError::FigureBufferFull`] if the buffer for
    /// `plot_index` is already at capacity.
    pub fn push(&mut self, plot_index: usize, x: f32, y: f32) -> Result<(), MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex {
                plot_index,
                max_plots: mjMAXLINE as usize,
            });
        }
        let plot = &mut self.linedata[plot_index];
        let capacity = plot.len() / 2;
        let point_index = self.linepnt[plot_index] as usize;
        if point_index >= capacity {
            return Err(MjSceneError::FigureBufferFull {
                plot_index,
                capacity,
            });
        }
        plot[2 * point_index] = x;
        plot[2 * point_index + 1] = y;
        self.linepnt[plot_index] += 1;
        Ok(())
    }
    /// Overrides existing data with a new data point at a specific `point_index` for specific plot with `plot_index`.
    ///
    /// # Panics
    /// Panics if `point_index` is at or above the plot capacity of
    /// `linedata[plot_index].len() / 2` points, which `linepnt[plot_index]` allows when it is
    /// itself above the capacity.
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    /// Returns [`MjSceneError::FigureIndexOutOfBounds`] if `point_index` is
    /// not within the current data range for the given plot.
    pub fn set_at(
        &mut self,
        plot_index: usize,
        point_index: usize,
        x: f32,
        y: f32,
    ) -> Result<(), MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex {
                plot_index,
                max_plots: mjMAXLINE as usize,
            });
        }
        let current_len = self.linepnt[plot_index].max(0) as usize;
        if point_index >= current_len {
            return Err(MjSceneError::FigureIndexOutOfBounds {
                plot_index,
                point_index,
                current_len,
            });
        }
        let plot = &mut self.linedata[plot_index];
        plot[2 * point_index] = x;
        plot[2 * point_index + 1] = y;
        Ok(())
    }
    /// Clears the plot with `maybe_plot_index`.
    /// If `maybe_plot_index` is [`None`], all plots will be cleared.
    ///
    /// # Panics
    /// Panics if `maybe_plot_index` is `Some(i)` and `i >= mjMAXLINE`.
    pub fn clear(&mut self, maybe_plot_index: Option<usize>) {
        if let Some(plot_index) = maybe_plot_index {
            self.linepnt[plot_index] = 0;
        } else {
            self.linepnt.fill(0);
        }
    }
    /// Pops the first element from the plot data of plot with `plot_index`.
    ///
    /// # Returns
    /// Returns `Some((x, y))` when the plot contains any elements, otherwise `None` is returned.
    ///
    /// # Panics
    /// Panics if `plot_index >= mjMAXLINE`, or if `linepnt[plot_index]` is above the plot
    /// capacity of `linedata[plot_index].len() / 2` points.
    ///
    /// Use [`MjvFigure::try_pop_front`] for a fallible alternative.
    pub fn pop_front(&mut self, plot_index: usize) -> Option<(f32, f32)> {
        self.try_pop_front(plot_index).unwrap()
    }
    /// Pops the first element from the plot data of plot with `plot_index`.
    ///
    /// # Panics
    /// Panics if `linepnt[plot_index]` is above the plot capacity of
    /// `linedata[plot_index].len() / 2` points.
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    ///
    /// Returns `Ok(Some((x, y)))` when the plot contains elements, `Ok(None)` when empty.
    ///
    /// Use [`MjvFigure::pop_front`] for a panicking alternative.
    pub fn try_pop_front(&mut self, plot_index: usize) -> Result<Option<(f32, f32)>, MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex {
                plot_index,
                max_plots: mjMAXLINE as usize,
            });
        }
        let len = self.linepnt[plot_index];
        if len <= 0 {
            return Ok(None);
        }
        let plot_data = &mut self.linedata[plot_index];
        let first = (plot_data[0], plot_data[1]);
        plot_data.copy_within(2..len as usize * 2, 0);
        self.linepnt[plot_index] -= 1;
        Ok(Some(first))
    }
    /// Pops the last element from the plot data of plot with `plot_index`.
    ///
    /// # Returns
    /// Returns `Some((x, y))` when the plot contains any elements, otherwise `None` is returned.
    ///
    /// # Panics
    /// Panics if `plot_index >= mjMAXLINE`, or if `linepnt[plot_index]` is above the plot
    /// capacity of `linedata[plot_index].len() / 2` points.
    ///
    /// Use [`MjvFigure::try_pop_back`] for a fallible alternative.
    pub fn pop_back(&mut self, plot_index: usize) -> Option<(f32, f32)> {
        self.try_pop_back(plot_index).unwrap()
    }
    /// Pops the last element from the plot data of plot with `plot_index`.
    ///
    /// # Panics
    /// Panics if `linepnt[plot_index]` is above the plot capacity of
    /// `linedata[plot_index].len() / 2` points.
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    ///
    /// Returns `Ok(Some((x, y)))` when the plot contains elements, `Ok(None)` when empty.
    ///
    /// Use [`MjvFigure::pop_back`] for a panicking alternative.
    pub fn try_pop_back(&mut self, plot_index: usize) -> Result<Option<(f32, f32)>, MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex {
                plot_index,
                max_plots: mjMAXLINE as usize,
            });
        }
        let old_len = self.linepnt[plot_index];
        if old_len <= 0 {
            return Ok(None);
        }
        let plot_data = &mut self.linedata[plot_index];
        let new_start = ((old_len - 1) * 2) as usize;
        self.linepnt[plot_index] -= 1;
        Ok(Some((plot_data[new_start], plot_data[new_start + 1])))
    }
    /// Cuts the first `n` elements from the plot data of plot with `plot_index`.
    ///
    /// If `n` exceeds the current length, this is a no-op.
    ///
    /// # Panics
    /// Panics if `linepnt[plot_index]` is above the plot capacity of
    /// `linedata[plot_index].len() / 2` points.
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    pub fn cut_front(&mut self, plot_index: usize, n: usize) -> Result<(), MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex {
                plot_index,
                max_plots: mjMAXLINE as usize,
            });
        }
        let len = self.linepnt[plot_index];
        if len < 0 || (len as usize) < n {
            return Ok(());
        }
        self.linedata[plot_index].copy_within(2 * n..(len as usize * 2), 0);
        self.linepnt[plot_index] -= n as i32;
        Ok(())
    }
    /// Cuts the last `n` elements from the plot data of plot with `plot_index`.
    ///
    /// If `n` exceeds the current length, this is a no-op.
    ///
    /// # Errors
    /// Returns [`MjSceneError::InvalidPlotIndex`] if `plot_index >= mjMAXLINE`.
    pub fn cut_end(&mut self, plot_index: usize, n: usize) -> Result<(), MjSceneError> {
        if plot_index >= mjMAXLINE as usize {
            return Err(MjSceneError::InvalidPlotIndex {
                plot_index,
                max_plots: mjMAXLINE as usize,
            });
        }
        let len = self.linepnt[plot_index];
        if len < 0 || (len as usize) < n {
            return Ok(());
        }
        self.linepnt[plot_index] -= n as i32;
        Ok(())
    }
}
/// Snapshot of the [`MjModel`] quantities that fix the size of every buffer that
/// [`MjvScene::new`] allocates, and the two element counts that bound the `objid` of a flex geom
/// and of a skin geom.
///
/// The compilation signature is no entry here: `mj_saveModel` does not write it, so a model that
/// came back from a buffer carries a zero and would be refused against the scene it built. The
/// entries below stand on their own. The flex tables are kept whole: `mjv_makeScene` sizes the
/// flex face buffer from `flex_dim`, `flex_elemnum`, `flex_shellnum` and `flex_elemlayer`
/// together, so no scalar total replaces them.
#[derive(Debug, Clone, PartialEq, Eq)]
struct MjvSceneLayout {
    nflexedge: MjtSize,
    nflexvert: MjtSize,
    nskin: MjtSize,
    nskinvert: MjtSize,
    flex_dim: Vec<i32>,
    flex_elemnum: Vec<i32>,
    flex_shellnum: Vec<i32>,
    flex_elemlayer: Vec<i32>,
}
impl From<&MjModel> for MjvSceneLayout {
    fn from(model: &MjModel) -> Self {
        let ffi = model.ffi();
        Self {
            nflexedge: ffi.nflexedge,
            nflexvert: ffi.nflexvert,
            nskin: ffi.nskin,
            nskinvert: ffi.nskinvert,
            flex_dim: model.flex_dim().to_vec(),
            flex_elemnum: model.flex_elemnum().to_vec(),
            flex_shellnum: model.flex_shellnum().to_vec(),
            flex_elemlayer: model.flex_elemlayer().to_vec(),
        }
    }
}
/// 3D scene visualization.
/// This struct provides a way to render visual-only geometry.
///
/// The scene does not hold a reference to the model.
/// Trying to use an existing scene with an incompatible model (see
/// [`MjvScene::is_compatible_with_model`]) will result in a panic.
#[derive(Debug)]
pub struct MjvScene {
    ffi: Box<mjvScene>,
    layout: MjvSceneLayout,
    /// Reported by [`MjvScene::signature`] and named in the panic of an incompatible model. It
    /// decides nothing: `MjvSceneLayout` carries every size the scene depends on.
    signature: u64,
}
impl MjvScene {
    /// Creates a new scene for `model`, allocating space for up to `max_geom` geoms.
    ///
    /// # Panics
    /// Panics if `max_geom` exceeds [`i32::MAX`].
    pub fn new<M: ModelType>(model: M, max_geom: usize) -> Self {
        let model_ffi = model.ffi();
        let layout = MjvSceneLayout::from(&*model);
        let scene = unsafe {
            let mut t = Box::new_uninit();
            mjv_defaultScene(t.as_mut_ptr());
            mjv_makeScene(model_ffi, t.as_mut_ptr(), checked_c_len(max_geom));
            t.assume_init()
        };
        let nflex = scene.nflex as usize;
        let nface = if scene.flexfacenum.is_null() {
            0
        } else {
            unsafe { std::slice::from_raw_parts(scene.flexfacenum, nflex) }
                .iter()
                .map(|&n| n as usize)
                .sum()
        };
        let nflexvert = layout.nflexvert as usize;
        let nskinvert = layout.nskinvert as usize;
        let maxgeom = scene.maxgeom as usize;
        let buffers: [(*mut u8, usize); 9] = [
            (scene.geoms.cast(), maxgeom * size_of::<mjvGeom>()),
            (scene.geomorder.cast(), maxgeom * size_of::<i32>()),
            (scene.flexfaceused.cast(), nflex * size_of::<i32>()),
            (scene.flexvert.cast(), 3 * nflexvert * size_of::<f32>()),
            (scene.flexface.cast(), 9 * nface * size_of::<f32>()),
            (scene.flexnormal.cast(), 9 * nface * size_of::<f32>()),
            (scene.flextexcoord.cast(), 6 * nface * size_of::<f32>()),
            (scene.skinvert.cast(), 3 * nskinvert * size_of::<f32>()),
            (scene.skinnormal.cast(), 3 * nskinvert * size_of::<f32>()),
        ];
        for (pointer, bytes) in buffers {
            if !pointer.is_null() && bytes != 0 {
                unsafe {
                    ptr::write_bytes(pointer, 0, bytes);
                }
            }
        }
        Self {
            ffi: scene,
            layout,
            signature: model.signature(),
        }
    }
    /// Returns the model signature this scene was created for.
    pub fn signature(&self) -> u64 {
        self.signature
    }
    /// Reports whether `model` can take the place of the model that created this scene.
    ///
    /// The scene buffers hold one entry per flex face, flex vertex, flex edge and skin vertex of
    /// the model that created them, and `mjv_updateScene` refills them with the counts of the
    /// model it receives. The test covers every count that sizes such a buffer, plus the flex and
    /// skin counts that bound the `objid` a geom carries into those buffers.
    /// [`MjvScene::signature`] takes no part: `mj_saveModel` does not write it, so a model that
    /// came back from a buffer would be refused against the scene it built.
    pub fn is_compatible_with_model(&self, model: &MjModel) -> bool {
        self.layout == MjvSceneLayout::from(model)
    }
    /// Reports whether `other` was created for a model that is compatible with this scene's
    /// model.
    ///
    /// A geom that moves between two scenes keeps its `objid`, which the renderer uses as an
    /// unchecked index into the destination scene's flex and skin arrays.
    pub fn is_compatible_with_scene(&self, other: &MjvScene) -> bool {
        self.layout == other.layout
    }
    /// Panics if `model` is not compatible with the model used to create the scene.
    fn assert_compatible(&self, model: &MjModel) {
        assert!(
            self.is_compatible_with_model(model),
            "the model is not compatible with the scene: scene signature {:#X}, model signature {:#X}",
            self.signature,
            model.signature()
        );
    }
    /// Updates the scene from the current simulation state in `data`.
    ///
    /// The `catmask` parameter controls which geom categories are included
    /// (e.g., [`MjtCatBit::mjCAT_ALL`] for everything, or a bitwise OR of
    /// [`MjtCatBit::mjCAT_STATIC`], [`MjtCatBit::mjCAT_DYNAMIC`], [`MjtCatBit::mjCAT_DECOR`]).
    ///
    /// The call resets `ngeom` to 0, so it drops every geom that [`MjvScene::create_geom`] added;
    /// create such geoms after the update. Unless `cam` is of type
    /// [`MjtCamera::mjCAMERA_USER`], it also overwrites the scene cameras and clears
    /// `enabletransform`, and it moves `cam.lookat` onto the tracked body for a tracking camera.
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process when `cam` is a tracking camera
    /// ([`MjtCamera::mjCAMERA_TRACKING`]) whose `trackbodyid` is negative or not below the number
    /// of bodies.
    ///
    /// # Panics
    /// - Panics if `data` was created from a model that is not compatible with this scene
    ///   (see [`MjvScene::is_compatible_with_model`]).
    /// - Panics if `cam` is a fixed camera ([`MjtCamera::mjCAMERA_FIXED`]) whose `fixedcamid` is
    ///   out of range for the model in `data`.
    /// - Panics if `perturb.select` is out of range (greater than or equal to the number of
    ///   bodies) for the model in `data`.
    pub fn update_with_catmask<M: ModelType>(
        &mut self,
        data: &mut MjData<M>,
        opt: &MjvOption,
        perturb: &MjvPerturb,
        cam: &mut MjvCamera,
        catmask: i32,
    ) {
        self.assert_compatible(data.model());
        if cam.type_ == MjtCamera::mjCAMERA_FIXED as i32 {
            let ncam = data.model().ncam();
            assert!(
                cam.fixedcamid >= 0 && (cam.fixedcamid as i64) < ncam,
                "fixed camera id {} is out of range for a model with {} cameras",
                cam.fixedcamid,
                ncam
            );
        }
        assert!(
            (perturb.select as MjtSize) < data.model().nbody(),
            "selected perturbated body ID is outside the valid range"
        );
        unsafe {
            mjv_updateScene(
                data.model().ffi(),
                data.ffi_mut(),
                opt,
                perturb,
                cam,
                catmask,
                self.ffi.as_mut(),
            );
        }
    }
    /// Updates the scene from the current simulation state in `data`, including all geom categories.
    ///
    /// This is equivalent to calling [`update_with_catmask`](Self::update_with_catmask) with
    /// [`MjtCatBit::mjCAT_ALL`].
    ///
    /// # Note
    /// MuJoCo stops the process under the same conditions as
    /// [`update_with_catmask`](Self::update_with_catmask).
    ///
    /// # Panics
    /// Panics under the same conditions as [`update_with_catmask`](Self::update_with_catmask):
    /// a model-signature mismatch, an out-of-range fixed-camera id, or an out-of-range
    /// `perturb.select`.
    pub fn update<M: ModelType>(
        &mut self,
        data: &mut MjData<M>,
        opt: &MjvOption,
        perturb: &MjvPerturb,
        cam: &mut MjvCamera,
    ) {
        self.update_with_catmask(data, opt, perturb, cam, MjtCatBit::mjCAT_ALL as i32);
    }
    /// Creates a new [`MjvGeom`] in this scene, returning a mutable reference to it.
    /// The geom reference is valid for the duration of the `&mut self` borrow.
    ///
    /// # Safety
    /// The caller must keep the returned geom's `matid`, `texid` and, on a flex or a skin geom,
    /// `objid` below the count of the model that the rendering context was created for (`matid`
    /// and `texid` also accept `-1` for none). The renderer reads all three as unchecked indices
    /// into the context and the scene arrays, so a larger value makes [`MjvScene::render`] read
    /// out of bounds. A flex or a skin geom needs its own `objid`: the geom starts at `-1`, which
    /// indexes neither array.
    ///
    /// # Panics
    /// Panics when `ngeom >= maxgeom` (the scene's geom buffer is full).
    ///
    /// Use [`MjvScene::try_create_geom`] for a fallible alternative.
    pub unsafe fn create_geom(
        &mut self,
        geom_type: MjtGeom,
        size: Option<[MjtNum; 3]>,
        pos: Option<[MjtNum; 3]>,
        mat: Option<[MjtNum; 9]>,
        rgba: Option<[f32; 4]>,
    ) -> &mut MjvGeom {
        unsafe { self.try_create_geom(geom_type, size, pos, mat, rgba) }
            .expect("create_geom failed: scene full")
    }
    /// Fallible version of [`MjvScene::create_geom`].
    ///
    /// # Safety
    /// The caller upholds the same index contract as in [`MjvScene::create_geom`].
    ///
    /// # Errors
    /// Returns [`MjSceneError::SceneFull`] when `ngeom >= maxgeom`.
    pub unsafe fn try_create_geom(
        &mut self,
        geom_type: MjtGeom,
        size: Option<[MjtNum; 3]>,
        pos: Option<[MjtNum; 3]>,
        mat: Option<[MjtNum; 9]>,
        rgba: Option<[f32; 4]>,
    ) -> Result<&mut MjvGeom, MjSceneError> {
        if self.ffi.ngeom >= self.ffi.maxgeom {
            return Err(MjSceneError::SceneFull {
                capacity: self.ffi.maxgeom,
            });
        }
        let size_ptr = size.as_ref().map_or(ptr::null(), |x| x);
        let pos_ptr = pos.as_ref().map_or(ptr::null(), |x| x);
        let mat_ptr = mat.as_ref().map_or(ptr::null(), |x| x);
        let rgba_ptr = rgba.as_ref().map_or(ptr::null(), |x| x);
        unsafe {
            let p_geom = self.ffi.geoms.add(self.ffi.ngeom as usize);
            mjv_initGeom(
                p_geom,
                geom_type as i32,
                size_ptr,
                pos_ptr,
                mat_ptr,
                rgba_ptr,
            );
            (*p_geom).objtype = MjtObj::mjOBJ_UNKNOWN as i32;
            (*p_geom).objid = -1;
            (*p_geom).category = MjtCatBit::mjCAT_DECOR as i32;
            (*p_geom).segid = self.ffi.ngeom;
            (*p_geom).label = [0; 100];
            (*p_geom).camdist = 0.0;
            (*p_geom).transparent = 0;
            self.ffi.ngeom += 1;
            Ok(&mut *p_geom)
        }
    }
    /// Clears the created geoms.
    pub fn clear_geom(&mut self) {
        self.ffi.ngeom = 0;
    }
    /// Removes the last geom from the scene.
    /// Does nothing if the scene contains no geoms.
    pub fn pop_geom(&mut self) {
        if self.ffi.ngeom == 0 {
            return;
        }
        self.ffi.ngeom -= 1;
    }
    /// Renders the scene into the buffer that `context` currently targets. This does not
    /// automatically make the OpenGL context current.
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process when the scene holds geoms but its cameras
    /// were never filled by [`MjvScene::update`], which leaves `frustum_near` at zero. With no
    /// geoms the call returns without drawing.
    ///
    /// # Panics
    /// Panics if `context` holds fewer skins than the scene.
    pub fn render(&mut self, viewport: &MjrRectangle, context: &MjrContext) {
        assert!(
            self.nskin() <= context.nskin(),
            "the context was created for a model with fewer skins"
        );
        unsafe {
            mjr_render(*viewport, self.ffi_mut(), context.ffi());
        }
    }
    /// Returns the selection point based on a mouse click.
    /// Wraps [`mjv_select`].
    ///
    /// `aspect_ratio` is the viewport width divided by its height. `relx` and `rely` are the cursor
    /// position as fractions of the viewport in `[0, 1]`, measured from the left and bottom edges.
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process when the scene cameras were never filled by
    /// [`MjvScene::update`], which leaves `frustum_near` at zero.
    ///
    /// # Panics
    /// Panics if `data` was created from a model that is not compatible with this scene
    /// (see [`MjvScene::is_compatible_with_model`]).
    pub fn find_selection<M: ModelType>(
        &self,
        data: &mut MjData<M>,
        option: &MjvOption,
        aspect_ratio: MjtNum,
        relx: MjtNum,
        rely: MjtNum,
    ) -> SceneSelection {
        self.assert_compatible(data.model());
        let (mut geom_id, mut flex_id, mut skin_id) = (-1, -1, -1);
        let mut selpnt = [0.0; 3];
        let body_id = unsafe {
            mjv_select(
                data.model().ffi(),
                data.ffi(),
                option,
                aspect_ratio,
                relx,
                rely,
                self.ffi(),
                &mut selpnt,
                &mut geom_id,
                &mut flex_id,
                &mut skin_id,
            )
        };
        let to_opt = |v| if v >= 0 { Some(v as usize) } else { None };
        SceneSelection {
            body_id: to_opt(body_id),
            geom_id: to_opt(geom_id),
            flex_id: to_opt(flex_id),
            skin_id: to_opt(skin_id),
            point: selpnt,
        }
    }
    /// Reference to the wrapped FFI struct.
    pub fn ffi(&self) -> &mjvScene {
        &self.ffi
    }
    /// Mutable reference to the wrapped FFI struct.
    ///
    /// # Safety
    /// Modifying the underlying FFI struct directly can break the invariants
    /// upheld by the `mujoco-rs` wrappers and cause undefined behavior.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjvScene {
        &mut self.ffi
    }
}
/// Array slices.
impl MjvScene {
    array_slice_dyn! {
        probe = probe_dynamic_arrays; (mut = unsafe) flexedge : & [[i32; 2] [force];
        "flex edge data"; layout.nflexedge], flexvert : & [[f32; 3] [force];
        "flex vertices"; layout.nflexvert], skinvert : & [[f32; 3] [force];
        "skin vertex data"; layout.nskinvert], skinnormal : & [[f32; 3] [force];
        "skin normal data"; layout.nskinvert], (mut = unsafe) geoms : & [MjvGeom;
        "buffer for geoms"; ffi.ngeom], geomorder : & [i32;
        "buffer for ordering geoms by distance to camera"; ffi.ngeom], (mut = unsafe)
        flexedgeadr : & [i32; "address of flex edges"; ffi.nflex], (mut = unsafe)
        flexedgenum : & [i32; "number of edges in flex"; ffi.nflex], (mut = unsafe)
        flexvertadr : & [i32; "address of flex vertices"; ffi.nflex], (mut = unsafe)
        flexvertnum : & [i32; "number of vertices in flex"; ffi.nflex], (mut = unsafe)
        flexfaceadr : & [i32; "address of flex faces"; ffi.nflex], (mut = unsafe)
        flexfacenum : & [i32; "number of flex faces allocated"; ffi.nflex], (mut =
        unsafe) flexfaceused : & [i32; "number of flex faces currently in use"; ffi
        .nflex], (mut = unsafe) skinfacenum : & [i32; "number of faces in skin"; ffi
        .nskin], (mut = unsafe) skinvertadr : & [i32; "address of skin vertices"; ffi
        .nskin], (mut = unsafe) skinvertnum : & [i32; "number of vertices in skin"; ffi
        .nskin], lights : as_ptr as_mut_ptr & [MjvLight; "buffer for lights"; ffi.nlight]
    }
    array_slice_dyn! {
        summed { flexface : & [f32; "flex faces vertices"; [9; (ffi.flexfacenum); (ffi
        .nflex)]], flexnormal : & [f32; "flex face normals"; [9; (ffi.flexfacenum); (ffi
        .nflex)]], flextexcoord : & [f32; "flex face texture coordinates"; [6; (ffi
        .flexfacenum); (ffi.nflex)]] }
    }
}
/// Public API getters / setters / builders.
impl MjvScene {
    getter_setter! {
        get, [[ffi] maxgeom : i32; "size of allocated geom buffer."; [ffi] ngeom : i32;
        "number of geoms currently in buffer."; [ffi] nflex : i32; "number of flexes.";
        [ffi] nskin : i32; "number of skins."; [ffi] nlight : i32;
        "number of lights currently in buffer."; [ffi] status : i32;
        "status; 0: ok, 1: geoms exhausted.";]
    }
    getter_setter! {
        get, [[ffi] flexvertopt : bool; "copy of mjVIS_FLEXVERT mjvOption flag."; [ffi]
        flexedgeopt : bool; "copy of mjVIS_FLEXEDGE mjvOption flag."; [ffi] flexfaceopt :
        bool; "copy of mjVIS_FLEXFACE mjvOption flag."; [ffi] flexskinopt : bool;
        "copy of mjVIS_FLEXSKIN mjvOption flag.";]
    }
    getter_setter! {
        with, get, set, [[ffi, ffi_mut] stereo : MjtStereo[force];
        "stereoscopic rendering.";]
    }
    getter_setter! {
        with, get, set, [[ffi, ffi_mut] scale : f32; "model scaling."; [ffi, ffi_mut]
        framewidth : i32; "frame pixel width; 0: disable framing.";]
    }
    getter_setter! {
        with, get, set, [[ffi, ffi_mut] enabletransform : bool;
        "enable model transformation.";]
    }
    getter_setter! {
        with, get, [[ffi, ffi_mut] camera : & [MjvGLCamera; 2]; "left and right camera.";
        [ffi, ffi_mut] translate : & [f32; 3]; "model translation."; [ffi, ffi_mut]
        rotate : & [f32; 4]; "model quaternion rotation."; [ffi, ffi_mut] framergb : &
        [f32; 3]; "frame color.";]
    }
    getter_setter! {
        get, [[ffi, ffi_mut] flags : & [MjtByte; MjtRndFlag::mjNRNDFLAG as usize];
        "rendering flags (indexed by mjtRndFlag).";]
    }
}
impl Drop for MjvScene {
    fn drop(&mut self) {
        unsafe {
            mjv_freeScene(self.ffi.as_mut());
        }
    }
}
unsafe impl Send for MjvScene {}
unsafe impl Sync for MjvScene {}
