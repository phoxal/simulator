//! MjModel related.
use super::mj_auxiliary::{MjStatistic, MjVfs, MjVisual};
use super::mj_primitive::*;
use crate::native_binding::error::{MjDataError, MjModelError};
use crate::native_binding::mujoco_c::*;
use crate::native_binding::util::{ERROR_BUF_LEN, assert_mujoco_version, checked_c_len};
use crate::native_binding::wrappers::mj_data::MjData;
use crate::native_binding::wrappers::mj_option::MjOption;
use crate::{array_slice_dyn, getter_setter, info_method, info_with_view, view_creator};
use bytemuck::must_cast_slice;
use log::debug;
use std::ffi::{CStr, CString, c_char, c_int, c_void};
use std::fmt::{Debug, Formatter, Result as FmtResult};
use std::path::Path;
use std::ptr::{self, NonNull};
use std::sync::{Arc, OnceLock};
pub mod traits;
/// Constants which are powers of 2. They are used as bitmasks for the field `disableflags` of `mjOption`.
/// At runtime this field is `m->opt.disableflags`. The number of these constants is given by `mjNDISABLE` which is
/// also the length of the global string array `mjDISABLESTRING` with text descriptions of these flags.
pub type MjtDisableBit = mjtDisableBit;
/// Constants which are powers of 2. They are used as bitmasks for the field `enableflags` of `mjOption`.
/// At runtime this field is `m->opt.enableflags`. The number of these constants is given by `mjNENABLE` which is also
/// the length of the global string array `mjENABLESTRING` with text descriptions of these flags.
pub type MjtEnableBit = mjtEnableBit;
/// Primitive joint types. These values are used in `m->jnt_type`. The numbers in the comments indicate how many
/// positional coordinates each joint type has. Note that ball joints and rotational components of free joints are
/// represented as unit quaternions - which have 4 positional coordinates but 3 degrees of freedom each.
pub type MjtJoint = mjtJoint;
/// Geometric types supported by MuJoCo. The first group are "official" geom types that can be used in the model. The
/// second group are geom types that cannot be used in the model but are used by the visualizer to add decorative
/// elements. These values are used in `m->geom_type` and `m->site_type`.
pub type MjtGeom = mjtGeom;
/// Type of camera projection. Used in `m->cam_projection`.
pub type MjtProjection = mjtProjection;
/// Dynamic modes for cameras and lights, specifying how the camera/light position and orientation are computed. These
/// values are used in `m->cam_mode` and `m->light_mode`.
pub type MjtCamLight = mjtCamLight;
/// The type of a light source describing how its position, orientation and other properties will interact with the
/// objects in the scene. These values are used in `m->light_type`.
pub type MjtLightType = mjtLightType;
/// Texture types, specifying how the texture will be mapped. These values are used in `m->tex_type`.
pub type MjtTexture = mjtTexture;
/// Texture roles, specifying how the renderer should interpret the texture.  Note that the MuJoCo built-in renderer only
/// uses RGB textures.  These values are used to store the texture index in the material's array `m->mat_texid`.
pub type MjtTextureRole = mjtTextureRole;
/// Type of color space encoding for textures.
pub type MjtColorSpace = mjtColorSpace;
/// Mode for actuator length-range computation.
pub type MjtLRMode = mjtLRMode;
/// Cube map face indices used by [`MjsTexture::set_cubefile`](super::mj_editing::MjsTexture::set_cubefile).
///
/// Each variant corresponds to one face of a cube-map texture, matching the order
/// MuJoCo uses internally (right=0, left=1, up=2, down=3, front=4, back=5).
///
/// **Note:** this enum is defined in mujoco-rs only; MuJoCo's C API uses raw integer
/// indices for cube-map faces.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(i32)]
pub enum MjtCubeFace {
    /// Positive-X face (index 0).
    Right = 0,
    /// Negative-X face (index 1).
    Left = 1,
    /// Positive-Y face (index 2).
    Up = 2,
    /// Negative-Y face (index 3).
    Down = 3,
    /// Positive-Z face (index 4).
    Front = 4,
    /// Negative-Z face (index 5).
    Back = 5,
}
/// Numerical integrator types. These values are used in `m->opt.integrator`.
pub type MjtIntegrator = mjtIntegrator;
/// Available friction cone types. These values are used in `m->opt.cone`.
pub type MjtCone = mjtCone;
/// Available Jacobian types. These values are used in `m->opt.jacobian`.
pub type MjtJacobian = mjtJacobian;
/// Available constraint solver algorithms. These values are used in `m->opt.solver`.
pub type MjtSolver = mjtSolver;
/// Equality constraint types. These values are used in `m->eq_type`.
pub type MjtEq = mjtEq;
/// Tendon wrapping object types. These values are used in `m->wrap_type`.
pub type MjtWrap = mjtWrap;
/// Actuator transmission types. These values are used in `m->actuator_trntype`.
pub type MjtTrn = mjtTrn;
/// Actuator dynamics types. These values are used in `m->actuator_dyntype`.
pub type MjtDyn = mjtDyn;
/// Actuator gain types. These values are used in `m->actuator_gaintype`.
pub type MjtGain = mjtGain;
/// Actuator bias types. These values are used in `m->actuator_biastype`.
pub type MjtBias = mjtBias;
/// Orientation input charts of so3 actuators. These values are used in `m->actuator_ctrlspec`.
pub type MjtCtrlChart = mjtCtrlChart;
/// Input signature bits of servo-family (`pid`, `dcmotor`) actuators. These values are OR-ed into
/// `m->actuator_ctrlspec`.
pub type MjtCtrlInput = mjtCtrlInput;
/// MuJoCo object types. These are used, for example, in the support functions `mj_name2id` and
/// `mj_id2name` to convert between object names and integer ids.
pub type MjtObj = mjtObj;
/// Sensor types. These values are used in `m->sensor_type`.
pub type MjtSensor = mjtSensor;
/// These are the compute stages for the skipstage parameters of `mj_forwardSkip` and
/// `mj_inverseSkip`.
pub type MjtStage = mjtStage;
/// These are the possible sensor data types, used in [`MjModel::sensor_datatype`].
pub type MjtDataType = mjtDataType;
/// Types of data fields returned by contact sensors.
pub type MjtConDataField = mjtConDataField;
/// Types of frame alignment of elements with their parent bodies. Used as shortcuts during `mj_kinematics` in the
/// last argument to `mj_local2Global`.
pub type MjtSameFrame = mjtSameFrame;
/// Sleep policy associated with a tree. The compiler automatically chooses between `NEVER` and `ALLOWED`, but the user
/// can override this choice. Only the user can set the `INIT` policy (initialized as asleep).
pub type MjtSleepPolicy = mjtSleepPolicy;
/// Types of flex self-collisions midphase.
pub type MjtFlexSelf = mjtFlexSelf;
/// Formulas used to combine SDFs when calling mjc_distance and mjc_gradient.
pub type MjtSDFType = mjtSDFType;
/// Data fields returned by rangefinder sensors.
pub type MjtRayDataField = mjtRayDataField;
/// Camera output type bitflags.
pub type MjtCamOutBit = mjtCamOutBit;
unsafe impl bytemuck::Zeroable for mjtTrn {}
unsafe impl bytemuck::Zeroable for mjtDyn {}
unsafe impl bytemuck::Zeroable for mjtGain {}
unsafe impl bytemuck::Zeroable for mjtBias {}
unsafe impl bytemuck::Zeroable for mjtObj {}
unsafe impl bytemuck::Zeroable for mjtSameFrame {}
unsafe impl bytemuck::Zeroable for mjtCamLight {}
unsafe impl bytemuck::Zeroable for mjtProjection {}
unsafe impl bytemuck::Zeroable for mjtEq {}
unsafe impl bytemuck::Zeroable for mjtGeom {}
unsafe impl bytemuck::Zeroable for mjtJoint {}
unsafe impl bytemuck::Zeroable for mjtLightType {}
unsafe impl bytemuck::Zeroable for mjtSensor {}
unsafe impl bytemuck::Zeroable for mjtDataType {}
unsafe impl bytemuck::Zeroable for mjtStage {}
unsafe impl bytemuck::Zeroable for mjtTexture {}
unsafe impl bytemuck::Zeroable for mjtColorSpace {}
unsafe impl bytemuck::Zeroable for mjtAlignFree {}
unsafe impl bytemuck::Zeroable for mjtBuiltin {}
unsafe impl bytemuck::Zeroable for mjtConflict {}
unsafe impl bytemuck::Zeroable for mjtConstraint {}
unsafe impl bytemuck::Zeroable for mjtConstraintState {}
unsafe impl bytemuck::Zeroable for mjtDepthMap {}
unsafe impl bytemuck::Zeroable for mjtFlexSelf {}
unsafe impl bytemuck::Zeroable for mjtInertiaFromGeom {}
unsafe impl bytemuck::Zeroable for mjtLimited {}
unsafe impl bytemuck::Zeroable for mjtLogLevel {}
unsafe impl bytemuck::Zeroable for mjtLogTopic {}
unsafe impl bytemuck::Zeroable for mjtMark {}
unsafe impl bytemuck::Zeroable for mjtSleepPolicy {}
unsafe impl bytemuck::Zeroable for mjtSleepState {}
unsafe impl bytemuck::Zeroable for mjtStereo {}
unsafe impl bytemuck::Zeroable for mjtWrap {}
unsafe impl bytemuck::NoUninit for mjtJoint {}
unsafe impl bytemuck::NoUninit for mjtGeom {}
unsafe impl bytemuck::NoUninit for mjtEq {}
unsafe impl bytemuck::NoUninit for mjtObj {}
unsafe impl bytemuck::NoUninit for mjtWrap {}
unsafe impl bytemuck::NoUninit for mjtTrn {}
unsafe impl bytemuck::NoUninit for mjtDyn {}
unsafe impl bytemuck::NoUninit for mjtGain {}
unsafe impl bytemuck::NoUninit for mjtBias {}
unsafe impl bytemuck::NoUninit for mjtSensor {}
unsafe impl bytemuck::NoUninit for mjtDataType {}
unsafe impl bytemuck::NoUninit for mjtStage {}
unsafe impl bytemuck::NoUninit for mjtTexture {}
/// Number of mesh, texture and heightfield count tables in [`MjSplitTables`].
const ASSET_SPLIT_TABLES: usize = 11;
/// Number of per-element and plugin count tables in [`MjSplitTables`].
const ELEMENT_SPLIT_TABLES: usize = 32;
/// Snapshot of an [`MjModel`]: the sizes that no per-element table determines, and the tables that
/// fix how each packed array divides between the elements.
///
/// An entry belongs here only when no other entry already determines it. `signature` is the
/// exception: it takes no part in the comparison at all, and only reports which model an `Info`
/// came from. The fields run cheapest first, because `PartialEq` tests them in that order and
/// stops at the first difference.
#[derive(Debug, Clone, Eq)]
#[expect(
    non_snake_case,
    reason = "the fields keep the MuJoCo size symbol names"
)]
pub(crate) struct MjModelLayout {
    signature: u64,
    nexclude: MjtSize,
    nmat: MjtSize,
    npair: MjtSize,
    nskin: MjtSize,
    nkey: MjtSize,
    nmocap: MjtSize,
    nuserdata: MjtSize,
    nhistory: MjtSize,
    nbvh: MjtSize,
    nbvhdynamic: MjtSize,
    nflexedge: MjtSize,
    nflexstiffness: MjtSize,
    nJmom: MjtSize,
    nJfe: MjtSize,
    nJfv: MjtSize,
    nC: MjtSize,
    nD: MjtSize,
    ntree: MjtSize,
    narena: MjtSize,
    nmeshgraph: MjtSize,
    nuser_body: MjtSize,
    nuser_jnt: MjtSize,
    nuser_geom: MjtSize,
    nuser_site: MjtSize,
    nuser_cam: MjtSize,
    nuser_tendon: MjtSize,
    nuser_actuator: MjtSize,
    nuser_sensor: MjtSize,
    split: MjSplitTables,
}
impl PartialEq for MjModelLayout {
    /// `signature` takes no part: only the compiler writes it, so `mj_loadModel` leaves it zero
    /// and a test would refuse a model against its own saved copy.
    fn eq(&self, other: &Self) -> bool {
        self.nexclude == other.nexclude
            && self.nmat == other.nmat
            && self.npair == other.npair
            && self.nskin == other.nskin
            && self.nkey == other.nkey
            && self.nmocap == other.nmocap
            && self.nuserdata == other.nuserdata
            && self.nhistory == other.nhistory
            && self.nbvh == other.nbvh
            && self.nbvhdynamic == other.nbvhdynamic
            && self.nflexedge == other.nflexedge
            && self.nflexstiffness == other.nflexstiffness
            && self.nJmom == other.nJmom
            && self.nJfe == other.nJfe
            && self.nJfv == other.nJfv
            && self.nC == other.nC
            && self.nD == other.nD
            && self.ntree == other.ntree
            && self.narena == other.narena
            && self.nmeshgraph == other.nmeshgraph
            && self.nuser_body == other.nuser_body
            && self.nuser_jnt == other.nuser_jnt
            && self.nuser_geom == other.nuser_geom
            && self.nuser_site == other.nuser_site
            && self.nuser_cam == other.nuser_cam
            && self.nuser_tendon == other.nuser_tendon
            && self.nuser_actuator == other.nuser_actuator
            && self.nuser_sensor == other.nuser_sensor
            && self.split == other.split
    }
}
impl MjModelLayout {
    /// Returns the compilation signature of the model this layout came from.
    pub(crate) fn signature(&self) -> u64 {
        self.signature
    }
    /// Returns the mesh, texture and heightfield count tables.
    fn asset_split(&self) -> &[Box<[u8]>; ASSET_SPLIT_TABLES] {
        &self.split.assets
    }
}
impl From<&MjModel> for MjModelLayout {
    fn from(model: &MjModel) -> Self {
        let m = model.ffi();
        Self {
            signature: m.signature,
            nexclude: m.nexclude,
            nmat: m.nmat,
            npair: m.npair,
            nskin: m.nskin,
            nkey: m.nkey,
            nmocap: m.nmocap,
            nuserdata: m.nuserdata,
            nhistory: m.nhistory,
            nbvh: m.nbvh,
            nbvhdynamic: m.nbvhdynamic,
            nflexedge: m.nflexedge,
            nflexstiffness: m.nflexstiffness,
            nJmom: m.nJmom,
            nJfe: m.nJfe,
            nJfv: m.nJfv,
            nC: m.nC,
            nD: m.nD,
            ntree: m.ntree,
            narena: m.narena,
            nmeshgraph: m.nmeshgraph,
            nuser_body: m.nuser_body,
            nuser_jnt: m.nuser_jnt,
            nuser_geom: m.nuser_geom,
            nuser_site: m.nuser_site,
            nuser_cam: m.nuser_cam,
            nuser_tendon: m.nuser_tendon,
            nuser_actuator: m.nuser_actuator,
            nuser_sensor: m.nuser_sensor,
            split: model.split_tables(),
        }
    }
}
/// Count tables of an [`MjModel`], one owned table per entry and as raw bytes, so that a
/// comparison tests the length of each table on its own and then its content.
///
/// One flat buffer would test the sum of the lengths instead, which leaves the element count of a
/// single table free.
#[derive(Clone, PartialEq, Eq)]
pub(crate) struct MjSplitTables {
    assets: [Box<[u8]>; ASSET_SPLIT_TABLES],
    elements: [Box<[u8]>; ELEMENT_SPLIT_TABLES],
}
impl Debug for MjSplitTables {
    fn fmt(&self, f: &mut Formatter<'_>) -> FmtResult {
        let assets = self.assets.each_ref().map(|table| table.len());
        let elements = self.elements.each_ref().map(|table| table.len());
        f.debug_struct("MjSplitTables")
            .field("assets", &assets)
            .field("elements", &elements)
            .finish_non_exhaustive()
    }
}
/// A Rust-safe wrapper around mjModel.
/// Automatically clean after itself on destruction.
#[derive(Debug)]
pub struct MjModel {
    ptr: NonNull<mjModel>,
    /// Memory layout for compatibility checks.
    layout: OnceLock<Arc<MjModelLayout>>,
}
unsafe impl Send for MjModel {}
unsafe impl Sync for MjModel {}
impl MjModel {
    /// Loads the model from an XML file. To load from a virtual file system, use [`MjModel::from_xml_vfs`].
    /// Wraps [`mj_loadXML`].
    /// # Returns
    /// On success, returns [`Ok`] variant containing the loaded [`MjModel`].
    /// # Errors
    /// - [`MjModelError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// - [`MjModelError::LoadFailed`] if MuJoCo fails to load the model.
    /// # Panics
    /// - when the `path` contains '\0'.
    /// - when the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_xml<T: AsRef<Path>>(path: T) -> Result<Self, MjModelError> {
        Self::from_xml_file(path, None)
    }
    /// Loads the model from an XML file, located in a virtual file system (`vfs`)
    /// Wraps [`mj_loadXML`].
    /// # Returns
    /// On success, returns [`Ok`] variant containing the loaded [`MjModel`].
    /// # Errors
    /// - [`MjModelError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// - [`MjModelError::LoadFailed`] if MuJoCo fails to load the model.
    /// # Panics
    /// - when the `path` contains '\0'.
    /// - when the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_xml_vfs<T: AsRef<Path>>(path: T, vfs: &MjVfs) -> Result<Self, MjModelError> {
        Self::from_xml_file(path, Some(vfs))
    }
    fn from_xml_file<T: AsRef<Path>>(path: T, vfs: Option<&MjVfs>) -> Result<Self, MjModelError> {
        assert_mujoco_version();
        let mut error_buffer = [0; ERROR_BUF_LEN];
        let path_str = path
            .as_ref()
            .to_str()
            .ok_or(MjModelError::InvalidUtf8Path)?;
        let path = CString::new(path_str).unwrap();
        let raw_ptr = unsafe {
            mj_loadXML(
                path.as_ptr(),
                vfs.map_or(ptr::null(), |v| v.ffi()),
                error_buffer.as_mut_ptr(),
                error_buffer.len() as c_int,
            )
        };
        Self::check_raw_model(raw_ptr, &error_buffer)
            .inspect(|_| debug!("loaded the model from \"{path_str}\""))
    }
    /// Loads the model from an XML string.
    /// Wraps [`mj_loadXML`].
    /// # Returns
    /// On success, returns [`Ok`] variant containing the loaded [`MjModel`].
    /// # Errors
    /// - [`MjModelError::VfsError`] if the internal VFS operation fails.
    /// - [`MjModelError::LoadFailed`] if MuJoCo fails to load the model.
    /// # Panics
    /// Panics if the linked MuJoCo version does not match the version expected by mujoco-rs.
    pub fn from_xml_string(data: &str) -> Result<Self, MjModelError> {
        assert_mujoco_version();
        let mut vfs = MjVfs::new();
        let filename = "model.xml";
        vfs.add_from_buffer(filename, data.as_bytes())?;
        let mut error_buffer = [0; ERROR_BUF_LEN];
        let filename_c = CString::new(filename).unwrap();
        let raw_ptr = unsafe {
            mj_loadXML(
                filename_c.as_ptr(),
                vfs.ffi(),
                error_buffer.as_mut_ptr(),
                error_buffer.len() as c_int,
            )
        };
        Self::check_raw_model(raw_ptr, &error_buffer)
    }
    /// Loads the model from MJB raw data.
    /// Wraps [`mj_loadModelBuffer`].
    /// # Returns
    /// On success, returns [`Ok`] variant containing the loaded [`MjModel`].
    /// # Errors
    /// Returns [`MjModelError::LoadFailed`] if MuJoCo fails to parse the MJB buffer.
    /// # Panics
    /// When the linked MuJoCo version does not match the expected from MuJoCo-rs, or when `data`
    /// is longer than [`i32::MAX`] bytes.
    pub fn from_buffer(data: &[u8]) -> Result<Self, MjModelError> {
        assert_mujoco_version();
        unsafe {
            Self::from_raw(mj_loadModelBuffer(
                data.as_ptr() as *const c_void,
                checked_c_len(data.len()),
            ))
        }
    }
    /// Creates a [`MjModel`] from a raw pointer.
    pub(crate) fn from_raw(ptr: *mut mjModel) -> Result<Self, MjModelError> {
        Self::check_raw_model(ptr, &[0])
    }
    /// Saves the last loaded XML to `filename`.
    /// Wraps [`mj_saveLastXML`].
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjModelError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// - [`MjModelError::SaveFailed`] with MuJoCo's error message if saving fails.
    /// # Panics
    /// When the path contains '\0' characters, a panic occurs.
    pub fn save_last_xml<T: AsRef<Path>>(&self, filename: T) -> Result<(), MjModelError> {
        let path_str = filename
            .as_ref()
            .to_str()
            .ok_or(MjModelError::InvalidUtf8Path)?;
        let mut error = [0; ERROR_BUF_LEN];
        let cstring = CString::new(path_str).unwrap();
        let result = unsafe {
            mj_saveLastXML(
                cstring.as_ptr(),
                self.ffi(),
                error.as_mut_ptr(),
                error.len() as i32,
            )
        };
        match result {
            1 => {
                debug!("saved the last loaded XML to \"{path_str}\"");
                Ok(())
            }
            _ => {
                let cstr_error = unsafe { CStr::from_ptr(error.as_ptr()) }
                    .to_string_lossy()
                    .into_owned();
                Err(MjModelError::SaveFailed(cstr_error))
            }
        }
    }
    /// Creates a new [`MjData`] instance linked to this model.
    ///
    /// # Panics
    /// Panics if MuJoCo fails to allocate the data structure.
    /// Use [`MjModel::try_make_data`] for a fallible alternative.
    pub fn make_data(&self) -> MjData<&Self> {
        MjData::new(self)
    }
    /// Fallible version of [`MjModel::make_data`].
    ///
    /// # Errors
    /// Returns [`MjDataError::AllocationFailed`] if MuJoCo fails to allocate
    /// the data structure.
    pub fn try_make_data(&self) -> Result<MjData<&Self>, MjDataError> {
        MjData::try_new(self)
    }
    /// Wraps a raw model pointer returned by MuJoCo load functions.
    /// Returns an error with the C error-buffer message if the pointer is null.
    fn check_raw_model(
        ptr_model: *mut mjModel,
        error_buffer: &[c_char],
    ) -> Result<Self, MjModelError> {
        match NonNull::new(ptr_model) {
            Some(nn) => Ok(Self {
                ptr: nn,
                layout: OnceLock::new(),
            }),
            None => {
                let message = unsafe { CStr::from_ptr(error_buffer.as_ptr()) }
                    .to_string_lossy()
                    .into_owned();
                Err(MjModelError::LoadFailed(message))
            }
        }
    }
    info_method! {
        Model, actuator, [trntype : 1, dyntype : 1, gaintype : 1, biastype : 1, ctrladr :
        1, ctrlnum : 1, ctrlspec : 1, outadr : 1, outnum : 1, trnid : 2, actadr : 1,
        actnum : 1, group : 1, history : 2, historyadr : 1, delay : 1, forcelimited : 1,
        actlimited : 1, dynprm : mjNDYN as usize, gainprm : mjNGAIN as usize, biasprm :
        mjNBIAS as usize, actearly : 1, forcerange : 2, actrange : 2, damping : 1,
        dampingpoly : mjNPOLY as usize, armature : 1, cranklength : 1, plugin : 1], [user
        : nuser_actuator], [ctrllimited : nu, ctrlrange : nu * 2, gear : nout * 6, acc0 :
        nout, length0 : nout, lengthrange : nout * 2]
    }
    info_method! {
        Model, body, [parentid : 1, rootid : 1, weldid : 1, mocapid : 1, jntnum : 1,
        jntadr : 1, dofnum : 1, dofadr : 1, treeid : 1, geomnum : 1, geomadr : 1, simple
        : 1, sameframe : 1, pos : 3, quat : 4, ipos : 3, iquat : 4, mass : 1, subtreemass
        : 1, inertia : 3, invweight0 : 2, gravcomp : 1, margin : 1, plugin : 1, contype :
        1, conaffinity : 1, bvhadr : 1, bvhnum : 1], [user : nuser_body], []
    }
    info_method! {
        Model, camera, [mode : 1, bodyid : 1, targetbodyid : 1, pos : 3, quat : 4,
        poscom0 : 3, pos0 : 3, mat0 : 9, projection : 1, fovy : 1, ipd : 1, resolution :
        2, output : 1, sensorsize : 2, intrinsic : 4], [user : nuser_cam], []
    }
    info_method! {
        Model, joint, [r#type : 1, qposadr : 1, dofadr : 1, group : 1, limited : 1,
        actfrclimited : 1, actgravcomp : 1, solref : mjNREF as usize, solimp : mjNIMP as
        usize, pos : 3, axis : 3, stiffness : 1, stiffnesspoly : mjNPOLY as usize, range
        : 2, actfrcrange : 2, margin : 1, bodyid : 1, actuatorid : 1], [user :
        nuser_jnt], [qpos0 : nq, qpos_spring : nq, jntid : nv, dof_bodyid : nv, parentid
        : nv, dof_treeid : nv, Madr : nv, simplenum : nv, frictionloss : nv, armature :
        nv, damping : nv, dampingpoly : nv * mjNPOLY as usize, invweight0 : nv, M0 : nv]
    }
    info_method! {
        Model, equality, [r#type : 1, obj1id : 1, obj2id : 1, active0 : 1, solref :
        mjNREF as usize, solimp : mjNIMP as usize, data : mjNEQDATA as usize, objtype :
        1], [], []
    }
    info_method! {
        Model, exclude, [signature : 1], [], []
    }
    info_method! {
        Model, geom, [r#type : 1, contype : 1, conaffinity : 1, condim : 1, bodyid : 1,
        dataid : 1, matid : 1, group : 1, priority : 1, plugin : 1, sameframe : 1, solmix
        : 1, solref : mjNREF as usize, solimp : mjNIMP as usize, size : 3, aabb : 6,
        rbound : 1, pos : 3, quat : 4, friction : 3, margin : 1, gap : 1, surfacevel : 6,
        adhesion : 1, fluid : mjNFLUID as usize, rgba : 4], [user : nuser_geom], []
    }
    info_method! {
        Model, hfield, [size : 4, nrow : 1, ncol : 1, adr : 1, pathadr : 1], [], [data :
        nhfielddata]
    }
    info_method! {
        Model, light, [mode : 1, bodyid : 1, targetbodyid : 1, r#type : 1, texid : 1,
        castshadow : 1, bulbradius : 1, intensity : 1, range : 1, active : 1, pos : 3,
        dir : 3, poscom0 : 3, pos0 : 3, dir0 : 3, attenuation : 3, cutoff : 1, softness :
        1, exponent : 1, ambient : 3, diffuse : 3, specular : 3], [], []
    }
    info_method! {
        Model, material, [texid : MjtTextureRole::mjNTEXROLE as usize, texuniform : 1,
        texrepeat : 2, emission : 1, specular : 1, shininess : 1, reflectance : 1, rgba :
        4, metallic : 1, roughness : 1], [], []
    }
    info_method! {
        Model, mesh, [vertadr : 1, vertnum : 1, texcoordadr : 1, faceadr : 1, facenum :
        1, graphadr : 1, extrema : 27, normaladr : 1, normalnum : 1, texcoordnum : 1,
        bvhadr : 1, bvhnum : 1, octadr : 1, octnum : 1, pathadr : 1, polynum : 1, polyadr
        : 1, scale : 3, pos : 3, quat : 4], [], []
    }
    info_method! {
        Model, numeric, [adr : 1, size : 1], [], [data : nnumericdata]
    }
    info_method! {
        Model, pair, [dim : 1, geom1 : 1, geom2 : 1, signature : 1, solref : mjNREF as
        usize, solimp : mjNIMP as usize, margin : 1, gap : 1, adhesion : 1, friction : 5,
        solreffriction : mjNREF as usize], [], []
    }
    info_method! {
        Model, sensor, [r#type : 1, datatype : 1, needstage : 1, objtype : 1, objid : 1,
        reftype : 1, refid : 1, intprm : mjNSENS as usize, dim : 1, adr : 1, cutoff : 1,
        noise : 1, history : 2, historyadr : 1, delay : 1, interval : 2, plugin : 1],
        [user : nuser_sensor], []
    }
    info_method! {
        Model, site, [r#type : 1, bodyid : 1, matid : 1, group : 1, sameframe : 1, size :
        3, pos : 3, quat : 4, rgba : 4], [user : nuser_site], []
    }
    info_method! {
        Model, skin, [matid : 1, group : 1, rgba : 4, inflate : 1, vertadr : 1, vertnum :
        1, texcoordadr : 1, faceadr : 1, facenum : 1, boneadr : 1, bonenum : 1, pathadr :
        1], [], []
    }
    info_method! {
        Model, tendon, [adr : 1, num : 1, matid : 1, actuatorid : 1, group : 1, treenum :
        1, treeid : 2, limited : 1, actfrclimited : 1, width : 1, solref_lim : mjNREF as
        usize, solimp_lim : mjNIMP as usize, solref_fri : mjNREF as usize, solimp_fri :
        mjNIMP as usize, range : 2, actfrcrange : 2, margin : 1, stiffness : 1,
        stiffnesspoly : mjNPOLY as usize, damping : 1, dampingpoly : mjNPOLY as usize,
        armature : 1, frictionloss : 1, lengthspring : 2, length0 : 1, invweight0 : 1,
        J_rownnz : 1, J_rowadr : 1, rgba : 4], [user : nuser_tendon], [J_colind : nJten]
    }
    info_method! {
        Model, texture, [r#type : 1, colorspace : 1, height : 1, width : 1, nchannel : 1,
        adr : 1, pathadr : 1], [], [data : ntexdata]
    }
    info_method! {
        Model, tuple, [adr : 1, size : 1], [], [objtype : ntupledata, objid : ntupledata,
        objprm : ntupledata]
    }
    info_method! {
        Model, key, [time : 1], [qpos : nq, qvel : nv, act : na, mpos : nmocap * 3, mquat
        : nmocap * 4, ctrl : nu], []
    }
    /// Translates `name` to the correct id. Wrapper around `mj_name2id`.
    /// Returns `None` if the name is not found.
    /// Wraps [`mj_name2id`].
    /// # Panics
    /// When the `name` contains '\0' characters, a panic occurs.
    pub fn name_to_id(&self, type_: MjtObj, name: &str) -> Option<usize> {
        let c_string = CString::new(name).unwrap();
        let id = unsafe { mj_name2id(self.ffi(), type_ as i32, c_string.as_ptr()) };
        if id == -1 { None } else { Some(id as usize) }
    }
    /// Fallible version of [`Clone::clone`].
    ///
    /// Wraps [`mj_copyModel`].
    /// # Note
    /// MuJoCo ends the process when the allocation fails, so this never returns `Err`.
    ///
    /// # Errors
    /// Returns [`MjModelError::AllocationFailed`] if MuJoCo returns a null model.
    #[deprecated(since = "6.0.0", note = "always returns Ok; use `clone`")]
    pub fn try_clone(&self) -> Result<MjModel, MjModelError> {
        let ptr = unsafe { mj_copyModel(ptr::null_mut(), self.ffi()) };
        NonNull::new(ptr)
            .map(|ptr| MjModel {
                ptr,
                layout: self.layout.clone(),
            })
            .ok_or(MjModelError::AllocationFailed)
    }
    /// Save model to binary MJB file.
    ///
    /// Wraps [`mj_saveModel`].
    /// # Returns
    /// `Ok(())` if the path is valid UTF-8 and contains no interior `\0` characters.
    /// **Note:** the underlying C function `mj_saveModel` returns `void`, so file I/O
    /// errors (e.g. permission denied, disk full) are not detectable; `Ok(())` does
    /// **not** guarantee the file was written.
    /// # Errors
    /// - [`MjModelError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// # Panics
    /// When the filename contains '\0' characters, a panic occurs.
    pub fn save_to_file<T: AsRef<Path>>(&self, filename: T) -> Result<(), MjModelError> {
        let path_str = filename
            .as_ref()
            .to_str()
            .ok_or(MjModelError::InvalidUtf8Path)?;
        let c_filename = CString::new(path_str).unwrap();
        unsafe { mj_saveModel(self.ffi(), c_filename.as_ptr(), ptr::null_mut(), 0) };
        debug!("saved the model to \"{path_str}\"");
        Ok(())
    }
    /// Save model to memory buffer.
    /// Wraps [`mj_saveModel`].
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjModelError::BufferTooSmall`] if the buffer is smaller than [`size()`](Self::size).
    pub fn save_to_buffer(&self, buffer: &mut [u8]) -> Result<(), MjModelError> {
        let needed = self.size();
        if buffer.len() < needed {
            return Err(MjModelError::BufferTooSmall {
                needed,
                available: buffer.len(),
            });
        }
        unsafe {
            mj_saveModel(
                self.ffi(),
                ptr::null(),
                buffer.as_mut_ptr() as *mut c_void,
                buffer.len() as i32,
            )
        };
        Ok(())
    }
    /// Return size of buffer needed to hold model.
    /// Wraps [`mj_sizeModel`].
    pub fn size(&self) -> usize {
        unsafe { mj_sizeModel(self.ffi()) as usize }
    }
    /// Print mjModel to text file, specifying format.
    /// float_format must be a valid printf-style format string for a single float value.
    ///
    /// Wraps [`mj_printFormattedModel`].
    /// # Returns
    /// `Ok(())` if the path and format string are valid UTF-8 and contain no interior `\0`
    /// characters. **Note:** the underlying C function `mj_printFormattedModel` returns `void`,
    /// so file I/O errors are not detectable; `Ok(())` does **not** guarantee the file was written.
    /// # Errors
    /// - [`MjModelError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// # Panics
    /// When either string contains '\0' characters, a panic occurs.
    pub fn print_formatted<T: AsRef<Path>>(
        &self,
        filename: T,
        float_format: &str,
    ) -> Result<(), MjModelError> {
        let path_str = filename
            .as_ref()
            .to_str()
            .ok_or(MjModelError::InvalidUtf8Path)?;
        let c_filename = CString::new(path_str).unwrap();
        let c_float_format = CString::new(float_format).unwrap();
        unsafe { mj_printFormattedModel(self.ffi(), c_filename.as_ptr(), c_float_format.as_ptr()) }
        Ok(())
    }
    /// Print model to text file.
    ///
    /// Wraps [`mj_printModel`].
    /// # Returns
    /// `Ok(())` if the path is valid UTF-8 and contains no interior `\0` characters.
    /// **Note:** the underlying C function `mj_printModel` returns `void`, so file I/O
    /// errors are not detectable; `Ok(())` does **not** guarantee the file was written.
    /// # Errors
    /// - [`MjModelError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// # Panics
    /// When the filename contains '\0' characters, a panic occurs.
    pub fn print<T: AsRef<Path>>(&self, filename: T) -> Result<(), MjModelError> {
        let path_str = filename
            .as_ref()
            .to_str()
            .ok_or(MjModelError::InvalidUtf8Path)?;
        let c_filename = CString::new(path_str).unwrap();
        unsafe { mj_printModel(self.ffi(), c_filename.as_ptr()) }
        Ok(())
    }
    /// Return size of state specification. The bits of the integer spec correspond to element fields of [`MjtState`](crate::native_binding::wrappers::mj_data::MjtState).
    /// Wraps [`mj_stateSize`].
    /// # Note
    /// MuJoCo aborts the process through `mjERROR` when `spec` is not below `1 << mjNSTATE`.
    pub fn state_size(&self, spec: u32) -> usize {
        unsafe { mj_stateSize(self.ffi(), spec as i32) as usize }
    }
    /// Extract the subset of components specified by `dst_spec` from a state `src`
    /// previously obtained via [`MjData::read_state_into`] or [`MjData::state`]
    /// with components specified by `src_spec`.
    ///
    /// Wraps [`mj_extractState`].
    /// # Note
    /// MuJoCo aborts the process through `mjERROR` when `src_spec` is not below
    /// `1 << mjNSTATE`.
    ///
    /// # Panics
    /// - When `src.len()` does not equal the size required by `src_spec`.
    /// - When `dst_spec` is not a subset of `src_spec`.
    ///
    /// Use [`MjModel::try_extract_state`] for a fallible alternative.
    pub fn extract_state(&self, src: &[MjtNum], src_spec: u32, dst_spec: u32) -> Box<[MjtNum]> {
        self.try_extract_state(src, src_spec, dst_spec).unwrap()
    }
    /// Fallible version of [`MjModel::extract_state`].
    ///
    /// Wraps [`mj_extractState`].
    /// # Note
    /// MuJoCo aborts the process through `mjERROR` when `src_spec` is not below
    /// `1 << mjNSTATE`.
    /// # Returns
    /// On success, returns [`Ok`] variant containing the extracted state.
    /// # Errors
    /// - When `src.len()` does not equal the size required by `src_spec`, [`MjModelError::StateSliceLengthMismatch`] is returned.
    /// - When `dst_spec` is not a subset of `src_spec`, [`MjModelError::SpecNotSubset`] is returned.
    pub fn try_extract_state(
        &self,
        src: &[MjtNum],
        src_spec: u32,
        dst_spec: u32,
    ) -> Result<Box<[MjtNum]>, MjModelError> {
        let expected = self.state_size(src_spec);
        if src.len() != expected {
            return Err(MjModelError::StateSliceLengthMismatch {
                expected,
                got: src.len(),
            });
        }
        if (dst_spec & src_spec) != dst_spec {
            return Err(MjModelError::SpecNotSubset);
        }
        let required_size = self.state_size(dst_spec);
        let mut dst = Vec::with_capacity(required_size);
        unsafe {
            mj_extractState(
                self.ffi(),
                src.as_ptr(),
                src_spec as i32,
                dst.as_mut_ptr(),
                dst_spec as i32,
            );
            dst.set_len(required_size);
            Ok(dst.into_boxed_slice())
        }
    }
    /// Extract into dst the subset of components specified by `dst_spec` from a state `src`
    /// previously obtained via [`MjData::read_state_into`] or [`MjData::state`]
    /// with components specified by `src_spec`.
    ///
    /// Wraps [`mj_extractState`].
    /// # Note
    /// MuJoCo aborts the process through `mjERROR` when `src_spec` is not below
    /// `1 << mjNSTATE`.
    ///
    /// # Panics
    /// - When `src.len()` does not equal the size required by `src_spec`.
    /// - When `dst_spec` is not a subset of `src_spec`.
    /// - When `dst` is too small to hold the requested components.
    ///
    /// Use [`MjModel::try_extract_state_into`] for a fallible alternative.
    pub fn extract_state_into(
        &self,
        src: &[MjtNum],
        src_spec: u32,
        dst: &mut [MjtNum],
        dst_spec: u32,
    ) -> usize {
        self.try_extract_state_into(src, src_spec, dst, dst_spec)
            .unwrap()
    }
    /// Fallible version of [`MjModel::extract_state_into`].
    ///
    /// Wraps [`mj_extractState`].
    /// # Note
    /// MuJoCo aborts the process through `mjERROR` when `src_spec` is not below
    /// `1 << mjNSTATE`.
    /// # Returns
    /// On success, returns [`Ok`] variant containing the number of elements written to `dst`.
    /// # Errors
    /// - When `src.len()` does not equal the size required by `src_spec`, [`MjModelError::StateSliceLengthMismatch`] is returned.
    /// - When `dst_spec` is not a subset of `src_spec`, [`MjModelError::SpecNotSubset`] is returned.
    /// - When `dst` is too small to hold the requested components, [`MjModelError::BufferTooSmall`] is returned.
    pub fn try_extract_state_into(
        &self,
        src: &[MjtNum],
        src_spec: u32,
        dst: &mut [MjtNum],
        dst_spec: u32,
    ) -> Result<usize, MjModelError> {
        let expected = self.state_size(src_spec);
        if src.len() != expected {
            return Err(MjModelError::StateSliceLengthMismatch {
                expected,
                got: src.len(),
            });
        }
        if (dst_spec & src_spec) != dst_spec {
            return Err(MjModelError::SpecNotSubset);
        }
        let required_size = self.state_size(dst_spec);
        let available_size = dst.len();
        if available_size < required_size {
            return Err(MjModelError::BufferTooSmall {
                needed: required_size,
                available: available_size,
            });
        }
        unsafe {
            mj_extractState(
                self.ffi(),
                src.as_ptr(),
                src_spec as i32,
                dst.as_mut_ptr(),
                dst_spec as i32,
            );
        }
        Ok(required_size)
    }
    /// Determine type of friction cone. Returns `true` if pyramidal, `false` if elliptic.
    /// Wraps [`mj_isPyramidal`].
    pub fn is_pyramidal(&self) -> bool {
        unsafe { mj_isPyramidal(self.ffi()) == 1 }
    }
    /// Determine type of constraint Jacobian. Returns `true` if sparse, `false` if dense.
    /// Wraps [`mj_isSparse`].
    pub fn is_sparse(&self) -> bool {
        unsafe { mj_isSparse(self.ffi()) == 1 }
    }
    /// Determine type of solver. Returns `true` for a dual solver: PGS, or any solver with
    /// `noslip_iterations > 0`.
    /// Wraps [`mj_isDual`].
    pub fn is_dual(&self) -> bool {
        unsafe { mj_isDual(self.ffi()) == 1 }
    }
    /// Get name of object with the specified [`MjtObj`] type and id, returns `None` if name not found.
    /// Wraps [`mj_id2name`].
    /// # Panics
    /// Panics if MuJoCo internally returns a C string that is not valid UTF-8.
    pub fn id_to_name(&self, type_: MjtObj, id: usize) -> Option<&str> {
        let ptr = unsafe { mj_id2name(self.ffi(), type_ as i32, id as i32) };
        if ptr.is_null() {
            None
        } else {
            let cstr = unsafe { CStr::from_ptr(ptr).to_str().unwrap() };
            Some(cstr)
        }
    }
    /// Sum all body masses.
    /// Wraps [`mj_getTotalmass`].
    pub fn totalmass(&self) -> MjtNum {
        unsafe { mj_getTotalmass(self.ffi()) }
    }
    /// Scale body masses and inertias to achieve specified total mass.
    /// Wraps [`mj_setTotalmass`].
    pub fn set_totalmass(&mut self, newmass: MjtNum) {
        unsafe { mj_setTotalmass(self.ffi_mut(), newmass) }
    }
    /// Return the maximum number of contacts that can be generated between two geoms.
    ///
    /// To pull margin from model, set `has_margin` to [`None`], otherwise pass `true` or `false`
    /// inside [`Some`] (true indicating a present margin).
    ///
    /// Wraps [`mj_maxContact`].
    /// # Panics
    /// Panics when either `geom1` or `geom2` are equal or greater than [`MjModel::ngeom`].
    /// Use [`MjModel::try_max_contacts`] for a fallible alternative.
    pub fn max_contacts(&self, geom1: usize, geom2: usize, has_margin: Option<bool>) -> u32 {
        self.try_max_contacts(geom1, geom2, has_margin).unwrap()
    }
    /// Fallible version of [`MjModel::max_contacts`].
    /// Wraps [`mj_maxContact`].
    /// # Errors
    /// Returns [`MjModelError::IndexOutOfBounds`] when either `geom1` or `geom2` are equal or
    /// greater than [`MjModel::ngeom`].
    pub fn try_max_contacts(
        &self,
        geom1: usize,
        geom2: usize,
        has_margin: Option<bool>,
    ) -> Result<u32, MjModelError> {
        let ngeom = self.ngeom() as usize;
        if geom1 >= ngeom {
            return Err(MjModelError::IndexOutOfBounds {
                id: geom1,
                len: ngeom,
            });
        }
        if geom2 >= ngeom {
            return Err(MjModelError::IndexOutOfBounds {
                id: geom2,
                len: ngeom,
            });
        }
        Ok(unsafe {
            mj_maxContact(
                self.ffi(),
                geom1 as i32,
                geom2 as i32,
                has_margin.map(|m| m as i32).unwrap_or(-1),
            ) as u32
        })
    }
    /// Returns the name that the type of actuator `id` gives to its control input `input`, an
    /// index into the control block of the actuator. For example, an orientation servo on the
    /// exponential-map chart names its first input `"rx"`.
    /// Wraps [`mj_actuatorInputName`].
    /// # Returns
    /// [`None`] when the actuator type defines no input names, or when `id` or `input` is out of
    /// range.
    /// # Panics
    /// Panics if the reported name is not valid UTF-8.
    pub fn actuator_input_name(&self, id: usize, input: usize) -> Option<&'static str> {
        unsafe {
            let c_ptr = mj_actuatorInputName(self.ffi(), id as i32, input as i32);
            (!c_ptr.is_null()).then(|| CStr::from_ptr(c_ptr).to_str().unwrap())
        }
    }
    /// Returns a reference to the wrapped FFI struct.
    pub fn ffi(&self) -> &mjModel {
        unsafe { self.ptr.as_ref() }
    }
    /// Returns a mutable reference to the wrapped FFI struct.
    ///
    /// # Safety
    /// The caller must ensure that any modifications to the underlying struct preserve
    /// the invariants that MuJoCo expects (e.g. do not corrupt computed fields or
    /// break index relationships). Violating these invariants can cause undefined behavior.
    /// A write that changes a size or an address table also leaves the cached layout stale, so
    /// [`MjModel::is_compatible_with_model`] then answers for the model as it was loaded.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjModel {
        unsafe { self.ptr.as_mut() }
    }
}
/// Public attribute methods.
impl MjModel {
    /// Compilation signature.
    pub fn signature(&self) -> u64 {
        self.ffi().signature
    }
    /// Reports whether `other` can take the place of this model in every object that this model
    /// built: an [`MjData`], and the index ranges an `Info` caches.
    ///
    /// The test covers every size that fixes an `mjData` buffer or a packed `mjModel` array, and
    /// the tables that fix how each array divides between the elements: the per-element counts,
    /// the joint addresses, the kinematic tree, the body of every element, and the type of every
    /// joint, geom, equality, wrap, actuator and sensor. [`MjModel::signature`] takes no part:
    /// `mj_saveModel` does not write it, so a model that came back from a buffer carries a zero.
    pub fn is_compatible_with_model(&self, other: &MjModel) -> bool {
        self.layout() == other.layout()
    }
    /// Reports whether `other` keeps its mesh, texture and heightfield data in the same memory
    /// as this model, and gives every texture the same kind: the same shape for every asset, and
    /// the same convex hull total.
    ///
    /// The count tables carry every other total, because each one is the plain sum, or the sum of
    /// the products, of the tables beside it. `nmeshgraph` is the exception: qhull sizes each
    /// convex hull and `mesh_graphadr` holds addresses only.
    pub fn is_asset_compatible_with_model(&self, other: &MjModel) -> bool {
        self.nmeshgraph() == other.nmeshgraph()
            && self.layout().asset_split() == other.layout().asset_split()
    }
    /// Returns the per-sensor, per-numeric, per-tuple, per-actuator, per-tendon, per-flex and
    /// plugin count tables, as raw bytes in a fixed order.
    ///
    /// Two models can hold the same element count and the same data total and still split that
    /// total differently. A caller that resolves one element through a range read from the other
    /// model then reads or writes the neighbouring element, and no length ever disagrees. Each
    /// address table is the running prefix sum of the count table beside it, so the counts pin
    /// the addresses and the address tables need no entry. The total of a packed array is the
    /// plain sum of the same counts, so it needs no entry either. Each table also enters the
    /// comparison with its own length, so a table pins the count of the elements it describes.
    /// `mjModel` holds no per-joint count array, and `jnt_type` fills that role: every `mjtJoint`
    /// value carries one fixed qpos and dof footprint, so `jnt_qposadr` and `jnt_dofadr` are the
    /// running prefix sums of the types.
    fn element_split_tables(&self) -> [&[u8]; ELEMENT_SPLIT_TABLES] {
        [
            must_cast_slice(self.sensor_dim()),
            must_cast_slice(self.numeric_size()),
            must_cast_slice(self.tuple_size()),
            must_cast_slice(self.actuator_actnum()),
            must_cast_slice(self.actuator_ctrlnum()),
            must_cast_slice(self.actuator_outnum()),
            must_cast_slice(self.ten_j_rownnz()),
            must_cast_slice(self.flex_dim()),
            must_cast_slice(self.flex_vertnum()),
            must_cast_slice(self.flex_elemnum()),
            must_cast_slice(self.plugin()),
            must_cast_slice(self.plugin_statenum()),
            must_cast_slice(self.jnt_type()),
            must_cast_slice(self.geom_type()),
            must_cast_slice(self.eq_type()),
            must_cast_slice(self.eq_objtype()),
            must_cast_slice(self.wrap_type()),
            must_cast_slice(self.actuator_trntype()),
            must_cast_slice(self.actuator_dyntype()),
            must_cast_slice(self.actuator_gaintype()),
            must_cast_slice(self.actuator_biastype()),
            must_cast_slice(self.sensor_type()),
            must_cast_slice(self.sensor_objtype()),
            must_cast_slice(self.sensor_reftype()),
            must_cast_slice(self.sensor_datatype()),
            must_cast_slice(self.sensor_needstage()),
            must_cast_slice(self.body_parentid()),
            must_cast_slice(self.jnt_bodyid()),
            must_cast_slice(self.geom_bodyid()),
            must_cast_slice(self.site_bodyid()),
            must_cast_slice(self.cam_bodyid()),
            must_cast_slice(self.light_bodyid()),
        ]
    }
    /// Returns the per-mesh, per-texture and per-heightfield count tables, plus the texture kinds,
    /// as raw bytes in a fixed order.
    fn asset_split_tables(&self) -> [&[u8]; ASSET_SPLIT_TABLES] {
        [
            must_cast_slice(self.mesh_vertnum()),
            must_cast_slice(self.mesh_normalnum()),
            must_cast_slice(self.mesh_texcoordnum()),
            must_cast_slice(self.mesh_facenum()),
            must_cast_slice(self.mesh_graphadr()),
            must_cast_slice(self.tex_width()),
            must_cast_slice(self.tex_height()),
            must_cast_slice(self.tex_nchannel()),
            must_cast_slice(self.tex_type()),
            must_cast_slice(self.hfield_nrow()),
            must_cast_slice(self.hfield_ncol()),
        ]
    }
    /// Copies every asset table and every element table into its own buffer.
    fn split_tables(&self) -> MjSplitTables {
        MjSplitTables {
            assets: self.asset_split_tables().map(Box::from),
            elements: self.element_split_tables().map(Box::from),
        }
    }
    /// Returns the memory layout snapshot of this model.
    pub(crate) fn layout(&self) -> &Arc<MjModelLayout> {
        self.layout
            .get_or_init(|| Arc::new(MjModelLayout::from(self)))
    }
    getter_setter! {
        get, [[ffi] nq : MjtSize; "number of generalized coordinates = dim(qpos)."; [ffi]
        nv : MjtSize; "number of degrees of freedom = dim(qvel)."; [ffi] nu : MjtSize;
        "number of scalar controls = dim(ctrl)."; [ffi] nactuator : MjtSize;
        "number of actuators."; [ffi] nout : MjtSize;
        "number of force outputs, derived from transmission type."; [ffi] na : MjtSize;
        "number of activation states = dim(act)."; [ffi] nbody : MjtSize;
        "number of bodies."; [ffi] nbvh : MjtSize;
        "number of total bounding volumes in all bodies."; [ffi] nbvhstatic : MjtSize;
        "number of static bounding volumes (aabb stored in mjModel)."; [ffi] nbvhdynamic
        : MjtSize; "number of dynamic bounding volumes (aabb stored in mjData)."; [ffi]
        noct : MjtSize; "number of total octree cells in all meshes."; [ffi] njnt :
        MjtSize; "number of joints."; [ffi] ntree : MjtSize;
        "number of kinematic trees under world body."; [ffi] nM : MjtSize;
        "number of non-zeros in sparse inertia matrix."; [ffi] nB : MjtSize;
        "number of non-zeros in sparse body-dof matrix."; [ffi] nC : MjtSize;
        "number of non-zeros in sparse reduced dof-dof matrix."; [ffi] nD : MjtSize;
        "number of non-zeros in sparse dof-dof matrix."; [ffi] ngeom : MjtSize;
        "number of geoms."; [ffi] nsite : MjtSize; "number of sites."; [ffi] ncam :
        MjtSize; "number of cameras."; [ffi] nlight : MjtSize; "number of lights."; [ffi]
        nflex : MjtSize; "number of flexes."; [ffi] nflexnode : MjtSize;
        "number of dofs in all flexes."; [ffi] nflexvert : MjtSize;
        "number of vertices in all flexes."; [ffi] nflexedge : MjtSize;
        "number of edges in all flexes."; [ffi] nflexelem : MjtSize;
        "number of elements in all flexes."; [ffi] nflexelemdata : MjtSize;
        "number of element vertex ids in all flexes."; [ffi] nflexstiffness : MjtSize;
        "number of stiffness parameters in all flexes."; [ffi] nflexbending : MjtSize;
        "number of bending parameters in all flexes"; [ffi] nefm0dof : MjtSize;
        "number of dofs covered by the constant metric factor."; [ffi] nefm0L : MjtSize;
        "number of non-zeros in the constant metric factor."; [ffi] nflexelemedge :
        MjtSize; "number of element edge ids in all flexes."; [ffi] nflexshelldata :
        MjtSize; "number of shell fragment vertex ids in all flexes."; [ffi] nflexevpair
        : MjtSize; "number of element-vertex pairs in all flexes."; [ffi] nflextexcoord :
        MjtSize; "number of vertices with texture coordinates."; [ffi] nJfe : MjtSize;
        "number of non-zeros in sparse flexedge Jacobian matrix."; [ffi] nJfv : MjtSize;
        "number of non-zeros in sparse flexvert Jacobian matrix."; [ffi] nmesh : MjtSize;
        "number of meshes."; [ffi] nmeshvert : MjtSize;
        "number of vertices in all meshes."; [ffi] nmeshnormal : MjtSize;
        "number of normals in all meshes."; [ffi] nmeshtexcoord : MjtSize;
        "number of texcoords in all meshes."; [ffi] nmeshface : MjtSize;
        "number of triangular faces in all meshes."; [ffi] nmeshgraph : MjtSize;
        "number of ints in mesh auxiliary data."; [ffi] nmeshpoly : MjtSize;
        "number of polygons in all meshes."; [ffi] nmeshpolyvert : MjtSize;
        "number of vertices in all polygons."; [ffi] nmeshpolymap : MjtSize;
        "number of polygons in vertex map."; [ffi] nskin : MjtSize; "number of skins.";
        [ffi] nskinvert : MjtSize; "number of vertices in all skins."; [ffi] nskintexvert
        : MjtSize; "number of vertices with texcoords in all skins."; [ffi] nskinface :
        MjtSize; "number of triangular faces in all skins."; [ffi] nskinbone : MjtSize;
        "number of bones in all skins."; [ffi] nskinbonevert : MjtSize;
        "number of vertices in all skin bones."; [ffi] nhfield : MjtSize;
        "number of heightfields."; [ffi] nhfielddata : MjtSize;
        "number of data points in all heightfields."; [ffi] ntex : MjtSize;
        "number of textures."; [ffi] ntexdata : MjtSize;
        "number of bytes in texture rgb data."; [ffi] nmat : MjtSize;
        "number of materials."; [ffi] npair : MjtSize;
        "number of predefined geom pairs."; [ffi] nexclude : MjtSize;
        "number of excluded geom pairs."; [ffi] neq : MjtSize;
        "number of equality constraints."; [ffi] ntendon : MjtSize; "number of tendons.";
        [ffi] nJten : MjtSize; "number of non-zeros in sparse tendon Jacobian matrix.";
        [ffi] nwrap : MjtSize; "number of wrap objects in all tendon paths."; [ffi]
        nsensor : MjtSize; "number of sensors."; [ffi] nnumeric : MjtSize;
        "number of numeric custom fields."; [ffi] nnumericdata : MjtSize;
        "number of mjtNums in all numeric fields."; [ffi] ntext : MjtSize;
        "number of text custom fields."; [ffi] ntextdata : MjtSize;
        "number of mjtBytes in all text fields."; [ffi] ntuple : MjtSize;
        "number of tuple custom fields."; [ffi] ntupledata : MjtSize;
        "number of objects in all tuple fields."; [ffi] nkey : MjtSize;
        "number of keyframes."; [ffi] nmocap : MjtSize; "number of mocap bodies."; [ffi]
        nplugin : MjtSize; "number of plugin instances."; [ffi] npluginattr : MjtSize;
        "number of chars in all plugin config attributes."; [ffi] nuser_body : MjtSize;
        "number of mjtNums in body_user."; [ffi] nuser_jnt : MjtSize;
        "number of mjtNums in jnt_user."; [ffi] nuser_geom : MjtSize;
        "number of mjtNums in geom_user."; [ffi] nuser_site : MjtSize;
        "number of mjtNums in site_user."; [ffi] nuser_cam : MjtSize;
        "number of mjtNums in cam_user."; [ffi] nuser_tendon : MjtSize;
        "number of mjtNums in tendon_user."; [ffi] nuser_actuator : MjtSize;
        "number of mjtNums in actuator_user."; [ffi] nuser_sensor : MjtSize;
        "number of mjtNums in sensor_user."; [ffi] nnames : MjtSize;
        "number of chars in all names."; [ffi] npaths : MjtSize;
        "number of chars in all paths."; [ffi] nnames_map : MjtSize;
        "number of slots in the names hash map."; [ffi] nJmom : MjtSize;
        "number of non-zeros in sparse actuator_moment matrix."; [ffi] ngravcomp :
        MjtSize; "number of bodies with nonzero gravcomp."; [ffi] nemax : MjtSize;
        "number of potential equality-constraint rows."; [ffi] njmax : MjtSize;
        "number of available rows in constraint Jacobian (legacy)."; [ffi] nconmax :
        MjtSize; "number of potential contacts in contact list (legacy)."; [ffi]
        npolygonmax : MjtSize; "maximum number of vertices in a mesh polygon."; [ffi]
        nmeshdegmax : MjtSize; "maximum number of edges adjacent to a mesh vertex.";
        [ffi] nuserdata : MjtSize; "number of mjtNums reserved for the user."; [ffi]
        nsensordata : MjtSize; "number of mjtNums in sensor data vector."; [ffi]
        npluginstate : MjtSize; "number of mjtNums in plugin state vector."; [ffi]
        nhistory : MjtSize; "number of mjtNums in history buffer."; [ffi] narena :
        MjtSize; "number of bytes in the mjData arena (inclusive of stack)."; [ffi]
        nbuffer : MjtSize; "number of bytes in buffer."; [ffi] flg_gravcomp : MjtBool;
        "whether any body has nonzero gravcomp."; [ffi] flg_surfacevel : MjtBool;
        "whether any geom has nonzero surfacevel."; [ffi] flg_adhesion : MjtBool;
        "whether any geom or pair has nonzero adhesion.";]
    }
    getter_setter! {
        get, [[ffi, ffi_mut] opt : & MjOption; "physics options."; [ffi, ffi_mut] vis : &
        MjVisual; "visualization options."; [ffi, ffi_mut] stat : & MjStatistic;
        "model statistics.";]
    }
}
/// Array slices.
impl MjModel {
    array_slice_dyn! {
        probe = probe_dynamic_arrays; qpos0 : & [MjtNum; "qpos values at default pose";
        ffi().nq], qpos_spring : & [MjtNum; "reference pose for springs"; ffi().nq], (mut
        = unsafe) body_parentid : & [i32; "id of body's parent"; ffi().nbody], (mut =
        unsafe) body_rootid : & [i32; "ancestor that is direct child of world"; ffi()
        .nbody], (mut = unsafe) body_weldid : & [i32;
        "top dof-less ancestor; mocap: own root"; ffi().nbody], (mut = unsafe)
        body_mocapid : & [i32; "id of mocap data; -1: none"; ffi().nbody], (mut = unsafe)
        body_jntnum : & [i32; "number of joints for this body"; ffi().nbody], (mut =
        unsafe) body_jntadr : & [i32; "start addr of joints; -1: no joints"; ffi()
        .nbody], (mut = unsafe) body_dofnum : & [i32;
        "number of motion degrees of freedom"; ffi().nbody], (mut = unsafe) body_dofadr :
        & [i32; "start addr of dofs; -1: no dofs"; ffi().nbody], (mut = unsafe)
        body_treeid : & [i32; "id of body's kinematic tree; -1: static"; ffi().nbody],
        (mut = unsafe) body_geomnum : & [i32; "number of geoms"; ffi().nbody], (mut =
        unsafe) body_geomadr : & [i32; "start addr of geoms; -1: no geoms"; ffi().nbody],
        body_simple : & [MjtByte; "1: diag M; 2: diag M, sliders only"; ffi().nbody],
        body_sameframe : & [MjtSameFrame[force]; "same frame as inertia"; ffi().nbody],
        body_pos : & [[MjtNum; 3] [force]; "position offset rel. to parent body"; ffi()
        .nbody], body_quat : & [[MjtNum; 4] [force];
        "orientation offset rel. to parent body"; ffi().nbody], body_ipos : & [[MjtNum;
        3] [force]; "local position of center of mass"; ffi().nbody], body_iquat : &
        [[MjtNum; 4] [force]; "local orientation of inertia ellipsoid"; ffi().nbody],
        body_mass : & [MjtNum; "mass"; ffi().nbody], body_subtreemass : & [MjtNum;
        "mass of subtree starting at this body"; ffi().nbody], body_inertia : & [[MjtNum;
        3] [force]; "diagonal inertia in ipos/iquat frame"; ffi().nbody], body_invweight0
        : & [[MjtNum; 2] [force]; "mean inv inert in qpos0 (trn, rot)"; ffi().nbody],
        body_gravcomp : & [MjtNum; "antigravity force, units of body weight"; ffi()
        .nbody], body_margin : & [MjtNum; "MAX over all geom margins+gaps"; ffi().nbody],
        (mut = unsafe) body_plugin : & [i32; "plugin instance id; -1: not in use"; ffi()
        .nbody], body_contype : & [i32; "OR over all geom contypes"; ffi().nbody],
        body_conaffinity : & [i32; "OR over all geom conaffinities"; ffi().nbody], (mut =
        unsafe) body_bvhadr : & [i32; "address of bvh root"; ffi().nbody], (mut = unsafe)
        body_bvhnum : & [i32; "number of bounding volumes"; ffi().nbody], bvh_depth : &
        [i32; "depth in the bounding volume hierarchy"; ffi().nbvh], (mut = unsafe)
        bvh_child : & [[i32; 2] [force]; "left and right children in tree"; ffi().nbvh],
        (mut = unsafe) bvh_nodeid : & [i32; "geom or elem id of node; -1: non-leaf";
        ffi().nbvh], bvh_aabb : & [[MjtNum; 6] [force];
        "local bounding box (center, size)"; ffi().nbvhstatic], oct_depth : & [i32;
        "depth in the octree"; ffi().noct], (mut = unsafe) oct_child : & [[i32; 8]
        [force]; "children of octree node"; ffi().noct], oct_aabb : & [[MjtNum; 6]
        [force]; "octree node bounding box (center, size)"; ffi().noct], oct_coeff : &
        [[MjtNum; 8] [force]; "octree interpolation coefficients"; ffi().noct], (mut =
        unsafe) jnt_type : & [MjtJoint[force]; "type of joint"; ffi().njnt], (mut =
        unsafe) jnt_qposadr : & [i32; "start addr in 'qpos' for joint's data"; ffi()
        .njnt], (mut = unsafe) jnt_dofadr : & [i32;
        "start addr in 'qvel' for joint's data"; ffi().njnt], (mut = unsafe) jnt_bodyid :
        & [i32; "id of joint's body"; ffi().njnt], (mut = unsafe) jnt_actuatorid : &
        [i32; "actuator contributing damping / armature"; ffi().njnt], jnt_group : &
        [i32; "group for visibility"; ffi().njnt], jnt_limited : & [MjtBool;
        "does joint have limits"; ffi().njnt], jnt_actfrclimited : & [MjtBool;
        "does joint have actuator force limits"; ffi().njnt], jnt_actgravcomp : &
        [MjtBool; "is gravcomp force applied via actuators"; ffi().njnt], jnt_solref : &
        [[MjtNum; mjNREF as usize] [force]; "constraint solver reference: limit"; ffi()
        .njnt], jnt_solimp : & [[MjtNum; mjNIMP as usize] [force];
        "constraint solver impedance: limit"; ffi().njnt], jnt_pos : & [[MjtNum; 3]
        [force]; "local anchor position"; ffi().njnt], jnt_axis : & [[MjtNum; 3] [force];
        "local joint axis"; ffi().njnt], jnt_stiffness : & [MjtNum;
        "linear stiffness coefficient"; ffi().njnt], jnt_stiffnesspoly : & [[MjtNum;
        mjNPOLY as usize] [force]; "high-order stiffness coefficients"; ffi().njnt],
        jnt_range : & [[MjtNum; 2] [force]; "joint limits"; ffi().njnt], jnt_actfrcrange
        : & [[MjtNum; 2] [force]; "range of total actuator force"; ffi().njnt],
        jnt_margin : & [MjtNum; "min distance for limit detection"; ffi().njnt], (mut =
        unsafe) dof_bodyid : & [i32; "id of dof's body"; ffi().nv], (mut = unsafe)
        dof_jntid : & [i32; "id of dof's joint"; ffi().nv], (mut = unsafe) dof_parentid :
        & [i32; "id of dof's parent; -1: none"; ffi().nv], (mut = unsafe) dof_treeid : &
        [i32; "id of dof's kinematic tree"; ffi().nv], (mut = unsafe) dof_Madr : & [i32;
        "dof address in M-diagonal"; ffi().nv], dof_simplenum : & [i32;
        "number of consecutive simple dofs"; ffi().nv], dof_solref : & [[MjtNum; mjNREF
        as usize] [force]; "constraint solver reference:frictionloss"; ffi().nv],
        dof_solimp : & [[MjtNum; mjNIMP as usize] [force];
        "constraint solver impedance:frictionloss"; ffi().nv], dof_frictionloss : &
        [MjtNum; "dof friction loss"; ffi().nv], dof_armature : & [MjtNum;
        "dof armature inertia/mass"; ffi().nv], dof_damping : & [MjtNum;
        "linear damping coefficient"; ffi().nv], dof_dampingpoly : & [[MjtNum; mjNPOLY as
        usize] [force]; "high-order damping coefficients"; ffi().nv], dof_invweight0 : &
        [MjtNum; "diag. inverse inertia in qpos0"; ffi().nv], dof_M0 : & [MjtNum;
        "diag. inertia in qpos0"; ffi().nv], dof_length : & [MjtNum;
        "linear: 1; angular: approx. length scale"; ffi().nv], (mut = unsafe)
        tree_bodyadr : & [i32; "start addr of bodies"; ffi().ntree], (mut = unsafe)
        tree_bodynum : & [i32; "number of bodies in tree"; ffi().ntree], (mut = unsafe)
        tree_dofadr : & [i32; "start addr of dofs"; ffi().ntree], (mut = unsafe)
        tree_dofnum : & [i32; "number of dofs in tree"; ffi().ntree], tree_sleep_policy :
        & [MjtSleepPolicy[force]; "sleep policy"; ffi().ntree], (mut = unsafe) geom_type
        : & [MjtGeom[force]; "geometric type"; ffi().ngeom], geom_contype : & [i32;
        "geom contact type"; ffi().ngeom], geom_conaffinity : & [i32;
        "geom contact affinity"; ffi().ngeom], (mut = unsafe) geom_condim : & [i32;
        "contact dimensionality (1, 3, 4, 6)"; ffi().ngeom], (mut = unsafe) geom_bodyid :
        & [i32; "id of geom's body"; ffi().ngeom], (mut = unsafe) geom_dataid : & [i32;
        "id of geom's mesh/hfield; -1: none"; ffi().ngeom], (mut = unsafe) geom_matid : &
        [i32; "material id for rendering; -1: none"; ffi().ngeom], geom_group : & [i32;
        "group for visibility"; ffi().ngeom], geom_priority : & [i32;
        "geom contact priority"; ffi().ngeom], (mut = unsafe) geom_plugin : & [i32;
        "plugin instance id; -1: not in use"; ffi().ngeom], geom_sameframe : &
        [MjtSameFrame[force]; "same frame as body"; ffi().ngeom], geom_solmix : &
        [MjtNum; "mixing coef for solref/imp in geom pair"; ffi().ngeom], geom_solref : &
        [[MjtNum; mjNREF as usize] [force]; "constraint solver reference: contact"; ffi()
        .ngeom], geom_solimp : & [[MjtNum; mjNIMP as usize] [force];
        "constraint solver impedance: contact"; ffi().ngeom], geom_size : & [[MjtNum; 3]
        [force]; "geom-specific size parameters"; ffi().ngeom], geom_aabb : & [[MjtNum;
        6] [force]; "bounding box, (center, size)"; ffi().ngeom], geom_rbound : &
        [MjtNum; "radius of bounding sphere"; ffi().ngeom], geom_pos : & [[MjtNum; 3]
        [force]; "local position offset rel. to body"; ffi().ngeom], geom_quat : &
        [[MjtNum; 4] [force]; "local orientation offset rel. to body"; ffi().ngeom],
        geom_friction : & [[MjtNum; 3] [force]; "friction for (slide, spin, roll)"; ffi()
        .ngeom], geom_margin : & [MjtNum; "geometric inflation for contact"; ffi()
        .ngeom], geom_gap : & [MjtNum; "additional contact detection buffer"; ffi()
        .ngeom], geom_surfacevel : & [[MjtNum; 6] [force];
        "surface velocity in local frame: lin,ang"; ffi().ngeom], geom_adhesion : &
        [MjtNum; "adhesive force of contacts"; ffi().ngeom], geom_fluid : & [[MjtNum;
        mjNFLUID as usize] [force]; "fluid interaction parameters"; ffi().ngeom],
        geom_rgba : & [[f32; 4] [force]; "rgba when material is omitted"; ffi().ngeom],
        site_type : & [MjtGeom[force]; "geom type for rendering"; ffi().nsite], (mut =
        unsafe) site_bodyid : & [i32; "id of site's body"; ffi().nsite], (mut = unsafe)
        site_matid : & [i32; "material id for rendering; -1: none"; ffi().nsite],
        site_group : & [i32; "group for visibility"; ffi().nsite], site_sameframe : &
        [MjtSameFrame[force]; "same frame as body"; ffi().nsite], site_size : & [[MjtNum;
        3] [force]; "geom size for rendering"; ffi().nsite], site_pos : & [[MjtNum; 3]
        [force]; "local position offset rel. to body"; ffi().nsite], site_quat : &
        [[MjtNum; 4] [force]; "local orientation offset rel. to body"; ffi().nsite],
        site_rgba : & [[f32; 4] [force]; "rgba when material is omitted"; ffi().nsite],
        cam_mode : & [MjtCamLight[force]; "camera tracking mode"; ffi().ncam], (mut =
        unsafe) cam_bodyid : & [i32; "id of camera's body"; ffi().ncam], (mut = unsafe)
        cam_targetbodyid : & [i32; "id of targeted body; -1: none"; ffi().ncam], cam_pos
        : & [[MjtNum; 3] [force]; "position rel. to body frame"; ffi().ncam], cam_quat :
        & [[MjtNum; 4] [force]; "orientation rel. to body frame"; ffi().ncam],
        cam_poscom0 : & [[MjtNum; 3] [force]; "global position rel. to sub-com in qpos0";
        ffi().ncam], cam_pos0 : & [[MjtNum; 3] [force];
        "global position rel. to body in qpos0"; ffi().ncam], cam_mat0 : & [[MjtNum; 9]
        [force]; "global orientation in qpos0"; ffi().ncam], cam_projection : &
        [MjtProjection[force]; "projection type"; ffi().ncam], cam_fovy : & [MjtNum;
        "y field-of-view (ortho ? len : deg)"; ffi().ncam], cam_ipd : & [MjtNum;
        "inter-pupillary distance"; ffi().ncam], (mut = unsafe) cam_resolution : & [[i32;
        2] [force]; "resolution: pixels [width, height]"; ffi().ncam], cam_output : &
        [i32; "output types (MjtCamOutBit bit flags)"; ffi().ncam], cam_sensorsize : &
        [[f32; 2] [force]; "sensor size: length [width, height]"; ffi().ncam],
        cam_intrinsic : & [[f32; 4] [force]; "[focal length; principal point]"; ffi()
        .ncam], light_mode : & [MjtCamLight[force]; "light tracking mode"; ffi().nlight],
        (mut = unsafe) light_bodyid : & [i32; "id of light's body"; ffi().nlight], (mut =
        unsafe) light_targetbodyid : & [i32; "id of targeted body; -1: none"; ffi()
        .nlight], light_type : & [MjtLightType[force]; "spot, directional, etc."; ffi()
        .nlight], (mut = unsafe) light_texid : & [i32; "texture id for image lights";
        ffi().nlight], light_castshadow : & [MjtBool; "does light cast shadows"; ffi()
        .nlight], light_bulbradius : & [f32; "light radius for soft shadows"; ffi()
        .nlight], light_intensity : & [f32; "intensity, in candela"; ffi().nlight],
        light_range : & [f32; "range of effectiveness"; ffi().nlight], light_active : &
        [MjtBool; "is light on"; ffi().nlight], light_pos : & [[MjtNum; 3] [force];
        "position rel. to body frame"; ffi().nlight], light_dir : & [[MjtNum; 3] [force];
        "direction rel. to body frame"; ffi().nlight], light_poscom0 : & [[MjtNum; 3]
        [force]; "global position rel. to sub-com in qpos0"; ffi().nlight], light_pos0 :
        & [[MjtNum; 3] [force]; "global position rel. to body in qpos0"; ffi().nlight],
        light_dir0 : & [[MjtNum; 3] [force]; "global direction in qpos0"; ffi().nlight],
        light_attenuation : & [[f32; 3] [force]; "OpenGL attenuation (quadratic model)";
        ffi().nlight], light_cutoff : & [f32; "OpenGL cutoff"; ffi().nlight],
        light_softness : & [f32; "spotlight edge softness"; ffi().nlight], light_exponent
        : & [f32; "OpenGL exponent"; ffi().nlight], light_ambient : & [[f32; 3] [force];
        "ambient rgb (alpha=1)"; ffi().nlight], light_diffuse : & [[f32; 3] [force];
        "diffuse rgb (alpha=1)"; ffi().nlight], light_specular : & [[f32; 3] [force];
        "specular rgb (alpha=1)"; ffi().nlight], flex_contype : & [i32;
        "flex contact type"; ffi().nflex], flex_conaffinity : & [i32;
        "flex contact affinity"; ffi().nflex], (mut = unsafe) flex_condim : & [i32;
        "contact dimensionality (1, 3, 4, 6)"; ffi().nflex], flex_priority : & [i32;
        "flex contact priority"; ffi().nflex], flex_solmix : & [MjtNum;
        "mix coef for solref/imp in contact pair"; ffi().nflex], flex_solref : &
        [[MjtNum; mjNREF as usize] [force]; "constraint solver reference: contact"; ffi()
        .nflex], flex_solimp : & [[MjtNum; mjNIMP as usize] [force];
        "constraint solver impedance: contact"; ffi().nflex], flex_friction : & [[MjtNum;
        3] [force]; "friction for (slide, spin, roll)"; ffi().nflex], flex_margin : &
        [MjtNum; "geometric inflation for contact"; ffi().nflex], flex_gap : & [MjtNum;
        "additional contact detection buffer"; ffi().nflex], flex_internal : & [MjtBool;
        "internal flex collision enabled"; ffi().nflex], flex_selfcollide : &
        [MjtFlexSelf[force]; "self collision mode"; ffi().nflex], flex_activelayers : &
        [i32; "number of active element layers, 3D only"; ffi().nflex], flex_passive : &
        [i32; "passive collisions enabled"; ffi().nflex], (mut = unsafe) flex_dim : &
        [i32; "1: lines, 2: triangles, 3: tetrahedra"; ffi().nflex], (mut = unsafe)
        flex_matid : & [i32; "material id for rendering"; ffi().nflex], flex_group : &
        [i32; "group for visibility"; ffi().nflex], (mut = unsafe) flex_interp : & [i32;
        "interpolation (0: vertex, 1: nodes)"; ffi().nflex], (mut = unsafe) flex_cellnum
        : & [[i32; 3] [force]; "finite cell num per dimension"; ffi().nflex], (mut =
        unsafe) flex_nodeadr : & [i32; "first node address"; ffi().nflex], (mut = unsafe)
        flex_nodenum : & [i32; "number of nodes"; ffi().nflex], (mut = unsafe)
        flex_vertadr : & [i32; "first vertex address"; ffi().nflex], (mut = unsafe)
        flex_vertnum : & [i32; "number of vertices"; ffi().nflex], (mut = unsafe)
        flex_edgeadr : & [i32; "first edge address"; ffi().nflex], (mut = unsafe)
        flex_edgenum : & [i32; "number of edges"; ffi().nflex], (mut = unsafe)
        flex_elemadr : & [i32; "first element address"; ffi().nflex], (mut = unsafe)
        flex_elemnum : & [i32; "number of elements"; ffi().nflex], (mut = unsafe)
        flex_elemdataadr : & [i32; "first element vertex id address"; ffi().nflex], (mut
        = unsafe) flex_stiffnessadr : & [i32; "stiffness matrix address"; ffi().nflex],
        (mut = unsafe) flex_elemedgeadr : & [i32; "first element edge id address"; ffi()
        .nflex], (mut = unsafe) flex_bendingadr : & [i32; "first bending data address";
        ffi().nflex], (mut = unsafe) flex_shellnum : & [i32; "number of shells"; ffi()
        .nflex], (mut = unsafe) flex_shelldataadr : & [i32; "first shell data address";
        ffi().nflex], (mut = unsafe) flex_evpairadr : & [i32; "first evpair address";
        ffi().nflex], (mut = unsafe) flex_evpairnum : & [i32; "number of evpairs"; ffi()
        .nflex], (mut = unsafe) flex_texcoordadr : & [i32;
        "address in flex_texcoord; -1: none"; ffi().nflex], (mut = unsafe)
        flex_nodebodyid : & [i32; "node body ids"; ffi().nflexnode], (mut = unsafe)
        flex_vertbodyid : & [i32; "vertex body ids"; ffi().nflexvert], (mut = unsafe)
        flex_vertedgeadr : & [i32; "first edge address"; ffi().nflexvert], (mut = unsafe)
        flex_vertedgenum : & [i32; "number of edges"; ffi().nflexvert], (mut = unsafe)
        flex_vertedge : & [[i32; 2] [force]; "edge indices"; ffi().nflexedge], (mut =
        unsafe) flex_edge : & [[i32; 2] [force]; "edge vertex ids (2 per edge)"; ffi()
        .nflexedge], (mut = unsafe) flex_edgeflap : & [[i32; 2] [force];
        "adjacent vertex ids (dim=2 only)"; ffi().nflexedge], (mut = unsafe) flex_elem :
        & [i32; "element vertex ids (dim+1 per elem)"; ffi().nflexelemdata], (mut =
        unsafe) flex_elemtexcoord : & [i32; "element texture coordinates (dim+1)"; ffi()
        .nflexelemdata], (mut = unsafe) flex_elemedge : & [i32; "element edge ids"; ffi()
        .nflexelemedge], (mut = unsafe) flex_elemlayer : & [i32;
        "element distance from surface, 3D only"; ffi().nflexelem], (mut = unsafe)
        flex_shell : & [i32; "shell fragment vertex ids (dim per frag)"; ffi()
        .nflexshelldata], (mut = unsafe) flex_evpair : & [[i32; 2] [force];
        "(element, vertex) collision pairs"; ffi().nflexevpair], flex_vert : & [[MjtNum;
        3] [force]; "vertex positions in local body frames"; ffi().nflexvert], flex_vert0
        : & [[MjtNum; 3] [force]; "vertex positions in qpos0 on [0, 1]^d"; ffi()
        .nflexvert], flex_vertmetric : & [[MjtNum; 4] [force];
        "inverse of reference shape matrix"; ffi().nflexvert], flex_node : & [[MjtNum; 3]
        [force]; "node positions in local body frames"; ffi().nflexnode], flex_node0 : &
        [[MjtNum; 3] [force]; "Cartesian node positions in qpos0"; ffi().nflexnode],
        flexedge_length0 : & [MjtNum; "edge lengths in qpos0"; ffi().nflexedge],
        flexedge_invweight0 : & [MjtNum; "edge inv. weight in qpos0"; ffi().nflexedge],
        flex_radius : & [MjtNum; "radius around primitive element"; ffi().nflex],
        flex_size : & [[MjtNum; 3] [force]; "vertex bounding box half sizes in qpos0";
        ffi().nflex], flex_stiffness : & [MjtNum; "finite element stiffness matrix";
        ffi().nflexstiffness], flex_bending : & [MjtNum; "bending stiffness"; ffi()
        .nflexbending], (mut = unsafe) efm0_dofid : & [i32;
        "constant metric factor row->dof address"; ffi().nefm0dof], (mut = unsafe)
        efm0_L_rownnz : & [i32; "constant metric factor row nonzeros"; ffi().nefm0dof],
        (mut = unsafe) efm0_L_rowadr : & [i32; "constant metric factor row addresses";
        ffi().nefm0dof], (mut = unsafe) efm0_L_colind : & [i32;
        "constant metric factor column indices"; ffi().nefm0L], efm0_L : & [MjtNum;
        "factor of M + (dt^2+dt*d)*K_bend"; ffi().nefm0L], flex_damping : & [MjtNum;
        "Rayleigh's damping coefficient"; ffi().nflex], flex_edgestiffness : & [MjtNum;
        "edge stiffness"; ffi().nflex], flex_edgedamping : & [MjtNum; "edge damping";
        ffi().nflex], flex_edgeequality : & [i32;
        "0: none, 1: edges, 2: vertices, 3: strain"; ffi().nflex], flex_rigid : &
        [MjtBool; "are all vertices in the same body"; ffi().nflex], flexedge_rigid : &
        [MjtBool; "are both edge vertices in same body"; ffi().nflexedge], flex_centered
        : & [MjtBool; "are all vertex coordinates (0,0,0)"; ffi().nflex], flex_flatskin :
        & [MjtBool; "render flex skin with flat shading"; ffi().nflex], (mut = unsafe)
        flex_bvhadr : & [i32; "address of bvh root; -1: no bvh"; ffi().nflex], (mut =
        unsafe) flex_bvhnum : & [i32; "number of bounding volumes"; ffi().nflex], (mut =
        unsafe) flexedge_J_rownnz : & [i32; "number of non-zeros in Jacobian row"; ffi()
        .nflexedge], (mut = unsafe) flexedge_J_rowadr : & [i32;
        "row start address in colind array"; ffi().nflexedge], (mut = unsafe)
        flexedge_J_colind : & [i32; "column indices in sparse Jacobian"; ffi().nJfe],
        (mut = unsafe) flexvert_J_rownnz : & [[i32; 2] [force];
        "number of non-zeros in Jacobian row"; ffi().nflexvert], (mut = unsafe)
        flexvert_J_rowadr : & [[i32; 2] [force]; "row start address in colind array";
        ffi().nflexvert], (mut = unsafe) flexvert_J_colind : & [[i32; 2] [force];
        "column indices in sparse Jacobian"; ffi().nJfv], flex_rgba : & [[f32; 4]
        [force]; "rgba when material is omitted"; ffi().nflex], flex_texcoord : & [[f32;
        2] [force]; "vertex texture coordinates"; ffi().nflextexcoord], (mut = unsafe)
        mesh_vertadr : & [i32; "first vertex address"; ffi().nmesh], (mut = unsafe)
        mesh_vertnum : & [i32; "number of vertices"; ffi().nmesh], (mut = unsafe)
        mesh_faceadr : & [i32; "first face address"; ffi().nmesh], (mut = unsafe)
        mesh_facenum : & [i32; "number of faces"; ffi().nmesh], (mut = unsafe)
        mesh_bvhadr : & [i32; "address of bvh root"; ffi().nmesh], (mut = unsafe)
        mesh_bvhnum : & [i32; "number of bvh"; ffi().nmesh], (mut = unsafe) mesh_octadr :
        & [i32; "address of octree root"; ffi().nmesh], (mut = unsafe) mesh_octnum : &
        [i32; "number of octree nodes"; ffi().nmesh], (mut = unsafe) mesh_normaladr : &
        [i32; "first normal address"; ffi().nmesh], (mut = unsafe) mesh_normalnum : &
        [i32; "number of normals"; ffi().nmesh], (mut = unsafe) mesh_texcoordadr : &
        [i32; "texcoord data address; -1: no texcoord"; ffi().nmesh], (mut = unsafe)
        mesh_texcoordnum : & [i32; "number of texcoord"; ffi().nmesh], (mut = unsafe)
        mesh_graphadr : & [i32; "graph data address; -1: no graph"; ffi().nmesh], (mut =
        unsafe) mesh_extrema : & [[i32; 27] [force];
        "extremum vertices in 3x3x3 directions"; ffi().nmesh], mesh_vert : & [[f32; 3]
        [force]; "vertex positions for all meshes"; ffi().nmeshvert], mesh_normal : &
        [[f32; 3] [force]; "normals for all meshes"; ffi().nmeshnormal], mesh_texcoord :
        & [[f32; 2] [force]; "vertex texcoords for all meshes"; ffi().nmeshtexcoord],
        (mut = unsafe) mesh_face : & [[i32; 3] [force]; "vertex face data"; ffi()
        .nmeshface], (mut = unsafe) mesh_facenormal : & [[i32; 3] [force];
        "normal face data"; ffi().nmeshface], (mut = unsafe) mesh_facetexcoord : & [[i32;
        3] [force]; "texture face data"; ffi().nmeshface], (mut = unsafe) mesh_graph : &
        [i32; "convex graph data"; ffi().nmeshgraph], mesh_scale : & [[MjtNum; 3]
        [force]; "scaling applied to asset vertices"; ffi().nmesh], mesh_pos : &
        [[MjtNum; 3] [force]; "translation applied to asset vertices"; ffi().nmesh],
        mesh_quat : & [[MjtNum; 4] [force]; "rotation applied to asset vertices"; ffi()
        .nmesh], (mut = unsafe) mesh_pathadr : & [i32;
        "address of asset path for mesh; -1: none"; ffi().nmesh], (mut = unsafe)
        mesh_polynum : & [i32; "number of polygons per mesh"; ffi().nmesh], (mut =
        unsafe) mesh_polyadr : & [i32; "first polygon address per mesh"; ffi().nmesh],
        mesh_polynormal : & [[MjtNum; 3] [force]; "all polygon normals"; ffi()
        .nmeshpoly], (mut = unsafe) mesh_polyvertadr : & [i32;
        "polygon vertex start address"; ffi().nmeshpoly], (mut = unsafe) mesh_polyvertnum
        : & [i32; "number of vertices per polygon"; ffi().nmeshpoly], (mut = unsafe)
        mesh_polyvert : & [i32; "all polygon vertices"; ffi().nmeshpolyvert], (mut =
        unsafe) mesh_polymapadr : & [i32; "first polygon address per vertex"; ffi()
        .nmeshvert], (mut = unsafe) mesh_polymapnum : & [i32;
        "number of polygons per vertex"; ffi().nmeshvert], (mut = unsafe) mesh_polymap :
        & [i32; "vertex to polygon map"; ffi().nmeshpolymap], (mut = unsafe) skin_matid :
        & [i32; "skin material id; -1: none"; ffi().nskin], skin_group : & [i32;
        "group for visibility"; ffi().nskin], skin_rgba : & [[f32; 4] [force];
        "skin rgba"; ffi().nskin], skin_inflate : & [f32;
        "inflate skin in normal direction"; ffi().nskin], (mut = unsafe) skin_vertadr : &
        [i32; "first vertex address"; ffi().nskin], (mut = unsafe) skin_vertnum : & [i32;
        "number of vertices"; ffi().nskin], (mut = unsafe) skin_texcoordadr : & [i32;
        "texcoord data address; -1: no texcoord"; ffi().nskin], (mut = unsafe)
        skin_faceadr : & [i32; "first face address"; ffi().nskin], (mut = unsafe)
        skin_facenum : & [i32; "number of faces"; ffi().nskin], (mut = unsafe)
        skin_boneadr : & [i32; "first bone in skin"; ffi().nskin], (mut = unsafe)
        skin_bonenum : & [i32; "number of bones in skin"; ffi().nskin], skin_vert : &
        [[f32; 3] [force]; "vertex positions for all skin meshes"; ffi().nskinvert],
        skin_texcoord : & [[f32; 2] [force]; "vertex texcoords for all skin meshes";
        ffi().nskintexvert], (mut = unsafe) skin_face : & [[i32; 3] [force];
        "triangle faces for all skin meshes"; ffi().nskinface], (mut = unsafe)
        skin_bonevertadr : & [i32; "first vertex in each bone"; ffi().nskinbone], (mut =
        unsafe) skin_bonevertnum : & [i32; "number of vertices in each bone"; ffi()
        .nskinbone], skin_bonebindpos : & [[f32; 3] [force]; "bind pos of each bone";
        ffi().nskinbone], skin_bonebindquat : & [[f32; 4] [force];
        "bind quat of each bone"; ffi().nskinbone], (mut = unsafe) skin_bonebodyid : &
        [i32; "body id of each bone"; ffi().nskinbone], (mut = unsafe) skin_bonevertid :
        & [i32; "mesh ids of vertices in each bone"; ffi().nskinbonevert],
        skin_bonevertweight : & [f32; "weights of vertices in each bone"; ffi()
        .nskinbonevert], (mut = unsafe) skin_pathadr : & [i32;
        "address of asset path for skin; -1: none"; ffi().nskin], hfield_size : &
        [[MjtNum; 4] [force]; "(x, y, z_top, z_bottom)"; ffi().nhfield], (mut = unsafe)
        hfield_nrow : & [i32; "number of rows in grid"; ffi().nhfield], (mut = unsafe)
        hfield_ncol : & [i32; "number of columns in grid"; ffi().nhfield], (mut = unsafe)
        hfield_adr : & [i32; "address in hfield_data"; ffi().nhfield], hfield_data : &
        [f32; "elevation data"; ffi().nhfielddata], (mut = unsafe) hfield_pathadr : &
        [i32; "address of hfield asset path; -1: none"; ffi().nhfield], (mut = unsafe)
        tex_type : & [MjtTexture[force]; "texture type"; ffi().ntex], tex_colorspace : &
        [MjtColorSpace[force]; "texture colorspace"; ffi().ntex], (mut = unsafe)
        tex_height : & [i32; "number of rows in texture image"; ffi().ntex], (mut =
        unsafe) tex_width : & [i32; "number of columns in texture image"; ffi().ntex],
        (mut = unsafe) tex_nchannel : & [i32; "number of channels in texture image";
        ffi().ntex], (mut = unsafe) tex_adr : & [MjtSize; "start address in tex_data";
        ffi().ntex], tex_data : & [MjtByte; "pixel values"; ffi().ntexdata], (mut =
        unsafe) tex_pathadr : & [i32; "address of texture asset path; -1: none"; ffi()
        .ntex], (mut = unsafe) mat_texid : & [[i32; MjtTextureRole::mjNTEXROLE as usize]
        [force]; "indices of textures; -1: none"; ffi().nmat], mat_texuniform : &
        [MjtBool; "make texture cube uniform"; ffi().nmat], mat_texrepeat : & [[f32; 2]
        [force]; "texture repetition for 2d mapping"; ffi().nmat], mat_emission : & [f32;
        "emission (x rgb)"; ffi().nmat], mat_specular : & [f32; "specular (x white)";
        ffi().nmat], mat_shininess : & [f32; "shininess coef"; ffi().nmat],
        mat_reflectance : & [f32; "reflectance (0: disable)"; ffi().nmat], mat_metallic :
        & [f32; "metallic coef"; ffi().nmat], mat_roughness : & [f32; "roughness coef";
        ffi().nmat], mat_rgba : & [[f32; 4] [force]; "rgba"; ffi().nmat], (mut = unsafe)
        pair_dim : & [i32; "contact dimensionality"; ffi().npair], (mut = unsafe)
        pair_geom1 : & [i32; "id of geom1"; ffi().npair], (mut = unsafe) pair_geom2 : &
        [i32; "id of geom2"; ffi().npair], pair_signature : & [i32;
        "body1 << 16 + body2"; ffi().npair], pair_solref : & [[MjtNum; mjNREF as usize]
        [force]; "solver reference: contact normal"; ffi().npair], pair_solreffriction :
        & [[MjtNum; mjNREF as usize] [force]; "solver reference: contact friction"; ffi()
        .npair], pair_solimp : & [[MjtNum; mjNIMP as usize] [force];
        "solver impedance: contact"; ffi().npair], pair_margin : & [MjtNum;
        "geometric inflation for contact"; ffi().npair], pair_gap : & [MjtNum;
        "additional contact detection buffer"; ffi().npair], pair_adhesion : & [MjtNum;
        "adhesive force of contacts"; ffi().npair], pair_friction : & [[MjtNum; 5]
        [force]; "tangent1, 2, spin, roll1, 2"; ffi().npair], exclude_signature : & [i32;
        "body1 << 16 + body2"; ffi().nexclude], (mut = unsafe) eq_type : & [MjtEq[force];
        "constraint type"; ffi().neq], (mut = unsafe) eq_obj1id : & [i32;
        "id of object 1"; ffi().neq], (mut = unsafe) eq_obj2id : & [i32;
        "id of object 2"; ffi().neq], (mut = unsafe) eq_objtype : & [MjtObj[force];
        "type of both objects"; ffi().neq], eq_active0 : & [MjtBool;
        "initial enable/disable constraint state"; ffi().neq], eq_solref : & [[MjtNum;
        mjNREF as usize] [force]; "constraint solver reference"; ffi().neq], eq_solimp :
        & [[MjtNum; mjNIMP as usize] [force]; "constraint solver impedance"; ffi().neq],
        eq_data : & [[MjtNum; mjNEQDATA as usize] [force]; "numeric data for constraint";
        ffi().neq], (mut = unsafe) tendon_adr : & [i32;
        "address of first object in tendon's path"; ffi().ntendon], (mut = unsafe)
        tendon_num : & [i32; "number of objects in tendon's path"; ffi().ntendon], (mut =
        unsafe) tendon_matid : & [i32; "material id for rendering"; ffi().ntendon], (mut
        = unsafe) tendon_actuatorid : & [i32; "actuator contributing damping / armature";
        ffi().ntendon], tendon_group : & [i32; "group for visibility"; ffi().ntendon],
        tendon_treenum : & [i32; "number of trees along tendon's path"; ffi().ntendon],
        (mut = unsafe) tendon_treeid : & [[i32; 2] [force];
        "first two trees along tendon's path"; ffi().ntendon], (mut = unsafe)
        ten_J_rownnz : & [i32; "number of non-zeros in Jacobian row"; ffi().ntendon],
        (mut = unsafe) ten_J_rowadr : & [i32; "row start address in colind array"; ffi()
        .ntendon], (mut = unsafe) ten_J_colind : & [i32;
        "column indices in sparse Jacobian"; ffi().nJten], tendon_limited : & [MjtBool;
        "does tendon have length limits"; ffi().ntendon], tendon_actfrclimited : &
        [MjtBool; "does tendon have actuator force limits"; ffi().ntendon], tendon_width
        : & [MjtNum; "width for rendering"; ffi().ntendon], tendon_solref_lim : &
        [[MjtNum; mjNREF as usize] [force]; "constraint solver reference: limit"; ffi()
        .ntendon], tendon_solimp_lim : & [[MjtNum; mjNIMP as usize] [force];
        "constraint solver impedance: limit"; ffi().ntendon], tendon_solref_fri : &
        [[MjtNum; mjNREF as usize] [force]; "constraint solver reference: friction";
        ffi().ntendon], tendon_solimp_fri : & [[MjtNum; mjNIMP as usize] [force];
        "constraint solver impedance: friction"; ffi().ntendon], tendon_range : &
        [[MjtNum; 2] [force]; "tendon length limits"; ffi().ntendon], tendon_actfrcrange
        : & [[MjtNum; 2] [force]; "range of total actuator force"; ffi().ntendon],
        tendon_margin : & [MjtNum; "min distance for limit detection"; ffi().ntendon],
        tendon_stiffness : & [MjtNum; "linear stiffness coefficient"; ffi().ntendon],
        tendon_stiffnesspoly : & [[MjtNum; mjNPOLY as usize] [force];
        "high-order stiffness coefficients"; ffi().ntendon], tendon_damping : & [MjtNum;
        "linear damping coefficient"; ffi().ntendon], tendon_dampingpoly : & [[MjtNum;
        mjNPOLY as usize] [force]; "high-order damping coefficients"; ffi().ntendon],
        tendon_armature : & [MjtNum; "inertia associated with tendon velocity"; ffi()
        .ntendon], tendon_frictionloss : & [MjtNum; "loss due to friction"; ffi()
        .ntendon], tendon_lengthspring : & [[MjtNum; 2] [force];
        "spring resting length range"; ffi().ntendon], tendon_length0 : & [MjtNum;
        "tendon length in qpos0"; ffi().ntendon], tendon_invweight0 : & [MjtNum;
        "inv. weight in qpos0"; ffi().ntendon], tendon_rgba : & [[f32; 4] [force];
        "rgba when material is omitted"; ffi().ntendon], (mut = unsafe) wrap_type : &
        [MjtWrap[force]; "wrap object type"; ffi().nwrap], (mut = unsafe) wrap_objid : &
        [i32; "object id: geom, site, joint"; ffi().nwrap], (mut = unsafe) wrap_prm : &
        [MjtNum; "divisor, joint coef, or site id"; ffi().nwrap], (mut = unsafe)
        actuator_trntype : & [MjtTrn[force]; "transmission type"; ffi().nactuator], (mut
        = unsafe) actuator_dyntype : & [MjtDyn[force]; "dynamics type"; ffi().nactuator],
        (mut = unsafe) actuator_gaintype : & [MjtGain[force]; "gain type"; ffi()
        .nactuator], actuator_biastype : & [MjtBias[force]; "bias type"; ffi()
        .nactuator], (mut = unsafe) actuator_ctrladr : & [i32;
        "address of first control"; ffi().nactuator], (mut = unsafe) actuator_ctrlnum : &
        [i32; "number of controls"; ffi().nactuator], (mut = unsafe) actuator_ctrlspec :
        & [i32; "input signature, scoped by gaintype"; ffi().nactuator], (mut = unsafe)
        actuator_outadr : & [i32; "address of first force output"; ffi().nactuator], (mut
        = unsafe) actuator_outnum : & [i32; "number of force outputs, from trntype";
        ffi().nactuator], (mut = unsafe) actuator_trnid : & [[i32; 2] [force];
        "transmission id: joint, tendon, site"; ffi().nactuator], (mut = unsafe)
        actuator_actadr : & [i32; "first activation address; -1: stateless"; ffi()
        .nactuator], (mut = unsafe) actuator_actnum : & [i32;
        "number of activation variables"; ffi().nactuator], actuator_group : & [i32;
        "group for visibility"; ffi().nactuator], (mut = unsafe) actuator_history : &
        [[i32; 2] [force]; "history buffer: [nsample, interp]"; ffi().nactuator], (mut =
        unsafe) actuator_historyadr : & [i32; "address in history buffer; -1: none";
        ffi().nactuator], actuator_delay : & [MjtNum;
        "delay time in seconds; 0: no delay"; ffi().nactuator], actuator_ctrllimited : &
        [MjtBool; "is control limited"; ffi().nu], actuator_forcelimited : & [MjtBool;
        "is force limited"; ffi().nactuator], actuator_actlimited : & [MjtBool;
        "is activation limited"; ffi().nactuator], actuator_dynprm : & [[MjtNum; mjNDYN
        as usize] [force]; "dynamics parameters"; ffi().nactuator], actuator_gainprm : &
        [[MjtNum; mjNGAIN as usize] [force]; "gain parameters"; ffi().nactuator],
        actuator_biasprm : & [[MjtNum; mjNBIAS as usize] [force]; "bias parameters";
        ffi().nactuator], actuator_actearly : & [MjtBool; "step activation before force";
        ffi().nactuator], actuator_ctrlrange : & [[MjtNum; 2] [force];
        "range of controls"; ffi().nu], actuator_forcerange : & [[MjtNum; 2] [force];
        "range of forces"; ffi().nactuator], actuator_actrange : & [[MjtNum; 2] [force];
        "range of activations"; ffi().nactuator], actuator_damping : & [MjtNum;
        "linear damping coefficient"; ffi().nactuator], actuator_dampingpoly : &
        [[MjtNum; mjNPOLY as usize] [force]; "high-order damping coefficients"; ffi()
        .nactuator], actuator_armature : & [MjtNum;
        "armature added to target (joint, tendon)"; ffi().nactuator], actuator_gear : &
        [[MjtNum; 6] [force]; "scale length and transmitted force"; ffi().nout],
        actuator_cranklength : & [MjtNum; "crank length for slider-crank"; ffi()
        .nactuator], actuator_acc0 : & [MjtNum; "acceleration from unit force in qpos0";
        ffi().nout], actuator_length0 : & [MjtNum; "actuator length in qpos0"; ffi()
        .nout], actuator_lengthrange : & [[MjtNum; 2] [force];
        "feasible actuator length range"; ffi().nout], (mut = unsafe) actuator_plugin : &
        [i32; "plugin instance id; -1: not a plugin"; ffi().nactuator], (mut = unsafe)
        sensor_type : & [MjtSensor[force]; "sensor type"; ffi().nsensor], sensor_datatype
        : & [MjtDataType[force]; "numeric data type"; ffi().nsensor], sensor_needstage :
        & [MjtStage[force]; "required compute stage"; ffi().nsensor], (mut = unsafe)
        sensor_objtype : & [MjtObj[force]; "type of sensorized object"; ffi().nsensor],
        (mut = unsafe) sensor_objid : & [i32; "id of sensorized object"; ffi().nsensor],
        (mut = unsafe) sensor_reftype : & [MjtObj[force]; "type of reference frame";
        ffi().nsensor], (mut = unsafe) sensor_refid : & [i32;
        "id of reference frame; -1: global frame"; ffi().nsensor], (mut = unsafe)
        sensor_intprm : & [[i32; mjNSENS as usize] [force]; "sensor parameters"; ffi()
        .nsensor], (mut = unsafe) sensor_dim : & [i32; "number of scalar outputs"; ffi()
        .nsensor], (mut = unsafe) sensor_adr : & [i32; "address in sensor array"; ffi()
        .nsensor], sensor_cutoff : & [MjtNum; "cutoff for real and positive; 0: ignore";
        ffi().nsensor], sensor_noise : & [MjtNum; "noise standard deviation"; ffi()
        .nsensor], (mut = unsafe) sensor_history : & [[i32; 2] [force];
        "history buffer: [nsample, interp]"; ffi().nsensor], (mut = unsafe)
        sensor_historyadr : & [i32; "address in history buffer; -1: none"; ffi()
        .nsensor], sensor_delay : & [MjtNum; "delay time in seconds; 0: no delay"; ffi()
        .nsensor], sensor_interval : & [[MjtNum; 2] [force];
        "interval: [period, phase] in seconds"; ffi().nsensor], (mut = unsafe)
        sensor_plugin : & [i32; "plugin instance id; -1: not a plugin"; ffi().nsensor],
        (mut = unsafe) plugin : & [i32; "globally registered plugin slot number"; ffi()
        .nplugin], (mut = unsafe) plugin_stateadr : & [i32;
        "address in the plugin state array"; ffi().nplugin], (mut = unsafe)
        plugin_statenum : & [i32; "number of states in the plugin instance"; ffi()
        .nplugin], (mut = unsafe) plugin_attr : & [c_char;
        "config attributes of plugin instances"; ffi().npluginattr], (mut = unsafe)
        plugin_attradr : & [i32; "address to each instance's config attrib"; ffi()
        .nplugin], (mut = unsafe) numeric_adr : & [i32;
        "address of field in numeric_data"; ffi().nnumeric], (mut = unsafe) numeric_size
        : & [i32; "size of numeric field"; ffi().nnumeric], numeric_data : & [MjtNum;
        "array of all numeric fields"; ffi().nnumericdata], (mut = unsafe) text_adr : &
        [i32; "address of text in text_data"; ffi().ntext], (mut = unsafe) text_size : &
        [i32; "size of text field (strlen+1)"; ffi().ntext], (mut = unsafe) text_data : &
        [c_char; "array of all text fields (0-terminated)"; ffi().ntextdata], (mut =
        unsafe) tuple_adr : & [i32; "address of tuple in tuple_objtype/objid/objprm";
        ffi().ntuple], (mut = unsafe) tuple_size : & [i32; "number of objects in tuple";
        ffi().ntuple], tuple_objtype : & [MjtObj[force];
        "array of object types in all tuples"; ffi().ntupledata], (mut = unsafe)
        tuple_objid : & [i32; "array of object ids in all tuples"; ffi().ntupledata],
        tuple_objprm : & [MjtNum; "array of object params in all tuples"; ffi()
        .ntupledata], key_time : & [MjtNum; "key time"; ffi().nkey], (mut = unsafe)
        name_bodyadr : & [i32; "body name pointers"; ffi().nbody], (mut = unsafe)
        name_jntadr : & [i32; "joint name pointers"; ffi().njnt], (mut = unsafe)
        name_geomadr : & [i32; "geom name pointers"; ffi().ngeom], (mut = unsafe)
        name_siteadr : & [i32; "site name pointers"; ffi().nsite], (mut = unsafe)
        name_camadr : & [i32; "camera name pointers"; ffi().ncam], (mut = unsafe)
        name_lightadr : & [i32; "light name pointers"; ffi().nlight], (mut = unsafe)
        name_flexadr : & [i32; "flex name pointers"; ffi().nflex], (mut = unsafe)
        name_meshadr : & [i32; "mesh name pointers"; ffi().nmesh], (mut = unsafe)
        name_skinadr : & [i32; "skin name pointers"; ffi().nskin], (mut = unsafe)
        name_hfieldadr : & [i32; "hfield name pointers"; ffi().nhfield], (mut = unsafe)
        name_texadr : & [i32; "texture name pointers"; ffi().ntex], (mut = unsafe)
        name_matadr : & [i32; "material name pointers"; ffi().nmat], (mut = unsafe)
        name_pairadr : & [i32; "geom pair name pointers"; ffi().npair], (mut = unsafe)
        name_excludeadr : & [i32; "exclude name pointers"; ffi().nexclude], (mut =
        unsafe) name_eqadr : & [i32; "equality constraint name pointers"; ffi().neq],
        (mut = unsafe) name_tendonadr : & [i32; "tendon name pointers"; ffi().ntendon],
        (mut = unsafe) name_actuatoradr : & [i32; "actuator name pointers"; ffi()
        .nactuator], (mut = unsafe) name_sensoradr : & [i32; "sensor name pointers";
        ffi().nsensor], (mut = unsafe) name_numericadr : & [i32; "numeric name pointers";
        ffi().nnumeric], (mut = unsafe) name_textadr : & [i32; "text name pointers";
        ffi().ntext], (mut = unsafe) name_tupleadr : & [i32; "tuple name pointers"; ffi()
        .ntuple], (mut = unsafe) name_keyadr : & [i32; "keyframe name pointers"; ffi()
        .nkey], (mut = unsafe) name_pluginadr : & [i32; "plugin instance name pointers";
        ffi().nplugin], (mut = unsafe) names : & [c_char;
        "names of all objects, 0-terminated"; ffi().nnames], (mut = unsafe) names_map : &
        [i32; "internal hash map of names"; ffi().nnames_map], paths : & [c_char;
        "paths to assets, 0-terminated"; ffi().npaths], (mut = unsafe) B_rownnz : & [i32;
        "body-dof: non-zeros in each row"; ffi().nbody], (mut = unsafe) B_rowadr : &
        [i32; "body-dof: row addresses"; ffi().nbody], (mut = unsafe) B_colind : & [i32;
        "body-dof: column indices"; ffi().nB], (mut = unsafe) M_rownnz : & [i32;
        "reduced inertia: non-zeros in each row"; ffi().nv], (mut = unsafe) M_rowadr : &
        [i32; "reduced inertia: row addresses"; ffi().nv], (mut = unsafe) M_colind : &
        [i32; "reduced inertia: column indices"; ffi().nC], (mut = unsafe) mapM2M : &
        [i32; "index mapping from qM to M"; ffi().nC], (mut = unsafe) D_rownnz : & [i32;
        "full inertia: non-zeros in each row"; ffi().nv], (mut = unsafe) D_rowadr : &
        [i32; "full inertia: row addresses"; ffi().nv], (mut = unsafe) D_diag : & [i32;
        "full inertia: index of diagonal element"; ffi().nv], (mut = unsafe) D_colind : &
        [i32; "full inertia: column indices"; ffi().nD], (mut = unsafe) mapM2D : & [i32;
        "index mapping from M to D"; ffi().nD], (mut = unsafe) mapD2M : & [i32;
        "index mapping from D to M"; ffi().nC]
    }
    array_slice_dyn! {
        sublen_dep { key_qpos : & [[MjtNum; ffi().nq] [force]; "key position"; ffi()
        .nkey], key_qvel : & [[MjtNum; ffi().nv] [force]; "key velocity"; ffi().nkey],
        key_act : & [[MjtNum; ffi().na] [force]; "key activation"; ffi().nkey], key_mpos
        : & [[MjtNum; ffi().nmocap * 3] [force]; "key mocap position"; ffi().nkey],
        key_mquat : & [[MjtNum; ffi().nmocap * 4] [force]; "key mocap quaternion"; ffi()
        .nkey], key_ctrl : & [[MjtNum; ffi().nu] [force]; "key control"; ffi().nkey],
        sensor_user : & [[MjtNum; ffi().nuser_sensor] [force]; "user data"; ffi()
        .nsensor], actuator_user : & [[MjtNum; ffi().nuser_actuator] [force];
        "user data"; ffi().nactuator], tendon_user : & [[MjtNum; ffi().nuser_tendon]
        [force]; "user data"; ffi().ntendon], cam_user : & [[MjtNum; ffi().nuser_cam]
        [force]; "user data"; ffi().ncam], site_user : & [[MjtNum; ffi().nuser_site]
        [force]; "user data"; ffi().nsite], geom_user : & [[MjtNum; ffi().nuser_geom]
        [force]; "user data"; ffi().ngeom], jnt_user : & [[MjtNum; ffi().nuser_jnt]
        [force]; "user data"; ffi().njnt], body_user : & [[MjtNum; ffi().nuser_body]
        [force]; "user data"; ffi().nbody] }
    }
}
impl Clone for MjModel {
    /// # Note
    /// MuJoCo aborts the process through `mjERROR` when an allocation fails, so this never fails.
    #[expect(
        deprecated,
        reason = "try_clone keeps the implementation until it is removed"
    )]
    fn clone(&self) -> Self {
        self.try_clone().expect("failed to clone model")
    }
}
impl Drop for MjModel {
    fn drop(&mut self) {
        unsafe {
            mj_deleteModel(self.ptr.as_ptr());
        }
    }
}
info_with_view!(
    Model, actuator, [[actuator_] group : i32, [actuator_] delay : MjtNum, [actuator_]
    ctrllimited : MjtBool, [actuator_] forcelimited : MjtBool, [actuator_] actlimited :
    MjtBool, [actuator_] dynprm : MjtNum, [actuator_] gainprm : MjtNum, [actuator_]
    biasprm : MjtNum, [actuator_] actearly : MjtBool, [actuator_] ctrlrange : MjtNum,
    [actuator_] forcerange : MjtNum, [actuator_] actrange : MjtNum, [actuator_] gear :
    MjtNum, [actuator_] damping : MjtNum, [actuator_] dampingpoly : MjtNum, [actuator_]
    armature : MjtNum, [actuator_] cranklength : MjtNum, [actuator_] acc0 : MjtNum,
    [actuator_] length0 : MjtNum, [actuator_] lengthrange : MjtNum, [actuator_] user :
    MjtNum, [actuator_] biastype : MjtBias[force], [actuator_] plugin : i32],
    [[actuator_] trntype : MjtTrn[force], [actuator_] dyntype : MjtDyn[force],
    [actuator_] ctrladr : i32, [actuator_] ctrlnum : i32, [actuator_] ctrlspec : i32,
    [actuator_] gaintype : MjtGain[force], [actuator_] outadr : i32, [actuator_] outnum :
    i32, [actuator_] trnid : i32, [actuator_] actadr : i32, [actuator_] actnum : i32,
    [actuator_] history : i32, [actuator_] historyadr : i32], []
);
info_with_view!(
    Model, body, [[body_] sameframe : MjtSameFrame[force], [body_] pos : MjtNum, [body_]
    quat : MjtNum, [body_] ipos : MjtNum, [body_] iquat : MjtNum, [body_] mass : MjtNum,
    [body_] subtreemass : MjtNum, [body_] inertia : MjtNum, [body_] invweight0 : MjtNum,
    [body_] gravcomp : MjtNum, [body_] margin : MjtNum, [body_] contype : i32, [body_]
    conaffinity : i32, [body_] user : MjtNum, [body_] simple : MjtByte, [body_] plugin :
    i32], [[body_] parentid : i32, [body_] rootid : i32, [body_] weldid : i32, [body_]
    mocapid : i32, [body_] jntnum : i32, [body_] jntadr : i32, [body_] dofnum : i32,
    [body_] dofadr : i32, [body_] treeid : i32, [body_] geomnum : i32, [body_] geomadr :
    i32, [body_] bvhadr : i32, [body_] bvhnum : i32], []
);
info_with_view!(
    Model, camera, [[cam_] mode : MjtCamLight[force], [cam_] pos : MjtNum, [cam_] quat :
    MjtNum, [cam_] poscom0 : MjtNum, [cam_] pos0 : MjtNum, [cam_] mat0 : MjtNum, [cam_]
    projection : MjtProjection[force], [cam_] fovy : MjtNum, [cam_] ipd : MjtNum, [cam_]
    output : i32, [cam_] sensorsize : f32, [cam_] intrinsic : f32, [cam_] user : MjtNum],
    [[cam_] bodyid : i32, [cam_] targetbodyid : i32, [cam_] resolution : i32], []
);
info_with_view!(
    Model, equality, [[eq_] active0 : MjtBool, [eq_] solref : MjtNum, [eq_] solimp :
    MjtNum, [eq_] data : MjtNum], [[eq_] r#type : MjtEq[force], [eq_] obj1id : i32, [eq_]
    obj2id : i32, [eq_] objtype : MjtObj[force]], []
);
info_with_view!(Model, exclude, [[exclude_] signature : i32], [], []);
info_with_view!(
    Model, geom, [[geom_] contype : i32, [geom_] conaffinity : i32, [geom_] group : i32,
    [geom_] priority : i32, [geom_] sameframe : MjtSameFrame[force], [geom_] solmix :
    MjtNum, [geom_] solref : MjtNum, [geom_] solimp : MjtNum, [geom_] size : MjtNum,
    [geom_] aabb : MjtNum, [geom_] rbound : MjtNum, [geom_] pos : MjtNum, [geom_] quat :
    MjtNum, [geom_] friction : MjtNum, [geom_] margin : MjtNum, [geom_] gap : MjtNum,
    [geom_] surfacevel : MjtNum, [geom_] adhesion : MjtNum, [geom_] fluid : MjtNum,
    [geom_] user : MjtNum, [geom_] rgba : f32], [[geom_] r#type : MjtGeom[force], [geom_]
    condim : i32, [geom_] bodyid : i32, [geom_] dataid : i32, [geom_] matid : i32,
    [geom_] plugin : i32], []
);
info_with_view!(
    Model, hfield, [[hfield_] size : MjtNum], [[hfield_] nrow : i32, [hfield_] ncol :
    i32, [hfield_] adr : i32, [hfield_] pathadr : i32], [[hfield_] data : f32]
);
info_with_view!(
    Model, joint, [qpos0 : MjtNum, qpos_spring : MjtNum, [jnt_] group : i32, [jnt_]
    limited : MjtBool, [jnt_] actfrclimited : MjtBool, [jnt_] actgravcomp : MjtBool,
    [jnt_] solref : MjtNum, [jnt_] solimp : MjtNum, [jnt_] pos : MjtNum, [jnt_] axis :
    MjtNum, [jnt_] stiffness : MjtNum, [jnt_] stiffnesspoly : MjtNum, [jnt_] range :
    MjtNum, [jnt_] actfrcrange : MjtNum, [jnt_] margin : MjtNum, [jnt_] user : MjtNum,
    [dof_] frictionloss : MjtNum, [dof_] armature : MjtNum, [dof_] damping : MjtNum,
    [dof_] dampingpoly : MjtNum, [dof_] invweight0 : MjtNum, [dof_] M0 : MjtNum, [dof_]
    simplenum : i32], [[jnt_] r#type : MjtJoint[force], [jnt_] qposadr : i32, [jnt_]
    dofadr : i32, [jnt_] bodyid : i32, [jnt_] actuatorid : i32, dof_bodyid : i32, [dof_]
    jntid : i32, [dof_] parentid : i32, dof_treeid : i32, [dof_] Madr : i32], []
);
info_with_view!(
    Model, light, [[light_] mode : MjtCamLight[force], [light_] r#type :
    MjtLightType[force], [light_] castshadow : MjtBool, [light_] bulbradius : f32,
    [light_] intensity : f32, [light_] range : f32, [light_] active : MjtBool, [light_]
    pos : MjtNum, [light_] dir : MjtNum, [light_] poscom0 : MjtNum, [light_] pos0 :
    MjtNum, [light_] dir0 : MjtNum, [light_] attenuation : f32, [light_] cutoff : f32,
    [light_] softness : f32, [light_] exponent : f32, [light_] ambient : f32, [light_]
    diffuse : f32, [light_] specular : f32], [[light_] bodyid : i32, [light_]
    targetbodyid : i32, [light_] texid : i32], []
);
info_with_view!(
    Model, material, [[mat_] texuniform : MjtBool, [mat_] texrepeat : f32, [mat_]
    emission : f32, [mat_] specular : f32, [mat_] shininess : f32, [mat_] reflectance :
    f32, [mat_] rgba : f32, [mat_] metallic : f32, [mat_] roughness : f32], [[mat_] texid
    : i32], []
);
info_with_view!(
    Model, mesh, [[mesh_] scale : MjtNum, [mesh_] pos : MjtNum, [mesh_] quat : MjtNum],
    [[mesh_] vertadr : i32, [mesh_] vertnum : i32, [mesh_] texcoordadr : i32, [mesh_]
    faceadr : i32, [mesh_] facenum : i32, [mesh_] graphadr : i32, [mesh_] extrema : i32,
    [mesh_] normaladr : i32, [mesh_] normalnum : i32, [mesh_] texcoordnum : i32, [mesh_]
    bvhadr : i32, [mesh_] bvhnum : i32, [mesh_] octadr : i32, [mesh_] octnum : i32,
    [mesh_] pathadr : i32, [mesh_] polynum : i32, [mesh_] polyadr : i32], []
);
info_with_view!(
    Model, numeric, [], [[numeric_] adr : i32, [numeric_] size : i32], [[numeric_] data :
    MjtNum]
);
info_with_view!(
    Model, pair, [[pair_] solref : MjtNum, [pair_] solimp : MjtNum, [pair_] margin :
    MjtNum, [pair_] gap : MjtNum, [pair_] adhesion : MjtNum, [pair_] friction : MjtNum,
    [pair_] solreffriction : MjtNum, [pair_] signature : i32], [[pair_] dim : i32,
    [pair_] geom1 : i32, [pair_] geom2 : i32], []
);
info_with_view!(
    Model, sensor, [[sensor_] cutoff : MjtNum, [sensor_] noise : MjtNum, [sensor_] delay
    : MjtNum, [sensor_] interval : MjtNum, [sensor_] user : MjtNum, [sensor_] datatype :
    MjtDataType[force], [sensor_] needstage : MjtStage[force]], [[sensor_] intprm : i32,
    [sensor_] r#type : MjtSensor[force], [sensor_] objid : i32, [sensor_] refid : i32,
    [sensor_] objtype : MjtObj[force], [sensor_] reftype : MjtObj[force], [sensor_] dim :
    i32, [sensor_] adr : i32, [sensor_] history : i32, [sensor_] historyadr : i32,
    [sensor_] plugin : i32], []
);
info_with_view!(
    Model, site, [[site_] group : i32, [site_] sameframe : MjtSameFrame[force], [site_]
    size : MjtNum, [site_] pos : MjtNum, [site_] quat : MjtNum, [site_] user : MjtNum,
    [site_] rgba : f32, [site_] r#type : MjtGeom[force]], [[site_] bodyid : i32, [site_]
    matid : i32], []
);
info_with_view!(
    Model, skin, [[skin_] group : i32, [skin_] rgba : f32, [skin_] inflate : f32],
    [[skin_] matid : i32, [skin_] vertadr : i32, [skin_] vertnum : i32, [skin_]
    texcoordadr : i32, [skin_] faceadr : i32, [skin_] facenum : i32, [skin_] boneadr :
    i32, [skin_] bonenum : i32, [skin_] pathadr : i32], []
);
info_with_view!(
    Model, tendon, [[tendon_] group : i32, [tendon_] limited : MjtBool, [tendon_]
    actfrclimited : MjtBool, [tendon_] width : MjtNum, [tendon_] solref_lim : MjtNum,
    [tendon_] solimp_lim : MjtNum, [tendon_] solref_fri : MjtNum, [tendon_] solimp_fri :
    MjtNum, [tendon_] range : MjtNum, [tendon_] actfrcrange : MjtNum, [tendon_] margin :
    MjtNum, [tendon_] stiffness : MjtNum, [tendon_] stiffnesspoly : MjtNum, [tendon_]
    damping : MjtNum, [tendon_] dampingpoly : MjtNum, [tendon_] armature : MjtNum,
    [tendon_] frictionloss : MjtNum, [tendon_] lengthspring : MjtNum, [tendon_] length0 :
    MjtNum, [tendon_] invweight0 : MjtNum, [tendon_] user : MjtNum, [tendon_] rgba : f32,
    [tendon_] treenum : i32], [[tendon_] matid : i32, [tendon_] actuatorid : i32,
    [tendon_] treeid : i32, [tendon_] adr : i32, [tendon_] num : i32, [ten_] J_rownnz :
    i32, [ten_] J_rowadr : i32, [ten_] J_colind : i32], []
);
info_with_view!(
    Model, texture, [[tex_] colorspace : MjtColorSpace[force]], [[tex_] r#type :
    MjtTexture[force], [tex_] height : i32, [tex_] width : i32, [tex_] nchannel : i32,
    [tex_] adr : MjtSize, [tex_] pathadr : i32], [[tex_] data : MjtByte]
);
info_with_view!(
    Model, tuple, [[tuple_] objprm : MjtNum, [tuple_] objtype : MjtObj[force]], [[tuple_]
    adr : i32, [tuple_] size : i32, [tuple_] objid : i32], []
);
info_with_view!(
    Model, key, [[key_] time : MjtNum, [key_] qpos : MjtNum, [key_] qvel : MjtNum, [key_]
    act : MjtNum, [key_] mpos : MjtNum, [key_] mquat : MjtNum, [key_] ctrl : MjtNum], [],
    []
);
