//! Definitions related to model editing.
use crate::native_binding::error::MjEditError;
use std::ffi::{CStr, CString, c_char, c_int};
use std::fmt;
use std::marker::PhantomData;
use std::path::Path;
use std::ptr::{self, NonNull};
#[macro_use]
mod utility;
use utility::*;
mod traits;
pub use traits::*;
mod default;
use super::mj_auxiliary::{MjLROpt, MjStatistic, MjVfs, MjVisual};
use super::mj_model::{
    MjModel, MjtBias, MjtCamLight, MjtColorSpace, MjtCubeFace, MjtDataType, MjtDyn, MjtEq,
    MjtFlexSelf, MjtGain, MjtGeom, MjtJoint, MjtLightType, MjtObj, MjtProjection, MjtSensor,
    MjtSleepPolicy, MjtStage, MjtTexture, MjtTextureRole, MjtTrn, MjtWrap,
};
use super::mj_option::MjOption;
use super::mj_primitive::*;
use crate::getter_setter;
use crate::native_binding::mujoco_c::*;
use crate::native_binding::mujoco_c::{
    mjs_addHField as mjs_addHfield, mjs_asHField as mjs_asHfield,
};
use crate::native_binding::util::{ERROR_BUF_LEN, assert_mujoco_version};
pub use default::*;
/// Validates that `t` is a real object type, i.e. an [`MjtObj`] discriminant below
/// [`MjtObj::mjNOBJECT`].
fn check_objtype(t: MjtObj) -> Result<(), MjEditError> {
    if (t as i32) < (MjtObj::mjNOBJECT as i32) {
        Ok(())
    } else {
        Err(MjEditError::InvalidParameter(format!(
            "object type must be a real MjtObj below MjtObj::mjNOBJECT, got {t:?}"
        )))
    }
}
/// Validates that a custom-numeric array size is non-negative.
fn check_numeric_size(size: i32) -> Result<(), MjEditError> {
    if size < 0 {
        Err(MjEditError::InvalidParameter(format!(
            "numeric size must be non-negative, got {size}"
        )))
    } else {
        Ok(())
    }
}
/// Type of inertia inference.
pub type MjtGeomInertia = mjtGeomInertia;
/// Type of mesh inertia.
pub type MjtMeshInertia = mjtMeshInertia;
/// Type of built-in procedural texture.
pub type MjtBuiltin = mjtBuiltin;
/// Type of built-in procedural mesh.
pub type MjtMeshBuiltin = mjtMeshBuiltin;
/// Mark type for procedural textures.
pub type MjtMark = mjtMark;
/// Type of limit specification.
pub type MjtLimited = mjtLimited;
/// Whether to align free joints with the inertial frame.
pub type MjtAlignFree = mjtAlignFree;
/// Whether to infer body inertias from child geoms.
pub type MjtInertiaFromGeom = mjtInertiaFromGeom;
/// Conflict-resolution policy used when attaching specifications.
pub type MjtConflict = mjtConflict;
/// Type of orientation specifier.
pub type MjtOrientation = mjtOrientation;
/// Compiler timing categories, used in `mjs_getTimer`.
pub type MjtCTimer = mjtCTimer;
/// Alternative orientation specifiers.
pub type MjsOrientation = mjsOrientation;
impl MjsOrientation {
    /// Sets orientation in Euler space.
    pub fn set_euler(&mut self, angle: &[f64; 3]) {
        self.type_ = MjtOrientation::mjORIENTATION_EULER;
        self.euler = *angle;
    }
    /// Sets orientation in axis angle space.
    pub fn set_axis_angle(&mut self, angle: &[f64; 4]) {
        self.type_ = MjtOrientation::mjORIENTATION_AXISANGLE;
        self.axisangle = *angle;
    }
    /// Sets orientation in XY axes space.
    pub fn set_xy_axis(&mut self, angle: &[f64; 6]) {
        self.type_ = MjtOrientation::mjORIENTATION_XYAXES;
        self.xyaxes = *angle;
    }
    /// Sets orientation in Z axis space.
    pub fn set_z_axis(&mut self, angle: &[f64; 3]) {
        self.type_ = MjtOrientation::mjORIENTATION_ZAXIS;
        self.zaxis = *angle;
    }
    /// Changes the orientation mode to quaternions. The orientation must
    /// be specified via the main angle attribute, not through [`MjsOrientation`].
    pub fn switch_quat(&mut self) {
        self.type_ = MjtOrientation::mjORIENTATION_QUAT;
    }
}
mjs_opaque!(
    MjsCompiler <= mjsCompiler,
    "Compiler options. An opaque handle for the FFI type [`mjsCompiler`], reached through \
[`ffi`](Self::ffi)."
);
impl MjsCompiler {
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] autolimits : bool;
        "infer \"limited\" attribute based on range."; [ffi, ffi_mut] balanceinertia :
        bool; "automatically impose A + B >= C rule."; [ffi, ffi_mut] fitaabb : bool;
        "meshfit to aabb instead of inertia box."; [ffi, ffi_mut] degree : bool;
        "angles in radians or degrees."; [ffi, ffi_mut] discardvisual : bool;
        "discard visual geoms in parser."; [ffi, ffi_mut] usethread : bool;
        "use multiple threads to speed up compiler."; [ffi, ffi_mut] fusestatic : bool;
        "fuse static bodies with parent."; [ffi, ffi_mut] saveinertial : bool;
        "save explicit inertial clause for all bodies to XML."; [ffi, ffi_mut] alignfree
        : bool; "align free joints with inertial frame.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] boundmass : f64;
        "enforce minimum body mass."; [ffi, ffi_mut] boundinertia : f64;
        "enforce minimum body diagonal inertia."; [ffi, ffi_mut] settotalmass : f64;
        "rescale masses and inertias; <=0: ignore.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] inertiafromgeom : MjtInertiaFromGeom[force];
        "use geom inertias."; [ffi, ffi_mut] conflict : MjtConflict[force];
        "conflict-resolution policy for attach.";]
    }
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] inertiagrouprange : & [i32; 2];
        "range of geom groups used to compute inertia."; [ffi, ffi_mut] eulerseq : &
        [c_char; 3]; "sequence for euler rotations."; [ffi, ffi_mut] LRopt : & MjLROpt;
        "options for lengthrange computation.";]
    }
    string_set_get_with! {
        [&] meshdir; "mesh and hfield directory."; texturedir; "texture directory.";
    }
    getter_setter! {
        get, [[ffi] authored : u64; "bitmask of authored compiler fields.";]
    }
}
/// Authored-field tracking bitmasks for [`mjModel`] structs.
///
/// Each field records, as a bitmask, which attributes of the corresponding section were
/// explicitly authored in the specification.
pub type MjsAuthored = mjsAuthored;
/// Model specification. This wraps the FFI type [`mjSpec`] internally.
///
/// Model editing is single-threaded. MuJoCo's C++ implementation shares unsynchronized state
/// between a specification and its elements, so this type is neither [`Send`] nor [`Sync`].
pub struct MjSpec {
    /// The specification that MuJoCo owns.
    ffi: NonNull<mjSpec>,
}
impl fmt::Debug for MjSpec {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MjSpec")
            .field("ffi", &self.ffi)
            .finish_non_exhaustive()
    }
}
impl MjSpec {
    /// Wraps a specification that MuJoCo allocated.
    fn from_ffi(ffi: NonNull<mjSpec>) -> Self {
        Self { ffi }
    }
    /// Creates an empty [`MjSpec`].
    ///
    /// # Panics
    /// When the linked MuJoCo version does not match the expected from MuJoCo-rs.
    #[expect(
        deprecated,
        reason = "try_new keeps the implementation until it is removed"
    )]
    pub fn new() -> Self {
        Self::try_new().expect("MuJoCo failed to allocate MjSpec")
    }
    /// Fallible version of [`MjSpec::new`].
    ///
    /// # Note
    /// MuJoCo ends the process when the allocation fails, so this never returns `Err`.
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] if MuJoCo fails to allocate
    /// the specification.
    ///
    /// # Panics
    /// When the linked MuJoCo version does not match the expected from MuJoCo-rs.
    #[deprecated(since = "6.0.0", note = "always returns Ok; use `new`")]
    pub fn try_new() -> Result<Self, MjEditError> {
        assert_mujoco_version();
        let ptr = unsafe { mj_makeSpec() };
        Ok(Self::from_ffi(
            NonNull::new(ptr).ok_or(MjEditError::AllocationFailed)?,
        ))
    }
    /// Creates a deep copy of this [`MjSpec`].
    ///
    /// A child specification that the model attaches from another file stays shared with the
    /// original, behind MuJoCo's own reference count.
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] if MuJoCo fails to allocate
    /// the copy (e.g. out of memory or an internal C++ exception).
    pub fn try_clone(&self) -> Result<Self, MjEditError> {
        let ptr = unsafe { mj_copySpec(self.ffi.as_ptr()) };
        NonNull::new(ptr)
            .map(Self::from_ffi)
            .ok_or(MjEditError::AllocationFailed)
    }
    /// Creates a [`MjSpec`] from the `path` to a file.
    /// # Errors
    /// - [`MjEditError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// - [`MjEditError::ParseFailed`] if MuJoCo fails to parse the XML.
    /// # Panics
    /// - when the `path` contains '\0'.
    /// - when the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_xml<T: AsRef<Path>>(path: T) -> Result<Self, MjEditError> {
        Self::from_xml_file(path, None)
    }
    /// Creates a [`MjSpec`] from the `path` to a file, located in a virtual file system (`vfs`).
    /// # Errors
    /// - [`MjEditError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// - [`MjEditError::ParseFailed`] if MuJoCo fails to parse the XML.
    /// # Panics
    /// - when the `path` contains '\0'.
    /// - when the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_xml_vfs<T: AsRef<Path>>(path: T, vfs: &MjVfs) -> Result<Self, MjEditError> {
        Self::from_xml_file(path, Some(vfs))
    }
    fn from_xml_file<T: AsRef<Path>>(path: T, vfs: Option<&MjVfs>) -> Result<Self, MjEditError> {
        assert_mujoco_version();
        let mut error_buffer = [0; ERROR_BUF_LEN];
        unsafe {
            let path_str = path.as_ref().to_str().ok_or(MjEditError::InvalidUtf8Path)?;
            let path = CString::new(path_str).unwrap();
            let raw_ptr = mj_parseXML(
                path.as_ptr(),
                vfs.map_or(ptr::null(), |v| v.ffi()),
                error_buffer.as_mut_ptr(),
                error_buffer.len() as c_int,
            );
            Self::check_spec(raw_ptr, &error_buffer)
        }
    }
    /// Creates a [`MjSpec`] from an `xml` string.
    /// # Errors
    /// Returns [`MjEditError::ParseFailed`] if MuJoCo encounters an error parsing the string.
    /// # Panics
    /// - when the `xml` contains '\0'.
    /// - when the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_xml_string(xml: &str) -> Result<Self, MjEditError> {
        assert_mujoco_version();
        let c_xml = CString::new(xml).unwrap();
        let mut error_buffer = [0; ERROR_BUF_LEN];
        unsafe {
            let spec_ptr = mj_parseXMLString(
                c_xml.as_ptr(),
                ptr::null(),
                error_buffer.as_mut_ptr(),
                error_buffer.len() as c_int,
            );
            Self::check_spec(spec_ptr, &error_buffer)
        }
    }
    /// Parse and create a [`MjSpec`] from `filename`.
    /// The `content_type` controls the decoder to use.
    /// This is a wrapper around low-level method [`mj_parse`].
    /// # Errors
    /// - [`MjEditError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// - [`MjEditError::ParseFailed`] if MuJoCo fails to parse the file.
    /// # Panics
    /// - When `content_type` or the path contain interior `\0` characters.
    /// - When the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_parse<T: AsRef<Path>>(
        filename: T,
        content_type: &str,
    ) -> Result<Self, MjEditError> {
        Self::from_parse_file(filename, content_type, None)
    }
    /// Same as [`MjSpec::from_parse`], except `filename` is taken from `vfs`.
    /// # Errors
    /// - [`MjEditError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// - [`MjEditError::ParseFailed`] if MuJoCo fails to parse the file.
    /// # Panics
    /// - When `content_type` or the path contain interior `\0` characters.
    /// - When the linked MuJoCo version does not match the expected from MuJoCo-rs.
    pub fn from_parse_vfs<T: AsRef<Path>>(
        filename: T,
        content_type: &str,
        vfs: &MjVfs,
    ) -> Result<Self, MjEditError> {
        Self::from_parse_file(filename, content_type, Some(vfs))
    }
    /// Parse and create a [`MjSpec`] from `filename`.
    /// The `content_type` controls the decoder to use.
    /// This is a wrapper around low-level method [`mj_parse`].
    /// # Panics
    /// - When `content_type` or the path contain interior `\0` characters.
    /// - When the linked MuJoCo version does not match the version MuJoCo-rs was compiled against.
    fn from_parse_file<T: AsRef<Path>>(
        filename: T,
        content_type: &str,
        vfs: Option<&MjVfs>,
    ) -> Result<Self, MjEditError> {
        assert_mujoco_version();
        let mut error_buffer = [0; ERROR_BUF_LEN];
        unsafe {
            let c_filename = CString::new(
                filename
                    .as_ref()
                    .to_str()
                    .ok_or(MjEditError::InvalidUtf8Path)?,
            )
            .unwrap();
            let c_content_type = CString::new(content_type).unwrap();
            let ptr = mj_parse(
                c_filename.as_ptr(),
                c_content_type.as_ptr(),
                vfs.map_or(ptr::null(), |v| v.ffi()),
                error_buffer.as_mut_ptr(),
                error_buffer.len() as i32,
            );
            Self::check_spec(ptr, &error_buffer)
        }
    }
    /// Handles spec pointer input.
    fn check_spec(spec_ptr: *mut mjSpec, error_buffer: &[c_char]) -> Result<Self, MjEditError> {
        if spec_ptr.is_null() {
            let message = unsafe { CStr::from_ptr(error_buffer.as_ptr()) }
                .to_string_lossy()
                .into_owned();
            Err(MjEditError::ParseFailed(message))
        } else {
            Ok(Self::from_ffi(unsafe { NonNull::new_unchecked(spec_ptr) }))
        }
    }
    /// An immutable reference to the internal FFI struct.
    pub fn ffi(&self) -> &mjSpec {
        unsafe { self.ffi.as_ref() }
    }
    /// A mutable reference to the internal FFI struct.
    ///
    /// # Safety
    /// Callers must ensure that any mutations performed through the returned reference
    /// preserve the invariants that MuJoCo expects for `mjSpec`.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjSpec {
        unsafe { self.ffi.as_mut() }
    }
    /// Delete an element from this specification.
    ///
    /// # Deprecated
    /// Call [`SpecObject::delete`] on the element handle instead.
    ///
    /// # Errors
    /// - [`MjEditError::DeleteFailed`] if `element` is null, does not belong to this spec, or
    ///   MuJoCo refuses the deletion.
    /// - [`MjEditError::UnsupportedOperation`] if `element` is a default class, a frame, a tendon
    ///   wrap, or the world body.
    ///
    /// # Safety
    /// Same contract as [`SpecObject::delete`], and `element` must point to an element of a
    /// specification.
    #[deprecated(since = "6.0.0", note = "use SpecObject::delete on the element handle")]
    pub unsafe fn delete_element(&mut self, element: *mut mjsElement) -> Result<(), MjEditError> {
        if element.is_null() {
            return Err(MjEditError::DeleteFailed("null element pointer".to_owned()));
        }
        if unsafe { (*element).elemtype } == MjtObj::mjOBJ_DEFAULT {
            return Err(MjEditError::UnsupportedOperation);
        }
        if unsafe { mjs_getSpec(element) } != self.ffi.as_ptr() {
            return Err(MjEditError::DeleteFailed(
                "element does not belong to this spec".to_owned(),
            ));
        }
        unsafe { utility::delete_element(element) }
    }
    /// Compile [`MjSpec`] to [`MjModel`].
    /// A spec can be edited and compiled multiple times,
    /// returning a new mjModel instance that takes the edits into account.
    /// # Errors
    /// Returns [`MjEditError::CompileFailed`] if the model fails to compile, including when a
    /// texture has a builtin pattern set while its `nchannel` is less than 3, and when a texture
    /// has a negative dimension or a pixel count that does not fit in an [`i32`].
    pub fn compile(&mut self) -> Result<MjModel, MjEditError> {
        for texture in self.texture_iter() {
            if texture.builtin() != MjtBuiltin::mjBUILTIN_NONE && texture.nchannel() < 3 {
                return Err(MjEditError::CompileFailed(
                    "texture with a builtin pattern requires nchannel >= 3".to_owned(),
                ));
            }
            let (nchannel, width, height) = (texture.nchannel(), texture.width(), texture.height());
            if nchannel < 0 || width < 0 || height < 0 {
                return Err(MjEditError::CompileFailed(
                    "texture nchannel, width and height must be non-negative".to_owned(),
                ));
            }
            if i64::from(nchannel) * i64::from(width) * i64::from(height) > i64::from(i32::MAX) {
                return Err(MjEditError::CompileFailed(
                    "texture nchannel*width*height must fit in an i32".to_owned(),
                ));
            }
        }
        let result = unsafe { MjModel::from_raw(mj_compile(self.ffi.as_ptr(), ptr::null())) };
        result.map_err(|_| {
            let error_msg: String = unsafe {
                let ptr = mjs_getError(self.ffi_mut());
                if ptr.is_null() {
                    "Compilation failed (unknown error)".to_owned()
                } else {
                    CStr::from_ptr(ptr).to_string_lossy().into_owned()
                }
            };
            MjEditError::CompileFailed(error_msg)
        })
    }
    /// Return the compiler timers, in seconds, in `mjtCTimer` order.
    pub fn timer(&self) -> &[f64; MjtCTimer::mjNCTIMER as usize] {
        unsafe { &*mjs_getTimer(self.ffi.as_ptr()).cast() }
    }
    /// Get number of warnings accumulated in the spec. Wraps [`mjs_numWarnings`].
    pub fn num_warnings(&self) -> i32 {
        unsafe { mjs_numWarnings(self.ffi.as_ptr()) }
    }
    /// Get the i-th warning message. Returns `None` if the index is out of bounds, or if the
    /// message is not valid UTF-8. Wraps [`mjs_getWarning`].
    pub fn warning(&self, index: i32) -> Option<&str> {
        let ptr = unsafe { mjs_getWarning(self.ffi.as_ptr(), index) };
        if ptr.is_null() {
            None
        } else {
            unsafe { CStr::from_ptr(ptr) }.to_str().ok()
        }
    }
    /// Saves the spec to an XML file.
    /// # Errors
    /// - [`MjEditError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// - [`MjEditError::SaveFailed`] with MuJoCo's error message if saving fails.
    /// # Panics
    /// When `filename` contains interior `\0` characters.
    pub fn save_xml<T: AsRef<Path>>(&self, filename: T) -> Result<(), MjEditError> {
        let mut error_buff = [0; ERROR_BUF_LEN];
        let cname = CString::new(
            filename
                .as_ref()
                .to_str()
                .ok_or(MjEditError::InvalidUtf8Path)?,
        )
        .unwrap();
        let result = unsafe {
            mj_saveXML(
                self.ffi(),
                cname.as_ptr(),
                error_buff.as_mut_ptr(),
                error_buff.len() as i32,
            )
        };
        match result {
            0 => Ok(()),
            _ => {
                let message = unsafe { CStr::from_ptr(error_buff.as_ptr()) }
                    .to_string_lossy()
                    .into_owned();
                Err(MjEditError::SaveFailed(message))
            }
        }
    }
    /// Saves the spec to an XML string.
    /// `buffer_size` controls how many bytes are allocated for the output.
    /// # Errors
    /// - [`MjEditError::XmlBufferTooSmall`] when `buffer_size` is too small.
    ///   The `required_size` field uses `snprintf`-style semantics (bytes to write, excluding NUL),
    ///   so retry with `required_size as usize + 1` bytes.
    /// - [`MjEditError::SaveFailed`] with MuJoCo's error message on any other failure.
    /// # Panics
    /// Panics if MuJoCo reports success but returns XML that is not NUL-terminated
    /// within the allocated output buffer.
    pub fn save_xml_string(&self, buffer_size: usize) -> Result<String, MjEditError> {
        let mut error_buff = [0; ERROR_BUF_LEN];
        let mut result_buff = vec![0u8; buffer_size];
        let result = unsafe {
            mj_saveXMLString(
                self.ffi(),
                result_buff.as_mut_ptr().cast(),
                result_buff.len() as i32,
                error_buff.as_mut_ptr(),
                error_buff.len() as i32,
            )
        };
        match result {
            0 => Ok(CStr::from_bytes_until_nul(&result_buff)
                .unwrap()
                .to_string_lossy()
                .into_owned()),
            r if r > 0 => Err(MjEditError::XmlBufferTooSmall {
                required_size: r as usize,
            }),
            _ => {
                let message = unsafe { CStr::from_ptr(error_buff.as_ptr()) }
                    .to_string_lossy()
                    .into_owned();
                Err(MjEditError::SaveFailed(message))
            }
        }
    }
}
/// Children accessor methods.
impl MjSpec {
    find_x_method! {
        body, geom, joint, site, camera, light, frame, actuator, sensor, flex, pair,
        equality, exclude, tendon, numeric, text, tuple, key, mesh, hfield, skin,
        texture, material, plugin
    }
    find_x_method_direct! {
        default
    }
    /// Returns an immutable reference to the world body.
    /// # Panics
    /// Panics if the "world" body is not found.
    pub fn world_body(&self) -> &MjsBody {
        self.body("world").unwrap()
    }
    /// Returns a mutable reference to the world body.
    /// # Panics
    /// Panics if the "world" body is not found.
    pub fn world_body_mut(&mut self) -> &mut MjsBody {
        self.body_mut("world").unwrap()
    }
}
/// Public attributes.
impl MjSpec {
    string_set_get_with! {
        modelname; "model name."; comment; "comment at top of XML."; modelfiledir;
        "path to model file.";
    }
    getter_setter! {
        with, get, [[ffi, ffi_mut] stat : & MjStatistic; "statistic overrides."; [ffi,
        ffi_mut] visual : & MjVisual; "visualization options."; [ffi, ffi_mut] option : &
        MjOption; "simulation options.";]
    }
    nested_handle!(compiler : MjsCompiler; "compiler options.");
    getter_setter! {
        get, [[ffi] (allow_mut = false) authored : & MjsAuthored;
        "authored-field tracking bitmasks.";]
    }
    getter_setter! {
        with, get, set, [[ffi, ffi_mut] strippath : bool;
        "whether to strip paths from mesh files."; [ffi, ffi_mut] hasImplicitPluginElem :
        bool; "already encountered an implicit plugin sensor/actuator.";]
    }
    getter_setter! {
        get, [[ffi] memory : MjtSize; "number of bytes in arena+stack memory."; [ffi]
        nemax : i32; "max number of equality constraints."; [ffi] nuserdata : i32;
        "number of mjtNums in userdata."; [ffi] nuser_body : i32;
        "number of mjtNums in body_user."; [ffi] nuser_jnt : i32;
        "number of mjtNums in jnt_user."; [ffi] nuser_geom : i32;
        "number of mjtNums in geom_user."; [ffi] nuser_site : i32;
        "number of mjtNums in site_user."; [ffi] nuser_cam : i32;
        "number of mjtNums in cam_user."; [ffi] nuser_tendon : i32;
        "number of mjtNums in tendon_user."; [ffi] nuser_actuator : i32;
        "number of mjtNums in actuator_user."; [ffi] nuser_sensor : i32;
        "number of mjtNums in sensor_user."; [ffi] nkey : i32; "number of keyframes.";]
    }
}
/// Methods for adding non-tree elements.
impl MjSpec {
    add_x_method! {
        actuator, pair, equality, tendon, mesh, material
    }
    add_x_method_no_default! {
        sensor, flex, exclude, numeric, text, tuple, key, plugin, hfield, skin, texture
    }
    /// Adds a new `<default>` element.
    ///
    /// # Panics
    /// Panics when `class_name` already exists or `parent_class_name` doesn't exist.
    /// Also panics when the `class_name` or `parent_class_name` contain '\0' characters.
    ///
    /// Use [`MjSpec::try_add_default`] for a fallible alternative.
    pub fn add_default(
        &mut self,
        class_name: &str,
        parent_class_name: Option<&str>,
    ) -> &mut MjsDefault {
        self.try_add_default(class_name, parent_class_name).unwrap()
    }
    /// Fallible version of [`MjSpec::add_default`].
    /// # Errors
    /// Returns [`MjEditError::AlreadyExists`] when `class_name` already exists.
    /// Returns [`MjEditError::NotFound`] when `parent_class_name` doesn't exist.
    /// # Panics
    /// When the `class_name` or `parent_class_name` contain '\0' characters, a panic occurs.
    pub fn try_add_default(
        &mut self,
        class_name: &str,
        parent_class_name: Option<&str>,
    ) -> Result<&mut MjsDefault, MjEditError> {
        let c_class_name = CString::new(class_name).unwrap();
        let parent_ptr = if let Some(name) = parent_class_name {
            self.default(name).ok_or(MjEditError::NotFound)?.ffi()
        } else {
            ptr::null()
        };
        unsafe {
            let ptr_default = mjs_addDefault(self.ffi_mut(), c_class_name.as_ptr(), parent_ptr);
            if ptr_default.is_null() {
                Err(MjEditError::AlreadyExists)
            } else {
                Ok(MjsDefault::from_ffi_ptr_mut(ptr_default).unwrap())
            }
        }
    }
}
/// Mutable iterator over items in [`MjSpec`].
#[derive(Debug)]
pub struct MjsSpecItemIterMut<'a, T> {
    /// Raw pointer to the spec; a borrow would alias the handles that the iterator yields.
    ffi_ptr: *mut mjSpec,
    /// Element that the last `next` yielded. Null marks the end of the iteration.
    last: *mut mjsElement,
    item_type: PhantomData<&'a mut T>,
}
/// Immutable iterator over items in [`MjSpec`].
#[derive(Debug, Clone)]
pub struct MjsSpecItemIter<'a, T> {
    ffi_ptr: *const mjSpec,
    /// Element that the last `next` yielded. Null marks the end of the iteration.
    last: *mut mjsElement,
    item_type: PhantomData<&'a T>,
}
impl<'a, T: SpecObject> MjsSpecItemIterMut<'a, T> {
    fn new(root: &'a mut MjSpec) -> Self {
        let last = unsafe { mjs_firstElement(root.ffi.as_ptr(), T::OBJ_TYPE) };
        Self {
            ffi_ptr: root.ffi.as_ptr(),
            last,
            item_type: PhantomData,
        }
    }
}
impl<'a, T: SpecObject> MjsSpecItemIter<'a, T> {
    fn new(root: &'a MjSpec) -> Self {
        let last = unsafe { mjs_firstElement(root.ffi.as_ptr(), T::OBJ_TYPE) };
        Self {
            ffi_ptr: root.ffi.as_ptr(),
            last,
            item_type: PhantomData,
        }
    }
}
impl<'a, T: SpecObject + 'a> Iterator for MjsSpecItemIterMut<'a, T> {
    type Item = &'a mut T;
    fn next(&mut self) -> Option<Self::Item> {
        if self.last.is_null() {
            return None;
        }
        unsafe {
            let out = T::from_element_as_ptr_mut(self.last).as_mut();
            self.last = mjs_nextElement(self.ffi_ptr, self.last);
            out
        }
    }
}
impl<'a, T: SpecObject + 'a> Iterator for MjsSpecItemIter<'a, T> {
    type Item = &'a T;
    fn next(&mut self) -> Option<Self::Item> {
        if self.last.is_null() {
            return None;
        }
        unsafe {
            let out = T::from_element_as_ptr_mut(self.last).as_ref();
            self.last = mjs_nextElement(self.ffi_ptr, self.last);
            out
        }
    }
}
impl<'a, T: SpecObject + 'a> std::iter::FusedIterator for MjsSpecItemIterMut<'a, T> {}
impl<'a, T: SpecObject + 'a> std::iter::FusedIterator for MjsSpecItemIter<'a, T> {}
/// Iterator methods.
impl MjSpec {
    spec_get_iter! {
        geom, joint, site, camera, light, frame, actuator, sensor, flex, pair, equality,
        exclude, tendon, numeric, text, tuple, key, mesh, hfield, skin, texture,
        material, plugin
    }
    spec_get_iter!(read_only : body);
}
impl Default for MjSpec {
    fn default() -> Self {
        Self::new()
    }
}
impl Drop for MjSpec {
    fn drop(&mut self) {
        unsafe {
            mj_deleteSpec(self.ffi.as_ptr());
        }
    }
}
impl Clone for MjSpec {
    /// Creates a deep copy of this [`MjSpec`].
    ///
    /// # Panics
    /// Panics if MuJoCo raises an error while it copies the spec.
    /// Use [`MjSpec::try_clone`] for a fallible alternative.
    fn clone(&self) -> Self {
        self.try_clone().expect("MuJoCo failed to clone MjSpec")
    }
}
mjs_struct!(Site with SpecObject : MjsSite <= mjsSite);
impl MjsSite {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] pos : & [f64; 3]; "position."; [ffi, ffi_mut] quat
        : & [f64; 4]; "orientation."; [ffi, ffi_mut] alt : & MjsOrientation;
        "alternative orientation."; [ffi, ffi_mut] fromto : & [f64; 6];
        "alternative for capsule, cylinder, box, ellipsoid."; [ffi, ffi_mut] size : &
        [f64; 3]; "geom size."; [ffi, ffi_mut] rgba : & [f32; 4];
        "rgba when material is omitted.";]
    }
    getter_setter!(
        [&] with, get, set, [[ffi, ffi_mut] type_ + _ : MjtGeom; "geom type."; [ffi,
        ffi_mut] group : i32; "group.";]
    );
    userdata_method!(f64);
    string_set_get_with! {
        [&] material; "name of material.";
    }
}
mjs_struct!(Joint with SpecObject : MjsJoint <= mjsJoint);
impl MjsJoint {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] pos : & [f64; 3]; "anchor position."; [ffi,
        ffi_mut] axis : & [f64; 3]; "joint axis."; [ffi, ffi_mut] ref_ + _ : & f64;
        "value at reference configuration: qpos0."; [ffi, ffi_mut] springdamper : & [f64;
        2]; "timeconst, dampratio."; [ffi, ffi_mut] stiffness : & [f64; mjNPOLY as usize
        + 1]; "stiffness coefficients."; [ffi, ffi_mut] range : & [f64; 2];
        "joint limits."; [ffi, ffi_mut] solref_limit : & [MjtNum; mjNREF as usize];
        "solver reference: joint limits."; [ffi, ffi_mut] solimp_limit : & [MjtNum;
        mjNIMP as usize]; "solver impedance: joint limits."; [ffi, ffi_mut] actfrcrange :
        & [f64; 2]; "actuator force limits."; [ffi, ffi_mut] damping : & [f64; mjNPOLY as
        usize + 1]; "damping coefficients."; [ffi, ffi_mut] solref_friction : & [MjtNum;
        mjNREF as usize]; "solver reference: dof friction."; [ffi, ffi_mut]
        solimp_friction : & [MjtNum; mjNIMP as usize];
        "solver impedance: dof friction.";]
    }
    getter_setter!(
        [&] with, get, set, [[ffi, ffi_mut] type_ + _ : MjtJoint; "joint type."; [ffi,
        ffi_mut] group : i32; "joint group."; [ffi, ffi_mut] springref : f64;
        "spring reference value: qpos_spring."; [ffi, ffi_mut] margin : f64;
        "margin value for joint limit detection."; [ffi, ffi_mut] armature : f64;
        "armature inertia (mass for slider)."; [ffi, ffi_mut] frictionloss : f64;
        "friction loss.";]
    );
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] align : MjtAlignFree[force];
        "align free joint with body com (mjtAlignFree)."; [ffi, ffi_mut] limited :
        MjtLimited[force]; "does joint have limits (mjtLimited)."; [ffi, ffi_mut]
        actfrclimited : MjtLimited[force];
        "are actuator forces on joint limited (mjtLimited).";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] actgravcomp : bool;
        "is gravcomp force applied via actuators.";]
    }
    userdata_method!(f64);
}
mjs_struct!(Geom with SpecObject : MjsGeom <= mjsGeom);
impl MjsGeom {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] pos : & [f64; 3]; "geom position."; [ffi, ffi_mut]
        quat : & [f64; 4]; "geom orientation."; [ffi, ffi_mut] alt : & MjsOrientation;
        "alternative orientation."; [ffi, ffi_mut] fromto : & [f64; 6];
        "alternative for capsule, cylinder, box, ellipsoid."; [ffi, ffi_mut] size : &
        [f64; 3]; "geom size."; [ffi, ffi_mut] rgba : & [f32; 4];
        "rgba when material is omitted."; [ffi, ffi_mut] friction : & [f64; 3];
        "one-sided friction coefficients: slide, spin, roll."; [ffi, ffi_mut] solref : &
        [MjtNum; mjNREF as usize]; "solver reference."; [ffi, ffi_mut] solimp : &
        [MjtNum; mjNIMP as usize]; "solver impedance."; [ffi, ffi_mut] surfacevel : &
        [f64; 6]; "surface velocity in local frame: linear, angular."; [ffi, ffi_mut]
        fluid_coefs : & [MjtNum; 5]; "ellipsoid-fluid interaction coefs."]
    }
    nested_handle!(plugin : MjsPluginReference; "sdf plugin.");
    getter_setter!(
        [&] with, get, set, [[ffi, ffi_mut] type_ + _ : MjtGeom; "geom type."; [ffi,
        ffi_mut] group : i32; "group."; [ffi, ffi_mut] contype : i32; "contact type.";
        [ffi, ffi_mut] conaffinity : i32; "contact affinity."; [ffi, ffi_mut] condim :
        i32; "contact dimensionality."; [ffi, ffi_mut] priority : i32;
        "contact priority."; [ffi, ffi_mut] solmix : f64;
        "solver mixing for contact pairs."; [ffi, ffi_mut] margin : f64;
        "margin for contact detection."; [ffi, ffi_mut] gap : f64;
        "additional contact detection buffer."; [ffi, ffi_mut] adhesion : f64;
        "adhesive force of contacts."; [ffi, ffi_mut] mass : f64;
        "used to compute density."; [ffi, ffi_mut] density : f64;
        "used to compute mass and inertia from volume or surface."; [ffi, ffi_mut]
        typeinertia : MjtGeomInertia; "selects between surface and volume inertia.";
        [ffi, ffi_mut] fluid_ellipsoid : MjtNum;
        "whether ellipsoid-fluid model is active."; [ffi, ffi_mut] fitscale : f64;
        "scale mesh uniformly.";]
    );
    userdata_method!(f64);
    string_set_get_with! {
        [&] meshname; "mesh attached to geom."; material; "name of material.";
        hfieldname; "heightfield attached to geom.";
    }
}
mjs_struct!(Camera with SpecObject : MjsCamera <= mjsCamera);
impl MjsCamera {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] pos : & [f64; 3]; "camera position."; [ffi,
        ffi_mut] quat : & [f64; 4]; "camera orientation."; [ffi, ffi_mut] alt : &
        MjsOrientation; "alternative orientation."; [ffi, ffi_mut] intrinsic : & [f32;
        4]; "intrinsic parameters."; [ffi, ffi_mut] sensor_size : & [f32; 2];
        "sensor size."; [ffi, ffi_mut] resolution : & [i32; 2]; "resolution."; [ffi,
        ffi_mut] focal_length : & [f32; 2]; "focal length (length)."; [ffi, ffi_mut]
        focal_pixel : & [f32; 2]; "focal length (pixel)."; [ffi, ffi_mut]
        principal_length : & [f32; 2]; "principal point (length)."; [ffi, ffi_mut]
        principal_pixel : & [f32; 2]; "principal point (pixel).";]
    }
    getter_setter!(
        [&] with, get, set, [[ffi, ffi_mut] mode : MjtCamLight; "camera mode."; [ffi,
        ffi_mut] fovy : f64; "field of view in y direction."; [ffi, ffi_mut] ipd : f64;
        "inter-pupillary distance for stereo."; [ffi, ffi_mut] proj : MjtProjection;
        "camera projection type."; [ffi, ffi_mut] output : i32;
        "bit flags for output type.";]
    );
    userdata_method!(f64);
    string_set_get_with! {
        [&] targetbody; "target body for tracking/targeting.";
    }
}
mjs_struct!(Light with SpecObject : MjsLight <= mjsLight);
impl MjsLight {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] pos : & [f64; 3]; "light position."; [ffi,
        ffi_mut] dir : & [f64; 3]; "light direction."; [ffi, ffi_mut] ambient : & [f32;
        3]; "ambient color."; [ffi, ffi_mut] diffuse : & [f32; 3]; "diffuse color.";
        [ffi, ffi_mut] specular : & [f32; 3]; "specular color."; [ffi, ffi_mut]
        attenuation : & [f32; 3]; "OpenGL attenuation (quadratic model).";]
    }
    getter_setter!(
        [&] with, get, set, [[ffi, ffi_mut] mode : MjtCamLight; "light mode."; [ffi,
        ffi_mut] type_ + _ : MjtLightType; "light type."; [ffi, ffi_mut] bulbradius :
        f32; "bulb radius, for soft shadows."; [ffi, ffi_mut] intensity : f32;
        "intensity, in candelas."; [ffi, ffi_mut] range : f32; "range of effectiveness.";
        [ffi, ffi_mut] cutoff : f32; "OpenGL cutoff."; [ffi, ffi_mut] softness : f32;
        "spotlight edge softness."; [ffi, ffi_mut] exponent : f32; "OpenGL exponent.";]
    );
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] active : bool; "active flag."; [ffi, ffi_mut]
        castshadow : bool; "whether light cast shadows."]
    }
    string_set_get_with! {
        [&] texture; "texture name for image lights."; targetbody;
        "target body for targeting.";
    }
}
mjs_struct!(Frame with SpecObject : MjsFrame <= mjsFrame);
impl MjsFrame {
    add_x_method_by_frame! {
        body, site, joint, geom, camera, light
    }
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] pos : & [f64; 3]; "frame position."; [ffi,
        ffi_mut] quat : & [f64; 4]; "frame orientation."; [ffi, ffi_mut] alt : &
        MjsOrientation; "alternative orientation.";]
    }
    string_set_get_with! {
        [&] childclass; "childclass name.";
    }
    /// Add and return a child frame.
    ///
    /// # Note
    /// MuJoCo ends the process when the allocation fails.
    #[expect(
        deprecated,
        reason = "try_add_frame keeps the implementation until it is removed"
    )]
    pub fn add_frame(&mut self) -> &mut MjsFrame {
        self.try_add_frame()
            .expect("mjs_addFrame returned null; allocation failed")
    }
    /// Fallible version of [`Self::add_frame`].
    ///
    /// # Note
    /// MuJoCo ends the process when the allocation fails, so this never returns `Err`.
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] when MuJoCo fails to allocate
    /// the frame, instead of panicking.
    #[deprecated(since = "6.0.0", note = "always returns Ok; use `add_frame`")]
    pub fn try_add_frame(&mut self) -> Result<&mut MjsFrame, MjEditError> {
        let parent_body = unsafe { mjs_getParent(self.element_mut_pointer()) };
        debug_assert!(
            !parent_body.is_null(),
            "mjs_getParent returned null; frame has no parent body"
        );
        let ptr = unsafe { mjs_addFrame(parent_body, self.ffi_mut()) };
        unsafe { MjsFrame::from_ffi_ptr_mut(ptr) }.ok_or(MjEditError::AllocationFailed)
    }
}
mjs_struct!(Actuator with SpecObject : MjsActuator <= mjsActuator);
impl MjsActuator {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] gear : & [f64; 6]; "gear parameters."; [ffi,
        ffi_mut] gainprm : & [f64; mjNGAIN as usize]; "gain parameters."; [ffi, ffi_mut]
        biasprm : & [f64; mjNBIAS as usize]; "bias parameters."; [ffi, ffi_mut] dynprm :
        & [f64; mjNDYN as usize]; "dynamic parameters."; [ffi, ffi_mut] lengthrange : &
        [f64; 2]; "transmission length range."; [ffi, ffi_mut] damping : & [f64; mjNPOLY
        as usize + 1]; "damping coefficients."; [ffi, ffi_mut] ctrlrange : & [f64; 2];
        "control range."; [ffi, ffi_mut] velrange : & [f64; 2];
        "range of the velocity-setpoint input (pid)."; [ffi, ffi_mut] ffrange : & [f64;
        2]; "range of the feedforward input (pid)."; [ffi, ffi_mut] forcerange : & [f64;
        2]; "force range."; [ffi, ffi_mut] actrange : & [f64; 2]; "activation range.";]
    }
    nested_handle!(plugin : MjsPluginReference; "actuator plugin.");
    getter_setter!(
        [&] with, get, set, [[ffi, ffi_mut] gaintype : MjtGain; "gain type."; [ffi,
        ffi_mut] biastype : MjtBias; "bias type."; [ffi, ffi_mut] dyntype : MjtDyn;
        "dyn type."; [ffi, ffi_mut] group : i32; "group."; [ffi, ffi_mut] actdim : i32;
        "number of activation variables."; [ffi, ffi_mut] trntype : MjtTrn;
        "transmission type."; [ffi, ffi_mut] cranklength : f64;
        "crank length, for slider-crank."; [ffi, ffi_mut] inheritrange : f64;
        "automatic range setting for position and intvelocity."; [ffi, ffi_mut] armature
        : f64; "armature inertia."; [ffi, ffi_mut] nsample : i32;
        "number of samples in history buffer."; [ffi, ffi_mut] interp : i32;
        "interpolation order (0=ZOH, 1=linear, 2=cubic)."; [ffi, ffi_mut] delay : f64;
        "delay time in seconds; 0: no delay."; [ffi, ffi_mut] ctrlspec : i32;
        "input signature, scoped by gaintype; 0: type default.";]
    );
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] ctrllimited : MjtLimited[force];
        "are control limits defined."; [ffi, ffi_mut] forcelimited : MjtLimited[force];
        "are force limits defined.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] actlimited : MjtLimited[force];
        "are activation limits defined.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] actearly : bool;
        "apply next activations to qfrc.";]
    }
    userdata_method!(f64);
    string_set_get_with! {
        [&] target; "name of transmission target."; refsite;
        "reference site, for site transmission."; slidersite;
        "site defining cylinder, for slider-crank.";
    }
}
/// Converts the string that MuJoCo's `mjs_setToX` actuator functions return into a [`Result`].
/// An empty string is success; anything else names the rejected parameter.
fn actuator_set_result(c_err_msg: *const c_char) -> Result<(), MjEditError> {
    let err_msg = unsafe { CStr::from_ptr(c_err_msg) }.to_string_lossy();
    if err_msg.is_empty() {
        Ok(())
    } else {
        Err(MjEditError::InvalidParameter(err_msg.into_owned()))
    }
}
/// Configuration for [`MjsActuator::set_to_position`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PositionConfig {
    /// Proportional (position) gain.
    pub kp: f64,
    /// Automatic range-inheritance factor (0 disables it).
    pub inheritrange: f64,
    /// Velocity feedback gain. Mutually exclusive with `dampratio`.
    pub kv: Option<f64>,
    /// Damping ratio. Mutually exclusive with `kv`.
    pub dampratio: Option<f64>,
    /// First-order activation-filter time constant.
    pub timeconst: Option<f64>,
}
impl PositionConfig {
    getter_setter! {
        with, [kp : f64; "the proportional (position) gain."; inheritrange : f64;
        "the automatic range-inheritance factor."; kv : f64;
        "the velocity feedback gain (mutually exclusive with dampratio)."; dampratio :
        f64; "the damping ratio (mutually exclusive with kv)."; timeconst : f64;
        "the first-order activation-filter time constant.";]
    }
}
/// Configuration for [`MjsActuator::set_to_int_velocity`]. Same parameters as [`PositionConfig`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct IntVelocityConfig {
    /// Proportional gain.
    pub kp: f64,
    /// Automatic range-inheritance factor (0 disables it).
    pub inheritrange: f64,
    /// Velocity feedback gain. Mutually exclusive with `dampratio`.
    pub kv: Option<f64>,
    /// Damping ratio. Mutually exclusive with `kv`.
    pub dampratio: Option<f64>,
    /// First-order activation-filter time constant.
    pub timeconst: Option<f64>,
}
impl IntVelocityConfig {
    getter_setter! {
        with, [kp : f64; "the proportional gain."; inheritrange : f64;
        "the automatic range-inheritance factor."; kv : f64;
        "the velocity feedback gain (mutually exclusive with dampratio)."; dampratio :
        f64; "the damping ratio (mutually exclusive with kv)."; timeconst : f64;
        "the first-order activation-filter time constant.";]
    }
}
/// Configuration for [`MjsActuator::set_to_dc_motor`].
///
/// Each optional field defaults to `None`, disabling the corresponding feature.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DcMotorConfig {
    /// Electrical resistance.
    pub resistance: f64,
    /// Input signature: a bitmask of
    /// [`MjtCtrlInput`](crate::native_binding::wrappers::mj_model::MjtCtrlInput) values.
    pub ctrlspec: i32,
    /// Torque and back-EMF constants `[Kt, Ke]`.
    pub motorconst: Option<[f64; 2]>,
    /// Nominal ratings `[voltage, stall_torque, no_load_speed]`.
    pub nominal: Option<[f64; 3]>,
    /// Saturation `[tau_max, i_max, di_dt_max]`.
    pub saturation: Option<[f64; 3]>,
    /// Inductance `[L, te]`.
    pub inductance: Option<[f64; 2]>,
    /// Cogging `[amplitude, periodicity, phase]`.
    pub cogging: Option<[f64; 3]>,
    /// Controller `[kp, ki, kd, slewmax, Imax, v_max]`.
    pub controller: Option<[f64; 6]>,
    /// Thermal `[R_th, C, tau_th, alpha, T0, T_ambient]`.
    pub thermal: Option<[f64; 6]>,
    /// LuGre friction `[stiffness, damping, coulomb, static, stribeck]`.
    pub lugre: Option<[f64; 5]>,
}
impl DcMotorConfig {
    getter_setter! {
        with, [resistance : f64; "the electrical resistance."; ctrlspec : i32;
        "the input signature bitmask ([`MjtCtrlInput`](crate::native_binding::wrappers::mj_model::MjtCtrlInput)).";
        motorconst : [f64; 2]; "the torque and back-EMF constants [Kt, Ke]."; nominal :
        [f64; 3]; "the nominal ratings [voltage, stall_torque, no_load_speed].";
        saturation : [f64; 3]; "the saturation [tau_max, i_max, di_dt_max]."; inductance
        : [f64; 2]; "the inductance [L, te]."; cogging : [f64; 3];
        "the cogging [amplitude, periodicity, phase]."; controller : [f64; 6];
        "the controller [kp, ki, kd, slewmax, Imax, v_max]."; thermal : [f64; 6];
        "the thermal [R_th, C, tau_th, alpha, T0, T_ambient]."; lugre : [f64; 5];
        "the LuGre friction [stiffness, damping, coulomb, static, stribeck].";]
    }
}
/// Configuration for [`MjsActuator::set_to_pid`].
///
/// Each optional field defaults to `None`, disabling the corresponding feature.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PidConfig {
    /// Proportional (position) gain.
    pub kp: f64,
    /// Velocity feedback gain. Mutually exclusive with `dampratio`.
    pub kv: Option<f64>,
    /// Damping ratio. Mutually exclusive with `kv`.
    pub dampratio: Option<f64>,
    /// Integral gain on the position error.
    pub ki: Option<f64>,
    /// Anti-windup limit on the integral state.
    pub imax: Option<f64>,
    /// Slew rate limit of the position setpoint.
    pub slewmax: Option<f64>,
    /// Automatic range-inheritance factor for the position-setpoint range (0 disables it).
    pub inheritrange: f64,
    /// Input signature: a bitmask of
    /// [`MjtCtrlInput`](crate::native_binding::wrappers::mj_model::MjtCtrlInput) values.
    pub ctrlspec: i32,
}
impl PidConfig {
    getter_setter! {
        with, [kp : f64; "the proportional (position) gain."; kv : f64;
        "the velocity feedback gain (mutually exclusive with dampratio)."; dampratio :
        f64; "the damping ratio (mutually exclusive with kv)."; ki : f64;
        "the integral gain on the position error."; imax : f64;
        "the anti-windup limit on the integral state."; slewmax : f64;
        "the position-setpoint slew rate limit."; inheritrange : f64;
        "the automatic range-inheritance factor."; ctrlspec : i32;
        "the input signature bitmask ([`MjtCtrlInput`](crate::native_binding::wrappers::mj_model::MjtCtrlInput)).";]
    }
}
/// Configuration for [`MjsActuator::set_to_orientation`].
///
/// Each optional field defaults to `None`, disabling the corresponding feature.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct OrientationConfig {
    /// Proportional gain, in torque per radian of geodesic error.
    pub kp: f64,
    /// Damping, per force output. Mutually exclusive with `dampratio`.
    pub kv: Option<f64>,
    /// Damping ratio. Mutually exclusive with `kv`.
    pub dampratio: Option<f64>,
    /// Chart of the commanded orientation
    /// ([`MjtCtrlChart`](crate::native_binding::wrappers::mj_model::MjtCtrlChart)).
    pub ctrlspec: i32,
}
impl OrientationConfig {
    getter_setter! {
        with, [kp : f64;
        "the proportional gain, in torque per radian of geodesic error."; kv : f64;
        "the velocity feedback gain (mutually exclusive with dampratio)."; dampratio :
        f64; "the damping ratio (mutually exclusive with kv)."; ctrlspec : i32;
        "the chart of the commanded orientation ([`MjtCtrlChart`](crate::native_binding::wrappers::mj_model::MjtCtrlChart)).";]
    }
}
impl MjsActuator {
    /// Configure the actuator to be a motor.
    pub fn set_to_motor(&mut self) {
        unsafe { mjs_setToMotor(self.ffi_mut()) };
    }
    /// Configure the actuator to be a positional-target motor (with a proportional regulator).
    /// # Errors
    /// Returns [`MjEditError::InvalidParameter`] when the configuration is rejected, e.g. `kv` and
    /// `dampratio` are both set, a value that must be non-negative is negative, or `inheritrange`
    /// is set together with a control range.
    pub fn set_to_position(&mut self, config: PositionConfig) -> Result<(), MjEditError> {
        let PositionConfig {
            kp,
            inheritrange,
            mut kv,
            mut dampratio,
            mut timeconst,
        } = config;
        let c_err_msg = unsafe {
            mjs_setToPosition(
                self.ffi_mut(),
                kp,
                kv.as_mut().map_or(ptr::null_mut(), |x| x),
                dampratio.as_mut().map_or(ptr::null_mut(), |x| x),
                timeconst.as_mut().map_or(ptr::null_mut(), |x| x),
                inheritrange,
            )
        };
        actuator_set_result(c_err_msg)
    }
    /// Configure the actuator to be an integrated-velocity servo. Behaves like
    /// [`MjsActuator::set_to_position`], but integrates the control signal into an activation
    /// variable.
    /// # Errors
    /// Returns [`MjEditError::InvalidParameter`] when `inheritrange` is set together with an
    /// activation range.
    pub fn set_to_int_velocity(&mut self, config: IntVelocityConfig) -> Result<(), MjEditError> {
        let IntVelocityConfig {
            kp,
            inheritrange,
            mut kv,
            mut dampratio,
            mut timeconst,
        } = config;
        let c_err_msg = unsafe {
            mjs_setToIntVelocity(
                self.ffi_mut(),
                kp,
                kv.as_mut().map_or(ptr::null_mut(), |x| x),
                dampratio.as_mut().map_or(ptr::null_mut(), |x| x),
                timeconst.as_mut().map_or(ptr::null_mut(), |x| x),
                inheritrange,
            )
        };
        actuator_set_result(c_err_msg)
    }
    /// Configure the actuator to be a velocity servo with velocity feedback gain `kv`.
    pub fn set_to_velocity(&mut self, kv: f64) {
        unsafe { mjs_setToVelocity(self.ffi_mut(), kv) };
    }
    /// Configure the actuator to be a damper with damping coefficient `kv`. The applied force is
    /// proportional to velocity and modulated by the (non-negative) control input.
    /// # Errors
    /// Returns [`MjEditError::InvalidParameter`] when `kv` is negative or the control range is
    /// negative.
    pub fn set_to_damper(&mut self, kv: f64) -> Result<(), MjEditError> {
        actuator_set_result(unsafe { mjs_setToDamper(self.ffi_mut(), kv) })
    }
    /// Configure the actuator to be a hydraulic or pneumatic cylinder. `timeconst` is the
    /// activation filter time constant, `bias` is added to the force, and the effective area is
    /// `area`; if `diameter` is non-negative the area is computed from it instead (pass a negative
    /// `diameter` to use `area` directly).
    pub fn set_to_cylinder(&mut self, timeconst: f64, bias: f64, area: f64, diameter: f64) {
        unsafe { mjs_setToCylinder(self.ffi_mut(), timeconst, bias, area, diameter) };
    }
    /// Configure the actuator to be a muscle. `timeconst` holds the activation and deactivation
    /// time constants, `range` the operating-length range, and the remaining scalars the muscle
    /// force-length-velocity parameters. A negative value for any array entry or scalar (except
    /// `tausmooth`) leaves the corresponding muscle default in place.
    /// # Errors
    /// Returns [`MjEditError::InvalidParameter`] when `tausmooth` is negative.
    #[allow(clippy::too_many_arguments)]
    pub fn set_to_muscle(
        &mut self,
        mut timeconst: [f64; 2],
        tausmooth: f64,
        mut range: [f64; 2],
        force: f64,
        scale: f64,
        lmin: f64,
        lmax: f64,
        vmax: f64,
        fpmax: f64,
        fvmax: f64,
    ) -> Result<(), MjEditError> {
        let c_err_msg = unsafe {
            mjs_setToMuscle(
                self.ffi_mut(),
                &mut timeconst,
                tausmooth,
                &mut range,
                force,
                scale,
                lmin,
                lmax,
                vmax,
                fpmax,
                fvmax,
            )
        };
        actuator_set_result(c_err_msg)
    }
    /// Configure the actuator to be an active-adhesion actuator with the given `gain`.
    /// # Errors
    /// Returns [`MjEditError::InvalidParameter`] when `gain` is negative or the control range is
    /// negative.
    pub fn set_to_adhesion(&mut self, gain: f64) -> Result<(), MjEditError> {
        actuator_set_result(unsafe { mjs_setToAdhesion(self.ffi_mut(), gain) })
    }
    /// Configure the actuator to be a DC motor.
    /// # Errors
    /// Returns [`MjEditError::InvalidParameter`] when MuJoCo cannot derive a positive motor
    /// constant or resistance, or when an inductance, thermal resistance, or thermal capacitance
    /// value is out of its allowed range.
    pub fn set_to_dc_motor(&mut self, config: DcMotorConfig) -> Result<(), MjEditError> {
        let DcMotorConfig {
            resistance,
            ctrlspec,
            mut motorconst,
            mut nominal,
            mut saturation,
            mut inductance,
            mut cogging,
            mut controller,
            mut thermal,
            mut lugre,
        } = config;
        let c_err_msg = unsafe {
            mjs_setToDCMotor(
                self.ffi_mut(),
                motorconst.as_mut().map_or(ptr::null_mut(), |x| x),
                resistance,
                nominal.as_mut().map_or(ptr::null_mut(), |x| x),
                saturation.as_mut().map_or(ptr::null_mut(), |x| x),
                inductance.as_mut().map_or(ptr::null_mut(), |x| x),
                cogging.as_mut().map_or(ptr::null_mut(), |x| x),
                controller.as_mut().map_or(ptr::null_mut(), |x| x),
                thermal.as_mut().map_or(ptr::null_mut(), |x| x),
                lugre.as_mut().map_or(ptr::null_mut(), |x| x),
                ctrlspec,
            )
        };
        actuator_set_result(c_err_msg)
    }
    /// Configure the actuator to be a PID controller on a single force output. The force is
    /// `kp * (u_pos - length) + kv * (u_vel - velocity) + ki * integral + ff`.
    /// # Errors
    /// Returns [`MjEditError::InvalidParameter`] when `kv` and `dampratio` are both set, when
    /// `kv`, `dampratio` or `slewmax` is negative, or when `inheritrange` is set together with a
    /// position-setpoint range.
    pub fn set_to_pid(&mut self, config: PidConfig) -> Result<(), MjEditError> {
        let PidConfig {
            kp,
            mut kv,
            mut dampratio,
            mut ki,
            mut imax,
            mut slewmax,
            inheritrange,
            ctrlspec,
        } = config;
        let c_err_msg = unsafe {
            mjs_setToPID(
                self.ffi_mut(),
                kp,
                kv.as_mut().map_or(ptr::null_mut(), |x| x),
                dampratio.as_mut().map_or(ptr::null_mut(), |x| x),
                ki.as_mut().map_or(ptr::null_mut(), |x| x),
                imax.as_mut().map_or(ptr::null_mut(), |x| x),
                slewmax.as_mut().map_or(ptr::null_mut(), |x| x),
                inheritrange,
                ctrlspec,
            )
        };
        actuator_set_result(c_err_msg)
    }
    /// Configure the actuator to be an orientation servo: a geodesic PD controller on a ball
    /// joint or a site with a reference site. The three force outputs carry the torque
    /// `kp * log(q^-1 * q_target) - kv * omega`, in the frame of the transmission target.
    /// # Errors
    /// Returns [`MjEditError::InvalidParameter`] when `kv` and `dampratio` are both set, or when
    /// `kv` or `dampratio` is negative.
    pub fn set_to_orientation(&mut self, config: OrientationConfig) -> Result<(), MjEditError> {
        let OrientationConfig {
            kp,
            mut kv,
            mut dampratio,
            ctrlspec,
        } = config;
        let c_err_msg = unsafe {
            mjs_setToOrientation(
                self.ffi_mut(),
                kp,
                kv.as_mut().map_or(ptr::null_mut(), |x| x),
                dampratio.as_mut().map_or(ptr::null_mut(), |x| x),
                ctrlspec,
            )
        };
        actuator_set_result(c_err_msg)
    }
}
mjs_struct!(Sensor with SpecObject : MjsSensor <= mjsSensor);
impl MjsSensor {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] intprm : & [i32; mjNSENS as usize];
        "integer parameters."; [ffi, ffi_mut] interval : & [f64; 2];
        "[period, time_prev] in seconds.";]
    }
    nested_handle!(plugin : MjsPluginReference; "sensor plugin.");
    getter_setter!(
        [&] with, get, set, [[ffi, ffi_mut] type_ + _ : MjtSensor; "sensor type."; [ffi,
        ffi_mut] objtype : MjtObj { check_objtype,
        "[`MjEditError::InvalidParameter`] when the object type is not a real object type (i.e. not below [`MjtObj::mjNOBJECT`])"
        } => MjEditError; "object type the sensor refers to."; [ffi, ffi_mut] reftype :
        MjtObj { check_objtype,
        "[`MjEditError::InvalidParameter`] when the reference type is not a real object type (i.e. not below [`MjtObj::mjNOBJECT`])"
        } => MjEditError; "type of referenced object."; [ffi, ffi_mut] datatype :
        MjtDataType; "data type."; [ffi, ffi_mut] cutoff : f64;
        "cutoff for real and positive datatypes."; [ffi, ffi_mut] noise : f64;
        "noise stdev."; [ffi, ffi_mut] needstage : MjtStage;
        "compute stage needed to simulate sensor."; [ffi, ffi_mut] dim : i32;
        "number of scalar outputs."; [ffi, ffi_mut] nsample : i32;
        "number of samples in history buffer."; [ffi, ffi_mut] interp : i32;
        "interpolation order (0=ZOH, 1=linear, 2=cubic)."; [ffi, ffi_mut] delay : f64;
        "delay time in seconds; 0: no delay.";]
    );
    userdata_method!(f64);
    string_set_get_with! {
        [&] refname; "name of referenced object."; objname; "name of sensorized object.";
    }
}
mjs_struct!(Flex with SpecObject : MjsFlex <= mjsFlex);
impl MjsFlex {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] rgba : & [f32; 4];
        "rgba when material is omitted."; [ffi, ffi_mut] friction : & [f64; 3];
        "one-sided friction coefficients: slide, spin, roll."; [ffi, ffi_mut] solref : &
        [MjtNum; mjNREF as usize]; "solver reference."; [ffi, ffi_mut] solimp : &
        [MjtNum; mjNIMP as usize]; "solver impedance."; [ffi, ffi_mut] size : & [f64; 3];
        "vertex bounding box half sizes in qpos0."; [ffi, ffi_mut] cellcount : & [i32;
        3]; "grid cell count for finite cell method.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] young : f64;
        "Young's modulus, in units of pressure (force/area)."; [ffi, ffi_mut] group :
        i32; "group."; [ffi, ffi_mut] contype : i32; "contact type."; [ffi, ffi_mut]
        conaffinity : i32; "contact affinity."; [ffi, ffi_mut] condim : i32;
        "contact dimensionality."; [ffi, ffi_mut] priority : i32; "contact priority.";
        [ffi, ffi_mut] solmix : f64; "solver mixing for contact pairs."; [ffi, ffi_mut]
        margin : f64; "margin for contact detection."; [ffi, ffi_mut] gap : f64;
        "additional contact detection buffer."; [ffi, ffi_mut] dim : i32;
        "element dimensionality."; [ffi, ffi_mut] radius : f64;
        "radius around primitive element."; [ffi, ffi_mut] activelayers : i32;
        "number of active element layers in 3D."; [ffi, ffi_mut] edgestiffness : f64;
        "edge stiffness."; [ffi, ffi_mut] edgedamping : f64; "edge damping."; [ffi,
        ffi_mut] poisson : f64; "Poisson's ratio."; [ffi, ffi_mut] damping : f64;
        "Rayleigh's damping."; [ffi, ffi_mut] thickness : f64; "thickness (2D only).";
        [ffi, ffi_mut] elastic2d : i32;
        "2D passive forces; 0: none, 1: bending, 2: stretching, 3: both."; [ffi, ffi_mut]
        order : i32; "interpolation order (1: trilinear, 2: quadratic).";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] internal : bool;
        "enable internal collisions."; [ffi, ffi_mut] flatskin : bool;
        "render flex skin with flat shading."; [ffi, ffi_mut] passive : bool;
        "mode for passive collisions.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] selfcollide : MjtFlexSelf[force];
        "mode for flex self collision.";]
    }
    string_set_get_with! {
        [&] material; "name of material used for rendering.";
    }
    vec_string_set_append! {
        nodebody; "node body names."; vertbody; "vertex body names.";
    }
    vec_set_get! {
        node : f64; "node positions."; vert : f64; "vertex positions.";
    }
    vec_set! {
        texcoord : f32; "vertex texture coordinates."; elem : i32; "element vertex ids.";
    }
    vec_set! {
        [unsafe :
        "The slice must have exactly `(dim + 1) * nelem` entries and every entry \
                  must be a valid index into the flex texture coordinates."]
        elemtexcoord : i32 => i32; "element texture coordinates.";
    }
}
mjs_struct!(Pair with SpecObject : MjsPair <= mjsPair);
impl MjsPair {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] friction : & [f64; 5];
        "contact friction: slide1, slide2, spin, roll1, roll2."; [ffi, ffi_mut] solref :
        & [MjtNum; mjNREF as usize]; "solver reference, normal direction."; [ffi,
        ffi_mut] solimp : & [MjtNum; mjNIMP as usize]; "solimp for the pair."; [ffi,
        ffi_mut] solreffriction : & [MjtNum; mjNREF as usize];
        "solver reference, frictional directions.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] margin : f64;
        "margin for contact detection."; [ffi, ffi_mut] gap : f64;
        "additional contact detection buffer."; [ffi, ffi_mut] adhesion : f64;
        "adhesive force of contacts."; [ffi, ffi_mut] condim : i32;
        "contact dimensionality.";]
    }
    string_set_get_with! {
        [&] geomname1; "name of geom 1."; geomname2; "name of geom 2.";
    }
}
mjs_struct!(Exclude with SpecObject : MjsExclude <= mjsExclude);
impl MjsExclude {
    string_set_get_with! {
        [&] bodyname1; "name of body 1."; bodyname2; "name of body 2.";
    }
}
mjs_struct!(Equality with SpecObject : MjsEquality <= mjsEquality);
impl MjsEquality {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] data : & [f64; mjNEQDATA as usize];
        "data array for equality parameters."; [ffi, ffi_mut] solref : & [f64; mjNREF as
        usize]; "solver reference."; [ffi, ffi_mut] solimp : & [f64; mjNIMP as usize];
        "solver impedance.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] active : bool; "active flag.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] type_ + _ : MjtEq; "equality type."; [ffi,
        ffi_mut] objtype : MjtObj; "type of both objects.";]
    }
    string_set_get_with! {
        [&] name1; "name of object 1"; name2; "name of object 2";
    }
}
mjs_struct!(Tendon with SpecObject : MjsTendon <= mjsTendon);
impl MjsTendon {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] damping : & [f64; mjNPOLY as usize + 1];
        "damping coefficients."; [ffi, ffi_mut] stiffness : & [f64; mjNPOLY as usize +
        1]; "stiffness coefficients."; [ffi, ffi_mut] springlength : & [f64; 2];
        "spring length."; [ffi, ffi_mut] solref_friction : & [f64; mjNREF as usize];
        "solver reference: tendon friction."; [ffi, ffi_mut] solimp_friction : & [f64;
        mjNIMP as usize]; "solver impedance: tendon friction."; [ffi, ffi_mut] range : &
        [f64; 2]; "range."; [ffi, ffi_mut] actfrcrange : & [f64; 2];
        "actuator force limits."; [ffi, ffi_mut] solref_limit : & [f64; mjNREF as usize];
        "solver reference: tendon limits."; [ffi, ffi_mut] solimp_limit : & [f64; mjNIMP
        as usize]; "solver impedance: tendon limits."; [ffi, ffi_mut] rgba : & [f32; 4];
        "rgba when material omitted.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] group : i32; "group."; [ffi, ffi_mut]
        frictionloss : f64; "friction loss."; [ffi, ffi_mut] armature : f64;
        "inertia associated with tendon velocity."; [ffi, ffi_mut] margin : f64;
        "margin value for tendon limit detection."; [ffi, ffi_mut] width : f64;
        "width for rendering.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] limited : MjtLimited[force];
        "does tendon have limits (mjtLimited)."; [ffi, ffi_mut] actfrclimited :
        MjtLimited[force]; "does tendon have actuator force limits."]
    }
    userdata_method!(f64);
    string_set_get_with! {
        [&] material; "name of material for rendering.";
    }
    /// Wrap a site corresponding to `name`, using the tendon.
    ///
    /// # Panics
    /// When the `name` contains '\0' characters.
    #[allow(deprecated)]
    pub fn wrap_site(&mut self, name: &str) -> &mut MjsWrap {
        self.try_wrap_site(name).expect("failed to wrap site")
    }
    /// Fallible version of [`MjsTendon::wrap_site`].
    ///
    /// # Note
    /// MuJoCo ends the process when the allocation fails, so this never returns `Err`.
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] if MuJoCo returns a null
    /// pointer.
    ///
    /// # Panics
    /// When the `name` contains '\0' characters.
    #[deprecated(since = "5.0.0", note = "always returns Ok; use `wrap_site`")]
    pub fn try_wrap_site(&mut self, name: &str) -> Result<&mut MjsWrap, MjEditError> {
        let cname = CString::new(name).unwrap();
        let wrap_ptr = unsafe { mjs_wrapSite(self.ffi_mut(), cname.as_ptr()) };
        unsafe { MjsWrap::from_ffi_ptr_mut(wrap_ptr) }.ok_or(MjEditError::AllocationFailed)
    }
    /// Wrap a geom corresponding to `name`, using the tendon.
    ///
    /// # Panics
    /// When `name` or `sidesite` contain '\0' characters.
    #[allow(deprecated)]
    pub fn wrap_geom(&mut self, name: &str, sidesite: &str) -> &mut MjsWrap {
        self.try_wrap_geom(name, sidesite)
            .expect("failed to wrap geom")
    }
    /// Fallible version of [`MjsTendon::wrap_geom`].
    ///
    /// # Note
    /// MuJoCo ends the process when the allocation fails, so this never returns `Err`.
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] if MuJoCo returns a null
    /// pointer.
    ///
    /// # Panics
    /// When `name` or `sidesite` contain '\0' characters.
    #[deprecated(since = "5.0.0", note = "always returns Ok; use `wrap_geom`")]
    pub fn try_wrap_geom(
        &mut self,
        name: &str,
        sidesite: &str,
    ) -> Result<&mut MjsWrap, MjEditError> {
        let cname = CString::new(name).unwrap();
        let csidesite = CString::new(sidesite).unwrap();
        let wrap_ptr = unsafe { mjs_wrapGeom(self.ffi_mut(), cname.as_ptr(), csidesite.as_ptr()) };
        unsafe { MjsWrap::from_ffi_ptr_mut(wrap_ptr) }.ok_or(MjEditError::AllocationFailed)
    }
    /// Wrap a joint corresponding to `name`, using the tendon.
    ///
    /// # Panics
    /// When `name` contains '\0' characters.
    #[allow(deprecated)]
    pub fn wrap_joint(&mut self, name: &str, coef: f64) -> &mut MjsWrap {
        self.try_wrap_joint(name, coef)
            .expect("failed to wrap joint")
    }
    /// Fallible version of [`MjsTendon::wrap_joint`].
    ///
    /// # Note
    /// MuJoCo ends the process when the allocation fails, so this never returns `Err`.
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] if MuJoCo returns a null
    /// pointer.
    ///
    /// # Panics
    /// When `name` contains '\0' characters.
    #[deprecated(since = "5.0.0", note = "always returns Ok; use `wrap_joint`")]
    pub fn try_wrap_joint(&mut self, name: &str, coef: f64) -> Result<&mut MjsWrap, MjEditError> {
        let cname = CString::new(name).unwrap();
        let wrap_ptr = unsafe { mjs_wrapJoint(self.ffi_mut(), cname.as_ptr(), coef) };
        unsafe { MjsWrap::from_ffi_ptr_mut(wrap_ptr) }.ok_or(MjEditError::AllocationFailed)
    }
    /// Wrap a pulley using the tendon.
    #[allow(deprecated)]
    pub fn wrap_pulley(&mut self, divisor: f64) -> &mut MjsWrap {
        self.try_wrap_pulley(divisor)
            .expect("failed to wrap pulley")
    }
    /// Fallible version of [`MjsTendon::wrap_pulley`].
    ///
    /// # Note
    /// MuJoCo ends the process when the allocation fails, so this never returns `Err`.
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] if MuJoCo returns a null
    /// pointer.
    #[deprecated(since = "5.0.0", note = "always returns Ok; use `wrap_pulley`")]
    pub fn try_wrap_pulley(&mut self, divisor: f64) -> Result<&mut MjsWrap, MjEditError> {
        let wrap_ptr = unsafe { mjs_wrapPulley(self.ffi_mut(), divisor) };
        unsafe { MjsWrap::from_ffi_ptr_mut(wrap_ptr) }.ok_or(MjEditError::AllocationFailed)
    }
    /// Return the number of wrap objects.
    pub fn wrap_num(&self) -> usize {
        unsafe { mjs_getWrapNum(self.ffi()) as usize }
    }
    /// Return an indexed wrap object.
    ///
    /// # Panics
    /// Panics if `i >= wrap_num()`. Use [`MjsTendon::try_wrap`] for a fallible alternative.
    pub fn wrap(&self, i: usize) -> &MjsWrap {
        self.try_wrap(i).unwrap()
    }
    /// Fallible version of [`MjsTendon::wrap`].
    ///
    /// # Errors
    /// Returns [`MjEditError::IndexOutOfBounds`] if `i >= wrap_num()`.
    pub fn try_wrap(&self, i: usize) -> Result<&MjsWrap, MjEditError> {
        let len = self.wrap_num();
        if i >= len {
            return Err(MjEditError::IndexOutOfBounds { id: i, len });
        }
        let ptr = unsafe { mjs_getWrap(self.ffi(), i as i32) };
        Ok(unsafe { MjsWrap::from_ffi_ptr(ptr) }.unwrap())
    }
    /// Return a mutable indexed wrap object.
    ///
    /// # Panics
    /// Panics if `i >= wrap_num()`. Use [`MjsTendon::try_wrap_mut`] for a fallible alternative.
    pub fn wrap_mut(&mut self, i: usize) -> &mut MjsWrap {
        self.try_wrap_mut(i).unwrap()
    }
    /// Fallible version of [`MjsTendon::wrap_mut`].
    ///
    /// # Errors
    /// Returns [`MjEditError::IndexOutOfBounds`] if `i >= wrap_num()`.
    pub fn try_wrap_mut(&mut self, i: usize) -> Result<&mut MjsWrap, MjEditError> {
        let len = self.wrap_num();
        if i >= len {
            return Err(MjEditError::IndexOutOfBounds { id: i, len });
        }
        let ptr = unsafe { mjs_getWrap(self.ffi(), i as i32) };
        Ok(unsafe { MjsWrap::from_ffi_ptr_mut(ptr) }.unwrap())
    }
}
mjs_struct!(
    MjsWrap <= mjsWrap { #[doc =
    " A wrap carries no name of its own; [`SpecItem::name`] reports the wrapped object's name."]
    #[doc = ""] #[doc = " # Errors"] #[doc =
    " Always returns [`MjEditError::UnsupportedOperation`]."] fn set_name(& mut self,
    _name : & str) -> Result < (), MjEditError > { Err(MjEditError::UnsupportedOperation)
    } #[doc = " A wrap carries no name of its own."] #[doc = ""] #[doc = " # Panics"]
    #[doc = " Always panics."] fn with_name(& mut self, _name : & str) -> & mut Self {
    panic!("a wrap carries no name of its own") } }
);
impl MjsWrap {
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] type_ + _ : MjtWrap; "wrap type.";]
    }
    /// Return the side site element. Returns `None` when the wrap is not a sphere or cylinder
    /// wrap, when it holds no side site, or when the named site is missing from the spec (MuJoCo
    /// logs a warning in that last case).
    ///
    /// There is no mutable counterpart, because it would alias. Edit the site through
    /// [`MjSpec::site_mut`] instead.
    pub fn side_site(&self) -> Option<&MjsSite> {
        let ptr = unsafe { mjs_getWrapSideSite(self.ffi()) };
        unsafe { MjsSite::from_ffi_ptr(ptr) }
    }
    /// Return the wrap divisor. For a wrap whose type is not [`MjtWrap::mjWRAP_PULLEY`], MuJoCo
    /// logs a warning and this returns 1.0.
    pub fn divisor(&self) -> f64 {
        unsafe { mjs_getWrapDivisor(self.ffi()) }
    }
    /// Return the wrap coefficient. For a wrap whose type is not [`MjtWrap::mjWRAP_JOINT`],
    /// MuJoCo logs a warning and this returns 1.0.
    pub fn coef(&self) -> f64 {
        unsafe { mjs_getWrapCoef(self.ffi()) }
    }
}
mjs_struct!(Numeric with SpecObject : MjsNumeric <= mjsNumeric);
impl MjsNumeric {
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] size : i32 { check_numeric_size,
        "[`MjEditError::InvalidParameter`] when the size is negative" } => MjEditError;
        "size of the numeric array.";]
    }
    vec_set_get! {
        data : f64; "initialization data.";
    }
}
mjs_struct!(Text with SpecObject : MjsText <= mjsText);
impl MjsText {
    string_set_get_with! {
        [&] data; "text string.";
    }
}
mjs_struct!(Tuple with SpecObject : MjsTuple <= mjsTuple);
impl MjsTuple {
    vec_set! {
        objtype : MjtObj => i32 { check_objtype,
        "[`MjEditError::InvalidParameter`] when any value is not a real object type (i.e. not below [`MjtObj::mjNOBJECT`])"
        } => MjEditError;
        "object types. Every value must be a real object type (an `MjtObj` below `mjNOBJECT`).";
    }
    vec_string_set_append! {
        objname; "object names.";
    }
    vec_set_get! {
        objprm : f64; "object parameters.";
    }
}
mjs_struct!(Key with SpecObject : MjsKey <= mjsKey);
impl MjsKey {
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] time : f64; "time."]
    }
    vec_set_get! {
        qpos : f64; "qpos."; qvel : f64; "qvel."; act : f64; "act."; mpos : f64;
        "mocap pos."; mquat : f64; "mocap quat."; ctrl : f64; "ctrl.";
    }
}
mjs_struct!(Plugin with SpecObject : MjsPlugin <= mjsPlugin);
mjs_opaque!(
    MjsPluginReference <= mjsPlugin,
    "Reference to the plugin instance that an element embeds.\n\n\
     A body, geom, mesh, actuator or sensor names the instance it uses through this reference. \
     The reference carries no element of its own: MuJoCo resolves the `element` field to the \
     [`MjsPlugin`] instance that the name selects. Reach the instance itself through \
     [`MjSpec::plugin`]."
);
impl MjsPluginReference {
    string_set_get_with! {
        [&] name; "instance name."; plugin_name; "plugin name.";
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] active : bool; "is the plugin active.";]
    }
}
impl MjsPlugin {
    string_set_get_with! {
        [&] name; "instance name."; plugin_name; "plugin name.";
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] active : bool; "is the plugin active.";]
    }
}
mjs_struct!(Mesh with SpecObject : MjsMesh <= mjsMesh);
impl MjsMesh {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] refpos : & [f64; 3]; "reference position."; [ffi,
        ffi_mut] refquat : & [f64; 4]; "reference orientation."; [ffi, ffi_mut] scale : &
        [f64; 3]; "scale vector.";]
    }
    nested_handle!(plugin : MjsPluginReference; "sdf plugin.");
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] inertia : MjtMeshInertia;
        "inertia type (convex, legacy, exact, shell)."; [ffi, ffi_mut] maxhullvert : i32;
        "maximum vertex count for the convex hull."; [ffi, ffi_mut] octree_maxdepth :
        i32; "max octree depth.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] smoothnormal : bool;
        "do not exclude large-angle faces from normals."; [ffi, ffi_mut] needsdf : bool;
        "compute sdf from mesh.";]
    }
    string_set_get_with! {
        [&] content_type; "content type of file."; file; "mesh file."; material;
        "name of material.";
    }
    vec_set! {
        uservert : f32; "user vertex data."; usernormal : f32; "user normal data.";
        usertexcoord : f32; "user texcoord data."; userface : i32;
        "user vertex indices.";
    }
    vec_set! {
        [unsafe :
        "Every entry must be in `0..N`, where `N` is the number of user normals: the \
                  length of the slice passed to `set_usernormal` divided by 3 (each normal is 3 \
                  `f32`: x, y, z)."]
        userfacenormal : i32 => i32; "user face normal indices."; [unsafe :
        "Every entry must be in `0..ntexcoord` (the number of user texture coordinates), and \
                  the slice length must equal the length of the slice passed to `set_userface` (3 per \
                  face). Unlike face-normal data, MuJoCo does not validate the texcoord-index length, \
                  so an oversized slice overflows the model's face-texcoord buffer at compile time."]
        userfacetexcoord : i32 => i32; "user texcoord indices.";
    }
}
mjs_struct!(HField with SpecObject : MjsHfield <= mjsHField);
impl MjsHfield {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] size : & [f64; 4]; "size of the hfield.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] nrow : i32; "number of rows."; [ffi, ffi_mut]
        ncol : i32; "number of columns.";]
    }
    string_set_get_with! {
        [&] content_type; "content type of file."; file;
        "file: (nrow, ncol, [elevation data]).";
    }
    /// Sets `userdata`.
    pub fn set_userdata<T: AsRef<[f32]>>(&mut self, userdata: T) {
        unsafe { write_mjs_vec_f32(userdata.as_ref(), self.ffi().userdata) };
    }
}
mjs_struct!(Skin with SpecObject : MjsSkin <= mjsSkin);
impl MjsSkin {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] rgba : & [f32; 4];
        "rgba when material is omitted.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] inflate : f32;
        "inflate in normal direction."; [ffi, ffi_mut] group : i32;
        "group for visualization.";]
    }
    string_set_get_with! {
        [&] material; "name of material used for rendering."; file; "skin file.";
    }
    vec_string_set_append! {
        bodyname; "body names.";
    }
    vec_set! {
        vert : f32; "vertex positions."; texcoord : f32; "texture coordinates."; bindpos
        : f32; "bind pos."; bindquat : f32; "bind quat.";
    }
    vec_set! {
        [unsafe :
        "The slice length must be a multiple of 3 and every entry must be in `0..nvert`  (the number of skin vertices)."]
        face : i32 => i32; "faces.";
    }
    vec_vec_append! {
        vertid : i32; "vertex ids."; vertweight : f32; "vertex weights.";
    }
}
mjs_struct!(Texture with SpecObject : MjsTexture <= mjsTexture);
/// # Note: cube-map files
///
/// `cubefiles` is a pre-sized string vector of 6 entries, one per cube face. Assign one face with
/// [`set_cubefile`](Self::set_cubefile); [`set_cubefiles`](Self::set_cubefiles) and
/// [`append_cubefiles`](Self::append_cubefiles) replace or extend the vector as a whole.
impl MjsTexture {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] rgb1 : & [f64; 3]; "first color for builtin.";
        [ffi, ffi_mut] rgb2 : & [f64; 3]; "second color for builtin."; [ffi, ffi_mut]
        markrgb : & [f64; 3]; "mark color."; [ffi, ffi_mut] gridsize : & [i32; 2];
        "size of grid for composite file; (1,1)-repeat."; [ffi, ffi_mut] gridlayout : &
        [c_char; 12]; "row-major: L,R,F,B,U,D for faces; . for unused.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] random : f64; "probability of random dots.";
        [ffi, ffi_mut] width : i32; "image width."; [ffi, ffi_mut] height : i32;
        "image height.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] nchannel : i32; "number of channels.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] type_ + _ : MjtTexture[force];
        "texture type."; [ffi, ffi_mut] colorspace : MjtColorSpace[force]; "colorspace.";
        [ffi, ffi_mut] builtin : MjtBuiltin[force]; "builtin type."; [ffi, ffi_mut] mark
        : MjtMark[force]; "mark type.";]
    }
    vec_string_set_append! {
        cubefiles[MjtCubeFace] => cubefile; "different file for each side of the cube.";
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] hflip : bool; "horizontal flip."; [ffi,
        ffi_mut] vflip : bool; "vertical flip.";]
    }
    /// Sets texture `data`.
    pub fn set_data<T: bytemuck::NoUninit>(&mut self, data: &[T]) {
        unsafe { write_mjs_vec_byte(data, self.ffi().data) };
    }
    string_set_get_with! {
        [&] file; "png file to load; use for all sides of cube."; content_type;
        "content type of file.";
    }
}
mjs_struct!(Material with SpecObject : MjsMaterial <= mjsMaterial);
/// # Note: texture assignment
///
/// `textures` is a pre-sized string vector of `mjNTEXROLE` entries, one per [`MjtTextureRole`].
/// Assign one role with [`set_texture`](Self::set_texture); [`set_textures`](Self::set_textures)
/// and [`append_textures`](Self::append_textures) replace or extend the vector as a whole and
/// break the pre-sized layout.
impl MjsMaterial {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] rgba : & [f32; 4]; "rgba color."; [ffi, ffi_mut]
        texrepeat : & [f32; 2]; "texture repetition for 2D mapping.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] texuniform : bool;
        "make texture cube uniform.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] emission : f32; "emission."; [ffi, ffi_mut]
        specular : f32; "specular."; [ffi, ffi_mut] shininess : f32; "shininess."; [ffi,
        ffi_mut] reflectance : f32; "reflectance."; [ffi, ffi_mut] metallic : f32;
        "metallic."; [ffi, ffi_mut] roughness : f32; "roughness.";]
    }
    vec_string_set_append! {
        textures[MjtTextureRole] => texture; "names of textures (empty: none).";
    }
}
mjs_struct!(Body with SpecObject : MjsBody <= mjsBody);
impl MjsBody {
    add_x_method! {
        body, site, joint, geom, camera, light
    }
    /// Obtain an immutable reference to a body with the given `name` in this body's subtree.
    /// The search is recursive and returns this body when its own name matches.
    ///
    /// # Panics
    /// When the `name` contains '\0' characters, a panic occurs.
    pub fn child(&self, name: &str) -> Option<&MjsBody> {
        let c_name = CString::new(name).unwrap();
        unsafe {
            let ptr = mjs_findChild(self.ffi(), c_name.as_ptr());
            MjsBody::from_ffi_ptr(ptr)
        }
    }
    /// Obtain a mutable reference to a body with the given `name` in this body's subtree.
    /// The search is recursive and returns this body when its own name matches.
    ///
    /// # Panics
    /// When the `name` contains '\0' characters, a panic occurs.
    ///
    /// # Examples
    /// ```
    /// # use mujoco_rs::prelude::*;
    /// let mut spec = MjSpec::new();
    /// spec.world_body_mut().add_body().with_name("ball");
    /// spec.world_body_mut().child_mut("ball").unwrap().set_gravcomp(1.0);
    /// ```
    pub fn child_mut(&mut self, name: &str) -> Option<&mut MjsBody> {
        let c_name = CString::new(name).unwrap();
        unsafe {
            let ptr = mjs_findChild(self.ffi(), c_name.as_ptr());
            MjsBody::from_ffi_ptr_mut(ptr)
        }
    }
    /// Add and return a child frame.
    ///
    /// # Note
    /// MuJoCo ends the process when the allocation fails.
    #[expect(
        deprecated,
        reason = "try_add_frame keeps the implementation until it is removed"
    )]
    pub fn add_frame(&mut self) -> &mut MjsFrame {
        self.try_add_frame()
            .expect("mjs_addFrame returned null; allocation failed")
    }
    /// Fallible version of [`Self::add_frame`].
    ///
    /// # Note
    /// MuJoCo ends the process when the allocation fails, so this never returns `Err`.
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] when MuJoCo fails to allocate
    /// the frame, instead of panicking.
    #[deprecated(since = "6.0.0", note = "always returns Ok; use `add_frame`")]
    pub fn try_add_frame(&mut self) -> Result<&mut MjsFrame, MjEditError> {
        let ptr = unsafe { mjs_addFrame(self.ffi_mut(), ptr::null_mut()) };
        unsafe { MjsFrame::from_ffi_ptr_mut(ptr) }.ok_or(MjEditError::AllocationFailed)
    }
}
/// Configuration for [`MjsBody::add_flexcomp`], mirroring the `flexcomp` element.
///
/// An unset array, string or VFS field reaches MuJoCo as null, so the compiler applies its own
/// default. `dim` is 2 and `radius` is 0.005; every other scalar is zero, which MuJoCo also
/// reads as its own default.
///
/// # Example
/// ```
/// # use mujoco_rs::prelude::*;
/// let mut spec = MjSpec::new();
///
/// // Configure a 3x3 cloth-like 2D grid flex.
/// let config = MjFlexcompConfig::default()
///     .with_type("grid")
///     .with_dim(2)
///     .with_count([3, 3, 1])
///     .with_spacing([0.1, 0.1, 0.1])
///     .with_mass(1.0);
///
/// let flex = spec.world_body_mut().add_flexcomp("cloth", &config);
/// assert_eq!(flex.dim(), 2);
///
/// // The configured flex is now part of the model and it compiles.
/// spec.compile().unwrap();
/// ```
#[derive(Debug, Clone)]
pub struct MjFlexcompConfig<'a> {
    /// Flexcomp type: "grid", "box", "cylinder", "ellipsoid", "square", "disc", "circle", "mesh",
    /// "gmsh" or "direct". MuJoCo falls back to "grid" for `None` and for any other string.
    pub r#type: Option<&'a str>,
    /// Dimensionality of the flex object (1, 2, or 3); ignored for types that imply it.
    /// Defaults to MuJoCo's default (2).
    pub dim: u8,
    /// Dof parametrization: "full", "radial", "trilinear", "quadratic", or "2d" (default "full").
    pub dof: Option<&'a str>,
    /// Number of generated points in each dimension (grid/box/cylinder/ellipsoid).
    pub count: Option<[u16; 3]>,
    /// Number of interpolation-grid cells in each dimension (trilinear/quadratic dofs).
    pub cellcount: Option<[u16; 3]>,
    /// Spacing between generated points in each dimension.
    pub spacing: Option<[f64; 3]>,
    /// Scaling of all point coordinates (applied after the pose transformation).
    pub scale: Option<[f64; 3]>,
    /// Radius of the flex elements. Defaults to MuJoCo's default (0.005).
    pub radius: f64,
    /// Total mass, divided evenly over the generated points. A value of 0 keeps MuJoCo's default.
    pub mass: f64,
    /// Equivalent-inertia box size used to set each body's rotational inertia. A value of 0 keeps
    /// MuJoCo's default.
    pub inertiabox: f64,
    /// Edge equality constraint: 0 none, 1 edge, 2 vertex, 3 strain.
    pub equality: u8,
    /// Whether all points are vertices in the parent body (no new bodies created).
    pub rigid: bool,
    /// Whether to render the flex skin with flat shading. MuJoCo forces this on for the "box" and
    /// "cylinder" types and for a 3D "grid".
    pub flatskin: bool,
    /// 2D passive force mode: 0 none, 1 bending, 2 stretching, 3 both.
    pub elastic2d: u8,
    /// Translation of all points relative to the parent body frame.
    pub pos: Option<[f64; 3]>,
    /// Quaternion rotation of all points around the position offset.
    pub quat: Option<[f64; 4]>,
    /// Flexcomp origin used to build a volumetric mesh from a surface mesh.
    pub origin: Option<[f64; 3]>,
    /// File to load the surface or volumetric mesh from.
    pub file: Option<&'a str>,
    /// Virtual file system used to resolve the mesh file.
    pub vfs: Option<&'a MjVfs>,
}
impl Default for MjFlexcompConfig<'_> {
    fn default() -> Self {
        Self {
            r#type: None,
            dim: 2,
            dof: None,
            count: None,
            cellcount: None,
            spacing: None,
            scale: None,
            radius: 0.005,
            mass: 0.0,
            inertiabox: 0.0,
            equality: 0,
            rigid: false,
            flatskin: false,
            elastic2d: 0,
            pos: None,
            quat: None,
            origin: None,
            file: None,
            vfs: None,
        }
    }
}
impl<'a> MjFlexcompConfig<'a> {
    getter_setter! {
        with, [r#type : &'a str;
        "the flexcomp type: \"grid\", \"box\", \"cylinder\", \"ellipsoid\", \"square\", \"disc\", \"circle\", \"mesh\", \"gmsh\", or \"direct\" (default \"grid\").";
        dim : u8;
        "the dimensionality of the flex object (1, 2, or 3); ignored for types that imply it.";
        dof : &'a str;
        "the dof parametrization: \"full\", \"radial\", \"trilinear\", \"quadratic\", or \"2d\" (default \"full\").";
        count : [u16; 3];
        "the number of generated points in each dimension (grid/box/cylinder/ellipsoid).";
        cellcount : [u16; 3];
        "the number of interpolation-grid cells in each dimension (trilinear/quadratic dofs).";
        spacing : [f64; 3]; "the spacing between generated points in each dimension.";
        scale : [f64; 3];
        "the scaling of all point coordinates (applied after the pose transformation).";
        radius : f64; "the radius of the flex elements."; mass : f64;
        "the total mass, divided evenly over the generated points."; inertiabox : f64;
        "the equivalent-inertia box size used to set each body's rotational inertia.";
        equality : u8;
        "the edge equality constraint: 0 none, 1 edge, 2 vertex, 3 strain."; rigid :
        bool;
        "whether all points are vertices in the parent body (no new bodies created).";
        flatskin : bool; "render flex skin with flat shading."; elastic2d : u8;
        "the 2D passive force mode: 0 none, 1 bending, 2 stretching, 3 both."; pos :
        [f64; 3]; "the translation of all points relative to the parent body frame.";
        quat : [f64; 4];
        "the quaternion rotation of all points around the position offset."; origin :
        [f64; 3];
        "the flexcomp origin used to build a volumetric mesh from a surface mesh."; file
        : &'a str; "the file to load the surface or volumetric mesh from."; vfs : &'a
        MjVfs; "the virtual file system used to resolve the mesh file.";]
    }
}
impl MjsBody {
    /// Add and return a child [`MjsFlex`].
    ///
    /// Creates a flex with auto-generated bodies, joints, and optional equality constraints, the
    /// programmatic equivalent of the `flexcomp` element, configured via
    /// [`MjFlexcompConfig`]. Wraps [`mjs_makeFlex`].
    ///
    /// # Panics
    /// Panics if MuJoCo fails to create the flex, or if `name` or any string in
    /// `config` contains an interior NUL byte.
    pub fn add_flexcomp(&mut self, name: &str, config: &MjFlexcompConfig) -> &mut MjsFlex {
        self.try_add_flexcomp(name, config)
            .expect("mjs_makeFlex returned null")
    }
    /// Fallible version of [`Self::add_flexcomp`]. Wraps [`mjs_makeFlex`].
    ///
    /// # Errors
    /// Returns [`MjEditError::AllocationFailed`] when MuJoCo fails to create the
    /// flex (returns null).
    ///
    /// # Panics
    /// Panics if `name` or any string in `config` contains an interior NUL byte.
    pub fn try_add_flexcomp(
        &mut self,
        name: &str,
        config: &MjFlexcompConfig,
    ) -> Result<&mut MjsFlex, MjEditError> {
        let c_name = CString::new(name).unwrap();
        let c_type = config.r#type.map(|s| CString::new(s).unwrap());
        let c_dof = config.dof.map(|s| CString::new(s).unwrap());
        let c_file = config.file.map(|s| CString::new(s).unwrap());
        let count = config.count.map(|c| c.map(|v| v as c_int));
        let cellcount = config.cellcount.map(|c| c.map(|v| v as c_int));
        let count_ptr = count
            .as_ref()
            .map_or(ptr::null(), |a| a as *const [c_int; 3]);
        let cellcount_ptr = cellcount
            .as_ref()
            .map_or(ptr::null(), |a| a as *const [c_int; 3]);
        let spacing_ptr = config
            .spacing
            .as_ref()
            .map_or(ptr::null(), |a| a as *const [f64; 3]);
        let scale_ptr = config
            .scale
            .as_ref()
            .map_or(ptr::null(), |a| a as *const [f64; 3]);
        let pos_ptr = config
            .pos
            .as_ref()
            .map_or(ptr::null(), |a| a as *const [f64; 3]);
        let quat_ptr = config
            .quat
            .as_ref()
            .map_or(ptr::null(), |a| a as *const [f64; 4]);
        let origin_ptr = config
            .origin
            .as_ref()
            .map_or(ptr::null(), |a| a as *const [f64; 3]);
        let ptr = unsafe {
            mjs_makeFlex(
                self.ffi_mut(),
                c_name.as_ptr(),
                c_type.as_ref().map_or(ptr::null(), |c| c.as_ptr()),
                config.dim as c_int,
                c_dof.as_ref().map_or(ptr::null(), |c| c.as_ptr()),
                count_ptr,
                cellcount_ptr,
                spacing_ptr,
                scale_ptr,
                config.radius,
                config.mass,
                config.inertiabox,
                config.equality as c_int,
                config.rigid as c_int,
                config.flatskin as c_int,
                config.elastic2d as c_int,
                pos_ptr,
                quat_ptr,
                origin_ptr,
                c_file.as_ref().map_or(ptr::null(), |c| c.as_ptr()),
                config.vfs.map_or(ptr::null(), |v| v.ffi() as *const mjVFS),
            )
        };
        unsafe { MjsFlex::from_ffi_ptr_mut(ptr) }.ok_or(MjEditError::AllocationFailed)
    }
}
impl MjsBody {
    getter_setter! {
        [&] with, get, [[ffi, ffi_mut] pos : & [f64; 3]; "frame position."; [ffi,
        ffi_mut] quat : & [f64; 4]; "frame orientation."; [ffi, ffi_mut] alt : &
        MjsOrientation; "frame alternative orientation."; [ffi, ffi_mut] ipos : & [f64;
        3]; "inertial frame position."; [ffi, ffi_mut] iquat : & [f64; 4];
        "inertial frame orientation."; [ffi, ffi_mut] inertia : & [f64; 3];
        "diagonal inertia (in i-frame)."; [ffi, ffi_mut] ialt : & MjsOrientation;
        "inertial frame alternative orientation."; [ffi, ffi_mut] fullinertia : & [f64;
        6]; "non-axis-aligned inertia matrix.";]
    }
    nested_handle!(plugin : MjsPluginReference; "passive force plugin.");
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] mass : f64; "mass."; [ffi, ffi_mut] gravcomp
        : f64; "gravity compensation."; [ffi, ffi_mut] sleep : MjtSleepPolicy;
        "sleep policy.";]
    }
    getter_setter! {
        [&] with, get, set, [[ffi, ffi_mut] mocap : bool;
        "whether this is a mocap body."; [ffi, ffi_mut] explicitinertial : bool;
        "whether to save the body with explicit inertial clause."; [ffi, ffi_mut] simple
        : bool; "simple body optimization (false: disabled, true: auto).";]
    }
    userdata_method!(f64);
}
/// Mutable iterator over items in [`MjsBody`].
#[derive(Debug)]
pub struct MjsBodyItemIterMut<'a, T> {
    /// Raw pointer to the body; a borrow would alias the handles that the iterator yields.
    ffi_ptr: *mut mjsBody,
    /// Element that the last `next` yielded. Null marks the end of the iteration.
    last: *mut mjsElement,
    recurse: bool,
    item_type: PhantomData<&'a mut T>,
}
impl<'a, T: SpecObject> MjsBodyItemIterMut<'a, T> {
    fn new(root: &'a mut MjsBody, recurse: bool) -> Self {
        let ffi_ptr = unsafe { root.ffi_mut() } as *mut mjsBody;
        let last = unsafe { mjs_firstChild(ffi_ptr, T::OBJ_TYPE, recurse.into()) };
        Self {
            ffi_ptr,
            last,
            recurse,
            item_type: PhantomData,
        }
    }
}
impl<'a, T: SpecObject + 'a> Iterator for MjsBodyItemIterMut<'a, T> {
    type Item = &'a mut T;
    fn next(&mut self) -> Option<Self::Item> {
        if self.last.is_null() {
            return None;
        }
        unsafe {
            let out = T::from_element_as_ptr_mut(self.last).as_mut();
            self.last = mjs_nextChild(self.ffi_ptr, self.last, self.recurse.into());
            out
        }
    }
}
impl<'a, T: SpecObject + 'a> std::iter::FusedIterator for MjsBodyItemIterMut<'a, T> {}
/// Immutable iterator over items in [`MjsBody`].
#[derive(Debug, Clone)]
pub struct MjsBodyItemIter<'a, T> {
    ffi_ptr: *const mjsBody,
    /// Element that the last `next` yielded. Null marks the end of the iteration.
    last: *const mjsElement,
    recurse: bool,
    item_type: PhantomData<&'a T>,
}
impl<'a, T: SpecObject> MjsBodyItemIter<'a, T> {
    fn new(root: &'a MjsBody, recurse: bool) -> Self {
        let ffi_ptr = root.ffi() as *const mjsBody;
        let last = unsafe { mjs_firstChild(ffi_ptr, T::OBJ_TYPE, recurse.into()) };
        Self {
            ffi_ptr,
            last,
            recurse,
            item_type: PhantomData,
        }
    }
}
impl<'a, T: SpecObject + 'a> Iterator for MjsBodyItemIter<'a, T> {
    type Item = &'a T;
    fn next(&mut self) -> Option<Self::Item> {
        if self.last.is_null() {
            return None;
        }
        unsafe {
            let out = T::from_element_as_ptr_mut(self.last as *mut _).as_ref();
            self.last = mjs_nextChild(self.ffi_ptr, self.last, self.recurse.into());
            out
        }
    }
}
impl<'a, T: SpecObject + 'a> std::iter::FusedIterator for MjsBodyItemIter<'a, T> {}
/// Iterator methods.
impl MjsBody {
    body_get_iter! {
        [joint, geom, site, camera, light, frame]
    }
    body_get_iter! {
        direct_children_mut : [body]
    }
}
