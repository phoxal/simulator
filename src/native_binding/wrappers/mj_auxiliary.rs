//! MuJoCo's auxiliary structs.
use crate::native_binding::error::MjVfsError;
use crate::native_binding::mujoco_c::*;
use std::ffi::{CString, c_void};
use std::mem::MaybeUninit;
use std::path::Path;
use std::ptr;
/// Visual properties of the model (headlight, rgba defaults, scale, etc.).
pub type MjVisual = mjVisual;
impl Default for MjVisual {
    fn default() -> Self {
        unsafe {
            let mut s = MaybeUninit::uninit();
            mj_defaultVisual(s.as_mut_ptr());
            s.assume_init()
        }
    }
}
/// Model statistics (center, extent, mean body mass, mean inertia, etc.).
pub type MjStatistic = mjStatistic;
/// Contact parameters set by narrowphase collision functions.
pub type MjPreContact = mjPreContact;
/// Contact point data (position, frame, friction/solver parameters, geom/flex ids, etc.).
pub type MjContact = mjContact;
unsafe impl bytemuck::Zeroable for mjContact_ {}
/// Collision callback type.
pub type MjfCollision = mjfCollision;
/// Resource provider callbacks and opaque provider data.
pub type MjpResourceProvider = mjpResourceProvider;
/// Options for the length-range computation of actuator length ranges.
pub type MjLROpt = mjLROpt;
impl Default for MjLROpt {
    fn default() -> Self {
        unsafe {
            let mut s = MaybeUninit::uninit();
            mj_defaultLROpt(s.as_mut_ptr());
            s.assume_init()
        }
    }
}
/// Wrapper around the virtual-file system.
#[derive(Debug)]
pub struct MjVfs {
    ffi: Box<mjVFS>,
}
impl MjVfs {
    /// Creates a new, empty virtual file system.
    pub fn new() -> Self {
        unsafe {
            let mut maybe_uninit = Box::new_uninit();
            mj_defaultVFS(maybe_uninit.as_mut_ptr());
            Self {
                ffi: maybe_uninit.assume_init(),
            }
        }
    }
    /// Adds a file from disk to the virtual file system, searching in `directory`.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjVfsError::InvalidUtf8Path`] if `directory` or `filename` contains invalid UTF-8.
    /// - [`MjVfsError::AlreadyExists`] if a file with the same name already exists in the VFS.
    /// - [`MjVfsError::LoadFailed`] if the file could not be loaded.
    /// - [`MjVfsError::Unknown`] for unrecognized MuJoCo return codes.
    /// # Panics
    /// When `directory` or `filename` contain interior `\0` characters.
    pub fn add_file_from<T: AsRef<Path>, U: AsRef<Path>>(
        &mut self,
        directory: T,
        filename: U,
    ) -> Result<(), MjVfsError> {
        let c_directory = CString::new(
            directory
                .as_ref()
                .to_str()
                .ok_or(MjVfsError::InvalidUtf8Path)?,
        )
        .unwrap();
        let c_filename = CString::new(
            filename
                .as_ref()
                .to_str()
                .ok_or(MjVfsError::InvalidUtf8Path)?,
        )
        .unwrap();
        Self::handle_add_result(unsafe {
            mj_addFileVFS(self.ffi_mut(), c_directory.as_ptr(), c_filename.as_ptr())
        })
    }
    /// Adds a file from disk to the virtual file system.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjVfsError::InvalidUtf8Path`] if `filename` contains invalid UTF-8.
    /// - [`MjVfsError::AlreadyExists`] if a file with the same name already exists in the VFS.
    /// - [`MjVfsError::LoadFailed`] if the file could not be loaded.
    /// - [`MjVfsError::Unknown`] for unrecognized MuJoCo return codes.
    /// # Panics
    /// When `filename` contains interior `\0` characters.
    pub fn add_file<T: AsRef<Path>>(&mut self, filename: T) -> Result<(), MjVfsError> {
        let c_filename = CString::new(
            filename
                .as_ref()
                .to_str()
                .ok_or(MjVfsError::InvalidUtf8Path)?,
        )
        .unwrap();
        Self::handle_add_result(unsafe {
            mj_addFileVFS(self.ffi_mut(), ptr::null(), c_filename.as_ptr())
        })
    }
    /// Adds a file to the virtual file system from a byte buffer.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjVfsError::InvalidUtf8Path`] if `filename` contains invalid UTF-8.
    /// - [`MjVfsError::AlreadyExists`] if a file with the same name already exists in the VFS.
    /// - [`MjVfsError::LoadFailed`] if MuJoCo fails to register the buffer.
    /// - [`MjVfsError::BufferTooLarge`] if `buffer` exceeds `i32::MAX` bytes.
    /// - [`MjVfsError::Unknown`] for unrecognized MuJoCo return codes.
    /// # Panics
    /// When the `filename` contains interior `\0` characters.
    pub fn add_from_buffer<T: AsRef<Path>>(
        &mut self,
        filename: T,
        buffer: &[u8],
    ) -> Result<(), MjVfsError> {
        let c_filename = CString::new(
            filename
                .as_ref()
                .to_str()
                .ok_or(MjVfsError::InvalidUtf8Path)?,
        )
        .unwrap();
        let nbuffer = i32::try_from(buffer.len()).map_err(|_| MjVfsError::BufferTooLarge)?;
        Self::handle_add_result(unsafe {
            mj_addBufferVFS(
                self.ffi_mut(),
                c_filename.as_ptr(),
                buffer.as_ptr() as *const c_void,
                nbuffer,
            )
        })
    }
    /// Removes a file from the virtual file system.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjVfsError::InvalidUtf8Path`] if `filename` contains invalid UTF-8.
    /// - [`MjVfsError::NotFound`] if the file doesn't exist.
    /// - [`MjVfsError::Unknown`] for unrecognized MuJoCo return codes.
    /// # Panics
    /// When the `filename` contains interior `\0` characters.
    pub fn delete_file<T: AsRef<Path>>(&mut self, filename: T) -> Result<(), MjVfsError> {
        let c_filename = CString::new(
            filename
                .as_ref()
                .to_str()
                .ok_or(MjVfsError::InvalidUtf8Path)?,
        )
        .unwrap();
        unsafe { Self::handle_remove_result(mj_deleteFileVFS(self.ffi_mut(), c_filename.as_ptr())) }
    }
    /// Check if file exists in VFS. MuJoCo keeps only the last path element of
    /// `directory`/`name` for the lookup, so `directory` does not restrict the search.
    ///
    /// A mutable borrow is required due to the internal mutex.
    ///
    /// # Panics
    /// When `name` or `directory` contain path with null elements.
    pub fn contains_file_in(
        &mut self,
        directory: impl AsRef<Path>,
        name: impl AsRef<Path>,
    ) -> bool {
        let c_name = if let Some(name) = name.as_ref().to_str() {
            CString::new(name).unwrap()
        } else {
            return false;
        };
        let directory = if let Some(directory) = directory.as_ref().to_str() {
            CString::new(directory).unwrap()
        } else {
            return false;
        };
        unsafe { mj_containsFileVFS(self.ffi_mut(), directory.as_ptr(), c_name.as_ptr()) != 0 }
    }
    /// Check if file exists in VFS.
    ///
    /// A mutable borrow is required due to the internal mutex.
    ///
    /// # Panics
    /// When `name` contains path with null elements.
    pub fn contains_file<T: AsRef<Path>>(&mut self, name: T) -> bool {
        let c_name = if let Some(name) = name.as_ref().to_str() {
            CString::new(name).unwrap()
        } else {
            return false;
        };
        unsafe { mj_containsFileVFS(self.ffi_mut(), ptr::null(), c_name.as_ptr()) != 0 }
    }
    fn handle_add_result(result: i32) -> Result<(), MjVfsError> {
        match result {
            0 => Ok(()),
            2 => Err(MjVfsError::AlreadyExists),
            -1 => Err(MjVfsError::LoadFailed),
            code => Err(MjVfsError::Unknown(code)),
        }
    }
    fn handle_remove_result(result: i32) -> Result<(), MjVfsError> {
        match result {
            0 => Ok(()),
            -1 => Err(MjVfsError::NotFound),
            code => Err(MjVfsError::Unknown(code)),
        }
    }
    /// Reference to the wrapped FFI struct.
    pub fn ffi(&self) -> &mjVFS {
        &self.ffi
    }
    /// Mutable reference to the wrapped FFI struct.
    ///
    /// # Safety
    /// Modifying the underlying FFI struct directly can break the invariants
    /// upheld by the `mujoco-rs` wrappers and cause undefined behavior.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjVFS {
        &mut self.ffi
    }
}
impl Default for MjVfs {
    fn default() -> Self {
        Self::new()
    }
}
unsafe impl Send for MjVfs {}
unsafe impl Sync for MjVfs {}
impl Drop for MjVfs {
    fn drop(&mut self) {
        unsafe {
            mj_deleteVFS(self.ffi.as_mut());
        }
    }
}
