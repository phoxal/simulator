//! Thread-owned, windowless macOS context for MuJoCo's compatibility renderer.

use std::{marker::PhantomData, ptr, rc::Rc};

/// CGL contexts are thread-local. This owner cannot move between threads.
#[derive(Debug)]
pub(super) struct Context {
    raw: cgl::CGLContextObj,
    _thread: PhantomData<Rc<()>>,
}

impl Context {
    pub(super) fn new() -> Result<Self, String> {
        // MuJoCo requires the default legacy profile. Let CGL select an ordinary
        // compatible renderer, including software when acceleration is unavailable.
        let attributes = [
            cgl::kCGLPFAColorSize,
            24,
            cgl::kCGLPFAAlphaSize,
            8,
            cgl::kCGLPFADepthSize,
            24,
            cgl::kCGLPFAStencilSize,
            8,
            0,
        ];
        let mut format = ptr::null_mut();
        let mut count = 0;
        // SAFETY: the attributes are terminated and both outputs point to valid storage.
        check("CGLChoosePixelFormat", unsafe {
            cgl::CGLChoosePixelFormat(attributes.as_ptr(), &mut format, &mut count)
        })?;
        if format.is_null() {
            return Err("CGL returned no compatible pixel format".into());
        }
        let mut raw = ptr::null_mut();
        // SAFETY: format is an owned, valid CGL pixel format; no context is shared.
        let status = unsafe { cgl::CGLCreateContext(format, ptr::null_mut(), &mut raw) };
        // SAFETY: creation has finished using the pixel format.
        unsafe { cgl::CGLReleasePixelFormat(format) };
        check("CGLCreateContext", status)?;
        if raw.is_null() {
            return Err("CGL returned no rendering context".into());
        }
        Ok(Self {
            raw,
            _thread: PhantomData,
        })
    }

    /// Restore the caller's context after each operation, including on error.
    pub(super) fn enter(&self) -> Result<Current<'_>, String> {
        // SAFETY: this owner remains on its creation thread and raw is still alive.
        let previous = unsafe { cgl::CGLGetCurrentContext() };
        // SAFETY: raw is a valid context exclusively owned on this thread.
        check("CGLSetCurrentContext", unsafe {
            cgl::CGLSetCurrentContext(self.raw)
        })?;
        Ok(Current {
            previous,
            _owner: self,
        })
    }
}

pub(super) struct Current<'a> {
    previous: cgl::CGLContextObj,
    _owner: &'a Context,
}

impl Drop for Current<'_> {
    fn drop(&mut self) {
        // SAFETY: the previous thread-local context was not released by this owner.
        unsafe { cgl::CGLSetCurrentContext(self.previous) };
    }
}

impl Drop for Context {
    fn drop(&mut self) {
        // SAFETY: all borrowed current guards have ended, and raw is released exactly once.
        unsafe { cgl::CGLDestroyContext(self.raw) };
    }
}

fn check(phase: &str, status: cgl::CGLError) -> Result<(), String> {
    if status == cgl::kCGLNoError {
        Ok(())
    } else {
        // SAFETY: CGL returns a static diagnostic string for its error code.
        let description = unsafe { cgl::CGLErrorString(status) };
        let description = if description.is_null() {
            "unknown CGL error".into()
        } else {
            // SAFETY: a non-null CGLErrorString result is a NUL-terminated static string.
            unsafe { std::ffi::CStr::from_ptr(description) }.to_string_lossy()
        };
        Err(format!(
            "{phase} failed with CGL code {status}: {description}"
        ))
    }
}

#[cfg(all(test, native_qualification))]
mod tests {
    use super::*;

    #[test]
    fn ordinary_context_restores_current_after_native_step_render_error_and_drop() {
        // SAFETY: reading this thread's current context does not change ownership.
        let original = unsafe { cgl::CGLGetCurrentContext() };
        {
            let caller = Context::new().unwrap();
            let _current = caller.enter().unwrap();
            let model = crate::mujoco::Model::from_file(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("src/mujoco/tests/fixtures/rendering/scene.xml"),
            )
            .unwrap();
            let camera = model.camera("camera").unwrap().unwrap();
            let mut workspace = crate::mujoco::Workspace::new(&model).unwrap();
            for _ in 0..10 {
                workspace.step().unwrap();
                let frame = workspace.render_camera(camera).unwrap();
                assert_eq!(frame.resolution(), [4, 3]);
                // SAFETY: caller remains owned and current on this thread.
                assert_eq!(unsafe { cgl::CGLGetCurrentContext() }, caller.raw);
            }
            assert!(
                workspace
                    .render_viewport(workspace.default_view_camera(), [0, 1])
                    .is_err()
            );
            // SAFETY: renderer refusal must preserve caller's current context.
            assert_eq!(unsafe { cgl::CGLGetCurrentContext() }, caller.raw);
            drop(workspace);
            // SAFETY: renderer destruction must preserve the still-owned caller.
            assert_eq!(unsafe { cgl::CGLGetCurrentContext() }, caller.raw);
            let mut accelerated = 0;
            // SAFETY: pixel format is borrowed from the still-live caller, output is writable.
            check("CGLDescribePixelFormat", unsafe {
                cgl::CGLDescribePixelFormat(
                    cgl::CGLGetPixelFormat(caller.raw),
                    0,
                    cgl::kCGLPFAAccelerated,
                    &mut accelerated,
                )
            })
            .unwrap();
            eprintln!("ORDINARY_CGL_SELECTED accelerated={accelerated}");
        }
        // SAFETY: guards and owned contexts have ended; the original is not released by them.
        assert_eq!(unsafe { cgl::CGLGetCurrentContext() }, original);
    }

    #[test]
    fn error_diagnostic_identifies_phase_and_bad_pixel_format() {
        for phase in [
            "CGLChoosePixelFormat",
            "CGLCreateContext",
            "CGLSetCurrentContext",
        ] {
            // kCGLBadPixelFormat in Apple CGLError.h; cgl 0.3.2 does not export it.
            let error = check(phase, 10002).unwrap_err();
            assert!(error.starts_with(phase));
            assert!(error.contains("10002"));
        }
    }
}
