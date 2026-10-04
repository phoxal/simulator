//! Definition of MjOption.
use crate::native_binding::mujoco_c::{mj_defaultOption, mjOption};
/// Simulation options (timestep, integrator, flags, etc.).
pub type MjOption = mjOption;
impl Default for MjOption {
    fn default() -> Self {
        unsafe {
            let mut opt = std::mem::MaybeUninit::uninit();
            mj_defaultOption(opt.as_mut_ptr());
            opt.assume_init()
        }
    }
}
