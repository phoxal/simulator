//! Module implements [`MjsDefault`], which is a special type of [`SpecItem`].
use super::traits::SpecItem;
use crate::native_binding::error::MjEditError;
use crate::native_binding::mujoco_c::*;
use crate::native_binding::wrappers::mj_editing::{
    MjsActuator, MjsCamera, MjsEquality, MjsFlex, MjsGeom, MjsJoint, MjsLight, MjsMaterial,
    MjsMesh, MjsPair, MjsSite, MjsTendon,
};
macro_rules! default_accessor_wrapper {
    ($($name:ident),*) => {
        paste::paste! { $(#[doc = concat!("Returns an immutable reference to ",
        stringify!($name), "'s defaults.")] pub fn $name (& self) -> & [< Mjs $name :
        camel >] { unsafe { [< Mjs $name : camel >] ::from_ffi_ptr(self.ffi().$name) }
        .unwrap() } #[doc = concat!("Returns a mutable reference to ", stringify!($name),
        "'s defaults.")] pub fn [<$name _mut >] (& mut self) -> & mut [< Mjs $name :
        camel >] { unsafe { [< Mjs $name : camel >] ::from_ffi_ptr_mut(self.ffi().$name)
        } .unwrap() })* }
    };
}
mjs_opaque!(
    MjsDefault <= mjsDefault,
    "Default specification. An opaque handle for the FFI type [`mjsDefault`], reached through \
[`ffi`](Self::ffi)."
);
impl MjsDefault {
    default_accessor_wrapper! {
        joint, geom, site, camera, light, flex, mesh, material, pair, equality, tendon,
        actuator
    }
}
impl super::traits::sealed::Sealed for MjsDefault {}
impl SpecItem for MjsDefault {
    fn element_pointer(&self) -> *const mjsElement {
        self.ffi().element
    }
    fn default(&self) -> Option<&MjsDefault> {
        Some(self)
    }
    /// A default class carries no id. Always returns `None`.
    fn id(&self) -> Option<usize> {
        None
    }
    /// A default class cannot be assigned to another default class.
    ///
    /// # Errors
    /// Always returns [`MjEditError::UnsupportedOperation`].
    fn set_default(&mut self, _class_name: &str) -> Result<(), MjEditError> {
        Err(MjEditError::UnsupportedOperation)
    }
    /// A default class cannot be assigned to another default class.
    ///
    /// # Errors
    /// Always returns [`MjEditError::UnsupportedOperation`].
    fn with_default(&mut self, _class_name: &str) -> Result<&mut Self, MjEditError> {
        Err(MjEditError::UnsupportedOperation)
    }
}
