//! Utility types and macros used throughout the crate.
use crate::native_binding::mujoco_c::{mj_version, mjVERSION_HEADER};
use std::any::type_name;
use std::ffi::{CString, c_char};
use std::sync::{Mutex, MutexGuard};
use std::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
};
/// Standard size of temporary error buffers passed to MuJoCo C functions.
/// MuJoCo NUL-terminates within this size, so the effective maximum
/// message length is `ERROR_BUF_LEN - 1` characters.
pub(crate) const ERROR_BUF_LEN: usize = 100;
/// Copies an ASCII `&str` into a fixed-size `c_char` buffer, NUL-terminating and
/// zero-filling the remainder.
///
/// # Panics
/// Panics if `value` is not valid ASCII, contains an interior NUL byte,
/// or if `value` (plus NUL) does not fit in `buf`.
pub(crate) fn write_ascii_to_buf(buf: &mut [c_char], value: &str) {
    assert!(value.is_ascii(), "value must be valid ASCII");
    let c_string = std::ffi::CString::new(value).unwrap();
    let bytes = c_string.into_bytes_with_nul();
    let dest: &mut [u8] = bytemuck::cast_slice_mut(buf);
    dest[..bytes.len()].copy_from_slice(&bytes);
    dest[bytes.len()..].fill(0);
}
/// Returns `msg` as a C string that escapes every `%`, so it is sound to pass as the format
/// string of a `printf`-family C function (`mju_error`, `mju_warning`, `mju_info`).
///
/// # Panics
/// Panics if `msg` contains interior `\0` characters.
pub(crate) fn printf_safe_cstring(msg: &str) -> CString {
    CString::new(msg.replace('%', "%%")).unwrap()
}
/// Returns `Some((start, len))` for item `id` inside a packed data array,
/// or `None` if the item has no data (address entry is negative).
///
/// Each entry in `addr_array` is either the item's start offset in the data array
/// or `-1`, meaning the item has no associated data.
///
/// The length is determined by scanning `addr_array[id + 1..]` for the first entry that is not
/// `-1` (the next item that *does* have data). If no such entry exists, the region extends to the
/// end of the data array (`data_len`).
///
/// # Examples
/// ```ignore
/// if let Some((graph_adr, graph_len)) = optional_sparse_addr_range(
///     model.mesh_graphadr(), mesh_id, model.mesh_graph().len()
/// ) {
///     // copy mesh_graph[graph_adr..graph_adr + graph_len]
/// }
/// ```
///
/// # Panics
/// Panics if `id >= addr_array.len()`.
pub(crate) fn optional_sparse_addr_range<T>(
    addr_array: &[T],
    id: usize,
    data_len: usize,
) -> Option<(usize, usize)>
where
    T: Into<i64> + Copy,
{
    let adr: i64 = addr_array[id].into();
    if adr < 0 {
        return None;
    }
    let adr = adr as usize;
    let len = addr_array
        .get(id + 1..)
        .unwrap_or(&[])
        .iter()
        .find(|&&next| next.into() != -1)
        .map(|&next| next.into() as usize)
        .unwrap_or(data_len)
        - adr;
    Some((adr, len))
}
/// Converts a buffer length into the element count type `T` that MuJoCo takes.
///
/// # Panics
/// Panics if `len` does not fit in `T`.
pub(crate) fn checked_c_len<T: TryFrom<usize>>(len: usize) -> T {
    T::try_from(len).unwrap_or_else(|_| {
        panic!(
            "length {len} exceeds the MuJoCo {} element count",
            type_name::<T>()
        )
    })
}
/// Offsets a raw array pointer, keeping a null pointer null.
///
/// A plain `add` turns null into a dangling address, which defeats the empty-slice guard that
/// every `PointerView` applies to a null pointer.
///
/// # Safety
/// When `ptr` is not null, `ptr.add(offset)` must stay inside the same allocation.
pub(crate) unsafe fn offset_or_null<T>(ptr: *mut T, offset: usize) -> *mut T {
    if ptr.is_null() {
        ptr
    } else {
        unsafe { ptr.add(offset) }
    }
}
/// Sets or clears a bit flag based on a boolean value.
///
/// # Examples
/// ```ignore
/// let mut flags = 0i32;
/// set_flag!(flags, 0x01, true);   // sets bit 0
/// set_flag!(flags, 0x01, false);  // clears bit 0
/// ```
#[doc(hidden)]
#[macro_export]
macro_rules! set_flag {
    ($flags:expr, $mask:expr, $enabled:expr) => {
        if $enabled {
            $flags |= $mask;
        } else {
            $flags &= !$mask;
        }
    };
}
/// Resolves the `(start, len)` pair of the packed range that element `$id` owns; the `nX` token
/// (`nq`, `nv`, `nu`, `nout`, `na`, `nsensordata`, ...) selects both the address array and the
/// total length of the data array. Returns `(0, 0)` when the element's address entry is `-1`, and
/// ends the range at the total length when no later element owns data.
#[macro_export]
#[doc(hidden)]
macro_rules! mj_model_dyn_range {
    ($model:expr, $id:expr, nq) => {
        $crate::native_binding::util::optional_sparse_addr_range(
            $model.jnt_qposadr(),
            $id,
            $model.nq() as usize,
        )
        .unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, nv) => {
        $crate::native_binding::util::optional_sparse_addr_range(
            $model.jnt_dofadr(),
            $id,
            $model.nv() as usize,
        )
        .unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, nsensordata) => {
        $crate::native_binding::util::optional_sparse_addr_range(
            $model.sensor_adr(),
            $id,
            $model.nsensordata() as usize,
        )
        .unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, ntupledata) => {
        $crate::native_binding::util::optional_sparse_addr_range(
            $model.tuple_adr(),
            $id,
            $model.ntupledata() as usize,
        )
        .unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, ntexdata) => {
        $crate::native_binding::util::optional_sparse_addr_range(
            $model.tex_adr(),
            $id,
            $model.ntexdata() as usize,
        )
        .unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, nnumericdata) => {
        $crate::native_binding::util::optional_sparse_addr_range(
            $model.numeric_adr(),
            $id,
            $model.nnumericdata() as usize,
        )
        .unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, nhfielddata) => {
        $crate::native_binding::util::optional_sparse_addr_range(
            $model.hfield_adr(),
            $id,
            $model.nhfielddata() as usize,
        )
        .unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, na) => {
        $crate::native_binding::util::optional_sparse_addr_range(
            $model.actuator_actadr(),
            $id,
            $model.na() as usize,
        )
        .unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, nu) => {
        $crate::native_binding::util::optional_sparse_addr_range(
            $model.actuator_ctrladr(),
            $id,
            $model.nu() as usize,
        )
        .unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, nout) => {
        $crate::native_binding::util::optional_sparse_addr_range(
            $model.actuator_outadr(),
            $id,
            $model.nout() as usize,
        )
        .unwrap_or((0, 0))
    };
    ($model:expr, $id:expr, nJten) => {
        $crate::native_binding::util::optional_sparse_addr_range(
            $model.ten_j_rowadr(),
            $id,
            $model.n_jten() as usize,
        )
        .unwrap_or((0, 0))
    };
}
/// Provides a more direct view to a C array.
/// # Safety
/// This does not check if the data is valid. It is assumed
/// the correct data is given and that it doesn't get dropped before this struct.
/// This does not break Rust's checks as we create the view each
/// time from the saved pointers.
/// This should ONLY be used within a wrapper that fully encapsulates the underlying data.
#[derive(Debug)]
pub struct PointerViewMut<'d, T> {
    ptr: *mut T,
    len: usize,
    phantom: PhantomData<&'d mut ()>,
}
impl<'d, T> PointerViewMut<'d, T> {
    pub(crate) const fn new(ptr: *mut T, len: usize, phantom: PhantomData<&'d mut ()>) -> Self {
        Self { ptr, len, phantom }
    }
}
/// Compares if the two views point to the same data with the same length.
impl<T> PartialEq for PointerViewMut<'_, T> {
    fn eq(&self, other: &Self) -> bool {
        self.ptr == other.ptr && self.len == other.len
    }
}
impl<T> Eq for PointerViewMut<'_, T> {}
impl<T> Deref for PointerViewMut<'_, T> {
    type Target = [T];
    fn deref(&self) -> &Self::Target {
        if self.ptr.is_null() {
            return &[];
        }
        unsafe { std::slice::from_raw_parts(self.ptr, self.len) }
    }
}
impl<T> DerefMut for PointerViewMut<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        if self.ptr.is_null() {
            return &mut [];
        }
        unsafe { std::slice::from_raw_parts_mut(self.ptr, self.len) }
    }
}
/// Provides a read-only view to a C array with explicit unsafe mutable access.
/// # Safety
/// This does not check if the data is valid. It is assumed
/// the correct data is given and that it doesn't get dropped before this struct.
/// Mutable access is only available via [`PointerViewUnsafeMut::as_mut_slice`],
/// where the caller must uphold Rust aliasing and validity guarantees.
/// This should ONLY be used within a wrapper that fully encapsulates the underlying data.
#[derive(Debug)]
pub struct PointerViewUnsafeMut<'d, T> {
    ptr: *mut T,
    len: usize,
    phantom: PhantomData<&'d mut ()>,
}
impl<'d, T> PointerViewUnsafeMut<'d, T> {
    pub(crate) const fn new(ptr: *mut T, len: usize, phantom: PhantomData<&'d mut ()>) -> Self {
        Self { ptr, len, phantom }
    }
    /// Returns a mutable slice over the underlying data.
    ///
    /// # Safety
    /// Caller must ensure that:
    /// - `self.ptr` points to `self.len` properly aligned and initialized `T` values (or is null with `len == 0`);
    /// - no other references (shared or mutable) to overlapping memory are alive while the returned slice is used;
    /// - written values preserve Rust type validity and MuJoCo invariants.
    pub unsafe fn as_mut_slice(&mut self) -> &mut [T] {
        if self.ptr.is_null() {
            return &mut [];
        }
        unsafe { std::slice::from_raw_parts_mut(self.ptr, self.len) }
    }
}
/// Compares if the two views point to the same data with the same length.
impl<T> PartialEq for PointerViewUnsafeMut<'_, T> {
    fn eq(&self, other: &Self) -> bool {
        self.ptr == other.ptr && self.len == other.len
    }
}
impl<T> Eq for PointerViewUnsafeMut<'_, T> {}
impl<T> Deref for PointerViewUnsafeMut<'_, T> {
    type Target = [T];
    fn deref(&self) -> &Self::Target {
        if self.ptr.is_null() {
            return &[];
        }
        unsafe { std::slice::from_raw_parts(self.ptr, self.len) }
    }
}
/// Provides a more direct view to a C array.
/// # Safety
/// This does not check if the data is valid. It is assumed
/// the correct data is given and that it doesn't get dropped before this struct.
/// This does not break Rust's checks as we create the view each
/// time from the saved pointers.
/// This should ONLY be used within a wrapper that fully encapsulates the underlying data.
#[derive(Debug)]
pub struct PointerView<'d, T> {
    ptr: *const T,
    len: usize,
    phantom: PhantomData<&'d ()>,
}
impl<'d, T> PointerView<'d, T> {
    pub(crate) const fn new(ptr: *const T, len: usize, phantom: PhantomData<&'d ()>) -> Self {
        Self { ptr, len, phantom }
    }
}
/// Compares if the two views point to the same data with the same length.
impl<T> PartialEq for PointerView<'_, T> {
    fn eq(&self, other: &Self) -> bool {
        self.ptr == other.ptr && self.len == other.len
    }
}
impl<T> Eq for PointerView<'_, T> {}
impl<T> Deref for PointerView<'_, T> {
    type Target = [T];
    fn deref(&self) -> &Self::Target {
        if self.ptr.is_null() {
            return &[];
        }
        unsafe { std::slice::from_raw_parts(self.ptr, self.len) }
    }
}
/// When @eval is given false, ignore the given contents.
/// In other cases, expand the given contents.
#[macro_export]
#[doc(hidden)]
macro_rules! eval_or_expand {
    (@ eval $(true)? { $($data:tt)* }) => {
        $($data)*
    };
    (@ eval false { $($data:tt)* }) => {};
}
/// Constructs a view struct by mapping fields to their corresponding locations in `$data`.
///
/// - `$field` list uses `$ptr_view` (read-write in `ViewMut`, read-only in `View`).
/// - `$field_ro` list uses `$ptr_view_ro` (`PointerViewUnsafeMut` in `ViewMut`, `PointerView` in `View`).
/// - `$opt_field` list uses `$ptr_view`, wrapped in `Option`; the field is `None` when the
///   recorded length is zero.
///
/// # Safety
/// Caller must ensure the data pointers remain valid for the lifetime of the view, and that every
/// `(offset, len)` pair in `$self` lies inside its array: the expansion offsets each pointer with
/// `add(offset)`.
#[macro_export]
#[doc(hidden)]
macro_rules! view_creator {
    (
        $self:expr, $view:ident, $data:expr, [$($([$prefix_field:ident])? $field:ident :
        $type_:ty $([$force:ident])?),*], [$($([$prefix_field_ro:ident])? $field_ro:ident
        : $type_ro:ty $([$force_ro:ident])?),*], [$($([$prefix_opt_field:ident])?
        $opt_field:ident : $type_opt:ty $([$force_opt:ident])?),*], $ptr_view:expr,
        $ptr_view_ro:expr
    ) => {
        paste::paste! { unsafe { $view { $($field : $ptr_view
        ($crate::maybe_force_cast!($crate::native_binding::util::offset_or_null($data .
        [<$($prefix_field)? $field >], $self .$field .0), $type_ $(, $force)?), $self
        .$field .1, std::marker::PhantomData),)* $($field_ro : $ptr_view_ro
        ($crate::maybe_force_cast!($crate::native_binding::util::offset_or_null($data .
        [<$($prefix_field_ro)? $field_ro >], $self .$field_ro .0), $type_ro $(,
        $force_ro)?), $self .$field_ro .1, std::marker::PhantomData),)* $($opt_field : if
        $self .$opt_field .1 > 0 { Some($ptr_view
        ($crate::maybe_force_cast!($crate::native_binding::util::offset_or_null($data .
        [<$($prefix_opt_field)? $opt_field >], $self .$opt_field .0), $type_opt $(,
        $force_opt)?), $self .$opt_field .1, std::marker::PhantomData)) } else { None
        },)* } } }
    };
}
/// Generates a lookup method `$type_(&self, name: &str) -> Option<Mj{Type}{InfoType}Info>` on
/// a wrapper. The optional `[$model]` token names the accessor that reaches the model (`[model]`
/// on `MjData`); omit it when `self` is the model.
///
/// The returned `Info` struct stores the name, id, and index ranges needed to
/// create views into the corresponding `MjData` or `MjModel` arrays.
///
/// # Entry formats
///
/// - **Fixed stride**: `attr: N`: index range is `(id * N, N)`.
/// - **FFI stride** (with optional multiplier): `attr: ffi_field (* k)`: stride taken from
///   `model.ffi_field`, optionally scaled by `k`.
/// - **Dynamic range**: `attr: nXXX (* k)`: start and length resolved via [`mj_model_dyn_range!`],
///   where `nXXX` is the total-length field of the packed data array (e.g. `nhfielddata`,
///   `ntexdata`), which also selects the address array.
///   The optional `* k` is a stride multiplier: each logical unit occupies `k` flat elements,
///   so both the start offset and the length are scaled by `k`. Use this when the target array
///   stores `k` flat values per logical unit (e.g. `dof_dampingpoly (nv x mjNPOLY)` viewed as
///   a flat `MjtNum` slice: offset = `dof_start * mjNPOLY`, length = `n_dofs * mjNPOLY`).
#[doc(hidden)]
#[macro_export]
macro_rules! info_method {
    (
        $info_type:ident, $([$model:ident],)? $type_:ident, [$($attr:ident :
        $len:expr),*], [$($attr_ffi:ident : $len_ffi:ident $(* $multiplier:expr)?),*],
        [$($attr_dyn:ident : $ffi_len_dyn:ident $(* $offset_mult:expr)?),*]
    ) => {
        paste::paste! { #[doc = concat!("Returns a [`", stringify!([< Mj $type_ : camel
        $info_type Info >]), "`] for the named ", stringify!($type_), ", ",
        "containing the indices required to create views into [`Mj",
        stringify!($info_type), "`] arrays, ", "or `None` when the model holds no ",
        stringify!($type_), " with that name.\n\n", "Call [`view`](", stringify!([< Mj
        $type_ : camel $info_type Info >]), "::view) or ", "[`try_view`](", stringify!([<
        Mj $type_ : camel $info_type Info >]),
        "::try_view) on the result to obtain the actual view.\n\n", "# Panics\n",
        "Panics if `name` contains a `\\0` byte.")] #[allow(non_snake_case)] pub fn
        $type_ (& self, name : & str) -> Option < [< Mj $type_ : camel $info_type Info >]
        > { let model_ref = self $(.$model ())?; let id = model_ref.name_to_id(MjtObj::
        [< mjOBJ_ $type_ : upper >], name) ?; #[allow(unused)] let model_ffi = model_ref
        .ffi(); let id = id as usize; $(let $attr = (id * $len, $len);)* $(let $attr_ffi
        = (id * model_ffi.$len_ffi as usize $(* $multiplier)*, model_ffi.$len_ffi as
        usize $(* $multiplier)*,);)* $(let $attr_dyn = { let (dyn_start, dyn_len) =
        $crate::mj_model_dyn_range!(model_ref, id, $ffi_len_dyn); (dyn_start $(*
        $offset_mult)?, dyn_len $(* $offset_mult)?) };)* let model_layout = model_ref
        .layout().clone(); Some([< Mj $type_ : camel $info_type Info >] { name : name
        .to_string(), id, model_layout, $($attr,)* $($attr_ffi,)* $($attr_dyn),* }) } }
    };
}
/// Generates `Info`, `ViewMut`, and `View` types for a named MuJoCo object, along with
/// `view`, `try_view`, `view_mut`, `try_view_mut` and `model_signature` on the `Info` type and
/// `zero` on `ViewMut`. A trailing `, Generic: Bound` (`M: ModelType`) is required
/// when the target wrapper is generic, as `MjData<M>` is.
///
/// # Field lists
///
/// - **`[rw fields]`**: read-write: `PointerViewMut` in `ViewMut`, `PointerView` in `View`.
/// - **`[ro fields]`**: read as `PointerViewUnsafeMut` in `ViewMut` (unsafe to mutate), `PointerView` in `View`.
/// - **`[opt fields]`**: optional read-write: `Option<PointerViewMut>` / `Option<PointerView>`;
///   `None` when the element owns no entries in that array (a stateless actuator's `act`, say).
///
/// # Field entry syntax
///
/// ```text
/// [prefix_] field_name : ElementType [force]
/// ```
///
/// - `[prefix_]`: optional prefix prepended to the FFI field name (e.g. `[actuator_]`).
/// - `[force]`: emit a forced pointer cast via [`maybe_force_cast!`] (needed when the Rust
///   element type differs from the C array element type, e.g. `f64` -> `[f64; 3]`).
#[doc(hidden)]
#[macro_export]
macro_rules! info_with_view {
    (
        $info_type:ident, $name:ident, [$($([$prefix_attr:ident])? $attr:ident :
        $type_:ty $([$force:ident])?),*], [$($([$prefix_attr_ro:ident])? $attr_ro:ident :
        $type_ro:ty $([$force_ro:ident])?),*], [$($([$prefix_opt_attr:ident])?
        $opt_attr:ident : $type_opt:ty $([$force_opt:ident])?),*] $(,$generics:ty :
        $bound:ty)?
    ) => {
        paste::paste! { #[doc = "Index ranges required to create views into [`Mj"
        $info_type "`] arrays for a " $name "."] #[allow(non_snake_case)] #[derive(Debug,
        Clone)] pub struct [< Mj $name : camel $info_type Info >] { #[doc =
        " Name of the element."] pub name : String, #[doc = " Index of the element."] pub
        id : usize, model_layout : std::sync::Arc
        <$crate::native_binding::wrappers::mj_model::MjModelLayout >, $($attr : (usize,
        usize),)* $($attr_ro : (usize, usize),)* $($opt_attr : (usize, usize),)* } impl
        [< Mj $name : camel $info_type Info >] { #[doc =
        " Returns the model signature this `Info` was created from."] pub fn
        model_signature(& self) -> u64 { self.model_layout.signature() } #[doc =
        concat!("Re-points this `Info` at the layout of `", stringify!([<$info_type :
        lower >]), "`.\n\n",
        "A compatible model holds the same index ranges, so the cached ranges stay correct and only ",
        "the layout handle changes. Every later view test against that same [`Mj",
        stringify!($info_type), "`] ", "is a pointer comparison again.\n\n",
        "# Errors\n", "Returns [`IncompatibleModel`](", stringify!([< Mj $info_type Error
        >]), "::IncompatibleModel) if `", stringify!([<$info_type : lower >]),
        "` was built from a model that is not compatible with this `Info`'s model.")] pub
        fn update_layout $(<$generics : $bound >)? (& mut self, [<$info_type : lower >] :
        & [< Mj $info_type >] $(<$generics >)?) -> Result < (),
        $crate::native_binding::error:: [< Mj $info_type Error >] > { let
        destination_layout = [<$info_type : lower >].layout(); if self.model_layout != *
        destination_layout { return Err($crate::native_binding::error:: [< Mj $info_type
        Error >] ::IncompatibleModel { source : self.model_layout.signature(),
        destination : destination_layout.signature(), }); } self.model_layout =
        std::sync::Arc::clone(destination_layout); Ok(()) } #[doc =
        concat!("Returns a mutable view into the [`Mj", stringify!($info_type),
        "`] arrays for this ", stringify!($name), ".\n\n",
        "Fields listed as read-only use [`PointerViewUnsafeMut`](crate::native_binding::util::PointerViewUnsafeMut): ",
        "read is safe, mutation requires [`as_mut_slice`](crate::native_binding::util::PointerViewUnsafeMut::as_mut_slice) and `unsafe`.\n\n",
        "# Errors\n", "Returns [`IncompatibleModel`](", stringify!([< Mj $info_type Error
        >]), "::IncompatibleModel) if `", stringify!($info_type),
        "` was built from a model that is not compatible with this `Info`'s model.",
        "\n\n# Note\n",
        "Every call tests the model layout. While this `Info` and the given [`Mj",
        stringify!($info_type), "`] ",
        "name the same model, that test is a pointer comparison; otherwise it compares the whole layout ",
        "snapshot. Call `update_layout` after a model swap to restore the pointer comparison.")]
        pub fn try_view_mut <'p $(, $generics : $bound)?> (& self, [<$info_type : lower
        >] : &'p mut [< Mj $info_type >] $(<$generics >)?) -> Result < [< Mj $name :
        camel $info_type ViewMut >] <'p >, $crate::native_binding::error:: [< Mj
        $info_type Error >] > { let destination_layout = [<$info_type : lower >]
        .layout(); if self.model_layout != * destination_layout { return
        Err($crate::native_binding::error:: [< Mj $info_type Error >] ::IncompatibleModel
        { source : self.model_layout.signature(), destination : destination_layout
        .signature(), }); } Ok(view_creator!(self, [< Mj $name : camel $info_type ViewMut
        >], [<$info_type : lower >].ffi(), [$($([$prefix_attr])? $attr : $type_
        $([$force])?),*], [$($([$prefix_attr_ro])? $attr_ro : $type_ro
        $([$force_ro])?),*], [$($([$prefix_opt_attr])? $opt_attr : $type_opt
        $([$force_opt])?),*], $crate::native_binding::util::PointerViewMut::new,
        $crate::native_binding::util::PointerViewUnsafeMut::new)) } #[doc =
        concat!("Returns a mutable view into the [`Mj", stringify!($info_type),
        "`] arrays for this ", stringify!($name), ".\n\n",
        "Fields listed as read-only use [`PointerViewUnsafeMut`](crate::native_binding::util::PointerViewUnsafeMut): ",
        "read is safe, mutation requires [`as_mut_slice`](crate::native_binding::util::PointerViewUnsafeMut::as_mut_slice) and `unsafe`.\n\n",
        "# Panics\n", "Panics if `", stringify!($info_type),
        "` was built from a model that is not compatible with this `Info`'s model. ",
        "Use [`try_view_mut`](Self::try_view_mut) to handle this as a `Result`.",
        "\n\n# Note\n",
        "Every call tests the model layout. While this `Info` and the given [`Mj",
        stringify!($info_type), "`] ",
        "name the same model, that test is a pointer comparison; otherwise it compares the whole layout ",
        "snapshot. Call `update_layout` after a model swap to restore the pointer comparison.")]
        pub fn view_mut <'p $(, $generics : $bound)?> (& self, [<$info_type : lower >] :
        &'p mut [< Mj $info_type >] $(<$generics >)?) -> [< Mj $name : camel $info_type
        ViewMut >] <'p > { self.try_view_mut([<$info_type : lower >]).unwrap_or_else(| _
        | panic!("the model is not compatible")) } #[doc =
        concat!("Returns an immutable view into the [`Mj", stringify!($info_type),
        "`] arrays for this ", stringify!($name), ".\n\n", "# Errors\n",
        "Returns [`IncompatibleModel`](", stringify!([< Mj $info_type Error >]),
        "::IncompatibleModel) if `", stringify!($info_type),
        "` was built from a model that is not compatible with this `Info`'s model.",
        "\n\n# Note\n",
        "Every call tests the model layout. While this `Info` and the given [`Mj",
        stringify!($info_type), "`] ",
        "name the same model, that test is a pointer comparison; otherwise it compares the whole layout ",
        "snapshot. Call `update_layout` after a model swap to restore the pointer comparison.")]
        pub fn try_view <'p $(, $generics : $bound)?> (& self, [<$info_type : lower >] :
        &'p[< Mj $info_type >] $(<$generics >)?) -> Result < [< Mj $name : camel
        $info_type View >] <'p >, $crate::native_binding::error:: [< Mj $info_type Error
        >] > { let destination_layout = [<$info_type : lower >].layout(); if self
        .model_layout != * destination_layout { return
        Err($crate::native_binding::error:: [< Mj $info_type Error >] ::IncompatibleModel
        { source : self.model_layout.signature(), destination : destination_layout
        .signature(), }); } Ok(view_creator!(self, [< Mj $name : camel $info_type View
        >], [<$info_type : lower >].ffi(), [$($([$prefix_attr])? $attr : $type_
        $([$force])?),*], [$($([$prefix_attr_ro])? $attr_ro : $type_ro
        $([$force_ro])?),*], [$($([$prefix_opt_attr])? $opt_attr : $type_opt
        $([$force_opt])?),*], $crate::native_binding::util::PointerView::new,
        $crate::native_binding::util::PointerView::new)) } #[doc =
        concat!("Returns an immutable view into the [`Mj", stringify!($info_type),
        "`] arrays for this ", stringify!($name), ".\n\n", "# Panics\n", "Panics if `",
        stringify!($info_type),
        "` was built from a model that is not compatible with this `Info`'s model. ",
        "Use [`try_view`](Self::try_view) to handle this as a `Result`.", "\n\n# Note\n",
        "Every call tests the model layout. While this `Info` and the given [`Mj",
        stringify!($info_type), "`] ",
        "name the same model, that test is a pointer comparison; otherwise it compares the whole layout ",
        "snapshot. Call `update_layout` after a model swap to restore the pointer comparison.")]
        pub fn view <'p $(, $generics : $bound)?> (& self, [<$info_type : lower >] :
        &'p[< Mj $info_type >] $(<$generics >)?) -> [< Mj $name : camel $info_type View
        >] <'p > { self.try_view([<$info_type : lower >]).unwrap_or_else(| _ |
        panic!("the model is not compatible")) } } #[doc = "Mutable view into [`Mj"
        $info_type "`] arrays for a " $name ".\n\n"
        "Read-write fields are [`PointerViewMut`](crate::native_binding::util::PointerViewMut); "
        "read-only fields are [`PointerViewUnsafeMut`](crate::native_binding::util::PointerViewUnsafeMut), "
        "which require [`as_mut_slice`](crate::native_binding::util::PointerViewUnsafeMut::as_mut_slice) and explicit `unsafe` to mutate."]
        #[allow(non_snake_case)] #[derive(Debug)] pub struct [< Mj $name : camel
        $info_type ViewMut >] <'d > { $(#[doc = concat!("Mutable view of `",
        stringify!($attr), "`.")] pub $attr :
        $crate::native_binding::util::PointerViewMut <'d, $type_ >,)* $(#[doc =
        concat!("Read-only view of `", stringify!($attr_ro),
        "`. Requires `unsafe` for mutation.")] pub $attr_ro :
        $crate::native_binding::util::PointerViewUnsafeMut <'d, $type_ro >,)* $(#[doc =
        concat!("Optional mutable view of `", stringify!($opt_attr),
        "`. `None` when this element owns no entries in the array.")] pub $opt_attr :
        Option <$crate::native_binding::util::PointerViewMut <'d, $type_opt >>,)* } impl
        [< Mj $name : camel $info_type ViewMut >] <'_ > { #[doc =
        " Zeroes all read-write fields. Read-only fields are left unchanged."] pub fn
        zero(& mut self) { $(self.$attr .fill(bytemuck::Zeroable::zeroed());)* $(if let
        Some(x) = & mut self.$opt_attr { x.fill(bytemuck::Zeroable::zeroed()); })* } }
        #[doc = "Immutable view into [`Mj" $info_type "`] arrays for a " $name "."]
        #[allow(non_snake_case)] #[derive(Debug)] pub struct [< Mj $name : camel
        $info_type View >] <'d > { $(#[doc = concat!("View of `", stringify!($attr),
        "`.")] pub $attr : $crate::native_binding::util::PointerView <'d, $type_ >,)*
        $(#[doc = concat!("View of `", stringify!($attr_ro), "`.")] pub $attr_ro :
        $crate::native_binding::util::PointerView <'d, $type_ro >,)* $(#[doc =
        concat!("Optional view of `", stringify!($opt_attr),
        "`. `None` when this element owns no entries in the array.")] pub $opt_attr :
        Option <$crate::native_binding::util::PointerView <'d, $type_opt >>,)* } }
    };
}
/// Generates getters, setters, and builder methods for struct fields.
///
/// ## Optional value constraints
///
/// Every arm that writes a value (`set`, `with`, `[&] with`, their `[force]` cast variants, and the
/// `get, set` / `with, get, set` / `with, get` aggregates) accepts an **optional per-field check**
/// for fields whose value can be invalid. Bool arms have no check (a bool is always in range).
///
/// The check is given as `{ check, "reason" }`, where `check` is the validating function
/// (`Fn(Type) -> Result<(), ErrType>`) and `"reason"` is a doc fragment, written in the crate's usual
/// `# Errors` style, that names the error and the condition, e.g.
/// `"[`MjEditError::InvalidParameter`] when `value` is out of range"`. The macro reuses that one
/// fragment to generate the per-method docs:
///
/// - On a `set`-style field: `name: Type { check, "reason" } => ErrType; "comment"` makes `set_name`
///   return `Result<(), ErrType>` (the check runs first; on `Err` the field is left unchanged) and
///   appends `# Errors` reading `Returns <reason>.`.
/// - On a `with`-style field: `name: Type { check, "reason" }; "comment"` makes the builder
///   `with_name` call `check` and **panic** (`expect`) on `Err`, since a builder must keep returning
///   `Self`, and appends `# Panics` reading `Panics with <reason>.`.
/// - In the combined `with, get, set` aggregate, supply `{ check, "reason" } => ErrType`; the check
///   and reason are wired into both the fallible `set_name` and the panicking `with_name`.
///
/// The `"comment"` describes the field itself and is shared verbatim by the getter, setter, and
/// builder; the getter never gets an `# Errors`/`# Panics` section, so do **not** hand-write
/// error/panic notes into `"comment"`; put them in the check `"reason"` instead.
///
/// Omitting the check yields the plain infallible setter/builder.
#[doc(hidden)]
#[macro_export]
macro_rules! getter_setter {
    (@ cast force { $($content:tt)* }) => {
        $crate::native_binding::util::force_cast($($content)*)
    };
    (@ cast { $($content:tt)* }) => {
        $($content)*. into()
    };
    (
        get, [$($([$ffi:ident])? $name:ident $(+ $symbol:tt)?: bool; $comment:expr);*
        $(;)?]
    ) => {
        paste::paste! { $(#[doc = concat!("Check ", $comment)] pub fn [<$name : camel :
        snake $($symbol)?>] (& self) -> bool { (self $(.$ffi ())?.$name as i32) != 0 })*
        }
    };
    (
        get, [$($([$ffi:ident $(,$ffi_mut:ident)?])? $((allow_mut = $cfg_mut:literal))?
        $name:ident $(+ $symbol:tt)?: & $type:ty; $comment:expr);* $(;)?]
    ) => {
        paste::paste! { $(#[doc = concat!("Return an immutable reference to ", $comment)]
        pub fn [<$name : camel : snake $($symbol)?>] (& self) -> &$type { & self $(.$ffi
        ())?.$name } $crate::eval_or_expand! { @ eval $($cfg_mut)? { #[doc =
        concat!("Return a mutable reference to ", $comment)] pub fn [<$name : camel :
        snake _mut >] (& mut self) -> & mut $type { #[allow(unused_unsafe)] unsafe { &
        mut self $(.$($ffi_mut ())?)?.$name } } } })* }
    };
    (
        get, [$($([$ffi:ident])? $name:ident $(+ $symbol:tt)?: $type:ty
        $([$cast_type:tt])?; $comment:expr);* $(;)?]
    ) => {
        paste::paste! { $(#[doc = concat!("Return value of ", $comment)] pub fn [<$name :
        camel : snake $($symbol)?>] (& self) -> $type { #[allow(unused_unsafe)] unsafe {
        $crate::getter_setter!(@ cast $($cast_type)? { self $(.$ffi ())?.$name }) } })* }
    };
    (
        set, [$($([$ffi_mut:ident])? $name:ident : $type:ty $([$cast_type:tt])? $({
        $check:expr, $reason:literal })? $(=> $err:ty)?; $comment:expr);* $(;)?]
    ) => {
        paste::paste! { $(#[doc = concat!("Set ", $comment $(, "\n\n# Errors\nReturns ",
        $reason, ".")?)] pub fn [< set_ $name : camel : snake >] (& mut self, value :
        $type) $(-> Result < (), $err >)? { $(($check) (value) ?;)?
        #[allow(unused_unsafe)] unsafe { self $(.$ffi_mut ())?.$name =
        $crate::getter_setter!(@ cast $($cast_type)? { value }) }; $(Ok::< (), $err >
        (()))? })* }
    };
    (with, [$($inner:tt)*]) => {
        $crate::getter_setter!(@ with_body[Self], [$($inner)*]);
    };
    ([&] with, [$($inner:tt)*]) => {
        $crate::getter_setter!(@ with_body[& mut Self], [$($inner)*]);
    };
    (
        @ with_body[$ret_ty:ty], [$($([$ffi_mut:ident])? $name:ident : $type:ty
        $([$cast_type:tt])? $({ $check:expr, $reason:literal })?; $comment:expr);* $(;)?]
    ) => {
        paste::paste! { $(#[allow(unused_mut)] #[doc =
        concat!("Builder method for setting ", $comment $(, "\n\n# Panics\nPanics with ",
        $reason, ".")?)] pub fn [< with_ $name : camel : snake >] (mut self : $ret_ty,
        value : $type) -> $ret_ty { $(($check) (value)
        .expect("invalid builder argument");)? #[allow(unused_unsafe)] unsafe { self
        $(.$ffi_mut ())?.$name = $crate::getter_setter!(@ cast $($cast_type)? { value })
        }; self })* }
    };
    (
        get, set, [$($([$ffi:ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? :
        bool; $comment:expr);* $(;)?]
    ) => {
        $crate::getter_setter!(get, [$($([$ffi])? $name $(+ $symbol)? : bool;
        $comment);*]); $crate::getter_setter!(set, [$($([$ffi_mut])? $name : bool;
        $comment);*]);
    };
    (
        get, set, [$($([$ffi:ident, $ffi_mut:ident])? $name:ident $(+ $symbol:tt)? :
        $type:ty $([$cast_type:tt])? $({ $check:expr, $reason:literal })? $(=> $err:ty)?;
        $comment:expr);* $(;)?]
    ) => {
        $crate::getter_setter!(get, [$($([$ffi])? $name $(+ $symbol)? : $type
        $([$cast_type])?; $comment);*]); $crate::getter_setter!(set, [$($([$ffi_mut])?
        $name : $type $([$cast_type])? $({ $check, $reason })? $(=> $err)?;
        $comment);*]);
    };
    (
        $([$token:tt])? with, get, set, [$($([$ffi:ident, $ffi_mut:ident])? $name:ident
        $(+ $symbol:tt)? : bool; $comment:expr);* $(;)?]
    ) => {
        $crate::getter_setter!(get, [$($([$ffi])? $name $(+ $symbol)? : bool;
        $comment);*]); $crate::getter_setter!(set, [$($([$ffi_mut])? $name : bool;
        $comment);*]); $crate::getter_setter!($([$token])? with, [$($([$ffi_mut])? $name
        : bool; $comment);*]);
    };
    (
        $([$token:tt])? with, get, set, [$($([$ffi:ident, $ffi_mut:ident])? $name:ident
        $(+ $symbol:tt)? : $type:ty $([$cast_type:tt])? $({ $check:expr, $reason:literal
        })? $(=> $err:ty)?; $comment:expr);* $(;)?]
    ) => {
        $crate::getter_setter!(get, [$($([$ffi])? $name $(+ $symbol)?: $type
        $([$cast_type])?; $comment);*]); $crate::getter_setter!(set, [$($([$ffi_mut])?
        $name : $type $([$cast_type])? $({ $check, $reason })? $(=> $err)?;
        $comment);*]); $crate::getter_setter!($([$token])? with, [$($([$ffi_mut])? $name
        : $type $([$cast_type])? $({ $check, $reason })?; $comment);*]);
    };
    (
        $([$token:tt])? with, get, [$($([$ffi:ident, $ffi_mut:ident])? $((allow_mut =
        $allow_mut:literal))? $name:ident $(+ $symbol:tt)? : & $type:ty $({ $check:expr,
        $reason:literal })?; $comment:expr);* $(;)?]
    ) => {
        $crate::getter_setter!(get, [$($([$ffi, $ffi_mut])? $((allow_mut = $allow_mut))?
        $name $(+ $symbol)? : & $type; $comment);*]); $crate::getter_setter!($([$token])?
        with, [$($([$ffi_mut])? $name : $type $({ $check, $reason })?; $comment);*]);
    };
}
#[doc(hidden)]
#[macro_export]
/// Constructs builder methods.
macro_rules! builder_setters {
    (
        $($name:ident : $type:ty $(where $generic_type:ident : $generic:path)?;
        $comment:expr);* $(;)?
    ) => {
        $(#[doc = concat!("Set ", $comment)] pub fn $name $(<$generic_type : $generic >)?
        (mut self, value : $type) -> Self { self.$name = value.into(); self })*
    };
}
/// Helper macro that routes one probe touch to the safe probe or to the unsafe probe.
///
/// The first argument names the probe being generated, the second is the accessor's optional
/// `unsafe` prefix. A touch lands in exactly one of the two probes, so together they cover every
/// accessor of the block exactly once.
#[doc(hidden)]
#[macro_export]
macro_rules! probe_touch {
    (safe;; $call:expr) => {
        $call
    };
    (safe; unsafe; $call:expr) => {};
    (unsafe;; $call:expr) => {};
    (unsafe; unsafe; $call:expr) => {
        unsafe { $call }
    };
}
/// Helper macro for conditionally generating `# Safety` docs on immutable array slice methods.
/// When `unsafe` is passed (method is unsafe), includes the uninitialized-arena safety section.
/// When no `unsafe` is passed (method is safe), only generates the basic doc.
#[doc(hidden)]
#[macro_export]
macro_rules! array_read_doc {
    (unsafe, $doc:literal) => {
        concat!("Immutable slice of the ", $doc,
        " array.\n\n# Safety\n\nMuJoCo allocates this array in the `mjData` arena and never zeroes it. The caller must ensure that the pipeline stage which computes the array has run for the current state, for example through [`MjData::forward`](crate::native_binding::wrappers::mj_data::MjData::forward) or [`MjData::step`](crate::native_binding::wrappers::mj_data::MjData::step). A read before that stage reads uninitialized memory.")
    };
    ($doc:literal) => {
        concat!("Immutable slice of the ", $doc, " array.")
    };
}
/// Helper macro for conditionally generating `# Safety` docs on mutable array slice methods.
/// When `unsafe` is passed (method is unsafe), includes the safety section.
/// When no `unsafe` is passed (method is safe), only generates the basic doc.
#[doc(hidden)]
#[macro_export]
macro_rules! array_mut_doc {
    (unsafe, $doc:literal) => {
        concat!("Mutable slice of the ", $doc,
        " array.\n\n# Safety\n\nDirect mutation of this array bypasses MuJoCo's internal consistency checks. The caller must ensure that all values written remain valid for MuJoCo's internal state.")
    };
    ($doc:literal) => {
        concat!("Mutable slice of the ", $doc, " array.")
    };
}
/// A macro for creating a slice over a raw array of dynamic size (given by some other variable in $len_accessor).
/// Syntax: attribute: <optional pre-transformations (e.g., `as_ptr as_mut_ptr`)> &[datatype; documentation string;
/// code to access the length attribute, appearing after `self.`]
/// Syntax for arrays whose size is a sum from some length array:
///     summed {
///         ...
///         attribute: &[datatype; documentation; [
///                 size multiplier;
///                 (code to access the length array, appearing after self);
///                 (code to access the length array's length, appearing after self)
///             ]
///         ],
///         ...
///     }
/// Syntax for arrays whose second dimension is itself a variable:
///     sublen_dep {
///         attribute: &[[datatype; code to access the inner length]; documentation;
///                      code to access the outer length]
///     }
/// The accessor generated by `sublen_dep` returns a flat slice of `outer * inner` elements.
///
/// Two optional prefixes restrict how an attribute may be reached:
///   `(mut = unsafe)`  the `_mut` accessor is unsafe; C reads the values as unguarded indices.
///   `(read = unsafe)` both accessors are unsafe; MuJoCo allocates the array in the never-zeroed
///                     arena, so a read before its computing stage reads uninitialized memory.
///
#[doc(hidden)]
#[macro_export]
macro_rules! array_slice_dyn {
    (probe = $probe:ident; $($rest:tt)*) => {
        $crate::array_slice_dyn!($($rest)*); $crate::array_slice_dyn!(@ probe safe,
        $probe, $($rest)*); $crate::array_slice_dyn!(@ probe unsafe, $probe, $($rest)*);
    };
    (
        @ probe safe, $probe:ident, $($((mut = $unsafe_mut:ident))? $((read =
        $unsafe_read:ident))? $name:ident : $($as_ptr:ident $as_mut_ptr:ident)? &
        [$type:ty $([$force:ident])?; $doc:literal; $($len_accessor:tt)*]),*
    ) => {
        paste::paste! { #[doc =
        " Touches the first and last element of every slice this block reaches through a safe"]
        #[doc =
        " accessor, on the read path and, where the setter is also safe, on the write path."]
        #[doc = ""] #[doc =
        " Each write restores the value it read, so no field changes. Call this on a freshly"]
        #[doc = " built value, before any pipeline stage runs."] #[cfg(test)] pub (crate)
        fn $probe (& mut self) { $($crate::probe_touch!(safe; $($unsafe_read)?;
        $crate::native_binding::util::testing::touch_slice_ends(& self. [<$name : camel :
        snake >] ())); $crate::probe_touch!(safe; $($unsafe_read)? $($unsafe_mut)?;
        $crate::native_binding::util::testing::touch_slice_ends_mut(self. [<$name : camel
        : snake _mut >] ()));)* } }
    };
    (
        @ probe unsafe, $probe:ident, $($((mut = $unsafe_mut:ident))? $((read =
        $unsafe_read:ident))? $name:ident : $($as_ptr:ident $as_mut_ptr:ident)? &
        [$type:ty $([$force:ident])?; $doc:literal; $($len_accessor:tt)*]),*
    ) => {
        paste::paste! { #[doc =
        " The counterpart of the safe probe, over the accessors this block marks"] #[doc
        = " `(read = unsafe)` or `(mut = unsafe)`."] #[doc = ""] #[doc = " # Safety"]
        #[doc =
        " Every `(read = unsafe)` accessor of this block requires the pipeline stage that"]
        #[doc = " computes its array to have run for the current state."] #[cfg(test)]
        pub (crate) unsafe fn [<$probe _unsafe >] (& mut self) {
        $($crate::probe_touch!(unsafe; $($unsafe_read)?;
        $crate::native_binding::util::testing::touch_slice_ends(& self. [<$name : camel :
        snake >] ())); $crate::probe_touch!(unsafe; $($unsafe_read)? $($unsafe_mut)?;
        $crate::native_binding::util::testing::touch_slice_ends_mut(self. [<$name : camel
        : snake _mut >] ()));)* } }
    };
    (
        $($((mut = $unsafe_mut:ident))? $((read = $unsafe_read:ident))? $name:ident :
        $($as_ptr:ident $as_mut_ptr:ident)? & [$type:ty $([$force:ident])?; $doc:literal;
        $($len_accessor:tt)*]),*
    ) => {
        paste::paste! { $(#[doc = $crate::array_read_doc!($($unsafe_read,)? $doc)] pub
        $($unsafe_read)? fn [<$name : camel : snake >] (& self) -> & [$type] { let length
        = self.$($len_accessor)* as usize; let ptr = $crate::maybe_force_cast!(self.ffi()
        .$name $(.$as_ptr ())?, $type $(, $force)?); if ptr.is_null() || length == 0 {
        return & []; } unsafe { std::slice::from_raw_parts(ptr, length) } } #[doc =
        $crate::array_mut_doc!($($unsafe_read,)? $($unsafe_mut,)? $doc)] pub
        $($unsafe_read)? $($unsafe_mut)? fn [<$name : camel : snake _mut >] (& mut self)
        -> & mut [$type] { let length = self.$($len_accessor)* as usize; let ptr =
        $crate::maybe_force_cast!(unsafe { self.ffi_mut().$name $(.$as_mut_ptr ())? },
        $type $(, $force)?); if ptr.is_null() || length == 0 { return & mut []; } unsafe
        { std::slice::from_raw_parts_mut(ptr, length) } })* }
    };
    (
        summed { $($(($unsafe_mut:ident))? $name:ident : & [$type:ty; $doc:literal;
        [$multiplier:literal; ($($len_array:tt)*); ($($len_array_length:tt)*)]]),* }
    ) => {
        paste::paste! { $(#[doc = concat!("Immutable slice of the ", $doc, " array.")]
        pub fn [<$name : camel : snake >] (& self) -> & [[$type; $multiplier]] { let
        length_array_length = self.$($len_array_length)* as usize; let data_ptr = self
        .ffi().$name; let length_ptr = self.$($len_array)*; if data_ptr.is_null() ||
        length_ptr.is_null() || length_array_length == 0 { return & []; } let length =
        unsafe { std::slice::from_raw_parts(length_ptr, length_array_length).iter()
        .map(|& x | x as u32).sum::< u32 > () as usize }; if length == 0 { return & []; }
        unsafe { std::slice::from_raw_parts($crate::maybe_force_cast!(data_ptr, [$type;
        $multiplier], force), length) } } #[doc = $crate::array_mut_doc!($($unsafe_mut,)?
        $doc)] pub $($unsafe_mut)? fn [<$name : camel : snake _mut >] (& mut self) -> &
        mut [[$type; $multiplier]] { let length_array_length = self.$($len_array_length)*
        as usize; let data_ptr = unsafe { self.ffi_mut().$name }; let length_ptr = self
        .$($len_array)*; if data_ptr.is_null() || length_ptr.is_null() ||
        length_array_length == 0 { return & mut []; } let length = unsafe {
        std::slice::from_raw_parts(length_ptr, length_array_length).iter().map(|& x | x
        as u32).sum::< u32 > () as usize }; if length == 0 { return & mut []; } unsafe {
        std::slice::from_raw_parts_mut($crate::maybe_force_cast!(data_ptr, [$type;
        $multiplier], force), length) } })* }
    };
    (
        sublen_dep { $($(($unsafe_mut:ident))? $name:ident : $($as_ptr:ident
        $as_mut_ptr:ident)? & [[$type:ty; $($inner_len_accessor:tt)*] $([$force:ident])?;
        $doc:literal; $($len_accessor:tt)*]),* }
    ) => {
        paste::paste! { $(#[doc = concat!("Immutable slice of the ", $doc, " array.")]
        pub fn [<$name : camel : snake >] (& self) -> & [$type] { let length = self
        .$($len_accessor)* as usize * (self.$($inner_len_accessor)*) as usize; let ptr =
        $crate::maybe_force_cast!(self.ffi().$name $(.$as_ptr ())?, $type $(, $force)?);
        if ptr.is_null() || length == 0 { return & []; } unsafe {
        std::slice::from_raw_parts(ptr, length) } } #[doc =
        $crate::array_mut_doc!($($unsafe_mut,)? $doc)] pub $($unsafe_mut)? fn [<$name :
        camel : snake _mut >] (& mut self) -> & mut [$type] { let length = self
        .$($len_accessor)* as usize * (self.$($inner_len_accessor)*) as usize; let ptr =
        $crate::maybe_force_cast!(unsafe { self.ffi_mut().$name $(.$as_mut_ptr ())? },
        $type $(, $force)?); if ptr.is_null() || length == 0 { return & mut []; } unsafe
        { std::slice::from_raw_parts_mut(ptr, length) } })* }
    };
}
/// Generates getter and setter methods for converting between Rust's &str type and C's char arrays.
///
/// The first tokens select the methods to create (`get` = getter, `set` = setter, `with` =
/// builder) in one of the accepted orders (`get`, `set`, `with`, `get, set`, `with, set`,
/// `with, get`, `with, get, set`), followed directly by the parameter group, with no comma in
/// between:
/// `c_str_as_str_method! {with, get, set { ... }}`.
///
/// # Panics
/// The generated getters panic if the `char` buffer holds no NUL terminator, or if the bytes
/// before it are not valid UTF-8. The generated setters and builders panic if the value is not
/// ASCII, holds an interior NUL byte, or does not fit in the buffer. A generated method that
/// takes a sub-index panics if that index is out of range.
///
/// The parameters are recursive and are as follows:
/// - ffi (optional): name of the method that returns some lower-level struct,
///                   which contains the actual attributes we want to read;
/// - name: the attribute name;
/// - sub_index_name: sub_index_type (optional): creates an additional parameter which indexes the `name` array
///                   in order to get a sub-array
///                   (e.g., `name` could be `[[i8; 100]; 10]` and we wish to get `[i8; 100]`);
/// - comment: the documentation comment to insert as the methods documentation.
///
#[doc(hidden)]
#[macro_export]
macro_rules! c_str_as_str_method {
    (
        get { $($([$ffi:ident])? $name:ident $([$sub_index_name:ident :
        $sub_index_type:ty])?; $comment:literal;)* }
    ) => {
        $(#[doc = concat!("Returns ", $comment, "\n\n# Panics",
        "\nPanics if the buffer has no NUL terminator or if the resulting string contains invalid UTF-8.",
        $("\nPanics if `", stringify!($sub_index_name), "` is out of range.",)?)] pub fn
        $name (& self $(, $sub_index_name : $sub_index_type)?) -> & str { let bytes : &
        [u8] = bytemuck::cast_slice(& self $(.$ffi ())?.$name $([$sub_index_name])?
        [..]); std::ffi::CStr::from_bytes_until_nul(bytes)
        .expect("no NUL terminator in C string buffer").to_str().unwrap() })*
    };
    (
        set { $($([$ffi:ident])? $name:ident $([$sub_index_name:ident :
        $sub_index_type:ty])?; $comment:literal;)* }
    ) => {
        paste::paste! { $(#[doc = concat!("Sets ", $comment, "\n\n# Panics",
        "\nPanics when `", stringify!($name),
        "` contains invalid ASCII, an interior NUL byte, or is too long.",
        $("\nPanics if `", stringify!($sub_index_name), "` is out of range.",)?)] pub fn
        [< set_ $name >] (& mut self, $($sub_index_name : $sub_index_type,)? $name : &
        str) { $crate::native_binding::util::write_ascii_to_buf(& mut self $(.$ffi
        ())?.$name $([$sub_index_name])?, $name,); })* }
    };
    (
        with { $($([$ffi:ident])? $name:ident $([$sub_index_name:ident :
        $sub_index_type:ty])?; $comment:literal;)* }
    ) => {
        paste::paste! { $(#[doc = concat!("Builder method for setting ", $comment,
        "\n\n# Panics", "\nPanics when `", stringify!($name),
        "` contains invalid ASCII, an interior NUL byte, or is too long.",
        $("\nPanics if `", stringify!($sub_index_name), "` is out of range.",)?)] pub fn
        [< with_ $name >] (mut self, $($sub_index_name : $sub_index_type,)? $name : &
        str) -> Self { $crate::native_binding::util::write_ascii_to_buf(& mut self
        $(.$ffi ())?.$name $([$sub_index_name])?, $name,); self })* }
    };
    (with, get, set { $($other:tt)* }) => {
        $crate::c_str_as_str_method!(get { $($other)* });
        $crate::c_str_as_str_method!(set { $($other)* });
        $crate::c_str_as_str_method!(with { $($other)* });
    };
    (get, set { $($other:tt)* }) => {
        $crate::c_str_as_str_method!(get { $($other)* });
        $crate::c_str_as_str_method!(set { $($other)* });
    };
    (with, set { $($other:tt)* }) => {
        $crate::c_str_as_str_method!(set { $($other)* });
        $crate::c_str_as_str_method!(with { $($other)* });
    };
    (with, get { $($other:tt)* }) => {
        $crate::c_str_as_str_method!(get { $($other)* });
        $crate::c_str_as_str_method!(with { $($other)* });
    };
}
/// assert_eq!, but with tolerance for floating point rounding.
#[doc(hidden)]
#[macro_export]
macro_rules! assert_relative_eq {
    ($a:expr, $b:expr, epsilon = $eps:expr) => {{
        let (a, b, eps) = ($a as f64, $b as f64, $eps as f64);
        assert!(
            (a - b).abs() <= eps,
            "left={:?} right={:?} eps={:?}",
            a,
            b,
            eps
        );
    }};
}
/// Tries to cast $value into requested type.
/// # Panics
/// Panics if the cast fails.
#[doc(hidden)]
#[macro_export]
macro_rules! cast_mut_info {
    ($value:expr $(, $debug_expr:expr)?) => {
        { match bytemuck::checked::try_cast_mut($value) { Ok(v) => v, Err(e) => { let
        evaluated = format!("{:?}", $value); #[allow(unused)] let mut debug_info =
        String::new(); $(debug_info = format!(" (debug info: '{} = {}')",
        stringify!($debug_expr), $debug_expr);)?
        panic!("failed to cast expression '{}', which evaluates to '{}' into requested type (error: {})\
                         {debug_info} --- \
                         most likely you have a bug in your program.",
        stringify!($value), evaluated, e); } } }
    };
}
/// Asserts that the MuJoCo version used matches
/// the one MuJoCo-rs was compiled with.
///
/// # Panics
/// Panics if the linked MuJoCo library version does not match
/// the version MuJoCo-rs was compiled against.
pub fn assert_mujoco_version() {
    let linked_version = unsafe { mj_version() as u32 };
    let mujoco_rs_version_string =
        option_env!("CARGO_PKG_VERSION").unwrap_or_else(|| "unknown+mj-unknown");
    assert_eq!(
        linked_version, mjVERSION_HEADER,
        "linked MuJoCo version value ({linked_version}) does not match expected version value ({mjVERSION_HEADER}), \
        with which MuJoCo-rs {mujoco_rs_version_string} FFI bindings were generated.",
    );
}
/// Forcefully casts a value of type `T` to type `U`.
/// Performs compile-time size and alignment checks, but does **not** guarantee
/// that the bit patterns are compatible.
///
/// # Safety
/// The bit pattern of `val` must be a valid representation for type `U`;
/// otherwise the behavior is undefined.
#[inline(always)]
pub unsafe fn force_cast<T, U>(val: T) -> U {
    const {
        assert!(std::mem::size_of::<T>() == std::mem::size_of::<U>());
        assert!(std::mem::align_of::<T>() == std::mem::align_of::<U>());
    }
    #[repr(C)]
    union Transmuter<T, U> {
        from: std::mem::ManuallyDrop<T>,
        to: std::mem::ManuallyDrop<U>,
    }
    unsafe {
        std::mem::ManuallyDrop::into_inner(
            Transmuter {
                from: std::mem::ManuallyDrop::new(val),
            }
            .to,
        )
    }
}
/// Asserts at compile time that casting from `Src` to `Dst` is
/// size-and-alignment compatible.
///
/// The target element size must be a multiple of the source element size
/// (covers both same-size type conversions and array-grouping casts like
/// `*const f64` to `*const [f64; 3]`), and the source and target alignments
/// must be equal.
///
/// The pointer argument is only used for type inference of `Src`; it is
/// never dereferenced.
#[inline(always)]
pub const fn assert_ptr_cast_valid<Src, Dst>(_ptr: *const Src) {
    const {
        assert!(
            std::mem::size_of::<Dst>().is_multiple_of(std::mem::size_of::<Src>()),
            "ptr cast: target size must be a multiple of source size"
        );
        assert!(
            std::mem::align_of::<Src>() == std::mem::align_of::<Dst>(),
            "ptr cast: source alignment must be == target alignment"
        );
    }
}
/// Conditionally casts a raw pointer to `$type` with compile-time
/// size and alignment checks.  When the `force` token is absent the
/// pointer is returned as-is.
#[doc(hidden)]
#[macro_export]
macro_rules! maybe_force_cast {
    ($ptr:expr, $type:ty) => {
        $ptr
    };
    ($ptr:expr, $type:ty, force) => {{
        let ptr = $ptr;
        $crate::native_binding::util::assert_ptr_cast_valid::<_, $type>(ptr as *const _);
        ptr.cast::<$type>()
    }};
}
/// Locks a synchronization primitive and resets its poison status.
/// This is useful on locations that don't need any special handling
/// after a thread panicked while holding a mutex lock.
pub trait LockUnpoison<T> {
    /// Locks the synchronization primitive, resetting its poison status if necessary.
    fn lock_unpoison(&self) -> MutexGuard<'_, T>;
}
/// Implements automatic unpoisoning on the [`Mutex`].
impl<T> LockUnpoison<T> for Mutex<T> {
    fn lock_unpoison(&self) -> MutexGuard<'_, T> {
        match self.lock() {
            Ok(lock) => lock,
            Err(e) => {
                self.clear_poison();
                e.into_inner()
            }
        }
    }
}

#[cfg(test)]
pub(crate) mod testing;
