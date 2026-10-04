//! Module containing safe wrappers around derivative functions.
use crate::native_binding::mujoco_c;
use crate::native_binding::wrappers::mj_primitive::*;
use std::ptr;
/// Derivatives of [`mju_sub_quat`](super::utility::mju_sub_quat).
/// Nullable: Da, Db.
/// Db = -Da^T
pub fn mjd_sub_quat(
    qa: &[MjtNum; 4],
    qb: &[MjtNum; 4],
    da: Option<&mut [MjtNum; 9]>,
    db: Option<&mut [MjtNum; 9]>,
) {
    unsafe {
        mujoco_c::mjd_subQuat(
            qa,
            qb,
            da.map_or(ptr::null_mut(), |d| d),
            db.map_or(ptr::null_mut(), |d| d),
        )
    }
}
/// Derivatives of [`mju_quat_integrate`](super::utility::mju_quat_integrate).
/// Nullable: Dquat, Dvel, Dscale.
pub fn mjd_quat_integrate(
    vel: &[MjtNum; 3],
    scale: MjtNum,
    dquat: Option<&mut [MjtNum; 9]>,
    dvel: Option<&mut [MjtNum; 9]>,
    dscale: Option<&mut [MjtNum; 3]>,
) {
    unsafe {
        mujoco_c::mjd_quatIntegrate(
            vel,
            scale,
            dquat.map_or(ptr::null_mut(), |d| d),
            dvel.map_or(ptr::null_mut(), |d| d),
            dscale.map_or(ptr::null_mut(), |d| d),
        )
    }
}
