//! MjData related.
use super::fun::utility::mju_norm_3;
use super::mj_auxiliary::MjContact;
use super::mj_model::traits::{ModelType, ModelTypeMut};
use super::mj_model::{MjModel, MjModelLayout, MjtObj, MjtSameFrame, MjtStage};
use super::mj_primitive::*;
use super::mj_statistic::{MjSolverStat, MjTimerStat, MjWarningStat};
use crate::native_binding::error::MjDataError;
use crate::native_binding::wrappers::mj_auxiliary::{MjStatistic, MjVisual};
use crate::native_binding::wrappers::mj_option::MjOption;
use crate::{array_slice_dyn, info_method, info_with_view, view_creator};
use crate::{getter_setter, native_binding::mujoco_c::*};
use std::borrow::Cow;
use std::ffi::CString;
use std::fmt::Debug;
use std::path::Path;
use std::ptr::{self, NonNull};
use std::sync::Arc;
/// State component elements as integer bitflags and several convenient combinations of these flags. Used by
/// `mj_getState`, `mj_setState` and `mj_stateSize`.
pub type MjtState = mjtState;
/// Constraint types. These values are not used in mjModel, but are used in the mjData field `d->efc_type` when the list
/// of active constraints is constructed at each simulation time step.
pub type MjtConstraint = mjtConstraint;
/// These values are used by the solver internally to keep track of the constraint states.
pub type MjtConstraintState = mjtConstraintState;
/// Warning types. The number of warning types is given by `mjNWARNING` which is also the length of the array
/// `mjData.warning`.
pub type MjtWarning = mjtWarning;
/// Timer types. The number of timer types is given by `mjNTIMER` which is also the length of the array
/// `mjData.timer`, as well as the length of the string array `mjTIMERSTRING` with timer names.
pub type MjtTimer = mjtTimer;
/// Sleep state of an object.
pub type MjtSleepState = mjtSleepState;
/// Wrapper around the `mjData` struct.
/// Provides lifetime guarantees as well as automatic cleanup.
#[derive(Debug)]
pub struct MjData<M: ModelType> {
    data: NonNull<mjData>,
    model: M,
}
unsafe impl<M: ModelType + Send> Send for MjData<M> {}
unsafe impl<M: ModelType + Sync> Sync for MjData<M> {}
impl<M: ModelType> MjData<M> {
    /// Creates a new [`MjData`] linked to `model`.
    ///
    /// # Note
    /// When the model has history buffers (`nhistory > 0`), its `timestep` must be positive;
    /// otherwise MuJoCo reports an error and stops the process.
    ///
    /// # Panics
    /// Panics if MuJoCo fails to allocate the data structure.
    /// Use [`MjData::try_new`] for a fallible alternative.
    pub fn new(model: M) -> Self {
        Self::try_new(model).expect("allocation of MjData failed")
    }
    /// Fallible version of [`MjData::new`].
    ///
    /// # Errors
    /// Returns [`MjDataError::AllocationFailed`] if MuJoCo returns a null pointer from `mj_makeData`.
    ///
    /// Prefer this method over [`MjData::new`] when you want to handle
    /// allocation failures without a panic.
    pub fn try_new(model: M) -> Result<Self, MjDataError> {
        let data_ptr = unsafe { mj_makeData(model.ffi()) };
        NonNull::new(data_ptr)
            .map(|data| Self { data, model })
            .ok_or(MjDataError::AllocationFailed)
    }
    /// Sets a new [`MjModel`] to be used within the instance. This can be used to modify [`MjModel`]'s
    /// parameters without causing size mismatches or violating borrow checker's requirements.
    /// This can be done by keeping a clone of the model, which is then modified and swapped.
    ///
    /// # Panics
    /// Panics if `model` is not compatible with the model this data belongs to
    /// (see [`MjModel::is_compatible_with_model`]).
    ///
    /// Use [`MjData::try_swap_model`] for a fallible alternative.
    ///
    /// # Notes
    /// This method only validates the model memory layout.
    /// **Not all model parameters are safe (for correct simulation) to change at runtime.**
    /// See [here](https://mujoco.readthedocs.io/en/3.12.0/programming/simulation.html#mjmodel-changes)
    /// to see what parameters can be changed.
    ///
    /// If `M` implements [`ModelTypeMut`], prefer
    /// [`model_mut`](MjData::model_mut) for direct in-place modification instead.
    ///
    /// If model recompilation speed is not an issue,
    /// it is recommended to use [`MjSpec`](crate::native_binding::wrappers::mj_editing::MjSpec) instead.
    ///
    /// # Example
    /// ```
    /// # use mujoco_rs::prelude::*;
    /// let mut model_template = Box::new(MjSpec::new().compile().unwrap());
    /// let model_used = model_template.clone();
    /// let mut data = MjData::new(model_used);
    ///
    /// model_template.opt_mut().timestep = 0.004;
    /// model_template = data.swap_model(model_template);
    /// ```
    pub fn swap_model(&mut self, model: M) -> M {
        self.try_swap_model(model)
            .expect("swap_model failed: the model is not compatible")
    }
    /// Fallible version of [`MjData::swap_model`].
    ///
    /// # Errors
    /// Returns [`MjDataError::IncompatibleModel`] if `model` is not compatible with the model
    /// this data belongs to (see [`MjModel::is_compatible_with_model`]).
    pub fn try_swap_model(&mut self, model: M) -> Result<M, MjDataError> {
        if !self.model.is_compatible_with_model(&model) {
            return Err(MjDataError::IncompatibleModel {
                source: model.signature(),
                destination: self.model.signature(),
            });
        }
        Ok(std::mem::replace(&mut self.model, model))
    }
    info_method! {
        Data, [model], body, [xfrc_applied : 6, xpos : 3, xquat : 4, xmat : 9, xipos : 3,
        ximat : 9, subtree_com : 3, cinert : 10, crb : 10, cvel : 6, subtree_linvel : 3,
        subtree_angmom : 3, cacc : 6, cfrc_int : 6, cfrc_ext : 6, awake : 1], [], []
    }
    info_method! {
        Data, [model], camera, [xpos : 3, xmat : 9], [], []
    }
    info_method! {
        Data, [model], geom, [xpos : 3, xmat : 9], [], []
    }
    info_method! {
        Data, [model], site, [xpos : 3, xmat : 9], [], []
    }
    info_method! {
        Data, [model], light, [xpos : 3, xdir : 3], [], []
    }
    info_method! {
        Data, [model], actuator, [], [], [ctrl : nu, length : nout, velocity : nout,
        force : nout, act : na]
    }
    /// Obtains a [`MjJointDataInfo`] struct containing information about the name, id, and
    /// indices required for obtaining a slice view to the correct locations in [`MjData`].
    /// The actual view can be obtained via [`MjJointDataInfo::view`].
    /// # Panics
    /// When the `name` contains '\0' characters, a panic occurs.
    pub fn joint(&self, name: &str) -> Option<MjJointDataInfo> {
        let model = self.model();
        let id = model.name_to_id(MjtObj::mjOBJ_JOINT, name)?;
        let nq_range = {
            let slice = model.jnt_qposadr();
            crate::native_binding::util::optional_sparse_addr_range(slice, id, model.nq() as usize)
                .unwrap_or((0, 0))
        };
        let nv_range = {
            let slice = model.jnt_dofadr();
            crate::native_binding::util::optional_sparse_addr_range(slice, id, model.nv() as usize)
                .unwrap_or((0, 0))
        };
        let qpos = nq_range;
        let qvel = nv_range;
        let qacc_warmstart = nv_range;
        let qfrc_applied = nv_range;
        let qacc = nv_range;
        let xanchor = (id * 3, 3);
        let xaxis = (id * 3, 3);
        #[allow(non_snake_case)]
        let qLDiagInv = nv_range;
        let qfrc_bias = nv_range;
        let qfrc_passive = nv_range;
        let qfrc_actuator = nv_range;
        let qfrc_smooth = nv_range;
        let qacc_smooth = nv_range;
        let qfrc_constraint = nv_range;
        let qfrc_inverse = nv_range;
        let qfrc_spring = nv_range;
        let qfrc_damper = nv_range;
        let qfrc_gravcomp = nv_range;
        let qfrc_fluid = nv_range;
        let qfrc_adhesion = nv_range;
        let model_layout = self.model.layout().clone();
        Some(MjJointDataInfo {
            name: name.to_string(),
            id,
            model_layout,
            qpos,
            qvel,
            qacc_warmstart,
            qfrc_applied,
            qacc,
            xanchor,
            xaxis,
            qLDiagInv,
            qfrc_bias,
            qfrc_spring,
            qfrc_damper,
            qfrc_gravcomp,
            qfrc_fluid,
            qfrc_adhesion,
            qfrc_passive,
            qfrc_actuator,
            qfrc_smooth,
            qacc_smooth,
            qfrc_constraint,
            qfrc_inverse,
        })
    }
    info_method! {
        Data, [model], sensor, [], [], [data : nsensordata]
    }
    info_method! {
        Data, [model], tendon, [wrapadr : 1, wrapnum : 1, efcadr : 1, length : 1,
        velocity : 1], [], [J : nJten]
    }
    /// Steps the MuJoCo simulation.
    pub fn step(&mut self) {
        unsafe {
            mj_step(self.model.ffi(), self.ffi_mut());
        }
    }
    /// Runs the first phase of a simulation step: computes kinematics and sensor data,
    /// before the user sets controls. Wraps [`mj_step1`].
    pub fn step1(&mut self) {
        unsafe {
            mj_step1(self.model.ffi(), self.ffi_mut());
        }
    }
    /// Runs the second phase of a simulation step: computes dynamics and integrates forward
    /// in time, after the user sets controls. Wraps [`mj_step2`].
    pub fn step2(&mut self) {
        unsafe {
            mj_step2(self.model.ffi(), self.ffi_mut());
        }
    }
    /// Forward dynamics: same as [`mj_step`] but do not integrate in time. Wraps [`mj_forward`].
    pub fn forward(&mut self) {
        unsafe {
            mj_forward(self.model.ffi(), self.ffi_mut());
        }
    }
    /// [`MjData::forward`] dynamics with skip. Wraps [`mj_forwardSkip`].
    pub fn forward_skip(&mut self, skipstage: MjtStage, skipsensor: bool) {
        unsafe {
            mj_forwardSkip(
                self.model.ffi(),
                self.ffi_mut(),
                skipstage as i32,
                skipsensor as i32,
            );
        }
    }
    /// Inverse dynamics: qacc must be set before calling this function. Wraps [`mj_inverse`].
    pub fn inverse(&mut self) {
        unsafe {
            mj_inverse(self.model.ffi(), self.ffi_mut());
        }
    }
    /// [`MjData::inverse`] dynamics with skip; skipstage is [`MjtStage`]. Wraps [`mj_inverseSkip`].
    pub fn inverse_skip(&mut self, skipstage: MjtStage, skipsensor: bool) {
        unsafe {
            mj_inverseSkip(
                self.model.ffi(),
                self.ffi_mut(),
                skipstage as i32,
                skipsensor as i32,
            );
        }
    }
    /// Extracts the contact force in the contact frame for the given `contact_id`.
    /// The `contact_id` matches the index of the contact when iterating
    /// via [`MjData::contact`]. Wraps [`mj_contactForce`].
    ///
    /// # Note
    /// When `contact_id >= ncon`, `[0; 6]` is returned.
    pub fn contact_force(&self, contact_id: usize) -> [MjtNum; 6] {
        let mut force = [0.0; 6];
        unsafe {
            mj_contactForce(
                self.model.ffi(),
                self.data.as_ptr(),
                contact_id as i32,
                &mut force,
            );
        }
        force
    }
    /// Reset data to defaults.
    ///
    /// # Note
    /// When the model has history buffers (`nhistory > 0`), its `timestep` must be positive;
    /// otherwise MuJoCo reports an error and stops the process.
    pub fn reset(&mut self) {
        unsafe { mj_resetData(self.model.ffi(), self.ffi_mut()) }
    }
    /// Reset data to defaults, fill everything else with debug_value.
    ///
    /// # Note
    /// When the model has history buffers (`nhistory > 0`), its `timestep` must be positive;
    /// otherwise MuJoCo reports an error and stops the process.
    ///
    /// # Safety
    /// `debug_value` is written as raw bytes into every buffer-resident array,
    /// including ones whose element types have validity invariants (e.g.
    /// [`bvh_active`](Self::bvh_active) -> `&[bool]`,
    /// [`body_awake`](Self::body_awake) -> `&[MjtSleepState]`). The caller must
    /// not call such accessors before a subsequent [`reset`](Self::reset) unless
    /// `debug_value` produces valid bit patterns for them.
    pub unsafe fn reset_debug(&mut self, debug_value: u8) {
        unsafe { mj_resetDataDebug(self.model.ffi(), self.ffi_mut(), debug_value) }
    }
    /// Reset data to keyframe `key` (zero-based index).
    ///
    /// # Note
    /// When the model has history buffers (`nhistory > 0`), its `timestep` must be positive;
    /// otherwise MuJoCo reports an error and stops the process.
    ///
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] if `key >= nkey`.
    pub fn reset_keyframe(&mut self, key: usize) -> Result<(), MjDataError> {
        let nkey = self.model.ffi().nkey as usize;
        if key >= nkey {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "key",
                id: key,
                upper: nkey,
            });
        }
        unsafe { mj_resetDataKeyframe(self.model.ffi(), self.ffi_mut(), key as i32) }
        Ok(())
    }
    /// Print mjData to text file, specifying format.
    /// float_format must be a valid printf-style format string for a single float value.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjDataError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// # Panics
    /// When either string contains '\0' characters, a panic occurs.
    pub fn print_formatted<T: AsRef<Path>>(
        &self,
        filename: T,
        float_format: &str,
    ) -> Result<(), MjDataError> {
        let path_str = filename
            .as_ref()
            .to_str()
            .ok_or(MjDataError::InvalidUtf8Path)?;
        let c_filename = CString::new(path_str).unwrap();
        let c_float_format = CString::new(float_format).unwrap();
        unsafe {
            mj_printFormattedData(
                self.model.ffi(),
                self.ffi(),
                c_filename.as_ptr(),
                c_float_format.as_ptr(),
            )
        }
        Ok(())
    }
    /// Print data to text file.
    /// # Returns
    /// `Ok(())` on success.
    /// # Errors
    /// - [`MjDataError::InvalidUtf8Path`] if the path contains invalid UTF-8.
    /// # Panics
    /// When the filename contains '\0' characters, a panic occurs.
    pub fn print<T: AsRef<Path>>(&self, filename: T) -> Result<(), MjDataError> {
        let path_str = filename
            .as_ref()
            .to_str()
            .ok_or(MjDataError::InvalidUtf8Path)?;
        let c_filename = CString::new(path_str).unwrap();
        unsafe { mj_printData(self.model.ffi(), self.ffi(), c_filename.as_ptr()) }
        Ok(())
    }
    /// Run position-dependent computations.
    pub fn fwd_position(&mut self) {
        unsafe { mj_fwdPosition(self.model.ffi(), self.ffi_mut()) }
    }
    /// Run velocity-dependent computations.
    pub fn fwd_velocity(&mut self) {
        unsafe { mj_fwdVelocity(self.model.ffi(), self.ffi_mut()) }
    }
    /// Compute actuator force qfrc_actuator.
    pub fn fwd_actuation(&mut self) {
        unsafe { mj_fwdActuation(self.model.ffi(), self.ffi_mut()) }
    }
    /// Add up all non-constraint forces, compute qacc_smooth.
    pub fn fwd_acceleration(&mut self) {
        unsafe { mj_fwdAcceleration(self.model.ffi(), self.ffi_mut()) }
    }
    /// Run selected constraint solver.
    pub fn fwd_constraint(&mut self) {
        unsafe { mj_fwdConstraint(self.model.ffi(), self.ffi_mut()) }
    }
    /// Euler integrator, semi-implicit in velocity.
    pub fn euler(&mut self) {
        unsafe { mj_Euler(self.model.ffi(), self.ffi_mut()) }
    }
    /// Runge-Kutta explicit order-N integrator.
    ///
    /// # Panics
    /// Panics if `n != 4`. The underlying MuJoCo C implementation only supports N=4;
    /// any other value causes an unconditional process abort via `mjERROR`.
    pub fn runge_kutta(&mut self, n: u32) {
        assert!(n == 4, "mj_RungeKutta only supports N=4, got {n}");
        unsafe { mj_RungeKutta(self.model.ffi(), self.ffi_mut(), n as i32) }
    }
    /// Implicit-in-velocity integrators.
    pub fn implicit(&mut self) {
        unsafe { mj_implicit(self.model.ffi(), self.ffi_mut()) }
    }
    /// Run position-dependent computations in inverse dynamics.
    pub fn inv_position(&mut self) {
        unsafe { mj_invPosition(self.model.ffi(), self.ffi_mut()) }
    }
    /// Run velocity-dependent computations in inverse dynamics.
    pub fn inv_velocity(&mut self) {
        unsafe { mj_invVelocity(self.model.ffi(), self.ffi_mut()) }
    }
    /// Apply the analytical formula for inverse constraint dynamics.
    pub fn inv_constraint(&mut self) {
        unsafe { mj_invConstraint(self.model.ffi(), self.ffi_mut()) }
    }
    /// Compare forward and inverse dynamics, save results in `solver_fwdinv`.
    pub fn compare_fwd_inv(&mut self) {
        unsafe { mj_compareFwdInv(self.model.ffi(), self.ffi_mut()) }
    }
    /// Evaluate position-dependent sensors.
    pub fn sensor_pos(&mut self) {
        unsafe { mj_sensorPos(self.model.ffi(), self.ffi_mut()) }
    }
    /// Evaluate velocity-dependent sensors.
    pub fn sensor_vel(&mut self) {
        unsafe { mj_sensorVel(self.model.ffi(), self.ffi_mut()) }
    }
    /// Evaluate acceleration and force-dependent sensors.
    pub fn sensor_acc(&mut self) {
        unsafe { mj_sensorAcc(self.model.ffi(), self.ffi_mut()) }
    }
    /// Evaluate position-dependent energy (potential).
    pub fn energy_pos(&mut self) {
        unsafe { mj_energyPos(self.model.ffi(), self.ffi_mut()) }
    }
    /// Evaluate velocity-dependent energy (kinetic).
    pub fn energy_vel(&mut self) {
        unsafe { mj_energyVel(self.model.ffi(), self.ffi_mut()) }
    }
    /// Check qpos, reset if any element is too big or nan.
    pub fn check_pos(&mut self) {
        unsafe { mj_checkPos(self.model.ffi(), self.ffi_mut()) }
    }
    /// Check qvel, reset if any element is too big or nan.
    pub fn check_vel(&mut self) {
        unsafe { mj_checkVel(self.model.ffi(), self.ffi_mut()) }
    }
    /// Check qacc, reset if any element is too big or nan.
    pub fn check_acc(&mut self) {
        unsafe { mj_checkAcc(self.model.ffi(), self.ffi_mut()) }
    }
    /// Run forward kinematics.
    pub fn kinematics(&mut self) {
        unsafe { mj_kinematics(self.model.ffi(), self.ffi_mut()) }
    }
    /// Map inertias and motion dofs to global frame centered at CoM.
    pub fn com_pos(&mut self) {
        unsafe { mj_comPos(self.model.ffi(), self.ffi_mut()) }
    }
    /// Compute camera and light positions and orientations.
    pub fn camlight(&mut self) {
        unsafe { mj_camlight(self.model.ffi(), self.ffi_mut()) }
    }
    /// Compute flex-related quantities.
    pub fn flex_comp(&mut self) {
        unsafe { mj_flex(self.model.ffi(), self.ffi_mut()) }
    }
    /// Compute tendon lengths, velocities and moment arms.
    pub fn tendon_comp(&mut self) {
        unsafe { mj_tendon(self.model.ffi(), self.ffi_mut()) }
    }
    /// Compute actuator transmission lengths and moments.
    pub fn transmission(&mut self) {
        unsafe { mj_transmission(self.model.ffi(), self.ffi_mut()) }
    }
    /// Run composite rigid body inertia algorithm (CRB).
    pub fn crb_comp(&mut self) {
        unsafe { mj_crb(self.model.ffi(), self.ffi_mut()) }
    }
    /// Make inertia matrix.
    pub fn make_m(&mut self) {
        unsafe { mj_makeM(self.model.ffi(), self.ffi_mut()) }
    }
    /// Compute sparse L'*D*L factorization of inertia matrix.
    pub fn factor_m(&mut self) {
        unsafe { mj_factorM(self.model.ffi(), self.ffi_mut()) }
    }
    /// Compute cvel, cdof_dot.
    pub fn com_vel(&mut self) {
        unsafe { mj_comVel(self.model.ffi(), self.ffi_mut()) }
    }
    /// Compute qfrc_passive from spring-dampers, gravity compensation and fluid forces.
    pub fn passive(&mut self) {
        unsafe { mj_passive(self.model.ffi(), self.ffi_mut()) }
    }
    /// Sub-tree linear velocity and angular momentum: compute subtree_linvel, subtree_angmom.
    pub fn subtree_vel(&mut self) {
        unsafe { mj_subtreeVel(self.model.ffi(), self.ffi_mut()) }
    }
    /// RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=false removes inertial term.
    /// Returns a newly allocated vector of `nv` elements. Wraps [`mj_rne`].
    pub fn rne(&mut self, flg_acc: bool) -> Vec<MjtNum> {
        let mut out = vec![0.0; self.model.ffi().nv as usize];
        self.rne_into(flg_acc, &mut out);
        out
    }
    /// Same as [`MjData::rne`], except it writes the `nv` elements into `result`.
    /// Elements of `result` above index `nv` keep their previous values.
    ///
    /// # Panics
    /// Panics if `result` holds fewer than `nv` elements.
    /// Use [`MjData::try_rne_into`] for a fallible alternative.
    pub fn rne_into(&mut self, flg_acc: bool, result: &mut [MjtNum]) {
        self.try_rne_into(flg_acc, result).unwrap()
    }
    /// Fallible version of [`MjData::rne_into`].
    ///
    /// # Errors
    /// Returns [`MjDataError::BufferTooSmall`] if `result.len() < nv`.
    pub fn try_rne_into(
        &mut self,
        flg_acc: bool,
        result: &mut [MjtNum],
    ) -> Result<(), MjDataError> {
        let nv = self.model.ffi().nv as usize;
        if result.len() < nv {
            return Err(MjDataError::BufferTooSmall {
                name: "result",
                got: result.len(),
                needed: nv,
            });
        }
        unsafe {
            mj_rne(
                self.model.ffi(),
                self.ffi_mut(),
                flg_acc as i32,
                result.as_mut_ptr(),
            )
        };
        Ok(())
    }
    /// RNE with complete data: compute cacc, cfrc_ext, cfrc_int.
    /// Wraps [`mj_rnePostConstraint`].
    pub fn rne_post_constraint(&mut self) {
        unsafe { mj_rnePostConstraint(self.model.ffi(), self.ffi_mut()) }
    }
    /// Run collision detection.
    pub fn collision(&mut self) {
        unsafe { mj_collision(self.model.ffi(), self.ffi_mut()) }
    }
    /// Construct constraints.
    pub fn make_constraint(&mut self) {
        unsafe { mj_makeConstraint(self.model.ffi(), self.ffi_mut()) }
    }
    /// Find constraint islands.
    pub fn island(&mut self) {
        unsafe { mj_island(self.model.ffi(), self.ffi_mut()) }
    }
    /// Compute inverse constraint inertia efc_AR.
    pub fn project_constraint(&mut self) {
        unsafe { mj_projectConstraint(self.model.ffi(), self.ffi_mut()) }
    }
    /// Compute efc_vel, efc_aref.
    pub fn reference_constraint(&mut self) {
        unsafe { mj_referenceConstraint(self.model.ffi(), self.ffi_mut()) }
    }
    /// Compute efc_state, efc_force, qfrc_constraint, and (optionally) cone Hessians.
    /// If cost is not `None`, set `*cost = s(jar)` where `jar = Jac*qacc - aref`.
    /// # Errors
    /// Returns [`MjDataError::BufferTooSmall`] if `jar.len() < nefc` (buffer too small).
    pub fn constraint_update(
        &mut self,
        jar: &[MjtNum],
        cost: Option<&mut MjtNum>,
        flg_cone_hessian: bool,
    ) -> Result<(), MjDataError> {
        let nefc = self.ffi().nefc as usize;
        if jar.len() < nefc {
            return Err(MjDataError::BufferTooSmall {
                name: "jar",
                got: jar.len(),
                needed: nefc,
            });
        }
        unsafe {
            mj_constraintUpdate(
                self.model.ffi(),
                self.ffi_mut(),
                jar.as_ptr(),
                cost.map_or(ptr::null_mut(), |x| x as *mut MjtNum),
                flg_cone_hessian as i32,
            )
        };
        Ok(())
    }
    /// Initializes the actuator history buffer for actuator `id` (wraps `mj_initCtrlHistory`).
    /// `times`: optional timestamps slice of length `nsample`; `None` keeps existing timestamps.
    /// `values`: control values slice of length `nsample`.
    /// # Note
    /// The timestamps must be strictly increasing, whether they come from `times` or from the
    /// existing buffer; otherwise MuJoCo reports an error and stops the process.
    /// # Errors
    /// - [`MjDataError::IndexOutOfBounds`] if `id >= nactuator`.
    /// - [`MjDataError::NoHistoryBuffer`] if the actuator has no history buffer.
    /// - [`MjDataError::LengthMismatch`] if `times` or `values` have the wrong length.
    pub fn init_ctrl_history(
        &mut self,
        id: usize,
        times: Option<&[MjtNum]>,
        values: &[MjtNum],
    ) -> Result<(), MjDataError> {
        let nactuator = self.model.ffi().nactuator as usize;
        if id >= nactuator {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "actuator_id",
                id,
                upper: nactuator,
            });
        }
        let nsample = self.model.actuator_history()[id][0];
        if nsample <= 0 {
            return Err(MjDataError::NoHistoryBuffer {
                kind: "actuator",
                id,
            });
        }
        let ns = nsample as usize;
        if let Some(t) = times
            && t.len() != ns
        {
            return Err(MjDataError::LengthMismatch {
                name: "times",
                expected: ns,
                got: t.len(),
            });
        }
        if values.len() != ns {
            return Err(MjDataError::LengthMismatch {
                name: "values",
                expected: ns,
                got: values.len(),
            });
        }
        unsafe {
            mj_initCtrlHistory(
                self.model.ffi(),
                self.ffi_mut(),
                id as i32,
                times.map_or(ptr::null(), |x| x.as_ptr()),
                values.as_ptr(),
            );
        }
        Ok(())
    }
    /// Initializes the sensor history buffer for sensor `id` (wraps `mj_initSensorHistory`).
    /// `times`: optional timestamps slice of length `nsample`; `None` keeps existing timestamps.
    /// `values`: sensor values slice of length `nsample * dim`.
    /// `phase`: time phase offset.
    /// # Note
    /// The timestamps must be strictly increasing, whether they come from `times` or from the
    /// existing buffer; otherwise MuJoCo reports an error and stops the process.
    /// # Errors
    /// - [`MjDataError::IndexOutOfBounds`] if `id >= nsensor`.
    /// - [`MjDataError::NoHistoryBuffer`] if the sensor has no history buffer.
    /// - [`MjDataError::LengthMismatch`] if `times` or `values` have the wrong length.
    pub fn init_sensor_history(
        &mut self,
        id: usize,
        times: Option<&[MjtNum]>,
        values: &[MjtNum],
        phase: MjtNum,
    ) -> Result<(), MjDataError> {
        let nsensor = self.model.ffi().nsensor as usize;
        if id >= nsensor {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "sensor_id",
                id,
                upper: nsensor,
            });
        }
        let nsample = self.model.sensor_history()[id][0];
        if nsample <= 0 {
            return Err(MjDataError::NoHistoryBuffer { kind: "sensor", id });
        }
        let dim = self.model.sensor_dim()[id] as usize;
        let required = (nsample as usize) * dim;
        if let Some(t) = times
            && t.len() != nsample as usize
        {
            return Err(MjDataError::LengthMismatch {
                name: "times",
                expected: nsample as usize,
                got: t.len(),
            });
        }
        if values.len() != required {
            return Err(MjDataError::LengthMismatch {
                name: "values",
                expected: required,
                got: values.len(),
            });
        }
        unsafe {
            mj_initSensorHistory(
                self.model.ffi(),
                self.ffi_mut(),
                id as i32,
                times.map_or(ptr::null(), |x| x.as_ptr()),
                values.as_ptr(),
                phase,
            );
        }
        Ok(())
    }
    /// Reads the control value for actuator `id` at `time`: the current `ctrl` entry when the
    /// actuator has no history buffer, otherwise the value from the history buffer
    /// (`interp`: -1=use the model's `interp` setting, 0=ZOH, 1=linear, 2=cubic).
    /// # Panics
    /// Panics when `id >= nactuator`. Use [`MjData::try_read_ctrl`] for a fallible alternative.
    pub fn read_ctrl(&self, id: usize, time: MjtNum, interp: i32) -> MjtNum {
        self.try_read_ctrl(id, time, interp).unwrap()
    }
    /// Fallible version of [`MjData::read_ctrl`].
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `id >= nactuator`.
    pub fn try_read_ctrl(
        &self,
        id: usize,
        time: MjtNum,
        interp: i32,
    ) -> Result<MjtNum, MjDataError> {
        let nactuator = self.model.ffi().nactuator as usize;
        if id >= nactuator {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "actuator_id",
                id,
                upper: nactuator,
            });
        }
        let val = unsafe { mj_readCtrl(self.model.ffi(), self.ffi(), id as i32, time, interp) };
        Ok(val)
    }
    /// Reads sensor `id` at `time` into `dst` (`interp`: -1=use the model's `interp` setting,
    /// 0=ZOH, 1=linear, 2=cubic).
    /// `dst` must be exactly `sensor_dim[id]` elements long.
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `id >= nsensor`.
    /// Returns [`MjDataError::LengthMismatch`] when `dst.len() != sensor_dim[id]`.
    pub fn read_sensor_into(
        &self,
        id: usize,
        time: MjtNum,
        interp: i32,
        dst: &mut [MjtNum],
    ) -> Result<(), MjDataError> {
        let nsensor = self.model.ffi().nsensor as usize;
        if id >= nsensor {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "sensor_id",
                id,
                upper: nsensor,
            });
        }
        let dim = self.model.sensor_dim()[id] as usize;
        if dst.len() != dim {
            return Err(MjDataError::LengthMismatch {
                name: "dst",
                expected: dim,
                got: dst.len(),
            });
        }
        let ptr = unsafe {
            mj_readSensor(
                self.model.ffi(),
                self.ffi(),
                id as i32,
                time,
                dst.as_mut_ptr(),
                interp,
            )
        };
        if !ptr.is_null() {
            dst.copy_from_slice(unsafe { std::slice::from_raw_parts(ptr, dim) });
        }
        Ok(())
    }
    /// Reads sensor `id` at `time` into a stack-allocated `[MjtNum; N]`
    /// (`interp`: -1=use the model's `interp` setting, 0=ZOH, 1=linear, 2=cubic). `N` must match `sensor_dim[id]`.
    /// See also [`read_sensor`](Self::read_sensor), [`read_sensor_into`](Self::read_sensor_into).
    /// # Panics
    /// Panics when `id >= nsensor` or `N != sensor_dim[id]`.
    /// Use [`MjData::try_read_sensor_fixed`] for a fallible alternative.
    pub fn read_sensor_fixed<const N: usize>(
        &self,
        id: usize,
        time: MjtNum,
        interp: i32,
    ) -> [MjtNum; N] {
        self.try_read_sensor_fixed(id, time, interp).unwrap()
    }
    /// Fallible version of [`MjData::read_sensor_fixed`].
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `id >= nsensor`.
    /// Returns [`MjDataError::LengthMismatch`] when `N != sensor_dim[id]`.
    pub fn try_read_sensor_fixed<const N: usize>(
        &self,
        id: usize,
        time: MjtNum,
        interp: i32,
    ) -> Result<[MjtNum; N], MjDataError> {
        let nsensor = self.model.ffi().nsensor as usize;
        if id >= nsensor {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "sensor_id",
                id,
                upper: nsensor,
            });
        }
        let dim = self.model.sensor_dim()[id] as usize;
        if N != dim {
            return Err(MjDataError::LengthMismatch {
                name: "N",
                expected: dim,
                got: N,
            });
        }
        let mut out = [0.0 as MjtNum; N];
        let ptr = unsafe {
            mj_readSensor(
                self.model.ffi(),
                self.ffi(),
                id as i32,
                time,
                out.as_mut_ptr(),
                interp,
            )
        };
        if !ptr.is_null() {
            out.copy_from_slice(unsafe { std::slice::from_raw_parts(ptr, N) });
        }
        Ok(out)
    }
    /// Reads sensor `id` at `time` (`interp`: -1=use the model's `interp` setting, 0=ZOH, 1=linear, 2=cubic).
    ///
    /// Returns [`Cow::Borrowed`] (zero-copy) for exact matches, ZOH, and extrapolation.
    /// Returns [`Cow::Owned`] for linear/cubic interpolation.
    /// See also [`read_sensor_fixed`](Self::read_sensor_fixed), [`read_sensor_into`](Self::read_sensor_into).
    /// # Panics
    /// Panics when `id >= nsensor`. Use [`MjData::try_read_sensor`] for a fallible alternative.
    pub fn read_sensor(&self, id: usize, time: MjtNum, interp: i32) -> Cow<'_, [MjtNum]> {
        self.try_read_sensor(id, time, interp).unwrap()
    }
    /// Fallible version of [`MjData::read_sensor`].
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `id >= nsensor`.
    pub fn try_read_sensor(
        &self,
        id: usize,
        time: MjtNum,
        interp: i32,
    ) -> Result<Cow<'_, [MjtNum]>, MjDataError> {
        let nsensor = self.model.ffi().nsensor as usize;
        if id >= nsensor {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "sensor_id",
                id,
                upper: nsensor,
            });
        }
        let dim = self.model.sensor_dim()[id] as usize;
        let mut out = vec![0.0 as MjtNum; dim];
        let ptr = unsafe {
            mj_readSensor(
                self.model.ffi(),
                self.ffi(),
                id as i32,
                time,
                out.as_mut_ptr(),
                interp,
            )
        };
        if !ptr.is_null() {
            Ok(Cow::Borrowed(unsafe {
                std::slice::from_raw_parts(ptr, dim)
            }))
        } else {
            Ok(Cow::Owned(out))
        }
    }
    /// Adds a contact to the contact list.
    ///
    /// This wraps `mj_addContact`, an advanced entry point intended for custom collision routines:
    /// it copies `con` into the data arena verbatim, without validating it.
    ///
    /// # Returns
    /// `Ok(())` on success.
    ///
    /// # Errors
    /// Returns [`MjDataError::ContactBufferFull`] if the contact buffer is full.
    ///
    /// # Safety
    /// The caller must ensure `con` is a valid contact for the model in this data. MuJoCo later
    /// indexes several of the stored contact's fields without any bounds check (when building
    /// constraints and when reading contact forces), so a malformed contact can cause out-of-bounds
    /// access.
    pub unsafe fn add_contact(&mut self, con: &MjContact) -> Result<(), MjDataError> {
        match unsafe { mj_addContact(self.model.ffi(), self.ffi_mut(), con) } {
            0 => Ok(()),
            _ => Err(MjDataError::ContactBufferFull),
        }
    }
    /// Compute 3/6-by-nv end-effector Jacobian of a global point attached to the given body.
    /// Set `jacp` to `true` to calculate the translational Jacobian and `jacr` to `true` for
    /// the rotational Jacobian. Returns a `(Vec, Vec)` for translation and rotation. Empty `Vec`s
    /// indicate that the corresponding Jacobian was not computed.
    /// # Panics
    /// Panics when `body_id >= nbody`. Use [`MjData::try_jac`] for a fallible alternative.
    pub fn jac(
        &self,
        jacp: bool,
        jacr: bool,
        point: &[MjtNum; 3],
        body_id: usize,
    ) -> (Vec<MjtNum>, Vec<MjtNum>) {
        self.try_jac(jacp, jacr, point, body_id).unwrap()
    }
    /// Fallible version of [`MjData::jac`].
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `body_id` is `>= nbody`.
    pub fn try_jac(
        &self,
        jacp: bool,
        jacr: bool,
        point: &[MjtNum; 3],
        body_id: usize,
    ) -> Result<(Vec<MjtNum>, Vec<MjtNum>), MjDataError> {
        let nbody = self.model.ffi().nbody;
        if body_id >= nbody as usize {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "body_id",
                id: body_id,
                upper: nbody as usize,
            });
        }
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = if jacp {
            vec![0 as MjtNum; required_len]
        } else {
            vec![]
        };
        let mut jacr_vec = if jacr {
            vec![0 as MjtNum; required_len]
        } else {
            vec![]
        };
        unsafe {
            mj_jac(
                self.model.ffi(),
                self.ffi(),
                if jacp {
                    jacp_vec.as_mut_ptr()
                } else {
                    ptr::null_mut()
                },
                if jacr {
                    jacr_vec.as_mut_ptr()
                } else {
                    ptr::null_mut()
                },
                point,
                body_id as i32,
            )
        };
        Ok((jacp_vec, jacr_vec))
    }
    /// Compute body frame end-effector Jacobian.
    /// Set `jacp`/`jacr` to `true` to calculate translational/rotational components.
    /// Returns `(Vec, Vec)` for translation and rotation. Empty `Vec`s indicate not computed.
    /// # Panics
    /// Panics when `body_id` is out of range. Use [`MjData::try_jac_body`] for a fallible alternative.
    pub fn jac_body(&self, jacp: bool, jacr: bool, body_id: usize) -> (Vec<MjtNum>, Vec<MjtNum>) {
        self.try_jac_body(jacp, jacr, body_id).unwrap()
    }
    /// Fallible version of [`MjData::jac_body`].
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `body_id` is out of range.
    pub fn try_jac_body(
        &self,
        jacp: bool,
        jacr: bool,
        body_id: usize,
    ) -> Result<(Vec<MjtNum>, Vec<MjtNum>), MjDataError> {
        let nbody = self.model.ffi().nbody;
        if body_id >= nbody as usize {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "body_id",
                id: body_id,
                upper: nbody as usize,
            });
        }
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = if jacp {
            vec![0 as MjtNum; required_len]
        } else {
            vec![]
        };
        let mut jacr_vec = if jacr {
            vec![0 as MjtNum; required_len]
        } else {
            vec![]
        };
        unsafe {
            mj_jacBody(
                self.model.ffi(),
                self.ffi(),
                if jacp {
                    jacp_vec.as_mut_ptr()
                } else {
                    ptr::null_mut()
                },
                if jacr {
                    jacr_vec.as_mut_ptr()
                } else {
                    ptr::null_mut()
                },
                body_id as i32,
            )
        };
        Ok((jacp_vec, jacr_vec))
    }
    /// Compute body center-of-mass end-effector Jacobian.
    /// Set `jacp`/`jacr` to `true` to calculate translational/rotational components.
    /// Returns `(Vec, Vec)` for translation and rotation. Empty `Vec`s indicate not computed.
    /// # Panics
    /// Panics when `body_id` is out of range. Use [`MjData::try_jac_body_com`] for a fallible alternative.
    pub fn jac_body_com(
        &self,
        jacp: bool,
        jacr: bool,
        body_id: usize,
    ) -> (Vec<MjtNum>, Vec<MjtNum>) {
        self.try_jac_body_com(jacp, jacr, body_id).unwrap()
    }
    /// Fallible version of [`MjData::jac_body_com`].
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `body_id` is out of range.
    pub fn try_jac_body_com(
        &self,
        jacp: bool,
        jacr: bool,
        body_id: usize,
    ) -> Result<(Vec<MjtNum>, Vec<MjtNum>), MjDataError> {
        let nbody = self.model.ffi().nbody;
        if body_id >= nbody as usize {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "body_id",
                id: body_id,
                upper: nbody as usize,
            });
        }
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = if jacp {
            vec![0 as MjtNum; required_len]
        } else {
            vec![]
        };
        let mut jacr_vec = if jacr {
            vec![0 as MjtNum; required_len]
        } else {
            vec![]
        };
        unsafe {
            mj_jacBodyCom(
                self.model.ffi(),
                self.ffi(),
                if jacp {
                    jacp_vec.as_mut_ptr()
                } else {
                    ptr::null_mut()
                },
                if jacr {
                    jacr_vec.as_mut_ptr()
                } else {
                    ptr::null_mut()
                },
                body_id as i32,
            )
        };
        Ok((jacp_vec, jacr_vec))
    }
    /// Compute subtree center-of-mass end-effector Jacobian (translational only).
    /// Returns a `Vec` of length `3 * nv` (row-major 3xnv matrix).
    /// # Panics
    /// Panics when `body_id` is out of range. Use [`MjData::try_jac_subtree_com`] for a fallible alternative.
    pub fn jac_subtree_com(&mut self, body_id: usize) -> Vec<MjtNum> {
        self.try_jac_subtree_com(body_id).unwrap()
    }
    /// Fallible version of [`MjData::jac_subtree_com`].
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `body_id` is out of range.
    pub fn try_jac_subtree_com(&mut self, body_id: usize) -> Result<Vec<MjtNum>, MjDataError> {
        let nbody = self.model.ffi().nbody;
        if body_id >= nbody as usize {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "body_id",
                id: body_id,
                upper: nbody as usize,
            });
        }
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = vec![0 as MjtNum; required_len];
        unsafe {
            mj_jacSubtreeCom(
                self.model.ffi(),
                self.ffi_mut(),
                jacp_vec.as_mut_ptr(),
                body_id as i32,
            )
        };
        Ok(jacp_vec)
    }
    /// Compute geom end-effector Jacobian.
    /// Set `jacp`/`jacr` to `true` to calculate translational/rotational components.
    /// Returns `(Vec, Vec)` for translation and rotation. Empty `Vec`s indicate not computed.
    /// # Panics
    /// Panics when `geom_id` is out of range. Use [`MjData::try_jac_geom`] for a fallible alternative.
    pub fn jac_geom(&self, jacp: bool, jacr: bool, geom_id: usize) -> (Vec<MjtNum>, Vec<MjtNum>) {
        self.try_jac_geom(jacp, jacr, geom_id).unwrap()
    }
    /// Fallible version of [`MjData::jac_geom`].
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `geom_id` is out of range.
    pub fn try_jac_geom(
        &self,
        jacp: bool,
        jacr: bool,
        geom_id: usize,
    ) -> Result<(Vec<MjtNum>, Vec<MjtNum>), MjDataError> {
        let ngeom = self.model.ffi().ngeom;
        if geom_id >= ngeom as usize {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "geom_id",
                id: geom_id,
                upper: ngeom as usize,
            });
        }
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = if jacp {
            vec![0 as MjtNum; required_len]
        } else {
            vec![]
        };
        let mut jacr_vec = if jacr {
            vec![0 as MjtNum; required_len]
        } else {
            vec![]
        };
        unsafe {
            mj_jacGeom(
                self.model.ffi(),
                self.ffi(),
                if jacp {
                    jacp_vec.as_mut_ptr()
                } else {
                    ptr::null_mut()
                },
                if jacr {
                    jacr_vec.as_mut_ptr()
                } else {
                    ptr::null_mut()
                },
                geom_id as i32,
            )
        };
        Ok((jacp_vec, jacr_vec))
    }
    /// Compute site end-effector Jacobian.
    /// Set `jacp`/`jacr` to `true` to calculate translational/rotational components.
    /// Returns `(Vec, Vec)` for translation and rotation. Empty `Vec`s indicate not computed.
    /// # Panics
    /// Panics when `site_id` is out of range. Use [`MjData::try_jac_site`] for a fallible alternative.
    pub fn jac_site(&self, jacp: bool, jacr: bool, site_id: usize) -> (Vec<MjtNum>, Vec<MjtNum>) {
        self.try_jac_site(jacp, jacr, site_id).unwrap()
    }
    /// Fallible version of [`MjData::jac_site`].
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `site_id` is out of range.
    pub fn try_jac_site(
        &self,
        jacp: bool,
        jacr: bool,
        site_id: usize,
    ) -> Result<(Vec<MjtNum>, Vec<MjtNum>), MjDataError> {
        let nsite = self.model.ffi().nsite;
        if site_id >= nsite as usize {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "site_id",
                id: site_id,
                upper: nsite as usize,
            });
        }
        let required_len = 3 * self.model.ffi().nv as usize;
        let mut jacp_vec = if jacp {
            vec![0 as MjtNum; required_len]
        } else {
            vec![]
        };
        let mut jacr_vec = if jacr {
            vec![0 as MjtNum; required_len]
        } else {
            vec![]
        };
        unsafe {
            mj_jacSite(
                self.model.ffi(),
                self.ffi(),
                if jacp {
                    jacp_vec.as_mut_ptr()
                } else {
                    ptr::null_mut()
                },
                if jacr {
                    jacr_vec.as_mut_ptr()
                } else {
                    ptr::null_mut()
                },
                site_id as i32,
            )
        };
        Ok((jacp_vec, jacr_vec))
    }
    /// Compute subtree angular momentum matrix.
    /// # Panics
    /// Panics when `body_id` is out of range. Use [`MjData::try_angmom_mat`] for a fallible alternative.
    pub fn angmom_mat(&mut self, body_id: usize) -> Vec<MjtNum> {
        self.try_angmom_mat(body_id).unwrap()
    }
    /// Fallible version of [`MjData::angmom_mat`].
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `body_id` is out of range.
    pub fn try_angmom_mat(&mut self, body_id: usize) -> Result<Vec<MjtNum>, MjDataError> {
        let nbody = self.model.ffi().nbody;
        if body_id >= nbody as usize {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "body_id",
                id: body_id,
                upper: nbody as usize,
            });
        }
        let mut mat = vec![0.0; 3 * self.model.ffi().nv as usize];
        unsafe {
            mj_angmomMat(
                self.model.ffi(),
                self.ffi_mut(),
                mat.as_mut_ptr(),
                body_id as i32,
            )
        };
        Ok(mat)
    }
    /// Run all kinematics-like computations (kinematics, comPos, camlight, flex, tendon).
    pub fn forward_kinematics(&mut self) {
        unsafe { mj_fwdKinematics(self.model.ffi(), self.ffi_mut()) }
    }
    /// Compute object 6D velocity (rot:lin) in object-centered frame, world/local orientation.
    /// # Panics
    /// Panics when `obj_type` is unsupported or `obj_id` is out of range.
    /// Use [`MjData::try_object_velocity`] for a fallible alternative.
    pub fn object_velocity(&self, obj_type: MjtObj, obj_id: usize, flg_local: bool) -> [MjtNum; 6] {
        self.try_object_velocity(obj_type, obj_id, flg_local)
            .unwrap()
    }
    /// Fallible version of [`MjData::object_velocity`].
    /// # Errors
    /// Returns:
    /// - [`MjDataError::UnsupportedObjectType`] when `obj_type` is not one of
    ///   `mjOBJ_BODY`, `mjOBJ_XBODY`, `mjOBJ_GEOM`, `mjOBJ_SITE`, `mjOBJ_CAMERA`.
    /// - [`MjDataError::IndexOutOfBounds`] when `obj_id` is out of range for the given type.
    pub fn try_object_velocity(
        &self,
        obj_type: MjtObj,
        obj_id: usize,
        flg_local: bool,
    ) -> Result<[MjtNum; 6], MjDataError> {
        let max_id = match obj_type {
            MjtObj::mjOBJ_BODY | MjtObj::mjOBJ_XBODY => self.model.ffi().nbody,
            MjtObj::mjOBJ_GEOM => self.model.ffi().ngeom,
            MjtObj::mjOBJ_SITE => self.model.ffi().nsite,
            MjtObj::mjOBJ_CAMERA => self.model.ffi().ncam,
            _ => return Err(MjDataError::UnsupportedObjectType(obj_type as i32)),
        };
        if obj_id >= max_id as usize {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "obj_id",
                id: obj_id,
                upper: max_id as usize,
            });
        }
        let mut result: [MjtNum; 6] = [0.0; 6];
        unsafe {
            mj_objectVelocity(
                self.model.ffi(),
                self.ffi(),
                obj_type as i32,
                obj_id as i32,
                &mut result,
                flg_local as i32,
            )
        };
        Ok(result)
    }
    /// Compute object 6D acceleration (rot:lin) in object-centered frame, world/local orientation.
    /// # Panics
    /// Panics when `obj_type` is unsupported or `obj_id` is out of range.
    /// Use [`MjData::try_object_acceleration`] for a fallible alternative.
    pub fn object_acceleration(
        &self,
        obj_type: MjtObj,
        obj_id: usize,
        flg_local: bool,
    ) -> [MjtNum; 6] {
        self.try_object_acceleration(obj_type, obj_id, flg_local)
            .unwrap()
    }
    /// Fallible version of [`MjData::object_acceleration`].
    /// # Errors
    /// Returns:
    /// - [`MjDataError::UnsupportedObjectType`] when `obj_type` is not supported.
    /// - [`MjDataError::IndexOutOfBounds`] when `obj_id` is out of range for the given type.
    pub fn try_object_acceleration(
        &self,
        obj_type: MjtObj,
        obj_id: usize,
        flg_local: bool,
    ) -> Result<[MjtNum; 6], MjDataError> {
        let max_id = match obj_type {
            MjtObj::mjOBJ_BODY | MjtObj::mjOBJ_XBODY => self.model.ffi().nbody,
            MjtObj::mjOBJ_GEOM => self.model.ffi().ngeom,
            MjtObj::mjOBJ_SITE => self.model.ffi().nsite,
            MjtObj::mjOBJ_CAMERA => self.model.ffi().ncam,
            _ => return Err(MjDataError::UnsupportedObjectType(obj_type as i32)),
        };
        if obj_id >= max_id as usize {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "obj_id",
                id: obj_id,
                upper: max_id as usize,
            });
        }
        let mut result: [MjtNum; 6] = [0.0; 6];
        unsafe {
            mj_objectAcceleration(
                self.model.ffi(),
                self.ffi(),
                obj_type as i32,
                obj_id as i32,
                &mut result,
                flg_local as i32,
            )
        };
        Ok(result)
    }
    /// Returns smallest signed distance between two geoms and optionally the segment from geom1 to geom2.
    /// # Panics
    /// Panics when either geom id is `>= ngeom`. Use [`MjData::try_geom_distance`] for a fallible alternative.
    pub fn geom_distance(
        &mut self,
        geom1_id: usize,
        geom2_id: usize,
        dist_max: MjtNum,
        fromto: Option<&mut [MjtNum; 6]>,
    ) -> MjtNum {
        self.try_geom_distance(geom1_id, geom2_id, dist_max, fromto)
            .unwrap()
    }
    /// Fallible version of [`MjData::geom_distance`].
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when either geom id is `>= ngeom`.
    pub fn try_geom_distance(
        &mut self,
        geom1_id: usize,
        geom2_id: usize,
        dist_max: MjtNum,
        fromto: Option<&mut [MjtNum; 6]>,
    ) -> Result<MjtNum, MjDataError> {
        let ngeom = self.model.ffi().ngeom;
        if geom1_id >= ngeom as usize {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "geom1_id",
                id: geom1_id,
                upper: ngeom as usize,
            });
        }
        if geom2_id >= ngeom as usize {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "geom2_id",
                id: geom2_id,
                upper: ngeom as usize,
            });
        }
        Ok(unsafe {
            mj_geomDistance(
                self.model.ffi(),
                self.ffi_mut(),
                geom1_id as i32,
                geom2_id as i32,
                dist_max,
                fromto.map_or(ptr::null_mut(), |x| x),
            )
        })
    }
    /// Map from body local to global Cartesian coordinates. Returns (global position, global orientation matrix).
    /// `sameframe` takes values from [`MjtSameFrame`]. Wraps `mj_local2Global`.
    /// # Panics
    /// Panics when `body_id` is out of range. Use [`MjData::try_local_to_global`] for a fallible alternative.
    pub fn local_to_global(
        &mut self,
        pos: &[MjtNum; 3],
        quat: &[MjtNum; 4],
        body_id: usize,
        sameframe: MjtSameFrame,
    ) -> ([MjtNum; 3], [MjtNum; 9]) {
        self.try_local_to_global(pos, quat, body_id, sameframe)
            .unwrap()
    }
    /// Fallible version of [`MjData::local_to_global`].
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] when `body_id` is out of range.
    pub fn try_local_to_global(
        &mut self,
        pos: &[MjtNum; 3],
        quat: &[MjtNum; 4],
        body_id: usize,
        sameframe: MjtSameFrame,
    ) -> Result<([MjtNum; 3], [MjtNum; 9]), MjDataError> {
        let nbody = self.model.ffi().nbody;
        if body_id >= nbody as usize {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "body_id",
                id: body_id,
                upper: nbody as usize,
            });
        }
        let mut xpos: [MjtNum; 3] = [0.0; 3];
        let mut xmat: [MjtNum; 9] = [0.0; 9];
        unsafe {
            mj_local2Global(
                self.ffi_mut(),
                &mut xpos,
                &mut xmat,
                pos,
                quat,
                body_id as i32,
                sameframe as MjtByte,
            )
        };
        Ok((xpos, xmat))
    }
    /// Intersect multiple rays emanating from a single point.
    /// Similar semantics to mj_ray, but `vec` is an array of (nray x 3) directions.
    /// If `normals_out` is `Some`, it must be a slice of `nray` elements filled with surface normals. Use `None` to skip normals.
    /// # Panics
    /// Panics if `normals_out` length does not match `vec.len()`.
    /// Use [`MjData::try_multi_ray`] for a fallible alternative.
    #[allow(clippy::too_many_arguments)]
    pub fn multi_ray(
        &mut self,
        pnt: &[MjtNum; 3],
        vec: &[[MjtNum; 3]],
        geomgroup: Option<&[MjtByte; mjNGROUP as usize]>,
        flg_static: MjtBool,
        bodyexclude: Option<usize>,
        cutoff: MjtNum,
        normals_out: Option<&mut [[MjtNum; 3]]>,
    ) -> (Vec<Option<usize>>, Vec<MjtNum>) {
        self.try_multi_ray(
            pnt,
            vec,
            geomgroup,
            flg_static,
            bodyexclude,
            cutoff,
            normals_out,
        )
        .unwrap()
    }
    /// Fallible version of [`MjData::multi_ray`].
    /// # Errors
    /// Returns [`MjDataError::LengthMismatch`] if `normals_out` length does not match `vec.len()`.
    #[allow(clippy::too_many_arguments)]
    pub fn try_multi_ray(
        &mut self,
        pnt: &[MjtNum; 3],
        vec: &[[MjtNum; 3]],
        geomgroup: Option<&[MjtByte; mjNGROUP as usize]>,
        flg_static: MjtBool,
        bodyexclude: Option<usize>,
        cutoff: MjtNum,
        normals_out: Option<&mut [[MjtNum; 3]]>,
    ) -> Result<(Vec<Option<usize>>, Vec<MjtNum>), MjDataError> {
        let nray = vec.len();
        if let Some(buf) = &normals_out
            && buf.len() != nray
        {
            return Err(MjDataError::LengthMismatch {
                name: "normals_out",
                expected: nray,
                got: buf.len(),
            });
        }
        let mut geom_id_raw = vec![-1i32; nray];
        let mut distance = vec![0.0; nray];
        unsafe {
            mj_multiRay(
                self.model.ffi(),
                self.ffi_mut(),
                pnt,
                bytemuck::cast_slice::<[MjtNum; 3], MjtNum>(vec).as_ptr(),
                geomgroup.map_or(ptr::null(), |x| x.as_ptr()),
                flg_static,
                bodyexclude.map_or(-1i32, |id| id as i32),
                geom_id_raw.as_mut_ptr(),
                distance.as_mut_ptr(),
                normals_out.map_or(ptr::null_mut(), |x| {
                    bytemuck::cast_slice_mut::<[MjtNum; 3], MjtNum>(x).as_mut_ptr()
                }),
                nray as i32,
                cutoff,
            )
        };
        let geom_id = geom_id_raw
            .into_iter()
            .map(|id| if id == -1 { None } else { Some(id as usize) })
            .collect();
        Ok((geom_id, distance))
    }
    /// Intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms in bodyexclude.
    /// Returns `(geomid, distance)` where distance is -1 if no intersection.
    /// If `normal_out` is `Some`, it will be filled with the surface normal at the intersection.
    /// `geomgroup` and `flg_static` are as in mjvOption; pass `None` for `geomgroup` to skip group exclusion.
    /// A `vec` shorter than `mjMINVAL` reports no intersection, as [`MjData::multi_ray`] does.
    pub fn ray(
        &mut self,
        pnt: &[MjtNum; 3],
        vec: &[MjtNum; 3],
        geomgroup: Option<&[MjtByte; mjNGROUP as usize]>,
        flg_static: MjtBool,
        bodyexclude: Option<usize>,
        normal_out: Option<&mut [MjtNum; 3]>,
    ) -> (Option<usize>, MjtNum) {
        if mju_norm_3(vec) < mjMINVAL {
            if let Some(normal) = normal_out {
                *normal = [0.0; 3];
            }
            return (None, -1.0);
        }
        let mut geom_id_raw = -1i32;
        let dist = unsafe {
            mj_ray(
                self.model.ffi(),
                self.ffi(),
                pnt,
                vec,
                geomgroup.map_or(ptr::null(), |x| x.as_ptr()),
                flg_static,
                bodyexclude.map_or(-1i32, |id| id as i32),
                &mut geom_id_raw,
                normal_out.map_or(ptr::null_mut(), |x| x),
            )
        };
        let geom_id = if geom_id_raw == -1 {
            None
        } else {
            Some(geom_id_raw as usize)
        };
        (geom_id, dist)
    }
    /// Intersect ray with visible flexes.
    /// Return distance to nearest surface, or -1 if no intersection.
    /// If `vertid` is `Some`, it will be filled with the id of the nearest vertex.
    /// If `normal_out` is `Some`, it will be filled with the surface normal at the intersection.
    /// `flex_layer`, `flg_vert`, `flg_edge`, `flg_face`, `flg_skin` and `flexid` control what and where to intersect.
    ///
    /// # Panics
    /// Panics if `flexid` is out of bounds (must be `0 <= flexid < nflex`).
    ///
    /// Use [`MjData::try_ray_flex`] for a fallible alternative.
    #[allow(clippy::too_many_arguments)]
    pub fn ray_flex(
        &self,
        flex_layer: i32,
        flg_vert: MjtBool,
        flg_edge: MjtBool,
        flg_face: MjtBool,
        flg_skin: MjtBool,
        flexid: usize,
        pnt: &[MjtNum; 3],
        vec: &[MjtNum; 3],
        vertid: Option<&mut i32>,
        normal_out: Option<&mut [MjtNum; 3]>,
    ) -> MjtNum {
        self.try_ray_flex(
            flex_layer, flg_vert, flg_edge, flg_face, flg_skin, flexid, pnt, vec, vertid,
            normal_out,
        )
        .unwrap()
    }
    /// Intersect ray with flex, returning the distance or -1.0 if no intersection.
    ///
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] if `flexid >= nflex`.
    ///
    /// Use [`MjData::ray_flex`] for a panicking alternative.
    #[allow(clippy::too_many_arguments)]
    pub fn try_ray_flex(
        &self,
        flex_layer: i32,
        flg_vert: MjtBool,
        flg_edge: MjtBool,
        flg_face: MjtBool,
        flg_skin: MjtBool,
        flexid: usize,
        pnt: &[MjtNum; 3],
        vec: &[MjtNum; 3],
        vertid: Option<&mut i32>,
        normal_out: Option<&mut [MjtNum; 3]>,
    ) -> Result<MjtNum, MjDataError> {
        let nflex = self.model.ffi().nflex as usize;
        if flexid >= nflex {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "flexid",
                id: flexid,
                upper: nflex,
            });
        }
        Ok(unsafe {
            mj_rayFlex(
                self.model.ffi(),
                self.ffi(),
                flex_layer,
                flg_vert,
                flg_edge,
                flg_face,
                flg_skin,
                flexid as i32,
                pnt,
                vec,
                vertid.map_or(ptr::null_mut(), |x| x),
                normal_out.map_or(ptr::null_mut(), |x| x),
            )
        })
    }
    /// Copies data state from `src` to `self` based on the specified `spec` combination of `mjtState` flags.
    ///
    /// # Errors
    /// Returns [`MjDataError::IncompatibleModel`] if `src` was created from a model that is not
    /// compatible with this data's model (see [`MjModel::is_compatible_with_model`]).
    pub fn copy_state_from_data<N: ModelType>(
        &mut self,
        src: &MjData<N>,
        spec: u32,
    ) -> Result<(), MjDataError> {
        if !self.model.is_compatible_with_model(&src.model) {
            return Err(MjDataError::IncompatibleModel {
                source: src.model.signature(),
                destination: self.model.signature(),
            });
        }
        unsafe {
            mj_copyState(self.model.ffi(), src.ffi(), self.ffi_mut(), spec as i32);
        }
        Ok(())
    }
    /// Intersect ray with hfield.
    /// Returns the distance to the intersection, or -1.0 if no intersection.
    /// # Note
    /// The geom must be of type `mjGEOM_HFIELD`; for any other type MuJoCo reports an error and
    /// stops the process.
    ///
    /// # Panics
    /// Panics if `geom_id` is out of bounds (must be `0 <= geom_id < ngeom`).
    ///
    /// Use [`MjData::try_ray_hfield`] for a fallible alternative.
    pub fn ray_hfield(
        &self,
        geom_id: usize,
        pnt: &[MjtNum; 3],
        vec: &[MjtNum; 3],
        normal_out: Option<&mut [MjtNum; 3]>,
    ) -> MjtNum {
        self.try_ray_hfield(geom_id, pnt, vec, normal_out).unwrap()
    }
    /// Intersect ray with hfield, returning the distance or -1.0 if no intersection.
    /// # Note
    /// The geom must be of type `mjGEOM_HFIELD`; for any other type MuJoCo reports an error and
    /// stops the process.
    ///
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] if `geom_id >= ngeom`.
    ///
    /// Use [`MjData::ray_hfield`] for a panicking alternative.
    pub fn try_ray_hfield(
        &self,
        geom_id: usize,
        pnt: &[MjtNum; 3],
        vec: &[MjtNum; 3],
        normal_out: Option<&mut [MjtNum; 3]>,
    ) -> Result<MjtNum, MjDataError> {
        let ngeom = self.model.ffi().ngeom as usize;
        if geom_id >= ngeom {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "geom_id",
                id: geom_id,
                upper: ngeom,
            });
        }
        Ok(unsafe {
            mj_rayHfield(
                self.model.ffi(),
                self.ffi(),
                geom_id as i32,
                pnt,
                vec,
                normal_out.map_or(ptr::null_mut(), |x| x),
            )
        })
    }
    /// Intersect ray with mesh.
    /// Returns the distance to the intersection, or -1.0 if no intersection.
    /// # Note
    /// The geom must be of type `mjGEOM_MESH`; for any other type MuJoCo reports an error and
    /// stops the process.
    ///
    /// # Panics
    /// Panics if `geom_id` is out of bounds (must be `0 <= geom_id < ngeom`).
    ///
    /// Use [`MjData::try_ray_mesh`] for a fallible alternative.
    pub fn ray_mesh(
        &mut self,
        geom_id: usize,
        pnt: &[MjtNum; 3],
        vec: &[MjtNum; 3],
        normal_out: Option<&mut [MjtNum; 3]>,
    ) -> MjtNum {
        self.try_ray_mesh(geom_id, pnt, vec, normal_out).unwrap()
    }
    /// Intersect ray with mesh, returning the distance or -1.0 if no intersection.
    /// # Note
    /// The geom must be of type `mjGEOM_MESH`; for any other type MuJoCo reports an error and
    /// stops the process.
    ///
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] if `geom_id >= ngeom`.
    ///
    /// Use [`MjData::ray_mesh`] for a panicking alternative.
    pub fn try_ray_mesh(
        &mut self,
        geom_id: usize,
        pnt: &[MjtNum; 3],
        vec: &[MjtNum; 3],
        normal_out: Option<&mut [MjtNum; 3]>,
    ) -> Result<MjtNum, MjDataError> {
        let ngeom = self.model.ffi().ngeom as usize;
        if geom_id >= ngeom {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "geom_id",
                id: geom_id,
                upper: ngeom,
            });
        }
        Ok(unsafe {
            mj_rayMesh(
                self.model.ffi(),
                self.ffi(),
                geom_id as i32,
                pnt,
                vec,
                normal_out.map_or(ptr::null_mut(), |x| x),
            )
        })
    }
    /// Apply Cartesian force and torque to a point on a body, and add the result to `qfrc_target`.
    ///
    /// # Errors
    /// Returns [`MjDataError::IndexOutOfBounds`] if `body` is not a valid body
    /// index, or [`MjDataError::BufferTooSmall`] if `qfrc_target` is shorter
    /// than `nv`.
    pub fn apply_ft(
        &mut self,
        force: &[MjtNum; 3],
        torque: &[MjtNum; 3],
        point: &[MjtNum; 3],
        body: usize,
        qfrc_target: &mut [MjtNum],
    ) -> Result<(), MjDataError> {
        let nbody = self.model.ffi().nbody;
        if body >= nbody as usize {
            return Err(MjDataError::IndexOutOfBounds {
                kind: "body",
                id: body,
                upper: nbody as usize,
            });
        }
        let nv = self.model.ffi().nv as usize;
        if qfrc_target.len() < nv {
            return Err(MjDataError::BufferTooSmall {
                name: "qfrc_target",
                got: qfrc_target.len(),
                needed: nv,
            });
        }
        unsafe {
            mj_applyFT(
                self.model.ffi(),
                self.ffi_mut(),
                force,
                torque,
                point,
                body as i32,
                qfrc_target.as_mut_ptr(),
            );
        }
        Ok(())
    }
    /// Reads data's state into `destination`. The `spec` parameter is a bit mask of [`MjtState`] elements,
    /// which controls what state gets copied. The `destination` parameter is a mutable
    /// slice to the location into which the state will be written.
    /// Wraps [`mj_getState`].
    ///
    /// # Note
    /// The `destination` buffer is allowed to be larger than the
    /// actual state length, and may thus contain old information.
    /// Only the first `state_size` elements of `destination` are updated by this function;
    /// any remaining elements in the buffer are left unchanged. This was done for possible
    /// performance improvements, where one array may hold different parts of simulation state
    /// at different times.
    ///
    /// You can use the returned number of [`MjtNum`] elements written to `destination`
    /// to create a subslice containing only the updated information.
    ///
    /// # Returns
    /// Number of [`MjtNum`] elements written to `destination`.
    ///
    /// # Panics
    /// A panic will occur if `destination` is smaller than [`MjModel::state_size`] with `spec` passed as parameter.
    /// Use [`MjData::try_read_state_into`] for a fallible alternative.
    pub fn read_state_into(&self, spec: u32, destination: &mut [MjtNum]) -> usize {
        self.try_read_state_into(spec, destination).unwrap()
    }
    /// Fallible version of [`MjData::read_state_into`].
    ///
    /// # Errors
    /// Returns [`MjDataError::BufferTooSmall`] if `destination` is smaller than
    /// the state size required by `spec`.
    ///
    /// On success, returns the number of [`MjtNum`] elements written.
    pub fn try_read_state_into(
        &self,
        spec: u32,
        destination: &mut [MjtNum],
    ) -> Result<usize, MjDataError> {
        let state_size = self.model.state_size(spec);
        if destination.len() < state_size {
            return Err(MjDataError::BufferTooSmall {
                name: "destination",
                got: destination.len(),
                needed: state_size,
            });
        }
        unsafe {
            mj_getState(
                self.model.ffi(),
                self.ffi(),
                destination.as_mut_ptr(),
                spec as i32,
            );
        }
        Ok(state_size)
    }
    /// Same as [`MjData::read_state_into`], except it allocates
    /// and returns new boxed data containing the state.
    pub fn state(&self, spec: u32) -> Box<[MjtNum]> {
        let mut destination = vec![0.0; self.model.state_size(spec)].into_boxed_slice();
        Self::read_state_into(self, spec, &mut destination);
        destination
    }
    /// Sets the `state` to [`MjData`]. Wraps [`mj_setState`].
    /// The `state` is an array containing the state to write, based on the `spec`
    /// bitmask of elements [`MjtState`].
    ///
    /// # Note
    /// The size of `state` is allowed to be larger. This was done to allow a preallocated
    /// buffer to store any possible state based on `spec`, without having to query the size
    /// every time. This benefits performance in some cases.
    ///
    /// # Errors
    /// - [`MjDataError::BufferTooSmall`] if `state` is smaller than the length required by `spec`.
    /// - [`MjDataError::InvalidHistoryCursor`] if `spec` selects [`MjtState::mjSTATE_HISTORY`]
    ///   and a cursor slot in `state` lies outside `0..nsample` for its buffer. The history
    ///   buffer keeps its previous contents; every other selected component stays written.
    pub fn set_state(&mut self, state: &[MjtNum], spec: u32) -> Result<(), MjDataError> {
        let required_len = self.model.state_size(spec);
        if state.len() < required_len {
            return Err(MjDataError::BufferTooSmall {
                name: "state",
                got: state.len(),
                needed: required_len,
            });
        }
        let previous =
            (spec & MjtState::mjSTATE_HISTORY as u32 != 0).then(|| self.history().to_vec());
        unsafe {
            mj_setState(
                self.model.ffi(),
                self.ffi_mut(),
                state.as_ptr(),
                spec as i32,
            );
        }
        if let Some(previous) = previous
            && let Some((kind, id, nsample)) = self.history_cursor_fault()
        {
            unsafe { self.history_mut() }.copy_from_slice(&previous);
            return Err(MjDataError::InvalidHistoryCursor { kind, id, nsample });
        }
        Ok(())
    }
    /// Returns the first history buffer whose cursor slot is outside `0..nsample`, as
    /// `(kind, id, nsample)`, or `None` when every cursor is valid.
    ///
    /// The cursor sits at offset 1 of each buffer, after the user slot.
    fn history_cursor_fault(&self) -> Option<(&'static str, usize, usize)> {
        let history = self.history();
        let fault = |kind, spec: &[[i32; 2]], adr: &[i32]| {
            spec.iter()
                .zip(adr)
                .enumerate()
                .find_map(|(id, (sample, &address))| {
                    let nsample = sample[0];
                    if nsample <= 0 || address < 0 {
                        return None;
                    }
                    let cursor = *history.get(address as usize + 1)?;
                    let valid =
                        cursor.fract() == 0.0 && cursor >= 0.0 && cursor <= f64::from(nsample - 1);
                    (!valid).then_some((kind, id, nsample as usize))
                })
        };
        fault(
            "actuator",
            self.model.actuator_history(),
            self.model.actuator_historyadr(),
        )
        .or_else(|| {
            fault(
                "sensor",
                self.model.sensor_history(),
                self.model.sensor_historyadr(),
            )
        })
    }
    /// Convert sparse inertia matrix into full (i.e. dense) matrix. Wraps [`mj_fullM`].
    ///
    /// # Errors
    /// Returns [`MjDataError::BufferTooSmall`] if `dst.len() < nv * nv`.
    pub fn full_m(&self, dst: &mut [MjtNum]) -> Result<(), MjDataError> {
        let nv = self.model.ffi().nv as usize;
        let needed = nv * nv;
        if dst.len() < needed {
            return Err(MjDataError::BufferTooSmall {
                name: "dst",
                got: dst.len(),
                needed,
            });
        }
        unsafe { mj_fullM(self.model.ffi(), self.ffi(), dst.as_mut_ptr()) };
        Ok(())
    }
    /// Create a thread pool with `nthread` worker threads. Wraps [`mju_threadpool`].
    pub fn set_threadpool(&mut self, nthread: i32) {
        unsafe { mju_threadpool(self.ffi_mut(), nthread) }
    }
    /// Copy [`MjData`] to `destination`, skipping large computed arrays not required for
    /// visualization: the mass and factorization matrices (`crb`, `M`, `qLD`, `qH`, `qDeriv`,
    /// `qLU`) and the sparse constraint Jacobian blocks (`efc_J_*`, `efc_Y_*`, `efc_AR_*`).
    /// Wraps [`mjv_copyData`].
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process when this data's stack is in use.
    ///
    /// # Errors
    /// Returns [`MjDataError::IncompatibleModel`] if `destination` was created from a model that
    /// is not compatible with this data's model (see [`MjModel::is_compatible_with_model`]).
    pub fn copy_visual_to<N: ModelType>(
        &self,
        destination: &mut MjData<N>,
    ) -> Result<(), MjDataError> {
        if !self.model.is_compatible_with_model(&destination.model) {
            return Err(MjDataError::IncompatibleModel {
                source: self.model.signature(),
                destination: destination.model.signature(),
            });
        }
        unsafe {
            mjv_copyData(destination.ffi_mut(), self.model.ffi(), self.ffi());
        }
        Ok(())
    }
    /// Copy [`MjData`] to `destination` in full.
    /// Wraps [`mj_copyData`].
    ///
    /// # Note
    /// MuJoCo reports an error and stops the process when this data's stack is in use.
    ///
    /// # Errors
    /// Returns [`MjDataError::IncompatibleModel`] if `destination` was created from a model that
    /// is not compatible with this data's model (see [`MjModel::is_compatible_with_model`]).
    pub fn copy_to<N: ModelType>(&self, destination: &mut MjData<N>) -> Result<(), MjDataError> {
        if !self.model.is_compatible_with_model(&destination.model) {
            return Err(MjDataError::IncompatibleModel {
                source: self.model.signature(),
                destination: destination.model.signature(),
            });
        }
        unsafe {
            mj_copyData(destination.ffi_mut(), self.model.ffi(), self.ffi());
        }
        Ok(())
    }
}
/// Some public attribute methods.
impl<M: ModelType> MjData<M> {
    /// Reference to the wrapped FFI struct.
    pub fn ffi(&self) -> &mjData {
        unsafe { self.data.as_ref() }
    }
    /// Mutable reference to the wrapped FFI struct.
    ///
    /// # Safety
    /// Modifying the underlying FFI struct directly can break the invariants
    /// upheld by the `mujoco-rs` wrappers and cause undefined behavior.
    pub unsafe fn ffi_mut(&mut self) -> &mut mjData {
        unsafe { self.data.as_mut() }
    }
    /// Returns a reference to data's [`MjModel`].
    ///
    /// See also [`model_mut`](MjData::model_mut) for mutable access
    /// (requires `M: ModelTypeMut`).
    pub fn model(&self) -> &MjModel {
        &self.model
    }
    /// Returns an immutable reference to the model physics options.
    pub fn model_opt(&self) -> &MjOption {
        self.model.opt()
    }
    /// Returns an immutable reference to the model visualization options.
    pub fn model_vis(&self) -> &MjVisual {
        self.model.vis()
    }
    /// Returns an immutable reference to the model statistics.
    pub fn model_stat(&self) -> &MjStatistic {
        self.model.stat()
    }
    /// Returns a clone of the stored model.
    /// Unlike [`model`](Self::model), this returns
    /// the inferred `M` type (cloned).
    pub fn model_clone(&self) -> M
    where
        M: Clone,
    {
        self.model.clone()
    }
    getter_setter! {
        get, [[ffi] narena : MjtSize;
        "size of the arena in bytes (inclusive of the stack)."; [ffi] nbuffer : MjtSize;
        "size of main buffer in bytes."; [ffi] nplugin : i32;
        "number of plugin instances."; [ffi] maxuse_stack : MjtSize;
        "maximum stack allocation in bytes (mutable)."; [ffi] maxuse_arena : MjtSize;
        "maximum arena allocation in bytes."; [ffi] maxuse_con : i32;
        "maximum number of contacts."; [ffi] maxuse_efc : i32;
        "maximum number of scalar constraints."; [ffi] ncon : i32;
        "number of detected contacts."; [ffi] ne : i32;
        "number of equality constraints."; [ffi] nf : i32;
        "number of friction constraints."; [ffi] nl : i32;
        "number of limit constraints."; [ffi] nefc : i32; "number of constraints."; [ffi]
        nJ : i32; "number of non-zeros in constraint Jacobian."; [ffi] nefmK : i32;
        "number of non-zeros in effective-stiffness CSR."; [ffi] nefmdof : i32;
        "number of 3x3 blocks in the effective-metric preconditioner."; [ffi] nefmL :
        i32; "size of the effective-metric block storage (9*nefmdof)."; [ffi] nY : i32;
        "number of non-zeros in constraint inverse inertia square root."; [ffi] nA : i32;
        "number of non-zeros in constraint inverse inertia matrix."; [ffi] nisland : i32;
        "number of detected constraint islands."; [ffi] nidof : i32;
        "number of dofs in all islands."; [ffi] ntree_awake : i32;
        "number of awake trees."; [ffi] nbody_awake : i32;
        "number of awake dynamic and static bodies."; [ffi] nparent_awake : i32;
        "number of bodies with awake parents."; [ffi] nv_awake : i32;
        "number of awake dofs."; [ffi] signature : u64; "compilation signature.";]
    }
    /// Returns the memory layout snapshot of the model this data belongs to.
    ///
    /// The buffers were allocated for the model that created this data. That model and
    /// [`MjData::model`] share one layout, because [`MjData::try_swap_model`] rejects a model
    /// that does not.
    pub(crate) fn layout(&self) -> &Arc<MjModelLayout> {
        self.model.layout()
    }
    getter_setter! {
        get, [[ffi] efm_active : bool;
        "whether the implicit effective metric M+K is active.";]
    }
    getter_setter! {
        get, [[ffi] flg_energypos : MjtBool; "has mj_energyPos been called."; [ffi]
        flg_energyvel : MjtBool; "has mj_energyVel been called."; [ffi] flg_subtreevel :
        MjtBool; "has mj_subtreeVel been called."; [ffi] flg_rnepost : MjtBool;
        "has mj_rnePostConstraint been called.";]
    }
    getter_setter! {
        with, get, set, [[ffi, ffi_mut] time : MjtNum; "simulation time."; [ffi, ffi_mut]
        threadlock : MjtBool; "disable stack freeing during threaded execution.";]
    }
    getter_setter! {
        with, get, [[ffi, ffi_mut] energy : & [MjtNum; 2]; "potential, kinetic energy.";]
    }
    getter_setter! {
        get, [[ffi, ffi_mut] solver : & [MjSolverStat; mjNISLAND as usize * mjNSOLVER as
        usize]; "solver statistics per island, per iteration."; [ffi, ffi_mut]
        solver_niter : & [i32; mjNISLAND as usize];
        "number of solver iterations, per island."; [ffi, ffi_mut] solver_nnz : & [i32;
        mjNISLAND as usize]; "number of nonzeros in solver matrix, per island."; [ffi,
        ffi_mut] solver_fwdinv : & [MjtNum; 2]; "forward-inverse comparison: qfrc, efc.";
        [ffi, ffi_mut] warning : & [MjWarningStat; MjtWarning::mjNWARNING as usize];
        "warning statistics (mutable)."; [ffi, ffi_mut] timer : & [MjTimerStat;
        MjtTimer::mjNTIMER as usize]; "timer statistics.";]
    }
}
impl<M: ModelTypeMut> MjData<M> {
    /// Returns a mutable reference to data's [`MjModel`].
    ///
    /// This is useful for modifying the physics parameters of the model
    /// (e.g., timestep, gravity) without having to rebuild the simulation.
    ///
    /// **Not all model parameters are safe to change at runtime.**
    /// See [MuJoCo's documentation](https://mujoco.readthedocs.io/en/3.12.0/programming/simulation.html#mjmodel-changes)
    /// for a list of parameters that are safe to change.
    ///
    /// Only available when the inner model type `M` implements [`ModelTypeMut`]
    /// (e.g., `Box<MjModel>`, `&mut MjModel`).
    /// Shared-ownership types such as `Arc<MjModel>` do not provide mutable
    /// access; use [`swap_model`](MjData::swap_model) instead.
    ///
    /// # Safety
    /// This method is marked unsafe as the owned model can be swapped entirely without any compatibility
    /// checks.
    ///
    /// It is the caller's responsibility to ensure that a swapped model is compatible with the
    /// model this data belongs to, as [`MjModel::is_compatible_with_model`] defines.
    ///
    /// For safe swapping consider [`MjData::swap_model`] or [`MjData::try_swap_model`] for a fallible alternative.
    ///
    /// # Example
    /// ```rust
    /// # use mujoco_rs::prelude::{MjModel, MjData};
    /// let model = Box::new(MjModel::from_xml_string("<mujoco/>").unwrap());
    /// let mut data = MjData::new(model);
    /// unsafe { data.model_mut() }.opt_mut().timestep = 0.001;
    /// unsafe { data.model_mut() }.opt_mut().gravity[2] = -5.0;
    /// ```
    pub unsafe fn model_mut(&mut self) -> &mut MjModel {
        &mut self.model
    }
    /// Returns a mutable reference to [`MjModel::opt_mut`] without allowing unsafe
    /// modifications to the rest of the [`MjModel`].
    ///
    /// Immutable references can be made through [`MjData::model_opt`].
    ///
    /// Can be used to modify the physics parameters.
    /// # Example
    /// ```rust
    /// # use mujoco_rs::prelude::{MjModel, MjData};
    /// let model = Box::new(MjModel::from_xml_string("<mujoco/>").unwrap());
    /// let mut data = MjData::new(model);
    /// data.model_opt_mut().timestep = 0.001;
    /// data.model_opt_mut().gravity[2] = -5.0;
    /// ```
    pub fn model_opt_mut(&mut self) -> &mut MjOption {
        self.model.opt_mut()
    }
    /// Returns a mutable reference to [`MjModel::vis_mut`] without allowing unsafe
    /// modifications to the rest of the [`MjModel`].
    ///
    /// Immutable references can be made through [`MjData::model_vis`].
    ///
    /// Can be used to modify the visualization parameters.
    /// # Example
    /// ```rust
    /// # use mujoco_rs::prelude::{MjModel, MjData};
    /// let model = Box::new(MjModel::from_xml_string("<mujoco/>").unwrap());
    /// let mut data = MjData::new(model);
    /// data.model_vis_mut().headlight.ambient = [0.0, 0.0, 0.0];
    /// data.model_vis_mut().headlight.active = 1;
    /// ```
    pub fn model_vis_mut(&mut self) -> &mut MjVisual {
        self.model.vis_mut()
    }
    /// Returns a mutable reference to [`MjModel::stat_mut`] without allowing unsafe
    /// modifications to the rest of the [`MjModel`].
    ///
    /// Immutable references can be made through [`MjData::model_stat`].
    ///
    /// Can be used to modify the model statistics.
    /// # Example
    /// ```rust
    /// # use mujoco_rs::prelude::{MjModel, MjData};
    /// let model = Box::new(MjModel::from_xml_string("<mujoco/>").unwrap());
    /// let mut data = MjData::new(model);
    /// data.model_stat_mut().center = [0.0, 0.0, 0.5];
    /// ```
    pub fn model_stat_mut(&mut self) -> &mut MjStatistic {
        self.model.stat_mut()
    }
}
/// Arrays of dynamic size.
impl<M: ModelType> MjData<M> {
    array_slice_dyn! {
        probe = probe_dynamic_arrays; qpos : & [MjtNum; "position"; model.ffi().nq], qvel
        : & [MjtNum; "velocity"; model.ffi().nv], act : & [MjtNum; "actuator activation";
        model.ffi().na], (mut = unsafe) history : & [MjtNum; "history buffer"; model
        .ffi().nhistory], qacc_warmstart : & [MjtNum; "acceleration used for warmstart";
        model.ffi().nv], plugin_state : & [MjtNum; "plugin state"; model.ffi()
        .npluginstate], ctrl : & [MjtNum; "control"; model.ffi().nu], qfrc_applied : &
        [MjtNum; "applied generalized force"; model.ffi().nv], xfrc_applied : & [[MjtNum;
        6] [force]; "applied Cartesian force/torque"; model.ffi().nbody], eq_active : &
        [MjtBool; "enable/disable constraints"; model.ffi().neq], mocap_pos : & [[MjtNum;
        3] [force]; "positions of mocap bodies"; model.ffi().nmocap], mocap_quat : &
        [[MjtNum; 4] [force]; "orientations of mocap bodies"; model.ffi().nmocap], qacc :
        & [MjtNum; "acceleration"; model.ffi().nv], act_dot : & [MjtNum;
        "time-derivative of actuator activation"; model.ffi().na], userdata : & [MjtNum;
        "user data, not touched by engine"; model.ffi().nuserdata], sensordata : &
        [MjtNum; "sensor data array"; model.ffi().nsensordata], (mut = unsafe)
        tree_asleep : & [i32; "<0: awake; >=0: index cycle of sleeping trees"; model
        .ffi().ntree], xpos : & [[MjtNum; 3] [force]; "Cartesian position of body frame";
        model.ffi().nbody], xquat : & [[MjtNum; 4] [force];
        "Cartesian orientation of body frame"; model.ffi().nbody], xmat : & [[MjtNum; 9]
        [force]; "Cartesian orientation of body frame"; model.ffi().nbody], xipos : &
        [[MjtNum; 3] [force]; "Cartesian position of body com"; model.ffi().nbody], ximat
        : & [[MjtNum; 9] [force]; "Cartesian orientation of body inertia"; model.ffi()
        .nbody], xanchor : & [[MjtNum; 3] [force]; "Cartesian position of joint anchor";
        model.ffi().njnt], xaxis : & [[MjtNum; 3] [force]; "Cartesian joint axis"; model
        .ffi().njnt], geom_xpos : & [[MjtNum; 3] [force]; "Cartesian geom position";
        model.ffi().ngeom], geom_xmat : & [[MjtNum; 9] [force];
        "Cartesian geom orientation"; model.ffi().ngeom], site_xpos : & [[MjtNum; 3]
        [force]; "Cartesian site position"; model.ffi().nsite], site_xmat : & [[MjtNum;
        9] [force]; "Cartesian site orientation"; model.ffi().nsite], cam_xpos : &
        [[MjtNum; 3] [force]; "Cartesian camera position"; model.ffi().ncam], cam_xmat :
        & [[MjtNum; 9] [force]; "Cartesian camera orientation"; model.ffi().ncam],
        light_xpos : & [[MjtNum; 3] [force]; "Cartesian light position"; model.ffi()
        .nlight], light_xdir : & [[MjtNum; 3] [force]; "Cartesian light direction"; model
        .ffi().nlight], subtree_com : & [[MjtNum; 3] [force];
        "center of mass of each subtree"; model.ffi().nbody], cdof : & [[MjtNum; 6]
        [force]; "com-based motion axis of each dof (rot:lin)"; model.ffi().nv], cinert :
        & [[MjtNum; 10] [force]; "com-based body inertia and mass"; model.ffi().nbody],
        flexvert_xpos : & [[MjtNum; 3] [force]; "Cartesian flex vertex positions"; model
        .ffi().nflexvert], flexelem_aabb : & [[MjtNum; 6] [force];
        "flex element bounding boxes (center, size)"; model.ffi().nflexelem],
        flexelem_krot : & [MjtNum; "corotated element stiffness (implicit only)"; model
        .ffi().nflexstiffness], flexedge_J : & [MjtNum; "flex edge Jacobian"; model.ffi()
        .nJfe], flexedge_length : & [MjtNum; "flex edge lengths"; model.ffi().nflexedge],
        flexvert_J : & [[MjtNum; 2] [force]; "flex vertex Jacobian"; model.ffi().nJfv],
        flexvert_length : & [[MjtNum; 2] [force]; "flex vertex lengths"; model.ffi()
        .nflexvert], bvh_aabb_dyn : & [[MjtNum; 6] [force];
        "global bounding box (center, size)"; model.ffi().nbvhdynamic], (mut = unsafe)
        ten_wrapadr : & [i32; "start address of tendon's path"; model.ffi().ntendon],
        (mut = unsafe) ten_wrapnum : & [i32; "number of wrap points in path"; model.ffi()
        .ntendon], ten_J : & [MjtNum; "tendon Jacobian"; model.ffi().nJten], ten_length :
        & [MjtNum; "tendon lengths"; model.ffi().ntendon], (mut = unsafe) wrap_obj : &
        [[i32; 2] [force]; "geom id; -1: site; -2: pulley"; model.ffi().nwrap], wrap_xpos
        : & [[MjtNum; 6] [force]; "Cartesian 3D points in all paths"; model.ffi().nwrap],
        actuator_length : & [MjtNum; "actuator lengths, one per force output"; model
        .ffi().nout], (mut = unsafe) moment_rownnz : & [i32;
        "number of non-zeros in actuator_moment row"; model.ffi().nout], (mut = unsafe)
        moment_rowadr : & [i32; "row start address in colind array"; model.ffi().nout],
        (mut = unsafe) moment_colind : & [i32; "column indices in sparse Jacobian"; model
        .ffi().nJmom], actuator_moment : & [MjtNum; "actuator moments"; model.ffi()
        .nJmom], crb : & [[MjtNum; 10] [force]; "com-based composite inertia and mass";
        model.ffi().nbody], M : & [MjtNum; "inertia (compressed sparse row)"; model.ffi()
        .nC], qLD : & [MjtNum; "L'*D*L factorization of M (sparse)"; model.ffi().nC],
        qLDiagInv : & [MjtNum; "1/diag(D)"; model.ffi().nv], bvh_active : & [MjtBool;
        "was bounding volume checked for collision"; model.ffi().nbvh], tree_awake : &
        [i32; "is tree awake; 0: asleep; 1: awake"; model.ffi().ntree], body_awake : &
        [MjtSleepState[force]; "body sleep state"; model.ffi().nbody], (mut = unsafe)
        body_awake_ind : & [i32; "indices of awake and static bodies"; model.ffi()
        .nbody], (mut = unsafe) parent_awake_ind : & [i32;
        "indices of bodies with awake or static parents"; model.ffi().nbody], (mut =
        unsafe) dof_awake_ind : & [i32; "indices of awake dofs"; model.ffi().nv],
        flexedge_velocity : & [MjtNum; "flex edge velocities"; model.ffi().nflexedge],
        ten_velocity : & [MjtNum; "tendon velocities"; model.ffi().ntendon],
        actuator_velocity : & [MjtNum; "actuator velocities, one per force output"; model
        .ffi().nout], cvel : & [[MjtNum; 6] [force]; "com-based velocity (rot:lin)";
        model.ffi().nbody], cdof_dot : & [[MjtNum; 6] [force];
        "time-derivative of cdof (rot:lin)"; model.ffi().nv], qfrc_bias : & [MjtNum;
        "C(qpos,qvel)"; model.ffi().nv], qfrc_spring : & [MjtNum; "passive spring force";
        model.ffi().nv], qfrc_damper : & [MjtNum; "passive damper force"; model.ffi()
        .nv], qfrc_gravcomp : & [MjtNum; "passive gravity compensation force"; model
        .ffi().nv], qfrc_fluid : & [MjtNum; "passive fluid force"; model.ffi().nv],
        qfrc_adhesion : & [MjtNum; "passive contact adhesion force"; model.ffi().nv],
        qfrc_passive : & [MjtNum; "total passive force"; model.ffi().nv], subtree_linvel
        : & [[MjtNum; 3] [force]; "linear velocity of subtree com"; model.ffi().nbody],
        subtree_angmom : & [[MjtNum; 3] [force]; "angular momentum about subtree com";
        model.ffi().nbody], qH : & [MjtNum; "L'*D*L factorization of modified M"; model
        .ffi().nC], qHDiagInv : & [MjtNum; "1/diag(D) of modified M"; model.ffi().nv],
        qDeriv : & [MjtNum; "d (passive + actuator - bias) / d qvel"; model.ffi().nD],
        qLU : & [MjtNum; "sparse LU of (M - dt*qDeriv)"; model.ffi().nD], actuator_force
        : & [MjtNum; "actuator force in actuation space"; model.ffi().nout],
        qfrc_actuator : & [MjtNum; "actuator force in joint space"; model.ffi().nv],
        qfrc_smooth : & [MjtNum; "net unconstrained force"; model.ffi().nv], qacc_smooth
        : & [MjtNum; "unconstrained acceleration"; model.ffi().nv], qfrc_constraint : &
        [MjtNum; "constraint force"; model.ffi().nv], qfrc_inverse : & [MjtNum;
        "net external force; should equal qfrc_applied + J'*xfrc_applied + qfrc_actuator";
        model.ffi().nv], cacc : & [[MjtNum; 6] [force]; "com-based acceleration"; model
        .ffi().nbody], cfrc_int : & [[MjtNum; 6] [force];
        "com-based interaction force with parent"; model.ffi().nbody], cfrc_ext : &
        [[MjtNum; 6] [force]; "com-based external force on body"; model.ffi().nbody],
        (mut = unsafe) contact : & [MjContact; "array of all detected contacts"; ffi()
        .ncon], (mut = unsafe) efc_type : & [MjtConstraint[force]; "constraint type";
        ffi().nefc], (mut = unsafe) efc_id : & [i32; "id of object of specified type";
        ffi().nefc], (read = unsafe) efc_J_rownnz : & [i32;
        "number of non-zeros in constraint Jacobian row"; ffi().nefc], (read = unsafe)
        efc_J_rowadr : & [i32; "row start address in colind array"; ffi().nefc], (read =
        unsafe) efc_J_rowsuper : & [i32; "number of subsequent rows in supernode"; ffi()
        .nefc], (read = unsafe) efc_J_colind : & [i32;
        "column indices in constraint Jacobian"; ffi().nJ], efc_J : & [MjtNum;
        "constraint Jacobian"; ffi().nJ], efc_pos : & [MjtNum;
        "constraint position (equality, contact)"; ffi().nefc], efc_margin : & [MjtNum;
        "inclusion margin (contact)"; ffi().nefc], efc_frictionloss : & [MjtNum;
        "frictionloss (friction)"; ffi().nefc], efc_diagA : & [MjtNum;
        "diagonal of A matrix, approximate or exact"; ffi().nefc], efc_KBIP : & [[MjtNum;
        4] [force]; "stiffness, damping, impedance, imp'"; ffi().nefc], efc_D : &
        [MjtNum; "constraint mass"; ffi().nefc], efc_R : & [MjtNum;
        "inverse constraint mass"; ffi().nefc], (mut = unsafe) tendon_efcadr : & [i32;
        "first efc address involving tendon; -1: none"; model.ffi().ntendon], (mut =
        unsafe) tree_island : & [i32; "island id of this tree; -1: none"; model.ffi()
        .ntree], (mut = unsafe) island_ntree : & [i32; "number of trees in this island";
        ffi().nisland], (mut = unsafe) island_itreeadr : & [i32;
        "island start address in itree vector"; ffi().nisland], (mut = unsafe)
        map_itree2tree : & [i32; "map from itree to tree"; model.ffi().ntree], (mut =
        unsafe) dof_island : & [i32; "island id of this dof; -1: none"; model.ffi().nv],
        (mut = unsafe) island_nv : & [i32; "number of dofs in this island"; ffi()
        .nisland], (mut = unsafe) island_idofadr : & [i32;
        "island start address in idof vector"; ffi().nisland], (mut = unsafe)
        island_dofadr : & [i32; "island start address in dof vector"; ffi().nisland],
        (mut = unsafe) map_dof2idof : & [i32; "map from dof to idof"; model.ffi().nv],
        (mut = unsafe) map_idof2dof : & [i32;
        "map from idof to dof;  >= nidof: unconstrained"; model.ffi().nv], (read =
        unsafe) ifrc_smooth : & [MjtNum; "net unconstrained force"; ffi().nidof], (read =
        unsafe) iacc_smooth : & [MjtNum; "unconstrained acceleration"; ffi().nidof],
        (read = unsafe) iacc : & [MjtNum; "acceleration"; ffi().nidof], (mut = unsafe)
        efc_island : & [i32; "island id of this constraint"; ffi().nefc], (mut = unsafe)
        island_ne : & [i32; "number of equality constraints in island"; ffi().nisland],
        (mut = unsafe) island_nf : & [i32; "number of friction constraints in island";
        ffi().nisland], (mut = unsafe) island_nefc : & [i32;
        "number of constraints in island"; ffi().nisland], (mut = unsafe) island_iefcadr
        : & [i32; "start address in iefc vector"; ffi().nisland], (mut = unsafe)
        map_efc2iefc : & [i32; "map from efc to iefc"; ffi().nefc], (mut = unsafe)
        map_iefc2efc : & [i32; "map from iefc to efc"; ffi().nefc], (mut = unsafe)
        iefc_type : & [MjtConstraint[force]; "constraint type"; ffi().nefc], (mut =
        unsafe) iefc_id : & [i32; "id of object of specified type"; ffi().nefc],
        iefc_frictionloss : & [MjtNum; "frictionloss (friction)"; ffi().nefc], iefc_D : &
        [MjtNum; "constraint mass"; ffi().nefc], iefc_R : & [MjtNum;
        "inverse constraint mass"; ffi().nefc], (mut = unsafe) efc_Y_rownnz : & [i32;
        "number of non-zeros in Y row"; ffi().nefc], (mut = unsafe) efc_Y_rowadr : &
        [i32; "row start address in Y colind array"; ffi().nefc], (mut = unsafe)
        efc_Y_colind : & [i32; "column indices in sparse Y"; ffi().nY], efc_Y : &
        [MjtNum; "whitened Jacobian Y = J*M^(-1/2)"; ffi().nY], (mut = unsafe)
        efc_AR_rownnz : & [i32; "number of non-zeros in AR"; ffi().nefc], (mut = unsafe)
        efc_AR_rowadr : & [i32; "row start address in colind array"; ffi().nefc], (mut =
        unsafe) efc_AR_colind : & [i32; "column indices in sparse AR"; ffi().nA], efc_AR
        : & [MjtNum; "J*inv(M)*J' + R"; ffi().nA], (read = unsafe) efc_vel : & [MjtNum;
        "velocity in constraint space: J*qvel"; ffi().nefc], (read = unsafe) efc_aref : &
        [MjtNum; "reference pseudo-acceleration"; ffi().nefc], efm_c : & [MjtNum;
        "smooth-force shift h*K*qvel"; model.ffi().nv], (mut = unsafe) efm_K_rownnz : &
        [i32; "effective-stiffness CSR row nonzeros"; model.ffi().nv], (mut = unsafe)
        efm_K_rowadr : & [i32; "effective-stiffness CSR row addresses"; model.ffi().nv],
        (mut = unsafe) efm_K_colind : & [i32; "effective-stiffness CSR column indices";
        ffi().nefmK], efm_K_val : & [MjtNum; "effective-stiffness CSR values"; ffi()
        .nefmK], (mut = unsafe) efm_dofid : & [i32;
        "block k -> dof address of its vertex triple"; ffi().nefmdof], efm_L : & [MjtNum;
        "factored 3x3 diagonal blocks of M+K"; ffi().nefmL], (read = unsafe) efc_b : &
        [MjtNum; "linear cost term: J*qacc_smooth - aref"; ffi().nefc], (read = unsafe)
        iefc_aref : & [MjtNum; "reference pseudo-acceleration"; ffi().nefc], (read =
        unsafe) iefc_state : & [MjtConstraintState[force]; "constraint state"; ffi()
        .nefc], (read = unsafe) iefc_force : & [MjtNum;
        "constraint force in constraint space"; ffi().nefc], (read = unsafe) efc_state :
        & [MjtConstraintState[force]; "constraint state"; ffi().nefc], (read = unsafe)
        efc_force : & [MjtNum; "constraint force in constraint space"; ffi().nefc], (read
        = unsafe) ifrc_constraint : & [MjtNum; "constraint force"; ffi().nidof]
    }
}
impl<M: ModelType> Drop for MjData<M> {
    fn drop(&mut self) {
        unsafe {
            mj_deleteData(self.data.as_ptr());
        }
    }
}
impl<M: ModelType + Clone> Clone for MjData<M> {
    /// # Note
    /// MuJoCo aborts the process through `mjERROR` when an allocation fails, so this never fails.
    #[expect(
        deprecated,
        reason = "try_clone keeps the implementation until it is removed"
    )]
    fn clone(&self) -> Self {
        self.try_clone().expect("not enough space to clone data")
    }
}
impl<M: ModelType + Clone> MjData<M> {
    /// Fallible version of [`Clone::clone`].
    ///
    /// # Note
    /// MuJoCo ends the process when the allocation fails, so this never returns `Err`.
    ///
    /// # Errors
    /// Returns [`MjDataError::AllocationFailed`] if MuJoCo fails to allocate
    /// the copy.
    #[deprecated(since = "6.0.0", note = "always returns Ok; use `clone`")]
    pub fn try_clone(&self) -> Result<Self, MjDataError> {
        let raw = unsafe { mj_copyData(ptr::null_mut(), self.model.ffi(), self.ffi()) };
        NonNull::new(raw)
            .map(|data| Self {
                data,
                model: self.model.clone(),
            })
            .ok_or(MjDataError::AllocationFailed)
    }
}
info_with_view!(
    Data, actuator, [ctrl : MjtNum, [actuator_] length : MjtNum, [actuator_] velocity :
    MjtNum, [actuator_] force : MjtNum], [], [act : MjtNum], M : ModelType
);
info_with_view!(
    Data, body, [xfrc_applied : MjtNum, xpos : MjtNum, xquat : MjtNum, xmat : MjtNum,
    xipos : MjtNum, ximat : MjtNum, subtree_com : MjtNum, cinert : MjtNum, crb : MjtNum,
    cvel : MjtNum, subtree_linvel : MjtNum, subtree_angmom : MjtNum, cacc : MjtNum,
    cfrc_int : MjtNum, cfrc_ext : MjtNum, [body_] awake : MjtSleepState[force]], [], [],
    M : ModelType
);
info_with_view!(
    Data, camera, [[cam_] xpos : MjtNum, [cam_] xmat : MjtNum], [], [], M : ModelType
);
info_with_view!(
    Data, geom, [[geom_] xpos : MjtNum, [geom_] xmat : MjtNum], [], [], M : ModelType
);
info_with_view!(
    Data, joint, [qpos : MjtNum, qvel : MjtNum, qacc_warmstart : MjtNum, qfrc_applied :
    MjtNum, qacc : MjtNum, xanchor : MjtNum, xaxis : MjtNum, qLDiagInv : MjtNum,
    qfrc_bias : MjtNum, qfrc_spring : MjtNum, qfrc_damper : MjtNum, qfrc_gravcomp :
    MjtNum, qfrc_fluid : MjtNum, qfrc_adhesion : MjtNum, qfrc_passive : MjtNum,
    qfrc_actuator : MjtNum, qfrc_smooth : MjtNum, qacc_smooth : MjtNum, qfrc_constraint :
    MjtNum, qfrc_inverse : MjtNum], [], [], M : ModelType
);
info_with_view!(
    Data, light, [[light_] xpos : MjtNum, [light_] xdir : MjtNum], [], [], M : ModelType
);
info_with_view!(Data, sensor, [[sensor] data : MjtNum], [], [], M : ModelType);
info_with_view!(
    Data, site, [[site_] xpos : MjtNum, [site_] xmat : MjtNum], [], [], M : ModelType
);
info_with_view!(
    Data, tendon, [[ten_] J : MjtNum, [ten_] length : MjtNum, [ten_] velocity : MjtNum],
    [[ten_] wrapadr : i32, [ten_] wrapnum : i32, [tendon_] efcadr : i32], [], M :
    ModelType
);
