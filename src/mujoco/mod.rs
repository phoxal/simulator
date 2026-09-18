//! Native MuJoCo integration owned by the independent simulator application.
//!
//! The default build contains only closed-model artifact validation and digesting.
//! Enable the `native` feature to compile MJCF with the pinned `mujoco-rs`
//! binding and use the native `Model`, `Workspace`, and `Scene` types.
//!
//! `Scene` is the sole owner of authoritative mutable physics data exposed by
//! this crate.
//! `Workspace` is an explicitly non-authoritative computation owner suitable
//! for a service that has chosen native model access.
//! `Model` is immutable after compilation and can be shared with read-only
//! consumers.

mod artifact;
mod error;

#[cfg(feature = "native")]
mod composition;
#[cfg(feature = "native")]
mod model;
#[cfg(feature = "rendering")]
mod provider;
#[cfg(feature = "native")]
mod scene;

pub use artifact::{ClosedModel, Resource};

#[cfg(feature = "native")]
pub use composition::{ComponentAttachment, SceneComposition, unique_direct_root_body};
#[cfg(any(test, feature = "rendering"))]
pub use composition::{CompositionError, ModelComposition, NAMESPACE_SEPARATOR, compose_model, compose_scene};
#[cfg(feature = "native")]
pub use error::ModelError;
#[cfg(feature = "rendering")]
pub use error::{SceneError, WorkspaceError};
#[cfg(feature = "native")]
pub use model::{Model, ModelIdentity};
#[cfg(feature = "native")]
pub(crate) use model::{
    ActuatorBinding, ActuatorHandle, ActuatorInfo, ActuatorMode, CameraBinding, JointKind,
    SensorBinding, SensorKind, SiteBinding,
};
#[cfg(any(test, feature = "rendering"))]
pub use model::{
    BodyHandle, BodyInfo, CameraHandle, CameraInfo, JointHandle, JointInfo, ModelCounts,
    ModelHandle, ObjectKind, SensorHandle, SensorInfo, SiteHandle, SiteInfo,
};
#[cfg(feature = "rendering")]
pub use provider::{
    ActuatorSelection, Boundary, ControlledError, ControlledPhase, ControlledScene, ControlledStep,
    ExecutionId, HoldProvider, ObservationReceipt, PrepareRequest, ProviderReset,
    SimulationProvider, SourceId, TimelineId,
};
#[cfg(feature = "rendering")]
pub use scene::RenderedCamera;
#[cfg(feature = "rendering")]
pub use scene::ViewCamera;
#[cfg(feature = "native")]
pub use scene::{PhysicsQuantum, StateSnapshot, Workspace};
#[cfg(feature = "rendering")]
pub use scene::{Scene, ScenePhase, SceneStep};

#[cfg(test)]
#[cfg(feature = "native")]
mod tests;
