//! Public-session MuJoCo simulation authority and native scene ownership.
//!
//! The simulator is an application, not a robot runtime process.  It owns the immutable
//! MuJoCo model and the one mutable `Scene`, while a public
//! [`phoxal::session::Simulation`] owns the authenticated supervisor boundary.
//! The only operation that can move either boundary is `RemoteSceneRun::step`:
//! it submits one complete typed observation cut, admits one actuation cut, and
//! then performs exactly one native quantum.
//!
//! This module deliberately does not import an execution runner, supervisor
//! host, or robot implementation crate.  The small `SimulationTransport` trait keeps the
//! lifecycle state machine testable with a transport fake while the blanket
//! implementation below uses the public session client in production.

use std::collections::BTreeSet;
use std::fmt;

use crate::authority::{
    AuthorityClient, AuthorityClientError, AuthorityState, SimulationTransport,
};
use phoxal::communication::session::MethodShape;
use phoxal::communication::simulation::{
    Actuation, Observation, ProductDisposition, ProgressResponse, ReleaseAuthorityResponse,
};
use sha2::{Digest, Sha256};

#[cfg(feature = "native")]
use crate::mujoco::{Model, PhysicsQuantum, Scene, SceneError, StateSnapshot};

const MAX_PROVIDER_REQUIREMENTS: usize = 256;
const MAX_PROVIDER_ID_BYTES: usize = 64;
const MAX_FQN_BYTES: usize = 512;
const MAX_PAYLOAD_BYTES: usize = 4 * 1024 * 1024;

/// Public simulation protocol selected by this application coordinator.
pub const SIMULATION_PROTOCOL: &str = "phoxal.simulation.v1";

/// One canonical, immutable provider set selected by the robot bundle.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProviderSet {
    requirements: Vec<phoxal::communication::simulation::ProviderRequirement>,
}

/// One exact typed actuation output admitted for a native scene.
///
/// Actuation ports are separate from observation providers.  They are owned by
/// the simulator's immutable scene configuration and are matched by the
/// service instance, port, and payload FQN before any native control is set.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActuationBinding {
    service_instance: String,
    port: String,
    payload_fqn: String,
    actuator_ids: Vec<String>,
}

impl ActuationBinding {
    /// Construct one exact output binding.
    pub fn new(
        service_instance: impl Into<String>,
        port: impl Into<String>,
        payload_fqn: impl Into<String>,
        mut actuator_ids: Vec<String>,
    ) -> Result<Self, ProviderSetError> {
        actuator_ids.sort();
        let binding = Self {
            service_instance: service_instance.into(),
            port: port.into(),
            payload_fqn: payload_fqn.into(),
            actuator_ids,
        };
        validate_identifier(&binding.service_instance, "actuation service instance")?;
        validate_identifier(&binding.port, "actuation port")?;
        validate_fqn(&binding.payload_fqn, "actuation payload FQN", false)?;
        if binding.actuator_ids.is_empty() {
            return Err(ProviderSetError::ActuatorMembership {
                service_instance: binding.service_instance.clone(),
                port: binding.port.clone(),
            });
        }
        let mut actuator_ids = BTreeSet::new();
        for actuator_id in binding.actuator_ids() {
            validate_native_name(actuator_id, "actuator id")?;
            if !actuator_ids.insert(actuator_id.as_str()) {
                return Err(ProviderSetError::ActuatorDuplicate {
                    service_instance: binding.service_instance.clone(),
                    port: binding.port.clone(),
                    actuator_id: actuator_id.clone(),
                });
            }
        }
        Ok(binding)
    }

    /// Service instance that owns this output port.
    #[must_use]
    pub fn service_instance(&self) -> &str {
        &self.service_instance
    }

    /// Generated output port name.
    #[must_use]
    pub fn port(&self) -> &str {
        &self.port
    }

    /// Fully-qualified typed payload name.
    #[must_use]
    pub fn payload_fqn(&self) -> &str {
        &self.payload_fqn
    }

    /// Native actuator identities covered by this output payload.
    #[must_use]
    pub fn actuator_ids(&self) -> &[String] {
        &self.actuator_ids
    }
}

impl ProviderSet {
    /// Validate and canonicalize the provider requirements from the bundle.
    pub fn new(
        requirements: Vec<phoxal::communication::simulation::ProviderRequirement>,
    ) -> Result<Self, ProviderSetError> {
        if requirements.is_empty() || requirements.len() > MAX_PROVIDER_REQUIREMENTS {
            return Err(ProviderSetError::Count {
                count: requirements.len(),
            });
        }

        let mut requirements = requirements;
        for requirement in &requirements {
            if requirement.rate_microhertz == 0 {
                return Err(ProviderSetError::InvalidIdentifier {
                    field: "provider rate",
                });
            }
            validate_identifier(&requirement.service_instance, "service instance")?;
            validate_identifier(&requirement.port, "provider port")?;
            validate_fqn(&requirement.payload_fqn, "payload FQN", false)?;
            validate_fqn(&requirement.input_fqn, "input FQN", true)?;
            let shape = MethodShape::try_from(requirement.shape).map_err(|_| {
                ProviderSetError::InvalidKind {
                    service_instance: requirement.service_instance.clone(),
                    port: requirement.port.clone(),
                }
            })?;
            if shape != MethodShape::Observation {
                return Err(ProviderSetError::InvalidKind {
                    service_instance: requirement.service_instance.clone(),
                    port: requirement.port.clone(),
                });
            }
        }
        requirements.sort_by(|left, right| {
            left.service_instance
                .cmp(&right.service_instance)
                .then_with(|| left.port.cmp(&right.port))
                .then_with(|| left.shape.cmp(&right.shape))
                .then_with(|| left.input_fqn.cmp(&right.input_fqn))
                .then_with(|| left.payload_fqn.cmp(&right.payload_fqn))
        });
        for pair in requirements.windows(2) {
            if pair[0].service_instance == pair[1].service_instance && pair[0].port == pair[1].port
            {
                return Err(ProviderSetError::Duplicate {
                    service_instance: pair[0].service_instance.clone(),
                    port: pair[0].port.clone(),
                });
            }
        }
        Ok(Self { requirements })
    }

    /// The canonical wire requirements.
    #[must_use]
    pub fn requirements(&self) -> &[phoxal::communication::simulation::ProviderRequirement] {
        &self.requirements
    }

    /// Return one provider's exact metadata.
    #[must_use]
    pub fn get(
        &self,
        service_instance: &str,
        port: &str,
    ) -> Option<&phoxal::communication::simulation::ProviderRequirement> {
        self.requirements.iter().find(|requirement| {
            requirement.service_instance == service_instance && requirement.port == port
        })
    }

    /// Require the complete provider set for one advance request.
    pub fn validate_observations(
        &self,
        observations: &[Observation],
        boundary: u64,
        quantum_ns: u64,
    ) -> Result<(), ProviderSetError> {
        if observations.len() != self.requirements.len() {
            return Err(ProviderSetError::Incomplete {
                expected: self.requirements.len(),
                actual: observations.len(),
            });
        }
        let expected_capture_ns = boundary
            .checked_mul(quantum_ns)
            .ok_or(ProviderSetError::CaptureTimeOverflow)?;
        let mut seen = BTreeSet::new();
        for observation in observations {
            let member =
                observation
                    .membership
                    .as_ref()
                    .ok_or(ProviderSetError::InvalidIdentifier {
                        field: "observation membership",
                    })?;
            if member.capture_boundary != boundary
                || member.sequence == 0
                || !match member.disposition {
                    ProductDisposition::Present => member.item_count == 1,
                    ProductDisposition::NotDue | ProductDisposition::Empty => {
                        member.item_count == 0 && observation.payload.is_empty()
                    }
                    _ => false,
                }
                || member.encoded_bytes != observation.payload.len() as u64
                || member.payload_digest != Sha256::digest(&observation.payload).as_slice()
            {
                return Err(ProviderSetError::InvalidIdentifier {
                    field: "observation membership integrity",
                });
            }
            if observation.payload.len() > MAX_PAYLOAD_BYTES {
                return Err(ProviderSetError::PayloadTooLarge {
                    service_instance: member.producer.clone(),
                    port: member.port.clone(),
                    bytes: observation.payload.len(),
                });
            }
            if member.capture_time_ns != expected_capture_ns {
                return Err(ProviderSetError::CaptureTime {
                    expected: expected_capture_ns,
                    actual: member.capture_time_ns,
                });
            }
            let key = (member.producer.as_str(), member.port.as_str());
            if !seen.insert(key) {
                return Err(ProviderSetError::Duplicate {
                    service_instance: member.producer.clone(),
                    port: member.port.clone(),
                });
            }
            let Some(requirement) = self.get(key.0, key.1) else {
                return Err(ProviderSetError::Unknown {
                    service_instance: member.producer.clone(),
                    port: member.port.clone(),
                });
            };
            let cadence =
                crate::cadence::Cadence::from_microhertz(requirement.rate_microhertz, quantum_ns)
                    .map_err(|_| ProviderSetError::InvalidIdentifier {
                    field: "provider rate or quantum",
                })?;
            if cadence.due(boundary) == (member.disposition == ProductDisposition::NotDue) {
                return Err(ProviderSetError::InvalidIdentifier {
                    field: "observation capture cadence",
                });
            }
            if requirement.payload_fqn.is_empty() {
                return Err(ProviderSetError::InvalidFqn {
                    field: "payload FQN",
                });
            }
        }
        Ok(())
    }
}

/// A provider-set admission failure.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ProviderSetError {
    /// The set is empty or exceeds the finite bound.
    Count { count: usize },
    /// A provider identity is empty, oversized, or not a route identifier.
    InvalidIdentifier { field: &'static str },
    /// A provider FQN is not a valid Protobuf name.
    InvalidFqn { field: &'static str },
    /// A provider uses an unsupported or unspecified semantic kind.
    InvalidKind {
        service_instance: String,
        port: String,
    },
    /// A service/port pair appears more than once.
    Duplicate {
        service_instance: String,
        port: String,
    },
    /// An advance omitted one or more immutable providers.
    Incomplete { expected: usize, actual: usize },
    /// The boundary-to-capture-time conversion overflowed.
    CaptureTimeOverflow,
    /// An observation timestamp is not the exact current-boundary timestamp.
    CaptureTime { expected: u64, actual: u64 },
    /// An observation is not in the immutable provider set.
    Unknown {
        service_instance: String,
        port: String,
    },
    /// A typed observation exceeded the bounded exchange payload.
    PayloadTooLarge {
        service_instance: String,
        port: String,
        bytes: usize,
    },
    /// An actuation binding omitted every native actuator.
    ActuatorMembership {
        service_instance: String,
        port: String,
    },
    /// An actuator identity is repeated within an actuation binding.
    ActuatorDuplicate {
        service_instance: String,
        port: String,
        actuator_id: String,
    },
}

impl fmt::Display for ProviderSetError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Count { count } => write!(formatter, "provider set has invalid size {count}"),
            Self::InvalidIdentifier { field } => write!(formatter, "{field} is invalid"),
            Self::InvalidFqn { field } => write!(formatter, "{field} is invalid"),
            Self::InvalidKind {
                service_instance,
                port,
            } => write!(
                formatter,
                "provider kind is invalid for {service_instance}/{port}"
            ),
            Self::Duplicate {
                service_instance,
                port,
            } => write!(
                formatter,
                "provider {service_instance}/{port} is duplicated"
            ),
            Self::Incomplete { expected, actual } => write!(
                formatter,
                "advance has {actual} observations, expected the complete set of {expected}"
            ),
            Self::CaptureTimeOverflow => write!(formatter, "capture time overflows nanoseconds"),
            Self::CaptureTime { expected, actual } => write!(
                formatter,
                "observation capture time {actual} does not equal current boundary time {expected}"
            ),
            Self::Unknown {
                service_instance,
                port,
            } => write!(
                formatter,
                "observation {service_instance}/{port} is not a provider"
            ),
            Self::PayloadTooLarge {
                service_instance,
                port,
                bytes,
            } => write!(
                formatter,
                "observation {service_instance}/{port} payload is {bytes} bytes"
            ),
            Self::ActuatorMembership {
                service_instance,
                port,
            } => write!(
                formatter,
                "actuation {service_instance}/{port} has no native actuators"
            ),
            Self::ActuatorDuplicate {
                service_instance,
                port,
                actuator_id,
            } => write!(
                formatter,
                "actuation {service_instance}/{port} repeats actuator {actuator_id}"
            ),
        }
    }
}

impl std::error::Error for ProviderSetError {}

fn validate_identifier(value: &str, field: &'static str) -> Result<(), ProviderSetError> {
    if value.is_empty()
        || value.len() > MAX_PROVIDER_ID_BYTES
        || !value.is_ascii()
        || value.bytes().any(|byte| byte.is_ascii_whitespace())
        || value
            .bytes()
            .next()
            .is_none_or(|byte| !(byte.is_ascii_lowercase() || byte.is_ascii_digit()))
        || !value.bytes().skip(1).all(|byte| {
            byte.is_ascii_lowercase() || byte.is_ascii_digit() || byte == b'_' || byte == b'-'
        })
    {
        return Err(ProviderSetError::InvalidIdentifier { field });
    }
    Ok(())
}

fn validate_native_name(value: &str, field: &'static str) -> Result<(), ProviderSetError> {
    if value.is_empty()
        || value.len() > MAX_PROVIDER_ID_BYTES
        || !value.is_ascii()
        || value
            .bytes()
            .any(|byte| byte.is_ascii_whitespace() || byte == 0)
    {
        return Err(ProviderSetError::InvalidIdentifier { field });
    }
    Ok(())
}

fn validate_fqn(
    value: &str,
    field: &'static str,
    allow_empty: bool,
) -> Result<(), ProviderSetError> {
    if value.is_empty() && allow_empty {
        return Ok(());
    }
    if value.is_empty()
        || value.len() > MAX_FQN_BYTES
        || !value.split('.').all(|segment| {
            !segment.is_empty()
                && segment
                    .bytes()
                    .next()
                    .is_some_and(|byte| byte.is_ascii_alphabetic() || byte == b'_')
                && segment
                    .bytes()
                    .all(|byte| byte.is_ascii_alphanumeric() || byte == b'_')
        })
    {
        return Err(ProviderSetError::InvalidFqn { field });
    }
    Ok(())
}

/// A native-side typed provider owned by the simulator application.
///
/// Implementations translate model-backed sensor values into the exact
/// Protobuf payloads named by [`ProviderSet`] and translate the robot's
/// returned actuation payloads into the complete MuJoCo control vector.  No
/// provider implementation is allowed to infer a missing port or actuator.
#[cfg(feature = "native")]
pub trait NativeProvider {
    /// Provider encoding/decoding failure.
    type Error: fmt::Display;

    /// The exact immutable provider set this implementation serves.
    fn providers(&self) -> &ProviderSet;

    /// The exact set of typed actuation ports that may reach this scene.
    fn actuation_bindings(&self) -> &[ActuationBinding];

    /// Encode one complete current-boundary sensor cut.
    fn observations(
        &mut self,
        model: &Model,
        state: &StateSnapshot,
        quantum_ns: u64,
    ) -> Result<Vec<Observation>, Self::Error>;

    /// Decode the complete returned actuation cut into native scalar controls.
    fn controls(&mut self, model: &Model, actuation: &[Actuation])
    -> Result<Vec<f64>, Self::Error>;

    /// Reset provider-local cursors after the native scene has reset.
    fn reset(&mut self, _model: &Model, _state: &StateSnapshot) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Errors returned by the common typed MuJoCo payload helpers.
#[cfg(feature = "native")]
#[derive(Debug, thiserror::Error)]
pub enum NativeProviderError {
    /// A model-local native binding could not be resolved.
    #[error("native model binding failed: {0}")]
    Model(#[from] crate::mujoco::ModelError),
    /// A Protobuf payload could not be decoded.
    #[error("typed provider payload could not be decoded: {0}")]
    Decode(#[from] prost::DecodeError),
    /// A typed payload failed domain validation.
    #[error("typed provider payload is invalid: {0}")]
    InvalidPayload(String),
    /// A returned actuation does not identify a configured native actuator.
    #[error("returned actuation is invalid: {0}")]
    InvalidActuation(String),
    /// A model exposes the named object but not the complete physical
    /// semantics required by the selected public capability.
    #[error("native provider capability is unsupported: {0}")]
    Unsupported(String),
}

/// Bundle-selected and caller-selected provenance facts required for a run.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ProvenanceInput {
    /// Authored robot identity obtained from the admitted bundle.
    robot_bundle_identity: String,
    /// Caller-selected finite run identity.
    run_id: String,
}

impl ProvenanceInput {
    /// Construct run facts from the exact immutable robot bundle selected by
    /// cargo-phoxal and the caller's run identity.
    pub fn new(
        robot_bundle_identity: impl Into<String>,
        run_id: impl Into<String>,
    ) -> Result<Self, ProvenanceError> {
        let input = Self {
            robot_bundle_identity: robot_bundle_identity.into(),
            run_id: run_id.into(),
        };
        for (field, value) in [
            ("robot bundle identity", &input.robot_bundle_identity),
            ("run identity", &input.run_id),
        ] {
            if value.is_empty()
                || value.len() > MAX_FQN_BYTES
                || value.chars().any(char::is_whitespace)
            {
                return Err(ProvenanceError::InvalidField { field });
            }
        }
        Ok(input)
    }
}

#[cfg(feature = "native")]
const APPLICATION_IDENTITY: &str = concat!(env!("CARGO_PKG_NAME"), "@", env!("CARGO_PKG_VERSION"));

/// Exact app/native/scene/model/robot-bundle/run evidence retained by a run.
#[derive(Clone, Debug, Eq, PartialEq, serde::Serialize)]
pub struct SimulationProvenance {
    /// Public simulation protocol used for the run.
    pub protocol: &'static str,
    /// Simulator application identity.
    pub app_identity: String,
    /// Exact native MuJoCo version linked by the app.
    pub native_version: String,
    /// Source-authored scene identity.
    pub scene_identity: String,
    /// Closed model/resource identity.
    pub model_identity: String,
    /// Common source-authored quantum in nanoseconds.
    pub quantum_ns: u64,
    /// Immutable robot bundle identity.
    pub robot_bundle_identity: String,
    /// Finite run identity.
    pub run_id: String,
    /// Supervisor execution identity selected for the run.
    pub execution_id: String,
    /// Supervisor timeline identity at acquisition.
    pub timeline_id: String,
}

#[cfg(feature = "native")]
impl SimulationProvenance {
    fn from_input(
        input: ProvenanceInput,
        model: &Model,
        quantum_ns: u64,
        execution_id: &str,
        timeline_id: &str,
    ) -> Result<Self, ProvenanceError> {
        if quantum_ns == 0 {
            return Err(ProvenanceError::InvalidQuantum);
        }
        for (field, value) in [
            ("execution identity", execution_id),
            ("timeline identity", timeline_id),
        ] {
            if value.is_empty()
                || value.len() > MAX_FQN_BYTES
                || value.chars().any(char::is_whitespace)
            {
                return Err(ProvenanceError::InvalidField { field });
            }
        }
        let native_version = Model::native_version().to_owned();
        if native_version.is_empty() {
            return Err(ProvenanceError::InvalidField {
                field: "native version",
            });
        }
        Ok(Self {
            protocol: SIMULATION_PROTOCOL,
            app_identity: APPLICATION_IDENTITY.to_owned(),
            native_version,
            scene_identity: model.identity().to_hex(),
            model_identity: model.identity().to_hex(),
            quantum_ns,
            robot_bundle_identity: input.robot_bundle_identity,
            run_id: input.run_id,
            execution_id: execution_id.to_owned(),
            timeline_id: timeline_id.to_owned(),
        })
    }
}

/// A missing or contradictory provenance fact.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ProvenanceError {
    /// One required fact was not supplied in a bounded form.
    InvalidField { field: &'static str },
    /// The native timestep cannot be represented exactly in wire nanoseconds.
    InvalidQuantum,
}

impl fmt::Display for ProvenanceError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidField { field } => write!(formatter, "{field} is invalid"),
            Self::InvalidQuantum => {
                formatter.write_str("native quantum is not a positive nanosecond value")
            }
        }
    }
}

impl std::error::Error for ProvenanceError {}

/// A returned actuation cut and the scalar controls actually applied natively.
#[derive(Clone, Debug)]
pub struct AppliedActuation {
    /// Completed boundary after the native transition.
    pub boundary: u64,
    /// Typed actuation payloads admitted by the remote authority.
    pub requested: Vec<Actuation>,
    /// Complete native control vector applied for this transition.
    pub native_controls: Box<[f64]>,
}

/// A public-authority/native-scene coordination failure.
#[cfg(feature = "native")]
#[derive(Debug)]
pub enum RemoteSceneError<TE: fmt::Display, PE> {
    /// Public-session authority lifecycle failure.
    Authority(AuthorityClientError<TE>),
    /// Native scene mutation or snapshot failure.
    Native(SceneError),
    /// Typed provider encoding/decoding failure.
    Provider(PE),
    /// The local and remote boundary identities diverged.
    Fencing(String),
    /// A required provenance fact was not available.
    Provenance(ProvenanceError),
    /// Acquisition reached the transport but its remote result is unconfirmed.
    AcquisitionUnconfirmed(AuthorityClientError<TE>),
    /// Initialization failed after a grant and remote release did not succeed.
    CleanupUnconfirmed {
        primary: Box<Self>,
        cleanup: AuthorityClientError<TE>,
    },
}

#[cfg(feature = "native")]
impl<TE: fmt::Display, PE: fmt::Display> fmt::Display for RemoteSceneError<TE, PE> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Authority(error) => error.fmt(formatter),
            Self::Native(error) => error.fmt(formatter),
            Self::Provider(error) => write!(formatter, "native provider failed: {error}"),
            Self::Fencing(detail) => {
                write!(formatter, "simulation boundary fencing failed: {detail}")
            }
            Self::Provenance(error) => error.fmt(formatter),
            Self::AcquisitionUnconfirmed(error) => {
                write!(formatter, "authority acquisition unconfirmed: {error}")
            }
            Self::CleanupUnconfirmed { primary, cleanup } => write!(
                formatter,
                "{primary}; native authority cleanup unconfirmed: {cleanup}"
            ),
        }
    }
}

#[cfg(feature = "native")]
impl<TE: fmt::Debug + fmt::Display, PE: fmt::Debug + fmt::Display> std::error::Error
    for RemoteSceneError<TE, PE>
{
}

/// The one native MuJoCo scene coordinated through a public simulation session.
#[cfg(feature = "native")]
pub struct RemoteSceneRun<T, P>
where
    T: SimulationTransport,
    P: NativeProvider,
{
    scene: Scene,
    authority: AuthorityClient<T>,
    provider: P,
    provenance: SimulationProvenance,
    state: StateSnapshot,
    applied_actuation: Option<AppliedActuation>,
    generation: u64,
}

#[cfg(feature = "native")]
impl<T, P> fmt::Debug for RemoteSceneRun<T, P>
where
    T: SimulationTransport,
    P: NativeProvider,
{
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("RemoteSceneRun")
            .field("authority", &self.authority)
            .field("provenance", &self.provenance)
            .field("boundary", &self.state.boundary())
            .field("generation", &self.generation)
            .finish_non_exhaustive()
    }
}

#[cfg(feature = "native")]
impl<T, P> RemoteSceneRun<T, P>
where
    T: SimulationTransport,
    P: NativeProvider,
{
    /// Acquire public authority and bind one native scene to it.
    pub async fn acquire(
        scene: Scene,
        transport: T,
        mut provider: P,
        execution_id: impl Into<String>,
        provenance: ProvenanceInput,
    ) -> Result<Self, RemoteSceneError<T::Error, P::Error>> {
        let model_identity = scene.model().identity().to_hex();
        let quantum_ns = quantum_nanoseconds(scene.quantum())
            .map_err(|error| RemoteSceneError::Fencing(error.to_string()))?;
        if provider.providers().requirements().is_empty() {
            return Err(RemoteSceneError::Fencing(
                "native provider has no immutable provider requirements".to_owned(),
            ));
        }
        if provider.actuation_bindings().is_empty() {
            return Err(RemoteSceneError::Fencing(
                "native provider has no immutable actuation bindings".to_owned(),
            ));
        }
        validate_bindings_for_model(scene.model(), provider.actuation_bindings())
            .map_err(|error| RemoteSceneError::Fencing(error.to_string()))?;
        let mut authority = AuthorityClient::new(
            transport,
            execution_id,
            model_identity,
            quantum_ns,
            provider.providers().clone(),
        )
        .map_err(RemoteSceneError::Authority)?;
        if let Err(error) = authority.acquire().await {
            return Err(match error {
                AuthorityClientError::Transport(_)
                | AuthorityClientError::Protocol(_)
                | AuthorityClientError::UncertainPhase { .. } => {
                    RemoteSceneError::AcquisitionUnconfirmed(error)
                }
                _ => RemoteSceneError::Authority(error),
            });
        }
        let initialized = async {
            if authority.boundary() != 0 {
                return Err(RemoteSceneError::Fencing(
                    "authority was acquired above native boundary zero".to_owned(),
                ));
            }
            let state = match scene.snapshot() {
                Ok(state) => state,
                Err(error) => {
                    return Err(RemoteSceneError::Native(error));
                }
            };
            let timeline_id = match authority.timeline_id() {
                Some(timeline_id) => timeline_id.to_owned(),
                None => {
                    return Err(RemoteSceneError::Fencing(
                        "authority was acquired without a timeline identity".to_owned(),
                    ));
                }
            };
            let provenance = match SimulationProvenance::from_input(
                provenance,
                scene.model(),
                quantum_ns,
                authority.execution_id(),
                &timeline_id,
            ) {
                Ok(provenance) => provenance,
                Err(error) => {
                    return Err(RemoteSceneError::Provenance(error));
                }
            };
            let observations = provider
                .observations(scene.model(), &state, quantum_ns)
                .map_err(RemoteSceneError::Provider)?;
            authority
                .admit_initial(observations)
                .await
                .map_err(RemoteSceneError::Authority)?;
            Ok((state, provenance))
        }
        .await;
        let (state, provenance) = match initialized {
            Ok(initialized) => initialized,
            Err(primary) => {
                return Err(match authority.release().await {
                    Ok(_) => primary,
                    Err(cleanup) => {
                        authority.mark_application_lost();
                        RemoteSceneError::CleanupUnconfirmed {
                            primary: Box::new(primary),
                            cleanup,
                        }
                    }
                });
            }
        };
        let generation = authority.generation();
        Ok(Self {
            scene,
            authority,
            provider,
            provenance,
            state,
            applied_actuation: None,
            generation,
        })
    }

    /// Immutable run provenance.
    #[must_use]
    pub fn provenance(&self) -> &SimulationProvenance {
        &self.provenance
    }

    /// Current public authority state.
    #[must_use]
    pub fn authority_state(&self) -> AuthorityState {
        self.authority.state()
    }

    /// Current reset-scoped generation fence.
    #[must_use]
    pub fn generation(&self) -> u64 {
        self.generation
    }

    /// Current native/remote completed boundary.
    #[must_use]
    pub fn boundary(&self) -> u64 {
        self.state.boundary()
    }

    /// Current reset-scoped timeline identity.
    #[must_use]
    pub fn timeline_id(&self) -> Option<&str> {
        self.authority.timeline_id()
    }

    /// Latest copied post-step native state.
    #[must_use]
    pub fn state(&self) -> &StateSnapshot {
        &self.state
    }

    /// The controls and typed actuation admitted at the latest step.
    #[must_use]
    pub fn applied_actuation(&self) -> Option<&AppliedActuation> {
        self.applied_actuation.as_ref()
    }

    #[cfg(feature = "rendering")]
    pub(crate) fn begin_drag(
        &mut self,
        body: usize,
        anchor: [f64; 3],
        camera: crate::mujoco::ViewCamera,
        paused: bool,
    ) -> Result<(), String> {
        self.ensure_generation().map_err(|e| e.to_string())?;
        if self.authority.state() != AuthorityState::Acquired {
            return Err("native interaction requires acquired authority".into());
        }
        self.scene.drag_begin(body, anchor, camera, paused)
    }
    #[cfg(feature = "rendering")]
    pub(crate) fn update_drag(&mut self, delta: [f64; 2], paused: bool) -> Result<(), String> {
        self.ensure_generation().map_err(|e| e.to_string())?;
        if self.authority.state() != AuthorityState::Acquired {
            self.scene.drag_cancel();
            return Err("native interaction requires acquired authority".into());
        }
        if let Err(error) = self.scene.drag_update(delta, paused) {
            if self.scene.phase() == crate::mujoco::ScenePhase::Failed {
                self.authority.mark_application_lost();
                self.generation = self.authority.generation();
            }
            return Err(error);
        }
        if paused {
            self.state = self.scene.snapshot().map_err(|e| e.to_string())?;
        }
        Ok(())
    }
    #[cfg(feature = "rendering")]
    pub(crate) fn renew_drag(&mut self, received: std::time::Instant) {
        self.scene.drag_renew(received);
    }
    #[cfg(feature = "rendering")]
    pub(crate) fn cancel_drag(&mut self) {
        self.scene.drag_cancel();
    }

    /// Prepare robot outputs, integrate once, and acknowledge the resulting observations.
    pub async fn step(&mut self) -> Result<&StateSnapshot, RemoteSceneError<T::Error, P::Error>> {
        self.ensure_generation()?;
        let actuation = self
            .authority
            .prepare()
            .await
            .map_err(RemoteSceneError::Authority)?;
        self.integrate_and_admit(actuation).await
    }

    async fn integrate_and_admit(
        &mut self,
        actuation: Vec<Actuation>,
    ) -> Result<&StateSnapshot, RemoteSceneError<T::Error, P::Error>> {
        self.integrate(actuation)?;
        let observations = self
            .provider
            .observations(self.scene.model(), &self.state, self.authority.quantum_ns())
            .map_err(RemoteSceneError::Provider)?;
        self.authority
            .admit(observations)
            .await
            .map_err(RemoteSceneError::Authority)?;
        Ok(&self.state)
    }

    /// Renew the authority lease and inspect remote progress.
    pub async fn watchdog_tick(
        &mut self,
    ) -> Result<ProgressResponse, RemoteSceneError<T::Error, P::Error>> {
        self.ensure_generation()?;
        self.authority
            .watchdog_tick()
            .await
            .map_err(RemoteSceneError::Authority)
    }

    /// Reset both authority timeline and native state to boundary zero.
    pub async fn reset(&mut self) -> Result<&StateSnapshot, RemoteSceneError<T::Error, P::Error>> {
        self.ensure_generation()?;
        self.authority
            .reset()
            .await
            .map_err(RemoteSceneError::Authority)?;
        let state = match self.scene.reset() {
            Ok(state) => state,
            Err(error) => {
                self.authority.mark_application_lost();
                self.generation = self.authority.generation();
                return Err(RemoteSceneError::Native(error));
            }
        };
        if let Err(error) = self.provider.reset(self.scene.model(), &state) {
            self.authority.mark_application_lost();
            self.generation = self.authority.generation();
            return Err(RemoteSceneError::Provider(error));
        }
        self.state = state;
        let observations = self
            .provider
            .observations(self.scene.model(), &self.state, self.authority.quantum_ns())
            .map_err(RemoteSceneError::Provider)?;
        self.authority
            .admit_initial(observations)
            .await
            .map_err(RemoteSceneError::Authority)?;
        self.applied_actuation = None;
        self.generation = self.authority.generation();
        Ok(&self.state)
    }

    /// Release authority after all native work is complete.
    pub async fn release(
        &mut self,
    ) -> Result<ReleaseAuthorityResponse, RemoteSceneError<T::Error, P::Error>> {
        self.ensure_generation()?;
        #[cfg(feature = "rendering")]
        self.scene.drag_cancel();
        let response = self
            .authority
            .release()
            .await
            .map_err(RemoteSceneError::Authority)?;
        self.generation = self.authority.generation();
        Ok(response)
    }

    /// Fail closed after the application loses ownership of native state.
    pub fn mark_application_lost(&mut self) {
        self.authority.mark_application_lost();
        self.generation = self.authority.generation();
    }

    fn integrate(
        &mut self,
        actuation: Vec<Actuation>,
    ) -> Result<&StateSnapshot, RemoteSceneError<T::Error, P::Error>> {
        if let Err(error) = validate_actuation_bindings(
            self.provider.actuation_bindings(),
            &actuation,
            self.state.boundary(),
            self.authority.quantum_ns(),
        ) {
            self.authority.mark_application_lost();
            self.generation = self.authority.generation();
            return Err(RemoteSceneError::Fencing(error.to_string()));
        }
        let controls = match self.provider.controls(self.scene.model(), &actuation) {
            Ok(controls) => controls,
            Err(error) => {
                self.authority.mark_application_lost();
                self.generation = self.authority.generation();
                return Err(RemoteSceneError::Provider(error));
            }
        };
        if let Err(error) = self.scene.set_controls(&controls) {
            self.authority.mark_application_lost();
            self.generation = self.authority.generation();
            return Err(RemoteSceneError::Native(error));
        }
        let native_step = match self.scene.step() {
            Ok(step) => step,
            Err(error) => {
                self.authority.mark_application_lost();
                self.generation = self.authority.generation();
                return Err(RemoteSceneError::Native(error));
            }
        };
        if Some(native_step.end_boundary) != self.authority.boundary().checked_add(1)
            || native_step.state.model_identity() != self.scene.model().identity()
        {
            self.authority.mark_application_lost();
            self.generation = self.authority.generation();
            return Err(RemoteSceneError::Fencing(
                "native and remote completed boundaries or model identities diverged".to_owned(),
            ));
        }
        self.state = native_step.state;
        self.applied_actuation = Some(AppliedActuation {
            boundary: self.state.boundary(),
            requested: actuation,
            native_controls: controls.into_boxed_slice(),
        });
        Ok(&self.state)
    }

    fn ensure_generation(&self) -> Result<(), RemoteSceneError<T::Error, P::Error>> {
        if self.generation != self.authority.generation() {
            return Err(RemoteSceneError::Fencing(
                "native scene handle belongs to a stale authority generation".to_owned(),
            ));
        }
        Ok(())
    }
}

#[cfg(feature = "native")]
fn validate_actuation_bindings(
    bindings: &[ActuationBinding],
    actuation: &[Actuation],
    boundary: u64,
    quantum_ns: u64,
) -> Result<(), ActuationBindingError> {
    if bindings.is_empty() || actuation.len() != bindings.len() {
        return Err(ActuationBindingError::Incomplete {
            expected: bindings.len(),
            actual: actuation.len(),
        });
    }
    let expected_valid_until = boundary
        .checked_add(1)
        .and_then(|boundary| boundary.checked_mul(quantum_ns))
        .ok_or(ActuationBindingError::ValidityOverflow)?;
    let expected = bindings
        .iter()
        .map(|binding| (binding.service_instance.as_str(), binding.port.as_str()))
        .collect::<BTreeSet<_>>();
    let mut actual = BTreeSet::new();
    for item in actuation {
        let member = item
            .membership
            .as_ref()
            .ok_or(ActuationBindingError::Incomplete {
                expected: bindings.len(),
                actual: 0,
            })?;
        if member.capture_boundary > boundary
            || member.capture_time_ns
                != member
                    .capture_boundary
                    .checked_mul(quantum_ns)
                    .ok_or(ActuationBindingError::ValidityOverflow)?
            || member.producer_incarnation.is_empty()
            || member.sequence == 0
            || member.item_count != 1
            || member.disposition != ProductDisposition::Present
            || member.encoded_bytes != item.payload.len() as u64
            || member.payload_digest != Sha256::digest(&item.payload).as_slice()
        {
            return Err(ActuationBindingError::Unknown {
                service_instance: member.producer.clone(),
                port: member.port.clone(),
            });
        }
        if item.payload.is_empty() {
            return Err(ActuationBindingError::EmptyPayload {
                service_instance: member.producer.clone(),
                port: member.port.clone(),
            });
        }
        if item.valid_until_ns < expected_valid_until {
            return Err(ActuationBindingError::Expired {
                expected: expected_valid_until,
                actual: item.valid_until_ns,
            });
        }
        let key = (member.producer.as_str(), member.port.as_str());
        if !actual.insert(key) {
            return Err(ActuationBindingError::Duplicate {
                service_instance: member.producer.clone(),
                port: member.port.clone(),
            });
        }
        let Some(binding) = bindings.iter().find(|binding| {
            binding.service_instance == member.producer && binding.port == member.port
        }) else {
            return Err(ActuationBindingError::Unknown {
                service_instance: member.producer.clone(),
                port: member.port.clone(),
            });
        };
        if binding.payload_fqn.is_empty() {
            return Err(ActuationBindingError::InvalidPayloadFqn {
                service_instance: member.producer.clone(),
                port: member.port.clone(),
            });
        }
    }
    if actual != expected {
        return Err(ActuationBindingError::Incomplete {
            expected: bindings.len(),
            actual: actuation.len(),
        });
    }
    Ok(())
}

#[cfg(feature = "native")]
fn validate_binding_set(bindings: &[ActuationBinding]) -> Result<(), ActuationBindingError> {
    let mut ports = BTreeSet::new();
    let mut actuators = BTreeSet::new();
    for binding in bindings {
        if !ports.insert((binding.service_instance.as_str(), binding.port.as_str())) {
            return Err(ActuationBindingError::Duplicate {
                service_instance: binding.service_instance.clone(),
                port: binding.port.clone(),
            });
        }
        for actuator_id in binding.actuator_ids() {
            if !actuators.insert(actuator_id.as_str()) {
                return Err(ActuationBindingError::ActuatorMappedTwice {
                    actuator_id: actuator_id.clone(),
                });
            }
        }
    }
    Ok(())
}

#[cfg(feature = "native")]
pub(crate) fn validate_bindings_for_model(
    model: &Model,
    bindings: &[ActuationBinding],
) -> Result<(), ActuationBindingError> {
    validate_binding_set(bindings)?;
    let mut controls = BTreeSet::new();
    for binding in bindings {
        for actuator_id in binding.actuator_ids() {
            let handle = model
                .actuator(actuator_id)
                .map_err(|error| ActuationBindingError::NativeModel {
                    detail: error.to_string(),
                })?
                .ok_or_else(|| ActuationBindingError::UnknownNativeActuator {
                    actuator_id: actuator_id.clone(),
                })?;
            let info = model.actuator_info(handle).map_err(|error| {
                ActuationBindingError::NativeModel {
                    detail: error.to_string(),
                }
            })?;
            if !controls.insert(info.control_index) {
                return Err(ActuationBindingError::ActuatorMappedTwice {
                    actuator_id: actuator_id.clone(),
                });
            }
        }
    }
    // Unwired native controls stay inactive. Only authored routes may claim a
    // control; unknown targets and duplicate ownership remain refused above.
    Ok(())
}

/// Failure while fencing returned actuation to the native scene contract.
#[cfg(feature = "native")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ActuationBindingError {
    /// The actuation cut does not contain exactly all configured outputs.
    Incomplete { expected: usize, actual: usize },
    /// A port appears twice in one actuation cut.
    Duplicate {
        service_instance: String,
        port: String,
    },
    /// A port was not configured for this scene.
    Unknown {
        service_instance: String,
        port: String,
    },
    /// The payload has no bytes and therefore cannot carry typed actuation.
    EmptyPayload {
        service_instance: String,
        port: String,
    },
    /// A returned actuation expires before its target boundary.
    Expired { expected: u64, actual: u64 },
    /// Boundary-to-validity conversion overflowed.
    ValidityOverflow,
    /// A configured binding does not carry a payload identity.
    InvalidPayloadFqn {
        service_instance: String,
        port: String,
    },
    /// One native actuator is mapped by more than one output binding.
    ActuatorMappedTwice { actuator_id: String },
    /// A configured actuator name does not exist in the immutable model.
    UnknownNativeActuator { actuator_id: String },
    /// A model lookup failed while validating a configured native actuator.
    NativeModel { detail: String },
}

#[cfg(feature = "native")]
impl fmt::Display for ActuationBindingError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Incomplete { expected, actual } => {
                write!(
                    formatter,
                    "actuation cut has {actual} items, expected {expected}"
                )
            }
            Self::Duplicate {
                service_instance,
                port,
            } => write!(
                formatter,
                "actuation {service_instance}/{port} is duplicated"
            ),
            Self::Unknown {
                service_instance,
                port,
            } => write!(
                formatter,
                "actuation {service_instance}/{port} is not configured"
            ),
            Self::EmptyPayload {
                service_instance,
                port,
            } => write!(
                formatter,
                "actuation {service_instance}/{port} has an empty payload"
            ),
            Self::Expired { expected, actual } => write!(
                formatter,
                "actuation valid_until_ns {actual} is before required {expected}"
            ),
            Self::ValidityOverflow => {
                formatter.write_str("actuation validity time overflows nanoseconds")
            }
            Self::InvalidPayloadFqn {
                service_instance,
                port,
            } => write!(
                formatter,
                "actuation {service_instance}/{port} has no payload FQN"
            ),
            Self::ActuatorMappedTwice { actuator_id } => {
                write!(
                    formatter,
                    "native actuator {actuator_id} is mapped more than once"
                )
            }
            Self::UnknownNativeActuator { actuator_id } => {
                write!(
                    formatter,
                    "native model has no configured actuator {actuator_id}"
                )
            }
            Self::NativeModel { detail } => {
                write!(formatter, "native model binding lookup failed: {detail}")
            }
        }
    }
}

#[cfg(feature = "native")]
impl std::error::Error for ActuationBindingError {}

/// Convert an exact native timestep into the wire's nanosecond quantum.
#[cfg(feature = "native")]
pub fn quantum_nanoseconds(quantum: PhysicsQuantum) -> Result<u64, QuantumError> {
    let nanos = quantum.as_seconds() * 1_000_000_000.0;
    if !nanos.is_finite() || nanos <= 0.0 || nanos > u64::MAX as f64 {
        return Err(QuantumError::OutOfRange {
            seconds: quantum.as_seconds(),
        });
    }
    let rounded = nanos.round();
    if (nanos - rounded).abs() > 1.0e-6 {
        return Err(QuantumError::NonIntegral {
            seconds: quantum.as_seconds(),
        });
    }
    Ok(rounded as u64)
}

/// Failure converting a native timestep into an exact wire quantum.
#[cfg(feature = "native")]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum QuantumError {
    /// The value is not finite, positive, or representable as `u64` nanoseconds.
    OutOfRange { seconds: f64 },
    /// The value is finite but not an integral nanosecond count.
    NonIntegral { seconds: f64 },
}

#[cfg(feature = "native")]
impl fmt::Display for QuantumError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::OutOfRange { seconds } => {
                write!(formatter, "native quantum {seconds} is out of range")
            }
            Self::NonIntegral { seconds } => write!(
                formatter,
                "native quantum {seconds} is not integral nanoseconds"
            ),
        }
    }
}

#[cfg(feature = "native")]
impl std::error::Error for QuantumError {}

#[cfg(test)]
mod provider_tests {
    use super::*;
    use phoxal::communication::simulation::{ProductMembership, ProviderRequirement};

    fn requirement(rate_microhertz: u64) -> ProviderRequirement {
        ProviderRequirement {
            service_instance: "camera".into(),
            port: "rgb".into(),
            payload_fqn: "fixture.Frame".into(),
            input_fqn: "google.protobuf.Empty".into(),
            shape: MethodShape::Observation as i32,
            rate_microhertz,
        }
    }

    #[test]
    fn wire_provider_rate_and_not_due_membership_must_agree() {
        assert!(ProviderSet::new(vec![requirement(0)]).is_err());
        let set = ProviderSet::new(vec![requirement(30_000_000)]).unwrap();
        for (boundary, disposition, accepted) in [
            (0, ProductDisposition::NotDue, false),
            (16, ProductDisposition::Empty, false),
            (16, ProductDisposition::NotDue, true),
            (17, ProductDisposition::NotDue, false),
            (17, ProductDisposition::Empty, true),
        ] {
            let observation = Observation {
                payload: Vec::new(),
                membership: Some(ProductMembership {
                    producer: "camera".into(),
                    port: "rgb".into(),
                    capture_boundary: boundary,
                    capture_time_ns: boundary * 2_000_000,
                    sequence: boundary + 1,
                    disposition,
                    payload_digest: Sha256::digest([]).to_vec(),
                    ..Default::default()
                }),
            };
            assert_eq!(
                set.validate_observations(&[observation], boundary, 2_000_000)
                    .is_ok(),
                accepted
            );
        }
    }
}
