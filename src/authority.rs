//! Exclusive simulation authority and fail-fast three-phase transitions.

use crate::remote::{ProviderSet, ProviderSetError};
use phoxal::communication::simulation::*;
use phoxal::session::{SessionError, Simulation};
use prost::Message;
use sha2::{Digest, Sha256};
use std::{
    fmt,
    future::Future,
    pin::Pin,
    time::{Duration, Instant},
};

struct ObservationCut {
    observations: Vec<Observation>,
    products: Vec<ProductMembership>,
}

pub type SimulationFuture<'a, T, E> = Pin<Box<dyn Future<Output = Result<T, E>> + Send + 'a>>;

pub trait SimulationTransport: Clone + Send + Sync + 'static {
    type Error: fmt::Display + Send + Sync + 'static;
    fn acquire_authority(
        &self,
        request: AcquireAuthorityRequest,
    ) -> SimulationFuture<'_, AcquireAuthorityResponse, Self::Error>;
    fn admit_initial_observations(
        &self,
        request: AdmitInitialObservationsRequest,
    ) -> SimulationFuture<'_, AdmitInitialObservationsResponse, Self::Error>;
    fn prepare_boundary(
        &self,
        request: PrepareBoundaryRequest,
    ) -> SimulationFuture<'_, PrepareBoundaryResponse, Self::Error>;
    fn admit_observations(
        &self,
        request: AdmitObservationsRequest,
    ) -> SimulationFuture<'_, AdmitObservationsResponse, Self::Error>;
    fn reset(&self, request: ResetRequest) -> SimulationFuture<'_, ResetResponse, Self::Error>;
    fn release_authority(
        &self,
        request: ReleaseAuthorityRequest,
    ) -> SimulationFuture<'_, ReleaseAuthorityResponse, Self::Error>;
    fn progress(
        &self,
        request: ProgressRequest,
    ) -> SimulationFuture<'_, ProgressResponse, Self::Error>;
}

impl SimulationTransport for Simulation {
    type Error = SessionError;
    fn acquire_authority(
        &self,
        request: AcquireAuthorityRequest,
    ) -> SimulationFuture<'_, AcquireAuthorityResponse, Self::Error> {
        Box::pin(self.acquire_authority(request))
    }
    fn admit_initial_observations(
        &self,
        request: AdmitInitialObservationsRequest,
    ) -> SimulationFuture<'_, AdmitInitialObservationsResponse, Self::Error> {
        Box::pin(self.admit_initial_observations(request))
    }
    fn prepare_boundary(
        &self,
        request: PrepareBoundaryRequest,
    ) -> SimulationFuture<'_, PrepareBoundaryResponse, Self::Error> {
        Box::pin(self.prepare_boundary(request))
    }
    fn admit_observations(
        &self,
        request: AdmitObservationsRequest,
    ) -> SimulationFuture<'_, AdmitObservationsResponse, Self::Error> {
        Box::pin(self.admit_observations(request))
    }
    fn reset(&self, request: ResetRequest) -> SimulationFuture<'_, ResetResponse, Self::Error> {
        Box::pin(self.reset(request))
    }
    fn release_authority(
        &self,
        request: ReleaseAuthorityRequest,
    ) -> SimulationFuture<'_, ReleaseAuthorityResponse, Self::Error> {
        Box::pin(self.release_authority(request))
    }
    fn progress(
        &self,
        request: ProgressRequest,
    ) -> SimulationFuture<'_, ProgressResponse, Self::Error> {
        Box::pin(self.progress(request))
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AuthorityState {
    Disconnected,
    Acquired,
    Lost,
    Failed,
    Released,
}

#[derive(Debug, thiserror::Error)]
pub enum AuthorityClientError<E: fmt::Display> {
    #[error("cannot {operation} in authority state {state:?}")]
    InvalidState {
        operation: &'static str,
        state: AuthorityState,
    },
    #[error("invalid simulation request: {0}")]
    InvalidRequest(String),
    #[error("{0}")]
    ProviderSet(ProviderSetError),
    #[error("public simulation transport failed: {0}")]
    Transport(E),
    #[error("simulation phase {phase:?} at boundary {boundary} is uncertain: {error}")]
    UncertainPhase {
        error: E,
        phase: PhaseStatus,
        boundary: u64,
    },
    #[error("simulation protocol violation: {0}")]
    Protocol(String),
    #[error("simulation authority lease expired")]
    LeaseExpired,
}

#[derive(Clone, Debug)]
enum PhaseRequest {
    Initial(AdmitInitialObservationsRequest),
    Prepare(PrepareBoundaryRequest),
    Observations(AdmitObservationsRequest),
}

#[derive(Clone, Debug)]
struct PendingPhase {
    request: PhaseRequest,
    key: TransitionKey,
    correlation: Vec<u8>,
    products: Vec<ProductMembership>,
    status: PhaseStatus,
}

#[derive(Debug)]
pub struct CompletedPhase {
    pub actuation: Vec<Actuation>,
}

pub struct AuthorityClient<T> {
    transport: T,
    providers: ProviderSet,
    execution_id: String,
    model_identity: String,
    quantum_ns: u64,
    session_id: Vec<u8>,
    authority_grant: Vec<u8>,
    timeline_id: String,
    boundary: u64,
    generation: u64,
    sequence: u64,
    correlation: u64,
    state: AuthorityState,
    initialized: bool,
    prepared: bool,
    lease: Duration,
    deadline: Option<Instant>,
}

impl<T> fmt::Debug for AuthorityClient<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AuthorityClient")
            .field("state", &self.state)
            .field("boundary", &self.boundary)
            .finish_non_exhaustive()
    }
}

pub fn membership_digest(products: &[ProductMembership]) -> Vec<u8> {
    let mut encoded: Vec<_> = products.iter().map(Message::encode_to_vec).collect();
    encoded.sort();
    let mut hash = Sha256::new();
    for item in encoded {
        hash.update((item.len() as u64).to_be_bytes());
        hash.update(item);
    }
    hash.finalize().to_vec()
}

impl<T: SimulationTransport> AuthorityClient<T> {
    pub fn new(
        transport: T,
        execution_id: impl Into<String>,
        model_identity: impl Into<String>,
        quantum_ns: u64,
        providers: ProviderSet,
    ) -> Result<Self, AuthorityClientError<T::Error>> {
        let execution_id = execution_id.into();
        let model_identity = model_identity.into();
        if quantum_ns == 0
            || [&execution_id, &model_identity].iter().any(|id| {
                id.is_empty()
                    || id.len() > 512
                    || !id.is_ascii()
                    || id.bytes().any(|b| b.is_ascii_whitespace())
            })
        {
            return Err(AuthorityClientError::InvalidRequest(
                "execution, model identity, or quantum is invalid".into(),
            ));
        }
        Ok(Self {
            transport,
            providers,
            execution_id,
            model_identity,
            quantum_ns,
            session_id: Vec::new(),
            authority_grant: Vec::new(),
            timeline_id: String::new(),
            boundary: 0,
            generation: 0,
            sequence: 1,
            correlation: 0,
            state: AuthorityState::Disconnected,
            initialized: false,
            prepared: false,
            lease: Duration::ZERO,
            deadline: None,
        })
    }
    pub fn execution_id(&self) -> &str {
        &self.execution_id
    }
    pub fn quantum_ns(&self) -> u64 {
        self.quantum_ns
    }
    pub fn state(&self) -> AuthorityState {
        self.state
    }
    pub fn generation(&self) -> u64 {
        self.generation
    }
    pub fn boundary(&self) -> u64 {
        self.boundary
    }
    pub fn timeline_id(&self) -> Option<&str> {
        (!self.timeline_id.is_empty()).then_some(self.timeline_id.as_str())
    }

    fn correlation(&mut self) -> Result<Vec<u8>, AuthorityClientError<T::Error>> {
        self.correlation = self
            .correlation
            .checked_add(1)
            .ok_or_else(|| self.protocol("correlation sequence exhausted"))?;
        Ok([
            self.generation.to_be_bytes(),
            self.correlation.to_be_bytes(),
        ]
        .concat())
    }
    fn key(&self) -> TransitionKey {
        TransitionKey {
            session_id: self.session_id.clone(),
            execution_id: self.execution_id.clone(),
            timeline_id: self.timeline_id.clone(),
            authority_grant: self.authority_grant.clone(),
            boundary: self.boundary,
            operation_sequence: self.sequence,
        }
    }
    fn protocol(&self, message: &str) -> AuthorityClientError<T::Error> {
        AuthorityClientError::Protocol(message.into())
    }
    fn ensure_live(
        &mut self,
        operation: &'static str,
    ) -> Result<(), AuthorityClientError<T::Error>> {
        if self.state != AuthorityState::Acquired {
            return Err(AuthorityClientError::InvalidState {
                operation,
                state: self.state,
            });
        }
        if self
            .deadline
            .is_none_or(|deadline| Instant::now() >= deadline)
        {
            self.mark_application_lost();
            return Err(AuthorityClientError::LeaseExpired);
        }
        Ok(())
    }
    fn refresh_lease(&mut self) -> Result<(), AuthorityClientError<T::Error>> {
        self.deadline = Instant::now().checked_add(self.lease);
        if self.deadline.is_none() {
            return Err(self.protocol("lease deadline overflow"));
        }
        Ok(())
    }
    pub async fn acquire(&mut self) -> Result<(), AuthorityClientError<T::Error>> {
        if self.state != AuthorityState::Disconnected {
            return Err(AuthorityClientError::InvalidState {
                operation: "acquire",
                state: self.state,
            });
        }
        let correlation = self.correlation()?;
        let response = self
            .transport
            .acquire_authority(AcquireAuthorityRequest {
                execution_id: self.execution_id.clone(),
                model_identity: self.model_identity.clone(),
                quantum_ns: self.quantum_ns,
                providers: self.providers.requirements().to_vec(),
                correlation_id: correlation.clone(),
                max_product_bytes: 4 * 1024 * 1024,
                max_cut_bytes: 8 * 1024 * 1024,
                session_id: Vec::new(),
            })
            .await
            .map_err(AuthorityClientError::Transport)?;
        if response.authority_grant.len() != 32
            || response.session_id.is_empty()
            || response.timeline_id.is_empty()
            || response.boundary != 0
            || response.lease_ms == 0
            || response.execution_id != self.execution_id
            || response.model_identity != self.model_identity
            || response.quantum_ns != self.quantum_ns
            || response.correlation_id != correlation
            || response.max_product_bytes != 4 * 1024 * 1024
            || response.max_cut_bytes != 8 * 1024 * 1024
        {
            self.state = AuthorityState::Failed;
            return Err(self.protocol("acquisition identity or capacity mismatch"));
        }
        self.session_id = response.session_id;
        self.authority_grant = response.authority_grant;
        self.timeline_id = response.timeline_id;
        self.generation = self
            .generation
            .checked_add(1)
            .ok_or_else(|| self.protocol("generation exhausted"))?;
        self.lease = Duration::from_millis(u64::from(response.lease_ms));
        self.refresh_lease()?;
        self.state = AuthorityState::Acquired;
        Ok(())
    }

    fn observations(
        &self,
        mut observations: Vec<Observation>,
        boundary: u64,
    ) -> Result<ObservationCut, AuthorityClientError<T::Error>> {
        self.providers
            .validate_observations(&observations, boundary, self.quantum_ns)
            .map_err(AuthorityClientError::ProviderSet)?;
        let mut products = Vec::with_capacity(observations.len());
        let mut bytes = 0usize;
        for observation in &mut observations {
            let member = observation
                .membership
                .as_mut()
                .ok_or_else(|| self.protocol("observation membership absent"))?;
            let mut hash = Sha256::new();
            hash.update(&self.authority_grant);
            hash.update(self.timeline_id.as_bytes());
            hash.update(member.producer.as_bytes());
            member.producer_incarnation = hash.finalize().to_vec();
            products.push(member.clone());
            bytes = bytes
                .checked_add(observation.encoded_len())
                .ok_or_else(|| self.protocol("observation byte overflow"))?;
            if bytes > 8 * 1024 * 1024 {
                return Err(self.protocol("observation cut exceeds negotiated byte capacity"));
            }
        }
        Ok(ObservationCut {
            observations,
            products,
        })
    }
    pub async fn admit_initial(
        &mut self,
        observations: Vec<Observation>,
    ) -> Result<(), AuthorityClientError<T::Error>> {
        self.ensure_live("admit initial observations")?;
        if self.initialized || self.prepared || self.boundary != 0 {
            return Err(self.protocol("initial cut already admitted"));
        }
        let ObservationCut {
            observations,
            products,
        } = self.observations(observations, 0)?;
        let key = self.key();
        let correlation = self.correlation()?;
        let request = AdmitInitialObservationsRequest {
            transition_key: Some(key.clone()),
            observations,
            membership_digest: membership_digest(&products),
            correlation_id: correlation.clone(),
        };
        let pending = PendingPhase {
            request: PhaseRequest::Initial(request),
            key,
            correlation,
            products,
            status: PhaseStatus::InitialAdmitted,
        };
        self.submit(pending).await.map(|_| ())
    }
    pub async fn prepare(&mut self) -> Result<Vec<Actuation>, AuthorityClientError<T::Error>> {
        self.ensure_live("prepare boundary")?;
        if !self.initialized || self.prepared {
            return Err(self.protocol("boundary is not ready for preparation"));
        }
        let key = self.key();
        let correlation = self.correlation()?;
        let request = PrepareBoundaryRequest {
            transition_key: Some(key.clone()),
            correlation_id: correlation.clone(),
        };
        let pending = PendingPhase {
            request: PhaseRequest::Prepare(request),
            key,
            correlation,
            products: Vec::new(),
            status: PhaseStatus::Prepared,
        };
        self.submit(pending).await.map(|done| done.actuation)
    }
    pub async fn admit(
        &mut self,
        observations: Vec<Observation>,
    ) -> Result<(), AuthorityClientError<T::Error>> {
        self.ensure_live("admit observations")?;
        if !self.prepared {
            return Err(self.protocol("native observations have no prepared transition"));
        }
        let boundary = self
            .boundary
            .checked_add(1)
            .ok_or_else(|| self.protocol("boundary overflow"))?;
        let ObservationCut {
            observations,
            products,
        } = self.observations(observations, boundary)?;
        let key = self.key();
        let correlation = self.correlation()?;
        let request = AdmitObservationsRequest {
            transition_key: Some(key.clone()),
            observations,
            membership_digest: membership_digest(&products),
            correlation_id: correlation.clone(),
        };
        let pending = PendingPhase {
            request: PhaseRequest::Observations(request),
            key,
            correlation,
            products,
            status: PhaseStatus::ObservationsAdmitted,
        };
        self.submit(pending).await.map(|_| ())
    }
    async fn submit(
        &mut self,
        pending: PendingPhase,
    ) -> Result<CompletedPhase, AuthorityClientError<T::Error>> {
        self.dispatch(&pending).await
    }
    async fn dispatch(
        &mut self,
        pending: &PendingPhase,
    ) -> Result<CompletedPhase, AuthorityClientError<T::Error>> {
        let result = match &pending.request {
            PhaseRequest::Initial(request) => self
                .transport
                .admit_initial_observations(request.clone())
                .await
                .map(|r| (r.receipt, Vec::new())),
            PhaseRequest::Prepare(request) => self
                .transport
                .prepare_boundary(request.clone())
                .await
                .map(|r| (r.receipt, r.actuation)),
            PhaseRequest::Observations(request) => self
                .transport
                .admit_observations(request.clone())
                .await
                .map(|r| (r.receipt, Vec::new())),
        };
        let (receipt, actuation) = result.map_err(|error| {
            self.state = AuthorityState::Failed;
            AuthorityClientError::UncertainPhase {
                error,
                phase: pending.status,
                boundary: pending.key.boundary,
            }
        })?;
        let Some(receipt) = receipt else {
            self.state = AuthorityState::Failed;
            return Err(self.protocol("phase receipt missing"));
        };
        if receipt.encoded_len() > 512 * 1024
            || actuation
                .iter()
                .any(|a| a.membership.is_none() || a.payload.len() > 4 * 1024 * 1024)
        {
            self.state = AuthorityState::Failed;
            return Err(self.protocol("phase receipt or actuation violates the negotiated bounds"));
        }
        let products = if pending.status == PhaseStatus::Prepared {
            actuation
                .iter()
                .map(|a| {
                    a.membership
                        .clone()
                        .ok_or_else(|| self.protocol("actuation membership absent"))
                })
                .collect::<Result<Vec<_>, _>>()?
        } else {
            pending.products.clone()
        };
        let expected_admitted = if pending.status == PhaseStatus::ObservationsAdmitted {
            self.boundary
                .checked_add(1)
                .ok_or_else(|| self.protocol("boundary overflow"))?
        } else {
            self.boundary
        };
        if receipt.transition_key.as_ref() != Some(&pending.key)
            || receipt.correlation_id != pending.correlation
            || receipt.status != pending.status as i32
            || receipt.membership_digest != membership_digest(&products)
            || membership_digest(&receipt.products) != membership_digest(&products)
            || receipt.prepared_boundary != self.boundary
            || receipt.admitted_observation_boundary != expected_admitted
        {
            self.state = AuthorityState::Failed;
            return Err(self.protocol("receipt does not match the exact transition and products"));
        }
        if actuation
            .iter()
            .map(Message::encoded_len)
            .try_fold(0usize, usize::checked_add)
            .is_none_or(|bytes| bytes > 8 * 1024 * 1024)
        {
            self.state = AuthorityState::Failed;
            return Err(self.protocol("actuation cut exceeds byte capacity"));
        }
        match pending.status {
            PhaseStatus::InitialAdmitted => {
                self.initialized = true;
                self.sequence = self
                    .sequence
                    .checked_add(1)
                    .ok_or_else(|| self.protocol("operation sequence exhausted"))?;
            }
            PhaseStatus::Prepared => self.prepared = true,
            PhaseStatus::ObservationsAdmitted => {
                self.boundary = expected_admitted;
                self.prepared = false;
                self.sequence = self
                    .sequence
                    .checked_add(1)
                    .ok_or_else(|| self.protocol("operation sequence exhausted"))?;
            }
            _ => return Err(self.protocol("invalid local phase")),
        }
        self.refresh_lease()?;
        Ok(CompletedPhase { actuation })
    }
    async fn progress(&mut self) -> Result<ProgressResponse, AuthorityClientError<T::Error>> {
        let correlation = self.correlation()?;
        let response = self
            .transport
            .progress(ProgressRequest {
                authority_grant: self.authority_grant.clone(),
                session_id: self.session_id.clone(),
                correlation_id: correlation.clone(),
            })
            .await
            .map_err(AuthorityClientError::Transport)?;
        if response.failed
            || response.session_id != self.session_id
            || response.authority_grant != self.authority_grant
            || response.execution_id != self.execution_id
            || response.timeline_id != self.timeline_id
            || response.correlation_id != correlation
        {
            self.state = AuthorityState::Failed;
            return Err(self.protocol("progress identity mismatch or remote execution failure"));
        }
        self.refresh_lease()?;
        Ok(response)
    }
    pub async fn watchdog_tick(
        &mut self,
    ) -> Result<ProgressResponse, AuthorityClientError<T::Error>> {
        self.ensure_live("renew authority")?;
        let response = self.progress().await?;
        if response.completed_boundary != self.boundary {
            self.state = AuthorityState::Failed;
            return Err(self.protocol("remote completed boundary diverged"));
        }
        Ok(response)
    }
    pub async fn reset(&mut self) -> Result<ResetResponse, AuthorityClientError<T::Error>> {
        self.ensure_live("reset")?;
        if self.prepared {
            return Err(self.protocol("reset requires a completed transition"));
        }
        let correlation = self.correlation()?;
        let request = ResetRequest {
            authority_grant: self.authority_grant.clone(),
            execution_id: self.execution_id.clone(),
            timeline_id: self.timeline_id.clone(),
            completed_boundary: self.boundary,
            session_id: self.session_id.clone(),
            correlation_id: correlation.clone(),
        };
        let response = match self.transport.reset(request).await {
            Ok(r) => r,
            Err(e) => {
                self.state = AuthorityState::Failed;
                return Err(AuthorityClientError::Transport(e));
            }
        };
        if response.session_id != self.session_id
            || response.authority_grant.is_empty()
            || response.authority_grant == self.authority_grant
            || response.execution_id != self.execution_id
            || response.previous_timeline_id != self.timeline_id
            || response.next_timeline_id.is_empty()
            || response.next_timeline_id == self.timeline_id
            || response.requested_boundary != self.boundary
            || response.boundary != 0
            || response.correlation_id != correlation
        {
            self.state = AuthorityState::Failed;
            return Err(self.protocol("reset response identity mismatch"));
        }
        self.authority_grant = response.authority_grant.clone();
        self.timeline_id = response.next_timeline_id.clone();
        self.boundary = 0;
        self.sequence = 1;
        self.initialized = false;
        self.prepared = false;
        self.generation = self
            .generation
            .checked_add(1)
            .ok_or_else(|| self.protocol("generation exhausted"))?;
        self.refresh_lease()?;
        Ok(response)
    }
    pub async fn release(
        &mut self,
    ) -> Result<ReleaseAuthorityResponse, AuthorityClientError<T::Error>> {
        self.ensure_live("release")?;
        if self.prepared {
            return Err(self.protocol("release requires a completed transition"));
        }
        let correlation = self.correlation()?;
        let response = self
            .transport
            .release_authority(ReleaseAuthorityRequest {
                authority_grant: self.authority_grant.clone(),
                session_id: self.session_id.clone(),
                correlation_id: correlation.clone(),
            })
            .await
            .map_err(AuthorityClientError::Transport)?;
        if response.session_id != self.session_id
            || response.authority_grant != self.authority_grant
            || response.execution_id != self.execution_id
            || response.timeline_id != self.timeline_id
            || response.completed_boundary != self.boundary
            || response.correlation_id != correlation
        {
            self.state = AuthorityState::Failed;
            return Err(self.protocol("release response identity mismatch"));
        }
        self.state = AuthorityState::Released;
        self.authority_grant.clear();
        self.deadline = None;
        Ok(response)
    }
    pub fn mark_application_lost(&mut self) {
        self.state = AuthorityState::Lost;
        self.authority_grant.clear();
        self.deadline = None;
        self.generation = self.generation.saturating_add(1);
    }
}

#[cfg(test)]
mod tests;
