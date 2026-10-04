use super::*;
use phoxal::communication::session::MethodShape;
use std::sync::{Arc, Mutex};

#[derive(Clone, Default)]
struct Peer(Arc<Mutex<PeerState>>);
#[derive(Default)]
struct PeerState {
    acquired: bool,
    hardware: bool,
    boundary: u64,
    timeline: u64,
    lost_reply: Option<PhaseStatus>,
    lost_reset: bool,
    corrupt_receipt: bool,
    receipts: Vec<CutReceipt>,
    applied: Vec<PhaseStatus>,
}

impl Peer {
    fn phase(
        &self,
        key: TransitionKey,
        correlation: Vec<u8>,
        products: Vec<ProductMembership>,
        phase: PhaseStatus,
    ) -> Result<CutReceipt, String> {
        let mut state = self.0.lock().unwrap();
        let next_boundary = key.boundary + u64::from(phase == PhaseStatus::ObservationsAdmitted);
        let mut receipt = CutReceipt {
            transition_key: Some(key.clone()),
            correlation_id: correlation,
            membership_digest: membership_digest(&products),
            products,
            prepared_boundary: key.boundary,
            admitted_observation_boundary: next_boundary,
            status: phase,
        };
        state.applied.push(phase);
        state.boundary = next_boundary;
        state.receipts.push(receipt.clone());
        if state.lost_reply == Some(phase) {
            state.lost_reply = None;
            return Err("reply lost after commit".into());
        }
        if state.corrupt_receipt {
            receipt.membership_digest[0] ^= 1;
        }
        Ok(receipt)
    }
}

impl SimulationTransport for Peer {
    type Error = String;
    fn acquire_authority(
        &self,
        r: AcquireAuthorityRequest,
    ) -> SimulationFuture<'_, AcquireAuthorityResponse, String> {
        Box::pin(async move {
            let mut state = self.0.lock().unwrap();
            if state.hardware || state.acquired {
                return Err("authority unavailable".into());
            }
            state.acquired = true;
            Ok(AcquireAuthorityResponse {
                execution_id: r.execution_id,
                model_identity: r.model_identity,
                quantum_ns: r.quantum_ns,
                authority_grant: vec![7; 32],
                timeline_id: "timeline-0".into(),
                boundary: 0,
                lease_ms: 10_000,
                session_id: vec![1; 16],
                correlation_id: r.correlation_id,
                max_product_bytes: r.max_product_bytes,
                max_cut_bytes: r.max_cut_bytes,
            })
        })
    }
    fn admit_initial_observations(
        &self,
        r: AdmitInitialObservationsRequest,
    ) -> SimulationFuture<'_, AdmitInitialObservationsResponse, String> {
        Box::pin(async move {
            let receipt = self.phase(
                r.transition_key.clone().unwrap(),
                r.correlation_id.clone(),
                r.observations
                    .iter()
                    .map(|o| o.membership.clone().unwrap())
                    .collect(),
                PhaseStatus::InitialAdmitted,
            )?;
            Ok(AdmitInitialObservationsResponse {
                receipt: Some(receipt),
            })
        })
    }
    fn prepare_boundary(
        &self,
        r: PrepareBoundaryRequest,
    ) -> SimulationFuture<'_, PrepareBoundaryResponse, String> {
        Box::pin(async move {
            let key = r.transition_key.as_ref().unwrap();
            let payload = phoxal::contracts::component::actuator::ActuatorSetpoint {
                targets: vec![phoxal::contracts::component::actuator::ActuatorTarget {
                    actuator_id: "motor".into(),
                    control: Some(
                        phoxal::contracts::component::actuator::Control::VelocityRadps(1.0),
                    ),
                }],
            }
            .encode_to_vec();
            let member = ProductMembership {
                producer: "motion".into(),
                port: crate::contract::simulator_api::ACTUATORS
                    .signature()
                    .endpoint
                    .into(),
                producer_incarnation: vec![2; 32],
                sequence: key.boundary + 1,
                capture_boundary: key.boundary,
                capture_time_ns: key.boundary * 2_000_000,
                disposition: ProductDisposition::Present,
                item_count: 1,
                encoded_bytes: payload.len() as u64,
                payload_digest: Sha256::digest(&payload).to_vec(),
            };
            let actuation = vec![Actuation {
                membership: Some(member.clone()),
                payload,
                valid_until_ns: (key.boundary + 1) * 2_000_000,
            }];
            let receipt = self.phase(
                key.clone(),
                r.correlation_id.clone(),
                vec![member],
                PhaseStatus::Prepared,
            )?;
            Ok(PrepareBoundaryResponse {
                receipt: Some(receipt),
                actuation,
            })
        })
    }
    fn admit_observations(
        &self,
        r: AdmitObservationsRequest,
    ) -> SimulationFuture<'_, AdmitObservationsResponse, String> {
        Box::pin(async move {
            let receipt = self.phase(
                r.transition_key.clone().unwrap(),
                r.correlation_id.clone(),
                r.observations
                    .iter()
                    .map(|o| o.membership.clone().unwrap())
                    .collect(),
                PhaseStatus::ObservationsAdmitted,
            )?;
            Ok(AdmitObservationsResponse {
                receipt: Some(receipt),
            })
        })
    }
    fn progress(&self, r: ProgressRequest) -> SimulationFuture<'_, ProgressResponse, String> {
        Box::pin(async move {
            let state = self.0.lock().unwrap();
            Ok(ProgressResponse {
                execution_id: "execution".into(),
                timeline_id: format!("timeline-{}", state.timeline),
                completed_boundary: state.boundary,
                session_id: r.session_id,
                authority_grant: r.authority_grant,
                correlation_id: r.correlation_id,
                ..Default::default()
            })
        })
    }
    fn reset(&self, r: ResetRequest) -> SimulationFuture<'_, ResetResponse, String> {
        Box::pin(async move {
            let mut state = self.0.lock().unwrap();
            state.timeline += 1;
            state.boundary = 0;
            state.receipts.clear();
            let response = ResetResponse {
                next_timeline_id: format!("timeline-{}", state.timeline),
                boundary: 0,
                session_id: r.session_id.clone(),
                execution_id: r.execution_id.clone(),
                authority_grant: format!("grant-{}", state.timeline).into_bytes(),
                correlation_id: r.correlation_id.clone(),
                previous_timeline_id: r.timeline_id.clone(),
                requested_boundary: r.completed_boundary,
            };
            if state.lost_reset {
                state.lost_reset = false;
                return Err("reset reply lost after commit".into());
            }
            Ok(response)
        })
    }
    fn release_authority(
        &self,
        r: ReleaseAuthorityRequest,
    ) -> SimulationFuture<'_, ReleaseAuthorityResponse, String> {
        Box::pin(async move {
            let mut state = self.0.lock().unwrap();
            state.acquired = false;
            Ok(ReleaseAuthorityResponse {
                execution_id: "execution".into(),
                timeline_id: format!("timeline-{}", state.timeline),
                completed_boundary: state.boundary,
                session_id: r.session_id,
                authority_grant: r.authority_grant,
                correlation_id: r.correlation_id,
            })
        })
    }
}

fn providers() -> ProviderSet {
    let port = crate::contract::simulator_api::ENCODER.signature();
    ProviderSet::new(vec![ProviderRequirement {
        rate_microhertz: 500_000_000,
        service_instance: "wheel".into(),
        port: port.endpoint.into(),
        payload_fqn: port.response.into(),
        input_fqn: port.request.into(),
        shape: MethodShape::Observation as i32,
    }])
    .unwrap()
}
fn client(peer: Peer) -> AuthorityClient<Peer> {
    AuthorityClient::new(peer, "execution", "model", 2_000_000, providers()).unwrap()
}
fn observation(boundary: u64) -> Vec<Observation> {
    let payload = vec![8, 1];
    vec![Observation {
        membership: Some(ProductMembership {
            producer: "wheel".into(),
            port: "encoder".into(),
            sequence: boundary + 1,
            capture_boundary: boundary,
            capture_time_ns: boundary * 2_000_000,
            disposition: ProductDisposition::Present,
            item_count: 1,
            encoded_bytes: payload.len() as u64,
            payload_digest: Sha256::digest(&payload).to_vec(),
            ..Default::default()
        }),
        payload,
    }]
}

#[tokio::test]
async fn authority_is_exclusive_and_hardware_refusal_does_not_mutate_client() {
    let peer = Peer::default();
    let mut first = client(peer.clone());
    first.acquire().await.unwrap();
    assert!(client(peer).acquire().await.is_err());
    let hardware = Peer::default();
    hardware.0.lock().unwrap().hardware = true;
    let mut client = client(hardware);
    assert!(client.acquire().await.is_err());
    assert_eq!(client.state(), AuthorityState::Disconnected);
}

#[tokio::test]
async fn lost_phase_reply_fails_without_a_second_mutation() {
    for phase in [
        PhaseStatus::InitialAdmitted,
        PhaseStatus::Prepared,
        PhaseStatus::ObservationsAdmitted,
    ] {
        let peer = Peer::default();
        let mut client = client(peer.clone());
        client.acquire().await.unwrap();
        peer.0.lock().unwrap().lost_reply = Some(phase);
        if phase == PhaseStatus::InitialAdmitted {
            assert!(client.admit_initial(observation(0)).await.is_err());
        } else {
            client.admit_initial(observation(0)).await.unwrap();
            if phase == PhaseStatus::Prepared {
                assert!(client.prepare().await.is_err());
            } else {
                client.prepare().await.unwrap();
                assert!(client.admit(observation(1)).await.is_err());
            }
        }
        assert_eq!(client.state(), AuthorityState::Failed);
        let applied = peer.0.lock().unwrap().applied.len();
        assert!(client.prepare().await.is_err());
        assert!(client.reset().await.is_err());
        assert_eq!(peer.0.lock().unwrap().applied.len(), applied);
    }
}

#[tokio::test]
async fn mismatched_receipt_fails_closed() {
    let peer = Peer::default();
    let mut client = client(peer.clone());
    client.acquire().await.unwrap();
    peer.0.lock().unwrap().corrupt_receipt = true;
    assert!(client.admit_initial(observation(0)).await.is_err());
    assert_eq!(client.state(), AuthorityState::Failed);
    assert!(client.prepare().await.is_err());
}

#[tokio::test]
async fn reset_rotates_timeline_and_requires_new_initial_cut() {
    let peer = Peer::default();
    let mut client = client(peer.clone());
    client.acquire().await.unwrap();
    client.admit_initial(observation(0)).await.unwrap();
    let previous_incarnation = peer.0.lock().unwrap().receipts[0].products[0]
        .producer_incarnation
        .clone();
    client.reset().await.unwrap();
    assert_eq!(client.generation(), 2);
    assert_eq!(client.boundary(), 0);
    assert!(client.prepare().await.is_err());
    client.admit_initial(observation(0)).await.unwrap();
    let state = peer.0.lock().unwrap();
    let receipt = &state.receipts[0];
    assert_ne!(
        receipt.products[0].producer_incarnation,
        previous_incarnation
    );
    assert_eq!(
        receipt.transition_key.as_ref().unwrap().operation_sequence,
        1
    );
    assert_eq!(
        receipt.transition_key.as_ref().unwrap().timeline_id,
        "timeline-1"
    );
}

#[tokio::test]
async fn lost_reply_stops_native_run_without_reintegration() {
    use crate::mujoco::{Model, Scene};
    use crate::{
        native_provider::{
            ActuationDeclaration, ActuatorTarget, ComponentProvider, NativeControlMode,
            ObservationBinding,
        },
        remote::{ProvenanceInput, RemoteSceneRun},
    };
    for lost in [PhaseStatus::Prepared, PhaseStatus::ObservationsAdmitted] {
        let model = Model::from_xml(include_str!("../../tests/fixtures/motor.xml")).unwrap();
        let provider = ComponentProvider::new(
            &model,
            providers(),
            [ObservationBinding::ddsm115_encoder_joint(
                "wheel",
                "motor_joint",
            )],
            [ActuationDeclaration::motion(
                "motion",
                [ActuatorTarget::new(
                    "motor",
                    "motor",
                    NativeControlMode::Velocity,
                )],
            )],
            std::collections::BTreeMap::from([(
                ("wheel".into(), "encoder".into()),
                crate::native_provider::Cadence::new(500.0, 2_000_000).unwrap(),
            )]),
        )
        .unwrap();
        let peer = Peer::default();
        let mut run = RemoteSceneRun::acquire(
            Scene::new(model).unwrap(),
            peer.clone(),
            provider,
            "execution",
            ProvenanceInput::new("bundle", "run").unwrap(),
        )
        .await
        .unwrap();
        peer.0.lock().unwrap().lost_reply = Some(lost);
        assert!(run.step().await.is_err());
        assert_eq!(
            run.state().boundary(),
            u64::from(lost == PhaseStatus::ObservationsAdmitted)
        );
        assert_eq!(run.authority_state(), AuthorityState::Failed);
        let applied = peer.0.lock().unwrap().applied.len();
        assert!(run.step().await.is_err());
        assert!(run.reset().await.is_err());
        assert_eq!(peer.0.lock().unwrap().applied.len(), applied);
        assert_eq!(
            run.state().time_seconds(),
            if lost == PhaseStatus::Prepared {
                0.0
            } else {
                0.002
            }
        );
    }
}

#[tokio::test]
async fn lost_reset_reply_fails_without_retry() {
    let peer = Peer::default();
    let mut client = client(peer.clone());
    client.acquire().await.unwrap();
    client.admit_initial(observation(0)).await.unwrap();
    peer.0.lock().unwrap().lost_reset = true;
    assert!(client.reset().await.is_err());
    assert_eq!(client.state(), AuthorityState::Failed);
    assert!(client.reset().await.is_err());
    assert_eq!(peer.0.lock().unwrap().timeline, 1);
}
