//! Sensor payloads and explicit source membership for each native observation cut.
#[cfg(feature = "rendering")]
use crate::remote::{NativeProviderError, ProviderSet};
use phoxal::communication::{
    session::PortKind,
    simulation::{Observation, ProductDisposition, ProductMembership},
};
#[cfg(feature = "rendering")]
use crate::mujoco::Model;
use crate::mujoco::StateSnapshot;
#[cfg(feature = "rendering")]
use prost::{Message, Name};
use sha2::{Digest, Sha256};

pub fn packet(
    producer: &str,
    port: &str,
    state: &StateSnapshot,
    quantum_ns: u64,
    payload: Vec<u8>,
) -> Result<Observation, String> {
    let capture_time_ns = state
        .boundary()
        .checked_mul(quantum_ns)
        .ok_or_else(|| "capture time overflow".to_owned())?;
    let sequence = state
        .boundary()
        .checked_add(1)
        .ok_or_else(|| "observation sequence overflow".to_owned())?;
    Ok(Observation {
        membership: Some(ProductMembership {
            producer: producer.to_owned(),
            port: port.to_owned(),
            producer_incarnation: Vec::new(),
            sequence,
            capture_boundary: state.boundary(),
            capture_time_ns,
            disposition: ProductDisposition::Present as i32,
            item_count: 1,
            encoded_bytes: payload.len() as u64,
            payload_digest: Sha256::digest(&payload).to_vec(),
        }),
        payload,
    })
}

/// Encode one model-backed joint as the shared robotics encoder sample.
#[cfg(feature = "rendering")]
pub fn encode_encoder_observation(
    providers: &ProviderSet,
    service_instance: &str,
    port: &str,
    joint_id: &str,
    model: &Model,
    state: &StateSnapshot,
    quantum_ns: u64,
) -> Result<Observation, NativeProviderError> {
    let requirement = providers.get(service_instance, port).ok_or_else(|| {
        NativeProviderError::InvalidPayload(format!(
            "no immutable provider requirement for {service_instance}/{port}"
        ))
    })?;
    let kind = PortKind::try_from(requirement.kind)
        .map_err(|_| NativeProviderError::InvalidPayload("provider kind is unknown".to_owned()))?;
    if kind != PortKind::Sample
        || requirement.payload_fqn != phoxal::robotics::EncoderSample::full_name()
    {
        return Err(NativeProviderError::InvalidPayload(
            "encoder observation does not match the generated robotics sample contract".to_owned(),
        ));
    }
    let handle = model.joint(joint_id)?.ok_or_else(|| {
        NativeProviderError::InvalidPayload(format!("model has no joint named {joint_id}"))
    })?;
    let info = model.joint_info(handle)?;
    let sample = phoxal::robotics::EncoderSample {
        position_rad: Some(state.qpos().get(info.qpos_offset).copied().ok_or_else(|| {
            NativeProviderError::InvalidPayload(format!(
                "joint {joint_id} qpos offset is outside the state"
            ))
        })?),
        velocity_radps: Some(state.qvel().get(info.dof_offset).copied().ok_or_else(|| {
            NativeProviderError::InvalidPayload(format!(
                "joint {joint_id} qvel offset is outside the state"
            ))
        })?),
    };
    sample
        .validate()
        .map_err(|error| NativeProviderError::InvalidPayload(error.to_string()))?;
    packet(
        service_instance,
        port,
        state,
        quantum_ns,
        sample.encode_to_vec(),
    )
}

pub fn not_due(
    producer: &str,
    port: &str,
    state: &StateSnapshot,
    quantum_ns: u64,
) -> Result<Observation, String> {
    let mut observation = packet(producer, port, state, quantum_ns, Vec::new())?;
    let member = observation
        .membership
        .as_mut()
        .ok_or_else(|| "observation membership absent".to_owned())?;
    member.disposition = ProductDisposition::NotDue as i32;
    member.item_count = 0;
    Ok(observation)
}
