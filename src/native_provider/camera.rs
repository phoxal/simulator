#[cfg(feature = "rendering")]
use super::observations::encode_observation;
use crate::remote::NativeProviderError;
use crate::remote::ProviderSet;
use phoxal::communication::simulation::Observation;
use phoxal_component_oak_d_lite as oak_contract;
use crate::mujoco::CameraBinding;
use crate::mujoco::StateSnapshot;
use crate::mujoco::Workspace;
use phoxal::port::PortSignature;
#[cfg(feature = "rendering")]
use prost::Message;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum CameraEncoding {
    Mono8,
    Rgb8,
}

#[cfg(feature = "rendering")]
#[allow(clippy::too_many_arguments)]
pub(super) fn encode_camera_observation(
    providers: &ProviderSet,
    service_instance: &str,
    signature: PortSignature,
    binding: &CameraBinding,
    render_workspace: Option<&mut Workspace>,
    state: &StateSnapshot,
    quantum_ns: u64,
    encoding: CameraEncoding,
) -> Result<Observation, NativeProviderError> {
    let workspace = render_workspace.ok_or_else(|| {
        NativeProviderError::Unsupported(format!(
            "provider {service_instance}/{} has no native renderer workspace",
            signature.name
        ))
    })?;
    let rendered = workspace
        .render_camera(binding.native)
        .map_err(|error| NativeProviderError::Unsupported(error.to_string()))?;
    let [width, height] = rendered.resolution();
    let data = match encoding {
        CameraEncoding::Rgb8 => rendered.rgb().to_vec(),
        CameraEncoding::Mono8 => rgb_to_mono8(rendered.rgb())?,
    };
    let expected_len = match encoding {
        CameraEncoding::Rgb8 => width
            .checked_mul(height)
            .and_then(|pixels| pixels.checked_mul(3)),
        CameraEncoding::Mono8 => width.checked_mul(height),
    }
    .ok_or_else(|| {
        NativeProviderError::InvalidPayload(format!(
            "provider {service_instance}/{} camera resolution overflows payload size",
            signature.name
        ))
    })?;
    if data.len() != expected_len {
        return Err(NativeProviderError::InvalidPayload(format!(
            "provider {service_instance}/{} renderer returned {} bytes, expected {expected_len}",
            signature.name,
            data.len()
        )));
    }
    let encoding = match encoding {
        CameraEncoding::Mono8 => oak_contract::ImageEncoding::Mono8 as i32,
        CameraEncoding::Rgb8 => oak_contract::ImageEncoding::Rgb8 as i32,
    };
    let width_px = u32::try_from(width).map_err(|_| {
        NativeProviderError::InvalidPayload(format!(
            "provider {service_instance}/{} camera width does not fit u32",
            signature.name
        ))
    })?;
    let height_px = u32::try_from(height).map_err(|_| {
        NativeProviderError::InvalidPayload(format!(
            "provider {service_instance}/{} camera height does not fit u32",
            signature.name
        ))
    })?;
    let frame = oak_contract::CameraFrame {
        width_px,
        height_px,
        encoding,
        data,
    };
    encode_observation(
        providers,
        service_instance,
        signature,
        state,
        quantum_ns,
        frame.encode_to_vec(),
    )
}

#[cfg(not(feature = "rendering"))]
#[allow(clippy::too_many_arguments)]
pub(super) fn encode_camera_observation(
    _providers: &ProviderSet,
    service_instance: &str,
    signature: PortSignature,
    _binding: &CameraBinding,
    _render_workspace: Option<&mut Workspace>,
    _state: &StateSnapshot,
    _quantum_ns: u64,
    _encoding: CameraEncoding,
) -> Result<Observation, NativeProviderError> {
    Err(NativeProviderError::Unsupported(format!(
        "provider {service_instance}/{} camera observations require the `rendering` feature; \
         rebuild with `--features native,rendering` to exercise this provider",
        signature.name
    )))
}

#[cfg(feature = "rendering")]
pub(super) fn encode_depth_observation(
    providers: &ProviderSet,
    service_instance: &str,
    binding: &CameraBinding,
    render_workspace: Option<&mut Workspace>,
    state: &StateSnapshot,
    quantum_ns: u64,
    range_m: [f64; 2],
) -> Result<Observation, NativeProviderError> {
    let workspace = render_workspace.ok_or_else(|| {
        NativeProviderError::Unsupported(format!(
            "provider {service_instance}/{} has no native renderer workspace",
            oak_contract::ports::DEPTH.name()
        ))
    })?;
    let rendered = workspace
        .render_camera(binding.native)
        .map_err(|error| NativeProviderError::Unsupported(error.to_string()))?;
    let [width, height] = rendered.resolution();
    let depth_mm = rendered
        .depth_m()
        .iter()
        .copied()
        .map(|depth| depth_millimetres(depth, range_m))
        .collect::<Result<Vec<_>, _>>()?;
    let expected_len = width.checked_mul(height).ok_or_else(|| {
        NativeProviderError::InvalidPayload(format!(
            "provider {service_instance}/{} camera resolution overflows depth payload size",
            oak_contract::ports::DEPTH.name()
        ))
    })?;
    if depth_mm.len() != expected_len {
        return Err(NativeProviderError::InvalidPayload(format!(
            "provider {service_instance}/{} renderer returned {} depth values, expected {expected_len}",
            oak_contract::ports::DEPTH.name(),
            depth_mm.len()
        )));
    }
    let frame = oak_contract::DepthFrame {
        width_px: u32::try_from(width).map_err(|_| {
            NativeProviderError::InvalidPayload(format!(
                "provider {service_instance}/{} camera width does not fit u32",
                oak_contract::ports::DEPTH.name()
            ))
        })?,
        height_px: u32::try_from(height).map_err(|_| {
            NativeProviderError::InvalidPayload(format!(
                "provider {service_instance}/{} camera height does not fit u32",
                oak_contract::ports::DEPTH.name()
            ))
        })?,
        depth_mm,
    };
    encode_observation(
        providers,
        service_instance,
        oak_contract::ports::DEPTH.signature(),
        state,
        quantum_ns,
        frame.encode_to_vec(),
    )
}

#[cfg(not(feature = "rendering"))]
pub(super) fn encode_depth_observation(
    _providers: &ProviderSet,
    service_instance: &str,
    _binding: &CameraBinding,
    _render_workspace: Option<&mut Workspace>,
    _state: &StateSnapshot,
    _quantum_ns: u64,
    _range_m: [f64; 2],
) -> Result<Observation, NativeProviderError> {
    Err(NativeProviderError::Unsupported(format!(
        "provider {service_instance}/{} {} observations require the `rendering` feature; \
         rebuild with `--features native,rendering` to exercise this provider",
        service_instance,
        oak_contract::ports::DEPTH.name()
    )))
}

pub(super) fn rgb_to_mono8(rgb: &[u8]) -> Result<Vec<u8>, NativeProviderError> {
    if !rgb.len().is_multiple_of(3) {
        return Err(NativeProviderError::InvalidPayload(
            "native RGB renderer returned a partial pixel".to_owned(),
        ));
    }
    let mut mono = Vec::with_capacity(rgb.len() / 3);
    for pixel in rgb.chunks_exact(3) {
        let value = 77_u16
            .saturating_mul(pixel[0] as u16)
            .saturating_add(150_u16.saturating_mul(pixel[1] as u16))
            .saturating_add(29_u16.saturating_mul(pixel[2] as u16));
        mono.push(((value + 128) / 256) as u8);
    }
    Ok(mono)
}

pub(super) fn depth_millimetres(
    depth_m: f32,
    range_m: [f64; 2],
) -> Result<u32, NativeProviderError> {
    if !depth_m.is_finite() || f64::from(depth_m) < range_m[0] || f64::from(depth_m) > range_m[1] {
        return Ok(0);
    }
    let millimetres = f64::from(depth_m) * 1_000.0;
    if !millimetres.is_finite() || millimetres >= f64::from(u32::MAX) {
        return Err(NativeProviderError::InvalidPayload(format!(
            "native geometric depth {depth_m}m cannot be represented as millimetres"
        )));
    }
    Ok(millimetres.round() as u32)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn depth_encoding_applies_the_authored_range_and_preserves_metric_units() {
        assert_eq!(depth_millimetres(2.0, [0.4, 12.0]).unwrap(), 2000);
        for depth in [0.0, 0.3, 12.1, f32::INFINITY, f32::NAN] {
            assert_eq!(depth_millimetres(depth, [0.4, 12.0]).unwrap(), 0);
        }
    }
}
