use crate::mujoco::Model;
use crate::native_provider::Georeference;

pub(super) const GEOREFERENCE_NUMERIC: &str = "phoxal_georeference";

pub(super) const GEOREFERENCE_AXES: &str = "phoxal_georeference_axes";

pub(super) const GEOREFERENCE_DATUM: &str = "phoxal_georeference_datum";

pub(super) fn georeference(model: &Model, instance: &str) -> Result<Georeference, String> {
    let values = model
        .custom_numeric(GEOREFERENCE_NUMERIC)
        .map_err(|error| format!("GNSS georeference metadata: {error}"))?
        .ok_or_else(|| {
            format!(
                "GNSS component {instance} requires scene numeric custom metadata {GEOREFERENCE_NUMERIC:?}"
            )
        })?;
    if values.len() != 7 {
        return Err(format!(
            "scene custom numeric {GEOREFERENCE_NUMERIC:?} must contain exactly seven values"
        ));
    }
    if values.iter().any(|value| !value.is_finite()) {
        return Err(format!(
            "scene custom numeric {GEOREFERENCE_NUMERIC:?} contains a non-finite value"
        ));
    }
    let axes = model
        .custom_text(GEOREFERENCE_AXES)
        .map_err(|error| format!("GNSS georeference metadata: {error}"))?
        .ok_or_else(|| {
            format!(
                "GNSS component {instance} requires scene text custom metadata {GEOREFERENCE_AXES:?}"
            )
        })?;
    if axes != "ENU" {
        return Err(format!(
            "scene custom georeference axes must be ENU, got {axes:?}"
        ));
    }
    let datum = model
        .custom_text(GEOREFERENCE_DATUM)
        .map_err(|error| format!("GNSS georeference metadata: {error}"))?
        .ok_or_else(|| {
            format!(
                "GNSS component {instance} requires scene text custom metadata {GEOREFERENCE_DATUM:?}"
            )
        })?;
    if datum != "WGS84_ELLIPSOIDAL" {
        return Err(format!(
            "scene custom georeference datum must be WGS84_ELLIPSOIDAL, got {datum:?}"
        ));
    }
    Georeference::with_yaw(
        values[0],
        values[1],
        values[2],
        [values[3], values[4], values[5]],
        values[6],
    )
    .map_err(|error| error.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mujoco::Model;
    #[test]
    fn zed_georeference_reads_scene_owned_mjcf_metadata() {
        let model = Model::from_xml(
            r#"<mujoco model="scene">
                <custom>
                    <numeric name="phoxal_georeference" data="52 5 10 1 2 3 0.25"/>
                    <text name="phoxal_georeference_axes" data="ENU"/>
                    <text name="phoxal_georeference_datum" data="WGS84_ELLIPSOIDAL"/>
                </custom>
                <worldbody/>
            </mujoco>"#,
        )
        .expect("scene model");
        let georeference = georeference(&model, "gnss").expect("scene georeference");
        assert_eq!(georeference.latitude_deg, 52.0);
        assert_eq!(georeference.longitude_deg, 5.0);
        assert_eq!(georeference.altitude_m, 10.0);
        assert_eq!(georeference.origin_m, [1.0, 2.0, 3.0]);
        assert_eq!(georeference.yaw_rad, 0.25);
    }

    #[test]
    fn zed_georeference_rejects_missing_scene_metadata() {
        let model =
            Model::from_xml(r#"<mujoco model="scene"><worldbody/></mujoco>"#).expect("scene model");
        let error = georeference(&model, "gnss").expect_err("metadata is required");
        assert!(error.contains("phoxal_georeference"));
    }
}
