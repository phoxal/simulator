use crate::remote::NativeProviderError;

/// Qualified maximum Euclidean distance from the scene reference, in meters.
pub(super) const MAX_REFERENCE_DISTANCE_M: f64 = 100_000.0;

pub(super) const WGS84_SEMI_MAJOR_AXIS_METERS: f64 = 6_378_137.0;

pub(super) const WGS84_FIRST_ECCENTRICITY_SQUARED: f64 = 6.694_379_990_141_316_5e-3;

/// Explicit geodetic origin used by the ZED-F9P reference projection.
///
/// The native site position is interpreted in a local ENU frame: X is east,
/// Y is north, and Z is up after applying `yaw_rad`.  The yaw rotates native
/// model XY coordinates into ENU coordinates, with zero meaning native +X is
/// east and native +Y is north.  `origin_m` is the local Cartesian point whose
/// coordinates correspond to the supplied WGS84 origin.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Georeference {
    /// WGS84 latitude at the local origin.
    pub latitude_deg: f64,
    /// WGS84 longitude at the local origin.
    pub longitude_deg: f64,
    /// WGS84 ellipsoidal altitude at the local origin.
    pub altitude_m: f64,
    /// Local ENU coordinates of the geodetic origin.
    pub origin_m: [f64; 3],
    /// Counter-clockwise rotation from native model XY to ENU XY, in radians.
    pub yaw_rad: f64,
}

impl Georeference {
    /// Creates a validated WGS84 local tangent-plane reference with an
    /// explicit native-to-ENU yaw alignment.
    pub fn with_yaw(
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_m: f64,
        origin_m: [f64; 3],
        yaw_rad: f64,
    ) -> Result<Self, NativeProviderError> {
        let georeference = Self {
            latitude_deg,
            longitude_deg,
            altitude_m,
            origin_m,
            yaw_rad,
        };
        georeference.validate()?;
        Ok(georeference)
    }

    pub(super) fn validate(self) -> Result<(), NativeProviderError> {
        let Self {
            latitude_deg,
            longitude_deg,
            altitude_m,
            origin_m,
            yaw_rad,
        } = self;
        if !latitude_deg.is_finite()
            || !(-90.0..=90.0).contains(&latitude_deg)
            || !longitude_deg.is_finite()
            || !(-180.0..=180.0).contains(&longitude_deg)
            || !altitude_m.is_finite()
            || !yaw_rad.is_finite()
            || origin_m.iter().any(|value| !value.is_finite())
        {
            return Err(NativeProviderError::InvalidPayload(
                "ZED-F9P georeference must contain finite WGS84 and ENU values".to_owned(),
            ));
        }
        if latitude_deg.abs() >= 90.0 {
            return Err(NativeProviderError::InvalidPayload(
                "ZED-F9P georeference latitude must leave a non-zero longitude scale".to_owned(),
            ));
        }
        Ok(())
    }

    pub(super) fn project(self, position_m: [f64; 3]) -> Result<[f64; 3], NativeProviderError> {
        if position_m.iter().any(|value| !value.is_finite()) {
            return Err(NativeProviderError::InvalidPayload(
                "ZED-F9P site position is non-finite".to_owned(),
            ));
        }
        let native_x = position_m[0] - self.origin_m[0];
        let native_y = position_m[1] - self.origin_m[1];
        let native_z = position_m[2] - self.origin_m[2];
        if native_x.hypot(native_y).hypot(native_z) > MAX_REFERENCE_DISTANCE_M {
            return Err(NativeProviderError::InvalidPayload(
                "GNSS antenna exceeds the qualified 100 km scene-reference distance".into(),
            ));
        }
        let yaw_sin = self.yaw_rad.sin();
        let yaw_cos = self.yaw_rad.cos();
        let east = yaw_cos * native_x - yaw_sin * native_y;
        let north = yaw_sin * native_x + yaw_cos * native_y;
        let up = position_m[2] - self.origin_m[2];
        let origin_latitude = self.latitude_deg.to_radians();
        let origin_longitude = self.longitude_deg.to_radians();
        let origin_ecef = geodetic_to_ecef(origin_latitude, origin_longitude, self.altitude_m);
        let sin_latitude = origin_latitude.sin();
        let cos_latitude = origin_latitude.cos();
        let sin_longitude = origin_longitude.sin();
        let cos_longitude = origin_longitude.cos();
        let east_axis = [-sin_longitude, cos_longitude, 0.0];
        let north_axis = [
            -sin_latitude * cos_longitude,
            -sin_latitude * sin_longitude,
            cos_latitude,
        ];
        let up_axis = [
            cos_latitude * cos_longitude,
            cos_latitude * sin_longitude,
            sin_latitude,
        ];
        let ecef = [
            origin_ecef[0] + east * east_axis[0] + north * north_axis[0] + up * up_axis[0],
            origin_ecef[1] + east * east_axis[1] + north * north_axis[1] + up * up_axis[1],
            origin_ecef[2] + east * east_axis[2] + north * north_axis[2] + up * up_axis[2],
        ];
        let [latitude, longitude, altitude_m] = ecef_to_geodetic(ecef)?;
        let latitude_deg = latitude.to_degrees();
        let longitude_deg = longitude.to_degrees();
        if !latitude_deg.is_finite()
            || !longitude_deg.is_finite()
            || !altitude_m.is_finite()
            || !(-90.0..=90.0).contains(&latitude_deg)
            || !(-180.0..=180.0).contains(&longitude_deg)
        {
            return Err(NativeProviderError::InvalidPayload(
                "ZED-F9P projected WGS84 position is outside finite bounds".to_owned(),
            ));
        }
        Ok([latitude_deg, longitude_deg, altitude_m])
    }
}

pub(super) fn geodetic_to_ecef(latitude_rad: f64, longitude_rad: f64, altitude_m: f64) -> [f64; 3] {
    let sin_latitude = latitude_rad.sin();
    let cos_latitude = latitude_rad.cos();
    let radius = WGS84_SEMI_MAJOR_AXIS_METERS
        / (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude).sqrt();
    [
        (radius + altitude_m) * cos_latitude * longitude_rad.cos(),
        (radius + altitude_m) * cos_latitude * longitude_rad.sin(),
        (radius * (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED) + altitude_m) * sin_latitude,
    ]
}

pub(super) fn ecef_to_geodetic(ecef: [f64; 3]) -> Result<[f64; 3], NativeProviderError> {
    if ecef.iter().any(|value| !value.is_finite()) {
        return Err(NativeProviderError::InvalidPayload(
            "ZED-F9P ECEF position is non-finite".to_owned(),
        ));
    }
    let [x, y, z] = ecef;
    let horizontal = x.hypot(y);
    if horizontal < f64::EPSILON && z.abs() < f64::EPSILON {
        return Err(NativeProviderError::InvalidPayload(
            "ZED-F9P ECEF position is at the Earth's center".to_owned(),
        ));
    }
    let longitude = y.atan2(x);
    let mut latitude = z.atan2(horizontal * (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED));
    let mut altitude;
    for _ in 0..16 {
        let sin_latitude = latitude.sin();
        let cos_latitude = latitude.cos();
        let radius = WGS84_SEMI_MAJOR_AXIS_METERS
            / (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude).sqrt();
        altitude = if horizontal > f64::EPSILON {
            horizontal / cos_latitude - radius
        } else {
            z.abs() - radius * (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED)
        };
        let next = if horizontal > f64::EPSILON {
            z.atan2(
                horizontal
                    * (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED * radius / (radius + altitude)),
            )
        } else {
            std::f64::consts::FRAC_PI_2.copysign(z)
        };
        if (next - latitude).abs() <= 1.0e-14 {
            latitude = next;
            break;
        }
        latitude = next;
    }
    let sin_latitude = latitude.sin();
    let cos_latitude = latitude.cos();
    let radius = WGS84_SEMI_MAJOR_AXIS_METERS
        / (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude).sqrt();
    altitude = if horizontal > f64::EPSILON {
        horizontal / cos_latitude - radius
    } else {
        z.abs() - radius * (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED)
    };
    Ok([latitude, longitude, altitude])
}
