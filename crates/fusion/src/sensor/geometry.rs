use crate::{math::wrap_angle, scenario::LandmarkConfig, truth::Sample};

#[derive(Debug, Clone, Copy)]
pub(super) struct LandmarkGeometry {
    pub(super) range_m: f64,
    pub(super) azimuth_body_rad: f64,
}

pub(super) fn landmark_geometry(
    platform: Sample,
    landmark: &LandmarkConfig,
) -> Option<LandmarkGeometry> {
    let dx_world_m = landmark.x_m - platform.x_world_m;
    let dy_world_m = landmark.y_m - platform.y_world_m;
    let range_m = dx_world_m.hypot(dy_world_m);
    if range_m <= 1.0e-9 {
        return None;
    }

    Some(LandmarkGeometry {
        range_m,
        azimuth_body_rad: wrap_angle(
            dy_world_m.atan2(dx_world_m) - platform.yaw_world_from_body_rad,
        ),
    })
}

pub(super) fn is_visible(
    geometry: LandmarkGeometry,
    horizontal_fov_rad: f64,
    max_range_m: f64,
) -> bool {
    geometry.range_m <= max_range_m && geometry.azimuth_body_rad.abs() <= 0.5 * horizontal_fov_rad
}

#[cfg(test)]
mod tests {
    use super::*;

    fn platform_at_origin() -> Sample {
        Sample {
            time_s: 0.0,
            x_world_m: 0.0,
            y_world_m: 0.0,
            yaw_world_from_body_rad: 0.0,
            speed_mps: 0.0,
            path_distance_m: 0.0,
            longitudinal_acceleration_mps2: 0.0,
            yaw_rate_radps: 0.0,
        }
    }

    #[test]
    fn landmark_ahead_has_body_forward_bearing() {
        let landmark = LandmarkConfig {
            id: "ahead".to_owned(),
            x_m: 3.0,
            y_m: 0.0,
            z_m: 0.0,
        };
        let geometry = landmark_geometry(platform_at_origin(), &landmark).unwrap();

        assert_eq!(geometry.range_m, 3.0);
        assert_eq!(geometry.azimuth_body_rad, 0.0);
    }

    #[test]
    fn field_of_view_uses_body_frame_azimuth() {
        let landmark = LandmarkConfig {
            id: "left".to_owned(),
            x_m: 0.0,
            y_m: 2.0,
            z_m: 0.0,
        };
        let geometry = landmark_geometry(platform_at_origin(), &landmark).unwrap();

        assert!(is_visible(geometry, std::f64::consts::PI, 3.0));
        assert!(!is_visible(geometry, 1.0, 3.0));
    }
}
