use crate::{math::wrap_angle, scenario::ObjectConfig, truth::Sample};

#[derive(Debug, Clone, Copy)]
pub(super) struct ObjectGeometry {
    pub(super) range_m: f64,
    pub(super) bearing_rad: f64,
}

pub(super) fn object_geometry(platform: Sample, object: &ObjectConfig) -> Option<ObjectGeometry> {
    let object_x = object.initial_position_m.x + object.velocity_world_mps.x * platform.time_s;
    let object_y = object.initial_position_m.y + object.velocity_world_mps.y * platform.time_s;
    let dx = object_x - platform.x_world_m;
    let dy = object_y - platform.y_world_m;
    let range_m = dx.hypot(dy);
    if range_m <= 1.0e-9 {
        return None;
    }
    Some(ObjectGeometry {
        range_m,
        bearing_rad: wrap_angle(dy.atan2(dx) - platform.yaw_world_from_body_rad),
    })
}

pub(super) fn is_visible(
    geometry: ObjectGeometry,
    horizontal_fov_rad: f64,
    max_range_m: f64,
) -> bool {
    geometry.range_m <= max_range_m && geometry.bearing_rad.abs() <= 0.5 * horizontal_fov_rad
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenario::Vec2Config;

    #[test]
    fn object_ahead_has_forward_bearing() {
        let platform = Sample {
            time_s: 0.0,
            x_world_m: 0.0,
            y_world_m: 0.0,
            yaw_world_from_body_rad: 0.0,
            speed_mps: 0.0,
            path_distance_m: 0.0,
            longitudinal_acceleration_mps2: 0.0,
            yaw_rate_radps: 0.0,
        };
        let object = ObjectConfig {
            id: "ahead".to_owned(),
            initial_position_m: Vec2Config { x: 3.0, y: 0.0 },
            velocity_world_mps: Vec2Config::default(),
        };
        let geometry = object_geometry(platform, &object).unwrap();
        assert!((geometry.range_m - 3.0).abs() < 1.0e-12);
        assert!(geometry.bearing_rad.abs() < 1.0e-12);
    }
}
