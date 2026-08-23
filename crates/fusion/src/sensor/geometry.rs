use crate::{
    math::wrap_angle,
    scenario::{ObjectConfig, SensorMountConfig},
    truth::Sample,
};

#[derive(Debug, Clone, Copy)]
pub(super) struct ObjectGeometry {
    pub(super) range_m: f64,
    pub(super) azimuth_sensor_rad: f64,
    pub(super) elevation_sensor_rad: f64,
}

pub(super) fn object_geometry(
    platform: Sample,
    object: &ObjectConfig,
    mount: &SensorMountConfig,
) -> Option<ObjectGeometry> {
    let body_yaw = platform.yaw_world_from_body_rad;
    let sensor_x_world = platform.x_world_m + body_yaw.cos() * mount.position_m.x
        - body_yaw.sin() * mount.position_m.y;
    let sensor_y_world = platform.y_world_m
        + body_yaw.sin() * mount.position_m.x
        + body_yaw.cos() * mount.position_m.y;
    let object_x = object.initial_position_m.x + object.velocity_world_mps.x * platform.time_s;
    let object_y = object.initial_position_m.y + object.velocity_world_mps.y * platform.time_s;
    let object_z = object.initial_position_m.z + object.velocity_world_mps.z * platform.time_s;
    let dx = object_x - sensor_x_world;
    let dy = object_y - sensor_y_world;
    let dz = object_z - mount.position_m.z;
    let horizontal_range = dx.hypot(dy);
    let range_m = horizontal_range.hypot(dz);
    if range_m <= 1.0e-9 {
        return None;
    }

    Some(ObjectGeometry {
        range_m,
        azimuth_sensor_rad: wrap_angle(dy.atan2(dx) - body_yaw - mount.yaw_rad),
        elevation_sensor_rad: dz.atan2(horizontal_range),
    })
}

pub(super) fn is_visible(
    geometry: ObjectGeometry,
    horizontal_fov_rad: f64,
    vertical_fov_rad: f64,
    max_range_m: f64,
) -> bool {
    geometry.range_m <= max_range_m
        && geometry.azimuth_sensor_rad.abs() <= 0.5 * horizontal_fov_rad
        && geometry.elevation_sensor_rad.abs() <= 0.5 * vertical_fov_rad
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenario::Vec3Config;

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

    fn mount() -> SensorMountConfig {
        SensorMountConfig {
            frame: "sensor".to_owned(),
            position_m: Vec3Config::default(),
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rad: 0.0,
        }
    }

    #[test]
    fn object_ahead_has_forward_bearing() {
        let object = ObjectConfig {
            id: "ahead".to_owned(),
            association_key: "track-a".to_owned(),
            initial_position_m: Vec3Config {
                x: 3.0,
                y: 0.0,
                z: 0.0,
            },
            velocity_world_mps: Vec3Config::default(),
        };
        let geometry = object_geometry(platform_at_origin(), &object, &mount()).unwrap();
        assert!((geometry.range_m - 3.0).abs() < 1.0e-12);
        assert!(geometry.azimuth_sensor_rad.abs() < 1.0e-12);
    }
}
