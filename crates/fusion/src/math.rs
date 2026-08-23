use fusion_schema::messages::{Pose, Quaternion, Vec3};
use nalgebra::{UnitQuaternion, Vector3};

pub const GRAVITY_WORLD_MPS2: Vector3<f64> = Vector3::new(0.0, 0.0, -9.806_65);

pub fn vec3(v: Vector3<f64>) -> Vec3 {
    Vec3 {
        x: v.x,
        y: v.y,
        z: v.z,
    }
}

pub fn yaw_pose(
    x_world_m: f64,
    y_world_m: f64,
    yaw_world_from_body_rad: f64,
    world_frame: &str,
    body_frame: &str,
) -> Pose {
    let q = UnitQuaternion::from_euler_angles(0.0, 0.0, yaw_world_from_body_rad);
    let q = q.quaternion();
    let sign = if q.w < 0.0 { -1.0 } else { 1.0 };
    Pose {
        position: Some(Vec3 {
            x: x_world_m,
            y: y_world_m,
            z: 0.0,
        }),
        orientation_xyzw: Some(Quaternion {
            x: sign * q.i,
            y: sign * q.j,
            z: sign * q.k,
            w: sign * q.w,
        }),
        parent_frame: world_frame.to_owned(),
        child_frame: body_frame.to_owned(),
    }
}

pub fn yaw_from_pose(pose: &Pose) -> f64 {
    let q = pose
        .orientation_xyzw
        .as_ref()
        .expect("validated pose quaternion");
    let unit = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(q.w, q.x, q.y, q.z));
    unit.euler_angles().2
}

pub fn wrap_angle(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(2.0 * std::f64::consts::PI) - std::f64::consts::PI
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quaternion_is_xyzw_and_canonical() {
        let pose = yaw_pose(0.0, 0.0, 1.2, "world", "body");
        let q = pose.orientation_xyzw.unwrap();
        assert!(q.w >= 0.0);
        assert!((q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn angle_wrap_handles_branch_cut() {
        assert!((wrap_angle(3.0 * std::f64::consts::PI) + std::f64::consts::PI).abs() < 1.0e-12);
    }
}
