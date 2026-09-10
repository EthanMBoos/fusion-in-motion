use fusion_schema::messages::{Pose2, Vec2};

pub fn pose2(x: f64, y: f64, yaw_rad: f64) -> Pose2 {
    Pose2 {
        position: Some(Vec2 { x, y }),
        yaw_rad: wrap_angle(yaw_rad),
    }
}

pub fn wrap_angle(angle: f64) -> f64 {
    let wrapped =
        (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
    if wrapped == -std::f64::consts::PI && angle > 0.0 {
        std::f64::consts::PI
    } else {
        wrapped
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn angle_wrap_handles_branch_cut() {
        assert!((wrap_angle(3.0 * std::f64::consts::PI) - std::f64::consts::PI).abs() < 1.0e-12);
        assert!((wrap_angle(-3.0 * std::f64::consts::PI) + std::f64::consts::PI).abs() < 1.0e-12);
    }
}
