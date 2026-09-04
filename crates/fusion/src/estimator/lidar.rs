use std::collections::BTreeMap;

use anyhow::{Result, ensure};
use fusion_schema::messages::{LidarScan, StampReference};
use nalgebra::Vector2;

use crate::{math, scenario::EstimatorConfig};

use super::{
    observation::{
        LandmarkGeometry, ScalarObservation, apply_scalar_update, landmark_position,
        require_landmarks,
    },
    state::{PlanarState, StateCorrection, StateCovariance, StateIndex},
};

pub(super) fn update(
    state: &mut PlanarState,
    covariance_p: &mut StateCovariance,
    landmarks: &BTreeMap<String, Vector2<f64>>,
    config: &EstimatorConfig,
    scan: &LidarScan,
) -> Result<()> {
    require_landmarks(landmarks)?;

    for hit in &scan.returns {
        let landmark_position_world_m = landmark_position(landmarks, &hit.landmark_id)?;
        let Some(geometry) = LandmarkGeometry::predict(state, landmark_position_world_m) else {
            continue;
        };
        let dx_world_m = geometry.displacement_world_m.x;
        let dy_world_m = geometry.displacement_world_m.y;

        let mut range_jacobian_h = StateCorrection::zeros();
        range_jacobian_h[StateIndex::PositionWorldX.index()] = -dx_world_m / geometry.range_m;
        range_jacobian_h[StateIndex::PositionWorldY.index()] = -dy_world_m / geometry.range_m;
        apply_scalar_update(
            state,
            covariance_p,
            ScalarObservation {
                residual: hit.range_m - geometry.range_m,
                measurement_jacobian_h: range_jacobian_h,
                measurement_variance_r: config.lidar_range_stddev_m.powi(2),
            },
        );

        let mut bearing_jacobian_h = StateCorrection::zeros();
        bearing_jacobian_h[StateIndex::PositionWorldX.index()] =
            dy_world_m / geometry.range_squared_m2;
        bearing_jacobian_h[StateIndex::PositionWorldY.index()] =
            -dx_world_m / geometry.range_squared_m2;
        bearing_jacobian_h[StateIndex::YawWorldFromBody.index()] = -1.0;
        apply_scalar_update(
            state,
            covariance_p,
            ScalarObservation {
                residual: math::wrap_angle(hit.azimuth_rad - geometry.bearing_body_rad),
                measurement_jacobian_h: bearing_jacobian_h,
                measurement_variance_r: config.lidar_bearing_stddev_rad.powi(2),
            },
        );
    }

    Ok(())
}

pub(super) fn deskew(
    scan: &LidarScan,
    state_at: impl Fn(i64) -> Option<PlanarState>,
) -> Result<LidarScan> {
    let header = scan
        .header
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("lidar scan is missing its header"))?;
    ensure!(
        header.stamp_reference == StampReference::End as i32,
        "lidar deskew requires the scan timestamp to reference acquisition end"
    );
    ensure!(
        header.acquisition_duration_ns >= 0,
        "lidar acquisition duration must be nonnegative"
    );
    if header.acquisition_duration_ns == 0 {
        return Ok(scan.clone());
    }

    let scan_start_ns = header.reported_stamp_ns - header.acquisition_duration_ns;
    let end_state = state_at(header.reported_stamp_ns).ok_or_else(|| {
        anyhow::anyhow!(
            "state history does not cover lidar scan end {} ns",
            header.reported_stamp_ns
        )
    })?;
    let mut deskewed = scan.clone();

    // TODO: Rigorous long-scan consistency studies should propagate pose-history
    // uncertainty through this transform. This deskew treats those poses as exact.
    for hit in &mut deskewed.returns {
        ensure!(
            (0..=header.acquisition_duration_ns).contains(&hit.acquisition_offset_ns),
            "lidar return acquisition offset {} ns is outside scan duration {} ns",
            hit.acquisition_offset_ns,
            header.acquisition_duration_ns
        );
        let acquisition_ns = scan_start_ns + hit.acquisition_offset_ns;
        let acquisition_state = state_at(acquisition_ns).ok_or_else(|| {
            anyhow::anyhow!("state history does not cover lidar return time {acquisition_ns} ns")
        })?;

        let point_body_at_acquisition = Vector2::new(
            hit.range_m * hit.azimuth_rad.cos(),
            hit.range_m * hit.azimuth_rad.sin(),
        );
        let point_world = acquisition_state.position_world_m
            + rotate(
                point_body_at_acquisition,
                acquisition_state.yaw_world_from_body_rad,
            );
        let point_body_at_end = rotate(
            point_world - end_state.position_world_m,
            -end_state.yaw_world_from_body_rad,
        );
        hit.range_m = point_body_at_end.norm();
        hit.azimuth_rad = math::wrap_angle(point_body_at_end.y.atan2(point_body_at_end.x));
    }
    Ok(deskewed)
}

fn rotate(vector: Vector2<f64>, angle_rad: f64) -> Vector2<f64> {
    let (sin, cos) = angle_rad.sin_cos();
    Vector2::new(
        cos * vector.x - sin * vector.y,
        sin * vector.x + cos * vector.y,
    )
}

#[cfg(test)]
mod tests {
    use fusion_schema::messages::{LidarReturn, RecordHeader};

    use super::*;

    #[test]
    fn deskew_moves_an_early_return_into_the_scan_end_frame() {
        let scan = LidarScan {
            header: Some(RecordHeader {
                reported_stamp_ns: 1_000_000_000,
                stamp_reference: StampReference::End as i32,
                acquisition_duration_ns: 1_000_000_000,
                ..RecordHeader::default()
            }),
            returns: vec![LidarReturn {
                landmark_id: "landmark".to_owned(),
                range_m: 10.0,
                azimuth_rad: 0.0,
                acquisition_offset_ns: 0,
            }],
            association_mode: "ORACLE".to_owned(),
        };
        let start = PlanarState::default();
        let mut end = start;
        end.position_world_m.x = 1.0;

        let result = deskew(
            &scan,
            |time_ns| {
                if time_ns == 0 { Some(start) } else { Some(end) }
            },
        )
        .unwrap();

        assert!((result.returns[0].range_m - 9.0).abs() < 1.0e-12);
        assert!(result.returns[0].azimuth_rad.abs() < 1.0e-12);
    }
}
