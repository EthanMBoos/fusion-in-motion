use std::collections::BTreeMap;

use anyhow::Result;
use fusion_schema::messages::LidarScan;
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
