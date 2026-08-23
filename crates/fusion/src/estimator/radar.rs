use std::collections::BTreeMap;

use anyhow::Result;
use fusion_schema::messages::RadarScan;
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
    scan: &RadarScan,
) -> Result<()> {
    require_landmarks(landmarks)?;
    let range_stddev_m = config
        .radar_range_stddev_m
        .ok_or_else(|| anyhow::anyhow!("radar range tuning is missing"))?;
    let bearing_stddev_rad = config
        .radar_bearing_stddev_rad
        .ok_or_else(|| anyhow::anyhow!("radar bearing tuning is missing"))?;
    let radial_velocity_stddev_mps = config
        .radar_radial_velocity_stddev_mps
        .ok_or_else(|| anyhow::anyhow!("radar radial-velocity tuning is missing"))?;

    for detection in &scan.detections {
        let landmark_position_world_m = landmark_position(landmarks, &detection.landmark_id)?;
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
                residual: detection.range_m - geometry.range_m,
                measurement_jacobian_h: range_jacobian_h,
                measurement_variance_r: range_stddev_m.powi(2),
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
                residual: math::wrap_angle(detection.azimuth_rad - geometry.bearing_body_rad),
                measurement_jacobian_h: bearing_jacobian_h,
                measurement_variance_r: bearing_stddev_rad.powi(2),
            },
        );

        let predicted_radial_velocity_mps =
            -state.forward_speed_mps * geometry.bearing_body_rad.cos();
        let mut radial_velocity_jacobian_h = StateCorrection::zeros();
        radial_velocity_jacobian_h[StateIndex::PositionWorldX.index()] =
            state.forward_speed_mps * geometry.bearing_body_rad.sin() * dy_world_m
                / geometry.range_squared_m2;
        radial_velocity_jacobian_h[StateIndex::PositionWorldY.index()] =
            -state.forward_speed_mps * geometry.bearing_body_rad.sin() * dx_world_m
                / geometry.range_squared_m2;
        radial_velocity_jacobian_h[StateIndex::YawWorldFromBody.index()] =
            -state.forward_speed_mps * geometry.bearing_body_rad.sin();
        radial_velocity_jacobian_h[StateIndex::ForwardSpeed.index()] =
            -geometry.bearing_body_rad.cos();
        apply_scalar_update(
            state,
            covariance_p,
            ScalarObservation {
                residual: detection.radial_velocity_mps - predicted_radial_velocity_mps,
                measurement_jacobian_h: radial_velocity_jacobian_h,
                measurement_variance_r: radial_velocity_stddev_mps.powi(2),
            },
        );
    }

    Ok(())
}
