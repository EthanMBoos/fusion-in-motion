use std::collections::BTreeMap;

use anyhow::Result;
use fusion_schema::messages::CameraFrame;
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
    frame: &CameraFrame,
) -> Result<()> {
    require_landmarks(landmarks)?;

    for feature in &frame.features {
        let landmark_position_world_m = landmark_position(landmarks, &feature.landmark_id)?;
        let Some(geometry) = LandmarkGeometry::predict(state, landmark_position_world_m) else {
            continue;
        };

        let dx_world_m = geometry.displacement_world_m.x;
        let dy_world_m = geometry.displacement_world_m.y;
        let mut measurement_jacobian_h = StateCorrection::zeros();
        measurement_jacobian_h[StateIndex::PositionWorldX.index()] =
            dy_world_m / geometry.range_squared_m2;
        measurement_jacobian_h[StateIndex::PositionWorldY.index()] =
            -dx_world_m / geometry.range_squared_m2;
        measurement_jacobian_h[StateIndex::YawWorldFromBody.index()] = -1.0;

        apply_scalar_update(
            state,
            covariance_p,
            ScalarObservation {
                residual: math::wrap_angle(feature.azimuth_rad - geometry.bearing_body_rad),
                measurement_jacobian_h,
                measurement_variance_r: config.camera_bearing_stddev_rad.powi(2),
            },
        );
    }

    Ok(())
}
