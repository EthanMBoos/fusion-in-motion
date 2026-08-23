use std::collections::BTreeMap;

use anyhow::{Result, ensure};
use nalgebra::Vector2;

use crate::math;

use super::state::{PlanarState, StateCorrection, StateCovariance};

pub(super) struct LandmarkGeometry {
    pub(super) displacement_world_m: Vector2<f64>,
    pub(super) range_m: f64,
    pub(super) bearing_body_rad: f64,
    pub(super) range_squared_m2: f64,
}

impl LandmarkGeometry {
    pub(super) fn predict(
        state: &PlanarState,
        landmark_position_world_m: Vector2<f64>,
    ) -> Option<Self> {
        let displacement_world_m = landmark_position_world_m - state.position_world_m;
        let range_squared_m2 = displacement_world_m.norm_squared();
        if range_squared_m2 <= 1.0e-12 {
            return None;
        }

        Some(Self {
            displacement_world_m,
            range_m: range_squared_m2.sqrt(),
            bearing_body_rad: math::wrap_angle(
                displacement_world_m.y.atan2(displacement_world_m.x)
                    - state.yaw_world_from_body_rad,
            ),
            range_squared_m2,
        })
    }
}

pub(super) struct ScalarObservation {
    pub(super) residual: f64,
    pub(super) measurement_jacobian_h: StateCorrection,
    pub(super) measurement_variance_r: f64,
}

pub(super) fn apply_scalar_update(
    state: &mut PlanarState,
    covariance_p: &mut StateCovariance,
    observation: ScalarObservation,
) {
    let ScalarObservation {
        residual,
        measurement_jacobian_h,
        measurement_variance_r,
    } = observation;

    let innovation_variance_s =
        (measurement_jacobian_h.transpose() * *covariance_p * measurement_jacobian_h)[0]
            + measurement_variance_r;
    if !innovation_variance_s.is_finite() || innovation_variance_s <= 1.0e-15 {
        return;
    }

    let kalman_gain_k = *covariance_p * measurement_jacobian_h / innovation_variance_s;
    let state_correction = kalman_gain_k * residual;
    state.apply_correction(&state_correction);

    // Joseph form is slightly longer than (I - KH)P but is much less likely to
    // lose positive-semidefiniteness through floating-point roundoff.
    let identity = StateCovariance::identity();
    let gain_times_jacobian_kh = kalman_gain_k * measurement_jacobian_h.transpose();
    let covariance_update = identity - gain_times_jacobian_kh;
    let updated_covariance = covariance_update * *covariance_p * covariance_update.transpose()
        + kalman_gain_k * measurement_variance_r * kalman_gain_k.transpose();
    *covariance_p = 0.5 * (updated_covariance + updated_covariance.transpose());
}

pub(super) fn require_landmarks(landmarks: &BTreeMap<String, Vector2<f64>>) -> Result<()> {
    ensure!(
        !landmarks.is_empty(),
        "perception observation arrived before the landmark map"
    );
    Ok(())
}

pub(super) fn landmark_position(
    landmarks: &BTreeMap<String, Vector2<f64>>,
    landmark_id: &str,
) -> Result<Vector2<f64>> {
    landmarks
        .get(landmark_id)
        .copied()
        .ok_or_else(|| anyhow::anyhow!("observation references unknown landmark {landmark_id}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::state::{STATE_DIMENSION, StateIndex, initial_covariance};

    #[test]
    fn scalar_update_moves_state_and_preserves_covariance_symmetry() {
        let mut state = PlanarState::default();
        let mut covariance_p = initial_covariance();
        let mut measurement_jacobian_h = StateCorrection::zeros();
        measurement_jacobian_h[StateIndex::PositionWorldX.index()] = -1.0;

        apply_scalar_update(
            &mut state,
            &mut covariance_p,
            ScalarObservation {
                residual: 0.5,
                measurement_jacobian_h,
                measurement_variance_r: 0.01,
            },
        );

        assert!(state.position_world_m.x < 0.0);
        for row in 0..STATE_DIMENSION {
            assert!(covariance_p[(row, row)] >= 0.0);
            for column in 0..STATE_DIMENSION {
                assert!(
                    (covariance_p[(row, column)] - covariance_p[(column, row)]).abs() < 1.0e-12
                );
            }
        }
    }
}
