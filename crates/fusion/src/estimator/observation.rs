use nalgebra::{Matrix2, SMatrix, SVector, Vector2};

use super::state::{PlanarState, STATE_DIMENSION, StateCovariance, StateIndex};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UpdateResult {
    Applied { normalized_residual: f64 },
    Rejected { normalized_residual: f64 },
    Invalid,
}

pub fn apply_position_update(
    state: &mut PlanarState,
    covariance: &mut StateCovariance,
    residual: Vector2<f64>,
    measurement_covariance: Matrix2<f64>,
    gate_sigma: f64,
) -> UpdateResult {
    if !residual.iter().all(|value| value.is_finite())
        || !measurement_covariance.iter().all(|value| value.is_finite())
        || !gate_sigma.is_finite()
        || gate_sigma < 0.0
    {
        return UpdateResult::Invalid;
    }

    let mut jacobian = SMatrix::<f64, 2, STATE_DIMENSION>::zeros();
    jacobian[(0, StateIndex::PositionWorldX.index())] = 1.0;
    jacobian[(1, StateIndex::PositionWorldY.index())] = 1.0;
    let innovation_covariance =
        jacobian * *covariance * jacobian.transpose() + measurement_covariance;
    let Some(innovation_cholesky) = innovation_covariance.cholesky() else {
        return UpdateResult::Invalid;
    };
    let normalized_residual_squared = residual.dot(&innovation_cholesky.solve(&residual));
    if !normalized_residual_squared.is_finite() || normalized_residual_squared < -1.0e-12 {
        return UpdateResult::Invalid;
    }
    let normalized_residual = normalized_residual_squared.max(0.0).sqrt();
    if normalized_residual > gate_sigma {
        return UpdateResult::Rejected {
            normalized_residual,
        };
    }

    let gain = innovation_cholesky
        .solve(&(jacobian * *covariance))
        .transpose();
    let correction = gain * residual;
    let mut updated_state = *state;
    updated_state.apply_correction(&correction);
    let identity = StateCovariance::identity();
    let left = identity - gain * jacobian;
    let updated_covariance =
        left * *covariance * left.transpose() + gain * measurement_covariance * gain.transpose();
    let updated_covariance = 0.5 * (updated_covariance + updated_covariance.transpose());
    if !state_finite(&updated_state)
        || !updated_covariance.iter().all(|value| value.is_finite())
        || updated_covariance.clone_owned().cholesky().is_none()
    {
        return UpdateResult::Invalid;
    }
    *state = updated_state;
    *covariance = updated_covariance;
    UpdateResult::Applied {
        normalized_residual,
    }
}

fn state_finite(state: &PlanarState) -> bool {
    let values = SVector::<f64, 6>::from_row_slice(&[
        state.position_world_m.x,
        state.position_world_m.y,
        state.yaw_world_from_body_rad,
        state.forward_speed_mps,
        state.gyro_bias_radps,
        state.accel_bias_mps2,
    ]);
    values.iter().all(|value| value.is_finite())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accepted_update_keeps_covariance_symmetric_and_positive_definite() {
        let mut state = PlanarState::default();
        let mut covariance = StateCovariance::identity();

        assert!(matches!(
            apply_position_update(
                &mut state,
                &mut covariance,
                Vector2::new(0.5, -0.25),
                Matrix2::identity() * 0.25,
                3.0,
            ),
            UpdateResult::Applied { .. }
        ));
        assert_ne!(state.position_world_m, Vector2::zeros());
        assert!(covariance.iter().all(|value| value.is_finite()));
        assert!((covariance - covariance.transpose()).amax() < 1.0e-12);
        assert!(covariance.cholesky().is_some());
    }

    #[test]
    fn gate_rejects_an_outlier_without_changing_the_filter() {
        let mut state = PlanarState::default();
        let mut covariance = StateCovariance::identity();
        let original_state = state;
        let original_covariance = covariance;

        assert!(matches!(
            apply_position_update(
                &mut state,
                &mut covariance,
                Vector2::new(100.0, 0.0),
                Matrix2::identity() * 0.25,
                3.0,
            ),
            UpdateResult::Rejected { .. }
        ));
        assert_eq!(state.position_world_m, original_state.position_world_m);
        assert_eq!(covariance, original_covariance);
    }

    #[test]
    fn gate_uses_the_joint_position_error() {
        let mut state = PlanarState::default();
        let mut covariance = StateCovariance::identity();
        let original_covariance = covariance;
        let component_residual = 2.5 * 1.25_f64.sqrt();

        assert!(matches!(
            apply_position_update(
                &mut state,
                &mut covariance,
                Vector2::repeat(component_residual),
                Matrix2::identity() * 0.25,
                3.0,
            ),
            UpdateResult::Rejected { .. }
        ));
        assert_eq!(state.position_world_m, Vector2::zeros());
        assert_eq!(covariance, original_covariance);
    }

    #[test]
    fn invalid_variance_is_reported_without_changing_the_filter() {
        let mut state = PlanarState::default();
        let mut covariance = StateCovariance::identity();
        let original_covariance = covariance;

        assert_eq!(
            apply_position_update(
                &mut state,
                &mut covariance,
                Vector2::new(1.0, 1.0),
                Matrix2::new(f64::NAN, 0.0, 0.0, 1.0),
                3.0,
            ),
            UpdateResult::Invalid
        );
        assert_eq!(covariance, original_covariance);
    }
}
