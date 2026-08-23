use nalgebra::SVector;

use super::state::{PlanarState, StateCorrection, StateCovariance};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UpdateResult {
    Applied { normalized_residual: f64 },
    Rejected { normalized_residual: f64 },
    Invalid,
}

pub fn apply_scalar_update(
    state: &mut PlanarState,
    covariance: &mut StateCovariance,
    residual: f64,
    jacobian: StateCorrection,
    measurement_variance: f64,
    gate_sigma: f64,
) -> UpdateResult {
    if !residual.is_finite()
        || !jacobian.iter().all(|value| value.is_finite())
        || !measurement_variance.is_finite()
        || measurement_variance < 0.0
    {
        return UpdateResult::Invalid;
    }
    let innovation_variance =
        (jacobian.transpose() * *covariance * jacobian)[0] + measurement_variance;
    if !innovation_variance.is_finite() || innovation_variance <= 1.0e-15 {
        return UpdateResult::Invalid;
    }
    let normalized_residual = residual / innovation_variance.sqrt();
    if normalized_residual.abs() > gate_sigma {
        return UpdateResult::Rejected {
            normalized_residual,
        };
    }

    let gain = *covariance * jacobian / innovation_variance;
    let correction = gain * residual;
    let mut updated_state = *state;
    updated_state.apply_correction(&correction);
    let identity = StateCovariance::identity();
    let left = identity - gain * jacobian.transpose();
    let updated_covariance =
        left * *covariance * left.transpose() + gain * measurement_variance * gain.transpose();
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
        let mut jacobian = StateCorrection::zeros();
        jacobian[0] = 1.0;

        assert!(matches!(
            apply_scalar_update(&mut state, &mut covariance, 0.5, jacobian, 0.25, 3.0),
            UpdateResult::Applied { .. }
        ));
        assert!(covariance.iter().all(|value| value.is_finite()));
        assert!((covariance - covariance.transpose()).amax() < 1.0e-12);
        assert!(covariance.cholesky().is_some());
    }

    #[test]
    fn gate_rejects_an_outlier_without_changing_the_filter() {
        let mut state = PlanarState::default();
        let mut covariance = StateCovariance::identity();
        let original_covariance = covariance;
        let mut jacobian = StateCorrection::zeros();
        jacobian[0] = 1.0;

        assert!(matches!(
            apply_scalar_update(&mut state, &mut covariance, 100.0, jacobian, 0.25, 3.0),
            UpdateResult::Rejected { .. }
        ));
        assert_eq!(state.position_world_m.x, 0.0);
        assert_eq!(covariance, original_covariance);
    }

    #[test]
    fn invalid_variance_is_reported_without_changing_the_filter() {
        let mut state = PlanarState::default();
        let mut covariance = StateCovariance::identity();
        let original_covariance = covariance;

        assert_eq!(
            apply_scalar_update(
                &mut state,
                &mut covariance,
                1.0,
                StateCorrection::zeros(),
                f64::NAN,
                3.0,
            ),
            UpdateResult::Invalid
        );
        assert_eq!(covariance, original_covariance);
    }
}
