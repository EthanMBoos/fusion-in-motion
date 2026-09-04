use nalgebra::{SMatrix, SVector, Vector2};

use crate::math;

pub(super) const STATE_DIMENSION: usize = 6;
pub(super) const STATE_NAMES: [&str; STATE_DIMENSION] = [
    "x",
    "y",
    "yaw",
    "forward_speed",
    "gyro_bias_z",
    "accel_bias_x",
];

const INITIAL_POSITION_VARIANCE_M2: f64 = 1.0e-4;
const INITIAL_YAW_VARIANCE_RAD2: f64 = 1.0e-4;
const INITIAL_FORWARD_SPEED_VARIANCE_M2PS2: f64 = 0.25;
const INITIAL_GYRO_BIAS_VARIANCE_RAD2PS2: f64 = 0.01;
const INITIAL_ACCEL_BIAS_VARIANCE_M2PS4: f64 = 0.25;

pub(super) type StateCorrection = SVector<f64, STATE_DIMENSION>;
pub(super) type StateCovariance = SMatrix<f64, STATE_DIMENSION, STATE_DIMENSION>;

/// Ordering used only at matrix boundaries. Physical state equations use the
/// named fields in [`PlanarState`].
#[derive(Debug, Clone, Copy)]
#[repr(usize)]
pub(super) enum StateIndex {
    PositionWorldX = 0,
    PositionWorldY = 1,
    YawWorldFromBody = 2,
    ForwardSpeed = 3,
    GyroBias = 4,
    AccelBias = 5,
}

impl StateIndex {
    pub(super) const fn index(self) -> usize {
        self as usize
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub(super) struct PlanarState {
    pub(super) position_world_m: Vector2<f64>,
    pub(super) yaw_world_from_body_rad: f64,
    pub(super) forward_speed_mps: f64,
    pub(super) gyro_bias_radps: f64,
    pub(super) accel_bias_mps2: f64,
}

impl PlanarState {
    pub(super) fn apply_correction(&mut self, correction: &StateCorrection) {
        self.position_world_m.x += correction[StateIndex::PositionWorldX.index()];
        self.position_world_m.y += correction[StateIndex::PositionWorldY.index()];
        self.yaw_world_from_body_rad = math::wrap_angle(
            self.yaw_world_from_body_rad + correction[StateIndex::YawWorldFromBody.index()],
        );
        self.forward_speed_mps += correction[StateIndex::ForwardSpeed.index()];
        self.gyro_bias_radps += correction[StateIndex::GyroBias.index()];
        self.accel_bias_mps2 += correction[StateIndex::AccelBias.index()];
    }
}

pub(super) fn initial_covariance() -> StateCovariance {
    let mut covariance_p = StateCovariance::zeros();
    set_variance(
        &mut covariance_p,
        StateIndex::PositionWorldX,
        INITIAL_POSITION_VARIANCE_M2,
    );
    set_variance(
        &mut covariance_p,
        StateIndex::PositionWorldY,
        INITIAL_POSITION_VARIANCE_M2,
    );
    set_variance(
        &mut covariance_p,
        StateIndex::YawWorldFromBody,
        INITIAL_YAW_VARIANCE_RAD2,
    );
    set_variance(
        &mut covariance_p,
        StateIndex::ForwardSpeed,
        INITIAL_FORWARD_SPEED_VARIANCE_M2PS2,
    );
    set_variance(
        &mut covariance_p,
        StateIndex::GyroBias,
        INITIAL_GYRO_BIAS_VARIANCE_RAD2PS2,
    );
    set_variance(
        &mut covariance_p,
        StateIndex::AccelBias,
        INITIAL_ACCEL_BIAS_VARIANCE_M2PS4,
    );
    covariance_p
}

pub(super) fn set_variance(covariance: &mut StateCovariance, state: StateIndex, variance: f64) {
    covariance[(state.index(), state.index())] = variance;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn correction_indices_map_to_named_physical_state() {
        let correction = StateCorrection::from_row_slice(&[1.0, 2.0, 0.3, 4.0, 0.05, 0.6]);
        let mut state = PlanarState::default();
        state.apply_correction(&correction);

        assert_eq!(state.position_world_m, Vector2::new(1.0, 2.0));
        assert!((state.yaw_world_from_body_rad - 0.3).abs() < 1.0e-12);
        assert_eq!(state.forward_speed_mps, 4.0);
        assert_eq!(state.gyro_bias_radps, 0.05);
        assert_eq!(state.accel_bias_mps2, 0.6);
    }
}
