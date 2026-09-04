use anyhow::{Result, ensure};
use fusion_schema::messages::ImuSample;
use nalgebra::Vector2;
use serde::{Deserialize, Serialize};

use crate::{math, scenario::ImuConfig};

use super::state::{PlanarState, StateCorrection, StateCovariance, StateIndex, set_variance};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ImuProcessNoise {
    pub gyro_white_noise_density_radps_sqrt_hz: f64,
    pub accel_white_noise_density_mps2_sqrt_hz: f64,
    pub gyro_bias_random_walk_radps_sqrt_s: f64,
    pub accel_bias_random_walk_mps2_sqrt_s: f64,
}

impl From<&ImuConfig> for ImuProcessNoise {
    fn from(config: &ImuConfig) -> Self {
        Self {
            gyro_white_noise_density_radps_sqrt_hz: config.gyro_white_noise_density_radps_sqrt_hz,
            accel_white_noise_density_mps2_sqrt_hz: config.accel_white_noise_density_mps2_sqrt_hz,
            gyro_bias_random_walk_radps_sqrt_s: config.gyro_bias_random_walk_radps_sqrt_s,
            accel_bias_random_walk_mps2_sqrt_s: config.accel_bias_random_walk_mps2_sqrt_s,
        }
    }
}

pub(super) fn propagate_imu(
    state: &mut PlanarState,
    covariance_p: &mut StateCovariance,
    last_imu_stamp_ns: &mut Option<i64>,
    imu: &ImuSample,
    process_noise: &ImuProcessNoise,
) -> Result<()> {
    let header = imu
        .header
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("IMU record is missing its header"))?;
    let angular_rate_body_radps = imu
        .angular_rate_radps
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("IMU record is missing angular rate"))?;
    let specific_force_body_mps2 = imu
        .specific_force_mps2
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("IMU record is missing specific force"))?;

    let Some(previous_stamp_ns) = last_imu_stamp_ns.replace(header.reported_stamp_ns) else {
        return Ok(());
    };
    let dt_s = (header.reported_stamp_ns - previous_stamp_ns) as f64 * 1.0e-9;
    ensure!(
        dt_s > 0.0,
        "baseline estimator requires increasing IMU device timestamps"
    );

    // Correct the visible IMU values using the biases estimated by the filter.
    let corrected_yaw_rate_radps = angular_rate_body_radps.z - state.gyro_bias_radps;
    let corrected_forward_accel_mps2 = specific_force_body_mps2.x - state.accel_bias_mps2;

    let yaw_rad = state.yaw_world_from_body_rad;
    let forward_speed_mps = state.forward_speed_mps;
    let forward_direction_world = Vector2::new(yaw_rad.cos(), yaw_rad.sin());
    let distance_m = forward_speed_mps * dt_s + 0.5 * corrected_forward_accel_mps2 * dt_s * dt_s;

    // Propagate the physical state first, at one consistent linearization point.
    state.position_world_m += forward_direction_world * distance_m;
    state.yaw_world_from_body_rad = math::wrap_angle(yaw_rad + corrected_yaw_rate_radps * dt_s);
    state.forward_speed_mps += corrected_forward_accel_mps2 * dt_s;

    let mut state_transition_f = StateCovariance::identity();
    state_transition_f[(
        StateIndex::PositionWorldX.index(),
        StateIndex::YawWorldFromBody.index(),
    )] = -distance_m * yaw_rad.sin();
    state_transition_f[(
        StateIndex::PositionWorldX.index(),
        StateIndex::ForwardSpeed.index(),
    )] = yaw_rad.cos() * dt_s;
    state_transition_f[(
        StateIndex::PositionWorldX.index(),
        StateIndex::AccelBias.index(),
    )] = -0.5 * yaw_rad.cos() * dt_s * dt_s;
    state_transition_f[(
        StateIndex::PositionWorldY.index(),
        StateIndex::YawWorldFromBody.index(),
    )] = distance_m * yaw_rad.cos();
    state_transition_f[(
        StateIndex::PositionWorldY.index(),
        StateIndex::ForwardSpeed.index(),
    )] = yaw_rad.sin() * dt_s;
    state_transition_f[(
        StateIndex::PositionWorldY.index(),
        StateIndex::AccelBias.index(),
    )] = -0.5 * yaw_rad.sin() * dt_s * dt_s;
    state_transition_f[(
        StateIndex::YawWorldFromBody.index(),
        StateIndex::GyroBias.index(),
    )] = -dt_s;
    state_transition_f[(
        StateIndex::ForwardSpeed.index(),
        StateIndex::AccelBias.index(),
    )] = -dt_s;

    let mut process_noise_q = StateCovariance::zeros();

    let gyro_sample_variance = process_noise.gyro_white_noise_density_radps_sqrt_hz.powi(2) / dt_s;
    let mut gyro_sensitivity = StateCorrection::zeros();
    gyro_sensitivity[StateIndex::YawWorldFromBody.index()] = dt_s;
    process_noise_q += gyro_sensitivity * gyro_sensitivity.transpose() * gyro_sample_variance;

    let accel_sample_variance = process_noise.accel_white_noise_density_mps2_sqrt_hz.powi(2) / dt_s;
    let mut accel_sensitivity = StateCorrection::zeros();
    accel_sensitivity[StateIndex::PositionWorldX.index()] = 0.5 * yaw_rad.cos() * dt_s * dt_s;
    accel_sensitivity[StateIndex::PositionWorldY.index()] = 0.5 * yaw_rad.sin() * dt_s * dt_s;
    accel_sensitivity[StateIndex::ForwardSpeed.index()] = dt_s;
    process_noise_q += accel_sensitivity * accel_sensitivity.transpose() * accel_sample_variance;

    set_variance(
        &mut process_noise_q,
        StateIndex::GyroBias,
        process_noise.gyro_bias_random_walk_radps_sqrt_s.powi(2) * dt_s,
    );
    set_variance(
        &mut process_noise_q,
        StateIndex::AccelBias,
        process_noise.accel_bias_random_walk_mps2_sqrt_s.powi(2) * dt_s,
    );

    *covariance_p =
        state_transition_f * *covariance_p * state_transition_f.transpose() + process_noise_q;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use fusion_schema::messages::{RecordHeader, Vec3};

    fn stationary_imu(stamp_ns: i64) -> ImuSample {
        ImuSample {
            header: Some(RecordHeader {
                reported_stamp_ns: stamp_ns,
                ..RecordHeader::default()
            }),
            angular_rate_radps: Some(Vec3::default()),
            specific_force_mps2: Some(Vec3::default()),
        }
    }

    fn process_noise() -> ImuProcessNoise {
        ImuProcessNoise {
            gyro_white_noise_density_radps_sqrt_hz: 0.0008,
            accel_white_noise_density_mps2_sqrt_hz: 0.012,
            gyro_bias_random_walk_radps_sqrt_s: 0.00002,
            accel_bias_random_walk_mps2_sqrt_s: 0.0005,
        }
    }

    #[test]
    fn stationary_unbiased_imu_preserves_zero_state_and_valid_covariance() {
        let mut state = PlanarState::default();
        let mut covariance_p = super::super::state::initial_covariance();
        let mut last_stamp_ns = None;

        propagate_imu(
            &mut state,
            &mut covariance_p,
            &mut last_stamp_ns,
            &stationary_imu(10_000_000),
            &process_noise(),
        )
        .unwrap();
        propagate_imu(
            &mut state,
            &mut covariance_p,
            &mut last_stamp_ns,
            &stationary_imu(20_000_000),
            &process_noise(),
        )
        .unwrap();

        assert_eq!(state.position_world_m, Vector2::zeros());
        assert_eq!(state.yaw_world_from_body_rad, 0.0);
        assert_eq!(state.forward_speed_mps, 0.0);
        for row in 0..super::super::state::STATE_DIMENSION {
            assert!(covariance_p[(row, row)] >= 0.0);
            for column in 0..super::super::state::STATE_DIMENSION {
                assert!(
                    (covariance_p[(row, column)] - covariance_p[(column, row)]).abs() < 1.0e-12
                );
            }
        }
    }

    #[test]
    fn configured_imu_noise_sets_discrete_process_covariance() {
        let propagate = |noise: ImuProcessNoise| {
            let mut state = PlanarState::default();
            let mut covariance = StateCovariance::zeros();
            let mut last_stamp_ns = None;
            propagate_imu(
                &mut state,
                &mut covariance,
                &mut last_stamp_ns,
                &stationary_imu(10_000_000),
                &noise,
            )
            .unwrap();
            propagate_imu(
                &mut state,
                &mut covariance,
                &mut last_stamp_ns,
                &stationary_imu(20_000_000),
                &noise,
            )
            .unwrap();
            covariance
        };
        let zero = ImuProcessNoise {
            gyro_white_noise_density_radps_sqrt_hz: 0.0,
            accel_white_noise_density_mps2_sqrt_hz: 0.0,
            gyro_bias_random_walk_radps_sqrt_s: 0.0,
            accel_bias_random_walk_mps2_sqrt_s: 0.0,
        };
        let configured = propagate(process_noise());
        let zero = propagate(zero);

        let dt_s: f64 = 0.01;
        let expected_yaw_variance = 0.0008_f64.powi(2) * dt_s;
        let expected_speed_variance = 0.012_f64.powi(2) * dt_s;
        let expected_position_x_variance = 0.25 * 0.012_f64.powi(2) * dt_s.powi(3);
        let expected_position_speed_covariance = 0.5 * 0.012_f64.powi(2) * dt_s.powi(2);
        let expected_gyro_bias_variance = 0.00002_f64.powi(2) * dt_s;
        let expected_accel_bias_variance = 0.0005_f64.powi(2) * dt_s;

        let assert_close = |actual: f64, expected: f64| {
            assert!(
                (actual - expected).abs() <= expected.abs() * 1.0e-12 + 1.0e-24,
                "actual {actual:e}, expected {expected:e}"
            );
        };

        assert_close(
            configured[(
                StateIndex::YawWorldFromBody.index(),
                StateIndex::YawWorldFromBody.index(),
            )],
            expected_yaw_variance,
        );
        assert_close(
            configured[(
                StateIndex::ForwardSpeed.index(),
                StateIndex::ForwardSpeed.index(),
            )],
            expected_speed_variance,
        );
        assert_close(
            configured[(
                StateIndex::PositionWorldX.index(),
                StateIndex::PositionWorldX.index(),
            )],
            expected_position_x_variance,
        );
        assert_close(
            configured[(
                StateIndex::PositionWorldX.index(),
                StateIndex::ForwardSpeed.index(),
            )],
            expected_position_speed_covariance,
        );
        assert_close(
            configured[(StateIndex::GyroBias.index(), StateIndex::GyroBias.index())],
            expected_gyro_bias_variance,
        );
        assert_close(
            configured[(StateIndex::AccelBias.index(), StateIndex::AccelBias.index())],
            expected_accel_bias_variance,
        );

        assert!(
            configured[(
                StateIndex::YawWorldFromBody.index(),
                StateIndex::YawWorldFromBody.index()
            )] > zero[(
                StateIndex::YawWorldFromBody.index(),
                StateIndex::YawWorldFromBody.index()
            )]
        );
        assert!(
            configured[(
                StateIndex::ForwardSpeed.index(),
                StateIndex::ForwardSpeed.index()
            )] > zero[(
                StateIndex::ForwardSpeed.index(),
                StateIndex::ForwardSpeed.index()
            )]
        );
        assert!(
            configured[(StateIndex::GyroBias.index(), StateIndex::GyroBias.index())]
                > zero[(StateIndex::GyroBias.index(), StateIndex::GyroBias.index())]
        );
        assert!(
            configured[(StateIndex::AccelBias.index(), StateIndex::AccelBias.index())]
                > zero[(StateIndex::AccelBias.index(), StateIndex::AccelBias.index())]
        );
    }
}
