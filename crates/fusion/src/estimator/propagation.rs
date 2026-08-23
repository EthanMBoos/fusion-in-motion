use anyhow::{Result, ensure};
use fusion_schema::messages::ImuSample;
use nalgebra::Vector2;

use crate::math;

use super::state::{PlanarState, StateCovariance, StateIndex, set_variance};

const POSITION_PROCESS_VARIANCE_RATE_M2PS: f64 = 1.0e-5;
const YAW_PROCESS_VARIANCE_RATE_RAD2PS: f64 = 2.5e-5;
const FORWARD_SPEED_PROCESS_VARIANCE_RATE_M2PS3: f64 = 1.0e-3;
const GYRO_BIAS_PROCESS_VARIANCE_RATE_RAD2PS3: f64 = 1.0e-7;
const ACCEL_BIAS_PROCESS_VARIANCE_RATE_M2PS5: f64 = 1.0e-5;

pub(super) fn propagate_imu(
    state: &mut PlanarState,
    covariance_p: &mut StateCovariance,
    last_imu_stamp_ns: &mut Option<i64>,
    imu: &ImuSample,
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
    )] = -forward_speed_mps * yaw_rad.sin() * dt_s;
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
    )] = forward_speed_mps * yaw_rad.cos() * dt_s;
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

    // These are baseline tuning values expressed as variance accumulated per
    // second. They are deliberately simple rather than a calibrated IMU model.
    let mut process_noise_q = StateCovariance::zeros();
    set_variance(
        &mut process_noise_q,
        StateIndex::PositionWorldX,
        POSITION_PROCESS_VARIANCE_RATE_M2PS * dt_s,
    );
    set_variance(
        &mut process_noise_q,
        StateIndex::PositionWorldY,
        POSITION_PROCESS_VARIANCE_RATE_M2PS * dt_s,
    );
    set_variance(
        &mut process_noise_q,
        StateIndex::YawWorldFromBody,
        YAW_PROCESS_VARIANCE_RATE_RAD2PS * dt_s,
    );
    set_variance(
        &mut process_noise_q,
        StateIndex::ForwardSpeed,
        FORWARD_SPEED_PROCESS_VARIANCE_RATE_M2PS3 * dt_s,
    );
    set_variance(
        &mut process_noise_q,
        StateIndex::GyroBias,
        GYRO_BIAS_PROCESS_VARIANCE_RATE_RAD2PS3 * dt_s,
    );
    set_variance(
        &mut process_noise_q,
        StateIndex::AccelBias,
        ACCEL_BIAS_PROCESS_VARIANCE_RATE_M2PS5 * dt_s,
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
        )
        .unwrap();
        propagate_imu(
            &mut state,
            &mut covariance_p,
            &mut last_stamp_ns,
            &stationary_imu(20_000_000),
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
}
