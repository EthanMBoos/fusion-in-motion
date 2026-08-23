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

pub fn propagate_imu(
    state: &mut PlanarState,
    covariance: &mut StateCovariance,
    last_imu_stamp_ns: &mut Option<i64>,
    imu: &ImuSample,
    noise: &ImuProcessNoise,
) -> Result<()> {
    let header = imu
        .header
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("IMU record has no header"))?;
    let gyro = imu
        .angular_rate_radps
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("IMU record has no angular rate"))?;
    let accel = imu
        .specific_force_mps2
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("IMU record has no specific force"))?;
    let Some(previous_stamp_ns) = last_imu_stamp_ns.replace(header.reported_stamp_ns) else {
        return Ok(());
    };
    let dt_s = (header.reported_stamp_ns - previous_stamp_ns) as f64 * 1.0e-9;
    ensure!(
        dt_s > 0.0,
        "ego estimator requires increasing IMU timestamps"
    );

    let yaw_rate = gyro.z - state.gyro_bias_radps;
    let forward_accel = accel.x - state.accel_bias_mps2;
    let yaw = state.yaw_world_from_body_rad;
    let speed = state.forward_speed_mps;
    let distance = speed * dt_s + 0.5 * forward_accel * dt_s * dt_s;
    state.position_world_m += Vector2::new(yaw.cos(), yaw.sin()) * distance;
    state.yaw_world_from_body_rad = math::wrap_angle(yaw + yaw_rate * dt_s);
    state.forward_speed_mps += forward_accel * dt_s;

    let mut transition = StateCovariance::identity();
    transition[(
        StateIndex::PositionWorldX.index(),
        StateIndex::YawWorldFromBody.index(),
    )] = -distance * yaw.sin();
    transition[(
        StateIndex::PositionWorldX.index(),
        StateIndex::ForwardSpeed.index(),
    )] = yaw.cos() * dt_s;
    transition[(
        StateIndex::PositionWorldX.index(),
        StateIndex::AccelBias.index(),
    )] = -0.5 * yaw.cos() * dt_s * dt_s;
    transition[(
        StateIndex::PositionWorldY.index(),
        StateIndex::YawWorldFromBody.index(),
    )] = distance * yaw.cos();
    transition[(
        StateIndex::PositionWorldY.index(),
        StateIndex::ForwardSpeed.index(),
    )] = yaw.sin() * dt_s;
    transition[(
        StateIndex::PositionWorldY.index(),
        StateIndex::AccelBias.index(),
    )] = -0.5 * yaw.sin() * dt_s * dt_s;
    transition[(
        StateIndex::YawWorldFromBody.index(),
        StateIndex::GyroBias.index(),
    )] = -dt_s;
    transition[(
        StateIndex::ForwardSpeed.index(),
        StateIndex::AccelBias.index(),
    )] = -dt_s;

    let mut process_noise = StateCovariance::zeros();
    let gyro_sample_variance = noise.gyro_white_noise_density_radps_sqrt_hz.powi(2) / dt_s;
    let mut gyro_sensitivity = StateCorrection::zeros();
    gyro_sensitivity[StateIndex::YawWorldFromBody.index()] = dt_s;
    process_noise += gyro_sensitivity * gyro_sensitivity.transpose() * gyro_sample_variance;
    let accel_sample_variance = noise.accel_white_noise_density_mps2_sqrt_hz.powi(2) / dt_s;
    let mut accel_sensitivity = StateCorrection::zeros();
    accel_sensitivity[StateIndex::PositionWorldX.index()] = 0.5 * yaw.cos() * dt_s * dt_s;
    accel_sensitivity[StateIndex::PositionWorldY.index()] = 0.5 * yaw.sin() * dt_s * dt_s;
    accel_sensitivity[StateIndex::ForwardSpeed.index()] = dt_s;
    process_noise += accel_sensitivity * accel_sensitivity.transpose() * accel_sample_variance;
    set_variance(
        &mut process_noise,
        StateIndex::GyroBias,
        noise.gyro_bias_random_walk_radps_sqrt_s.powi(2) * dt_s,
    );
    set_variance(
        &mut process_noise,
        StateIndex::AccelBias,
        noise.accel_bias_random_walk_mps2_sqrt_s.powi(2) * dt_s,
    );

    *covariance = transition * *covariance * transition.transpose() + process_noise;
    *covariance = 0.5 * (*covariance + covariance.transpose());
    Ok(())
}
