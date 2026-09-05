use anyhow::{Result, ensure};
use fusion_schema::messages::{EgoStateEstimate, GpsFix, ImuSample};
use nalgebra::{Matrix2, SMatrix, SVector, Vector2};

use crate::{math, scenario::EgoEstimatorConfig};

use super::{ImuProcessNoise, UpdateResult};

const STATE_DIMENSION: usize = 4;
pub(super) const STATE_NAMES: [&str; STATE_DIMENSION] = [
    "position_world_x_m",
    "position_world_y_m",
    "yaw_world_from_body_rad",
    "forward_speed_mps",
];

type StateCorrection = SVector<f64, STATE_DIMENSION>;
type StateCovariance = SMatrix<f64, STATE_DIMENSION, STATE_DIMENSION>;

#[derive(Debug, Clone, Copy)]
#[repr(usize)]
enum StateIndex {
    PositionWorldX = 0,
    PositionWorldY = 1,
    YawWorldFromBody = 2,
    ForwardSpeed = 3,
}

impl StateIndex {
    const fn index(self) -> usize {
        self as usize
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct State {
    position_world_m: Vector2<f64>,
    yaw_world_from_body_rad: f64,
    forward_speed_mps: f64,
}

impl State {
    fn apply_correction(&mut self, correction: &StateCorrection) {
        self.position_world_m.x += correction[StateIndex::PositionWorldX.index()];
        self.position_world_m.y += correction[StateIndex::PositionWorldY.index()];
        self.yaw_world_from_body_rad = math::wrap_angle(
            self.yaw_world_from_body_rad + correction[StateIndex::YawWorldFromBody.index()],
        );
        self.forward_speed_mps += correction[StateIndex::ForwardSpeed.index()];
    }

    fn is_finite(self) -> bool {
        [
            self.position_world_m.x,
            self.position_world_m.y,
            self.yaw_world_from_body_rad,
            self.forward_speed_mps,
        ]
        .into_iter()
        .all(f64::is_finite)
    }
}

pub(super) struct BasicEkf {
    state: State,
    covariance: StateCovariance,
    last_imu_stamp_ns: Option<i64>,
}

impl BasicEkf {
    pub(super) fn new(config: &EgoEstimatorConfig) -> Self {
        let stddevs = [
            config.initial_position_stddev_m,
            config.initial_position_stddev_m,
            config.initial_yaw_stddev_rad,
            config.initial_speed_stddev_mps,
        ];
        Self {
            state: State::default(),
            covariance: StateCovariance::from_diagonal(&SVector::from_fn(|index, _| {
                stddevs[index].powi(2)
            })),
            last_imu_stamp_ns: None,
        }
    }

    pub(super) fn covariance_diagonal(&self) -> Vec<f64> {
        (0..STATE_DIMENSION)
            .map(|index| self.covariance[(index, index)])
            .collect()
    }

    pub(super) fn propagate(&mut self, imu: &ImuSample, noise: &ImuProcessNoise) -> Result<()> {
        let time = imu
            .time
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("IMU record has no time"))?;
        let Some(previous_stamp_ns) = self.last_imu_stamp_ns.replace(time.measurement_time_ns)
        else {
            return Ok(());
        };
        let dt_s = (time.measurement_time_ns - previous_stamp_ns) as f64 * 1.0e-9;
        ensure!(
            dt_s > 0.0,
            "ego estimator requires increasing IMU timestamps"
        );

        let yaw = self.state.yaw_world_from_body_rad;
        let speed = self.state.forward_speed_mps;
        let distance = speed * dt_s + 0.5 * imu.forward_acceleration_mps2 * dt_s * dt_s;
        self.state.position_world_m += Vector2::new(yaw.cos(), yaw.sin()) * distance;
        self.state.yaw_world_from_body_rad = math::wrap_angle(yaw + imu.yaw_rate_radps * dt_s);
        self.state.forward_speed_mps += imu.forward_acceleration_mps2 * dt_s;

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
            StateIndex::PositionWorldY.index(),
            StateIndex::YawWorldFromBody.index(),
        )] = distance * yaw.cos();
        transition[(
            StateIndex::PositionWorldY.index(),
            StateIndex::ForwardSpeed.index(),
        )] = yaw.sin() * dt_s;

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

        self.covariance = transition * self.covariance * transition.transpose() + process_noise;
        self.covariance = 0.5 * (self.covariance + self.covariance.transpose());
        Ok(())
    }

    pub(super) fn update_gps(
        &mut self,
        config: &EgoEstimatorConfig,
        fix: &GpsFix,
    ) -> Result<UpdateResult> {
        let position = fix
            .position_world_m
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("GPS fix has no position"))?;
        ensure!(
            fix.horizontal_position_variance_m2.is_finite()
                && fix.horizontal_position_variance_m2 >= 0.0,
            "GPS variance must be finite and nonnegative"
        );
        let residual = Vector2::new(position.x, position.y) - self.state.position_world_m;
        let measurement_covariance = Matrix2::identity() * fix.horizontal_position_variance_m2;
        Ok(self.apply_position_update(residual, measurement_covariance, config.gps_gate_sigma))
    }

    fn apply_position_update(
        &mut self,
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
            jacobian * self.covariance * jacobian.transpose() + measurement_covariance;
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
            .solve(&(jacobian * self.covariance))
            .transpose();
        let correction = gain * residual;
        let mut updated_state = self.state;
        updated_state.apply_correction(&correction);
        let identity = StateCovariance::identity();
        let left = identity - gain * jacobian;
        let updated_covariance = left * self.covariance * left.transpose()
            + gain * measurement_covariance * gain.transpose();
        let updated_covariance = 0.5 * (updated_covariance + updated_covariance.transpose());
        if !updated_state.is_finite()
            || !updated_covariance.iter().all(|value| value.is_finite())
            || updated_covariance.cholesky().is_none()
        {
            return UpdateResult::Invalid;
        }
        self.state = updated_state;
        self.covariance = updated_covariance;
        UpdateResult::Applied {
            normalized_residual,
        }
    }

    pub(super) fn estimate(
        &self,
        estimate_time_ns: i64,
        available_time_ns: i64,
    ) -> EgoStateEstimate {
        EgoStateEstimate {
            estimate_time_ns,
            available_time_ns,
            pose_world: Some(math::pose2(
                self.state.position_world_m.x,
                self.state.position_world_m.y,
                self.state.yaw_world_from_body_rad,
            )),
            forward_speed_mps: self.state.forward_speed_mps,
            state_covariance: (0..STATE_DIMENSION)
                .flat_map(|row| {
                    (0..STATE_DIMENSION).map(move |column| self.covariance[(row, column)])
                })
                .collect(),
            gyro_bias_z_radps: None,
            accel_bias_x_mps2: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fusion_schema::messages::MeasurementTime;

    fn imu(time_s: i64, yaw_rate_radps: f64, forward_acceleration_mps2: f64) -> ImuSample {
        ImuSample {
            time: Some(MeasurementTime {
                measurement_time_ns: time_s * 1_000_000_000,
                arrival_time_ns: time_s * 1_000_000_000,
            }),
            yaw_rate_radps,
            forward_acceleration_mps2,
        }
    }

    #[test]
    fn propagation_matches_constant_acceleration_and_yaw_rate() -> Result<()> {
        let mut filter = BasicEkf::new(&EgoEstimatorConfig::default());
        let noise = ImuProcessNoise {
            gyro_white_noise_density_radps_sqrt_hz: 0.0,
            accel_white_noise_density_mps2_sqrt_hz: 0.0,
            gyro_bias_random_walk_radps_sqrt_s: 0.0,
            accel_bias_random_walk_mps2_sqrt_s: 0.0,
        };

        filter.propagate(&imu(0, 0.5, 2.0), &noise)?;
        filter.propagate(&imu(1, 0.5, 2.0), &noise)?;

        assert!((filter.state.position_world_m.x - 1.0).abs() < 1.0e-12);
        assert!(filter.state.position_world_m.y.abs() < 1.0e-12);
        assert!((filter.state.yaw_world_from_body_rad - 0.5).abs() < 1.0e-12);
        assert!((filter.state.forward_speed_mps - 2.0).abs() < 1.0e-12);
        assert!((filter.covariance - filter.covariance.transpose()).amax() < 1.0e-12);
        Ok(())
    }

    #[test]
    fn gps_update_keeps_covariance_symmetric_and_positive_definite() {
        let mut filter = BasicEkf::new(&EgoEstimatorConfig::default());
        filter.covariance = StateCovariance::identity();

        assert!(matches!(
            filter
                .apply_position_update(Vector2::new(0.5, -0.25), Matrix2::identity() * 0.25, 3.0,),
            UpdateResult::Applied { .. }
        ));
        assert_ne!(filter.state.position_world_m, Vector2::zeros());
        assert!((filter.covariance - filter.covariance.transpose()).amax() < 1.0e-12);
        assert!(filter.covariance.cholesky().is_some());
    }

    #[test]
    fn gps_gate_rejects_an_outlier_without_changing_the_filter() {
        let mut filter = BasicEkf::new(&EgoEstimatorConfig::default());
        filter.covariance = StateCovariance::identity();
        let original_state = filter.state;
        let original_covariance = filter.covariance;

        assert!(matches!(
            filter
                .apply_position_update(Vector2::new(100.0, 0.0), Matrix2::identity() * 0.25, 3.0,),
            UpdateResult::Rejected { .. }
        ));
        assert_eq!(
            filter.state.position_world_m,
            original_state.position_world_m
        );
        assert_eq!(filter.covariance, original_covariance);
    }
}
