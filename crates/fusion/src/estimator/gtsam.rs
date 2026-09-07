use anyhow::Result;
use fusion_gtsam::{GpsUpdate, GtsamEkfEstimator, PlanarConfig};
use fusion_schema::messages::{EgoStateEstimate, GpsFix, ImuSample};

use crate::math;

use super::{EstimatorSettings, UpdateResult};

pub(super) struct GtsamEkfPlanarEstimator {
    estimator: GtsamEkfEstimator,
}

impl GtsamEkfPlanarEstimator {
    pub(super) fn new(settings: &EstimatorSettings) -> Result<Self> {
        anyhow::ensure!(
            Self::version() == "4.2.2",
            "expected GTSAM 4.2.2, found {}",
            Self::version()
        );
        Ok(Self {
            estimator: GtsamEkfEstimator::new(&planar_config(settings))?,
        })
    }

    pub(super) fn version() -> String {
        GtsamEkfEstimator::version()
    }

    pub(super) fn propagate(&mut self, imu: &ImuSample) -> Result<()> {
        let time = imu
            .time
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("IMU record has no time"))?;
        self.estimator.process_imu(
            time.measurement_time_ns,
            imu.yaw_rate_radps,
            imu.forward_acceleration_mps2,
        )?;
        Ok(())
    }

    pub(super) fn update_gps(&mut self, fix: &GpsFix) -> Result<UpdateResult> {
        let position = fix
            .position_world_m
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("GPS fix has no position"))?;
        let result = self.estimator.process_gps(
            position.x,
            position.y,
            fix.horizontal_position_variance_m2,
        )?;
        Ok(match result.update {
            GpsUpdate::Applied => UpdateResult::Applied {
                normalized_residual: result.normalized_residual,
            },
            GpsUpdate::Rejected => UpdateResult::Rejected {
                normalized_residual: result.normalized_residual,
            },
            GpsUpdate::Invalid => UpdateResult::Invalid,
            _ => unreachable!("cxx enum contained an unknown GPS update"),
        })
    }

    pub(super) fn estimate(
        &self,
        estimate_time_ns: i64,
        available_time_ns: i64,
    ) -> Result<EgoStateEstimate> {
        let estimate = self.estimator.estimate()?;
        Ok(EgoStateEstimate {
            estimate_time_ns,
            available_time_ns,
            pose_world: Some(math::pose2(
                estimate.position_world_x_m,
                estimate.position_world_y_m,
                estimate.yaw_world_from_body_rad,
            )),
            forward_speed_mps: estimate.forward_speed_mps,
            state_covariance: estimate.state_covariance.into_iter().collect(),
            gyro_bias_z_radps: Some(estimate.gyro_bias_z_radps),
            accel_bias_x_mps2: Some(estimate.accel_bias_x_mps2),
        })
    }
}

fn planar_config(settings: &EstimatorSettings) -> PlanarConfig {
    let noise = settings.imu_process_noise;
    PlanarConfig {
        initial_position_variance_m2: settings.initial_position_variance_m2,
        initial_yaw_variance_rad2: settings.initial_yaw_variance_rad2,
        initial_speed_variance_m2ps2: settings.initial_speed_variance_m2ps2,
        initial_gyro_bias_variance_rad2ps2: settings.initial_gyro_bias_variance_rad2ps2,
        initial_accel_bias_variance_m2ps4: settings.initial_accel_bias_variance_m2ps4,
        gyro_white_noise_density_radps_sqrt_hz: noise.gyro_white_noise_density_radps_sqrt_hz,
        accel_white_noise_density_mps2_sqrt_hz: noise.accel_white_noise_density_mps2_sqrt_hz,
        gyro_bias_random_walk_radps_sqrt_s: noise.gyro_bias_random_walk_radps_sqrt_s,
        accel_bias_random_walk_mps2_sqrt_s: noise.accel_bias_random_walk_mps2_sqrt_s,
        gps_gate_sigma: settings.gps_gate_sigma,
    }
}

#[cfg(test)]
mod tests {
    use fusion_schema::messages::MeasurementTime;

    use super::*;

    fn settings() -> EstimatorSettings {
        let config = crate::scenario::EgoEstimatorConfig {
            algorithm: crate::scenario::EgoEstimatorAlgorithm::GtsamEkfPlanar,
            ..Default::default()
        };
        let imu = crate::scenario::ImuConfig {
            gyro_white_noise_density_radps_sqrt_hz: 0.0,
            accel_white_noise_density_mps2_sqrt_hz: 0.0,
            gyro_bias_random_walk_radps_sqrt_s: 0.0,
            accel_bias_random_walk_mps2_sqrt_s: 0.0,
            ..Default::default()
        };
        EstimatorSettings::resolve(&config, &imu)
    }

    fn imu(time_s: i64, yaw_rate_radps: f64, acceleration_mps2: f64) -> ImuSample {
        ImuSample {
            time: Some(MeasurementTime {
                measurement_time_ns: time_s * 1_000_000_000,
                arrival_time_ns: time_s * 1_000_000_000,
            }),
            yaw_rate_radps,
            forward_acceleration_mps2: acceleration_mps2,
        }
    }

    #[test]
    fn gtsam_propagation_matches_a_hand_calculated_step() -> Result<()> {
        let mut filter = GtsamEkfPlanarEstimator::new(&settings())?;
        filter.propagate(&imu(0, 0.5, 2.0))?;
        filter.propagate(&imu(1, 0.5, 2.0))?;
        let estimate = filter.estimate(1_000_000_000, 1_000_000_000)?;
        let pose = estimate.pose_world.unwrap();
        let position = pose.position.unwrap();

        assert!((position.x - 1.0).abs() < 1.0e-10);
        assert!(position.y.abs() < 1.0e-10);
        assert!((pose.yaw_rad - 0.5).abs() < 1.0e-10);
        assert!((estimate.forward_speed_mps - 2.0).abs() < 1.0e-10);
        Ok(())
    }

    #[test]
    fn cxx_turns_a_gtsam_constructor_exception_into_an_error() {
        let mut settings = settings();
        settings.initial_position_variance_m2 = -1.0;
        let error = GtsamEkfPlanarEstimator::new(&settings)
            .err()
            .expect("invalid covariance should fail");
        assert!(error.to_string().contains("initial covariance"));
    }
}
