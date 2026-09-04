use fusion_schema::messages::{ImuBiasTruth, ImuSample};

use crate::{
    bundle::MeasurementRecord, random::DeterministicRandom, scenario::ResolvedScenario,
    truth::Trajectory,
};

use super::{PendingMeasurement, measurement_time, period_ns, seconds_to_ns};

const DELIVERY_PRIORITY: u8 = 10;

pub(super) fn generate(
    scenario: &ResolvedScenario,
    trajectory: &Trajectory,
    random: &DeterministicRandom,
    output: &mut Vec<PendingMeasurement>,
) {
    let config = &scenario.imu;
    let sample_period_ns = period_ns(config.rate_hz);
    let sample_period_s = sample_period_ns as f64 * 1.0e-9;
    let end_ns = seconds_to_ns(scenario.effective_duration_s());
    let mut gyro_bias = config.gyro_bias_radps;
    let mut accel_bias = config.accel_bias_mps2;
    let mut sample_end_ns = sample_period_ns;
    let mut sequence = 0_u64;

    while sample_end_ns <= end_ns {
        let ideal = trajectory.sample_ns(sample_end_ns);
        gyro_bias += config.gyro_bias_random_walk_radps_sqrt_s
            * sample_period_s.sqrt()
            * random.normal("imu", sequence, "gyro_bias_drive", 0);
        accel_bias += config.accel_bias_random_walk_mps2_sqrt_s
            * sample_period_s.sqrt()
            * random.normal("imu", sequence, "accel_bias_drive", 0);

        let yaw_rate = ideal.yaw_rate_radps
            + gyro_bias
            + config.gyro_white_noise_density_radps_sqrt_hz / sample_period_s.sqrt()
                * random.normal("imu", sequence, "gyro_white_noise", 0);
        let acceleration = ideal.longitudinal_acceleration_mps2
            + accel_bias
            + config.accel_white_noise_density_mps2_sqrt_hz / sample_period_s.sqrt()
                * random.normal("imu", sequence, "accel_white_noise", 0);
        let measurement_ns = sample_end_ns + config.clock_offset_ns;
        let arrival_ns = sample_end_ns + config.latency_ns;
        output.push(PendingMeasurement {
            arrival_ns,
            priority: DELIVERY_PRIORITY,
            stable_event_id: format!("imu:{sequence:010}"),
            measurement: MeasurementRecord::Imu(ImuSample {
                time: Some(measurement_time(measurement_ns, arrival_ns)),
                yaw_rate_radps: yaw_rate,
                forward_acceleration_mps2: acceleration,
            }),
            imu_bias_truth: Some(ImuBiasTruth {
                time_ns: measurement_ns,
                gyro_bias_z_radps: gyro_bias,
                accel_bias_x_mps2: accel_bias,
            }),
        });

        sample_end_ns += sample_period_ns;
        sequence += 1;
    }
}
