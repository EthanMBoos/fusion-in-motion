use fusion_schema::messages::{ImuSample, ObservationTruth, observation_truth};
use nalgebra::Vector3;

use crate::{
    bundle::MeasurementRecord,
    math::GRAVITY_WORLD_MPS2,
    random::DeterministicRandom,
    scenario::{ImuConfig, ResolvedScenario, Vec3Config},
    truth::Trajectory,
};

use super::{PendingMeasurement, period_ns, record_header, seconds_to_ns, to_proto};

const DELIVERY_PRIORITY: u8 = 10;
const AVERAGING_POINTS: usize = 8;

struct IdealImuSignal {
    angular_rate_body_radps: Vector3<f64>,
    specific_force_body_mps2: Vector3<f64>,
}

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
    let mut gyro_bias_body_radps = config.gyro_turn_on_bias_radps.to_vector();
    let mut accel_bias_body_mps2 = config.accel_turn_on_bias_mps2.to_vector();
    let mut sample_end_ns = sample_period_ns;
    let mut sequence = 0_u64;

    while sample_end_ns <= end_ns {
        let sample_start_ns = sample_end_ns - sample_period_ns;
        let ideal = average_ideal_signal(trajectory, sample_start_ns, sample_end_ns);

        drive_bias_random_walk(
            config,
            random,
            sequence,
            sample_period_s,
            &mut gyro_bias_body_radps,
            &mut accel_bias_body_mps2,
        );
        let (measured_gyro_body_radps, measured_accel_body_mps2) = apply_measurement_effects(
            config,
            random,
            sequence,
            sample_period_s,
            &ideal,
            gyro_bias_body_radps,
            accel_bias_body_mps2,
        );

        let record_id = format!("{}:{sequence:010}", config.instance_id);
        let arrival_ns = sample_end_ns + config.latency_ns;
        let common_header = record_header(
            scenario,
            &record_id,
            &config.instance_id,
            sample_end_ns + config.clock_offset_ns,
            sample_period_ns,
            arrival_ns,
            sequence,
        );
        let ideal_observation = ImuSample {
            header: Some(common_header.clone()),
            angular_rate_radps: Some(to_proto(ideal.angular_rate_body_radps)),
            specific_force_mps2: Some(to_proto(ideal.specific_force_body_mps2)),
        };
        let visible_measurement = ImuSample {
            header: Some(common_header),
            angular_rate_radps: Some(to_proto(measured_gyro_body_radps)),
            specific_force_mps2: Some(to_proto(measured_accel_body_mps2)),
        };

        output.push(PendingMeasurement {
            arrival_ns,
            priority: DELIVERY_PRIORITY,
            stable_event_id: record_id.clone(),
            measurement: MeasurementRecord::Imu(visible_measurement),
            observation_truth: Some(ObservationTruth {
                visible_record_id: record_id,
                acquisition_start_truth_ns: sample_start_ns,
                acquisition_end_truth_ns: sample_end_ns,
                publish_truth_ns: arrival_ns,
                arrival_truth_ns: arrival_ns,
                effect_values_json: serde_json::json!({
                    "gyro_bias_radps": [
                        gyro_bias_body_radps.x,
                        gyro_bias_body_radps.y,
                        gyro_bias_body_radps.z
                    ],
                    "accel_bias_mps2": [
                        accel_bias_body_mps2.x,
                        accel_bias_body_mps2.y,
                        accel_bias_body_mps2.z
                    ]
                })
                .to_string(),
                ideal_observation: Some(observation_truth::IdealObservation::IdealImu(
                    ideal_observation,
                )),
            }),
        });

        sample_end_ns += sample_period_ns;
        sequence += 1;
    }
}

fn drive_bias_random_walk(
    config: &ImuConfig,
    random: &DeterministicRandom,
    sequence: u64,
    sample_period_s: f64,
    gyro_bias_body_radps: &mut Vector3<f64>,
    accel_bias_body_mps2: &mut Vector3<f64>,
) {
    for axis in 0..3 {
        gyro_bias_body_radps[axis] += config.gyro_bias_random_walk_radps_sqrt_s
            * sample_period_s.sqrt()
            * random.normal(
                &config.instance_id,
                sequence,
                "gyro_bias_drive",
                axis as u64,
            );
        accel_bias_body_mps2[axis] += config.accel_bias_random_walk_mps2_sqrt_s
            * sample_period_s.sqrt()
            * random.normal(
                &config.instance_id,
                sequence,
                "accel_bias_drive",
                axis as u64,
            );
    }
}

#[allow(clippy::too_many_arguments)]
fn apply_measurement_effects(
    config: &ImuConfig,
    random: &DeterministicRandom,
    sequence: u64,
    sample_period_s: f64,
    ideal: &IdealImuSignal,
    gyro_bias_body_radps: Vector3<f64>,
    accel_bias_body_mps2: Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let mut measured_gyro_body_radps = ideal.angular_rate_body_radps + gyro_bias_body_radps;
    let mut measured_accel_body_mps2 = ideal.specific_force_body_mps2 + accel_bias_body_mps2;
    let gyro_noise_stddev_radps =
        config.gyro_white_noise_density_radps_sqrt_hz / sample_period_s.sqrt();
    let accel_noise_stddev_mps2 =
        config.accel_white_noise_density_mps2_sqrt_hz / sample_period_s.sqrt();

    for axis in 0..3 {
        measured_gyro_body_radps[axis] += gyro_noise_stddev_radps
            * random.normal(
                &config.instance_id,
                sequence,
                "gyro_white_noise",
                axis as u64,
            );
        measured_accel_body_mps2[axis] += accel_noise_stddev_mps2
            * random.normal(
                &config.instance_id,
                sequence,
                "accel_white_noise",
                axis as u64,
            );
    }

    measured_gyro_body_radps = measured_gyro_body_radps.map(|value_radps| {
        apply_limits(
            value_radps,
            config.gyro_saturation_radps,
            config.quantization_step,
        )
    });
    measured_accel_body_mps2 = measured_accel_body_mps2.map(|value_mps2| {
        apply_limits(
            value_mps2,
            config.accel_saturation_mps2,
            config.quantization_step,
        )
    });

    (measured_gyro_body_radps, measured_accel_body_mps2)
}

fn average_ideal_signal(
    trajectory: &Trajectory,
    sample_start_ns: i64,
    sample_end_ns: i64,
) -> IdealImuSignal {
    let mut angular_rate_body_radps = Vector3::zeros();
    let mut specific_force_body_mps2 = Vector3::zeros();

    for point in 0..AVERAGING_POINTS {
        let fraction = (point as f64 + 0.5) / AVERAGING_POINTS as f64;
        let time_ns =
            sample_start_ns + ((sample_end_ns - sample_start_ns) as f64 * fraction).round() as i64;
        let sample = trajectory.sample_ns(time_ns);

        angular_rate_body_radps.z += sample.yaw_rate_radps;
        let specific_force_world_mps2 = sample.acceleration_world() - GRAVITY_WORLD_MPS2;
        let cos_yaw = sample.yaw_world_from_body_rad.cos();
        let sin_yaw = sample.yaw_world_from_body_rad.sin();
        specific_force_body_mps2.x +=
            cos_yaw * specific_force_world_mps2.x + sin_yaw * specific_force_world_mps2.y;
        specific_force_body_mps2.y +=
            -sin_yaw * specific_force_world_mps2.x + cos_yaw * specific_force_world_mps2.y;
        specific_force_body_mps2.z += specific_force_world_mps2.z;
    }

    IdealImuSignal {
        angular_rate_body_radps: angular_rate_body_radps / AVERAGING_POINTS as f64,
        specific_force_body_mps2: specific_force_body_mps2 / AVERAGING_POINTS as f64,
    }
}

fn apply_limits(value: f64, saturation: f64, quantization: f64) -> f64 {
    let clipped = value.clamp(-saturation, saturation);
    if quantization > 0.0 {
        (clipped / quantization).round() * quantization
    } else {
        clipped
    }
}

trait Vec3ConfigExt {
    fn to_vector(self) -> Vector3<f64>;
}

impl Vec3ConfigExt for Vec3Config {
    fn to_vector(self) -> Vector3<f64> {
        Vector3::new(self.x, self.y, self.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn limits_clip_before_quantizing() {
        assert_eq!(apply_limits(1.08, 1.0, 0.1), 1.0);
        assert!((apply_limits(0.26, 1.0, 0.1) - 0.3).abs() < 1.0e-12);
    }
}
