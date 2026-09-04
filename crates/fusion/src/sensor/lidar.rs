use fusion_schema::messages::{LidarReturn, LidarScan, observation_truth};

use crate::{
    bundle::MeasurementRecord,
    math::wrap_angle,
    random::DeterministicRandom,
    scenario::{LandmarkConfig, LidarConfig, ResolvedScenario},
    truth::{Sample, Trajectory},
};

use super::{
    PendingMeasurement, build_observation_truth, for_each_sample,
    geometry::{is_visible, landmark_geometry},
    record_header,
};

const DELIVERY_PRIORITY: u8 = 30;

pub(super) fn generate(
    scenario: &ResolvedScenario,
    trajectory: &Trajectory,
    random: &DeterministicRandom,
    output: &mut Vec<PendingMeasurement>,
) {
    let config = &scenario.lidar;
    for_each_sample(
        scenario.effective_duration_s(),
        config.rate_hz,
        |scan_end_ns, sequence| {
            let landmark_count = scenario.world.landmarks.len().max(1);
            let mut ideal_returns = Vec::new();
            for (landmark_index, landmark) in scenario.world.landmarks.iter().enumerate() {
                let acquisition_offset_ns =
                    acquisition_offset_ns(landmark_index, landmark_count, config.scan_duration_ns);
                let ray_time_ns = scan_end_ns - config.scan_duration_ns + acquisition_offset_ns;
                let platform = trajectory.sample_ns(ray_time_ns);
                if let Some(hit) = ideal_return(platform, landmark, config, acquisition_offset_ns) {
                    ideal_returns.push(hit);
                }
            }

            let mut visible_returns = Vec::new();
            for ideal in &ideal_returns {
                let detected = random.uniform_named(
                    &config.instance_id,
                    sequence,
                    "detection",
                    &ideal.landmark_id,
                    0,
                ) <= config.detection_probability;
                if !detected {
                    continue;
                }

                let range_noise_m = config.range_noise_stddev_m
                    * random.normal_named(
                        &config.instance_id,
                        sequence,
                        "range_noise",
                        &ideal.landmark_id,
                        0,
                    );
                let bearing_noise_rad = config.bearing_noise_stddev_rad
                    * random.normal_named(
                        &config.instance_id,
                        sequence,
                        "bearing_noise",
                        &ideal.landmark_id,
                        0,
                    );
                visible_returns.push(LidarReturn {
                    landmark_id: ideal.landmark_id.clone(),
                    range_m: (ideal.range_m + range_noise_m).max(0.0),
                    azimuth_rad: wrap_angle(ideal.azimuth_rad + bearing_noise_rad),
                    acquisition_offset_ns: ideal.acquisition_offset_ns,
                });
            }

            let record_id = format!("{}:{sequence:010}", config.instance_id);
            let arrival_ns = scan_end_ns + config.latency_ns;
            let common_header = record_header(
                scenario,
                &record_id,
                &config.instance_id,
                scan_end_ns,
                config.scan_duration_ns,
                arrival_ns,
                sequence,
            );
            let ideal_observation = LidarScan {
                header: Some(common_header.clone()),
                returns: ideal_returns,
                association_mode: "ORACLE".to_owned(),
            };
            let visible_measurement = LidarScan {
                header: Some(common_header),
                returns: visible_returns,
                association_mode: "ORACLE".to_owned(),
            };

            output.push(PendingMeasurement {
                arrival_ns,
                priority: DELIVERY_PRIORITY,
                stable_event_id: record_id.clone(),
                measurement: MeasurementRecord::Lidar(visible_measurement),
                observation_truth: Some(build_observation_truth(
                    record_id,
                    scan_end_ns - config.scan_duration_ns,
                    scan_end_ns,
                    arrival_ns,
                    observation_truth::IdealObservation::IdealLidar(ideal_observation),
                )),
            });
        },
    );
}

fn acquisition_offset_ns(
    landmark_index: usize,
    landmark_count: usize,
    scan_duration_ns: i64,
) -> i64 {
    if landmark_count == 1 {
        return 0;
    }
    (scan_duration_ns as f64 * landmark_index as f64 / (landmark_count - 1) as f64).round() as i64
}

fn ideal_return(
    platform: Sample,
    landmark: &LandmarkConfig,
    config: &LidarConfig,
    acquisition_offset_ns: i64,
) -> Option<LidarReturn> {
    let geometry = landmark_geometry(platform, landmark)?;
    if !is_visible(geometry, config.horizontal_fov_rad, config.max_range_m) {
        return None;
    }

    Some(LidarReturn {
        landmark_id: landmark.id.clone(),
        range_m: geometry.range_m,
        azimuth_rad: geometry.azimuth_body_rad,
        acquisition_offset_ns,
    })
}
