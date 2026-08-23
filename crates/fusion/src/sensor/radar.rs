use fusion_schema::messages::{RadarDetection, RadarScan, observation_truth};

use crate::{
    bundle::MeasurementRecord,
    math::wrap_angle,
    random::DeterministicRandom,
    scenario::{LandmarkConfig, RadarConfig, ResolvedScenario},
    truth::{Sample, Trajectory},
};

use super::{
    PendingMeasurement, build_observation_truth, for_each_sample,
    geometry::{is_visible, landmark_geometry},
    record_header,
};

const DELIVERY_PRIORITY: u8 = 40;

pub(super) fn generate(
    scenario: &ResolvedScenario,
    trajectory: &Trajectory,
    random: &DeterministicRandom,
    output: &mut Vec<PendingMeasurement>,
) {
    let Some(config) = &scenario.radar else {
        return;
    };
    for_each_sample(
        scenario.effective_duration_s(),
        config.rate_hz,
        |time_ns, sequence| {
            let platform = trajectory.sample_ns(time_ns);
            let mut ideal_detections = Vec::new();
            for landmark in &scenario.world.landmarks {
                if let Some(detection) = ideal_detection(platform, landmark, config) {
                    ideal_detections.push(detection);
                }
            }

            let mut visible_detections = Vec::new();
            for ideal in &ideal_detections {
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
                let radial_velocity_noise_mps = config.radial_velocity_noise_stddev_mps
                    * random.normal_named(
                        &config.instance_id,
                        sequence,
                        "radial_velocity_noise",
                        &ideal.landmark_id,
                        0,
                    );
                visible_detections.push(RadarDetection {
                    landmark_id: ideal.landmark_id.clone(),
                    range_m: (ideal.range_m + range_noise_m).max(0.0),
                    azimuth_rad: wrap_angle(ideal.azimuth_rad + bearing_noise_rad),
                    radial_velocity_mps: ideal.radial_velocity_mps + radial_velocity_noise_mps,
                });
            }

            let record_id = format!("{}:{sequence:010}", config.instance_id);
            let arrival_ns = time_ns + config.latency_ns;
            let common_header = record_header(
                scenario,
                &record_id,
                &config.instance_id,
                time_ns,
                0,
                arrival_ns,
                sequence,
            );
            let ideal_observation = RadarScan {
                header: Some(common_header.clone()),
                detections: ideal_detections,
                association_mode: "ORACLE".to_owned(),
            };
            let visible_measurement = RadarScan {
                header: Some(common_header),
                detections: visible_detections,
                association_mode: "ORACLE".to_owned(),
            };

            output.push(PendingMeasurement {
                arrival_ns,
                priority: DELIVERY_PRIORITY,
                stable_event_id: record_id.clone(),
                measurement: MeasurementRecord::Radar(visible_measurement),
                observation_truth: Some(build_observation_truth(
                    record_id,
                    time_ns,
                    time_ns,
                    arrival_ns,
                    serde_json::json!({"ideal_count": ideal_observation.detections.len()}),
                    observation_truth::IdealObservation::IdealRadar(ideal_observation),
                )),
            });
        },
    );
}

fn ideal_detection(
    platform: Sample,
    landmark: &LandmarkConfig,
    config: &RadarConfig,
) -> Option<RadarDetection> {
    let geometry = landmark_geometry(platform, landmark)?;
    if !is_visible(geometry, config.horizontal_fov_rad, config.max_range_m) {
        return None;
    }

    Some(RadarDetection {
        landmark_id: landmark.id.clone(),
        range_m: geometry.range_m,
        azimuth_rad: geometry.azimuth_body_rad,
        radial_velocity_mps: -platform.speed_mps * geometry.azimuth_body_rad.cos(),
    })
}
