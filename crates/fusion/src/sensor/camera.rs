use fusion_schema::messages::{CameraFeature, CameraFrame, observation_truth};

use crate::{
    bundle::MeasurementRecord,
    math::wrap_angle,
    random::DeterministicRandom,
    scenario::{CameraConfig, LandmarkConfig, ResolvedScenario},
    truth::{Sample, Trajectory},
};

use super::{
    PendingMeasurement, build_observation_truth, for_each_sample,
    geometry::{is_visible, landmark_geometry},
    record_header,
};

const DELIVERY_PRIORITY: u8 = 20;

pub(super) fn generate(
    scenario: &ResolvedScenario,
    trajectory: &Trajectory,
    random: &DeterministicRandom,
    output: &mut Vec<PendingMeasurement>,
) {
    let config = &scenario.camera;
    for_each_sample(
        scenario.effective_duration_s(),
        config.rate_hz,
        |time_ns, sequence| {
            let platform = trajectory.sample_ns(time_ns);
            let mut ideal_features = Vec::new();
            for landmark in &scenario.world.landmarks {
                if let Some(feature) = ideal_feature(platform, landmark, config) {
                    ideal_features.push(feature);
                }
            }

            let mut visible_features = Vec::new();
            for ideal in &ideal_features {
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

                let bearing_noise_rad = config.bearing_noise_stddev_rad
                    * random.normal_named(
                        &config.instance_id,
                        sequence,
                        "bearing_noise",
                        &ideal.landmark_id,
                        0,
                    );
                visible_features.push(CameraFeature {
                    landmark_id: ideal.landmark_id.clone(),
                    azimuth_rad: wrap_angle(ideal.azimuth_rad + bearing_noise_rad),
                    elevation_rad: ideal.elevation_rad,
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
            let ideal_observation = CameraFrame {
                header: Some(common_header.clone()),
                features: ideal_features,
                association_mode: "ORACLE".to_owned(),
            };
            let visible_measurement = CameraFrame {
                header: Some(common_header),
                features: visible_features,
                association_mode: "ORACLE".to_owned(),
            };

            output.push(PendingMeasurement {
                arrival_ns,
                priority: DELIVERY_PRIORITY,
                stable_event_id: record_id.clone(),
                measurement: MeasurementRecord::Camera(visible_measurement),
                observation_truth: Some(build_observation_truth(
                    record_id,
                    time_ns,
                    time_ns,
                    arrival_ns,
                    serde_json::json!({"ideal_count": ideal_observation.features.len()}),
                    observation_truth::IdealObservation::IdealCamera(ideal_observation),
                )),
            });
        },
    );
}

fn ideal_feature(
    platform: Sample,
    landmark: &LandmarkConfig,
    config: &CameraConfig,
) -> Option<CameraFeature> {
    let geometry = landmark_geometry(platform, landmark)?;
    if !is_visible(geometry, config.horizontal_fov_rad, config.max_range_m) {
        return None;
    }

    Some(CameraFeature {
        landmark_id: landmark.id.clone(),
        azimuth_rad: geometry.azimuth_body_rad,
        elevation_rad: landmark.z_m.atan2(geometry.range_m),
    })
}
