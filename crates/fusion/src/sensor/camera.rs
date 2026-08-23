use fusion_schema::messages::{CameraDetection, CameraFrame, DetectionTruth, observation_truth};

use crate::{
    bundle::MeasurementRecord, math::wrap_angle, random::DeterministicRandom,
    scenario::ResolvedScenario, truth::Trajectory,
};

use super::{
    PendingMeasurement, build_observation_truth, for_each_sample,
    geometry::{is_visible, object_geometry},
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
            let record_id = format!("{}:{sequence:010}", config.instance_id);
            let mut ideal_detections = Vec::new();
            let mut visible_detections = Vec::new();
            let mut detection_truth = Vec::new();

            for object in &scenario.world.objects {
                let Some(geometry) = object_geometry(platform, object, &config.mount) else {
                    continue;
                };
                if !is_visible(
                    geometry,
                    config.horizontal_fov_rad,
                    config.vertical_fov_rad,
                    config.max_range_m,
                ) {
                    continue;
                }
                let detection_id = format!("{record_id}:{}", object.association_key);
                let covariance = vec![
                    config.bearing_noise_stddev_rad.powi(2),
                    0.0,
                    0.0,
                    config.bearing_noise_stddev_rad.powi(2),
                ];
                ideal_detections.push(CameraDetection {
                    detection_id: detection_id.clone(),
                    association_key: object.association_key.clone(),
                    azimuth_rad: geometry.azimuth_sensor_rad,
                    elevation_rad: geometry.elevation_sensor_rad,
                    angular_covariance: covariance.clone(),
                });
                if random.uniform_named(&config.instance_id, sequence, "detection", &object.id, 0)
                    > config.detection_probability
                {
                    continue;
                }
                detection_truth.push(DetectionTruth {
                    detection_id: detection_id.clone(),
                    object_id: object.id.clone(),
                });
                visible_detections.push(CameraDetection {
                    detection_id,
                    association_key: object.association_key.clone(),
                    azimuth_rad: wrap_angle(
                        geometry.azimuth_sensor_rad
                            + config.bearing_noise_stddev_rad
                                * random.normal_named(
                                    &config.instance_id,
                                    sequence,
                                    "bearing_noise",
                                    &object.id,
                                    0,
                                ),
                    ),
                    elevation_rad: geometry.elevation_sensor_rad
                        + config.bearing_noise_stddev_rad
                            * random.normal_named(
                                &config.instance_id,
                                sequence,
                                "bearing_noise",
                                &object.id,
                                1,
                            ),
                    angular_covariance: covariance,
                });
            }

            let arrival_ns = time_ns + config.latency_ns;
            let header = record_header(
                scenario,
                &record_id,
                &config.instance_id,
                time_ns,
                0,
                arrival_ns,
                sequence,
            );
            let ideal = CameraFrame {
                header: Some(header.clone()),
                frame_id: config.mount.frame.clone(),
                detections: ideal_detections,
                association_mode: "PROVIDED".to_owned(),
            };
            let visible = CameraFrame {
                header: Some(header),
                frame_id: config.mount.frame.clone(),
                detections: visible_detections,
                association_mode: "PROVIDED".to_owned(),
            };
            output.push(PendingMeasurement {
                arrival_ns,
                priority: DELIVERY_PRIORITY,
                stable_event_id: record_id.clone(),
                measurement: MeasurementRecord::Camera(visible),
                observation_truth: Some(build_observation_truth(
                    record_id,
                    time_ns,
                    time_ns,
                    arrival_ns,
                    observation_truth::IdealObservation::IdealCamera(ideal),
                    None,
                    detection_truth,
                )),
            });
        },
    );
}
