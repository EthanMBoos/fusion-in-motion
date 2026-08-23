use fusion_schema::messages::{DetectionTruth, LidarDetection, LidarScan, observation_truth};

use crate::{
    bundle::MeasurementRecord, math::wrap_angle, random::DeterministicRandom,
    scenario::ResolvedScenario, truth::Trajectory,
};

use super::{
    PendingMeasurement, build_observation_truth, for_each_sample,
    geometry::{is_visible, object_geometry},
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
            let object_count = scenario.world.objects.len().max(1);
            let record_id = format!("{}:{sequence:010}", config.instance_id);
            let mut ideal_detections = Vec::new();
            let mut visible_detections = Vec::new();
            let mut detection_truth = Vec::new();

            for (index, object) in scenario.world.objects.iter().enumerate() {
                let acquisition_offset_ns = if object_count == 1 {
                    config.scan_duration_ns
                } else {
                    (config.scan_duration_ns as f64 * index as f64 / (object_count - 1) as f64)
                        .round() as i64
                };
                let time_ns = scan_end_ns - config.scan_duration_ns + acquisition_offset_ns;
                let platform = trajectory.sample_ns(time_ns);
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
                    config.range_noise_stddev_m.powi(2),
                    0.0,
                    0.0,
                    0.0,
                    config.bearing_noise_stddev_rad.powi(2),
                    0.0,
                    0.0,
                    0.0,
                    config.bearing_noise_stddev_rad.powi(2),
                ];
                ideal_detections.push(LidarDetection {
                    detection_id: detection_id.clone(),
                    association_key: object.association_key.clone(),
                    range_m: geometry.range_m,
                    azimuth_rad: geometry.azimuth_sensor_rad,
                    elevation_rad: geometry.elevation_sensor_rad,
                    acquisition_offset_ns,
                    spherical_covariance: covariance.clone(),
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
                visible_detections.push(LidarDetection {
                    detection_id,
                    association_key: object.association_key.clone(),
                    range_m: (geometry.range_m
                        + config.range_noise_stddev_m
                            * random.normal_named(
                                &config.instance_id,
                                sequence,
                                "range_noise",
                                &object.id,
                                0,
                            ))
                    .max(0.0),
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
                    acquisition_offset_ns,
                    spherical_covariance: covariance,
                });
            }

            let arrival_ns = scan_end_ns + config.latency_ns;
            let header = record_header(
                scenario,
                &record_id,
                &config.instance_id,
                scan_end_ns,
                config.scan_duration_ns,
                arrival_ns,
                sequence,
            );
            let ideal = LidarScan {
                header: Some(header.clone()),
                frame_id: config.mount.frame.clone(),
                detections: ideal_detections,
                association_mode: "PROVIDED".to_owned(),
            };
            let visible = LidarScan {
                header: Some(header),
                frame_id: config.mount.frame.clone(),
                detections: visible_detections,
                association_mode: "PROVIDED".to_owned(),
            };
            output.push(PendingMeasurement {
                arrival_ns,
                priority: DELIVERY_PRIORITY,
                stable_event_id: record_id.clone(),
                measurement: MeasurementRecord::Lidar(visible),
                observation_truth: Some(build_observation_truth(
                    record_id,
                    scan_end_ns - config.scan_duration_ns,
                    scan_end_ns,
                    arrival_ns,
                    observation_truth::IdealObservation::IdealLidar(ideal),
                    None,
                    detection_truth,
                )),
            });
        },
    );
}
