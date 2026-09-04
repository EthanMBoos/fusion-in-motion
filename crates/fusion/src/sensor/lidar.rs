use fusion_schema::messages::{LidarDetection, LidarScan};

use crate::{
    bundle::MeasurementRecord, math::wrap_angle, random::DeterministicRandom,
    scenario::ResolvedScenario, truth::Trajectory,
};

use super::{
    PendingMeasurement, for_each_sample,
    geometry::{is_visible, object_geometry},
    measurement_time,
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
            let mut detections = Vec::new();
            for (index, object) in scenario.world.objects.iter().enumerate() {
                let offset_ns = if object_count == 1 {
                    config.scan_duration_ns
                } else {
                    (config.scan_duration_ns as f64 * index as f64 / (object_count - 1) as f64)
                        .round() as i64
                };
                let time_ns = scan_end_ns - config.scan_duration_ns + offset_ns;
                let platform = trajectory.sample_ns(time_ns);
                let Some(geometry) = object_geometry(platform, object) else {
                    continue;
                };
                if !is_visible(geometry, config.horizontal_fov_rad, config.max_range_m)
                    || random.uniform_named("lidar", sequence, "detection", &object.id, 0)
                        > config.detection_probability
                {
                    continue;
                }
                detections.push(LidarDetection {
                    track_key: object.id.clone(),
                    measurement_time_ns: time_ns,
                    range_m: (geometry.range_m
                        + config.range_noise_stddev_m
                            * random.normal_named("lidar", sequence, "range_noise", &object.id, 0))
                    .max(0.0),
                    bearing_rad: wrap_angle(
                        geometry.bearing_rad
                            + config.bearing_noise_stddev_rad
                                * random.normal_named(
                                    "lidar",
                                    sequence,
                                    "bearing_noise",
                                    &object.id,
                                    0,
                                ),
                    ),
                    range_variance_m2: config.range_noise_stddev_m.powi(2),
                    bearing_variance_rad2: config.bearing_noise_stddev_rad.powi(2),
                });
            }

            let arrival_ns = scan_end_ns + config.latency_ns;
            output.push(PendingMeasurement {
                arrival_ns,
                priority: DELIVERY_PRIORITY,
                stable_event_id: format!("lidar:{sequence:010}"),
                measurement: MeasurementRecord::Lidar(LidarScan {
                    time: Some(measurement_time(scan_end_ns, arrival_ns)),
                    detections,
                }),
                imu_bias_truth: None,
            });
        },
    );
}
