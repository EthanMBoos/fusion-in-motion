use fusion_schema::messages::{CameraDetection, CameraFrame};

use crate::{
    bundle::MeasurementRecord, math::wrap_angle, random::DeterministicRandom,
    scenario::ResolvedScenario, truth::Trajectory,
};

use super::{
    PendingMeasurement, for_each_sample,
    geometry::{is_visible, object_geometry},
    measurement_time,
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
            let mut detections = Vec::new();
            for object in &scenario.world.objects {
                let Some(geometry) = object_geometry(platform, object) else {
                    continue;
                };
                if !is_visible(geometry, config.horizontal_fov_rad, config.max_range_m)
                    || random.uniform_named("camera", sequence, "detection", &object.id, 0)
                        > config.detection_probability
                {
                    continue;
                }
                detections.push(CameraDetection {
                    bearing_rad: wrap_angle(
                        geometry.bearing_rad
                            + config.bearing_noise_stddev_rad
                                * random.normal_named(
                                    "camera",
                                    sequence,
                                    "bearing_noise",
                                    &object.id,
                                    0,
                                ),
                    ),
                    bearing_variance_rad2: config.bearing_noise_stddev_rad.powi(2),
                });
            }
            let arrival_ns = time_ns + config.latency_ns;
            output.push(PendingMeasurement {
                arrival_ns,
                priority: DELIVERY_PRIORITY,
                stable_event_id: format!("camera:{sequence:010}"),
                measurement: MeasurementRecord::Camera(CameraFrame {
                    time: Some(measurement_time(time_ns, arrival_ns)),
                    detections,
                }),
                imu_bias_truth: None,
            });
        },
    );
}
