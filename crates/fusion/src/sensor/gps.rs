use fusion_schema::messages::{GpsFix, Vec3, observation_truth};

use crate::{
    bundle::MeasurementRecord, random::DeterministicRandom, scenario::ResolvedScenario,
    truth::Trajectory,
};

use super::{PendingMeasurement, build_observation_truth, for_each_sample, record_header};

const DELIVERY_PRIORITY: u8 = 15;

pub(super) fn generate(
    scenario: &ResolvedScenario,
    trajectory: &Trajectory,
    random: &DeterministicRandom,
    output: &mut Vec<PendingMeasurement>,
) {
    let config = &scenario.gps;
    for_each_sample(
        scenario.effective_duration_s(),
        config.rate_hz,
        |time_ns, sequence| {
            let platform = trajectory.sample_ns(time_ns);
            let mount = &config.mount;
            let antenna_x = platform.x_world_m
                + platform.yaw_world_from_body_rad.cos() * mount.position_m.x
                - platform.yaw_world_from_body_rad.sin() * mount.position_m.y;
            let antenna_y = platform.y_world_m
                + platform.yaw_world_from_body_rad.sin() * mount.position_m.x
                + platform.yaw_world_from_body_rad.cos() * mount.position_m.y;
            let ideal_position = Vec3 {
                x: antenna_x,
                y: antenna_y,
                z: mount.position_m.z,
            };
            let measured_position = Vec3 {
                x: ideal_position.x
                    + config.horizontal_position_stddev_m
                        * random.normal(&config.instance_id, sequence, "position_noise", 0),
                y: ideal_position.y
                    + config.horizontal_position_stddev_m
                        * random.normal(&config.instance_id, sequence, "position_noise", 1),
                z: ideal_position.z
                    + config.vertical_position_stddev_m
                        * random.normal(&config.instance_id, sequence, "position_noise", 2),
            };
            let covariance = vec![
                config.horizontal_position_stddev_m.powi(2),
                0.0,
                0.0,
                0.0,
                config.horizontal_position_stddev_m.powi(2),
                0.0,
                0.0,
                0.0,
                config.vertical_position_stddev_m.powi(2),
            ];
            let record_id = format!("{}:{sequence:010}", config.instance_id);
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
            let ideal = GpsFix {
                header: Some(header.clone()),
                position_world_m: Some(ideal_position),
                position_covariance: covariance.clone(),
                frame_id: scenario.platform.world_frame.clone(),
                altitude_valid: config.altitude_valid,
            };
            let visible = GpsFix {
                header: Some(header),
                position_world_m: Some(measured_position),
                position_covariance: covariance,
                frame_id: scenario.platform.world_frame.clone(),
                altitude_valid: config.altitude_valid,
            };
            output.push(PendingMeasurement {
                arrival_ns,
                priority: DELIVERY_PRIORITY,
                stable_event_id: record_id.clone(),
                measurement: MeasurementRecord::Gps(visible),
                observation_truth: Some(build_observation_truth(
                    record_id,
                    time_ns,
                    time_ns,
                    arrival_ns,
                    observation_truth::IdealObservation::IdealGps(ideal),
                    None,
                    Vec::new(),
                )),
            });
        },
    );
}
