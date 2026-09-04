mod camera;
mod geometry;
mod gps;
mod imu;
mod lidar;

use anyhow::Result;
use fusion_schema::messages::{ImuBiasTruth, MeasurementTime};

use crate::{
    bundle::{GeneratedRun, MeasurementRecord},
    random::DeterministicRandom,
    scenario::ResolvedScenario,
    truth::{Trajectory, object_truth_states},
};

#[derive(Debug)]
pub(super) struct PendingMeasurement {
    pub(super) arrival_ns: i64,
    pub(super) priority: u8,
    pub(super) stable_event_id: String,
    pub(super) measurement: MeasurementRecord,
    pub(super) imu_bias_truth: Option<ImuBiasTruth>,
}

pub fn generate(scenario: &ResolvedScenario) -> Result<GeneratedRun> {
    let trajectory = Trajectory::new(scenario)?;
    let random = DeterministicRandom::new(scenario.root_seed);
    let mut pending = Vec::new();

    if scenario.imu.enabled {
        imu::generate(scenario, &trajectory, &random, &mut pending);
    }
    if scenario.gps.enabled {
        gps::generate(scenario, &trajectory, &random, &mut pending);
    }
    if scenario.camera.enabled {
        camera::generate(scenario, &trajectory, &random, &mut pending);
    }
    if scenario.lidar.enabled {
        lidar::generate(scenario, &trajectory, &random, &mut pending);
    }

    pending.sort_by(|left, right| {
        left.arrival_ns
            .cmp(&right.arrival_ns)
            .then(left.priority.cmp(&right.priority))
            .then(left.stable_event_id.cmp(&right.stable_event_id))
    });

    let mut measurements = Vec::with_capacity(pending.len());
    let mut imu_bias_truth = Vec::new();
    for event in pending {
        measurements.push(event.measurement);
        if let Some(truth) = event.imu_bias_truth {
            imu_bias_truth.push(truth);
        }
    }
    imu_bias_truth.sort_by_key(|truth| truth.time_ns);

    let truth_period_ns = period_ns(scenario.imu.rate_hz);
    let end_ns = seconds_to_ns(scenario.effective_duration_s());
    let mut ego_truth_states = vec![trajectory.truth_state(0)];
    let mut time_ns = truth_period_ns;
    while time_ns <= end_ns {
        ego_truth_states.push(trajectory.truth_state(time_ns));
        time_ns += truth_period_ns;
    }

    Ok(GeneratedRun {
        measurements,
        ego_truth_states,
        object_truth_states: object_truth_states(scenario, truth_period_ns),
        imu_bias_truth,
    })
}

pub(super) fn measurement_time(measurement_time_ns: i64, arrival_time_ns: i64) -> MeasurementTime {
    MeasurementTime {
        measurement_time_ns,
        arrival_time_ns,
    }
}

pub(super) fn for_each_sample(duration_s: f64, rate_hz: f64, mut callback: impl FnMut(i64, u64)) {
    let end_ns = seconds_to_ns(duration_s);
    let step_ns = period_ns(rate_hz);
    let mut time_ns = step_ns;
    let mut sequence = 0;
    while time_ns <= end_ns {
        callback(time_ns, sequence);
        time_ns += step_ns;
        sequence += 1;
    }
}

pub(super) fn period_ns(rate_hz: f64) -> i64 {
    (1.0e9 / rate_hz).round() as i64
}

pub(super) fn seconds_to_ns(seconds: f64) -> i64 {
    (seconds * 1.0e9).round() as i64
}
