mod camera;
mod geometry;
mod imu;
mod lidar;
mod radar;

use anyhow::Result;
use fusion_schema::messages::{
    Landmark, LandmarkMap, ObservationTruth, RecordHeader, StampReference, Vec3, observation_truth,
};

use crate::{
    bundle::{GeneratedRun, MeasurementRecord},
    random::DeterministicRandom,
    scenario::ResolvedScenario,
    truth::Trajectory,
};

const MAP_PRIORITY: u8 = 0;

#[derive(Debug)]
pub(super) struct PendingMeasurement {
    pub(super) arrival_ns: i64,
    pub(super) priority: u8,
    pub(super) stable_event_id: String,
    pub(super) measurement: MeasurementRecord,
    pub(super) observation_truth: Option<ObservationTruth>,
}

pub fn generate(scenario: &ResolvedScenario) -> Result<GeneratedRun> {
    let trajectory = Trajectory::new(scenario)?;
    let random = DeterministicRandom::new(scenario.root_seed);
    let mut pending = vec![map_event(scenario)];

    imu::generate(scenario, &trajectory, &random, &mut pending);
    camera::generate(scenario, &trajectory, &random, &mut pending);
    lidar::generate(scenario, &trajectory, &random, &mut pending);
    radar::generate(scenario, &trajectory, &random, &mut pending);

    // Receipt time is the estimator-visible order. Priority and stable ID make
    // simultaneous arrivals deterministic without depending on insertion order.
    pending.sort_by(|left, right| {
        left.arrival_ns
            .cmp(&right.arrival_ns)
            .then(left.priority.cmp(&right.priority))
            .then(left.stable_event_id.cmp(&right.stable_event_id))
    });

    let mut measurements = Vec::with_capacity(pending.len());
    let mut observation_truth = Vec::with_capacity(pending.len().saturating_sub(1));
    for (delivery_index, mut event) in pending.into_iter().enumerate() {
        event.measurement.header_mut().delivery_index = delivery_index as u64;
        measurements.push(event.measurement);
        if let Some(truth) = event.observation_truth {
            observation_truth.push(truth);
        }
    }

    let imu_period_ns = period_ns(scenario.imu.rate_hz);
    let end_ns = seconds_to_ns(scenario.effective_duration_s());
    let mut truth_states = vec![trajectory.truth_state(0)];
    let mut time_ns = imu_period_ns;
    while time_ns <= end_ns {
        truth_states.push(trajectory.truth_state(time_ns));
        time_ns += imu_period_ns;
    }

    Ok(GeneratedRun {
        measurements,
        truth_states,
        observation_truth,
    })
}

fn map_event(scenario: &ResolvedScenario) -> PendingMeasurement {
    let record_id = "map.reference:0000000000".to_owned();
    let header = record_header(scenario, &record_id, "map.reference", 0, 0, 0, 0);
    let landmarks = scenario
        .world
        .landmarks
        .iter()
        .map(|landmark| Landmark {
            id: landmark.id.clone(),
            position_world_m: Some(Vec3 {
                x: landmark.x_m,
                y: landmark.y_m,
                z: landmark.z_m,
            }),
        })
        .collect();

    PendingMeasurement {
        arrival_ns: 0,
        priority: MAP_PRIORITY,
        stable_event_id: record_id,
        measurement: MeasurementRecord::Map(LandmarkMap {
            header: Some(header),
            frame_id: scenario.platform.world_frame.clone(),
            landmarks,
        }),
        observation_truth: None,
    }
}

pub(super) fn build_observation_truth(
    record_id: String,
    acquisition_start_truth_ns: i64,
    acquisition_end_truth_ns: i64,
    arrival_truth_ns: i64,
    effect_values: serde_json::Value,
    ideal_observation: observation_truth::IdealObservation,
) -> ObservationTruth {
    ObservationTruth {
        visible_record_id: record_id,
        acquisition_start_truth_ns,
        acquisition_end_truth_ns,
        publish_truth_ns: arrival_truth_ns,
        arrival_truth_ns,
        effect_values_json: effect_values.to_string(),
        ideal_observation: Some(ideal_observation),
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

#[allow(clippy::too_many_arguments)]
pub(super) fn record_header(
    scenario: &ResolvedScenario,
    record_id: &str,
    sensor_instance_id: &str,
    reported_stamp_ns: i64,
    acquisition_duration_ns: i64,
    receipt_time_ns: i64,
    sensor_sequence: u64,
) -> RecordHeader {
    RecordHeader {
        format_version: 1,
        run_id: scenario.run_id.clone(),
        record_id: record_id.to_owned(),
        sensor_instance_id: sensor_instance_id.to_owned(),
        reported_stamp_ns,
        stamp_reference: StampReference::End as i32,
        acquisition_duration_ns,
        receipt_time_ns,
        sensor_sequence,
        delivery_index: 0,
        valid: true,
        quality_flags: Vec::new(),
    }
}

pub(super) fn period_ns(rate_hz: f64) -> i64 {
    (1.0e9 / rate_hz).round() as i64
}

pub(super) fn seconds_to_ns(seconds: f64) -> i64 {
    (seconds * 1.0e9).round() as i64
}

pub(super) fn to_proto(vector: nalgebra::Vector3<f64>) -> Vec3 {
    Vec3 {
        x: vector.x,
        y: vector.y,
        z: vector.z,
    }
}
