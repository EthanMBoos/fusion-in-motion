use std::collections::BTreeMap;

use fusion_schema::messages::{
    EgoStateEstimate, EgoTruthState, ImuBiasTruth, ObjectTrack, ObjectTruthState,
};
use serde::{Deserialize, Serialize};

use crate::{math, scenario::ResolvedScenario, tracker::EgoSource};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EgoMetrics {
    pub estimate_samples: usize,
    pub matched_samples: usize,
    pub position_rmse_m: f64,
    pub yaw_rmse_rad: f64,
    pub final_position_error_m: f64,
    pub maximum_position_error_m: f64,
    pub time_coverage_fraction: f64,
    pub invalid_output_count: usize,
    pub position_threshold_exceeded_count: usize,
    pub gyro_bias_rmse_radps: Option<f64>,
    pub accel_bias_rmse_mps2: Option<f64>,
    pub gyro_bias_95pct_coverage: Option<f64>,
    pub accel_bias_95pct_coverage: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackMetrics {
    pub ego_source: String,
    pub track_samples: usize,
    pub matched_samples: usize,
    pub track_count: usize,
    pub position_rmse_m: Option<f64>,
    pub velocity_rmse_mps: Option<f64>,
    pub relative_position_rmse_m: Option<f64>,
    pub final_position_error_m: Option<f64>,
    pub maximum_position_error_m: Option<f64>,
    pub time_coverage_fraction: f64,
    pub invalid_output_count: usize,
    pub position_threshold_exceeded_count: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RunMetrics {
    pub metric_version: String,
    pub ego: EgoMetrics,
    pub tracks_with_estimated_ego: TrackMetrics,
    pub tracks_with_truth_ego: TrackMetrics,
    pub estimated_ego_position_rmse_delta_m: Option<f64>,
}

pub fn evaluate(
    scenario: &ResolvedScenario,
    ego_truth: &[EgoTruthState],
    object_truth: &[ObjectTruthState],
    imu_bias_truth: &[ImuBiasTruth],
    estimates: &[EgoStateEstimate],
    estimated_ego_tracks: &[ObjectTrack],
    truth_ego_tracks: &[ObjectTrack],
) -> RunMetrics {
    let ego = evaluate_ego(scenario, ego_truth, imu_bias_truth, estimates);
    let tracks_with_estimated_ego = evaluate_tracks(
        scenario,
        ego_truth,
        object_truth,
        estimates,
        estimated_ego_tracks,
        EgoSource::Estimated,
    );
    let tracks_with_truth_ego = evaluate_tracks(
        scenario,
        ego_truth,
        object_truth,
        estimates,
        truth_ego_tracks,
        EgoSource::Truth,
    );
    RunMetrics {
        metric_version: "fusion-eval-3.0".to_owned(),
        estimated_ego_position_rmse_delta_m: tracks_with_estimated_ego
            .position_rmse_m
            .zip(tracks_with_truth_ego.position_rmse_m)
            .map(|(estimated, truth)| estimated - truth),
        ego,
        tracks_with_estimated_ego,
        tracks_with_truth_ego,
    }
}

pub fn evaluate_ego(
    scenario: &ResolvedScenario,
    truth: &[EgoTruthState],
    bias_truth: &[ImuBiasTruth],
    estimates: &[EgoStateEstimate],
) -> EgoMetrics {
    let mut position_squared = 0.0;
    let mut yaw_squared = 0.0;
    let mut matched = 0;
    let mut final_error = 0.0;
    let mut maximum_error: f64 = 0.0;
    let mut invalid = 0;
    let mut threshold_exceeded = 0;
    let mut first_time = None;
    let mut last_time = None;
    let mut gyro_bias_squared = 0.0;
    let mut accel_bias_squared = 0.0;
    let mut bias_samples = 0;
    let mut bias_covariance_samples = 0;
    let mut gyro_covered = 0;
    let mut accel_covered = 0;

    for estimate in estimates {
        let (Some(estimate_pose), Some(truth_state)) = (
            estimate.pose_world.as_ref(),
            nearest_ego_truth(
                truth,
                estimate.estimate_time_ns,
                scenario.metrics.max_truth_match_gap_ns,
            ),
        ) else {
            invalid += 1;
            continue;
        };
        let (Some(estimate_position), Some(truth_pose), Some(truth_position)) = (
            estimate_pose.position.as_ref(),
            truth_state.pose_world.as_ref(),
            truth_state
                .pose_world
                .as_ref()
                .and_then(|pose| pose.position.as_ref()),
        ) else {
            invalid += 1;
            continue;
        };
        let error =
            (estimate_position.x - truth_position.x).hypot(estimate_position.y - truth_position.y);
        let yaw_error = math::wrap_angle(estimate_pose.yaw_rad - truth_pose.yaw_rad);
        position_squared += error * error;
        yaw_squared += yaw_error * yaw_error;
        final_error = error;
        maximum_error = maximum_error.max(error);
        threshold_exceeded += usize::from(error > scenario.metrics.ego_divergence_position_error_m);
        matched += 1;
        first_time.get_or_insert(estimate.estimate_time_ns);
        last_time = Some(estimate.estimate_time_ns);

        if let (Some(gyro_estimate), Some(accel_estimate), Some(bias)) = (
            estimate.gyro_bias_z_radps,
            estimate.accel_bias_x_mps2,
            nearest_by_time(
                bias_truth,
                estimate.estimate_time_ns,
                scenario.metrics.max_truth_match_gap_ns,
                |bias| bias.time_ns,
            ),
        ) {
            let gyro_error = gyro_estimate - bias.gyro_bias_z_radps;
            let accel_error = accel_estimate - bias.accel_bias_x_mps2;
            gyro_bias_squared += gyro_error * gyro_error;
            accel_bias_squared += accel_error * accel_error;
            let covariance = &estimate.state_covariance;
            if covariance.len() == 36 {
                let gyro_variance = covariance[4 * 6 + 4];
                let accel_variance = covariance[5 * 6 + 5];
                if gyro_variance.is_finite()
                    && accel_variance.is_finite()
                    && gyro_variance >= 0.0
                    && accel_variance >= 0.0
                {
                    gyro_covered += usize::from(gyro_error.abs() <= 1.96 * gyro_variance.sqrt());
                    accel_covered += usize::from(accel_error.abs() <= 1.96 * accel_variance.sqrt());
                    bias_covariance_samples += 1;
                }
            }
            bias_samples += 1;
        }
    }
    let duration_ns = truth.last().map_or(0, |state| state.time_ns)
        - truth.first().map_or(0, |state| state.time_ns);
    EgoMetrics {
        estimate_samples: estimates.len(),
        matched_samples: matched,
        position_rmse_m: rms(position_squared, matched),
        yaw_rmse_rad: rms(yaw_squared, matched),
        final_position_error_m: final_error,
        maximum_position_error_m: maximum_error,
        time_coverage_fraction: coverage(first_time, last_time, duration_ns),
        invalid_output_count: invalid,
        position_threshold_exceeded_count: threshold_exceeded,
        gyro_bias_rmse_radps: (bias_samples > 0).then(|| rms(gyro_bias_squared, bias_samples)),
        accel_bias_rmse_mps2: (bias_samples > 0).then(|| rms(accel_bias_squared, bias_samples)),
        gyro_bias_95pct_coverage: (bias_covariance_samples > 0)
            .then(|| gyro_covered as f64 / bias_covariance_samples as f64),
        accel_bias_95pct_coverage: (bias_covariance_samples > 0)
            .then(|| accel_covered as f64 / bias_covariance_samples as f64),
    }
}

pub fn evaluate_tracks(
    scenario: &ResolvedScenario,
    ego_truth: &[EgoTruthState],
    object_truth: &[ObjectTruthState],
    ego_estimates: &[EgoStateEstimate],
    tracks: &[ObjectTrack],
    ego_source: EgoSource,
) -> TrackMetrics {
    let mut position_squared = 0.0;
    let mut velocity_squared = 0.0;
    let mut relative_squared = 0.0;
    let mut relative_matched = 0;
    let mut matched = 0;
    let mut maximum_error: f64 = 0.0;
    let mut threshold_exceeded = 0;
    let mut objects = BTreeMap::<String, (i64, i64, f64)>::new();
    let truth_assignments = track_truth_assignments(
        tracks,
        object_truth,
        scenario.metrics.max_truth_match_gap_ns,
    );
    for track in tracks {
        let Some(truth_key) = truth_assignments.get(&track.track_id) else {
            continue;
        };
        let Some(truth) = nearest_object_truth(
            object_truth,
            truth_key,
            track.estimate_time_ns,
            scenario.metrics.max_truth_match_gap_ns,
        ) else {
            continue;
        };
        let (Some(position), Some(velocity), Some(truth_position), Some(truth_velocity)) = (
            track.position_world_m.as_ref(),
            track.velocity_world_mps.as_ref(),
            truth.position_world_m.as_ref(),
            truth.velocity_world_mps.as_ref(),
        ) else {
            continue;
        };
        let position_error = (position.x - truth_position.x).hypot(position.y - truth_position.y);
        let velocity_error = (velocity.x - truth_velocity.x).hypot(velocity.y - truth_velocity.y);
        position_squared += position_error.powi(2);
        velocity_squared += velocity_error.powi(2);
        maximum_error = maximum_error.max(position_error);
        threshold_exceeded +=
            usize::from(position_error > scenario.metrics.track_divergence_position_error_m);
        matched += 1;
        objects
            .entry(track.track_id.clone())
            .and_modify(|range| {
                if track.estimate_time_ns >= range.1 {
                    range.1 = track.estimate_time_ns;
                    range.2 = position_error;
                }
            })
            .or_insert((
                track.estimate_time_ns,
                track.estimate_time_ns,
                position_error,
            ));

        if let Some(relative_error) = relative_position_error(
            track,
            truth,
            ego_truth,
            ego_estimates,
            ego_source,
            scenario.metrics.max_truth_match_gap_ns,
        ) {
            relative_squared += relative_error.powi(2);
            relative_matched += 1;
        }
    }
    let truth_duration = ego_truth.last().map_or(0, |state| state.time_ns)
        - ego_truth.first().map_or(0, |state| state.time_ns);
    let time_coverage_fraction = if objects.is_empty() || truth_duration <= 0 {
        0.0
    } else {
        objects
            .values()
            .map(|(first, last, _)| (last - first) as f64 / truth_duration as f64)
            .sum::<f64>()
            / objects.len() as f64
    };
    let final_position_error_m = (!objects.is_empty()).then(|| {
        (objects
            .values()
            .map(|(_, _, final_error)| final_error.powi(2))
            .sum::<f64>()
            / objects.len() as f64)
            .sqrt()
    });
    TrackMetrics {
        ego_source: ego_source.label().to_owned(),
        track_samples: tracks.len(),
        matched_samples: matched,
        track_count: objects.len(),
        position_rmse_m: (matched > 0).then(|| rms(position_squared, matched)),
        velocity_rmse_mps: (matched > 0).then(|| rms(velocity_squared, matched)),
        relative_position_rmse_m: (relative_matched > 0)
            .then(|| rms(relative_squared, relative_matched)),
        final_position_error_m,
        maximum_position_error_m: (matched > 0).then_some(maximum_error),
        time_coverage_fraction,
        invalid_output_count: tracks.len() - matched,
        position_threshold_exceeded_count: threshold_exceeded,
    }
}

pub(crate) fn track_truth_assignments(
    tracks: &[ObjectTrack],
    truth: &[ObjectTruthState],
    max_gap_ns: i64,
) -> BTreeMap<String, String> {
    let track_ids = tracks
        .iter()
        .map(|track| track.track_id.clone())
        .collect::<std::collections::BTreeSet<_>>()
        .into_iter()
        .collect::<Vec<_>>();
    let truth_ids = truth
        .iter()
        .map(|state| state.track_key.clone())
        .collect::<std::collections::BTreeSet<_>>()
        .into_iter()
        .collect::<Vec<_>>();
    if track_ids.is_empty() || truth_ids.is_empty() {
        return BTreeMap::new();
    }

    let unmatched_cost = 1.0e12;
    let invalid_cost = 1.0e18;
    let mut costs = Vec::with_capacity(track_ids.len());
    for track_id in &track_ids {
        let samples = tracks
            .iter()
            .filter(|track| &track.track_id == track_id)
            .collect::<Vec<_>>();
        let mut row = Vec::with_capacity(truth_ids.len() + track_ids.len());
        for truth_id in &truth_ids {
            let mut squared = 0.0;
            let mut matched = 0;
            for track in &samples {
                let (Some(position), Some(state)) = (
                    track.position_world_m.as_ref(),
                    nearest_object_truth(truth, truth_id, track.estimate_time_ns, max_gap_ns),
                ) else {
                    continue;
                };
                let Some(truth_position) = state.position_world_m.as_ref() else {
                    continue;
                };
                squared += (position.x - truth_position.x).powi(2)
                    + (position.y - truth_position.y).powi(2);
                matched += 1;
            }
            row.push(if matched == 0 {
                invalid_cost
            } else {
                squared / matched as f64
            });
        }
        row.extend(std::iter::repeat_n(unmatched_cost, track_ids.len()));
        costs.push(row);
    }

    math::minimum_cost_assignment(&costs)
        .into_iter()
        .enumerate()
        .filter(|(track_index, truth_index)| {
            *truth_index < truth_ids.len() && costs[*track_index][*truth_index] < unmatched_cost
        })
        .map(|(track_index, truth_index)| {
            (
                track_ids[track_index].clone(),
                truth_ids[truth_index].clone(),
            )
        })
        .collect()
}

fn relative_position_error(
    track: &ObjectTrack,
    object_truth: &ObjectTruthState,
    ego_truth: &[EgoTruthState],
    ego_estimates: &[EgoStateEstimate],
    ego_source: EgoSource,
    max_gap_ns: i64,
) -> Option<f64> {
    let track_position = track.position_world_m.as_ref()?;
    let object_position = object_truth.position_world_m.as_ref()?;
    let truth_ego = nearest_ego_truth(ego_truth, track.estimate_time_ns, max_gap_ns)?;
    let truth_pose = truth_ego.pose_world.as_ref()?;
    let truth_ego_position = truth_pose.position.as_ref()?;
    let (ego_x, ego_y, ego_yaw) = match ego_source {
        EgoSource::Truth => (
            truth_ego_position.x,
            truth_ego_position.y,
            truth_pose.yaw_rad,
        ),
        EgoSource::Estimated => {
            let estimate = nearest_estimate(ego_estimates, track.estimate_time_ns, max_gap_ns)?;
            let pose = estimate.pose_world.as_ref()?;
            let position = pose.position.as_ref()?;
            (position.x, position.y, pose.yaw_rad)
        }
    };
    let estimated_relative =
        rotate_into_body(track_position.x - ego_x, track_position.y - ego_y, ego_yaw);
    let truth_relative = rotate_into_body(
        object_position.x - truth_ego_position.x,
        object_position.y - truth_ego_position.y,
        truth_pose.yaw_rad,
    );
    Some((estimated_relative.0 - truth_relative.0).hypot(estimated_relative.1 - truth_relative.1))
}

fn rotate_into_body(x: f64, y: f64, yaw: f64) -> (f64, f64) {
    (
        yaw.cos() * x + yaw.sin() * y,
        -yaw.sin() * x + yaw.cos() * y,
    )
}

fn nearest_ego_truth(
    truth: &[EgoTruthState],
    time_ns: i64,
    max_gap_ns: i64,
) -> Option<&EgoTruthState> {
    nearest_by_time(truth, time_ns, max_gap_ns, |state| state.time_ns)
}

fn nearest_estimate(
    estimates: &[EgoStateEstimate],
    time_ns: i64,
    max_gap_ns: i64,
) -> Option<&EgoStateEstimate> {
    nearest_by_time(estimates, time_ns, max_gap_ns, |estimate| {
        estimate.estimate_time_ns
    })
}

fn nearest_object_truth<'a>(
    truth: &'a [ObjectTruthState],
    track_key: &str,
    time_ns: i64,
    max_gap_ns: i64,
) -> Option<&'a ObjectTruthState> {
    truth
        .iter()
        .filter(|state| state.track_key == track_key)
        .min_by_key(|state| state.time_ns.abs_diff(time_ns))
        .filter(|state| state.time_ns.abs_diff(time_ns) <= max_gap_ns as u64)
}

fn nearest_by_time<T>(
    values: &[T],
    time_ns: i64,
    max_gap_ns: i64,
    time: impl Fn(&T) -> i64,
) -> Option<&T> {
    values
        .iter()
        .min_by_key(|value| time(value).abs_diff(time_ns))
        .filter(|value| time(value).abs_diff(time_ns) <= max_gap_ns as u64)
}

fn rms(sum_squared: f64, count: usize) -> f64 {
    if count == 0 {
        0.0
    } else {
        (sum_squared / count as f64).sqrt()
    }
}

fn coverage(first: Option<i64>, last: Option<i64>, duration_ns: i64) -> f64 {
    match (first, last) {
        (Some(first), Some(last)) if duration_ns > 0 => {
            ((last - first) as f64 / duration_ns as f64).clamp(0.0, 1.0)
        }
        _ => 0.0,
    }
}
