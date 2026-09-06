use std::collections::BTreeMap;

use fusion_schema::messages::{
    EgoStateEstimate, EgoTruthState, ImuBiasTruth, ObjectTrack, ObjectTrackFrame, ObjectTruthState,
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
    pub missed_object_samples: usize,
    pub false_track_samples: usize,
    pub identity_switch_count: usize,
    pub track_fragment_count: usize,
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
    estimated_ego_tracks: &[ObjectTrackFrame],
    truth_ego_tracks: &[ObjectTrackFrame],
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
        metric_version: "fusion-eval-4.0".to_owned(),
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
    frames: &[ObjectTrackFrame],
    ego_source: EgoSource,
) -> TrackMetrics {
    let mut position_squared = 0.0;
    let mut velocity_squared = 0.0;
    let mut relative_squared = 0.0;
    let mut relative_matched = 0;
    let mut matched = 0;
    let mut maximum_error: f64 = 0.0;
    let mut threshold_exceeded = 0;
    let mut track_ids = std::collections::BTreeSet::new();
    let mut invalid = 0;
    let mut missed_object_samples = 0;
    let mut false_track_samples = 0;
    let mut truth_sample_count = 0;
    let mut latest_errors = BTreeMap::<String, (i64, f64)>::new();
    let mut continuity = BTreeMap::<String, TruthContinuity>::new();
    let mut identity_switch_count = 0;
    let mut track_fragment_count = 0;

    for frame in frames {
        track_ids.extend(frame.tracks.iter().map(|track| track.track_id.clone()));
        let truth_at_time = object_truth_at_time(
            object_truth,
            frame.estimate_time_ns,
            scenario.metrics.max_truth_match_gap_ns,
        );
        if truth_at_time.is_empty() {
            invalid += frame.tracks.len();
            continue;
        }
        let valid_tracks = frame
            .tracks
            .iter()
            .enumerate()
            .filter(|(_, track)| valid_track(track))
            .collect::<Vec<_>>();
        invalid += frame.tracks.len() - valid_tracks.len();
        let assignments = frame_truth_assignments(
            frame,
            object_truth,
            scenario.metrics.max_truth_match_gap_ns,
            scenario.metrics.track_truth_match_max_distance_m,
        );
        false_track_samples += valid_tracks.len() - assignments.len();
        missed_object_samples += truth_at_time.len() - assignments.len();
        truth_sample_count += truth_at_time.len();

        let matched_truth = assignments
            .values()
            .map(|truth_key| truth_key.as_str())
            .collect::<std::collections::BTreeSet<_>>();
        for truth_key in truth_at_time.keys() {
            let state = continuity.entry(truth_key.clone()).or_default();
            if !matched_truth.contains(truth_key.as_str()) {
                state.matched_previous_frame = false;
            }
        }

        for (track_index, truth_key) in assignments {
            let track = &frame.tracks[track_index];
            let truth = truth_at_time[&truth_key];
            let position = track.position_world_m.as_ref().expect("validated track");
            let velocity = track.velocity_world_mps.as_ref().expect("validated track");
            let truth_position = truth
                .position_world_m
                .as_ref()
                .expect("matched truth has position");
            let truth_velocity = truth
                .velocity_world_mps
                .as_ref()
                .expect("matched truth has velocity");
            let position_error =
                (position.x - truth_position.x).hypot(position.y - truth_position.y);
            let velocity_error =
                (velocity.x - truth_velocity.x).hypot(velocity.y - truth_velocity.y);
            position_squared += position_error.powi(2);
            velocity_squared += velocity_error.powi(2);
            maximum_error = maximum_error.max(position_error);
            threshold_exceeded +=
                usize::from(position_error > scenario.metrics.track_divergence_position_error_m);
            matched += 1;
            latest_errors
                .entry(truth_key.clone())
                .and_modify(|latest| {
                    if frame.estimate_time_ns >= latest.0 {
                        *latest = (frame.estimate_time_ns, position_error);
                    }
                })
                .or_insert((frame.estimate_time_ns, position_error));

            let state = continuity.entry(truth_key).or_default();
            if state.matched_previous_frame
                && state.last_track_id.as_deref() != Some(track.track_id.as_str())
            {
                identity_switch_count += 1;
            }
            if state.ever_matched && !state.matched_previous_frame {
                track_fragment_count += 1;
            }
            state.ever_matched = true;
            state.matched_previous_frame = true;
            state.last_track_id = Some(track.track_id.clone());

            if let Some(relative_error) = relative_position_error(
                track,
                truth,
                frame.estimate_time_ns,
                ego_truth,
                ego_estimates,
                ego_source,
                scenario.metrics.max_truth_match_gap_ns,
            ) {
                relative_squared += relative_error.powi(2);
                relative_matched += 1;
            }
        }
    }

    let final_position_error_m = (!latest_errors.is_empty()).then(|| {
        rms(
            latest_errors.values().map(|(_, error)| error.powi(2)).sum(),
            latest_errors.len(),
        )
    });
    TrackMetrics {
        ego_source: ego_source.label().to_owned(),
        track_samples: frames.iter().map(|frame| frame.tracks.len()).sum(),
        matched_samples: matched,
        track_count: track_ids.len(),
        position_rmse_m: (matched > 0).then(|| rms(position_squared, matched)),
        velocity_rmse_mps: (matched > 0).then(|| rms(velocity_squared, matched)),
        relative_position_rmse_m: (relative_matched > 0)
            .then(|| rms(relative_squared, relative_matched)),
        final_position_error_m,
        maximum_position_error_m: (matched > 0).then_some(maximum_error),
        time_coverage_fraction: if truth_sample_count > 0 {
            matched as f64 / truth_sample_count as f64
        } else {
            0.0
        },
        invalid_output_count: invalid,
        position_threshold_exceeded_count: threshold_exceeded,
        missed_object_samples,
        false_track_samples,
        identity_switch_count,
        track_fragment_count,
    }
}

#[derive(Default)]
struct TruthContinuity {
    last_track_id: Option<String>,
    ever_matched: bool,
    matched_previous_frame: bool,
}

pub(crate) fn frame_truth_assignments(
    frame: &ObjectTrackFrame,
    truth: &[ObjectTruthState],
    max_gap_ns: i64,
    max_distance_m: f64,
) -> BTreeMap<usize, String> {
    let track_indices = frame
        .tracks
        .iter()
        .enumerate()
        .filter(|(_, track)| valid_track(track))
        .map(|(index, _)| index)
        .collect::<Vec<_>>();
    let truth_at_time = object_truth_at_time(truth, frame.estimate_time_ns, max_gap_ns);
    let truth_ids = truth_at_time.keys().cloned().collect::<Vec<_>>();
    if track_indices.is_empty() || truth_ids.is_empty() {
        return BTreeMap::new();
    }

    let max_match_cost = max_distance_m.powi(2);
    let unmatched_cost = max_match_cost + 1.0;
    let invalid_cost = unmatched_cost * 1.0e6;
    let mut costs = Vec::with_capacity(track_indices.len());
    for &track_index in &track_indices {
        let track = &frame.tracks[track_index];
        let position = track.position_world_m.as_ref().expect("validated track");
        let mut row = Vec::with_capacity(truth_ids.len() + track_indices.len());
        for truth_id in &truth_ids {
            let truth_position = truth_at_time[truth_id]
                .position_world_m
                .as_ref()
                .expect("matched truth has position");
            let squared =
                (position.x - truth_position.x).powi(2) + (position.y - truth_position.y).powi(2);
            row.push(if squared <= max_match_cost {
                squared
            } else {
                invalid_cost
            });
        }
        row.extend(std::iter::repeat_n(unmatched_cost, track_indices.len()));
        costs.push(row);
    }

    math::minimum_cost_assignment(&costs)
        .into_iter()
        .enumerate()
        .filter(|(track_index, truth_index)| {
            *truth_index < truth_ids.len() && costs[*track_index][*truth_index] <= max_match_cost
        })
        .map(|(track_index, truth_index)| {
            (track_indices[track_index], truth_ids[truth_index].clone())
        })
        .collect()
}

fn object_truth_at_time(
    truth: &[ObjectTruthState],
    time_ns: i64,
    max_gap_ns: i64,
) -> BTreeMap<String, &ObjectTruthState> {
    truth
        .iter()
        .map(|state| state.track_key.as_str())
        .collect::<std::collections::BTreeSet<_>>()
        .into_iter()
        .filter_map(|track_key| {
            let state = nearest_object_truth(truth, track_key, time_ns, max_gap_ns)?;
            state
                .position_world_m
                .as_ref()
                .zip(state.velocity_world_mps.as_ref())?;
            Some((track_key.to_owned(), state))
        })
        .collect()
}

fn valid_track(track: &ObjectTrack) -> bool {
    let (Some(position), Some(velocity)) = (&track.position_world_m, &track.velocity_world_mps)
    else {
        return false;
    };
    [position.x, position.y, velocity.x, velocity.y]
        .into_iter()
        .all(f64::is_finite)
}

fn relative_position_error(
    track: &ObjectTrack,
    object_truth: &ObjectTruthState,
    time_ns: i64,
    ego_truth: &[EgoTruthState],
    ego_estimates: &[EgoStateEstimate],
    ego_source: EgoSource,
    max_gap_ns: i64,
) -> Option<f64> {
    let track_position = track.position_world_m.as_ref()?;
    let object_position = object_truth.position_world_m.as_ref()?;
    let truth_ego = nearest_ego_truth(ego_truth, time_ns, max_gap_ns)?;
    let truth_pose = truth_ego.pose_world.as_ref()?;
    let truth_ego_position = truth_pose.position.as_ref()?;
    let (ego_x, ego_y, ego_yaw) = match ego_source {
        EgoSource::Truth => (
            truth_ego_position.x,
            truth_ego_position.y,
            truth_pose.yaw_rad,
        ),
        EgoSource::Estimated => {
            let estimate = nearest_estimate(ego_estimates, time_ns, max_gap_ns)?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use fusion_schema::messages::Vec2;

    fn track(track_id: &str, x: f64) -> ObjectTrack {
        ObjectTrack {
            track_id: track_id.to_owned(),
            position_world_m: Some(Vec2 { x, y: 0.0 }),
            velocity_world_mps: Some(Vec2 { x: 0.0, y: 0.0 }),
            state_covariance: vec![0.0; 16],
        }
    }

    fn truth(track_key: &str, time_ns: i64, x: f64) -> ObjectTruthState {
        ObjectTruthState {
            track_key: track_key.to_owned(),
            time_ns,
            position_world_m: Some(Vec2 { x, y: 0.0 }),
            velocity_world_mps: Some(Vec2 { x: 0.0, y: 0.0 }),
        }
    }

    #[test]
    fn time_local_matching_counts_switches_fragments_misses_and_false_tracks() {
        let mut scenario: ResolvedScenario = serde_yaml_ng::from_str(
            "root_seed: 1\nworld: { objects: [] }\ntrajectory:\n  - { id: test, duration_s: 1.0, longitudinal_acceleration_mps2: 0.0, yaw_rate_radps: 0.0 }\n",
        )
        .unwrap();
        scenario.metrics.track_truth_match_max_distance_m = 1.0;
        let times = [0, 1_000_000, 2_000_000, 3_000_000];
        let object_truth = times
            .into_iter()
            .flat_map(|time_ns| [truth("a", time_ns, 0.0), truth("b", time_ns, 10.0)])
            .collect::<Vec<_>>();
        let frames = vec![
            ObjectTrackFrame {
                estimate_time_ns: times[0],
                available_time_ns: times[0],
                tracks: vec![track("track-1", 0.0), track("track-2", 10.0)],
            },
            ObjectTrackFrame {
                estimate_time_ns: times[1],
                available_time_ns: times[1],
                tracks: vec![track("track-1", 10.0), track("track-2", 0.0)],
            },
            ObjectTrackFrame {
                estimate_time_ns: times[2],
                available_time_ns: times[2],
                tracks: vec![track("track-1", 10.0)],
            },
            ObjectTrackFrame {
                estimate_time_ns: times[3],
                available_time_ns: times[3],
                tracks: vec![
                    track("track-1", 10.0),
                    track("track-3", 0.0),
                    track("false", 50.0),
                ],
            },
        ];

        let metrics = evaluate_tracks(
            &scenario,
            &[],
            &object_truth,
            &[],
            &frames,
            EgoSource::Truth,
        );
        assert_eq!(metrics.matched_samples, 7);
        assert_eq!(metrics.missed_object_samples, 1);
        assert_eq!(metrics.false_track_samples, 1);
        assert_eq!(metrics.identity_switch_count, 2);
        assert_eq!(metrics.track_fragment_count, 1);
        assert_eq!(metrics.time_coverage_fraction, 7.0 / 8.0);
        assert_eq!(metrics.invalid_output_count, 0);
    }
}
