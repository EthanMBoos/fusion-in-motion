use anyhow::{Result, ensure};
use fusion_schema::messages::{EstimateStatus, StateEstimate, TruthState};
use serde::{Deserialize, Serialize};

use crate::{math, scenario::MetricsConfig};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Metrics {
    pub metric_version: String,
    pub alignment: String,
    pub estimator_id: String,
    pub truth_samples: usize,
    pub estimate_samples: usize,
    pub matched_samples: usize,
    pub position_rmse_m: f64,
    pub yaw_rmse_rad: f64,
    pub translational_ate_rmse_m: f64,
    pub rotational_ate_rmse_rad: f64,
    pub final_position_error_m: f64,
    pub final_drift_per_distance: f64,
    pub availability_fraction: f64,
    pub time_to_first_valid_s: f64,
    pub invalid_output_count: usize,
    pub diverged_output_count: usize,
    pub maximum_position_error_m: f64,
    pub time_coverage_fraction: f64,
}

pub fn evaluate(
    truth: &[TruthState],
    estimates: &[StateEstimate],
    config: &MetricsConfig,
) -> Result<Metrics> {
    ensure!(!truth.is_empty(), "truth stream is empty");
    ensure!(!estimates.is_empty(), "estimate stream is empty");
    let estimator_id = estimates[0].estimator_id.clone();
    let mut sum_position_sq = 0.0;
    let mut sum_yaw_sq = 0.0;
    let mut matched = 0_usize;
    let mut valid = 0_usize;
    let mut invalid = 0_usize;
    let mut first_valid_ns = None;
    let mut final_position_error = 0.0;
    let mut final_path_distance = 0.0;
    let mut maximum_position_error: f64 = 0.0;
    let mut diverged = 0_usize;
    let mut last_valid_ns = None;

    for estimate in estimates {
        if estimate.status != EstimateStatus::Valid as i32 {
            invalid += 1;
            continue;
        }
        valid += 1;
        first_valid_ns.get_or_insert(estimate.emission_time_ns);
        last_valid_ns = Some(estimate.estimate_time_ns);
        let Some(reference) = nearest_truth(truth, estimate.estimate_time_ns) else {
            continue;
        };
        let estimate_pose = estimate
            .pose_w_b
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("valid estimate is missing its pose"))?;
        let truth_pose = reference
            .pose_w_b
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("truth state is missing its pose"))?;
        let ep = estimate_pose
            .position
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("estimate pose is missing position"))?;
        let tp = truth_pose
            .position
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("truth pose is missing position"))?;
        let position_error =
            ((ep.x - tp.x).powi(2) + (ep.y - tp.y).powi(2) + (ep.z - tp.z).powi(2)).sqrt();
        let yaw_error =
            math::wrap_angle(math::yaw_from_pose(estimate_pose) - math::yaw_from_pose(truth_pose));
        sum_position_sq += position_error.powi(2);
        sum_yaw_sq += yaw_error.powi(2);
        final_position_error = position_error;
        final_path_distance = reference.path_distance_m;
        maximum_position_error = maximum_position_error.max(position_error);
        if position_error > config.divergence_position_error_m {
            diverged += 1;
        }
        matched += 1;
    }
    ensure!(matched > 0, "no estimates could be matched to truth");
    let position_rmse = (sum_position_sq / matched as f64).sqrt();
    let yaw_rmse = (sum_yaw_sq / matched as f64).sqrt();
    let expected_outputs = truth.len().saturating_sub(1).max(1);
    let truth_start_ns = truth.first().map(|state| state.truth_time_ns).unwrap_or(0);
    let truth_end_ns = truth.last().map(|state| state.truth_time_ns).unwrap_or(0);
    let truth_duration_ns = truth_end_ns.saturating_sub(truth_start_ns);
    let time_coverage_fraction = if truth_duration_ns > 0 {
        last_valid_ns
            .map(|time| time.saturating_sub(truth_start_ns) as f64 / truth_duration_ns as f64)
            .unwrap_or(0.0)
            .clamp(0.0, 1.0)
    } else {
        1.0
    };
    Ok(Metrics {
        metric_version: "fusion-eval-0.1".to_owned(),
        alignment: "NONE".to_owned(),
        estimator_id,
        truth_samples: truth.len(),
        estimate_samples: estimates.len(),
        matched_samples: matched,
        position_rmse_m: position_rmse,
        yaw_rmse_rad: yaw_rmse,
        translational_ate_rmse_m: position_rmse,
        rotational_ate_rmse_rad: yaw_rmse,
        final_position_error_m: final_position_error,
        final_drift_per_distance: if final_path_distance.abs() > 1.0e-12 {
            final_position_error / final_path_distance.abs()
        } else {
            0.0
        },
        availability_fraction: (valid as f64 / expected_outputs as f64).clamp(0.0, 1.0),
        time_to_first_valid_s: first_valid_ns.unwrap_or(0) as f64 * 1.0e-9,
        invalid_output_count: invalid,
        diverged_output_count: diverged,
        maximum_position_error_m: maximum_position_error,
        time_coverage_fraction,
    })
}

fn nearest_truth(truth: &[TruthState], time_ns: i64) -> Option<&TruthState> {
    truth
        .iter()
        .min_by_key(|state| state.truth_time_ns.abs_diff(time_ns))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math;
    use fusion_schema::messages::{CovarianceKind, Pose, Vec3};

    #[test]
    fn perfect_estimate_scores_zero() {
        let pose: Pose = math::yaw_pose(1.0, 2.0, 0.3, "world", "body");
        let truth = TruthState {
            truth_time_ns: 1_000,
            pose_w_b: Some(pose.clone()),
            velocity_world_mps: Some(Vec3::default()),
            acceleration_world_mps2: Some(Vec3::default()),
            angular_velocity_body_radps: Some(Vec3::default()),
            forward_speed_mps: 0.0,
            path_distance_m: 1.0,
        };
        let estimate = StateEstimate {
            estimator_id: "test".to_owned(),
            estimate_time_ns: 1_000,
            emission_time_ns: 1_000,
            pose_w_b: Some(pose.clone()),
            velocity_world_mps: Some(Vec3::default()),
            status: EstimateStatus::Valid as i32,
            covariance_kind: CovarianceKind::Unknown as i32,
            covariance: Vec::new(),
            revision: 0,
        };
        let metrics = evaluate(
            &[truth],
            &[estimate],
            &MetricsConfig {
                alignment: "NONE".to_owned(),
                divergence_position_error_m: 5.0,
            },
        )
        .unwrap();
        assert_eq!(metrics.position_rmse_m, 0.0);
        assert_eq!(metrics.yaw_rmse_rad, 0.0);
    }

    #[test]
    fn early_termination_reduces_availability_and_coverage() {
        let pose: Pose = math::yaw_pose(0.0, 0.0, 0.0, "world", "body");
        let truth_at = |time| TruthState {
            truth_time_ns: time,
            pose_w_b: Some(pose.clone()),
            velocity_world_mps: Some(Vec3::default()),
            acceleration_world_mps2: Some(Vec3::default()),
            angular_velocity_body_radps: Some(Vec3::default()),
            forward_speed_mps: 0.0,
            path_distance_m: 0.0,
        };
        let estimate = StateEstimate {
            estimator_id: "test".to_owned(),
            estimate_time_ns: 1_000,
            emission_time_ns: 1_000,
            pose_w_b: Some(pose.clone()),
            velocity_world_mps: Some(Vec3::default()),
            status: EstimateStatus::Valid as i32,
            covariance_kind: CovarianceKind::Unknown as i32,
            covariance: Vec::new(),
            revision: 0,
        };
        let metrics = evaluate(
            &[truth_at(0), truth_at(1_000), truth_at(2_000)],
            &[estimate],
            &MetricsConfig {
                alignment: "NONE".to_owned(),
                divergence_position_error_m: 5.0,
            },
        )
        .unwrap();
        assert_eq!(metrics.availability_fraction, 0.5);
        assert_eq!(metrics.time_coverage_fraction, 0.5);
    }
}
