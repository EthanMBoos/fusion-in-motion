use anyhow::{Context, Result, ensure};
use fusion_schema::messages::{CovarianceKind, EstimateStatus, StateEstimate, TruthState};
use nalgebra::{SMatrix, SVector};
use serde::{Deserialize, Serialize};

use crate::{math, scenario::MetricsConfig};

pub const METRIC_VERSION: &str = "fusion-eval-0.2";

const FULL_STATE_DIMENSION: usize = 6;
const CONSISTENCY_STATE_DIMENSION: usize = 4;
const STANDARD_NORMAL_95: f64 = 1.959_963_984_540_054;
const CHI_SQUARED_2D_95: f64 = 5.991_464_547_107_979;

pub(crate) type FullCovariance = SMatrix<f64, FULL_STATE_DIMENSION, FULL_STATE_DIMENSION>;
type ConsistencyCovariance = SMatrix<f64, CONSISTENCY_STATE_DIMENSION, CONSISTENCY_STATE_DIMENSION>;
type ConsistencyError = SVector<f64, CONSISTENCY_STATE_DIMENSION>;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MarginalCoverage {
    pub x_fraction: f64,
    pub y_fraction: f64,
    pub yaw_fraction: f64,
    pub forward_speed_fraction: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CovarianceConsistency {
    pub evaluated_state_order: Vec<String>,
    pub covariance_state_order: Vec<String>,
    pub error_coordinates: String,
    pub degrees_of_freedom: usize,
    pub expected_anees: f64,
    pub anees: f64,
    pub normalized_anees: f64,
    pub expected_coverage_fraction: f64,
    pub marginal_coverage_95: MarginalCoverage,
}

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
    pub full_covariance_samples: usize,
    pub missing_covariance_samples: usize,
    pub covariance_consistency: Option<CovarianceConsistency>,
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
    let mut full_covariance_samples = 0_usize;
    let mut missing_covariance_samples = 0_usize;
    let mut sum_nees = 0.0;
    let mut coverage_counts = [0_usize; CONSISTENCY_STATE_DIMENSION];

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

        match validated_covariance(estimate).with_context(|| {
            format!(
                "invalid covariance at estimate time {} ns",
                estimate.estimate_time_ns
            )
        })? {
            Some(covariance) => {
                let estimate_velocity = estimate.velocity_world_mps.as_ref().ok_or_else(|| {
                    anyhow::anyhow!("full covariance estimate is missing its velocity")
                })?;
                ensure!(
                    estimate_velocity.x.is_finite()
                        && estimate_velocity.y.is_finite()
                        && estimate_velocity.z.is_finite(),
                    "full covariance estimate has non-finite velocity"
                );
                let estimate_yaw = math::yaw_from_pose(estimate_pose);
                let estimate_forward_speed = estimate_velocity.x * estimate_yaw.cos()
                    + estimate_velocity.y * estimate_yaw.sin();
                let error = ConsistencyError::from_row_slice(&[
                    ep.x - tp.x,
                    ep.y - tp.y,
                    yaw_error,
                    estimate_forward_speed - reference.forward_speed_mps,
                ]);
                let covariance = consistency_covariance(&covariance);
                let cholesky = covariance.cholesky().ok_or_else(|| {
                    anyhow::anyhow!(
                        "covariance for [x, y, yaw, forward_speed] is not positive definite"
                    )
                })?;
                let nees = error.dot(&cholesky.solve(&error));
                ensure!(
                    nees.is_finite() && nees >= 0.0,
                    "computed NEES is not finite and nonnegative"
                );
                sum_nees += nees;
                for index in 0..CONSISTENCY_STATE_DIMENSION {
                    let bound = STANDARD_NORMAL_95 * covariance[(index, index)].sqrt();
                    if error[index].abs() <= bound {
                        coverage_counts[index] += 1;
                    }
                }
                full_covariance_samples += 1;
            }
            None => missing_covariance_samples += 1,
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
    let covariance_consistency = (full_covariance_samples > 0).then(|| {
        let anees = sum_nees / full_covariance_samples as f64;
        CovarianceConsistency {
            evaluated_state_order: ["x", "y", "yaw", "forward_speed"]
                .map(str::to_owned)
                .to_vec(),
            covariance_state_order: [
                "x",
                "y",
                "yaw",
                "forward_speed",
                "gyro_bias_z",
                "accel_bias_x",
            ]
            .map(str::to_owned)
            .to_vec(),
            error_coordinates: "additive world-frame x/y, wrapped world-from-body yaw, signed body-forward speed; bias states are not evaluated because realized bias truth is not stored"
                .to_owned(),
            degrees_of_freedom: CONSISTENCY_STATE_DIMENSION,
            expected_anees: CONSISTENCY_STATE_DIMENSION as f64,
            anees,
            normalized_anees: anees / CONSISTENCY_STATE_DIMENSION as f64,
            expected_coverage_fraction: 0.95,
            marginal_coverage_95: MarginalCoverage {
                x_fraction: coverage_counts[0] as f64 / full_covariance_samples as f64,
                y_fraction: coverage_counts[1] as f64 / full_covariance_samples as f64,
                yaw_fraction: coverage_counts[2] as f64 / full_covariance_samples as f64,
                forward_speed_fraction: coverage_counts[3] as f64
                    / full_covariance_samples as f64,
            },
        }
    });
    Ok(Metrics {
        metric_version: METRIC_VERSION.to_owned(),
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
        full_covariance_samples,
        missing_covariance_samples,
        covariance_consistency,
    })
}

pub(crate) fn validated_covariance(estimate: &StateEstimate) -> Result<Option<FullCovariance>> {
    let kind = CovarianceKind::try_from(estimate.covariance_kind)
        .map_err(|value| anyhow::anyhow!("unknown covariance kind {value}"))?;
    match kind {
        CovarianceKind::Unspecified | CovarianceKind::Unknown => {
            ensure!(
                estimate.covariance.is_empty(),
                "unknown covariance must not contain matrix values"
            );
            Ok(None)
        }
        CovarianceKind::Full => {
            ensure!(
                estimate.covariance.len() == FULL_STATE_DIMENSION * FULL_STATE_DIMENSION,
                "full covariance has {} values; expected {} for a 6x6 row-major matrix",
                estimate.covariance.len(),
                FULL_STATE_DIMENSION * FULL_STATE_DIMENSION
            );
            ensure!(
                estimate.covariance.iter().all(|value| value.is_finite()),
                "full covariance contains a non-finite value"
            );
            let covariance = FullCovariance::from_row_slice(&estimate.covariance);
            let scale = covariance
                .iter()
                .fold(0.0_f64, |current, value| current.max(value.abs()));
            let tolerance = 1.0e-12 + 1.0e-9 * scale;
            for row in 0..FULL_STATE_DIMENSION {
                ensure!(
                    covariance[(row, row)] >= 0.0,
                    "full covariance has negative variance at state index {row}"
                );
                for column in (row + 1)..FULL_STATE_DIMENSION {
                    ensure!(
                        (covariance[(row, column)] - covariance[(column, row)]).abs() <= tolerance,
                        "full covariance is not symmetric at ({row}, {column})"
                    );
                }
            }
            let covariance = 0.5 * (covariance + covariance.transpose());
            ensure!(
                covariance
                    .symmetric_eigen()
                    .eigenvalues
                    .iter()
                    .all(|eigenvalue| *eigenvalue >= -tolerance),
                "full covariance is not positive semidefinite"
            );
            Ok(Some(covariance))
        }
    }
}

pub(crate) fn error_bounds_95(covariance: &FullCovariance) -> (f64, f64) {
    let variance_x = covariance[(0, 0)];
    let covariance_xy = covariance[(0, 1)];
    let variance_y = covariance[(1, 1)];
    let largest_position_eigenvalue = 0.5
        * (variance_x
            + variance_y
            + ((variance_x - variance_y).powi(2) + 4.0 * covariance_xy.powi(2)).sqrt());
    let position_bound_m = (CHI_SQUARED_2D_95 * largest_position_eigenvalue.max(0.0)).sqrt();
    let yaw_bound_rad = STANDARD_NORMAL_95 * covariance[(2, 2)].max(0.0).sqrt();
    (position_bound_m, yaw_bound_rad)
}

fn consistency_covariance(covariance: &FullCovariance) -> ConsistencyCovariance {
    covariance
        .fixed_view::<CONSISTENCY_STATE_DIMENSION, CONSISTENCY_STATE_DIMENSION>(0, 0)
        .into_owned()
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
        assert_eq!(metrics.full_covariance_samples, 0);
        assert_eq!(metrics.missing_covariance_samples, 1);
        assert!(metrics.covariance_consistency.is_none());
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

    #[test]
    fn scores_anees_and_marginal_coverage_for_known_covariance() -> Result<()> {
        let truth_pose: Pose = math::yaw_pose(0.0, 0.0, 0.0, "world", "body");
        let truth = TruthState {
            truth_time_ns: 1_000,
            pose_w_b: Some(truth_pose),
            velocity_world_mps: Some(Vec3::default()),
            acceleration_world_mps2: Some(Vec3::default()),
            angular_velocity_body_radps: Some(Vec3::default()),
            forward_speed_mps: 0.0,
            path_distance_m: 1.0,
        };
        let estimate_yaw = 0.5_f64;
        let estimate_speed = 3.0;
        let estimate = StateEstimate {
            estimator_id: "test".to_owned(),
            estimate_time_ns: 1_000,
            emission_time_ns: 1_000,
            pose_w_b: Some(math::yaw_pose(1.0, 2.0, estimate_yaw, "world", "body")),
            velocity_world_mps: Some(Vec3 {
                x: estimate_speed * estimate_yaw.cos(),
                y: estimate_speed * estimate_yaw.sin(),
                z: 0.0,
            }),
            status: EstimateStatus::Valid as i32,
            covariance_kind: CovarianceKind::Full as i32,
            covariance: FullCovariance::identity().iter().copied().collect(),
            revision: 0,
        };
        let metrics = evaluate(
            &[truth],
            &[estimate],
            &MetricsConfig {
                alignment: "NONE".to_owned(),
                divergence_position_error_m: 5.0,
            },
        )?;
        let consistency = metrics.covariance_consistency.unwrap();
        assert!((consistency.anees - 14.25).abs() < 1.0e-12);
        assert!((consistency.normalized_anees - 3.5625).abs() < 1.0e-12);
        assert_eq!(consistency.marginal_coverage_95.x_fraction, 1.0);
        assert_eq!(consistency.marginal_coverage_95.y_fraction, 0.0);
        assert_eq!(consistency.marginal_coverage_95.yaw_fraction, 1.0);
        assert_eq!(consistency.marginal_coverage_95.forward_speed_fraction, 0.0);
        Ok(())
    }

    #[test]
    fn rejects_malformed_full_covariance() {
        let estimate = |covariance: Vec<f64>| StateEstimate {
            covariance_kind: CovarianceKind::Full as i32,
            covariance,
            ..Default::default()
        };

        assert!(validated_covariance(&estimate(vec![0.0; 35])).is_err());

        let mut asymmetric = FullCovariance::identity()
            .iter()
            .copied()
            .collect::<Vec<_>>();
        asymmetric[1] = 0.25;
        assert!(validated_covariance(&estimate(asymmetric)).is_err());

        let mut non_finite = FullCovariance::identity()
            .iter()
            .copied()
            .collect::<Vec<_>>();
        non_finite[0] = f64::NAN;
        assert!(validated_covariance(&estimate(non_finite)).is_err());

        let mut negative_variance = FullCovariance::identity()
            .iter()
            .copied()
            .collect::<Vec<_>>();
        negative_variance[0] = -1.0;
        assert!(validated_covariance(&estimate(negative_variance)).is_err());
    }

    #[test]
    fn position_plot_uses_outer_radius_of_95_percent_ellipse() {
        let mut covariance = FullCovariance::identity();
        covariance[(0, 0)] = 4.0;
        covariance[(1, 1)] = 1.0;
        covariance[(2, 2)] = 0.25;
        let (position_bound, yaw_bound) = error_bounds_95(&covariance);
        assert!((position_bound - (CHI_SQUARED_2D_95 * 4.0).sqrt()).abs() < 1.0e-12);
        assert!((yaw_bound - STANDARD_NORMAL_95 * 0.5).abs() < 1.0e-12);
    }
}
