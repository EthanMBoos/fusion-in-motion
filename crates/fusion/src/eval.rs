use anyhow::{Context, Result, ensure};
use fusion_schema::messages::{CovarianceKind, EstimateStatus, StateEstimate, TruthState};
use nalgebra::{SMatrix, SVector};
use serde::{Deserialize, Serialize};

use crate::math;

pub const METRIC_VERSION: &str = "fusion-eval-0.3";
pub const METRIC_ALIGNMENT: &str = "NONE";

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
    pub position_rmse_m: Option<f64>,
    pub yaw_rmse_rad: Option<f64>,
    pub translational_ate_rmse_m: Option<f64>,
    pub rotational_ate_rmse_rad: Option<f64>,
    pub final_position_error_m: Option<f64>,
    pub final_drift_per_distance: Option<f64>,
    pub valid_output_count: usize,
    pub initializing_output_count: usize,
    pub diverged_output_count: usize,
    pub unspecified_output_count: usize,
    pub unknown_status_output_count: usize,
    pub valid_output_fraction: f64,
    pub unmatched_valid_output_count: usize,
    pub time_to_first_valid_output_s: Option<f64>,
    pub last_valid_estimate_time_s: Option<f64>,
    pub maximum_position_error_m: Option<f64>,
    pub full_covariance_samples: usize,
    pub missing_covariance_samples: usize,
    pub covariance_consistency: Option<CovarianceConsistency>,
}

pub fn evaluate(truth: &[TruthState], estimates: &[StateEstimate]) -> Result<Metrics> {
    ensure!(!truth.is_empty(), "truth stream is empty");
    ensure!(!estimates.is_empty(), "estimate stream is empty");
    ensure!(
        truth
            .windows(2)
            .all(|pair| pair[0].truth_time_ns < pair[1].truth_time_ns),
        "truth timestamps must be strictly increasing"
    );
    let estimator_id = estimates[0].estimator_id.clone();
    let mut sum_position_sq = 0.0;
    let mut sum_yaw_sq = 0.0;
    let mut matched = 0_usize;
    let mut valid = 0_usize;
    let mut initializing = 0_usize;
    let mut diverged = 0_usize;
    let mut unspecified = 0_usize;
    let mut unknown_status = 0_usize;
    let mut unmatched_valid = 0_usize;
    let mut first_valid_ns = None;
    let mut final_position_error = None;
    let mut final_path_distance = None;
    let mut maximum_position_error = None;
    let mut last_valid_ns = None;
    let mut full_covariance_samples = 0_usize;
    let mut missing_covariance_samples = 0_usize;
    let mut sum_nees = 0.0;
    let mut coverage_counts = [0_usize; CONSISTENCY_STATE_DIMENSION];

    for estimate in estimates {
        match EstimateStatus::try_from(estimate.status) {
            Ok(EstimateStatus::Valid) => valid += 1,
            Ok(EstimateStatus::Initializing) => {
                initializing += 1;
                continue;
            }
            Ok(EstimateStatus::Diverged) => {
                diverged += 1;
                continue;
            }
            Ok(EstimateStatus::Unspecified) => {
                unspecified += 1;
                continue;
            }
            Err(_) => {
                unknown_status += 1;
                continue;
            }
        }
        first_valid_ns = Some(
            first_valid_ns
                .map(|time: i64| time.min(estimate.emission_time_ns))
                .unwrap_or(estimate.emission_time_ns),
        );
        last_valid_ns = Some(
            last_valid_ns
                .map(|time: i64| time.max(estimate.estimate_time_ns))
                .unwrap_or(estimate.estimate_time_ns),
        );
        let Some(reference) = truth_at_time(truth, estimate.estimate_time_ns)? else {
            unmatched_valid += 1;
            continue;
        };
        let estimate_pose = estimate
            .pose_w_b
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("valid estimate is missing its pose"))?;
        let ep = estimate_pose
            .position
            .as_ref()
            .ok_or_else(|| anyhow::anyhow!("estimate pose is missing position"))?;
        let position_error = ((ep.x - reference.x_m).powi(2)
            + (ep.y - reference.y_m).powi(2)
            + (ep.z - reference.z_m).powi(2))
        .sqrt();
        let yaw_error = math::wrap_angle(
            math::yaw_from_pose(estimate_pose) - reference.yaw_world_from_body_rad,
        );
        sum_position_sq += position_error.powi(2);
        sum_yaw_sq += yaw_error.powi(2);
        final_position_error = Some(position_error);
        final_path_distance = Some(reference.path_distance_m);
        maximum_position_error = Some(
            maximum_position_error
                .map(|error: f64| error.max(position_error))
                .unwrap_or(position_error),
        );

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
                    ep.x - reference.x_m,
                    ep.y - reference.y_m,
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
    let position_rmse = (matched > 0).then(|| (sum_position_sq / matched as f64).sqrt());
    let yaw_rmse = (matched > 0).then(|| (sum_yaw_sq / matched as f64).sqrt());
    let truth_start_ns = truth.first().map(|state| state.truth_time_ns).unwrap_or(0);
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
            error_coordinates: "additive world-frame x/y, wrapped world-from-body yaw, signed body-forward speed; bias states are not evaluated"
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
        alignment: METRIC_ALIGNMENT.to_owned(),
        estimator_id,
        truth_samples: truth.len(),
        estimate_samples: estimates.len(),
        matched_samples: matched,
        position_rmse_m: position_rmse,
        yaw_rmse_rad: yaw_rmse,
        translational_ate_rmse_m: position_rmse,
        rotational_ate_rmse_rad: yaw_rmse,
        final_position_error_m: final_position_error,
        final_drift_per_distance: final_position_error.zip(final_path_distance).map(
            |(position_error, path_distance)| {
                if path_distance.abs() > 1.0e-12 {
                    position_error / path_distance.abs()
                } else {
                    0.0
                }
            },
        ),
        valid_output_count: valid,
        initializing_output_count: initializing,
        diverged_output_count: diverged,
        unspecified_output_count: unspecified,
        unknown_status_output_count: unknown_status,
        valid_output_fraction: valid as f64 / estimates.len() as f64,
        unmatched_valid_output_count: unmatched_valid,
        time_to_first_valid_output_s: first_valid_ns
            .map(|time| (time as f64 - truth_start_ns as f64) * 1.0e-9),
        last_valid_estimate_time_s: last_valid_ns
            .map(|time| (time as f64 - truth_start_ns as f64) * 1.0e-9),
        maximum_position_error_m: maximum_position_error,
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

#[derive(Debug, Clone, Copy)]
struct TruthReference {
    x_m: f64,
    y_m: f64,
    z_m: f64,
    yaw_world_from_body_rad: f64,
    forward_speed_mps: f64,
    path_distance_m: f64,
}

fn truth_at_time(truth: &[TruthState], time_ns: i64) -> Result<Option<TruthReference>> {
    let upper = truth.partition_point(|state| state.truth_time_ns < time_ns);
    if upper < truth.len() && truth[upper].truth_time_ns == time_ns {
        return truth_reference(&truth[upper]).map(Some);
    }
    if upper == 0 || upper == truth.len() {
        return Ok(None);
    }

    let before = &truth[upper - 1];
    let after = &truth[upper];
    let before_reference = truth_reference(before)?;
    let after_reference = truth_reference(after)?;
    let interval_ns = i128::from(after.truth_time_ns) - i128::from(before.truth_time_ns);
    let offset_ns = i128::from(time_ns) - i128::from(before.truth_time_ns);
    let alpha = offset_ns as f64 / interval_ns as f64;
    let interpolate = |start: f64, end: f64| start + alpha * (end - start);

    Ok(Some(TruthReference {
        x_m: interpolate(before_reference.x_m, after_reference.x_m),
        y_m: interpolate(before_reference.y_m, after_reference.y_m),
        z_m: interpolate(before_reference.z_m, after_reference.z_m),
        yaw_world_from_body_rad: math::wrap_angle(
            before_reference.yaw_world_from_body_rad
                + alpha
                    * math::wrap_angle(
                        after_reference.yaw_world_from_body_rad
                            - before_reference.yaw_world_from_body_rad,
                    ),
        ),
        forward_speed_mps: interpolate(
            before_reference.forward_speed_mps,
            after_reference.forward_speed_mps,
        ),
        path_distance_m: interpolate(
            before_reference.path_distance_m,
            after_reference.path_distance_m,
        ),
    }))
}

fn truth_reference(state: &TruthState) -> Result<TruthReference> {
    let pose = state
        .pose_w_b
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("truth state is missing its pose"))?;
    let position = pose
        .position
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("truth pose is missing position"))?;
    Ok(TruthReference {
        x_m: position.x,
        y_m: position.y,
        z_m: position.z,
        yaw_world_from_body_rad: math::yaw_from_pose(pose),
        forward_speed_mps: state.forward_speed_mps,
        path_distance_m: state.path_distance_m,
    })
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
        let metrics = evaluate(&[truth], &[estimate]).unwrap();
        assert_eq!(metrics.position_rmse_m, Some(0.0));
        assert_eq!(metrics.yaw_rmse_rad, Some(0.0));
        assert_eq!(metrics.valid_output_count, 1);
        assert_eq!(metrics.valid_output_fraction, 1.0);
        assert_eq!(metrics.full_covariance_samples, 0);
        assert_eq!(metrics.missing_covariance_samples, 1);
        assert!(metrics.covariance_consistency.is_none());
    }

    #[test]
    fn output_fraction_does_not_assume_truth_cadence() {
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
        )
        .unwrap();
        assert_eq!(metrics.valid_output_count, 1);
        assert_eq!(metrics.valid_output_fraction, 1.0);
        assert!((metrics.last_valid_estimate_time_s.unwrap() - 1.0e-6).abs() < 1.0e-15);
    }

    #[test]
    fn counts_estimator_statuses_without_inventing_divergence() {
        let pose: Pose = math::yaw_pose(10.0, 0.0, 0.0, "world", "body");
        let truth = TruthState {
            truth_time_ns: 0,
            pose_w_b: Some(math::yaw_pose(0.0, 0.0, 0.0, "world", "body")),
            velocity_world_mps: Some(Vec3::default()),
            acceleration_world_mps2: Some(Vec3::default()),
            angular_velocity_body_radps: Some(Vec3::default()),
            forward_speed_mps: 0.0,
            path_distance_m: 0.0,
        };
        let estimate_with_status = |status| StateEstimate {
            estimator_id: "test".to_owned(),
            pose_w_b: Some(pose.clone()),
            velocity_world_mps: Some(Vec3::default()),
            status,
            covariance_kind: CovarianceKind::Unknown as i32,
            ..Default::default()
        };
        let estimates = [
            estimate_with_status(EstimateStatus::Valid as i32),
            estimate_with_status(EstimateStatus::Initializing as i32),
            estimate_with_status(EstimateStatus::Diverged as i32),
            estimate_with_status(EstimateStatus::Unspecified as i32),
            estimate_with_status(99),
        ];

        let metrics = evaluate(&[truth], &estimates).unwrap();
        assert_eq!(metrics.maximum_position_error_m, Some(10.0));
        assert_eq!(metrics.valid_output_count, 1);
        assert_eq!(metrics.initializing_output_count, 1);
        assert_eq!(metrics.diverged_output_count, 1);
        assert_eq!(metrics.unspecified_output_count, 1);
        assert_eq!(metrics.unknown_status_output_count, 1);
        assert_eq!(metrics.valid_output_fraction, 0.2);
    }

    #[test]
    fn reports_status_when_no_output_can_be_scored() {
        let estimate = StateEstimate {
            estimator_id: "test".to_owned(),
            status: EstimateStatus::Diverged as i32,
            ..Default::default()
        };
        let truth = TruthState {
            truth_time_ns: 0,
            pose_w_b: Some(math::yaw_pose(0.0, 0.0, 0.0, "world", "body")),
            ..Default::default()
        };

        let metrics = evaluate(&[truth], &[estimate]).unwrap();
        assert_eq!(metrics.diverged_output_count, 1);
        assert_eq!(metrics.valid_output_fraction, 0.0);
        assert_eq!(metrics.matched_samples, 0);
        assert_eq!(metrics.position_rmse_m, None);
        assert_eq!(metrics.time_to_first_valid_output_s, None);
    }

    #[test]
    fn interpolates_truth_and_does_not_extrapolate() {
        let truth_at = |time, x, yaw, speed, distance| TruthState {
            truth_time_ns: time,
            pose_w_b: Some(math::yaw_pose(x, 0.0, yaw, "world", "body")),
            velocity_world_mps: Some(Vec3::default()),
            acceleration_world_mps2: Some(Vec3::default()),
            angular_velocity_body_radps: Some(Vec3::default()),
            forward_speed_mps: speed,
            path_distance_m: distance,
        };
        let estimate_at = |time, x, yaw| StateEstimate {
            estimator_id: "test".to_owned(),
            estimate_time_ns: time,
            emission_time_ns: time,
            pose_w_b: Some(math::yaw_pose(x, 0.0, yaw, "world", "body")),
            velocity_world_mps: Some(Vec3::default()),
            status: EstimateStatus::Valid as i32,
            covariance_kind: CovarianceKind::Unknown as i32,
            covariance: Vec::new(),
            revision: 0,
        };
        let truth = [
            truth_at(0, 0.0, 179.0_f64.to_radians(), 0.0, 0.0),
            truth_at(2_000, 2.0, (-179.0_f64).to_radians(), 2.0, 2.0),
        ];
        let estimates = [
            estimate_at(1_000, 1.0, std::f64::consts::PI),
            estimate_at(3_000, 2.0, (-179.0_f64).to_radians()),
        ];

        let metrics = evaluate(&truth, &estimates).unwrap();
        assert!(metrics.position_rmse_m.unwrap() < 1.0e-12);
        assert!(metrics.yaw_rmse_rad.unwrap() < 1.0e-12);
        assert_eq!(metrics.valid_output_count, 2);
        assert_eq!(metrics.matched_samples, 1);
        assert_eq!(metrics.unmatched_valid_output_count, 1);

        let outside = [estimate_at(3_000, 2.0, (-179.0_f64).to_radians())];
        let outside_metrics = evaluate(&truth, &outside).unwrap();
        assert_eq!(outside_metrics.matched_samples, 0);
        assert_eq!(outside_metrics.unmatched_valid_output_count, 1);
        assert_eq!(outside_metrics.position_rmse_m, None);
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
        let metrics = evaluate(&[truth], &[estimate])?;
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
