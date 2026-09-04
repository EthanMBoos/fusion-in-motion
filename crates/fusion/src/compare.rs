use std::{fs, path::Path};

use anyhow::{Context, Result};

use crate::eval::Metrics;

pub fn render(baseline: &Path, variant: &Path) -> Result<String> {
    let baseline_metrics = load(baseline)?;
    let variant_metrics = load(variant)?;
    let mut out = format!(
        "Comparing {} -> {}\n\n{:<28} {:>10}    {:>10}    {:>17}\n",
        baseline.display(),
        variant.display(),
        "RESULT",
        "BASELINE",
        "VARIANT",
        "CHANGE",
    );
    optional_row(
        &mut out,
        "position RMSE (m)",
        baseline_metrics.position_rmse_m,
        variant_metrics.position_rmse_m,
        false,
    );
    optional_row(
        &mut out,
        "yaw RMSE (rad)",
        baseline_metrics.yaw_rmse_rad,
        variant_metrics.yaw_rmse_rad,
        false,
    );
    optional_row(
        &mut out,
        "final position error (m)",
        baseline_metrics.final_position_error_m,
        variant_metrics.final_position_error_m,
        false,
    );
    optional_row(
        &mut out,
        "maximum position error (m)",
        baseline_metrics.maximum_position_error_m,
        variant_metrics.maximum_position_error_m,
        false,
    );
    optional_row(
        &mut out,
        "gyro bias RMSE (rad/s)",
        baseline_metrics
            .bias_evaluation
            .as_ref()
            .and_then(|bias| bias.gyro_bias_z_rmse_radps),
        variant_metrics
            .bias_evaluation
            .as_ref()
            .and_then(|bias| bias.gyro_bias_z_rmse_radps),
        false,
    );
    optional_row(
        &mut out,
        "accel bias RMSE (m/s²)",
        baseline_metrics
            .bias_evaluation
            .as_ref()
            .and_then(|bias| bias.accel_bias_x_rmse_mps2),
        variant_metrics
            .bias_evaluation
            .as_ref()
            .and_then(|bias| bias.accel_bias_x_rmse_mps2),
        false,
    );
    row(
        &mut out,
        "valid outputs (%)",
        baseline_metrics.valid_output_fraction * 100.0,
        variant_metrics.valid_output_fraction * 100.0,
        true,
    );
    if let (Some(baseline_consistency), Some(variant_consistency)) = (
        baseline_metrics.covariance_consistency.as_ref(),
        variant_metrics.covariance_consistency.as_ref(),
    ) {
        row(
            &mut out,
            "normalized ANEES",
            baseline_consistency.normalized_anees,
            variant_consistency.normalized_anees,
            false,
        );
        for (name, baseline, variant) in [
            (
                "x 95% coverage (%)",
                baseline_consistency.marginal_coverage_95.x_fraction,
                variant_consistency.marginal_coverage_95.x_fraction,
            ),
            (
                "y 95% coverage (%)",
                baseline_consistency.marginal_coverage_95.y_fraction,
                variant_consistency.marginal_coverage_95.y_fraction,
            ),
            (
                "yaw 95% coverage (%)",
                baseline_consistency.marginal_coverage_95.yaw_fraction,
                variant_consistency.marginal_coverage_95.yaw_fraction,
            ),
            (
                "speed 95% coverage (%)",
                baseline_consistency
                    .marginal_coverage_95
                    .forward_speed_fraction,
                variant_consistency
                    .marginal_coverage_95
                    .forward_speed_fraction,
            ),
        ] {
            row(&mut out, name, baseline * 100.0, variant * 100.0, true);
        }
    }
    if let (Some(baseline_consistency), Some(variant_consistency)) = (
        baseline_metrics
            .bias_evaluation
            .as_ref()
            .and_then(|bias| bias.covariance_consistency.as_ref()),
        variant_metrics
            .bias_evaluation
            .as_ref()
            .and_then(|bias| bias.covariance_consistency.as_ref()),
    ) {
        row(
            &mut out,
            "normalized bias ANEES",
            baseline_consistency.normalized_anees,
            variant_consistency.normalized_anees,
            false,
        );
        row(
            &mut out,
            "gyro bias 95% coverage (%)",
            baseline_consistency.marginal_coverage_95.gyro_z_fraction * 100.0,
            variant_consistency.marginal_coverage_95.gyro_z_fraction * 100.0,
            true,
        );
        row(
            &mut out,
            "accel bias 95% coverage (%)",
            baseline_consistency.marginal_coverage_95.accel_x_fraction * 100.0,
            variant_consistency.marginal_coverage_95.accel_x_fraction * 100.0,
            true,
        );
    }
    out.push_str(
        "\nPositive error deltas are worse. Positive valid-output deltas are better.\nNormalized ANEES should be judged against 1.0 and marginal coverage against 95%, not by delta direction alone.\n",
    );
    Ok(out)
}

fn load(path: &Path) -> Result<Metrics> {
    let metrics_path = path.join("reports/baseline/metrics.json");
    let bytes = fs::read(&metrics_path)
        .with_context(|| format!("{} is not a completed run", path.display()))?;
    serde_json::from_slice(&bytes)
        .with_context(|| format!("failed to read metrics from {}", metrics_path.display()))
}

fn row(out: &mut String, name: &str, before: f64, after: f64, percentage_points: bool) {
    let delta = after - before;
    let change = if percentage_points {
        format!("{delta:+.2} pp")
    } else if before.abs() > 1.0e-12 {
        format!("{delta:+.4} ({:+.1}%)", delta / before * 100.0)
    } else {
        format!("{delta:+.4}")
    };
    out.push_str(&format!(
        "{name:<28} {before:>10.4}    {after:>10.4}    {change:>17}\n"
    ));
}

fn optional_row(
    out: &mut String,
    name: &str,
    before: Option<f64>,
    after: Option<f64>,
    percentage_points: bool,
) {
    match (before, after) {
        (Some(before), Some(after)) => row(out, name, before, after, percentage_points),
        _ => out.push_str(&format!(
            "{name:<28} {:>10}    {:>10}    {:>17}\n",
            before
                .map(|value| format!("{value:.4}"))
                .unwrap_or_else(|| "—".to_owned()),
            after
                .map(|value| format!("{value:.4}"))
                .unwrap_or_else(|| "—".to_owned()),
            "—",
        )),
    }
}
