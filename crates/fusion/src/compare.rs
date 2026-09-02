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
    row(
        &mut out,
        "position RMSE (m)",
        baseline_metrics.position_rmse_m,
        variant_metrics.position_rmse_m,
        false,
    );
    row(
        &mut out,
        "yaw RMSE (rad)",
        baseline_metrics.yaw_rmse_rad,
        variant_metrics.yaw_rmse_rad,
        false,
    );
    row(
        &mut out,
        "final position error (m)",
        baseline_metrics.final_position_error_m,
        variant_metrics.final_position_error_m,
        false,
    );
    row(
        &mut out,
        "maximum position error (m)",
        baseline_metrics.maximum_position_error_m,
        variant_metrics.maximum_position_error_m,
        false,
    );
    row(
        &mut out,
        "availability (%)",
        baseline_metrics.availability_fraction * 100.0,
        variant_metrics.availability_fraction * 100.0,
        true,
    );
    out.push_str("\nPositive error deltas are worse. Positive availability deltas are better.\n");
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
