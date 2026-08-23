use std::{fs, path::Path};

use anyhow::{Context, Result};

use crate::eval::RunMetrics;

pub fn render(baseline: &Path, variant: &Path) -> Result<String> {
    let baseline_metrics = read_metrics(baseline)?;
    let variant_metrics = read_metrics(variant)?;
    Ok(format!(
        "Baseline: {}\nVariant:  {}\n\nVehicle position RMSE: {:.3} -> {:.3} m ({:+.3})\nVehicle heading RMSE:  {:.3} -> {:.3} rad ({:+.3})\nObject RMSE, estimated ego: {:.3} -> {:.3} m ({:+.3})\nObject RMSE, truth ego:     {:.3} -> {:.3} m ({:+.3})\nEgo cost in object tracks:  {:.3} -> {:.3} m ({:+.3})\n",
        baseline.display(),
        variant.display(),
        baseline_metrics.ego.position_rmse_m,
        variant_metrics.ego.position_rmse_m,
        variant_metrics.ego.position_rmse_m - baseline_metrics.ego.position_rmse_m,
        baseline_metrics.ego.yaw_rmse_rad,
        variant_metrics.ego.yaw_rmse_rad,
        variant_metrics.ego.yaw_rmse_rad - baseline_metrics.ego.yaw_rmse_rad,
        baseline_metrics.tracks_with_estimated_ego.position_rmse_m,
        variant_metrics.tracks_with_estimated_ego.position_rmse_m,
        variant_metrics.tracks_with_estimated_ego.position_rmse_m
            - baseline_metrics.tracks_with_estimated_ego.position_rmse_m,
        baseline_metrics.tracks_with_truth_ego.position_rmse_m,
        variant_metrics.tracks_with_truth_ego.position_rmse_m,
        variant_metrics.tracks_with_truth_ego.position_rmse_m
            - baseline_metrics.tracks_with_truth_ego.position_rmse_m,
        baseline_metrics.estimated_ego_position_rmse_delta_m,
        variant_metrics.estimated_ego_position_rmse_delta_m,
        variant_metrics.estimated_ego_position_rmse_delta_m
            - baseline_metrics.estimated_ego_position_rmse_delta_m,
    ))
}

fn read_metrics(run: &Path) -> Result<RunMetrics> {
    let path = run.join("reports/baseline/metrics.json");
    let json: serde_json::Value = serde_json::from_slice(
        &fs::read(&path).with_context(|| format!("failed to read {}", path.display()))?,
    )?;
    Ok(serde_json::from_value(json["metrics"].clone())?)
}
