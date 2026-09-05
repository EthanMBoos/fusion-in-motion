use std::{collections::BTreeSet, fs, path::Path};

use anyhow::{Context, Result};
use serde_yaml_ng::Value;

use crate::{eval::RunMetrics, scenario};

pub fn render(baseline: &Path, variant: &Path) -> Result<String> {
    let baseline_metrics = read_metrics(baseline)?;
    let variant_metrics = read_metrics(variant)?;
    let baseline_scenario = scenario::load_and_resolve(&baseline.join("scenario.resolved.yaml"))?;
    let variant_scenario = scenario::load_and_resolve(&variant.join("scenario.resolved.yaml"))?;
    let mut changes = Vec::new();
    collect_changes(
        "",
        &serde_yaml_ng::to_value(baseline_scenario)?,
        &serde_yaml_ng::to_value(variant_scenario)?,
        &mut changes,
    )?;
    let changes = if changes.is_empty() {
        "  none\n".to_owned()
    } else {
        changes
            .into_iter()
            .map(|change| format!("  {change}\n"))
            .collect()
    };
    let estimated_ego_tracks = render_track_change(
        &baseline_metrics.tracks_with_estimated_ego,
        &variant_metrics.tracks_with_estimated_ego,
    );
    let truth_ego_tracks = render_track_change(
        &baseline_metrics.tracks_with_truth_ego,
        &variant_metrics.tracks_with_truth_ego,
    );
    let ego_cost = render_optional_change(
        baseline_metrics.estimated_ego_position_rmse_delta_m,
        variant_metrics.estimated_ego_position_rmse_delta_m,
    );
    Ok(format!(
        "Baseline: {}\nVariant:  {}\n\nChanged settings:\n{}\nResults:\nVehicle position RMSE: {:.3} -> {:.3} m ({:+.3})\nVehicle heading RMSE:  {:.3} -> {:.3} rad ({:+.3})\nObject RMSE, estimated ego: {estimated_ego_tracks}\nObject RMSE, truth ego:     {truth_ego_tracks}\nEgo cost in object tracks:  {ego_cost}\n",
        baseline.display(),
        variant.display(),
        changes,
        baseline_metrics.ego.position_rmse_m,
        variant_metrics.ego.position_rmse_m,
        variant_metrics.ego.position_rmse_m - baseline_metrics.ego.position_rmse_m,
        baseline_metrics.ego.yaw_rmse_rad,
        variant_metrics.ego.yaw_rmse_rad,
        variant_metrics.ego.yaw_rmse_rad - baseline_metrics.ego.yaw_rmse_rad,
    ))
}

fn render_track_change(
    baseline: &crate::eval::TrackMetrics,
    variant: &crate::eval::TrackMetrics,
) -> String {
    render_optional_change(baseline.position_rmse_m, variant.position_rmse_m)
}

fn render_optional_change(baseline: Option<f64>, variant: Option<f64>) -> String {
    match (baseline, variant) {
        (Some(baseline), Some(variant)) => {
            format!(
                "{baseline:.3} -> {variant:.3} m ({:+.3})",
                variant - baseline
            )
        }
        (Some(baseline), None) => format!("{baseline:.3} m -> no track"),
        (None, Some(variant)) => format!("no track -> {variant:.3} m"),
        (None, None) => "no track".to_owned(),
    }
}

fn collect_changes(
    path: &str,
    baseline: &Value,
    variant: &Value,
    changes: &mut Vec<String>,
) -> Result<()> {
    match (baseline, variant) {
        (Value::Mapping(baseline), Value::Mapping(variant)) => {
            let keys = baseline
                .keys()
                .chain(variant.keys())
                .filter_map(Value::as_str)
                .collect::<BTreeSet<_>>();
            for key in keys {
                let key_value = Value::String(key.to_owned());
                let next_path = join_path(path, key);
                match (baseline.get(&key_value), variant.get(&key_value)) {
                    (Some(left), Some(right)) => collect_changes(&next_path, left, right, changes)?,
                    (left, right) => changes.push(format!(
                        "{next_path}: {} -> {}",
                        render_value(left)?,
                        render_value(right)?,
                    )),
                }
            }
        }
        (Value::Sequence(baseline), Value::Sequence(variant)) => {
            for (index, (left, right)) in baseline.iter().zip(variant).enumerate() {
                collect_changes(&format!("{path}[{index}]"), left, right, changes)?;
            }
            if baseline.len() != variant.len() {
                changes.push(format!(
                    "{path}.length: {} -> {}",
                    baseline.len(),
                    variant.len()
                ));
            }
        }
        _ if baseline != variant => changes.push(format!(
            "{path}: {} -> {}",
            render_value(Some(baseline))?,
            render_value(Some(variant))?,
        )),
        _ => {}
    }
    Ok(())
}

fn join_path(path: &str, key: &str) -> String {
    if path.is_empty() {
        key.to_owned()
    } else {
        format!("{path}.{key}")
    }
}

fn render_value(value: Option<&Value>) -> Result<String> {
    value.map_or_else(
        || Ok("<missing>".to_owned()),
        |value| Ok(serde_yaml_ng::to_string(value)?.trim().to_owned()),
    )
}

fn read_metrics(run: &Path) -> Result<RunMetrics> {
    let path = run.join("reports/baseline/metrics.json");
    let json: serde_json::Value = serde_json::from_slice(
        &fs::read(&path).with_context(|| format!("failed to read {}", path.display()))?,
    )?;
    Ok(serde_json::from_value(json["metrics"].clone())?)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reports_changed_leaf_settings() -> Result<()> {
        let baseline: Value = serde_yaml_ng::from_str(
            "gps:\n  latency_ns: 0\nego_estimator:\n  timing_compensation: false\n",
        )?;
        let variant: Value = serde_yaml_ng::from_str(
            "gps:\n  latency_ns: 120000000\nego_estimator:\n  timing_compensation: true\n",
        )?;
        let mut changes = Vec::new();
        collect_changes("", &baseline, &variant, &mut changes)?;
        assert_eq!(
            changes,
            [
                "ego_estimator.timing_compensation: false -> true",
                "gps.latency_ns: 0 -> 120000000",
            ]
        );
        Ok(())
    }
}
