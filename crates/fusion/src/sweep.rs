use std::{
    collections::BTreeMap,
    fs,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result, bail, ensure};
use serde::{Deserialize, Serialize};
use serde_yaml_ng::Value;

use crate::{eval::RunMetrics, run_resolved_experiment, scenario};

const MAX_CASES: usize = 10_000;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SweepSpec {
    pub name: String,
    pub base_scenario: PathBuf,
    #[serde(default = "default_seeds")]
    pub seeds: Vec<u64>,
    #[serde(default)]
    pub parameters: BTreeMap<String, Vec<Value>>,
}

#[derive(Debug, Clone, Serialize)]
pub struct SweepReport {
    pub name: String,
    pub base_scenario: String,
    pub case_count: usize,
    pub successful_cases: usize,
    pub failed_cases: usize,
    pub cases: Vec<SweepCaseResult>,
    pub groups: Vec<SweepGroup>,
}

#[derive(Debug, Clone, Serialize)]
pub struct SweepCaseResult {
    pub case_id: String,
    pub root_seed: u64,
    pub parameters: BTreeMap<String, Value>,
    pub status: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metrics: Option<RunMetrics>,
}

#[derive(Debug, Clone, Serialize)]
pub struct SweepGroup {
    pub parameters: BTreeMap<String, Value>,
    pub cases: usize,
    pub successful_cases: usize,
    pub failed_cases: usize,
    pub mean_ego_position_rmse_m: Option<f64>,
    pub stddev_ego_position_rmse_m: Option<f64>,
    pub mean_estimated_ego_track_rmse_m: Option<f64>,
    pub stddev_estimated_ego_track_rmse_m: Option<f64>,
    pub mean_track_ego_cost_m: Option<f64>,
    pub sample_warning: Option<String>,
}

#[derive(Debug)]
struct ExpandedCase {
    case_id: String,
    root_seed: u64,
    parameters: BTreeMap<String, Value>,
    scenario: scenario::ResolvedScenario,
}

pub fn run(sweep_path: &Path, output: &Path) -> Result<SweepReport> {
    ensure!(
        !output.exists(),
        "sweep output {} already exists; choose a new directory",
        output.display()
    );
    let (spec, base_path, cases) = load_and_expand(sweep_path)?;
    fs::create_dir_all(output.join("reports"))?;
    fs::write(
        output.join("sweep.resolved.yaml"),
        serde_yaml_ng::to_string(&spec)?,
    )?;

    let mut results = Vec::with_capacity(cases.len());
    for case in cases {
        let case_path = output.join(&case.case_id);
        match run_resolved_experiment(&case.scenario, &case_path, false) {
            Ok(metrics) => results.push(SweepCaseResult {
                case_id: case.case_id,
                root_seed: case.root_seed,
                parameters: case.parameters,
                status: "COMPLETE".to_owned(),
                error: None,
                metrics: Some(metrics),
            }),
            Err(error) => results.push(SweepCaseResult {
                case_id: case.case_id,
                root_seed: case.root_seed,
                parameters: case.parameters,
                status: "FAILED".to_owned(),
                error: Some(format!("{error:#}")),
                metrics: None,
            }),
        }
    }

    let successful_cases = results.iter().filter(|case| case.metrics.is_some()).count();
    let report = SweepReport {
        name: spec.name,
        base_scenario: base_path.display().to_string(),
        case_count: results.len(),
        successful_cases,
        failed_cases: results.len() - successful_cases,
        groups: aggregate(&results),
        cases: results,
    };
    write_reports(output, &report)?;
    Ok(report)
}

fn load_and_expand(sweep_path: &Path) -> Result<(SweepSpec, PathBuf, Vec<ExpandedCase>)> {
    let text = fs::read_to_string(sweep_path)
        .with_context(|| format!("failed to read sweep {}", sweep_path.display()))?;
    let spec: SweepSpec = serde_yaml_ng::from_str(&text)
        .with_context(|| format!("invalid sweep {}", sweep_path.display()))?;
    validate_spec(&spec)?;

    let base_path = if spec.base_scenario.is_absolute() {
        spec.base_scenario.clone()
    } else {
        sweep_path
            .parent()
            .unwrap_or_else(|| Path::new("."))
            .join(&spec.base_scenario)
    };
    let source = fs::read_to_string(&base_path)
        .with_context(|| format!("failed to read base scenario {}", base_path.display()))?;
    let base: Value = serde_yaml_ng::from_str(&source)
        .with_context(|| format!("invalid base scenario {}", base_path.display()))?;
    let parameter_sets = expand_parameters(&spec.parameters);
    let case_count = parameter_sets
        .len()
        .checked_mul(spec.seeds.len())
        .ok_or_else(|| anyhow::anyhow!("sweep case count overflow"))?;
    ensure!(
        case_count <= MAX_CASES,
        "sweep expands to {case_count} cases; maximum is {MAX_CASES}"
    );

    let mut cases = Vec::with_capacity(case_count);
    for parameters in parameter_sets {
        for seed in &spec.seeds {
            let case_id = format!("case-{:04}", cases.len());
            let mut configured = base.clone();
            for (path, value) in &parameters {
                set_path(&mut configured, path, value.clone())?;
            }
            set_path(&mut configured, "root_seed", Value::Number((*seed).into()))?;
            let resolved: scenario::ResolvedScenario = serde_yaml_ng::from_value(configured)
                .with_context(|| format!("invalid resolved sweep {case_id}"))?;
            scenario::validate(&resolved)
                .with_context(|| format!("invalid resolved sweep {case_id}"))?;
            cases.push(ExpandedCase {
                case_id,
                root_seed: *seed,
                parameters: parameters.clone(),
                scenario: resolved,
            });
        }
    }
    Ok((spec, base_path, cases))
}

fn validate_spec(spec: &SweepSpec) -> Result<()> {
    ensure!(!spec.name.trim().is_empty(), "sweep name must not be empty");
    ensure!(
        !spec.seeds.is_empty(),
        "sweep must contain at least one seed"
    );
    for (path, values) in &spec.parameters {
        ensure!(
            !path.trim().is_empty(),
            "sweep parameter path must not be empty"
        );
        ensure!(path != "root_seed", "{path} is managed by the sweep runner");
        ensure!(!values.is_empty(), "sweep parameter {path} has no values");
    }
    Ok(())
}

fn expand_parameters(parameters: &BTreeMap<String, Vec<Value>>) -> Vec<BTreeMap<String, Value>> {
    let mut combinations = vec![BTreeMap::new()];
    for (path, values) in parameters {
        let mut expanded = Vec::with_capacity(combinations.len() * values.len());
        for existing in &combinations {
            for value in values {
                let mut next = existing.clone();
                next.insert(path.clone(), value.clone());
                expanded.push(next);
            }
        }
        combinations = expanded;
    }
    combinations
}

fn set_path(root: &mut Value, path: &str, replacement: Value) -> Result<()> {
    let parts: Vec<_> = path.split('.').collect();
    set_parts(root, &parts, replacement)
        .with_context(|| format!("cannot apply sweep parameter {path}"))
}

fn set_parts(current: &mut Value, parts: &[&str], replacement: Value) -> Result<()> {
    let Some((part, rest)) = parts.split_first() else {
        *current = replacement;
        return Ok(());
    };
    match current {
        Value::Mapping(mapping) => {
            let key = Value::String((*part).to_owned());
            let next = mapping
                .get_mut(&key)
                .ok_or_else(|| anyhow::anyhow!("field {part} does not exist"))?;
            set_parts(next, rest, replacement)
        }
        Value::Sequence(sequence) => {
            let index: usize = part
                .parse()
                .with_context(|| format!("{part} is not a sequence index"))?;
            let next = sequence
                .get_mut(index)
                .ok_or_else(|| anyhow::anyhow!("sequence index {index} is out of bounds"))?;
            set_parts(next, rest, replacement)
        }
        _ => bail!("{part} is below a scalar value"),
    }
}

fn aggregate(results: &[SweepCaseResult]) -> Vec<SweepGroup> {
    let mut grouped: BTreeMap<String, Vec<&SweepCaseResult>> = BTreeMap::new();
    for result in results {
        let key = serde_json::to_string(&result.parameters).unwrap_or_default();
        grouped.entry(key).or_default().push(result);
    }
    grouped
        .into_values()
        .map(|cases| {
            let successful: Vec<_> = cases
                .iter()
                .filter_map(|case| case.metrics.as_ref())
                .collect();
            let count = successful.len();
            let mean = |value: fn(&RunMetrics) -> f64| {
                (count > 0).then(|| successful.iter().map(|m| value(m)).sum::<f64>() / count as f64)
            };
            let stddev = |value: fn(&RunMetrics) -> f64| {
                if count < 2 {
                    return None;
                }
                let mean = successful.iter().map(|m| value(m)).sum::<f64>() / count as f64;
                Some(
                    (successful
                        .iter()
                        .map(|m| (value(m) - mean).powi(2))
                        .sum::<f64>()
                        / (count - 1) as f64)
                        .sqrt(),
                )
            };
            SweepGroup {
                parameters: cases[0].parameters.clone(),
                cases: cases.len(),
                successful_cases: count,
                failed_cases: cases.len() - count,
                mean_ego_position_rmse_m: mean(|m| m.ego.position_rmse_m),
                stddev_ego_position_rmse_m: stddev(|m| m.ego.position_rmse_m),
                mean_estimated_ego_track_rmse_m: mean(|m| {
                    m.tracks_with_estimated_ego.position_rmse_m
                }),
                stddev_estimated_ego_track_rmse_m: stddev(|m| {
                    m.tracks_with_estimated_ego.position_rmse_m
                }),
                mean_track_ego_cost_m: mean(|m| m.estimated_ego_position_rmse_delta_m),
                sample_warning: (count < 3).then(|| {
                    "Fewer than three successful seeds; do not draw a statistical conclusion."
                        .to_owned()
                }),
            }
        })
        .collect()
}

fn write_reports(output: &Path, report: &SweepReport) -> Result<()> {
    let report_dir = output.join("reports");
    fs::write(
        report_dir.join("results.json"),
        serde_json::to_vec_pretty(report)?,
    )?;

    let mut csv = String::from(
        "case_id,status,root_seed,parameters,ego_position_rmse_m,ego_yaw_rmse_rad,estimated_ego_track_rmse_m,truth_ego_track_rmse_m,track_ego_cost_m,error\n",
    );
    for case in &report.cases {
        let metrics = case.metrics.as_ref();
        csv.push_str(&format!(
            "{},{},{},{},{},{},{},{},{},{}\n",
            csv_field(&case.case_id),
            csv_field(&case.status),
            case.root_seed,
            csv_field(&serde_json::to_string(&case.parameters)?),
            optional_number(metrics.map(|m| m.ego.position_rmse_m)),
            optional_number(metrics.map(|m| m.ego.yaw_rmse_rad)),
            optional_number(metrics.map(|m| m.tracks_with_estimated_ego.position_rmse_m)),
            optional_number(metrics.map(|m| m.tracks_with_truth_ego.position_rmse_m)),
            optional_number(metrics.map(|m| m.estimated_ego_position_rmse_delta_m)),
            csv_field(case.error.as_deref().unwrap_or("")),
        ));
    }
    fs::write(report_dir.join("results.csv"), csv)?;

    let mut summary = format!(
        "# {}\n\nCases: {}  \nSuccessful: {}  \nFailed: {}\n\n## Parameter groups\n\n| Parameters | Runs | Failed | Ego RMSE mean ± stddev (m) | Object RMSE mean ± stddev (m) | Ego cost in tracks (m) |\n| --- | ---: | ---: | ---: | ---: | ---: |\n",
        report.name, report.case_count, report.successful_cases, report.failed_cases
    );
    for group in &report.groups {
        summary.push_str(&format!(
            "| {} | {} | {} | {} | {} | {} |\n",
            markdown_parameters(&group.parameters),
            group.cases,
            group.failed_cases,
            display_mean_stddev(
                group.mean_ego_position_rmse_m,
                group.stddev_ego_position_rmse_m
            ),
            display_mean_stddev(
                group.mean_estimated_ego_track_rmse_m,
                group.stddev_estimated_ego_track_rmse_m
            ),
            display_number(group.mean_track_ego_cost_m),
        ));
        if let Some(warning) = &group.sample_warning {
            summary.push_str(&format!(
                "\n> {} {}\n",
                markdown_parameters(&group.parameters),
                warning
            ));
        }
    }
    summary.push_str(
        "\nEach `case-NNNN` directory is a normal experiment bundle. Open an interesting case with `fusion view <case-directory>`.\n",
    );
    fs::write(report_dir.join("summary.md"), summary)?;
    Ok(())
}

fn default_seeds() -> Vec<u64> {
    vec![1]
}

fn optional_number(value: Option<f64>) -> String {
    value.map(|number| number.to_string()).unwrap_or_default()
}

fn display_number(value: Option<f64>) -> String {
    value
        .map(|number| format!("{number:.6}"))
        .unwrap_or_else(|| "—".to_owned())
}

fn display_mean_stddev(mean: Option<f64>, stddev: Option<f64>) -> String {
    match (mean, stddev) {
        (Some(mean), Some(stddev)) => format!("{mean:.6} ± {stddev:.6}"),
        (Some(mean), None) => format!("{mean:.6} ± —"),
        _ => "—".to_owned(),
    }
}

fn csv_field(value: &str) -> String {
    format!("\"{}\"", value.replace('"', "\"\""))
}

fn markdown_parameters(parameters: &BTreeMap<String, Value>) -> String {
    parameters
        .iter()
        .map(|(path, value)| {
            let rendered = serde_yaml_ng::to_string(value)
                .unwrap_or_else(|_| "?".to_owned())
                .trim()
                .to_owned();
            format!("`{path}={rendered}`")
        })
        .collect::<Vec<_>>()
        .join("<br>")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dotted_paths_update_mappings_and_sequences() {
        let mut value: Value = serde_yaml_ng::from_str(
            "camera:\n  rate_hz: 10.0\ntrajectory:\n  - yaw_rate_radps: 0.1\n",
        )
        .unwrap();
        set_path(&mut value, "camera.rate_hz", Value::Number(5.into())).unwrap();
        set_path(
            &mut value,
            "trajectory.0.yaw_rate_radps",
            serde_yaml_ng::from_str("0.5").unwrap(),
        )
        .unwrap();
        assert_eq!(value["camera"]["rate_hz"].as_i64(), Some(5));
        assert_eq!(value["trajectory"][0]["yaw_rate_radps"].as_f64(), Some(0.5));
    }

    #[test]
    fn parameter_expansion_is_a_cartesian_product() {
        let parameters = BTreeMap::from([
            (
                "camera.rate_hz".to_owned(),
                vec![Value::Number(5.into()), Value::Number(10.into())],
            ),
            (
                "motion_speed_factor".to_owned(),
                vec![
                    serde_yaml_ng::from_str("0.5").unwrap(),
                    Value::Number(1.into()),
                ],
            ),
        ]);
        let expanded = expand_parameters(&parameters);
        assert_eq!(expanded.len(), 4);
        assert!(expanded.iter().all(|case| case.len() == 2));
    }
}
