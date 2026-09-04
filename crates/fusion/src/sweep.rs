use std::{
    collections::BTreeMap,
    fs,
    path::{Path, PathBuf},
};

use anyhow::{Context, Result, bail, ensure};
use serde::{Deserialize, Serialize};
use serde_yaml_ng::Value;

use crate::{eval::Metrics, run_resolved_experiment, scenario};

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
    pub warnings: Vec<String>,
    pub cases: Vec<SweepCaseResult>,
    pub groups: Vec<SweepGroup>,
}

#[derive(Debug, Clone, Serialize)]
pub struct SweepCaseResult {
    pub case_id: String,
    pub run_id: String,
    pub root_seed: u64,
    pub parameters: BTreeMap<String, Value>,
    pub status: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub metrics: Option<Metrics>,
}

#[derive(Debug, Clone, Serialize)]
pub struct SweepGroup {
    pub parameters: BTreeMap<String, Value>,
    pub cases: usize,
    pub successful_cases: usize,
    pub failed_cases: usize,
    pub position_rmse_m: Option<SweepStatistic>,
    pub yaw_rmse_rad: Option<SweepStatistic>,
    pub final_position_error_m: Option<SweepStatistic>,
    pub valid_output_fraction: Option<SweepStatistic>,
    pub normalized_anees: Option<SweepStatistic>,
    pub x_coverage_95_fraction: Option<SweepStatistic>,
    pub y_coverage_95_fraction: Option<SweepStatistic>,
    pub yaw_coverage_95_fraction: Option<SweepStatistic>,
    pub forward_speed_coverage_95_fraction: Option<SweepStatistic>,
}

#[derive(Debug, Clone, Serialize)]
pub struct SweepStatistic {
    pub sample_count: usize,
    pub mean: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sample_standard_deviation: Option<f64>,
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
        let run_id = case.scenario.run_id.clone();
        match run_resolved_experiment(&case.scenario, &base_path, &case_path, false) {
            Ok(metrics) => results.push(SweepCaseResult {
                case_id: case.case_id,
                run_id,
                root_seed: case.root_seed,
                parameters: case.parameters,
                status: "COMPLETE".to_owned(),
                error: None,
                metrics: Some(metrics),
            }),
            Err(error) => results.push(SweepCaseResult {
                case_id: case.case_id,
                run_id,
                root_seed: case.root_seed,
                parameters: case.parameters,
                status: "FAILED".to_owned(),
                error: Some(format!("{error:#}")),
                metrics: None,
            }),
        }
    }

    let successful_cases = results.iter().filter(|case| case.metrics.is_some()).count();
    let warnings = (spec.seeds.len() == 1)
        .then(|| {
            "The sweep uses one seed; each group is a single realization, so dispersion is unavailable."
                .to_owned()
        })
        .into_iter()
        .collect();
    let report = SweepReport {
        name: spec.name,
        base_scenario: base_path.display().to_string(),
        case_count: results.len(),
        successful_cases,
        failed_cases: results.len() - successful_cases,
        warnings,
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
            let mut resolved: scenario::ResolvedScenario = serde_yaml_ng::from_value(configured)
                .with_context(|| format!("invalid resolved sweep {case_id}"))?;
            resolved.run_id = format!("{}-{case_id}-seed-{seed}", resolved.run_id);
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
        ensure!(
            path != "root_seed" && path != "run_id",
            "{path} is managed by the sweep runner"
        );
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
            let statistic = |value: fn(&Metrics) -> f64| {
                summarize(successful.iter().map(|metrics| value(metrics)).collect())
            };
            let optional_statistic = |value: fn(&Metrics) -> Option<f64>| {
                summarize(
                    successful
                        .iter()
                        .filter_map(|metrics| value(metrics))
                        .collect(),
                )
            };
            SweepGroup {
                parameters: cases[0].parameters.clone(),
                cases: cases.len(),
                successful_cases: count,
                failed_cases: cases.len() - count,
                position_rmse_m: optional_statistic(|m| m.position_rmse_m),
                yaw_rmse_rad: optional_statistic(|m| m.yaw_rmse_rad),
                final_position_error_m: optional_statistic(|m| m.final_position_error_m),
                valid_output_fraction: statistic(|m| m.valid_output_fraction),
                normalized_anees: optional_statistic(|metrics| {
                    metrics
                        .covariance_consistency
                        .as_ref()
                        .map(|consistency| consistency.normalized_anees)
                }),
                x_coverage_95_fraction: optional_statistic(|metrics| {
                    metrics
                        .covariance_consistency
                        .as_ref()
                        .map(|consistency| consistency.marginal_coverage_95.x_fraction)
                }),
                y_coverage_95_fraction: optional_statistic(|metrics| {
                    metrics
                        .covariance_consistency
                        .as_ref()
                        .map(|consistency| consistency.marginal_coverage_95.y_fraction)
                }),
                yaw_coverage_95_fraction: optional_statistic(|metrics| {
                    metrics
                        .covariance_consistency
                        .as_ref()
                        .map(|consistency| consistency.marginal_coverage_95.yaw_fraction)
                }),
                forward_speed_coverage_95_fraction: optional_statistic(|metrics| {
                    metrics
                        .covariance_consistency
                        .as_ref()
                        .map(|consistency| consistency.marginal_coverage_95.forward_speed_fraction)
                }),
            }
        })
        .collect()
}

fn summarize(values: Vec<f64>) -> Option<SweepStatistic> {
    if values.is_empty() {
        return None;
    }
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    let sample_standard_deviation = (values.len() > 1).then(|| {
        let variance = values
            .iter()
            .map(|value| (value - mean).powi(2))
            .sum::<f64>()
            / (values.len() - 1) as f64;
        variance.sqrt()
    });
    Some(SweepStatistic {
        sample_count: values.len(),
        mean,
        sample_standard_deviation,
    })
}

fn write_reports(output: &Path, report: &SweepReport) -> Result<()> {
    let report_dir = output.join("reports");
    fs::write(
        report_dir.join("results.json"),
        serde_json::to_vec_pretty(report)?,
    )?;

    let mut csv = String::from(
        "case_id,status,root_seed,parameters,position_rmse_m,yaw_rmse_rad,final_position_error_m,valid_output_fraction,valid_output_count,initializing_output_count,diverged_output_count,unspecified_output_count,unknown_status_output_count,unmatched_valid_output_count,anees,normalized_anees,x_coverage_95_fraction,y_coverage_95_fraction,yaw_coverage_95_fraction,forward_speed_coverage_95_fraction,error\n",
    );
    for case in &report.cases {
        let metrics = case.metrics.as_ref();
        csv.push_str(&format!(
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n",
            csv_field(&case.case_id),
            csv_field(&case.status),
            case.root_seed,
            csv_field(&serde_json::to_string(&case.parameters)?),
            optional_number(metrics.and_then(|m| m.position_rmse_m)),
            optional_number(metrics.and_then(|m| m.yaw_rmse_rad)),
            optional_number(metrics.and_then(|m| m.final_position_error_m)),
            optional_number(metrics.map(|m| m.valid_output_fraction)),
            optional_usize(metrics.map(|m| m.valid_output_count)),
            optional_usize(metrics.map(|m| m.initializing_output_count)),
            optional_usize(metrics.map(|m| m.diverged_output_count)),
            optional_usize(metrics.map(|m| m.unspecified_output_count)),
            optional_usize(metrics.map(|m| m.unknown_status_output_count)),
            optional_usize(metrics.map(|m| m.unmatched_valid_output_count)),
            optional_number(metrics.and_then(|m| {
                m.covariance_consistency
                    .as_ref()
                    .map(|consistency| consistency.anees)
            })),
            optional_number(metrics.and_then(|m| {
                m.covariance_consistency
                    .as_ref()
                    .map(|consistency| consistency.normalized_anees)
            })),
            optional_number(metrics.and_then(|m| {
                m.covariance_consistency
                    .as_ref()
                    .map(|consistency| consistency.marginal_coverage_95.x_fraction)
            })),
            optional_number(metrics.and_then(|m| {
                m.covariance_consistency
                    .as_ref()
                    .map(|consistency| consistency.marginal_coverage_95.y_fraction)
            })),
            optional_number(metrics.and_then(|m| {
                m.covariance_consistency
                    .as_ref()
                    .map(|consistency| consistency.marginal_coverage_95.yaw_fraction)
            })),
            optional_number(metrics.and_then(|m| {
                m.covariance_consistency
                    .as_ref()
                    .map(|consistency| consistency.marginal_coverage_95.forward_speed_fraction)
            })),
            csv_field(case.error.as_deref().unwrap_or("")),
        ));
    }
    fs::write(report_dir.join("results.csv"), csv)?;

    let mut summary = format!(
        "# {}\n\n- Cases: {}\n- Successful: {}\n- Failed: {}\n",
        report.name, report.case_count, report.successful_cases, report.failed_cases
    );
    for warning in &report.warnings {
        summary.push_str(&format!("- Warning: {warning}\n"));
    }
    summary.push_str(
        "\n## Parameter groups\n\n| Parameters | Runs | Failed | Position RMSE (m) | Yaw RMSE (rad) | Normalized ANEES | 95% coverage x/y/yaw/speed | Valid outputs |\n| --- | ---: | ---: | ---: | ---: | ---: | --- | ---: |\n",
    );
    for group in &report.groups {
        summary.push_str(&format!(
            "| {} | {} | {} | {} | {} | {} | {} | {} |\n",
            markdown_parameters(&group.parameters),
            group.cases,
            group.failed_cases,
            display_statistic(group.position_rmse_m.as_ref(), false),
            display_statistic(group.yaw_rmse_rad.as_ref(), false),
            display_statistic(group.normalized_anees.as_ref(), false),
            display_coverage(group),
            display_statistic(group.valid_output_fraction.as_ref(), true),
        ));
    }
    summary.push_str(
        "\nValues are the mean ± sample standard deviation across successful runs. A group with one successful run is labeled `n=1` instead of showing dispersion. Every parameter group uses the same configured seed set. `results.csv` retains `root_seed` for later paired analysis. Normalized ANEES has expected mean 1.0; each marginal coverage target is 95%.\n\nEach `case-NNNN` directory is a normal experiment bundle. Open an interesting case with `fusion view <case-directory>`.\n",
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

fn optional_usize(value: Option<usize>) -> String {
    value.map(|number| number.to_string()).unwrap_or_default()
}

fn display_statistic(statistic: Option<&SweepStatistic>, percentage: bool) -> String {
    let Some(statistic) = statistic else {
        return "—".to_owned();
    };
    if percentage {
        match statistic.sample_standard_deviation {
            Some(standard_deviation) => format!(
                "{:.1}% ± {:.1}%",
                statistic.mean * 100.0,
                standard_deviation * 100.0
            ),
            None => format!("{:.1}% (`n=1`)", statistic.mean * 100.0),
        }
    } else {
        match statistic.sample_standard_deviation {
            Some(standard_deviation) => {
                format!("{:.6} ± {:.6}", statistic.mean, standard_deviation)
            }
            None => format!("{:.6} (`n=1`)", statistic.mean),
        }
    }
}

fn display_coverage(group: &SweepGroup) -> String {
    [
        group.x_coverage_95_fraction.as_ref(),
        group.y_coverage_95_fraction.as_ref(),
        group.yaw_coverage_95_fraction.as_ref(),
        group.forward_speed_coverage_95_fraction.as_ref(),
    ]
    .map(|statistic| display_statistic(statistic, true))
    .join(" / ")
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

    #[test]
    fn summary_uses_sample_standard_deviation() {
        let statistic = summarize(vec![1.0, 2.0, 3.0]).unwrap();
        assert_eq!(statistic.sample_count, 3);
        assert_eq!(statistic.mean, 2.0);
        assert_eq!(statistic.sample_standard_deviation, Some(1.0));

        let single = summarize(vec![4.0]).unwrap();
        assert_eq!(single.sample_count, 1);
        assert_eq!(single.sample_standard_deviation, None);
    }
}
