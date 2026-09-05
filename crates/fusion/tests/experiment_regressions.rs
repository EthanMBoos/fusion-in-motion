use std::{
    collections::BTreeMap,
    fs,
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{Context, Result, ensure};
use fusion_in_motion::{
    estimator::{GpsDiagnostics, TimingDiagnostics},
    eval::RunMetrics,
    scenario::{self, ResolvedScenario},
};
use serde::Deserialize;
use serde_json::Value as JsonValue;
use serde_yaml_ng::Value as YamlValue;

const RUN_FILES: &[&str] = &[
    "scenario.resolved.yaml",
    "measurements.mcap",
    "truth.mcap",
    "estimates/ego-baseline.mcap",
    "tracks/estimated-ego.mcap",
    "tracks/truth-ego.mcap",
    "reports/baseline/metrics.json",
    "reports/baseline/summary.md",
];

#[derive(Debug, Deserialize)]
struct RunReport {
    metrics: RunMetrics,
    ego_timing: TimingDiagnostics,
    gps_fixes: GpsDiagnostics,
}

#[derive(Debug, Deserialize)]
struct SweepReport {
    case_count: usize,
    successful_cases: usize,
    failed_cases: usize,
    cases: Vec<SweepCase>,
    groups: Vec<SweepGroup>,
}

#[derive(Debug, Deserialize)]
struct SweepCase {
    case_id: String,
    root_seed: u64,
    parameters: BTreeMap<String, YamlValue>,
    status: String,
    error: Option<String>,
    metrics: Option<RunMetrics>,
}

#[derive(Debug, Deserialize)]
struct SweepGroup {
    parameters: BTreeMap<String, YamlValue>,
    mean_ego_position_rmse_m: Option<f64>,
    mean_truth_ego_track_rmse_m: Option<f64>,
    mean_truth_ego_track_time_coverage_fraction: Option<f64>,
}

#[derive(Debug, Deserialize)]
struct Baselines {
    relative_tolerance_fraction: f64,
    experiments: BTreeMap<String, JsonValue>,
    sweeps: BTreeMap<String, JsonValue>,
}

#[derive(Debug)]
struct CompletedRun {
    report: RunReport,
    raw_report: JsonValue,
}

fn repository_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn read_baselines() -> Result<Baselines> {
    let path =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/experiment_baselines.json");
    serde_json::from_slice(&fs::read(&path)?)
        .with_context(|| format!("invalid experiment baselines {}", path.display()))
}

fn experiment_files() -> Result<Vec<PathBuf>> {
    let mut paths = fs::read_dir(repository_root().join("experiments"))?
        .map(|entry| Ok(entry?.path()))
        .collect::<Result<Vec<_>>>()?;
    paths.retain(|path| path.extension().and_then(|value| value.to_str()) == Some("yaml"));
    paths.sort();
    Ok(paths)
}

fn is_sweep(path: &Path) -> Result<bool> {
    let source = fs::read_to_string(path)?;
    let value: YamlValue = serde_yaml_ng::from_str(&source)?;
    Ok(value
        .as_mapping()
        .is_some_and(|mapping| mapping.contains_key("base_scenario")))
}

fn run(command: &mut Command) -> Result<()> {
    let description = format!("{command:?}");
    let output = command
        .output()
        .with_context(|| format!("failed to start {description}"))?;
    ensure!(
        output.status.success(),
        "{description} failed\nstdout:\n{}\nstderr:\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
    Ok(())
}

fn check_run_files(run: &Path, include_visualization: bool) -> Result<()> {
    for relative in RUN_FILES {
        let path = run.join(relative);
        ensure!(path.is_file(), "{} was not written", path.display());
    }
    if include_visualization {
        let path = run.join("reports/baseline/visualization.rrd");
        ensure!(path.is_file(), "{} was not written", path.display());
    }
    Ok(())
}

fn read_run_report(run: &Path) -> Result<(RunReport, JsonValue)> {
    let path = run.join("reports/baseline/metrics.json");
    let bytes = fs::read(&path)?;
    let report = serde_json::from_slice(&bytes)
        .with_context(|| format!("invalid run report {}", path.display()))?;
    let raw = serde_json::from_slice(&bytes)?;
    Ok((report, raw))
}

fn run_scenario(name: &str, scenario: &Path, output: &Path) -> Result<CompletedRun> {
    run(Command::new(env!("CARGO_BIN_EXE_fusion"))
        .arg("run")
        .arg(scenario)
        .arg("--output")
        .arg(output))
    .with_context(|| format!("experiment {name} failed"))?;
    check_run_files(output, true)
        .with_context(|| format!("experiment {name} wrote an incomplete run"))?;
    let (report, raw_report) = read_run_report(output)?;
    check_run_health(name, &report.metrics)?;
    Ok(CompletedRun { report, raw_report })
}

fn run_variant(name: &str, scenario: &ResolvedScenario, directory: &Path) -> Result<CompletedRun> {
    let path = directory.join(format!("{name}.yaml"));
    fs::write(&path, scenario::canonical_yaml(scenario)?)?;
    let output = directory.join(name);
    run(Command::new(env!("CARGO_BIN_EXE_fusion"))
        .arg("run")
        .arg(&path)
        .arg("--output")
        .arg(&output))
    .with_context(|| format!("control {name} failed"))?;
    check_run_files(&output, true)
        .with_context(|| format!("control {name} wrote an incomplete run"))?;
    let (report, raw_report) = read_run_report(&output)?;
    Ok(CompletedRun { report, raw_report })
}

fn check_ego_health(name: &str, metrics: &RunMetrics) -> Result<()> {
    ensure!(
        metrics.ego.invalid_output_count == 0,
        "{name}: {} invalid vehicle estimates",
        metrics.ego.invalid_output_count
    );
    ensure!(
        metrics.ego.matched_samples == metrics.ego.estimate_samples,
        "{name}: only {} of {} vehicle estimates matched truth",
        metrics.ego.matched_samples,
        metrics.ego.estimate_samples
    );
    ensure!(
        metrics.ego.position_rmse_m.is_finite() && metrics.ego.yaw_rmse_rad.is_finite(),
        "{name}: vehicle error is not finite"
    );
    Ok(())
}

fn check_run_health(name: &str, metrics: &RunMetrics) -> Result<()> {
    check_ego_health(name, metrics)?;
    for tracks in [
        &metrics.tracks_with_estimated_ego,
        &metrics.tracks_with_truth_ego,
    ] {
        ensure!(
            tracks.invalid_output_count == 0,
            "{name}: {} tracker produced {} invalid outputs",
            tracks.ego_source,
            tracks.invalid_output_count
        );
        ensure!(
            tracks.position_rmse_m.is_none_or(f64::is_finite)
                && tracks.time_coverage_fraction.is_finite(),
            "{name}: {} tracker error is not finite",
            tracks.ego_source
        );
    }
    Ok(())
}

fn check_close(label: &str, actual: f64, expected: f64, relative_tolerance: f64) -> Result<()> {
    let difference = (actual - expected).abs();
    let allowed = (expected.abs() * relative_tolerance).max(1.0e-9);
    ensure!(
        actual.is_finite() && difference <= allowed,
        "{label} changed: baseline {expected:.6}, current {actual:.6}, difference {difference:.6}, allowed {allowed:.6}"
    );
    Ok(())
}

fn check_baseline(
    label: &str,
    actual: &JsonValue,
    expected: &JsonValue,
    relative_tolerance: f64,
) -> Result<()> {
    match expected {
        JsonValue::Object(expected) => {
            let actual = actual
                .as_object()
                .with_context(|| format!("{label} is not an object"))?;
            for (key, expected) in expected {
                check_baseline(
                    &format!("{label}.{key}"),
                    actual
                        .get(key)
                        .with_context(|| format!("{label} is missing {key}"))?,
                    expected,
                    relative_tolerance,
                )?;
            }
            Ok(())
        }
        JsonValue::Array(expected) => {
            let actual = actual
                .as_array()
                .with_context(|| format!("{label} is not an array"))?;
            ensure!(
                actual.len() == expected.len(),
                "{label} length changed: baseline {}, current {}",
                expected.len(),
                actual.len()
            );
            for (index, (actual, expected)) in actual.iter().zip(expected).enumerate() {
                check_baseline(
                    &format!("{label}[{index}]"),
                    actual,
                    expected,
                    relative_tolerance,
                )?;
            }
            Ok(())
        }
        JsonValue::Number(expected) if expected.is_f64() => check_close(
            label,
            actual
                .as_f64()
                .with_context(|| format!("{label} is not a number"))?,
            expected.as_f64().unwrap(),
            relative_tolerance,
        ),
        _ => {
            ensure!(
                actual == expected,
                "{label} changed: baseline {expected}, current {actual}"
            );
            Ok(())
        }
    }
}

fn read_sweep_report(output: &Path) -> Result<(SweepReport, JsonValue)> {
    let path = output.join("reports/results.json");
    let bytes = fs::read(&path)?;
    let report = serde_json::from_slice(&bytes)
        .with_context(|| format!("invalid sweep report {}", path.display()))?;
    let raw = serde_json::from_slice(&bytes)?;
    Ok((report, raw))
}

fn check_sweep_completed(name: &str, output: &Path, report: &SweepReport) -> Result<()> {
    let failures = report
        .cases
        .iter()
        .filter(|case| case.status == "FAILED")
        .map(|case| {
            format!(
                "{}: {}",
                case.case_id,
                case.error.as_deref().unwrap_or("no error reported")
            )
        })
        .collect::<Vec<_>>()
        .join("\n");
    ensure!(report.case_count > 0, "{name} expanded to no cases");
    ensure!(
        report.successful_cases == report.case_count && report.failed_cases == 0,
        "{name} completed {} of {} cases\n{failures}",
        report.successful_cases,
        report.case_count
    );
    for case in &report.cases {
        let metrics = case
            .metrics
            .as_ref()
            .with_context(|| format!("{name} {} has no metrics", case.case_id))?;
        check_ego_health(&format!("{name} {}", case.case_id), metrics)?;
        check_run_files(&output.join(&case.case_id), false)
            .with_context(|| format!("{name} {} wrote an incomplete run", case.case_id))?;
    }
    Ok(())
}

fn number(parameters: &BTreeMap<String, YamlValue>, name: &str) -> Option<f64> {
    parameters.get(name).and_then(YamlValue::as_f64)
}

fn boolean(parameters: &BTreeMap<String, YamlValue>, name: &str) -> Option<bool> {
    parameters.get(name).and_then(YamlValue::as_bool)
}

fn check_initial_lesson(run: &RunReport) -> Result<()> {
    ensure!(
        run.metrics.tracks_with_truth_ego.position_rmse_m
            < run.metrics.tracks_with_estimated_ego.position_rmse_m,
        "initial.yaml no longer shows vehicle error carrying into object tracks"
    );
    Ok(())
}

fn check_outlier_lesson(runs: &BTreeMap<String, CompletedRun>, temp: &Path) -> Result<()> {
    let gated = &runs["outliers"].report;
    ensure!(
        gated.gps_fixes.accepted_fixes > 0 && gated.gps_fixes.rejected_fixes > 0,
        "outliers.yaml must accept useful fixes and reject bad fixes"
    );
    let mut scenario =
        scenario::load_and_resolve(&repository_root().join("experiments/outliers.yaml"))?;
    scenario.ego_estimator.gps_gate_sigma = 1.0e6;
    let ungated = run_variant("outliers-without-gating", &scenario, temp)?;
    ensure!(
        gated.metrics.ego.position_rmse_m < ungated.report.metrics.ego.position_rmse_m,
        "outliers.yaml gating no longer improves vehicle position RMSE: {:.3} m gated, {:.3} m ungated",
        gated.metrics.ego.position_rmse_m,
        ungated.report.metrics.ego.position_rmse_m
    );
    Ok(())
}

fn check_timing_lesson(runs: &BTreeMap<String, CompletedRun>, temp: &Path) -> Result<()> {
    let compensated = &runs["timing"].report;
    ensure!(
        compensated.ego_timing.delayed_measurements > 0
            && compensated.ego_timing.replayed_measurements > 0
            && compensated.ego_timing.discarded_measurements == 0,
        "timing.yaml no longer reorders delayed GPS measurements"
    );
    let mut scenario =
        scenario::load_and_resolve(&repository_root().join("experiments/timing.yaml"))?;
    scenario.ego_estimator.timing_compensation = false;
    let arrival_order = run_variant("timing-without-compensation", &scenario, temp)?;
    ensure!(
        compensated.metrics.ego.position_rmse_m < arrival_order.report.metrics.ego.position_rmse_m,
        "timing.yaml compensation no longer improves vehicle position RMSE: {:.3} m compensated, {:.3} m arrival order",
        compensated.metrics.ego.position_rmse_m,
        arrival_order.report.metrics.ego.position_rmse_m
    );
    Ok(())
}

fn localization_rmse(report: &SweepReport, rate: f64, noise: f64) -> Result<f64> {
    report
        .groups
        .iter()
        .find(|group| {
            number(&group.parameters, "gps.rate_hz") == Some(rate)
                && number(&group.parameters, "gps.horizontal_position_stddev_m") == Some(noise)
        })
        .and_then(|group| group.mean_ego_position_rmse_m)
        .with_context(|| format!("missing localization group for {rate} Hz and {noise} m noise"))
}

fn check_localization_sweep(report: &SweepReport) -> Result<()> {
    for rate in [1.0, 2.0, 5.0] {
        let clean = localization_rmse(report, rate, 0.1)?;
        let medium = localization_rmse(report, rate, 0.5)?;
        let noisy = localization_rmse(report, rate, 1.0)?;
        ensure!(
            clean < medium && medium < noisy,
            "localization_sweep.yaml no longer shows increasing error as GPS gets noisier at {rate} Hz: {clean:.3}, {medium:.3}, {noisy:.3} m"
        );
    }
    for noise in [0.1, 0.5, 1.0] {
        let slow = localization_rmse(report, 1.0, noise)?;
        let medium = localization_rmse(report, 2.0, noise)?;
        let fast = localization_rmse(report, 5.0, noise)?;
        ensure!(
            fast < medium && medium < slow,
            "localization_sweep.yaml no longer shows decreasing error as GPS gets faster at {noise} m noise: {slow:.3}, {medium:.3}, {fast:.3} m"
        );
    }
    Ok(())
}

fn check_perception_sweep(report: &SweepReport) -> Result<()> {
    let group = |camera, lidar| {
        report.groups.iter().find(|group| {
            boolean(&group.parameters, "camera.enabled") == Some(camera)
                && boolean(&group.parameters, "lidar.enabled") == Some(lidar)
        })
    };
    let camera_only = group(true, false).context("missing camera-only group")?;
    let lidar_only = group(false, true).context("missing lidar-only group")?;
    let both = group(true, true).context("missing camera-and-lidar group")?;
    ensure!(
        camera_only.mean_truth_ego_track_rmse_m.is_none()
            && camera_only.mean_truth_ego_track_time_coverage_fraction == Some(0.0),
        "perception_sweep.yaml camera-only case unexpectedly created a metric track"
    );
    ensure!(
        both.mean_truth_ego_track_rmse_m < lidar_only.mean_truth_ego_track_rmse_m,
        "perception_sweep.yaml no longer shows lower object error from camera and lidar together"
    );
    ensure!(
        both.mean_truth_ego_track_time_coverage_fraction
            > lidar_only.mean_truth_ego_track_time_coverage_fraction,
        "perception_sweep.yaml no longer shows better coverage from camera and lidar together"
    );

    for seed in 10..=14 {
        let case = |camera, lidar| {
            report.cases.iter().find(|case| {
                case.root_seed == seed
                    && boolean(&case.parameters, "camera.enabled") == Some(camera)
                    && boolean(&case.parameters, "lidar.enabled") == Some(lidar)
            })
        };
        let camera_only = case(true, false).context("missing camera-only case")?;
        let lidar_only = case(false, true)
            .and_then(|case| case.metrics.as_ref())
            .context("missing lidar-only metrics")?;
        let both = case(true, true)
            .and_then(|case| case.metrics.as_ref())
            .context("missing camera-and-lidar metrics")?;
        ensure!(
            camera_only
                .metrics
                .as_ref()
                .is_some_and(|metrics| metrics.tracks_with_truth_ego.position_rmse_m.is_none()),
            "perception_sweep.yaml camera-only seed {seed} unexpectedly created a metric track"
        );
        for metrics in [lidar_only, both] {
            ensure!(
                metrics.tracks_with_truth_ego.invalid_output_count == 0,
                "perception_sweep.yaml produced invalid tracks for seed {seed}"
            );
        }
        ensure!(
            lidar_only.ego.position_rmse_m == both.ego.position_rmse_m,
            "perception sensor settings changed ego localization for seed {seed}"
        );
        ensure!(
            both.tracks_with_truth_ego.position_rmse_m
                < lidar_only.tracks_with_truth_ego.position_rmse_m,
            "camera and lidar together no longer beat lidar alone for seed {seed}"
        );
    }
    Ok(())
}

#[test]
fn checked_in_experiments_keep_their_results() -> Result<()> {
    let baselines = read_baselines()?;
    ensure!(
        baselines.relative_tolerance_fraction > 0.0,
        "experiment baseline tolerance must be positive"
    );
    let (sweeps, scenarios): (Vec<_>, Vec<_>) = experiment_files()?
        .into_iter()
        .map(|path| Ok((is_sweep(&path)?, path)))
        .collect::<Result<Vec<_>>>()?
        .into_iter()
        .partition(|(is_sweep, _)| *is_sweep);

    let temp = tempfile::tempdir()?;
    let mut runs = BTreeMap::new();
    for (_, scenario) in scenarios {
        let name = scenario.file_stem().unwrap().to_string_lossy().into_owned();
        let completed = run_scenario(&name, &scenario, &temp.path().join(&name))?;
        check_baseline(
            &name,
            &completed.raw_report,
            baselines
                .experiments
                .get(&name)
                .with_context(|| format!("missing numerical baseline for {name}.yaml"))?,
            baselines.relative_tolerance_fraction,
        )?;
        runs.insert(name, completed);
    }
    ensure!(
        runs.len() == baselines.experiments.len(),
        "experiment baselines contain a scenario that was not run"
    );

    check_initial_lesson(&runs["initial"].report)?;
    check_outlier_lesson(&runs, temp.path())?;
    check_timing_lesson(&runs, temp.path())?;

    let mut sweep_reports = BTreeMap::new();
    for (_, sweep) in sweeps {
        let name = sweep.file_stem().unwrap().to_string_lossy().into_owned();
        let output = temp.path().join(&name);
        run(Command::new(env!("CARGO_BIN_EXE_fusion"))
            .arg("sweep")
            .arg(&sweep)
            .arg("--output")
            .arg(&output))
        .with_context(|| format!("sweep {name} failed"))?;
        let (report, raw_report) = read_sweep_report(&output)?;
        check_sweep_completed(&name, &output, &report)?;
        check_baseline(
            &name,
            &raw_report,
            baselines
                .sweeps
                .get(&name)
                .with_context(|| format!("missing numerical baseline for {name}.yaml"))?,
            baselines.relative_tolerance_fraction,
        )?;
        sweep_reports.insert(name, report);
    }
    ensure!(
        sweep_reports.len() == baselines.sweeps.len(),
        "sweep baselines contain a sweep that was not run"
    );

    check_localization_sweep(&sweep_reports["localization_sweep"])?;
    check_perception_sweep(&sweep_reports["perception_sweep"])?;
    Ok(())
}
