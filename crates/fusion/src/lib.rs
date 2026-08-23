pub mod bundle;
pub mod compare;
pub mod estimator;
pub mod eval;
pub mod external;
pub mod math;
pub mod random;
pub mod scenario;
pub mod sensor;
pub mod sweep;
pub mod tracker;
pub mod truth;
pub mod viz;

use std::path::{Path, PathBuf};

use anyhow::Result;
use fusion_schema::messages::EgoSource;

use crate::{
    bundle::{BundleManifest, GeneratedRun, MeasurementRecord},
    estimator::{EgoMeasurement, run_baseline},
    scenario::ResolvedScenario,
    tracker::{EgoHistory, PerceptionMeasurement},
};

pub fn resolve_scenario(path: &Path) -> Result<ResolvedScenario> {
    scenario::load_and_resolve(path)
}

pub fn run_experiment(scenario_path: &Path, output: &Path) -> Result<PathBuf> {
    let scenario = resolve_scenario(scenario_path)?;
    run_resolved_experiment(&scenario, output, true)?;
    Ok(output.to_path_buf())
}

pub fn run_numbered_experiment(scenario_path: &Path, output: Option<&Path>) -> Result<PathBuf> {
    let output = output
        .map(Path::to_path_buf)
        .unwrap_or_else(|| next_run_path(Path::new("runs")));
    run_experiment(scenario_path, &output)
}

pub fn next_run_path(root: &Path) -> PathBuf {
    for number in 1_u64.. {
        let candidate = root.join(format!("run{number:03}"));
        if !candidate.exists() {
            return candidate;
        }
    }
    unreachable!("run number overflow")
}

pub(crate) fn run_resolved_experiment(
    scenario: &ResolvedScenario,
    output: &Path,
    build_visualization: bool,
) -> Result<eval::RunMetrics> {
    let mut manifest = BundleManifest::start(scenario);
    bundle::prepare(output, scenario)?;
    manifest.write(output)?;
    let result = (|| {
        let generated = sensor::generate(scenario)?;
        bundle::write_generated(output, &generated)?;
        manifest.record_generated(output)?;

        let replayed = bundle::read_measurements(&output.join("measurements.mcap"))?;
        let ego_measurements = replayed
            .iter()
            .filter_map(|record| match record {
                MeasurementRecord::Imu(value) => Some(EgoMeasurement::Imu(value.clone())),
                MeasurementRecord::Gps(value) => Some(EgoMeasurement::Gps(value.clone())),
                _ => None,
            })
            .collect::<Vec<_>>();
        let perception_measurements = replayed
            .iter()
            .filter_map(|record| match record {
                MeasurementRecord::Camera(value) => {
                    Some(PerceptionMeasurement::Camera(value.clone()))
                }
                MeasurementRecord::Lidar(value) => {
                    Some(PerceptionMeasurement::Lidar(value.clone()))
                }
                _ => None,
            })
            .collect::<Vec<_>>();

        let ego_run = run_baseline(
            &scenario.ego_estimator,
            &scenario.imu,
            &scenario.gps,
            &ego_measurements,
        )?;
        bundle::write_ego_estimates(output, &ego_run.estimates)?;

        let estimated_history = EgoHistory::from_estimates(&ego_run.estimates)?;
        let estimated_tracker = tracker::run(
            &scenario.object_tracker,
            &scenario.camera,
            &scenario.lidar,
            &perception_measurements,
            &estimated_history,
            EgoSource::Estimated,
            &scenario.platform.world_frame,
        )?;
        bundle::write_tracks(output, "estimated-ego", &estimated_tracker.tracks)?;

        let truth_path = output.join("truth.mcap");
        let ego_truth = bundle::read_ego_truth(&truth_path)?;
        let truth_history = EgoHistory::from_truth(&ego_truth)?;
        let truth_tracker = tracker::run(
            &scenario.object_tracker,
            &scenario.camera,
            &scenario.lidar,
            &perception_measurements,
            &truth_history,
            EgoSource::Truth,
            &scenario.platform.world_frame,
        )?;
        anyhow::ensure!(
            estimated_tracker.processed_detection_ids == truth_tracker.processed_detection_ids,
            "tracker comparison did not use identical detections"
        );
        bundle::write_tracks(output, "truth-ego", &truth_tracker.tracks)?;
        manifest.record_outputs(output)?;

        let object_truth = bundle::read_object_truth(&truth_path)?;
        let observation_truth = bundle::read_observation_truth(&truth_path)?;
        let metrics = eval::evaluate(
            scenario,
            &ego_truth,
            &object_truth,
            &observation_truth,
            &ego_run.estimates,
            &estimated_tracker.tracks,
            &truth_tracker.tracks,
        );
        bundle::write_reports(
            output,
            &metrics,
            &ego_run.timing,
            &ego_run.diagnostics,
            &estimated_tracker.diagnostics,
            &truth_tracker.diagnostics,
            &ego_run.assumptions,
        )?;
        if build_visualization {
            viz::write_bundle_visualization(output, &viz::default_visualization_path(output))?;
        }
        manifest.finish(output)?;
        Ok(metrics)
    })();
    if let Err(error) = &result {
        let _ = manifest.fail(output, error);
    }
    result
}

pub fn generate_bundle(scenario_path: &Path, output: &Path) -> Result<GeneratedRun> {
    let scenario = resolve_scenario(scenario_path)?;
    bundle::prepare(output, &scenario)?;
    let generated = sensor::generate(&scenario)?;
    bundle::write_generated(output, &generated)?;
    Ok(generated)
}

pub fn score_ego_csv(run: &Path, csv: &Path, id: &str) -> Result<eval::EgoMetrics> {
    validate_external_id(id)?;
    let scenario = resolve_scenario(&run.join("scenario.resolved.yaml"))?;
    let estimates = external::read_ego_csv(
        csv,
        id,
        &scenario.platform.world_frame,
        &scenario.platform.body_frame,
    )?;
    let relative = format!("estimates/{id}.mcap");
    anyhow::ensure!(
        !run.join(&relative).exists(),
        "{} already exists",
        run.join(&relative).display()
    );
    bundle::write_ego_estimates_file(&run.join(&relative), id, &estimates)?;
    let truth_path = run.join("truth.mcap");
    let metrics = eval::evaluate_ego(
        &scenario,
        &bundle::read_ego_truth(&truth_path)?,
        &bundle::read_observation_truth(&truth_path)?,
        &estimates,
    );
    write_external_report(run, id, &metrics)?;
    bundle::refresh_artifact(run, &relative)?;
    bundle::refresh_artifact(run, &format!("reports/{id}/metrics.json"))?;
    Ok(metrics)
}

pub fn score_tracks_csv(
    run: &Path,
    csv: &Path,
    id: &str,
    ego_source: EgoSource,
) -> Result<eval::TrackMetrics> {
    validate_external_id(id)?;
    let scenario = resolve_scenario(&run.join("scenario.resolved.yaml"))?;
    let tracks = external::read_tracks_csv(csv, id, &scenario.platform.world_frame, ego_source)?;
    let relative = format!("tracks/{id}.mcap");
    anyhow::ensure!(
        !run.join(&relative).exists(),
        "{} already exists",
        run.join(&relative).display()
    );
    bundle::write_tracks_file(&run.join(&relative), id, &tracks)?;
    let truth_path = run.join("truth.mcap");
    let ego_label = if ego_source == EgoSource::Truth {
        "truth"
    } else {
        "estimated"
    };
    let metrics = eval::evaluate_tracks(
        &scenario,
        &bundle::read_ego_truth(&truth_path)?,
        &bundle::read_object_truth(&truth_path)?,
        &bundle::read_ego_estimates(&run.join("estimates/ego-baseline.mcap"))?,
        &tracks,
        ego_label,
    );
    write_external_report(run, id, &metrics)?;
    bundle::refresh_artifact(run, &relative)?;
    bundle::refresh_artifact(run, &format!("reports/{id}/metrics.json"))?;
    Ok(metrics)
}

fn validate_external_id(id: &str) -> Result<()> {
    anyhow::ensure!(
        !id.is_empty()
            && id.chars().all(
                |character| character.is_ascii_alphanumeric() || matches!(character, '-' | '_')
            ),
        "ID must contain only letters, numbers, '-' and '_'"
    );
    Ok(())
}

fn write_external_report(run: &Path, id: &str, metrics: &impl serde::Serialize) -> Result<()> {
    let directory = run.join("reports").join(id);
    std::fs::create_dir_all(&directory)?;
    std::fs::write(
        directory.join("metrics.json"),
        serde_json::to_vec_pretty(metrics)?,
    )?;
    Ok(())
}
