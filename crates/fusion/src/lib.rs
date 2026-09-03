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
pub mod truth;
pub mod viz;

use std::path::{Path, PathBuf};

use anyhow::Result;

use crate::{
    bundle::{BundleManifest, GeneratedRun},
    estimator::run_baseline,
    eval::evaluate,
    scenario::ResolvedScenario,
    sensor::generate,
};

pub fn resolve_scenario(path: &Path) -> Result<ResolvedScenario> {
    scenario::load_and_resolve(path)
}

pub fn run_experiment(scenario_path: &Path, output: &Path) -> Result<PathBuf> {
    let scenario = resolve_scenario(scenario_path)?;
    run_resolved_experiment(&scenario, scenario_path, output, true)?;
    Ok(output.to_path_buf())
}

/// Run a scenario in an explicit folder or the next free `runs/runNNN` folder.
pub fn run_numbered_experiment(scenario_path: &Path, output: Option<&Path>) -> Result<PathBuf> {
    let output = output
        .map(Path::to_path_buf)
        .unwrap_or_else(|| next_run_path(Path::new("runs")));
    let scenario = resolve_scenario(scenario_path)?;
    run_resolved_experiment(&scenario, scenario_path, &output, true)?;
    Ok(output)
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
    scenario_path: &Path,
    output: &Path,
    build_visualization: bool,
) -> Result<eval::Metrics> {
    let mut manifest = BundleManifest::start(scenario, scenario_path)?;
    bundle::prepare(output, scenario)?;
    manifest.write(output)?;

    let result = (|| {
        let generated = generate(scenario)?;
        bundle::write_generated(output, &generated)?;
        manifest.record_generated(output)?;

        // Replay from the persisted estimator-visible artifact. The baseline never
        // receives the in-memory truth stream or the truth MCAP path.
        let replayed_measurements = bundle::read_measurements(&output.join("measurements.mcap"))?;
        let estimator_run = run_baseline(&scenario.estimator, &replayed_measurements)?;
        bundle::write_estimates(output, &estimator_run.estimates)?;
        manifest.record_estimates(output)?;

        // Evaluation likewise reads both persisted outputs, exercising the bundle
        // boundary instead of sharing simulator objects.
        let replayed_truth = bundle::read_truth_states(&output.join("truth.mcap"))?;
        let replayed_estimates = bundle::read_estimates(&output.join("estimates/baseline.mcap"))?;
        let metrics = evaluate(&replayed_truth, &replayed_estimates, &scenario.metrics)?;
        bundle::write_reports(output, &metrics, scenario, &estimator_run.timing)?;
        if build_visualization {
            viz::write_bundle_visualization(
                output,
                "baseline",
                &viz::default_visualization_path(output),
            )?;
        }
        manifest.finish(output, &metrics)?;
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
    let generated = generate(&scenario)?;
    bundle::write_generated(output, &generated)?;
    Ok(generated)
}

pub fn score_estimate_csv(
    run: &Path,
    csv_path: &Path,
    estimator_id: &str,
) -> Result<eval::Metrics> {
    anyhow::ensure!(
        !estimator_id.is_empty()
            && estimator_id.chars().all(
                |character| character.is_ascii_alphanumeric() || matches!(character, '-' | '_')
            ),
        "estimator ID must contain only ASCII letters, numbers, '-' and '_'"
    );
    let scenario = resolve_scenario(&run.join("scenario.resolved.yaml"))?;
    let estimates = external::read_estimates_csv(
        csv_path,
        estimator_id,
        &scenario.estimator.output_world_frame,
        &scenario.estimator.output_body_frame,
    )?;
    let estimate_relative = format!("estimates/{estimator_id}.mcap");
    let estimate_path = run.join(&estimate_relative);
    anyhow::ensure!(
        !estimate_path.exists(),
        "estimate output {} already exists",
        estimate_path.display()
    );
    let report_relative = format!("reports/{estimator_id}");
    anyhow::ensure!(
        !run.join(&report_relative).exists(),
        "report output {} already exists",
        run.join(&report_relative).display()
    );

    bundle::write_estimates_file(&estimate_path, estimator_id, &estimates)?;
    let truth = bundle::read_truth_states(&run.join("truth.mcap"))?;
    let metrics = evaluate(&truth, &estimates, &scenario.metrics)?;
    bundle::write_named_reports(run, estimator_id, &metrics)?;
    bundle::refresh_artifact(run, &estimate_relative)?;
    bundle::refresh_artifact(run, &format!("{report_relative}/metrics.json"))?;
    bundle::refresh_artifact(run, &format!("{report_relative}/summary.md"))?;
    Ok(metrics)
}
