use std::{fs, path::PathBuf};

use anyhow::Result;
use fusion_in_motion::{bundle, math, scenario, sensor, sweep};

fn example() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../examples/initial.yaml")
}

fn radar_example() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../examples/with_radar.yaml")
}

#[test]
fn generation_is_logically_deterministic() -> Result<()> {
    let scenario = scenario::load_and_resolve(&example())?;
    let first = sensor::generate(&scenario)?;
    let second = sensor::generate(&scenario)?;
    assert_eq!(first.measurements.len(), second.measurements.len());
    for (left, right) in first.measurements.iter().zip(&second.measurements) {
        assert_eq!(left.header(), right.header());
        match (left, right) {
            (bundle::MeasurementRecord::Map(a), bundle::MeasurementRecord::Map(b)) => {
                assert_eq!(a, b)
            }
            (bundle::MeasurementRecord::Imu(a), bundle::MeasurementRecord::Imu(b)) => {
                assert_eq!(a, b)
            }
            (bundle::MeasurementRecord::Camera(a), bundle::MeasurementRecord::Camera(b)) => {
                assert_eq!(a, b)
            }
            (bundle::MeasurementRecord::Lidar(a), bundle::MeasurementRecord::Lidar(b)) => {
                assert_eq!(a, b)
            }
            (bundle::MeasurementRecord::Radar(a), bundle::MeasurementRecord::Radar(b)) => {
                assert_eq!(a, b)
            }
            _ => panic!("measurement type changed between deterministic runs"),
        }
    }
    Ok(())
}

#[test]
fn complete_run_writes_replayable_bundle() -> Result<()> {
    let temp = tempfile::tempdir()?;
    let output = temp.path().join("run");
    fusion_in_motion::run_experiment(&example(), &output)?;

    let stale_artifact = output.join("stale-artifact");
    fs::write(&stale_artifact, "removed when the same run is repeated")?;
    fusion_in_motion::run_experiment(&example(), &output)?;
    assert!(
        !stale_artifact.exists(),
        "rerunning the same experiment should replace its output bundle"
    );

    for relative in [
        "manifest.json",
        "scenario.resolved.yaml",
        "measurements.mcap",
        "truth.mcap",
        "estimates/baseline.mcap",
        "reports/baseline/metrics.json",
        "reports/baseline/summary.md",
        "reports/baseline/visualization.rrd",
    ] {
        assert!(output.join(relative).is_file(), "missing {relative}");
    }
    let manifest: serde_json::Value =
        serde_json::from_slice(&fs::read(output.join("manifest.json"))?)?;
    assert_eq!(manifest["status"], "COMPLETE");
    assert!(
        manifest["artifacts"]["reports/baseline/visualization.rrd"]
            .as_str()
            .is_some()
    );
    assert!(fs::metadata(output.join("reports/baseline/visualization.rrd"))?.len() > 100);
    let measurements = bundle::read_measurements(&output.join("measurements.mcap"))?;
    assert!(
        measurements
            .iter()
            .any(|record| matches!(record, bundle::MeasurementRecord::Map(_)))
    );
    assert!(
        measurements
            .iter()
            .any(|record| matches!(record, bundle::MeasurementRecord::Imu(_)))
    );
    assert!(
        measurements
            .iter()
            .any(|record| matches!(record, bundle::MeasurementRecord::Camera(_)))
    );
    assert!(
        measurements
            .iter()
            .any(|record| matches!(record, bundle::MeasurementRecord::Lidar(_)))
    );
    assert!(
        measurements
            .iter()
            .all(|record| !matches!(record, bundle::MeasurementRecord::Radar(_)))
    );
    let truth = bundle::read_truth_states(&output.join("truth.mcap"))?;
    assert!(!truth.is_empty());
    assert!(!bundle::read_estimates(&output.join("estimates/baseline.mcap"))?.is_empty());

    let external_csv = temp.path().join("perfect.csv");
    let mut csv = String::from("estimate_time_ns,x_m,y_m,yaw_rad\n");
    for state in truth.iter().skip(1) {
        let pose = state.pose_w_b.as_ref().expect("generated truth pose");
        let position = pose.position.as_ref().expect("generated truth position");
        csv.push_str(&format!(
            "{},{},{},{}\n",
            state.truth_time_ns,
            position.x,
            position.y,
            math::yaw_from_pose(pose)
        ));
    }
    fs::write(&external_csv, csv)?;
    let external_metrics =
        fusion_in_motion::score_estimate_csv(&output, &external_csv, "perfect-csv")?;
    assert!(external_metrics.position_rmse_m < 1.0e-12);
    assert!(external_metrics.yaw_rmse_rad < 1.0e-12);
    assert!(output.join("estimates/perfect-csv.mcap").is_file());
    assert!(output.join("reports/perfect-csv/summary.md").is_file());
    Ok(())
}

#[test]
fn radar_example_generates_radar_measurements() -> Result<()> {
    let scenario = scenario::load_and_resolve(&radar_example())?;
    let generated = sensor::generate(&scenario)?;

    assert!(
        generated
            .measurements
            .iter()
            .any(|record| matches!(record, bundle::MeasurementRecord::Radar(_)))
    );
    Ok(())
}

#[test]
fn sweep_runs_parameter_cases_and_writes_aggregate_reports() -> Result<()> {
    let temp = tempfile::tempdir()?;
    let sweep_path = temp.path().join("sweep.yaml");
    let output = temp.path().join("sweep-run");
    fs::write(
        &sweep_path,
        format!(
            "name: test sweep\nbase_scenario: {}\nseeds: [7]\nparameters:\n  motion_speed_factor: [1.0, 2.0]\n",
            example().display()
        ),
    )?;

    let report = sweep::run(&sweep_path, &output)?;
    assert_eq!(report.case_count, 2);
    assert_eq!(report.successful_cases, 2);
    assert_eq!(report.failed_cases, 0);
    assert!(output.join("reports/results.json").is_file());
    assert!(output.join("reports/results.csv").is_file());
    assert!(output.join("reports/summary.md").is_file());
    assert!(output.join("case-0000/manifest.json").is_file());
    assert!(output.join("case-0001/manifest.json").is_file());
    assert!(
        !output
            .join("case-0000/reports/baseline/visualization.rrd")
            .exists()
    );
    Ok(())
}
