use std::{fs, path::PathBuf};

use anyhow::Result;
use fusion_in_motion::{bundle, estimator, eval, math, scenario, sensor, sweep};

fn example() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../examples/initial.yaml")
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
            _ => panic!("measurement type changed between deterministic runs"),
        }
    }
    Ok(())
}

#[test]
fn complete_run_writes_replayable_bundle() -> Result<()> {
    let temp = tempfile::tempdir()?;
    let output = temp.path().join("run");
    let scenario = scenario::load_and_resolve(&example())?;
    fusion_in_motion::run_experiment(&example(), &output)?;

    let repeated = fusion_in_motion::run_experiment(&example(), &output);
    assert!(
        repeated.is_err(),
        "an existing run folder should never be replaced"
    );

    for relative in [
        "manifest.json",
        "scenario.resolved.yaml",
        "measurements.mcap",
        "truth.mcap",
        "estimates/baseline.mcap",
        "reports/baseline/metrics.json",
        "reports/baseline/timing.json",
        "reports/baseline/assumptions.json",
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
    let summary = fs::read_to_string(output.join("reports/baseline/summary.md"))?;
    assert!(summary.contains("Covariance consistency"));
    assert!(summary.contains("Normalized ANEES"));
    assert!(summary.contains("Output status:"));
    assert!(summary.contains("Valid output fraction:"));
    assert!(summary.contains("Estimator uncertainty"));
    assert!(summary.contains("IMU process noise: `scenario.imu`"));
    let metrics: fusion_in_motion::eval::Metrics =
        serde_json::from_slice(&fs::read(output.join("reports/baseline/metrics.json"))?)?;
    assert!(metrics.covariance_consistency.is_some());
    let timing: estimator::TimingDiagnostics =
        serde_json::from_slice(&fs::read(output.join("reports/baseline/timing.json"))?)?;
    assert!(timing.timing_compensation);
    let assumptions: estimator::BaselineAssumptions =
        serde_json::from_slice(&fs::read(output.join("reports/baseline/assumptions.json"))?)?;
    assert_eq!(
        assumptions.state_order,
        [
            "x",
            "y",
            "yaw",
            "forward_speed",
            "gyro_bias_z",
            "accel_bias_x"
        ]
    );
    assert_eq!(
        assumptions.initial_covariance_diagonal,
        [0.0001, 0.0001, 0.0001, 0.25, 0.01, 0.25]
    );
    assert!(assumptions.initial_cross_covariances_zero);
    assert_eq!(assumptions.imu_process_noise_source, "scenario.imu");
    assert_eq!(
        assumptions
            .imu_process_noise
            .gyro_white_noise_density_radps_sqrt_hz,
        scenario.imu.gyro_white_noise_density_radps_sqrt_hz
    );
    assert_eq!(
        assumptions
            .imu_process_noise
            .accel_white_noise_density_mps2_sqrt_hz,
        scenario.imu.accel_white_noise_density_mps2_sqrt_hz
    );
    assert_eq!(
        assumptions
            .imu_process_noise
            .gyro_bias_random_walk_radps_sqrt_s,
        scenario.imu.gyro_bias_random_walk_radps_sqrt_s
    );
    assert_eq!(
        assumptions
            .imu_process_noise
            .accel_bias_random_walk_mps2_sqrt_s,
        scenario.imu.accel_bias_random_walk_mps2_sqrt_s
    );
    assert!(!assumptions.uses_additional_process_noise);
    assert_eq!(
        assumptions.camera_bearing_stddev_rad,
        scenario.estimator.camera_bearing_stddev_rad
    );
    assert_eq!(
        assumptions.lidar_range_stddev_m,
        scenario.estimator.lidar_range_stddev_m
    );
    assert_eq!(
        assumptions.lidar_bearing_stddev_rad,
        scenario.estimator.lidar_bearing_stddev_rad
    );
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
    assert!(external_metrics.position_rmse_m.unwrap() < 1.0e-12);
    assert!(external_metrics.yaw_rmse_rad.unwrap() < 1.0e-12);
    assert!(external_metrics.covariance_consistency.is_none());
    assert!(output.join("estimates/perfect-csv.mcap").is_file());
    assert!(output.join("reports/perfect-csv/summary.md").is_file());

    let diverged_csv = temp.path().join("diverged.csv");
    fs::write(
        &diverged_csv,
        "estimate_time_ns,x_m,y_m,yaw_rad,status\n10000000,0.0,0.0,0.0,DIVERGED\n",
    )?;
    let diverged_metrics =
        fusion_in_motion::score_estimate_csv(&output, &diverged_csv, "diverged-csv")?;
    assert_eq!(diverged_metrics.diverged_output_count, 1);
    assert_eq!(diverged_metrics.position_rmse_m, None);
    let diverged_summary = fs::read_to_string(output.join("reports/diverged-csv/summary.md"))?;
    assert!(diverged_summary.contains("Position RMSE: — m"));
    assert!(diverged_summary.contains("1 diverged"));
    Ok(())
}

#[test]
fn timing_compensation_recovers_delayed_and_scanned_measurements() -> Result<()> {
    let mut scenario = scenario::load_and_resolve(&example())?;
    scenario.motion_speed_factor = 2.0;
    scenario.camera.latency_ns = 500_000_000;
    scenario.lidar.scan_duration_ns = 400_000_000;
    let generated = sensor::generate(&scenario)?;

    let mut uncompensated_config = scenario.estimator.clone();
    uncompensated_config.timing_compensation = false;
    let uncompensated = estimator::run_baseline(
        &uncompensated_config,
        &scenario.imu,
        &generated.measurements,
    )?;
    let uncompensated_metrics = eval::evaluate(&generated.truth_states, &uncompensated.estimates)?;

    let compensated =
        estimator::run_baseline(&scenario.estimator, &scenario.imu, &generated.measurements)?;
    let compensated_metrics = eval::evaluate(&generated.truth_states, &compensated.estimates)?;

    assert!(uncompensated.timing.delayed_measurements > 0);
    assert_eq!(uncompensated.timing.replayed_measurements, 0);
    assert_eq!(uncompensated.timing.revised_estimates, 0);
    assert_eq!(uncompensated.timing.deskewed_lidar_scans, 0);
    assert!(compensated.timing.replayed_measurements > 0);
    assert!(compensated.timing.deskewed_lidar_scans > 0);
    assert!(compensated.timing.revised_estimates > 0);
    assert_eq!(compensated.timing.discarded_measurements, 0);
    assert!(
        compensated_metrics.position_rmse_m.unwrap()
            < uncompensated_metrics.position_rmse_m.unwrap() * 0.2
    );
    Ok(())
}

#[test]
fn fixed_lag_discards_measurements_older_than_its_history() -> Result<()> {
    let mut scenario = scenario::load_and_resolve(&example())?;
    scenario.camera.latency_ns = 500_000_000;
    scenario.estimator.history_duration_ns = 100_000_000;
    let generated = sensor::generate(&scenario)?;
    let run = estimator::run_baseline(&scenario.estimator, &scenario.imu, &generated.measurements)?;

    assert!(run.timing.discarded_measurements > 0);
    assert!(run.timing.delayed_measurements > run.timing.replayed_measurements);
    Ok(())
}

#[test]
fn numbered_runs_choose_the_first_free_folder() -> Result<()> {
    let temp = tempfile::tempdir()?;
    fs::create_dir(temp.path().join("run001"))?;
    fs::create_dir(temp.path().join("run002"))?;
    assert_eq!(
        fusion_in_motion::next_run_path(temp.path()),
        temp.path().join("run003")
    );
    Ok(())
}

#[test]
fn comparison_shows_metric_differences() -> Result<()> {
    let temp = tempfile::tempdir()?;
    let baseline = temp.path().join("run001");
    let variant = temp.path().join("run002");
    fusion_in_motion::run_experiment(&example(), &baseline)?;

    let variant_scenario = temp.path().join("variant.yaml");
    let source = fs::read_to_string(example())?
        .replace("motion_speed_factor: 1.0", "motion_speed_factor: 2.0");
    fs::write(&variant_scenario, source)?;
    fusion_in_motion::run_experiment(&variant_scenario, &variant)?;

    let comparison = fusion_in_motion::compare::render(&baseline, &variant)?;
    assert!(comparison.contains("position RMSE (m)"));
    assert!(comparison.contains("normalized ANEES"));
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
    let csv = fs::read_to_string(output.join("reports/results.csv"))?;
    let header = csv.lines().next().expect("sweep CSV header");
    assert!(header.contains("root_seed"));
    assert!(header.contains("normalized_anees"));
    assert!(header.contains("valid_output_fraction"));
    let summary = fs::read_to_string(output.join("reports/summary.md"))?;
    assert!(summary.contains("single realization"));
    assert!(summary.contains("`n=1`"));
    Ok(())
}
