use std::path::PathBuf;

use anyhow::Result;
use fusion_in_motion::{
    bundle::{self, MeasurementRecord},
    estimator::{self, EgoMeasurement},
    scenario, sensor, sweep,
    tracker::{self, EgoHistory, EgoSource, PerceptionMeasurement},
};
use prost::Message;

fn starter_experiment() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../experiments/initial.yaml")
}

fn experiment(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(format!("../../experiments/{name}"))
}

fn split(records: &[MeasurementRecord]) -> (Vec<EgoMeasurement>, Vec<PerceptionMeasurement>) {
    let ego = records
        .iter()
        .filter_map(|record| match record {
            MeasurementRecord::Imu(value) => Some(EgoMeasurement::Imu(*value)),
            MeasurementRecord::Gps(value) => Some(EgoMeasurement::Gps(*value)),
            _ => None,
        })
        .collect();
    let perception = records
        .iter()
        .filter_map(|record| match record {
            MeasurementRecord::Camera(value) => Some(PerceptionMeasurement::Camera(value.clone())),
            MeasurementRecord::Lidar(value) => Some(PerceptionMeasurement::Lidar(value.clone())),
            _ => None,
        })
        .collect();
    (ego, perception)
}

#[test]
fn generation_is_repeatable_and_contains_all_four_sensors() -> Result<()> {
    let scenario = scenario::load_and_resolve(&starter_experiment())?;
    let first = sensor::generate(&scenario)?;
    let second = sensor::generate(&scenario)?;
    assert_eq!(first.measurements.len(), second.measurements.len());
    for (left, right) in first.measurements.iter().zip(&second.measurements) {
        assert_eq!(left.time(), right.time());
        match (left, right) {
            (MeasurementRecord::Imu(a), MeasurementRecord::Imu(b)) => assert_eq!(a, b),
            (MeasurementRecord::Gps(a), MeasurementRecord::Gps(b)) => assert_eq!(a, b),
            (MeasurementRecord::Camera(a), MeasurementRecord::Camera(b)) => assert_eq!(a, b),
            (MeasurementRecord::Lidar(a), MeasurementRecord::Lidar(b)) => assert_eq!(a, b),
            _ => panic!("measurement type changed"),
        }
    }
    for sensor in ["gps", "camera", "lidar"] {
        let found = first.measurements.iter().any(|record| match sensor {
            "gps" => matches!(record, MeasurementRecord::Gps(_)),
            "camera" => matches!(record, MeasurementRecord::Camera(_)),
            "lidar" => matches!(record, MeasurementRecord::Lidar(_)),
            _ => false,
        });
        assert!(found, "missing {sensor}");
    }
    assert_eq!(
        first
            .object_truth_states
            .iter()
            .map(|state| &state.track_key)
            .collect::<std::collections::BTreeSet<_>>()
            .len(),
        2
    );
    Ok(())
}

#[test]
fn complete_run_writes_the_small_bundle() -> Result<()> {
    let temp = tempfile::tempdir()?;
    let run = temp.path().join("run");
    fusion_in_motion::run_experiment(&starter_experiment(), &run)?;
    for relative in [
        "scenario.resolved.yaml",
        "measurements.mcap",
        "truth.mcap",
        "estimates/ego-baseline.mcap",
        "tracks/estimated-ego.mcap",
        "tracks/truth-ego.mcap",
        "reports/baseline/metrics.json",
        "reports/baseline/summary.md",
        "reports/baseline/visualization.rrd",
    ] {
        assert!(run.join(relative).is_file(), "missing {relative}");
    }
    assert!(!run.join("manifest.json").exists());
    assert!(!bundle::read_ego_estimates(&run.join("estimates/ego-baseline.mcap"))?.is_empty());
    assert!(!bundle::read_tracks(&run.join("tracks/estimated-ego.mcap"))?.is_empty());
    let summary = std::fs::read_to_string(run.join("reports/baseline/summary.md"))?;
    assert!(summary.contains("GPS fixes accepted/rejected/invalid: 32/0/0"));
    assert!(!summary.contains("## IMU bias"));
    assert!(!summary.contains("## Vehicle timing"));
    Ok(())
}

#[test]
fn lesson_reports_show_the_result_and_changed_settings() -> Result<()> {
    let temp = tempfile::tempdir()?;
    let bias_run = temp.path().join("bias");
    let timing_run = temp.path().join("timing");
    fusion_in_motion::run_experiment(&experiment("imu_bias.yaml"), &bias_run)?;
    fusion_in_motion::run_experiment(&experiment("timing.yaml"), &timing_run)?;

    let bias_summary = std::fs::read_to_string(bias_run.join("reports/baseline/summary.md"))?;
    assert!(bias_summary.contains("## IMU bias"));
    assert!(bias_summary.contains("Gyroscope bias RMSE"));
    assert!(!bias_summary.contains("## Vehicle timing"));

    let timing_summary = std::fs::read_to_string(timing_run.join("reports/baseline/summary.md"))?;
    assert!(timing_summary.contains("## Vehicle timing"));
    assert!(timing_summary.contains("Processing: offline measurement-time order"));
    assert!(timing_summary.contains("Delayed measurements reordered: 31"));
    assert!(!timing_summary.contains("## IMU bias"));

    let comparison = fusion_in_motion::compare::render(&bias_run, &timing_run)?;
    assert!(comparison.contains("gps.latency_ns: 0 -> 500000000"));
    assert!(comparison.contains("ego_estimator.timing_compensation: false -> true"));
    assert!(comparison.find("Changed settings:") < comparison.find("Results:"));
    Ok(())
}

#[test]
fn perception_settings_cannot_change_ego_estimates() -> Result<()> {
    let scenario = scenario::load_and_resolve(&starter_experiment())?;
    let baseline = sensor::generate(&scenario)?;
    let mut changed = scenario.clone();
    changed.camera.bearing_noise_stddev_rad *= 20.0;
    changed.lidar.range_noise_stddev_m *= 20.0;
    changed.lidar.detection_probability = 0.1;
    let changed = sensor::generate(&changed)?;
    let (baseline_ego, _) = split(&baseline.measurements);
    let (changed_ego, _) = split(&changed.measurements);
    assert_eq!(baseline_ego, changed_ego);
    let first = estimator::run_baseline(&scenario.ego_estimator, &scenario.imu, &baseline_ego)?;
    let second = estimator::run_baseline(&scenario.ego_estimator, &scenario.imu, &changed_ego)?;
    assert_eq!(first.estimates, second.estimates);
    Ok(())
}

#[test]
fn both_tracker_controls_use_the_same_detections() -> Result<()> {
    let scenario = scenario::load_and_resolve(&starter_experiment())?;
    let generated = sensor::generate(&scenario)?;
    let (ego_measurements, perception) = split(&generated.measurements);
    let ego_run =
        estimator::run_baseline(&scenario.ego_estimator, &scenario.imu, &ego_measurements)?;
    let estimated = tracker::run(
        &scenario.object_tracker,
        &perception,
        &EgoHistory::from_estimates(&ego_run.estimates)?,
    )?;
    let truth = tracker::run(
        &scenario.object_tracker,
        &perception,
        &EgoHistory::from_truth(&generated.ego_truth_states)?,
    )?;
    assert_eq!(estimated.processed_detections, truth.processed_detections);
    assert!(estimated.tracks[0].state_covariance[0] > truth.tracks[0].state_covariance[0]);
    assert_eq!(
        estimated
            .tracks
            .iter()
            .map(|track| track.track_id.as_str())
            .collect::<std::collections::BTreeSet<_>>(),
        std::collections::BTreeSet::from(["track-001", "track-002"])
    );
    assert!(
        estimated
            .tracks
            .iter()
            .all(|track| !matches!(track.track_id.as_str(), "stationary" | "moving"))
    );

    let camera_only = perception
        .iter()
        .filter(|measurement| matches!(measurement, PerceptionMeasurement::Camera(_)))
        .cloned()
        .collect::<Vec<_>>();
    let camera_only = tracker::run(
        &scenario.object_tracker,
        &camera_only,
        &EgoHistory::from_truth(&generated.ego_truth_states)?,
    )?;
    assert!(camera_only.tracks.is_empty());
    assert!(camera_only.diagnostics.waiting_for_range > 0);
    Ok(())
}

#[test]
fn association_experiment_keeps_two_tracker_owned_ids() -> Result<()> {
    let scenario = scenario::load_and_resolve(&experiment("association.yaml"))?;
    let generated = sensor::generate(&scenario)?;
    let (_, perception) = split(&generated.measurements);
    let run = tracker::run(
        &scenario.object_tracker,
        &perception,
        &EgoHistory::from_truth(&generated.ego_truth_states)?,
    )?;
    assert_eq!(run.diagnostics.created_tracks, 2);
    assert_eq!(run.diagnostics.confirmed_tracks, 2);
    assert_eq!(run.diagnostics.deleted_tracks, 0);
    assert_eq!(
        run.tracks
            .iter()
            .map(|track| track.track_id.as_str())
            .collect::<std::collections::BTreeSet<_>>(),
        std::collections::BTreeSet::from(["track-001", "track-002"])
    );
    Ok(())
}

#[test]
fn perception_study_compares_camera_lidar_and_both() -> Result<()> {
    let temp = tempfile::tempdir()?;
    let output = temp.path().join("perception");
    let report = sweep::run(&experiment("perception_sweep.yaml"), &output)?;
    assert_eq!(report.case_count, 15);
    assert_eq!(report.groups.len(), 3);

    let group = |camera, lidar| {
        report
            .groups
            .iter()
            .find(|group| {
                group.parameters["camera.enabled"].as_bool() == Some(camera)
                    && group.parameters["lidar.enabled"].as_bool() == Some(lidar)
            })
            .unwrap()
    };
    let camera_only = group(true, false);
    let lidar_only = group(false, true);
    let both = group(true, true);

    assert!(camera_only.mean_truth_ego_track_rmse_m.is_none());
    assert_eq!(
        camera_only.mean_truth_ego_track_time_coverage_fraction,
        Some(0.0)
    );
    assert!(
        both.mean_truth_ego_track_rmse_m.unwrap() < lidar_only.mean_truth_ego_track_rmse_m.unwrap()
    );
    assert!(
        both.mean_truth_ego_track_time_coverage_fraction.unwrap()
            > lidar_only
                .mean_truth_ego_track_time_coverage_fraction
                .unwrap()
    );

    for seed in 10..=14 {
        let case = |camera, lidar| {
            report
                .cases
                .iter()
                .find(|case| {
                    case.root_seed == seed
                        && case.parameters["camera.enabled"].as_bool() == Some(camera)
                        && case.parameters["lidar.enabled"].as_bool() == Some(lidar)
                })
                .unwrap()
                .metrics
                .as_ref()
                .unwrap()
        };
        let camera_only = case(true, false);
        let lidar_only = case(false, true);
        let both = case(true, true);
        assert_eq!(camera_only.tracks_with_truth_ego.position_rmse_m, None);
        assert_eq!(camera_only.estimated_ego_position_rmse_delta_m, None);
        assert_eq!(lidar_only.ego.position_rmse_m, both.ego.position_rmse_m);
        assert!(
            both.tracks_with_truth_ego.position_rmse_m.unwrap()
                < lidar_only.tracks_with_truth_ego.position_rmse_m.unwrap()
        );
    }

    let camera_summary =
        std::fs::read_to_string(output.join("case-0000/reports/baseline/summary.md"))?;
    assert!(camera_summary.contains("— (no matched tracks)"));
    let comparison =
        fusion_in_motion::compare::render(&output.join("case-0000"), &output.join("case-0005"))?;
    assert!(comparison.contains("Object RMSE, truth ego:     no track ->"));
    Ok(())
}

#[test]
fn gps_reduces_position_drift() -> Result<()> {
    let mut scenario = scenario::load_and_resolve(&starter_experiment())?;
    scenario.imu.accel_bias_mps2 = 0.025;
    let generated = sensor::generate(&scenario)?;
    let (with_gps, _) = split(&generated.measurements);
    let imu_only = with_gps
        .iter()
        .filter(|measurement| matches!(measurement, EgoMeasurement::Imu(_)))
        .cloned()
        .collect::<Vec<_>>();
    let fused = estimator::run_baseline(&scenario.ego_estimator, &scenario.imu, &with_gps)?;
    let drifting = estimator::run_baseline(&scenario.ego_estimator, &scenario.imu, &imu_only)?;
    let truth = generated
        .ego_truth_states
        .last()
        .unwrap()
        .pose_world
        .as_ref()
        .unwrap()
        .position
        .as_ref()
        .unwrap();
    let final_error = |run: &estimator::EstimatorRun| {
        let position = run
            .estimates
            .last()
            .unwrap()
            .pose_world
            .as_ref()
            .unwrap()
            .position
            .as_ref()
            .unwrap();
        (position.x - truth.x).hypot(position.y - truth.y)
    };
    assert!(final_error(&fused) < final_error(&drifting));
    Ok(())
}

#[test]
fn gps_outlier_is_rejected_when_gating_is_enabled() -> Result<()> {
    let mut scenario = scenario::load_and_resolve(&starter_experiment())?;
    scenario.ego_estimator.gps_gate_sigma = 4.0;
    let generated = sensor::generate(&scenario)?;
    let (mut ego_measurements, _) = split(&generated.measurements);
    let gps_fix_count = ego_measurements
        .iter()
        .filter(|measurement| matches!(measurement, EgoMeasurement::Gps(_)))
        .count();
    assert_eq!(gps_fix_count, 32);
    let fix = ego_measurements
        .iter_mut()
        .find_map(|measurement| match measurement {
            EgoMeasurement::Gps(fix) => Some(fix),
            EgoMeasurement::Imu(_) => None,
        })
        .unwrap();
    fix.position_world_m.as_mut().unwrap().x += 1_000.0;
    let run = estimator::run_baseline(&scenario.ego_estimator, &scenario.imu, &ego_measurements)?;
    assert_eq!(run.gps_diagnostics.attempted_fixes, gps_fix_count);
    assert_eq!(
        run.gps_diagnostics.accepted_fixes
            + run.gps_diagnostics.rejected_fixes
            + run.gps_diagnostics.invalid_fixes,
        gps_fix_count
    );
    assert!(run.gps_diagnostics.rejected_fixes >= 1);
    assert!(run.gps_diagnostics.accepted_fixes >= 1);
    Ok(())
}

#[test]
fn planar_detection_round_trips() -> Result<()> {
    let detection = fusion_schema::messages::CameraDetection {
        bearing_rad: 0.2,
        bearing_variance_rad2: 0.01,
    };
    let decoded =
        fusion_schema::messages::CameraDetection::decode(detection.encode_to_vec().as_slice())?;
    assert_eq!(decoded, detection);
    Ok(())
}

#[test]
fn advanced_experiments_turn_on_one_named_effect() -> Result<()> {
    let starter = scenario::load_and_resolve(&starter_experiment())?;
    assert_eq!(
        starter.ego_estimator.algorithm,
        scenario::EgoEstimatorAlgorithm::Basic
    );
    assert!(!starter.ego_estimator.timing_compensation);
    assert_eq!(starter.gps.outlier_probability, 0.0);
    let generated = sensor::generate(&starter)?;
    let (measurements, _) = split(&generated.measurements);
    let run = estimator::run_baseline(&starter.ego_estimator, &starter.imu, &measurements)?;
    assert!(run.estimates.iter().all(|estimate| {
        estimate.state_covariance.len() == 16
            && estimate.gyro_bias_z_radps.is_none()
            && estimate.accel_bias_x_mps2.is_none()
    }));

    let bias = scenario::load_and_resolve(&experiment("imu_bias.yaml"))?;
    assert_eq!(
        bias.ego_estimator.algorithm,
        scenario::EgoEstimatorAlgorithm::ImuBias
    );
    let generated = sensor::generate(&bias)?;
    let (measurements, _) = split(&generated.measurements);
    let run = estimator::run_baseline(&bias.ego_estimator, &bias.imu, &measurements)?;
    assert!(
        run.estimates
            .iter()
            .all(|estimate| estimate.state_covariance.len() == 36
                && estimate.gyro_bias_z_radps.is_some()
                && estimate.accel_bias_x_mps2.is_some())
    );

    let timing = scenario::load_and_resolve(&experiment("timing.yaml"))?;
    assert_eq!(timing.imu.latency_ns, 0);
    assert_eq!(timing.gps.latency_ns, 500_000_000);
    assert_eq!(timing.camera.latency_ns, 0);
    assert_eq!(timing.lidar.latency_ns, 0);
    assert_eq!(timing.lidar.scan_duration_ns, 0);
    assert!(!timing.object_tracker.timing_compensation);
    let generated = sensor::generate(&timing)?;
    let (measurements, _) = split(&generated.measurements);
    let run = estimator::run_baseline(&timing.ego_estimator, &timing.imu, &measurements)?;
    assert!(run.timing.replayed_measurements > 0);

    let outliers = scenario::load_and_resolve(&experiment("outliers.yaml"))?;
    let generated = sensor::generate(&outliers)?;
    let (measurements, _) = split(&generated.measurements);
    let run = estimator::run_baseline(&outliers.ego_estimator, &outliers.imu, &measurements)?;
    assert!(run.gps_diagnostics.rejected_fixes > 0);
    Ok(())
}

#[test]
fn external_outputs_can_be_scored() -> Result<()> {
    let temp = tempfile::tempdir()?;
    let run = temp.path().join("run");
    fusion_in_motion::run_experiment(&starter_experiment(), &run)?;

    let ego_truth = bundle::read_ego_truth(&run.join("truth.mcap"))?;
    let ego_csv = temp.path().join("perfect-ego.csv");
    let mut source = String::from("estimate_time_ns,x_m,y_m,yaw_rad\n");
    for state in ego_truth.iter().skip(1) {
        let pose = state.pose_world.as_ref().unwrap();
        let position = pose.position.as_ref().unwrap();
        source.push_str(&format!(
            "{},{},{},{}\n",
            state.time_ns, position.x, position.y, pose.yaw_rad
        ));
    }
    std::fs::write(&ego_csv, source)?;
    let metrics = fusion_in_motion::score_ego_csv(&run, &ego_csv, "perfect")?;
    assert!(metrics.position_rmse_m < 1.0e-12);
    assert_eq!(metrics.matched_samples, metrics.estimate_samples);
    assert_eq!(metrics.invalid_output_count, 0);

    let object_truth = bundle::read_object_truth(&run.join("truth.mcap"))?;
    let track_csv = temp.path().join("perfect-tracks.csv");
    let mut source = String::from("estimate_time_ns,track_id,x_m,y_m,vx_mps,vy_mps\n");
    for state in object_truth {
        let position = state.position_world_m.as_ref().unwrap();
        let velocity = state.velocity_world_mps.as_ref().unwrap();
        source.push_str(&format!(
            "{},{},{},{},{},{}\n",
            state.time_ns, state.track_key, position.x, position.y, velocity.x, velocity.y,
        ));
    }
    std::fs::write(&track_csv, source)?;
    let metrics =
        fusion_in_motion::score_tracks_csv(&run, &track_csv, "perfect-tracks", EgoSource::Truth)?;
    assert!(metrics.position_rmse_m.unwrap() < 1.0e-12);
    assert!(metrics.velocity_rmse_mps.unwrap() < 1.0e-12);
    assert_eq!(metrics.matched_samples, metrics.track_samples);
    assert_eq!(metrics.invalid_output_count, 0);
    Ok(())
}

#[test]
fn unusable_external_outputs_are_rejected_before_writing_results() -> Result<()> {
    let temp = tempfile::tempdir()?;
    let run = temp.path().join("run");
    fusion_in_motion::run_experiment(&starter_experiment(), &run)?;

    let empty_ego = temp.path().join("empty-ego.csv");
    std::fs::write(&empty_ego, "estimate_time_ns,x_m,y_m,yaw_rad\n")?;
    let error = fusion_in_motion::score_ego_csv(&run, &empty_ego, "empty-ego")
        .expect_err("empty ego output must fail");
    assert!(error.to_string().contains("ego CSV contains no rows"));
    assert!(!run.join("estimates/empty-ego.mcap").exists());

    let scenario = scenario::load_and_resolve(&run.join("scenario.resolved.yaml"))?;
    let unmatched_time = bundle::read_ego_truth(&run.join("truth.mcap"))?
        .last()
        .unwrap()
        .time_ns
        + scenario.metrics.max_truth_match_gap_ns
        + 1;
    let unmatched_ego = temp.path().join("unmatched-ego.csv");
    std::fs::write(
        &unmatched_ego,
        format!("estimate_time_ns,x_m,y_m,yaw_rad\n{unmatched_time},0,0,0\n"),
    )?;
    let error = fusion_in_motion::score_ego_csv(&run, &unmatched_ego, "unmatched-ego")
        .expect_err("unmatched ego output must fail");
    assert!(error.to_string().contains("0 of 1 estimates matched truth"));
    assert!(!run.join("estimates/unmatched-ego.mcap").exists());
    assert!(!run.join("reports/unmatched-ego").exists());

    let unmatched_tracks = temp.path().join("unmatched-tracks.csv");
    std::fs::write(
        &unmatched_tracks,
        format!(
            "estimate_time_ns,track_id,x_m,y_m,vx_mps,vy_mps\n{unmatched_time},track-1,0,0,0,0\n"
        ),
    )?;
    let error = fusion_in_motion::score_tracks_csv(
        &run,
        &unmatched_tracks,
        "unmatched-tracks",
        EgoSource::Truth,
    )
    .expect_err("unmatched track output must fail");
    assert!(error.to_string().contains("0 of 1 tracks matched truth"));
    assert!(!run.join("tracks/unmatched-tracks.mcap").exists());
    assert!(!run.join("reports/unmatched-tracks").exists());
    Ok(())
}
