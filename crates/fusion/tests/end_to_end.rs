use std::path::PathBuf;

use anyhow::Result;
use fusion_in_motion::{
    bundle::{self, MeasurementRecord},
    estimator::{self, EgoMeasurement},
    scenario, sensor,
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
    let first = estimator::run_baseline(
        &scenario.ego_estimator,
        &scenario.imu,
        &scenario.gps,
        &baseline_ego,
    )?;
    let second = estimator::run_baseline(
        &scenario.ego_estimator,
        &scenario.imu,
        &scenario.gps,
        &changed_ego,
    )?;
    assert_eq!(first.estimates, second.estimates);
    Ok(())
}

#[test]
fn both_tracker_controls_use_the_same_detections() -> Result<()> {
    let scenario = scenario::load_and_resolve(&starter_experiment())?;
    let generated = sensor::generate(&scenario)?;
    let (ego_measurements, perception) = split(&generated.measurements);
    let ego_run = estimator::run_baseline(
        &scenario.ego_estimator,
        &scenario.imu,
        &scenario.gps,
        &ego_measurements,
    )?;
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
    let fused = estimator::run_baseline(
        &scenario.ego_estimator,
        &scenario.imu,
        &scenario.gps,
        &with_gps,
    )?;
    let drifting = estimator::run_baseline(
        &scenario.ego_estimator,
        &scenario.imu,
        &scenario.gps,
        &imu_only,
    )?;
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
    let fix = ego_measurements
        .iter_mut()
        .find_map(|measurement| match measurement {
            EgoMeasurement::Gps(fix) => Some(fix),
            EgoMeasurement::Imu(_) => None,
        })
        .unwrap();
    fix.position_world_m.as_mut().unwrap().x += 1_000.0;
    let run = estimator::run_baseline(
        &scenario.ego_estimator,
        &scenario.imu,
        &scenario.gps,
        &ego_measurements,
    )?;
    assert!(run.diagnostics.rejected_updates >= 1);
    assert!(run.diagnostics.applied_updates >= 1);
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
    assert!(!starter.ego_estimator.estimate_imu_bias);
    assert!(!starter.ego_estimator.timing_compensation);
    assert_eq!(starter.gps.outlier_probability, 0.0);

    let bias = scenario::load_and_resolve(&experiment("imu_bias.yaml"))?;
    let generated = sensor::generate(&bias)?;
    let (measurements, _) = split(&generated.measurements);
    let run = estimator::run_baseline(&bias.ego_estimator, &bias.imu, &bias.gps, &measurements)?;
    assert!(
        run.estimates
            .iter()
            .all(|estimate| estimate.gyro_bias_z_radps.is_some())
    );

    let timing = scenario::load_and_resolve(&experiment("timing.yaml"))?;
    let generated = sensor::generate(&timing)?;
    let (measurements, _) = split(&generated.measurements);
    let run = estimator::run_baseline(
        &timing.ego_estimator,
        &timing.imu,
        &timing.gps,
        &measurements,
    )?;
    assert!(run.timing.replayed_measurements > 0);

    let outliers = scenario::load_and_resolve(&experiment("outliers.yaml"))?;
    let generated = sensor::generate(&outliers)?;
    let (measurements, _) = split(&generated.measurements);
    let run = estimator::run_baseline(
        &outliers.ego_estimator,
        &outliers.imu,
        &outliers.gps,
        &measurements,
    )?;
    assert!(run.diagnostics.rejected_updates > 0);
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
    assert!(metrics.position_rmse_m < 1.0e-12);
    assert!(metrics.velocity_rmse_mps < 1.0e-12);
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
