use std::path::PathBuf;

use anyhow::Result;
use fusion_in_motion::{
    bundle::{self, MeasurementRecord},
    estimator::{self, EgoMeasurement},
    scenario, sensor,
    tracker::{self, EgoHistory, PerceptionMeasurement},
};
use fusion_schema::messages::EgoSource;
use prost::Message;

fn example() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../examples/initial.yaml")
}

fn split(records: &[MeasurementRecord]) -> (Vec<EgoMeasurement>, Vec<PerceptionMeasurement>) {
    let ego = records
        .iter()
        .filter_map(|record| match record {
            MeasurementRecord::Imu(value) => Some(EgoMeasurement::Imu(value.clone())),
            MeasurementRecord::Gps(value) => Some(EgoMeasurement::Gps(value.clone())),
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
fn generation_is_repeatable_and_has_the_intended_sensor_roles() -> Result<()> {
    let scenario = scenario::load_and_resolve(&example())?;
    let first = sensor::generate(&scenario)?;
    let second = sensor::generate(&scenario)?;
    assert_eq!(first.measurements.len(), second.measurements.len());
    for (left, right) in first.measurements.iter().zip(&second.measurements) {
        assert_eq!(left.header(), right.header());
        match (left, right) {
            (MeasurementRecord::Calibration(a), MeasurementRecord::Calibration(b)) => {
                assert_eq!(a, b)
            }
            (MeasurementRecord::Imu(a), MeasurementRecord::Imu(b)) => assert_eq!(a, b),
            (MeasurementRecord::Gps(a), MeasurementRecord::Gps(b)) => assert_eq!(a, b),
            (MeasurementRecord::Camera(a), MeasurementRecord::Camera(b)) => assert_eq!(a, b),
            (MeasurementRecord::Lidar(a), MeasurementRecord::Lidar(b)) => assert_eq!(a, b),
            _ => panic!("measurement type changed"),
        }
    }
    assert!(
        first
            .measurements
            .iter()
            .any(|record| matches!(record, MeasurementRecord::Gps(_)))
    );
    assert!(
        first
            .measurements
            .iter()
            .any(|record| matches!(record, MeasurementRecord::Camera(_)))
    );
    assert!(
        first
            .measurements
            .iter()
            .any(|record| matches!(record, MeasurementRecord::Lidar(_)))
    );
    assert_eq!(
        first
            .object_truth_states
            .iter()
            .map(|state| &state.object_id)
            .collect::<std::collections::BTreeSet<_>>()
            .len(),
        2
    );
    let visible_detection_ids = first
        .measurements
        .iter()
        .flat_map(|record| match record {
            MeasurementRecord::Camera(frame) => frame
                .detections
                .iter()
                .map(|detection| detection.detection_id.as_str())
                .collect::<Vec<_>>(),
            MeasurementRecord::Lidar(scan) => scan
                .detections
                .iter()
                .map(|detection| detection.detection_id.as_str())
                .collect::<Vec<_>>(),
            _ => Vec::new(),
        })
        .collect::<std::collections::BTreeSet<_>>();
    let mapped_detection_ids = first
        .observation_truth
        .iter()
        .flat_map(|truth| {
            truth
                .detection_truth
                .iter()
                .map(|mapping| mapping.detection_id.as_str())
        })
        .collect::<std::collections::BTreeSet<_>>();
    assert_eq!(visible_detection_ids, mapped_detection_ids);
    Ok(())
}

#[test]
fn complete_run_writes_separate_ego_and_track_outputs() -> Result<()> {
    let temp = tempfile::tempdir()?;
    let run = temp.path().join("run");
    fusion_in_motion::run_experiment(&example(), &run)?;
    for relative in [
        "manifest.json",
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
    assert!(!bundle::read_ego_estimates(&run.join("estimates/ego-baseline.mcap"))?.is_empty());
    assert!(!bundle::read_tracks(&run.join("tracks/estimated-ego.mcap"))?.is_empty());
    assert!(!bundle::read_tracks(&run.join("tracks/truth-ego.mcap"))?.is_empty());
    Ok(())
}

#[test]
fn perception_settings_cannot_change_ego_estimates() -> Result<()> {
    let scenario = scenario::load_and_resolve(&example())?;
    let baseline = sensor::generate(&scenario)?;
    let mut changed = scenario.clone();
    changed.camera.bearing_noise_stddev_rad *= 20.0;
    changed.lidar.range_noise_stddev_m *= 20.0;
    changed.lidar.detection_probability = 0.1;
    let changed = sensor::generate(&changed)?;
    let (baseline_ego, _) = split(&baseline.measurements);
    let (changed_ego, _) = split(&changed.measurements);
    assert_eq!(baseline_ego.len(), changed_ego.len());
    for (left, right) in baseline_ego.iter().zip(&changed_ego) {
        match (left, right) {
            (EgoMeasurement::Imu(a), EgoMeasurement::Imu(b)) => assert_eq!(a, b),
            (EgoMeasurement::Gps(a), EgoMeasurement::Gps(b)) => assert_eq!(a, b),
            _ => panic!("ego measurement type changed"),
        }
    }
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
fn both_tracker_modes_use_identical_detections() -> Result<()> {
    let scenario = scenario::load_and_resolve(&example())?;
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
        &scenario.camera,
        &scenario.lidar,
        &perception,
        &EgoHistory::from_estimates(&ego_run.estimates)?,
        EgoSource::Estimated,
        &scenario.platform.world_frame,
    )?;
    let truth = tracker::run(
        &scenario.object_tracker,
        &scenario.camera,
        &scenario.lidar,
        &perception,
        &EgoHistory::from_truth(&generated.ego_truth_states)?,
        EgoSource::Truth,
        &scenario.platform.world_frame,
    )?;
    assert_eq!(
        estimated.processed_detection_ids,
        truth.processed_detection_ids
    );
    assert!(estimated.diagnostics.waiting_for_range > 0);
    assert!(estimated.tracks[0].covariance[0] > truth.tracks[0].covariance[0]);

    let camera_only = perception
        .iter()
        .filter(|measurement| matches!(measurement, PerceptionMeasurement::Camera(_)))
        .cloned()
        .collect::<Vec<_>>();
    let camera_only = tracker::run(
        &scenario.object_tracker,
        &scenario.camera,
        &scenario.lidar,
        &camera_only,
        &EgoHistory::from_truth(&generated.ego_truth_states)?,
        EgoSource::Truth,
        &scenario.platform.world_frame,
    )?;
    assert!(camera_only.tracks.is_empty());
    assert!(camera_only.diagnostics.waiting_for_range > 0);
    Ok(())
}

#[test]
fn gps_reduces_position_drift() -> Result<()> {
    let scenario = scenario::load_and_resolve(&example())?;
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
        .pose_w_b
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
            .pose_w_b
            .as_ref()
            .unwrap()
            .position
            .as_ref()
            .unwrap();
        (position.x - truth.x).hypot(position.y - truth.y)
    };
    assert!(final_error(&fused) < final_error(&drifting));
    assert!(fused.timing.replayed_measurements > 0);
    assert!(fused.estimates.iter().any(|estimate| estimate.revision > 0));
    Ok(())
}

#[test]
fn gps_outlier_is_rejected_and_reported() -> Result<()> {
    let scenario = scenario::load_and_resolve(&example())?;
    let generated = sensor::generate(&scenario)?;
    let (mut ego_measurements, _) = split(&generated.measurements);
    let fix = ego_measurements
        .iter_mut()
        .find_map(|measurement| match measurement {
            EgoMeasurement::Gps(fix) => Some(fix),
            EgoMeasurement::Imu(_) => None,
        })
        .expect("starter scenario has GPS fixes");
    fix.position_world_m.as_mut().unwrap().x += 1_000.0;

    let run = estimator::run_baseline(
        &scenario.ego_estimator,
        &scenario.imu,
        &scenario.gps,
        &ego_measurements,
    )?;
    assert!(run.diagnostics.rejected_updates >= 1);
    assert!(run.diagnostics.applied_updates >= 1);
    assert_eq!(run.diagnostics.invalid_updates, 0);
    Ok(())
}

#[test]
fn three_dimensional_detection_values_round_trip() -> Result<()> {
    let detection = fusion_schema::messages::CameraDetection {
        detection_id: "detection".to_owned(),
        association_key: "object".to_owned(),
        azimuth_rad: 0.2,
        elevation_rad: -0.4,
        angular_covariance: vec![0.1, 0.0, 0.0, 0.2],
    };
    let decoded =
        fusion_schema::messages::CameraDetection::decode(detection.encode_to_vec().as_slice())?;
    assert_eq!(decoded, detection);
    Ok(())
}

#[test]
fn external_ego_csv_can_be_scored() -> Result<()> {
    let temp = tempfile::tempdir()?;
    let run = temp.path().join("run");
    fusion_in_motion::run_experiment(&example(), &run)?;
    let truth = bundle::read_ego_truth(&run.join("truth.mcap"))?;
    let csv = temp.path().join("perfect.csv");
    let mut source = String::from("estimate_time_ns,x_m,y_m,yaw_rad\n");
    for state in truth.iter().skip(1) {
        let pose = state.pose_w_b.as_ref().unwrap();
        let position = pose.position.as_ref().unwrap();
        source.push_str(&format!(
            "{},{},{},{}\n",
            state.truth_time_ns,
            position.x,
            position.y,
            fusion_in_motion::math::yaw_from_pose(pose)
        ));
    }
    std::fs::write(&csv, source)?;
    let metrics = fusion_in_motion::score_ego_csv(&run, &csv, "perfect")?;
    assert!(metrics.position_rmse_m < 1.0e-12);
    assert!(run.join("estimates/perfect.mcap").is_file());
    Ok(())
}

#[test]
fn external_track_csv_can_be_scored() -> Result<()> {
    let temp = tempfile::tempdir()?;
    let run = temp.path().join("run");
    fusion_in_motion::run_experiment(&example(), &run)?;
    let scenario = scenario::load_and_resolve(&run.join("scenario.resolved.yaml"))?;
    let associations = scenario
        .world
        .objects
        .iter()
        .map(|object| (&object.id, &object.association_key))
        .collect::<std::collections::BTreeMap<_, _>>();
    let truth = bundle::read_object_truth(&run.join("truth.mcap"))?;
    let csv = temp.path().join("perfect-tracks.csv");
    let mut source =
        String::from("estimate_time_ns,track_id,association_key,x_m,y_m,vx_mps,vy_mps\n");
    for state in truth {
        let position = state.position_world_m.as_ref().unwrap();
        let velocity = state.velocity_world_mps.as_ref().unwrap();
        let association = associations[&state.object_id];
        source.push_str(&format!(
            "{},track-{},{},{},{},{},{}\n",
            state.truth_time_ns,
            association,
            association,
            position.x,
            position.y,
            velocity.x,
            velocity.y,
        ));
    }
    std::fs::write(&csv, source)?;
    let metrics =
        fusion_in_motion::score_tracks_csv(&run, &csv, "perfect-tracks", EgoSource::Truth)?;
    assert!(metrics.position_rmse_m < 1.0e-12);
    assert!(metrics.velocity_rmse_mps < 1.0e-12);
    assert!(run.join("tracks/perfect-tracks.mcap").is_file());
    Ok(())
}
