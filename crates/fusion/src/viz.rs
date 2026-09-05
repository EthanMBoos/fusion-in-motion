use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{Context, Result, bail};
use fusion_schema::messages::{
    EgoStateEstimate, EgoTruthState, ImuBiasTruth, ObjectTrack, ObjectTruthState, Pose2,
};

use crate::{
    bundle::{self, MeasurementRecord},
    math,
    scenario::load_and_resolve,
};

const TRUTH_COLOR: u32 = 0x33cc66ff;
const EGO_COLOR: u32 = 0xff4fa3ff;
const ESTIMATED_TRACK_COLOR: u32 = 0xffa62bff;
const TRUTH_EGO_TRACK_COLOR: u32 = 0x7a77ffff;
const CAMERA_COLOR: u32 = 0x35d0e6ff;
const LIDAR_COLOR: u32 = 0x3388ffff;
const GPS_COLOR: u32 = 0xffdd33ff;
const REFERENCE_COLOR: u32 = 0x88888899;

pub fn default_visualization_path(run: &Path) -> PathBuf {
    run.join("reports/baseline/visualization.rrd")
}

pub fn ensure_bundle_visualization(run: &Path, force: bool) -> Result<PathBuf> {
    let output = default_visualization_path(run);
    if output.exists() && !force {
        return Ok(output);
    }
    write_bundle_visualization(run, &output)?;
    Ok(output)
}

pub fn write_bundle_visualization(run: &Path, output: &Path) -> Result<()> {
    let measurements = bundle::read_measurements(&run.join("measurements.mcap"))?;
    let ego_truth = bundle::read_ego_truth(&run.join("truth.mcap"))?;
    let object_truth = bundle::read_object_truth(&run.join("truth.mcap"))?;
    let imu_bias_truth = bundle::read_imu_bias_truth(&run.join("truth.mcap"))?;
    let estimates = bundle::read_ego_estimates(&run.join("estimates/ego-baseline.mcap"))?;
    let estimated_tracks = bundle::read_tracks(&run.join("tracks/estimated-ego.mcap"))?;
    let truth_tracks = bundle::read_tracks(&run.join("tracks/truth-ego.mcap"))?;
    let scenario = load_and_resolve(&run.join("scenario.resolved.yaml"))?;
    if ego_truth.is_empty() {
        bail!("cannot visualize a run without ego truth");
    }
    if let Some(parent) = output.parent() {
        fs::create_dir_all(parent)?;
    }
    let run_name = run
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("run");
    let rec = rerun::RecordingStreamBuilder::new("fusion_in_motion")
        .recording_name(run_name)
        .save(output)
        .with_context(|| format!("failed to create {}", output.display()))?;

    rec.log_static(
        "dashboard/guide",
        &rerun::TextDocument::new(format!(
            "# Fusion in Motion — {run_name}\n\nVehicle: green truth, pink GPS/IMU estimate, yellow GPS fixes.\n\nObjects: green truth, orange tracks using the vehicle estimate, purple tracks using the true vehicle pose. Labels are tracker IDs.\n\nCamera: cyan direction only. Lidar: blue range and direction."
        )),
    )?;
    log_styles(&rec)?;
    log_paths(
        &rec,
        &ego_truth,
        &estimates,
        &object_truth,
        &estimated_tracks,
        &truth_tracks,
    )?;
    log_map_bounds(
        &rec,
        &ego_truth,
        &estimates,
        &object_truth,
        &estimated_tracks,
        &truth_tracks,
    )?;
    log_sensor_references(&rec, &scenario, &measurements)?;
    log_ego(&rec, &ego_truth, &estimates)?;
    log_objects(&rec, &object_truth, &estimated_tracks, &truth_tracks)?;
    log_measurements(&rec, &measurements)?;
    log_bias(&rec, &imu_bias_truth, &estimates)?;
    log_errors(
        &rec,
        &scenario,
        &ego_truth,
        &object_truth,
        &estimates,
        &estimated_tracks,
        &truth_tracks,
    )?;
    send_blueprint(&rec, &scenario)?;
    rec.flush_blocking()?;
    Ok(())
}

pub fn open_in_viewer(path: &Path) -> Result<()> {
    let status = Command::new("rerun")
        .arg(path)
        .status()
        .with_context(|| format!("failed to launch Rerun for {}", path.display()))?;
    if !status.success() {
        bail!("Rerun exited with {status}");
    }
    Ok(())
}

fn log_styles(rec: &rerun::RecordingStream) -> Result<()> {
    for (path, name, color) in [
        (
            "plots/ego/position_error_m",
            "vehicle position error (m)",
            EGO_COLOR,
        ),
        (
            "plots/ego/yaw_error_rad",
            "vehicle heading error (rad)",
            GPS_COLOR,
        ),
        (
            "plots/objects/estimated_ego_error_m",
            "object error: estimated ego (m)",
            ESTIMATED_TRACK_COLOR,
        ),
        (
            "plots/objects/truth_ego_error_m",
            "object error: truth ego (m)",
            TRUTH_EGO_TRACK_COLOR,
        ),
        ("plots/imu/gyro_z_radps", "gyro z (rad/s)", CAMERA_COLOR),
        (
            "plots/imu/accel_x_mps2",
            "accelerometer x (m/s²)",
            LIDAR_COLOR,
        ),
        ("plots/gps/age_ms", "GPS delivery age (ms)", GPS_COLOR),
        ("plots/detections/camera", "camera detections", CAMERA_COLOR),
        ("plots/detections/lidar", "lidar detections", LIDAR_COLOR),
        ("plots/bias/gyro/truth", "true bias", TRUTH_COLOR),
        ("plots/bias/gyro/estimate", "estimated bias", EGO_COLOR),
        ("plots/bias/gyro/lower", "95% lower", REFERENCE_COLOR),
        ("plots/bias/gyro/upper", "95% upper", REFERENCE_COLOR),
        ("plots/bias/accel/truth", "true bias", TRUTH_COLOR),
        ("plots/bias/accel/estimate", "estimated bias", EGO_COLOR),
        ("plots/bias/accel/lower", "95% lower", REFERENCE_COLOR),
        ("plots/bias/accel/upper", "95% upper", REFERENCE_COLOR),
    ] {
        let [r, g, b, _] = color.to_be_bytes();
        rec.log_static(
            path,
            &rerun::SeriesLines::new()
                .with_colors([[r, g, b]])
                .with_names([name]),
        )?;
    }
    Ok(())
}

fn log_paths(
    rec: &rerun::RecordingStream,
    ego_truth: &[EgoTruthState],
    estimates: &[EgoStateEstimate],
    object_truth: &[ObjectTruthState],
    estimated_tracks: &[ObjectTrack],
    truth_tracks: &[ObjectTrack],
) -> Result<()> {
    let truth_samples = ego_truth.iter().filter_map(|state| {
        Some((
            state.time_ns,
            point2(state.pose_world.as_ref()?.position.as_ref()?),
        ))
    });
    log_path_samples(
        rec,
        "map/paths/vehicle_truth",
        truth_samples,
        TRUTH_COLOR,
        0.05,
    )?;
    let estimate_samples = estimates.iter().filter_map(|state| {
        Some((
            state.estimate_time_ns,
            point2(state.pose_world.as_ref()?.position.as_ref()?),
        ))
    });
    log_path_samples(
        rec,
        "map/paths/vehicle_estimate",
        estimate_samples,
        EGO_COLOR,
        0.03,
    )?;
    log_object_paths(rec, "map/paths/object_truth", object_truth, TRUTH_COLOR)?;
    log_track_paths(
        rec,
        "map/paths/tracks_estimated_ego",
        estimated_tracks,
        ESTIMATED_TRACK_COLOR,
    )?;
    log_track_paths(
        rec,
        "map/paths/tracks_truth_ego",
        truth_tracks,
        TRUTH_EGO_TRACK_COLOR,
    )?;
    Ok(())
}

fn log_path_samples(
    rec: &rerun::RecordingStream,
    path: &str,
    samples: impl Iterator<Item = (i64, [f32; 2])>,
    color: u32,
    radius: f32,
) -> Result<()> {
    let mut points = Vec::new();
    let mut last_logged_ns = i64::MIN;
    for (time_ns, point) in samples {
        points.push(point);
        if time_ns.saturating_sub(last_logged_ns) < 50_000_000 {
            continue;
        }
        set_time(rec, time_ns);
        rec.log(
            path,
            &rerun::LineStrips2D::new([points.clone()])
                .with_colors([color])
                .with_radii([radius]),
        )?;
        last_logged_ns = time_ns;
    }
    Ok(())
}

fn log_map_bounds(
    rec: &rerun::RecordingStream,
    ego_truth: &[EgoTruthState],
    estimates: &[EgoStateEstimate],
    object_truth: &[ObjectTruthState],
    estimated_tracks: &[ObjectTrack],
    truth_tracks: &[ObjectTrack],
) -> Result<()> {
    let ego_truth_points = ego_truth
        .iter()
        .filter_map(|state| state.pose_world.as_ref()?.position.as_ref())
        .map(point2);
    let estimate_points = estimates
        .iter()
        .filter_map(|state| state.pose_world.as_ref()?.position.as_ref())
        .map(point2);
    let object_truth_points = object_truth
        .iter()
        .filter_map(|state| state.position_world_m.as_ref())
        .map(point2);
    let track_points = estimated_tracks
        .iter()
        .chain(truth_tracks)
        .filter_map(|track| track.position_world_m.as_ref())
        .map(point2);
    let mut points = ego_truth_points
        .chain(estimate_points)
        .chain(object_truth_points)
        .chain(track_points);
    let Some(first) = points.next() else {
        return Ok(());
    };
    let ([mut min_x, mut min_y], [mut max_x, mut max_y]) = points.fold(
        (first, first),
        |([min_x, min_y], [max_x, max_y]), [x, y]| {
            ([min_x.min(x), min_y.min(y)], [max_x.max(x), max_y.max(y)])
        },
    );
    let padding = ((max_x - min_x).max(max_y - min_y) * 0.08).max(0.5);
    min_x -= padding;
    min_y -= padding;
    max_x += padding;
    max_y += padding;
    rec.log_static(
        "map/view_bounds",
        &rerun::Points2D::new([[min_x, min_y], [max_x, max_y]])
            .with_colors([0x00000000])
            .with_radii([0.0]),
    )?;
    Ok(())
}

fn log_object_paths(
    rec: &rerun::RecordingStream,
    root: &str,
    states: &[ObjectTruthState],
    color: u32,
) -> Result<()> {
    let mut by_object = std::collections::BTreeMap::<&str, Vec<(i64, [f32; 2])>>::new();
    for state in states {
        if let Some(position) = &state.position_world_m {
            by_object
                .entry(&state.track_key)
                .or_default()
                .push((state.time_ns, point2(position)));
        }
    }
    for (id, points) in by_object {
        log_path_samples(
            rec,
            &format!("{root}/{id}"),
            points.into_iter(),
            color,
            0.025,
        )?;
    }
    Ok(())
}

fn log_track_paths(
    rec: &rerun::RecordingStream,
    root: &str,
    tracks: &[ObjectTrack],
    color: u32,
) -> Result<()> {
    let mut by_track = std::collections::BTreeMap::<&str, Vec<(i64, [f32; 2])>>::new();
    for track in tracks {
        if let Some(position) = &track.position_world_m {
            by_track
                .entry(&track.track_id)
                .or_default()
                .push((track.estimate_time_ns, point2(position)));
        }
    }
    for (id, points) in by_track {
        log_path_samples(
            rec,
            &format!("{root}/{id}"),
            points.into_iter(),
            color,
            0.02,
        )?;
    }
    Ok(())
}

fn log_sensor_references(
    rec: &rerun::RecordingStream,
    scenario: &crate::scenario::ResolvedScenario,
    measurements: &[MeasurementRecord],
) -> Result<()> {
    for root in ["sensors/camera", "sensors/lidar"] {
        rec.log_static(
            format!("{root}/vehicle_forward"),
            &rerun::Arrows2D::from_vectors([[1.0, 0.0]])
                .with_origins([[0.0, 0.0]])
                .with_colors([TRUTH_COLOR])
                .with_radii([0.05]),
        )?;
    }
    log_fov(
        rec,
        "sensors/camera/fov",
        scenario.camera.horizontal_fov_rad,
        4.0,
    )?;
    if scenario.lidar.horizontal_fov_rad < std::f64::consts::TAU - 1.0e-6 {
        log_fov(
            rec,
            "sensors/lidar/fov",
            scenario.lidar.horizontal_fov_rad,
            scenario.lidar.max_range_m,
        )?;
    }
    let lidar_points = measurements
        .iter()
        .filter_map(|measurement| match measurement {
            MeasurementRecord::Lidar(scan) => Some(scan),
            _ => None,
        })
        .flat_map(|scan| &scan.detections)
        .map(|detection| polar(detection.range_m, detection.bearing_rad));
    let ([mut min_x, mut min_y], [mut max_x, mut max_y]) = lidar_points.fold(
        ([0.0_f32, 0.0_f32], [0.0_f32, 0.0_f32]),
        |([min_x, min_y], [max_x, max_y]), [x, y]| {
            ([min_x.min(x), min_y.min(y)], [max_x.max(x), max_y.max(y)])
        },
    );
    let padding = ((max_x - min_x).max(max_y - min_y) * 0.08).max(0.5);
    min_x -= padding;
    min_y -= padding;
    max_x += padding;
    max_y += padding;
    rec.log_static(
        "sensors/lidar/view_bounds",
        &rerun::Points2D::new([[min_x, min_y], [max_x, max_y]])
            .with_colors([0x00000000])
            .with_radii([0.0]),
    )?;
    Ok(())
}

fn log_fov(rec: &rerun::RecordingStream, path: &str, fov: f64, length: f64) -> Result<()> {
    let half = fov * 0.5;
    let lines = [-half, half].map(|angle| vec![[0.0, 0.0], polar(length, angle)]);
    rec.log_static(
        path,
        &rerun::LineStrips2D::new(lines)
            .with_colors([REFERENCE_COLOR])
            .with_radii([0.01]),
    )?;
    Ok(())
}

fn log_ego(
    rec: &rerun::RecordingStream,
    truth: &[EgoTruthState],
    estimates: &[EgoStateEstimate],
) -> Result<()> {
    for state in truth {
        if let Some(pose) = &state.pose_world {
            set_time(rec, state.time_ns);
            log_pose(rec, "map/vehicle/truth", pose, TRUTH_COLOR, 0.24, 1.0)?;
        }
    }
    for state in estimates {
        if let Some(pose) = &state.pose_world {
            set_time(rec, state.estimate_time_ns);
            log_pose(rec, "map/vehicle/estimate", pose, EGO_COLOR, 0.14, 0.7)?;
        }
    }
    Ok(())
}

fn log_objects(
    rec: &rerun::RecordingStream,
    truth: &[ObjectTruthState],
    estimated: &[ObjectTrack],
    truth_ego: &[ObjectTrack],
) -> Result<()> {
    for state in truth {
        if let Some(position) = &state.position_world_m {
            set_time(rec, state.time_ns);
            rec.log(
                format!("map/objects/truth/{}", state.track_key),
                &rerun::Points2D::new([point2(position)])
                    .with_colors([TRUTH_COLOR])
                    .with_radii([0.19]),
            )?;
        }
    }
    for (root, tracks, color) in [
        (
            "map/objects/estimated_ego",
            estimated,
            ESTIMATED_TRACK_COLOR,
        ),
        ("map/objects/truth_ego", truth_ego, TRUTH_EGO_TRACK_COLOR),
    ] {
        for track in tracks {
            if let Some(position) = &track.position_world_m {
                set_time(rec, track.estimate_time_ns);
                let points = rerun::Points2D::new([point2(position)])
                    .with_colors([color])
                    .with_radii([0.12]);
                let points = if root == "map/objects/estimated_ego" {
                    points.with_labels([track.track_id.as_str()])
                } else {
                    points
                };
                rec.log(format!("{root}/{}", track.track_id), &points)?;
            }
        }
    }
    Ok(())
}

fn log_measurements(
    rec: &rerun::RecordingStream,
    measurements: &[MeasurementRecord],
) -> Result<()> {
    for measurement in measurements {
        let time = measurement.time();
        set_time(rec, time.measurement_time_ns);
        match measurement {
            MeasurementRecord::Imu(sample) => {
                rec.log(
                    "plots/imu/gyro_z_radps",
                    &rerun::Scalars::single(sample.yaw_rate_radps),
                )?;
                rec.log(
                    "plots/imu/accel_x_mps2",
                    &rerun::Scalars::single(sample.forward_acceleration_mps2),
                )?;
            }
            MeasurementRecord::Gps(fix) => {
                if let Some(position) = &fix.position_world_m {
                    rec.log(
                        "map/gps/fixes",
                        &rerun::Points2D::new([point2(position)])
                            .with_colors([GPS_COLOR])
                            .with_radii([0.08]),
                    )?;
                }
                rec.log(
                    "plots/gps/age_ms",
                    &rerun::Scalars::single(
                        (time.arrival_time_ns - time.measurement_time_ns) as f64 * 1.0e-6,
                    ),
                )?;
            }
            MeasurementRecord::Camera(frame) => {
                let rays = frame
                    .detections
                    .iter()
                    .map(|detection| vec![[0.0, 0.0], polar(3.0, detection.bearing_rad)])
                    .collect::<Vec<_>>();
                rec.log(
                    "sensors/camera/bearings",
                    &rerun::LineStrips2D::new(rays)
                        .with_colors([CAMERA_COLOR])
                        .with_radii([0.025]),
                )?;
                rec.log(
                    "plots/detections/camera",
                    &rerun::Scalars::single(frame.detections.len() as f64),
                )?;
            }
            MeasurementRecord::Lidar(scan) => {
                let points = scan
                    .detections
                    .iter()
                    .map(|detection| polar(detection.range_m, detection.bearing_rad))
                    .collect::<Vec<_>>();
                let rays = points
                    .iter()
                    .map(|point| vec![[0.0, 0.0], *point])
                    .collect::<Vec<_>>();
                rec.log(
                    "sensors/lidar/returns",
                    &rerun::Points2D::new(points)
                        .with_colors([LIDAR_COLOR])
                        .with_radii([0.13]),
                )?;
                rec.log(
                    "sensors/lidar/rays",
                    &rerun::LineStrips2D::new(rays)
                        .with_colors([LIDAR_COLOR])
                        .with_radii([0.012]),
                )?;
                rec.log(
                    "plots/detections/lidar",
                    &rerun::Scalars::single(scan.detections.len() as f64),
                )?;
            }
        }
    }
    Ok(())
}

fn log_errors(
    rec: &rerun::RecordingStream,
    scenario: &crate::scenario::ResolvedScenario,
    ego_truth: &[EgoTruthState],
    object_truth: &[ObjectTruthState],
    estimates: &[EgoStateEstimate],
    estimated_tracks: &[ObjectTrack],
    truth_tracks: &[ObjectTrack],
) -> Result<()> {
    for estimate in estimates {
        let Some(truth) = ego_truth
            .iter()
            .min_by_key(|truth| (truth.time_ns - estimate.estimate_time_ns).abs())
        else {
            continue;
        };
        let (Some(estimate_pose), Some(truth_pose)) = (&estimate.pose_world, &truth.pose_world)
        else {
            continue;
        };
        let (Some(estimate_position), Some(truth_position)) =
            (&estimate_pose.position, &truth_pose.position)
        else {
            continue;
        };
        set_time(rec, estimate.estimate_time_ns);
        rec.log(
            "plots/ego/position_error_m",
            &rerun::Scalars::single(
                (estimate_position.x - truth_position.x)
                    .hypot(estimate_position.y - truth_position.y),
            ),
        )?;
        rec.log(
            "plots/ego/yaw_error_rad",
            &rerun::Scalars::single(
                math::wrap_angle(estimate_pose.yaw_rad - truth_pose.yaw_rad).abs(),
            ),
        )?;
    }
    for (path, tracks) in [
        ("plots/objects/estimated_ego_error_m", estimated_tracks),
        ("plots/objects/truth_ego_error_m", truth_tracks),
    ] {
        let assignments = crate::eval::track_truth_assignments(
            tracks,
            object_truth,
            scenario.metrics.max_truth_match_gap_ns,
        );
        for track in tracks {
            let Some(truth_key) = assignments.get(&track.track_id) else {
                continue;
            };
            let Some(truth) = object_truth
                .iter()
                .filter(|truth| &truth.track_key == truth_key)
                .min_by_key(|truth| (truth.time_ns - track.estimate_time_ns).abs())
            else {
                continue;
            };
            let (Some(position), Some(truth_position)) =
                (&track.position_world_m, &truth.position_world_m)
            else {
                continue;
            };
            set_time(rec, track.estimate_time_ns);
            rec.log(
                path,
                &rerun::Scalars::single(
                    (position.x - truth_position.x).hypot(position.y - truth_position.y),
                ),
            )?;
        }
    }
    Ok(())
}

fn log_bias(
    rec: &rerun::RecordingStream,
    imu_bias_truth: &[ImuBiasTruth],
    estimates: &[EgoStateEstimate],
) -> Result<()> {
    let truth = imu_bias_truth
        .iter()
        .map(|bias| (bias.time_ns, bias))
        .collect::<std::collections::BTreeMap<_, _>>();

    for estimate in estimates {
        let (Some(bias), Some(gyro_estimate), Some(accel_estimate)) = (
            truth.get(&estimate.estimate_time_ns),
            estimate.gyro_bias_z_radps,
            estimate.accel_bias_x_mps2,
        ) else {
            continue;
        };
        if estimate.state_covariance.len() != 36 {
            continue;
        }

        set_time(rec, estimate.estimate_time_ns);
        let gyro_interval = 1.96 * estimate.state_covariance[4 * 6 + 4].max(0.0).sqrt();
        for (path, value) in [
            ("plots/bias/gyro/truth", bias.gyro_bias_z_radps),
            ("plots/bias/gyro/estimate", gyro_estimate),
            ("plots/bias/gyro/lower", gyro_estimate - gyro_interval),
            ("plots/bias/gyro/upper", gyro_estimate + gyro_interval),
        ] {
            rec.log(path, &rerun::Scalars::single(value))?;
        }

        let accel_interval = 1.96 * estimate.state_covariance[5 * 6 + 5].max(0.0).sqrt();
        for (path, value) in [
            ("plots/bias/accel/truth", bias.accel_bias_x_mps2),
            ("plots/bias/accel/estimate", accel_estimate),
            ("plots/bias/accel/lower", accel_estimate - accel_interval),
            ("plots/bias/accel/upper", accel_estimate + accel_interval),
        ] {
            rec.log(path, &rerun::Scalars::single(value))?;
        }
    }
    Ok(())
}

fn log_pose(
    rec: &rerun::RecordingStream,
    path: &str,
    pose: &Pose2,
    color: u32,
    radius: f32,
    heading_length: f32,
) -> Result<()> {
    let Some(position) = &pose.position else {
        return Ok(());
    };
    let origin = point2(position);
    let yaw = pose.yaw_rad as f32;
    rec.log(
        format!("{path}/position"),
        &rerun::Points2D::new([origin])
            .with_colors([color])
            .with_radii([radius]),
    )?;
    rec.log(
        format!("{path}/heading"),
        &rerun::Arrows2D::from_vectors([[heading_length * yaw.cos(), heading_length * yaw.sin()]])
            .with_origins([origin])
            .with_colors([color])
            .with_radii([0.05]),
    )?;
    Ok(())
}

fn send_blueprint(
    rec: &rerun::RecordingStream,
    scenario: &crate::scenario::ResolvedScenario,
) -> Result<()> {
    use rerun::blueprint::{
        Blueprint, BlueprintActivation, BlueprintPanel, Grid, Horizontal, SelectionPanel,
        Spatial2DView, TextDocumentView, TimePanel, TimeSeriesView, Vertical,
        components::{LoopMode, PanelState, PlayState},
    };
    let top = Horizontal::new([
        Spatial2DView::new("Vehicle and object tracks")
            .with_origin("map")
            .into(),
        TextDocumentView::new("What am I looking at?")
            .with_origin("dashboard")
            .into(),
    ])
    .with_column_shares(vec![4.0, 1.5]);
    let sensors = Grid::new([
        Spatial2DView::new("Camera: direction only")
            .with_origin("sensors/camera")
            .into(),
        Spatial2DView::new("Lidar: range and direction")
            .with_origin("sensors/lidar")
            .into(),
    ])
    .with_grid_columns(2);
    let mut plot_views: Vec<rerun::blueprint::ContainerLike> = vec![
        TimeSeriesView::new("Vehicle error")
            .with_origin("plots/ego")
            .into(),
        TimeSeriesView::new("Object error")
            .with_origin("plots/objects")
            .into(),
        TimeSeriesView::new("IMU").with_origin("plots/imu").into(),
        TimeSeriesView::new("Detections")
            .with_origin("plots/detections")
            .into(),
    ];
    if scenario.ego_estimator.estimate_imu_bias {
        plot_views.push(
            TimeSeriesView::new("Gyro bias")
                .with_origin("plots/bias/gyro")
                .into(),
        );
        plot_views.push(
            TimeSeriesView::new("Accelerometer bias")
                .with_origin("plots/bias/accel")
                .into(),
        );
    }
    if [
        scenario.imu.latency_ns,
        scenario.gps.latency_ns,
        scenario.camera.latency_ns,
        scenario.lidar.latency_ns,
    ]
    .into_iter()
    .any(|latency| latency > 0)
    {
        plot_views.push(
            TimeSeriesView::new("GPS timing")
                .with_origin("plots/gps")
                .into(),
        );
    }
    let plot_columns = match plot_views.len() {
        5 | 6 => 3,
        _ => 4,
    };
    let plots = Grid::new(plot_views).with_grid_columns(plot_columns);
    let root = Vertical::new([top.into(), sensors.into(), plots.into()])
        .with_row_shares(vec![2.8, 1.8, 3.0]);
    Blueprint::new(root)
        .with_auto_views(false)
        .with_auto_layout(false)
        .with_blueprint_panel(BlueprintPanel::new().with_state(PanelState::Collapsed))
        .with_selection_panel(SelectionPanel::new().with_state(PanelState::Collapsed))
        .with_time_panel(
            TimePanel::new()
                .with_state(PanelState::Collapsed)
                .with_timeline("time")
                .with_play_state(PlayState::Playing)
                .with_loop_mode(LoopMode::All)
                .with_playback_speed(1.0),
        )
        .send(rec, BlueprintActivation::default())?;
    Ok(())
}

fn set_time(rec: &rerun::RecordingStream, time_ns: i64) {
    rec.set_duration_secs("time", time_ns as f64 * 1.0e-9);
}

fn point2(value: &fusion_schema::messages::Vec2) -> [f32; 2] {
    [value.x as f32, value.y as f32]
}

fn polar(range: f64, angle: f64) -> [f32; 2] {
    [(range * angle.cos()) as f32, (range * angle.sin()) as f32]
}
