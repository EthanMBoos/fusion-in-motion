use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{Context, Result, bail};
use fusion_schema::messages::{ObservationTruth, Pose, RecordHeader, StateEstimate, TruthState};

use crate::{
    bundle::{self, MeasurementRecord},
    eval,
    math::{wrap_angle, yaw_from_pose},
    scenario::{ResolvedScenario, load_and_resolve},
};

const TRUTH_COLOR: u32 = 0x33CC66FF;
const ESTIMATE_COLOR: u32 = 0xFF5577FF;
const LANDMARK_COLOR: u32 = 0xFFD54FFF;
const CAMERA_COLOR: u32 = 0x33CCFFFF;
const LIDAR_COLOR: u32 = 0x4488FFFF;
const YAW_ERROR_COLOR: u32 = 0xFF9933FF;
const POSITION_BOUND_COLOR: u32 = 0xFFB3C1FF;
const YAW_BOUND_COLOR: u32 = 0xFFD199FF;
const REFERENCE_COLOR: u32 = 0x667080FF;

pub fn default_visualization_path(bundle: &Path) -> PathBuf {
    visualization_path(bundle, "baseline")
}

pub fn visualization_path(bundle: &Path, estimator_id: &str) -> PathBuf {
    bundle
        .join("reports")
        .join(estimator_id)
        .join("visualization.rrd")
}

pub fn ensure_bundle_visualization(
    bundle_path: &Path,
    estimator_id: &str,
    output: Option<&Path>,
    force: bool,
) -> Result<PathBuf> {
    let default_output = visualization_path(bundle_path, estimator_id);
    let output = output
        .map(Path::to_path_buf)
        .unwrap_or_else(|| default_output.clone());
    if output.exists() && !force {
        return Ok(output);
    }
    write_bundle_visualization(bundle_path, estimator_id, &output)?;
    if output == default_output {
        bundle::refresh_artifact(
            bundle_path,
            &format!("reports/{estimator_id}/visualization.rrd"),
        )?;
    }
    Ok(output)
}

pub fn write_bundle_visualization(
    bundle_path: &Path,
    estimator_id: &str,
    output: &Path,
) -> Result<()> {
    let measurements = bundle::read_measurements(&bundle_path.join("measurements.mcap"))?;
    let truth_path = bundle_path.join("truth.mcap");
    let truth = bundle::read_truth_states(&truth_path)?;
    let observation_truth = bundle::read_observation_truth(&truth_path)?;
    let estimates = bundle::read_estimates(
        &bundle_path
            .join("estimates")
            .join(format!("{estimator_id}.mcap")),
    )?;
    let scenario = load_and_resolve(&bundle_path.join("scenario.resolved.yaml"))?;
    if truth.is_empty() {
        bail!("cannot visualize a bundle with no truth states");
    }

    if let Some(parent) = output.parent() {
        fs::create_dir_all(parent)?;
    }
    let rec = rerun::RecordingStreamBuilder::new("fusion_in_motion")
        .save(output)
        .with_context(|| format!("failed to create Rerun recording {}", output.display()))?;

    rec.log_static(
        "dashboard/setup",
        &rerun::TextDocument::from_markdown(format!(
            "**Path:** {:.2}x speed · {:.1} s  \n**Camera:** {:.1} Hz · {:.1} ms latency  \n**Lidar:** {:.1} Hz · {:.1} ms latency · {:.1} ms scan  \n**Timing compensation:** {} · {:.0} ms history  \n**Seed:** {}",
            scenario.motion_speed_factor,
            scenario.effective_duration_s(),
            scenario.camera.rate_hz,
            scenario.camera.latency_ns as f64 / 1.0e6,
            scenario.lidar.rate_hz,
            scenario.lidar.latency_ns as f64 / 1.0e6,
            scenario.lidar.scan_duration_ns as f64 / 1.0e6,
            if scenario.estimator.timing_compensation {
                "on"
            } else {
                "off"
            },
            scenario.estimator.history_duration_ns as f64 / 1.0e6,
            scenario.root_seed,
        )),
    )?;
    rec.log_static(
        "dashboard/guide",
        &rerun::TextDocument::from_markdown(
            "- **Map:** green truth, pink estimate.\n- **Camera:** bearings only; no depth.\n- **Lidar:** range and bearing; orange is early in the scan, cyan is late.\n- **Timing:** age is receipt time minus reported time. Acquisition is the interval covered by one record.\n- **Error:** absolute error and reported 95% bounds.\n- **Bias:** green truth and pink estimate. Normalized error should usually stay inside ±1.96.\n\nDrag the timeline to inspect one time across every panel.",
        ),
    )?;

    log_series_styles(&rec)?;
    log_static_map(&rec, &measurements, &truth, &estimates)?;
    log_sensor_references(&rec, &scenario)?;
    log_motion_segments(&rec, &scenario)?;
    log_vehicle_motion(&rec, &truth, &estimates)?;
    log_measurements(&rec, &measurements, &truth)?;
    log_errors(&rec, &truth, &estimates)?;
    log_biases(&rec, &observation_truth, &estimates)?;
    send_dashboard_blueprint(&rec)?;
    rec.flush_blocking()?;
    Ok(())
}

pub fn open_in_viewer(path: &Path) -> Result<()> {
    let status = Command::new("rerun")
        .arg(path)
        .status()
        .with_context(|| {
            format!(
                "failed to launch the Rerun viewer; install it as described in docs/INSTALL.md, then run `rerun {}`",
                path.display()
            )
        })?;
    if !status.success() {
        bail!("Rerun viewer exited with {status}");
    }
    Ok(())
}

fn log_series_styles(rec: &rerun::RecordingStream) -> Result<()> {
    let series = [
        (
            "plots/error/position_m",
            "position error (m)",
            ESTIMATE_COLOR,
        ),
        ("plots/error/yaw_rad", "yaw error (rad)", YAW_ERROR_COLOR),
        (
            "plots/error/position_bound_95_m",
            "position 95% outer bound (m)",
            POSITION_BOUND_COLOR,
        ),
        (
            "plots/error/yaw_bound_95_rad",
            "yaw 95% bound (rad)",
            YAW_BOUND_COLOR,
        ),
        ("plots/imu/gyro_z_radps", "gyro z (rad/s)", CAMERA_COLOR),
        ("plots/imu/accel_x_mps2", "accel x (m/s²)", LIDAR_COLOR),
        ("plots/observations/camera", "camera features", CAMERA_COLOR),
        ("plots/observations/lidar", "lidar returns", LIDAR_COLOR),
        ("plots/timing/imu_age_ms", "IMU age (ms)", REFERENCE_COLOR),
        (
            "plots/timing/camera_age_ms",
            "camera age (ms)",
            CAMERA_COLOR,
        ),
        ("plots/timing/lidar_age_ms", "lidar age (ms)", LIDAR_COLOR),
        (
            "plots/timing/lidar_acquisition_ms",
            "lidar acquisition (ms)",
            TRUTH_COLOR,
        ),
        (
            "plots/timing/imu_acquisition_ms",
            "IMU acquisition (ms)",
            ESTIMATE_COLOR,
        ),
        (
            "plots/bias/gyro/value/true_radps",
            "true gyro-z bias (rad/s)",
            TRUTH_COLOR,
        ),
        (
            "plots/bias/gyro/value/estimate_radps",
            "estimated gyro-z bias (rad/s)",
            ESTIMATE_COLOR,
        ),
        (
            "plots/bias/gyro/normalized/error",
            "gyro-z error / σ",
            YAW_ERROR_COLOR,
        ),
        (
            "plots/bias/gyro/normalized/lower_95",
            "95% lower (-1.96)",
            REFERENCE_COLOR,
        ),
        (
            "plots/bias/gyro/normalized/upper_95",
            "95% upper (+1.96)",
            REFERENCE_COLOR,
        ),
        (
            "plots/bias/accel/value/true_mps2",
            "true accel-x bias (m/s²)",
            TRUTH_COLOR,
        ),
        (
            "plots/bias/accel/value/estimate_mps2",
            "estimated accel-x bias (m/s²)",
            ESTIMATE_COLOR,
        ),
        (
            "plots/bias/accel/normalized/error",
            "accel-x error / σ",
            YAW_ERROR_COLOR,
        ),
        (
            "plots/bias/accel/normalized/lower_95",
            "95% lower (-1.96)",
            REFERENCE_COLOR,
        ),
        (
            "plots/bias/accel/normalized/upper_95",
            "95% upper (+1.96)",
            REFERENCE_COLOR,
        ),
    ];
    for (path, name, color) in series {
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

fn log_static_map(
    rec: &rerun::RecordingStream,
    measurements: &[MeasurementRecord],
    truth: &[TruthState],
    estimates: &[StateEstimate],
) -> Result<()> {
    if let Some(MeasurementRecord::Map(map)) = measurements
        .iter()
        .find(|measurement| matches!(measurement, MeasurementRecord::Map(_)))
    {
        let points = map
            .landmarks
            .iter()
            .filter_map(|landmark| landmark.position_world_m.as_ref())
            .map(point2)
            .collect::<Vec<_>>();
        let labels = map
            .landmarks
            .iter()
            .map(|landmark| landmark.id.as_str())
            .collect::<Vec<_>>();
        rec.log_static(
            "map/landmarks",
            &rerun::Points2D::new(points)
                .with_colors([LANDMARK_COLOR])
                .with_radii([0.18])
                .with_labels(labels),
        )?;
    }

    let truth_path = truth
        .iter()
        .filter_map(|state| state.pose_w_b.as_ref()?.position.as_ref())
        .map(point2)
        .collect::<Vec<_>>();
    rec.log_static(
        "map/trajectories/truth",
        &rerun::LineStrips2D::new([truth_path])
            .with_colors([TRUTH_COLOR])
            .with_radii([rerun::Radius::new_ui_points(2.0)]),
    )?;

    let estimate_path = estimates
        .iter()
        .filter_map(|estimate| estimate.pose_w_b.as_ref()?.position.as_ref())
        .map(point2)
        .collect::<Vec<_>>();
    let marker_stride = (estimate_path.len() / 24).max(1);
    let estimate_markers = estimate_path
        .iter()
        .copied()
        .step_by(marker_stride)
        .collect::<Vec<_>>();
    rec.log_static(
        "map/trajectories/estimate",
        &rerun::LineStrips2D::new([estimate_path])
            .with_colors([ESTIMATE_COLOR])
            .with_radii([rerun::Radius::new_ui_points(0.9)]),
    )?;
    rec.log_static(
        "map/trajectories/estimate_samples",
        &rerun::Points2D::new(estimate_markers)
            .with_colors([ESTIMATE_COLOR])
            .with_radii([rerun::Radius::new_ui_points(3.0)]),
    )?;
    Ok(())
}

fn log_sensor_references(rec: &rerun::RecordingStream, scenario: &ResolvedScenario) -> Result<()> {
    for root in ["sensors/camera", "sensors/lidar"] {
        rec.log_static(
            format!("{root}/platform"),
            &rerun::Arrows2D::from_vectors([[1.0, 0.0]])
                .with_origins([[0.0, 0.0]])
                .with_colors([TRUTH_COLOR])
                .with_radii([0.06])
                .with_labels(["forward"]),
        )?;
    }

    log_fov_reference(
        rec,
        "sensors/camera/reference/fov",
        scenario.camera.horizontal_fov_rad,
        3.0,
    )?;
    log_range_reference(
        rec,
        "sensors/lidar/reference/range",
        scenario.lidar.max_range_m,
        std::f64::consts::TAU,
    )?;
    Ok(())
}

fn log_fov_reference(
    rec: &rerun::RecordingStream,
    path: &str,
    fov_rad: f64,
    length_m: f64,
) -> Result<()> {
    let half = fov_rad * 0.5;
    let edges = [-half, half]
        .map(|angle| vec![[0.0, 0.0], polar(length_m, angle)])
        .to_vec();
    rec.log_static(
        path,
        &rerun::LineStrips2D::new(edges)
            .with_colors([REFERENCE_COLOR])
            .with_radii([0.012]),
    )?;
    Ok(())
}

fn log_range_reference(
    rec: &rerun::RecordingStream,
    path: &str,
    max_range_m: f64,
    fov_rad: f64,
) -> Result<()> {
    let radii = [max_range_m * 0.25, max_range_m * 0.5];
    let rings = radii
        .into_iter()
        .map(|radius| arc(radius, fov_rad, 96))
        .collect::<Vec<_>>();
    let labels = radii.map(|radius| format!("{radius:.0} m"));
    let label_positions = radii.map(|radius| polar(radius, std::f64::consts::FRAC_PI_2));
    rec.log_static(
        path,
        &rerun::LineStrips2D::new(rings)
            .with_colors([REFERENCE_COLOR])
            .with_radii([rerun::Radius::new_ui_points(0.75)]),
    )?;
    rec.log_static(
        format!("{path}_labels"),
        &rerun::Points2D::new(label_positions)
            .with_colors([REFERENCE_COLOR])
            .with_radii([rerun::Radius::new_ui_points(1.5)])
            .with_labels(labels),
    )?;
    if fov_rad < std::f64::consts::TAU - 1.0e-6 {
        log_fov_reference(rec, &format!("{path}_edges"), fov_rad, max_range_m)?;
    }
    Ok(())
}

fn log_vehicle_motion(
    rec: &rerun::RecordingStream,
    truth: &[TruthState],
    estimates: &[StateEstimate],
) -> Result<()> {
    for state in truth {
        let Some(pose) = state.pose_w_b.as_ref() else {
            continue;
        };
        rec.set_duration_secs("time", seconds(state.truth_time_ns));
        log_pose(rec, "map/vehicle/truth", pose, TRUTH_COLOR, 0.26, 1.2)?;
    }
    for estimate in estimates {
        let Some(pose) = estimate.pose_w_b.as_ref() else {
            continue;
        };
        rec.set_duration_secs("time", seconds(estimate.estimate_time_ns));
        log_pose(rec, "map/vehicle/estimate", pose, ESTIMATE_COLOR, 0.15, 0.8)?;
    }
    Ok(())
}

fn log_motion_segments(rec: &rerun::RecordingStream, scenario: &ResolvedScenario) -> Result<()> {
    let mut start_s = 0.0;
    for (index, segment) in scenario.trajectory.iter().enumerate() {
        rec.set_duration_secs("time", start_s / scenario.motion_speed_factor);
        rec.log(
            "dashboard/now",
            &rerun::TextDocument::from_markdown(format!(
                "**{}** · segment {} of {}\n\nAcceleration: {:.2} m/s²  \nYaw rate: {:.2} rad/s",
                segment.id,
                index + 1,
                scenario.trajectory.len(),
                segment.longitudinal_acceleration_mps2 * scenario.motion_speed_factor.powi(2),
                segment.yaw_rate_radps * scenario.motion_speed_factor,
            )),
        )?;
        start_s += segment.duration_s;
    }
    Ok(())
}

fn log_pose(
    rec: &rerun::RecordingStream,
    path: &str,
    pose: &Pose,
    color: u32,
    radius: f32,
    heading_length: f32,
) -> Result<()> {
    let Some(position) = pose.position.as_ref() else {
        return Ok(());
    };
    let origin = point2(position);
    let yaw = yaw_from_pose(pose) as f32;
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
            .with_radii([0.06]),
    )?;
    Ok(())
}

fn log_measurements(
    rec: &rerun::RecordingStream,
    measurements: &[MeasurementRecord],
    truth: &[TruthState],
) -> Result<()> {
    for measurement in measurements {
        match measurement {
            MeasurementRecord::Map(_) => {}
            MeasurementRecord::Imu(sample) => {
                let time_ns = sample
                    .header
                    .as_ref()
                    .map_or(0, |header| header.reported_stamp_ns);
                rec.set_duration_secs("time", seconds(time_ns));
                if let Some(header) = sample.header.as_ref() {
                    log_timing(rec, "imu", header)?;
                }
                if let Some(gyro) = sample.angular_rate_radps.as_ref() {
                    rec.log("plots/imu/gyro_z_radps", &rerun::Scalars::single(gyro.z))?;
                }
                if let Some(accel) = sample.specific_force_mps2.as_ref() {
                    rec.log("plots/imu/accel_x_mps2", &rerun::Scalars::single(accel.x))?;
                }
            }
            MeasurementRecord::Camera(frame) => {
                let time_ns = frame
                    .header
                    .as_ref()
                    .map_or(0, |header| header.reported_stamp_ns);
                rec.set_duration_secs("time", seconds(time_ns));
                if let Some(header) = frame.header.as_ref() {
                    log_timing(rec, "camera", header)?;
                }
                let strips = frame
                    .features
                    .iter()
                    .map(|feature| vec![[0.0, 0.0], polar(3.0, feature.azimuth_rad)])
                    .collect::<Vec<_>>();
                let endpoints = frame
                    .features
                    .iter()
                    .map(|feature| polar(3.0, feature.azimuth_rad))
                    .collect::<Vec<_>>();
                rec.log(
                    "sensors/camera/bearings",
                    &rerun::LineStrips2D::new(strips)
                        .with_colors([CAMERA_COLOR])
                        .with_radii([0.025]),
                )?;
                rec.log(
                    "sensors/camera/features",
                    &rerun::Points2D::new(endpoints)
                        .with_colors([CAMERA_COLOR])
                        .with_radii([0.09]),
                )?;
                rec.log(
                    "plots/observations/camera",
                    &rerun::Scalars::single(frame.features.len() as f64),
                )?;
            }
            MeasurementRecord::Lidar(scan) => {
                let Some(header) = scan.header.as_ref() else {
                    continue;
                };
                rec.set_duration_secs("time", seconds(header.reported_stamp_ns));
                log_timing(rec, "lidar", header)?;
                let mut points = Vec::with_capacity(scan.returns.len());
                let mut rays = Vec::with_capacity(scan.returns.len());
                let mut colors = Vec::with_capacity(scan.returns.len());
                for return_ in &scan.returns {
                    let endpoint = polar(return_.range_m, return_.azimuth_rad);
                    points.push(endpoint);
                    rays.push(vec![[0.0, 0.0], endpoint]);
                    let fraction = if header.acquisition_duration_ns > 0 {
                        return_.acquisition_offset_ns as f64 / header.acquisition_duration_ns as f64
                    } else {
                        1.0
                    };
                    colors.push(lidar_time_color(fraction));
                }
                rec.log(
                    "sensors/lidar/returns",
                    &rerun::Points2D::new(points)
                        .with_colors(colors.clone())
                        .with_radii([rerun::Radius::new_ui_points(3.5)]),
                )?;
                rec.log(
                    "sensors/lidar/rays",
                    &rerun::LineStrips2D::new(rays)
                        .with_colors(colors)
                        .with_radii([rerun::Radius::new_ui_points(0.8)]),
                )?;
                log_lidar_platform_motion(rec, truth, header)?;
                rec.log(
                    "plots/observations/lidar",
                    &rerun::Scalars::single(scan.returns.len() as f64),
                )?;
            }
        }
    }
    Ok(())
}

fn log_timing(rec: &rerun::RecordingStream, sensor: &str, header: &RecordHeader) -> Result<()> {
    let age_ms = (header.receipt_time_ns - header.reported_stamp_ns) as f64 / 1.0e6;
    rec.log(
        format!("plots/timing/{sensor}_age_ms"),
        &rerun::Scalars::single(age_ms),
    )?;
    if header.acquisition_duration_ns > 0 {
        rec.log(
            format!("plots/timing/{sensor}_acquisition_ms"),
            &rerun::Scalars::single(header.acquisition_duration_ns as f64 / 1.0e6),
        )?;
    }
    Ok(())
}

fn log_lidar_platform_motion(
    rec: &rerun::RecordingStream,
    truth: &[TruthState],
    header: &RecordHeader,
) -> Result<()> {
    let start = header.reported_stamp_ns - header.acquisition_duration_ns;
    let Some(end_state) = nearest_truth(truth, header.reported_stamp_ns) else {
        return Ok(());
    };
    let Some(end_pose) = end_state.pose_w_b.as_ref() else {
        return Ok(());
    };
    let Some(end_position) = end_pose.position.as_ref() else {
        return Ok(());
    };
    let yaw = yaw_from_pose(end_pose);
    let (sin, cos) = yaw.sin_cos();
    let path = truth
        .iter()
        .filter(|state| (start..=header.reported_stamp_ns).contains(&state.truth_time_ns))
        .filter_map(|state| state.pose_w_b.as_ref()?.position.as_ref())
        .map(|position| {
            let dx = position.x - end_position.x;
            let dy = position.y - end_position.y;
            [(cos * dx + sin * dy) as f32, (-sin * dx + cos * dy) as f32]
        })
        .collect::<Vec<_>>();
    if path.len() >= 2 {
        rec.log(
            "sensors/lidar/platform_motion_during_scan",
            &rerun::LineStrips2D::new([path])
                .with_colors([TRUTH_COLOR])
                .with_radii([0.05]),
        )?;
    }
    Ok(())
}

fn lidar_time_color(fraction: f64) -> u32 {
    let f = fraction.clamp(0.0, 1.0);
    let mix = |early: f64, late: f64| (early + (late - early) * f).round() as u32;
    (mix(255.0, 70.0) << 24) | (mix(145.0, 220.0) << 16) | (mix(35.0, 255.0) << 8) | 255
}

fn send_dashboard_blueprint(rec: &rerun::RecordingStream) -> Result<()> {
    use rerun::blueprint::{
        Blueprint, BlueprintActivation, BlueprintPanel, Grid, Horizontal, SelectionPanel,
        Spatial2DView, TextDocumentView, TimePanel, TimeSeriesView, Vertical,
        components::{LoopMode, PanelState, PlayState},
    };

    let overview = Spatial2DView::new("Map: truth vs estimate").with_origin("map");
    let setup = TextDocumentView::new("Scenario").with_origin("dashboard/setup");
    let now = TextDocumentView::new("Motion").with_origin("dashboard/now");
    let guide = TextDocumentView::new("Guide").with_origin("dashboard/guide");
    let sensors = Grid::new([
        Spatial2DView::new("Camera bearings (no depth)")
            .with_origin("sensors/camera")
            .into(),
        Spatial2DView::new("Lidar scan (orange → cyan)")
            .with_origin("sensors/lidar")
            .into(),
    ])
    .with_grid_columns(2);
    let plots = Grid::new([
        TimeSeriesView::new("Error + 95% bounds")
            .with_origin("plots/error")
            .into(),
        TimeSeriesView::new("Measurement timing")
            .with_origin("plots/timing")
            .into(),
        TimeSeriesView::new("IMU").with_origin("plots/imu").into(),
        TimeSeriesView::new("Observations per frame")
            .with_origin("plots/observations")
            .into(),
        TimeSeriesView::new("Gyro-z bias")
            .with_origin("plots/bias/gyro/value")
            .into(),
        TimeSeriesView::new("Accelerometer-x bias")
            .with_origin("plots/bias/accel/value")
            .into(),
        TimeSeriesView::new("Gyro-z normalized error")
            .with_origin("plots/bias/gyro/normalized")
            .into(),
        TimeSeriesView::new("Accelerometer-x normalized error")
            .with_origin("plots/bias/accel/normalized")
            .into(),
    ])
    .with_grid_columns(4);
    let root = Vertical::new([
        Horizontal::new([
            overview.into(),
            Vertical::new([setup.into(), now.into(), guide.into()])
                .with_row_shares(vec![1.2, 1.0, 2.2])
                .into(),
        ])
        .with_column_shares(vec![3.6, 1.7])
        .into(),
        sensors.into(),
        plots.into(),
    ])
    .with_row_shares(vec![3.0, 2.4, 2.4]);

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

fn log_errors(
    rec: &rerun::RecordingStream,
    truth: &[TruthState],
    estimates: &[StateEstimate],
) -> Result<()> {
    for estimate in estimates {
        let (Some(estimate_pose), Some(truth_state)) = (
            estimate.pose_w_b.as_ref(),
            nearest_truth(truth, estimate.estimate_time_ns),
        ) else {
            continue;
        };
        let Some(truth_pose) = truth_state.pose_w_b.as_ref() else {
            continue;
        };
        let (Some(estimate_position), Some(truth_position)) = (
            estimate_pose.position.as_ref(),
            truth_pose.position.as_ref(),
        ) else {
            continue;
        };
        let position_error = ((estimate_position.x - truth_position.x).powi(2)
            + (estimate_position.y - truth_position.y).powi(2))
        .sqrt();
        let yaw_error = wrap_angle(yaw_from_pose(estimate_pose) - yaw_from_pose(truth_pose)).abs();
        rec.set_duration_secs("time", seconds(estimate.estimate_time_ns));
        rec.log(
            "plots/error/position_m",
            &rerun::Scalars::single(position_error),
        )?;
        rec.log("plots/error/yaw_rad", &rerun::Scalars::single(yaw_error))?;
        if let Some(covariance) = eval::validated_covariance(estimate)? {
            let (position_bound_m, yaw_bound_rad) = eval::error_bounds_95(&covariance);
            rec.log(
                "plots/error/position_bound_95_m",
                &rerun::Scalars::single(position_bound_m),
            )?;
            rec.log(
                "plots/error/yaw_bound_95_rad",
                &rerun::Scalars::single(yaw_bound_rad),
            )?;
        }
    }
    Ok(())
}

fn log_biases(
    rec: &rerun::RecordingStream,
    observation_truth: &[ObservationTruth],
    estimates: &[StateEstimate],
) -> Result<()> {
    let truth = eval::bias_truth_samples(observation_truth)?;
    for truth in &truth {
        rec.set_duration_secs("time", seconds(truth.reported_stamp_ns));
        rec.log(
            "plots/bias/gyro/value/true_radps",
            &rerun::Scalars::single(truth.gyro_bias_z_radps),
        )?;
        rec.log(
            "plots/bias/accel/value/true_mps2",
            &rerun::Scalars::single(truth.accel_bias_x_mps2),
        )?;
    }
    for estimate in estimates {
        let (Some(gyro_bias), Some(accel_bias)) =
            (estimate.gyro_bias_z_radps, estimate.accel_bias_x_mps2)
        else {
            continue;
        };
        rec.set_duration_secs("time", seconds(estimate.estimate_time_ns));
        rec.log(
            "plots/bias/gyro/value/estimate_radps",
            &rerun::Scalars::single(gyro_bias),
        )?;
        rec.log(
            "plots/bias/accel/value/estimate_mps2",
            &rerun::Scalars::single(accel_bias),
        )?;
        if let Some(covariance) = eval::validated_covariance(estimate)? {
            let (gyro_sigma, accel_sigma) = eval::bias_standard_deviations(&covariance);
            if let Ok(index) = truth.binary_search_by_key(&estimate.estimate_time_ns, |sample| {
                sample.reported_stamp_ns
            }) {
                let reference = truth[index];
                if gyro_sigma > 0.0 {
                    rec.log(
                        "plots/bias/gyro/normalized/error",
                        &rerun::Scalars::single(
                            (gyro_bias - reference.gyro_bias_z_radps) / gyro_sigma,
                        ),
                    )?;
                    rec.log(
                        "plots/bias/gyro/normalized/lower_95",
                        &rerun::Scalars::single(-eval::STANDARD_NORMAL_95),
                    )?;
                    rec.log(
                        "plots/bias/gyro/normalized/upper_95",
                        &rerun::Scalars::single(eval::STANDARD_NORMAL_95),
                    )?;
                }
                if accel_sigma > 0.0 {
                    rec.log(
                        "plots/bias/accel/normalized/error",
                        &rerun::Scalars::single(
                            (accel_bias - reference.accel_bias_x_mps2) / accel_sigma,
                        ),
                    )?;
                    rec.log(
                        "plots/bias/accel/normalized/lower_95",
                        &rerun::Scalars::single(-eval::STANDARD_NORMAL_95),
                    )?;
                    rec.log(
                        "plots/bias/accel/normalized/upper_95",
                        &rerun::Scalars::single(eval::STANDARD_NORMAL_95),
                    )?;
                }
            }
        }
    }
    Ok(())
}

fn nearest_truth(truth: &[TruthState], time_ns: i64) -> Option<&TruthState> {
    let index = truth.partition_point(|state| state.truth_time_ns < time_ns);
    match (index.checked_sub(1), truth.get(index)) {
        (None, next) => next,
        (Some(previous), None) => truth.get(previous),
        (Some(previous), Some(next)) => {
            let previous = &truth[previous];
            if (time_ns - previous.truth_time_ns).abs() <= (next.truth_time_ns - time_ns).abs() {
                Some(previous)
            } else {
                Some(next)
            }
        }
    }
}

fn point2(value: &fusion_schema::messages::Vec3) -> [f32; 2] {
    [value.x as f32, value.y as f32]
}

fn seconds(time_ns: i64) -> f64 {
    time_ns as f64 * 1.0e-9
}

fn polar(range_m: f64, azimuth_rad: f64) -> [f32; 2] {
    [
        (range_m * azimuth_rad.cos()) as f32,
        (range_m * azimuth_rad.sin()) as f32,
    ]
}

fn arc(radius_m: f64, angle_rad: f64, segments: usize) -> Vec<[f32; 2]> {
    let start = -angle_rad * 0.5;
    (0..=segments)
        .map(|index| {
            let fraction = index as f64 / segments as f64;
            polar(radius_m, start + angle_rad * fraction)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use fusion_schema::messages::TruthState;

    use super::nearest_truth;

    #[test]
    fn chooses_nearest_truth_sample() {
        let truth = [
            TruthState {
                truth_time_ns: 0,
                ..Default::default()
            },
            TruthState {
                truth_time_ns: 10,
                ..Default::default()
            },
        ];
        assert_eq!(nearest_truth(&truth, -2).unwrap().truth_time_ns, 0);
        assert_eq!(nearest_truth(&truth, 4).unwrap().truth_time_ns, 0);
        assert_eq!(nearest_truth(&truth, 6).unwrap().truth_time_ns, 10);
        assert_eq!(nearest_truth(&truth, 20).unwrap().truth_time_ns, 10);
    }
}
