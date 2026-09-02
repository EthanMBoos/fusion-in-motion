use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
};

use anyhow::{Context, Result, bail};
use fusion_schema::messages::{Pose, RecordHeader, StateEstimate, TruthState};

use crate::{
    bundle::{self, MeasurementRecord},
    math::{wrap_angle, yaw_from_pose},
    scenario::{ResolvedScenario, load_and_resolve},
};

const TRUTH_COLOR: u32 = 0x33CC66FF;
const ESTIMATE_COLOR: u32 = 0xFF5577FF;
const LANDMARK_COLOR: u32 = 0xFFD54FFF;
const CAMERA_COLOR: u32 = 0x33CCFFFF;
const LIDAR_COLOR: u32 = 0x4488FFFF;
const YAW_ERROR_COLOR: u32 = 0xFF9933FF;
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
    let truth = bundle::read_truth_states(&bundle_path.join("truth.mcap"))?;
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
        &rerun::TextDocument::new(format!(
            "# Scenario\n\n**Motion:** {:.2}x speed · {:.1} s simulated path\n\n**Camera:** {:.1} Hz · {:.1} ms latency\n\n**Lidar:** {:.1} Hz · {:.1} ms latency · {:.1} ms scan\n\n**Seed:** {}",
            scenario.motion_speed_factor,
            scenario.effective_duration_s(),
            scenario.camera.rate_hz,
            scenario.camera.latency_ns as f64 / 1.0e6,
            scenario.lidar.rate_hz,
            scenario.lidar.latency_ns as f64 / 1.0e6,
            scenario.lidar.scan_duration_ns as f64 / 1.0e6,
            scenario.root_seed,
        )),
    )?;
    rec.log_static(
        "dashboard/guide",
        &rerun::TextDocument::new(
            "# Dashboard\n\n**Map:** green truth and pink estimate.\n\n**Sensors:** camera rays show bearing only. Lidar points show range and bearing; color runs from early to late in the scan.\n\n**Timing:** age is receipt time minus reported timestamp. Acquisition duration is the interval covered by one record.\n\n**Error:** the gray line is the divergence threshold. These plots show accuracy, not covariance consistency.\n\nDrag the timeline to inspect one time across all panels.",
        ),
    )?;

    log_series_styles(&rec)?;
    log_static_map(&rec, &measurements, &truth, &estimates)?;
    log_sensor_references(&rec, &scenario)?;
    log_motion_segments(&rec, &scenario)?;
    log_vehicle_motion(&rec, &truth, &estimates)?;
    log_measurements(&rec, &measurements, &truth)?;
    log_errors(
        &rec,
        &truth,
        &estimates,
        scenario.metrics.divergence_position_error_m,
    )?;
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
            "plots/error/divergence_threshold_m",
            "divergence threshold (m)",
            REFERENCE_COLOR,
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
            .with_radii([0.055]),
    )?;

    let estimate_path = estimates
        .iter()
        .filter_map(|estimate| estimate.pose_w_b.as_ref()?.position.as_ref())
        .map(point2)
        .collect::<Vec<_>>();
    rec.log_static(
        "map/trajectories/estimate",
        &rerun::LineStrips2D::new([estimate_path])
            .with_colors([ESTIMATE_COLOR])
            .with_radii([0.025]),
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
                .with_labels(["platform forward"]),
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
    let rings = [max_range_m * 0.5, max_range_m]
        .into_iter()
        .map(|radius| arc(radius, fov_rad, 96))
        .collect::<Vec<_>>();
    rec.log_static(
        path,
        &rerun::LineStrips2D::new(rings)
            .with_colors([REFERENCE_COLOR])
            .with_radii([0.012]),
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
            &rerun::TextDocument::new(format!(
                "# Motion\n\n**{}** ({}/{})\n\nAcceleration: {:.2} m/s² · yaw rate: {:.2} rad/s",
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
                        .with_radii([0.14]),
                )?;
                rec.log(
                    "sensors/lidar/rays",
                    &rerun::LineStrips2D::new(rays)
                        .with_colors(colors)
                        .with_radii([0.012]),
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
    (mix(35.0, 120.0) << 24) | (mix(75.0, 225.0) << 16) | (255 << 8) | 255
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
        Spatial2DView::new("Camera bearing (no depth)")
            .with_origin("sensors/camera")
            .into(),
        Spatial2DView::new("Lidar range + bearing")
            .with_origin("sensors/lidar")
            .into(),
    ])
    .with_grid_columns(2);
    let plots = Grid::new([
        TimeSeriesView::new("Estimation error")
            .with_origin("plots/error")
            .into(),
        TimeSeriesView::new("Measurement timing")
            .with_origin("plots/timing")
            .into(),
        TimeSeriesView::new("IMU").with_origin("plots/imu").into(),
        TimeSeriesView::new("Observations per frame")
            .with_origin("plots/observations")
            .into(),
    ])
    .with_grid_columns(2);
    let root = Vertical::new([
        Horizontal::new([
            overview.into(),
            Vertical::new([setup.into(), now.into(), guide.into()])
                .with_row_shares(vec![1.2, 0.7, 2.6])
                .into(),
        ])
        .with_column_shares(vec![3.6, 1.7])
        .into(),
        sensors.into(),
        plots.into(),
    ])
    .with_row_shares(vec![3.0, 2.2, 1.8]);

    Blueprint::new(root)
        .with_auto_views(false)
        .with_auto_layout(false)
        .with_blueprint_panel(BlueprintPanel::new().with_state(PanelState::Collapsed))
        .with_selection_panel(SelectionPanel::new().with_state(PanelState::Collapsed))
        .with_time_panel(
            TimePanel::new()
                .with_state(PanelState::Expanded)
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
    divergence_threshold_m: f64,
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
        rec.log(
            "plots/error/divergence_threshold_m",
            &rerun::Scalars::single(divergence_threshold_m),
        )?;
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
