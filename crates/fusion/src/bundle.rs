use std::{
    collections::BTreeMap,
    fs::{self, File},
    io::BufWriter,
    path::Path,
    process::Command,
};

use anyhow::{Context, Result, bail};
use fusion_schema::{
    FILE_DESCRIPTOR_SET,
    messages::{
        CameraFrame, ImuSample, LandmarkMap, LidarScan, ObservationTruth, RecordHeader,
        StateEstimate, TruthState,
    },
};
use mcap::{Writer, records::MessageHeader};
use prost::Message;
use serde::{Deserialize, Serialize};

use crate::{
    estimator::{BaselineAssumptions, FilterDiagnostics, TimingDiagnostics},
    eval::{METRIC_ALIGNMENT, METRIC_VERSION, Metrics},
    scenario::{ResolvedScenario, canonical_yaml, sha256},
};

#[derive(Debug, Clone)]
pub enum MeasurementRecord {
    Map(LandmarkMap),
    Imu(ImuSample),
    Camera(CameraFrame),
    Lidar(LidarScan),
}

impl MeasurementRecord {
    pub fn header(&self) -> &RecordHeader {
        match self {
            Self::Map(message) => message.header.as_ref(),
            Self::Imu(message) => message.header.as_ref(),
            Self::Camera(message) => message.header.as_ref(),
            Self::Lidar(message) => message.header.as_ref(),
        }
        .expect("generated measurements always have headers")
    }

    pub fn header_mut(&mut self) -> &mut RecordHeader {
        match self {
            Self::Map(message) => message.header.as_mut(),
            Self::Imu(message) => message.header.as_mut(),
            Self::Camera(message) => message.header.as_mut(),
            Self::Lidar(message) => message.header.as_mut(),
        }
        .expect("generated measurements always have headers")
    }
}

#[derive(Debug, Clone)]
pub struct GeneratedRun {
    pub measurements: Vec<MeasurementRecord>,
    pub truth_states: Vec<TruthState>,
    pub observation_truth: Vec<ObservationTruth>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BundleManifest {
    pub format_version: u32,
    pub run_id: String,
    pub status: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    pub warnings: Vec<String>,
    pub source_scenario_sha256: String,
    pub resolved_scenario_sha256: String,
    pub simulator_commit: String,
    pub dirty_tree: bool,
    pub compiler: String,
    pub target: String,
    pub build_profile: String,
    pub operating_system: String,
    pub architecture: String,
    pub dependency_lock_sha256: String,
    pub determinism_level: String,
    pub floating_point_assumptions: String,
    pub root_seed: u64,
    pub random_algorithm: String,
    pub random_version: String,
    pub estimator_id: String,
    pub metric_version: String,
    pub metric_alignment: String,
    pub artifacts: BTreeMap<String, String>,
}

impl BundleManifest {
    pub fn start(scenario: &ResolvedScenario, source_path: &Path) -> Result<Self> {
        let source = fs::read(source_path)?;
        let resolved = canonical_yaml(scenario)?;
        let (commit, dirty_tree) = git_provenance();
        Ok(Self {
            format_version: scenario.format_version,
            run_id: scenario.run_id.clone(),
            status: "STARTED".to_owned(),
            error: None,
            warnings: vec![
                "initial analytic example: landmark observations use oracle association; raw images and point clouds are not simulated"
                    .to_owned(),
            ],
            source_scenario_sha256: sha256(&source),
            resolved_scenario_sha256: sha256(resolved.as_bytes()),
            simulator_commit: commit,
            dirty_tree,
            compiler: command_output("rustc", &["--version"]).unwrap_or_else(|| "unknown".to_owned()),
            target: format!("{}-{}", std::env::consts::ARCH, std::env::consts::OS),
            build_profile: if cfg!(debug_assertions) { "debug" } else { "release" }.to_owned(),
            operating_system: std::env::consts::OS.to_owned(),
            architecture: std::env::consts::ARCH.to_owned(),
            dependency_lock_sha256: file_hash(Path::new("Cargo.lock")).unwrap_or_else(|_| "unavailable".to_owned()),
            determinism_level: "logical_deterministic".to_owned(),
            floating_point_assumptions: "IEEE-754 f64; same target and dependency lock for reference results".to_owned(),
            root_seed: scenario.root_seed,
            random_algorithm: "SHA-256 to uniform f64; Box-Muller normal transform".to_owned(),
            random_version: "fusion-deterministic-random-1".to_owned(),
            estimator_id: scenario.estimator.id.clone(),
            metric_version: METRIC_VERSION.to_owned(),
            metric_alignment: METRIC_ALIGNMENT.to_owned(),
            artifacts: BTreeMap::new(),
        })
    }

    pub fn record_generated(&mut self, output: &Path) -> Result<()> {
        self.record(output, "scenario.resolved.yaml")?;
        self.record(output, "measurements.mcap")?;
        self.record(output, "truth.mcap")?;
        Ok(())
    }

    pub fn record_estimates(&mut self, output: &Path) -> Result<()> {
        self.record(output, "estimates/baseline.mcap")
    }

    pub fn finish(&mut self, output: &Path, _metrics: &Metrics) -> Result<()> {
        self.record(output, "reports/baseline/metrics.json")?;
        self.record(output, "reports/baseline/timing.json")?;
        self.record(output, "reports/baseline/diagnostics.json")?;
        self.record(output, "reports/baseline/assumptions.json")?;
        self.record(output, "reports/baseline/summary.md")?;
        let visualization = output.join("reports/baseline/visualization.rrd");
        if visualization.is_file() {
            self.record(output, "reports/baseline/visualization.rrd")?;
        }
        self.status = "COMPLETE".to_owned();
        self.write(output)
    }

    pub fn fail(&mut self, output: &Path, error: &anyhow::Error) -> Result<()> {
        self.status = "FAILED".to_owned();
        self.error = Some(format!("{error:#}"));
        self.write(output)
    }

    pub fn write(&self, output: &Path) -> Result<()> {
        fs::write(
            output.join("manifest.json"),
            serde_json::to_vec_pretty(self)?,
        )?;
        Ok(())
    }

    fn record(&mut self, output: &Path, relative: &str) -> Result<()> {
        self.artifacts
            .insert(relative.to_owned(), file_hash(&output.join(relative))?);
        Ok(())
    }
}

pub fn prepare(output: &Path, scenario: &ResolvedScenario) -> Result<()> {
    anyhow::ensure!(
        !output.exists(),
        "run directory {} already exists; omit --output to create the next numbered run",
        output.display()
    );
    fs::create_dir_all(output.join("estimates"))?;
    fs::create_dir_all(output.join("reports/baseline"))?;
    fs::write(
        output.join("scenario.resolved.yaml"),
        canonical_yaml(scenario)?,
    )?;
    Ok(())
}

pub fn write_generated(output: &Path, generated: &GeneratedRun) -> Result<()> {
    write_measurements(&output.join("measurements.mcap"), &generated.measurements)?;
    write_truth(
        &output.join("truth.mcap"),
        &generated.truth_states,
        &generated.observation_truth,
    )?;
    Ok(())
}

pub fn write_estimates(output: &Path, estimates: &[StateEstimate]) -> Result<()> {
    let path = output.join("estimates/baseline.mcap");
    write_estimates_file(&path, "baseline", estimates)
}

pub fn write_estimates_file(
    path: &Path,
    channel_name: &str,
    estimates: &[StateEstimate],
) -> Result<()> {
    let mut writer = new_writer(path)?;
    let schema = writer.add_schema("fusion.StateEstimate", "protobuf", FILE_DESCRIPTOR_SET)?;
    let channel = writer.add_channel(
        schema,
        &format!("/estimate/{channel_name}"),
        "protobuf",
        &BTreeMap::new(),
    )?;
    for (sequence, estimate) in estimates.iter().enumerate() {
        write_message(
            &mut writer,
            channel,
            sequence as u32,
            estimate.emission_time_ns,
            estimate.emission_time_ns,
            &estimate.encode_to_vec(),
        )?;
    }
    writer.finish()?;
    Ok(())
}

pub fn read_measurements(path: &Path) -> Result<Vec<MeasurementRecord>> {
    let bytes = fs::read(path).with_context(|| format!("failed to read {}", path.display()))?;
    let mut measurements = Vec::new();
    let mut previous_log_time = None;
    let mut previous_delivery = None;
    for item in mcap::MessageStream::new(&bytes)? {
        let message = item?;
        if let Some(previous) = previous_log_time
            && message.log_time < previous
        {
            bail!("measurement MCAP physical order is not causal arrival order");
        }
        previous_log_time = Some(message.log_time);
        let schema_name = message
            .channel
            .schema
            .as_ref()
            .map(|schema| schema.name.as_str())
            .ok_or_else(|| anyhow::anyhow!("measurement channel has no schema"))?;
        let record = match schema_name {
            "fusion.LandmarkMap" => {
                MeasurementRecord::Map(LandmarkMap::decode(message.data.as_ref())?)
            }
            "fusion.ImuSample" => MeasurementRecord::Imu(ImuSample::decode(message.data.as_ref())?),
            "fusion.CameraFrame" => {
                MeasurementRecord::Camera(CameraFrame::decode(message.data.as_ref())?)
            }
            "fusion.LidarScan" => {
                MeasurementRecord::Lidar(LidarScan::decode(message.data.as_ref())?)
            }
            other => bail!("unsupported estimator-visible schema {other}"),
        };
        if let Some(previous) = previous_delivery
            && record.header().delivery_index <= previous
        {
            bail!("measurement delivery_index is not strictly increasing");
        }
        previous_delivery = Some(record.header().delivery_index);
        measurements.push(record);
    }
    Ok(measurements)
}

pub fn read_truth_states(path: &Path) -> Result<Vec<TruthState>> {
    let bytes = fs::read(path).with_context(|| format!("failed to read {}", path.display()))?;
    let mut states = Vec::new();
    for item in mcap::MessageStream::new(&bytes)? {
        let message = item?;
        let schema_name = message
            .channel
            .schema
            .as_ref()
            .map(|schema| schema.name.as_str());
        if schema_name == Some("fusion.TruthState") {
            states.push(TruthState::decode(message.data.as_ref())?);
        }
    }
    Ok(states)
}

pub fn read_observation_truth(path: &Path) -> Result<Vec<ObservationTruth>> {
    let bytes = fs::read(path).with_context(|| format!("failed to read {}", path.display()))?;
    let mut observations = Vec::new();
    for item in mcap::MessageStream::new(&bytes)? {
        let message = item?;
        let schema_name = message
            .channel
            .schema
            .as_ref()
            .map(|schema| schema.name.as_str());
        if schema_name == Some("fusion.ObservationTruth") {
            observations.push(ObservationTruth::decode(message.data.as_ref())?);
        }
    }
    Ok(observations)
}

pub fn read_estimates(path: &Path) -> Result<Vec<StateEstimate>> {
    let bytes = fs::read(path).with_context(|| format!("failed to read {}", path.display()))?;
    let mut estimates = Vec::new();
    for item in mcap::MessageStream::new(&bytes)? {
        let message = item?;
        let schema_name = message
            .channel
            .schema
            .as_ref()
            .map(|schema| schema.name.as_str());
        if schema_name == Some("fusion.StateEstimate") {
            estimates.push(StateEstimate::decode(message.data.as_ref())?);
        }
    }
    Ok(estimates)
}

pub fn write_reports(
    output: &Path,
    metrics: &Metrics,
    scenario: &ResolvedScenario,
    timing: &TimingDiagnostics,
    diagnostics: &FilterDiagnostics,
    assumptions: &BaselineAssumptions,
) -> Result<()> {
    write_named_reports(output, "baseline", metrics)?;
    let report_dir = output.join("reports/baseline");
    fs::write(
        report_dir.join("timing.json"),
        serde_json::to_vec_pretty(timing)?,
    )?;
    fs::write(
        report_dir.join("diagnostics.json"),
        serde_json::to_vec_pretty(diagnostics)?,
    )?;
    fs::write(
        report_dir.join("assumptions.json"),
        serde_json::to_vec_pretty(assumptions)?,
    )?;
    let timing_mode = if timing.timing_compensation {
        format!(
            "fixed-lag replay and lidar deskew ({:.1} ms history)",
            timing.history_duration_ns as f64 / 1.0e6
        )
    } else {
        "arrival-time updates; lidar scans treated as instantaneous".to_owned()
    };
    let mut summary = format!(
        "# Results\n\n## Scenario\n\n\
         - Motion speed: {:.2}x\n\
         - Camera: {:.1} Hz, {:.1} ms latency, {:.0}% detection probability\n\
         - Lidar: {:.1} Hz, {:.1} ms latency, {:.1} ms scan duration\n\
         - Estimator timing: {}\n\
         - Random seed: {}\n\n\
         ## Result\n\n\
         - Position RMSE: {} m (position error over the run)\n\
         - Yaw RMSE: {} rad (heading error over the run)\n\
         - Final position error: {} m\n\
         - Maximum position error: {} m\n\
         - Matched valid outputs: {} / {}\n\
         - Valid output fraction: {:.1}%\n\
         - Output status: {} valid, {} initializing, {} diverged, {} unspecified, {} unknown\n\
         - Valid outputs outside the truth interval: {}\n\
         - First valid output: {} s\n\
         - Last valid estimate time: {} s\n\n",
        scenario.motion_speed_factor,
        scenario.camera.rate_hz,
        scenario.camera.latency_ns as f64 / 1.0e6,
        scenario.camera.detection_probability * 100.0,
        scenario.lidar.rate_hz,
        scenario.lidar.latency_ns as f64 / 1.0e6,
        scenario.lidar.scan_duration_ns as f64 / 1.0e6,
        timing_mode,
        scenario.root_seed,
        display_optional(metrics.position_rmse_m, 3),
        display_optional(metrics.yaw_rmse_rad, 3),
        display_optional(metrics.final_position_error_m, 3),
        display_optional(metrics.maximum_position_error_m, 3),
        metrics.matched_samples,
        metrics.valid_output_count,
        metrics.valid_output_fraction * 100.0,
        metrics.valid_output_count,
        metrics.initializing_output_count,
        metrics.diverged_output_count,
        metrics.unspecified_output_count,
        metrics.unknown_status_output_count,
        metrics.unmatched_valid_output_count,
        display_optional(metrics.time_to_first_valid_output_s, 3),
        display_optional(metrics.last_valid_estimate_time_s, 3),
    );
    summary.push_str(&format!(
        "## Filter diagnostics\n\n\
         - Scalar observation updates: {} applied / {} attempted\n\
         - Invalid scalar updates skipped: {}\n\n",
        diagnostics.applied_scalar_updates,
        diagnostics.attempted_scalar_updates,
        diagnostics.invalid_scalar_updates,
    ));
    summary.push_str(&format!(
        "## Timing processing\n\n\
         - Delayed measurements observed: {}\n\
         - Measurements replayed: {}\n\
         - Measurements discarded outside history: {}\n\
         - Revised estimates: {}\n\
         - Deskewed lidar scans: {} ({} returns)\n\
         - Maximum delivery age: {:.1} ms\n\n",
        timing.delayed_measurements,
        timing.replayed_measurements,
        timing.discarded_measurements,
        timing.revised_estimates,
        timing.deskewed_lidar_scans,
        timing.deskewed_lidar_returns,
        timing.maximum_delivery_age_ns as f64 / 1.0e6,
    ));
    let initial = &assumptions.initial_covariance_diagonal;
    summary.push_str(&format!(
        "## Estimator uncertainty\n\n\
         - Initial 1σ: x {:.3} m, y {:.3} m, yaw {:.3} rad, speed {:.3} m/s, gyro bias {:.3} rad/s, accel bias {:.3} m/s²\n\
         - IMU process noise: `scenario.imu` white noise and bias random walk\n\
         - Additional process noise: none\n\
         - Measurement 1σ: camera bearing {:.4} rad, lidar range {:.4} m, lidar bearing {:.4} rad\n\n",
        initial[0].sqrt(),
        initial[1].sqrt(),
        initial[2].sqrt(),
        initial[3].sqrt(),
        initial[4].sqrt(),
        initial[5].sqrt(),
        assumptions.camera_bearing_stddev_rad,
        assumptions.lidar_range_stddev_m,
        assumptions.lidar_bearing_stddev_rad,
    ));
    summary.push_str("## Covariance consistency\n\n");
    summary.push_str(&consistency_markdown(metrics));
    summary.push_str("\n## Bias states\n\n");
    summary.push_str(&bias_markdown(metrics));
    summary.push_str(
        "\nANEES from one time-correlated trajectory is a diagnostic. Use paired seeds before drawing a statistical conclusion.\n\n",
    );
    summary.push_str(&format!(
        "## Commands\n\n- `fusion view {}`\n- `fusion compare <baseline-run> {}`\n",
        output.display(),
        output.display(),
    ));
    fs::write(report_dir.join("summary.md"), summary)?;
    Ok(())
}

pub fn write_named_reports(output: &Path, name: &str, metrics: &Metrics) -> Result<()> {
    let report_dir = output.join("reports").join(name);
    fs::create_dir_all(&report_dir)?;
    fs::write(
        report_dir.join("metrics.json"),
        serde_json::to_vec_pretty(metrics)?,
    )?;
    let mut summary = format!(
        "# Fusion in Motion baseline result\n\n\
         - Estimator: `{}`\n\
         - Alignment: `{}`\n\
         - Matched valid outputs: {} / {}\n\
         - Position RMSE: {} m\n\
         - Yaw RMSE: {} rad\n\
         - Final position error: {} m\n\
         - Final drift per distance: {}\n\
         - Maximum position error: {} m\n\
         - Valid output fraction: {:.2}%\n\
         - Output status: {} valid, {} initializing, {} diverged, {} unspecified, {} unknown\n\
         - Valid outputs outside the truth interval: {}\n\
         - First valid output: {} s\n\
         - Last valid estimate time: {} s\n\n\
         ## Covariance consistency\n\n",
        metrics.estimator_id,
        metrics.alignment,
        metrics.matched_samples,
        metrics.valid_output_count,
        display_optional(metrics.position_rmse_m, 6),
        display_optional(metrics.yaw_rmse_rad, 6),
        display_optional(metrics.final_position_error_m, 6),
        display_optional(metrics.final_drift_per_distance, 6),
        display_optional(metrics.maximum_position_error_m, 6),
        metrics.valid_output_fraction * 100.0,
        metrics.valid_output_count,
        metrics.initializing_output_count,
        metrics.diverged_output_count,
        metrics.unspecified_output_count,
        metrics.unknown_status_output_count,
        metrics.unmatched_valid_output_count,
        display_optional(metrics.time_to_first_valid_output_s, 6),
        display_optional(metrics.last_valid_estimate_time_s, 6),
    );
    summary.push_str(&consistency_markdown(metrics));
    summary.push_str("\n## Bias states\n\n");
    summary.push_str(&bias_markdown(metrics));
    fs::write(report_dir.join("summary.md"), summary)?;
    Ok(())
}

fn display_optional(value: Option<f64>, precision: usize) -> String {
    value
        .map(|value| format!("{value:.precision$}"))
        .unwrap_or_else(|| "—".to_owned())
}

fn consistency_markdown(metrics: &Metrics) -> String {
    let Some(consistency) = metrics.covariance_consistency.as_ref() else {
        return format!(
            "- Full covariance samples: 0\n- Missing covariance samples: {}\n- ANEES and confidence coverage: unavailable\n",
            metrics.missing_covariance_samples
        );
    };
    let coverage = &consistency.marginal_coverage_95;
    format!(
        "- Full covariance samples: {}\n\
         - Missing covariance samples: {}\n\
         - ANEES ({} DoF): {:.3}\n\
         - Normalized ANEES: {:.3} (expected mean 1.000)\n\
         - Marginal 95% coverage: x {:.1}%, y {:.1}%, yaw {:.1}%, forward speed {:.1}%\n\
         - Covariance order: `[x, y, yaw, forward_speed, gyro_bias_z, accel_bias_x]`, row-major\n\
         - Error coordinates: additive world-frame x/y, wrapped world-from-body yaw, signed body-forward speed\n",
        metrics.full_covariance_samples,
        metrics.missing_covariance_samples,
        consistency.degrees_of_freedom,
        consistency.anees,
        consistency.normalized_anees,
        coverage.x_fraction * 100.0,
        coverage.y_fraction * 100.0,
        coverage.yaw_fraction * 100.0,
        coverage.forward_speed_fraction * 100.0,
    )
}

fn bias_markdown(metrics: &Metrics) -> String {
    let Some(bias) = metrics.bias_evaluation.as_ref() else {
        return "- Bias truth: unavailable\n".to_owned();
    };
    let mut output = format!(
        "- Alignment: {}\n\
         - Matched bias estimates: {} / {}\n\
         - Gyro-z bias RMSE: {} rad/s\n\
         - Accelerometer-x bias RMSE: {} m/s²\n\
         - Final gyro-z bias error: {} rad/s\n\
         - Final accelerometer-x bias error: {} m/s²\n",
        bias.alignment,
        bias.matched_samples,
        bias.estimate_samples,
        display_optional(bias.gyro_bias_z_rmse_radps, 6),
        display_optional(bias.accel_bias_x_rmse_mps2, 6),
        display_optional(bias.final_gyro_bias_z_error_radps, 6),
        display_optional(bias.final_accel_bias_x_error_mps2, 6),
    );
    if let Some(consistency) = bias.covariance_consistency.as_ref() {
        output.push_str(&format!(
            "- Bias ANEES (2 DoF): {:.3}\n\
             - Normalized bias ANEES: {:.3} (expected mean 1.000)\n\
             - Bias marginal 95% coverage: gyro z {:.1}%, accelerometer x {:.1}%\n",
            consistency.anees,
            consistency.normalized_anees,
            consistency.marginal_coverage_95.gyro_z_fraction * 100.0,
            consistency.marginal_coverage_95.accel_x_fraction * 100.0,
        ));
    } else {
        output.push_str("- Bias ANEES and confidence coverage: unavailable\n");
    }
    output
}

pub fn refresh_visualization_artifact(output: &Path) -> Result<()> {
    let manifest_path = output.join("manifest.json");
    if !manifest_path.is_file() {
        return Ok(());
    }
    let mut manifest: BundleManifest = serde_json::from_slice(&fs::read(&manifest_path)?)?;
    manifest.record(output, "reports/baseline/visualization.rrd")?;
    fs::write(manifest_path, serde_json::to_vec_pretty(&manifest)?)?;
    Ok(())
}

pub fn refresh_artifact(output: &Path, relative: &str) -> Result<()> {
    let manifest_path = output.join("manifest.json");
    let mut manifest: BundleManifest = serde_json::from_slice(&fs::read(&manifest_path)?)?;
    manifest.record(output, relative)?;
    fs::write(manifest_path, serde_json::to_vec_pretty(&manifest)?)?;
    Ok(())
}

fn write_measurements(path: &Path, measurements: &[MeasurementRecord]) -> Result<()> {
    let mut writer = new_writer(path)?;
    let map_schema = writer.add_schema("fusion.LandmarkMap", "protobuf", FILE_DESCRIPTOR_SET)?;
    let imu_schema = writer.add_schema("fusion.ImuSample", "protobuf", FILE_DESCRIPTOR_SET)?;
    let camera_schema = writer.add_schema("fusion.CameraFrame", "protobuf", FILE_DESCRIPTOR_SET)?;
    let lidar_schema = writer.add_schema("fusion.LidarScan", "protobuf", FILE_DESCRIPTOR_SET)?;
    let map_channel =
        writer.add_channel(map_schema, "/map/landmarks", "protobuf", &BTreeMap::new())?;
    let imu_channel = writer.add_channel(
        imu_schema,
        "/measurement/imu/primary",
        "protobuf",
        &BTreeMap::new(),
    )?;
    let camera_channel = writer.add_channel(
        camera_schema,
        "/measurement/camera/primary",
        "protobuf",
        &BTreeMap::new(),
    )?;
    let lidar_channel = writer.add_channel(
        lidar_schema,
        "/measurement/lidar/primary",
        "protobuf",
        &BTreeMap::new(),
    )?;
    for measurement in measurements {
        let header = measurement.header();
        match measurement {
            MeasurementRecord::Map(message) => write_message(
                &mut writer,
                map_channel,
                header.sensor_sequence as u32,
                header.receipt_time_ns,
                header.receipt_time_ns,
                &message.encode_to_vec(),
            )?,
            MeasurementRecord::Imu(message) => write_message(
                &mut writer,
                imu_channel,
                header.sensor_sequence as u32,
                header.receipt_time_ns,
                header.receipt_time_ns,
                &message.encode_to_vec(),
            )?,
            MeasurementRecord::Camera(message) => write_message(
                &mut writer,
                camera_channel,
                header.sensor_sequence as u32,
                header.receipt_time_ns,
                header.receipt_time_ns,
                &message.encode_to_vec(),
            )?,
            MeasurementRecord::Lidar(message) => write_message(
                &mut writer,
                lidar_channel,
                header.sensor_sequence as u32,
                header.receipt_time_ns,
                header.receipt_time_ns,
                &message.encode_to_vec(),
            )?,
        }
    }
    writer.finish()?;
    Ok(())
}

fn write_truth(
    path: &Path,
    states: &[TruthState],
    observations: &[ObservationTruth],
) -> Result<()> {
    let mut writer = new_writer(path)?;
    let state_schema = writer.add_schema("fusion.TruthState", "protobuf", FILE_DESCRIPTOR_SET)?;
    let observation_schema =
        writer.add_schema("fusion.ObservationTruth", "protobuf", FILE_DESCRIPTOR_SET)?;
    let state_channel =
        writer.add_channel(state_schema, "/truth/state", "protobuf", &BTreeMap::new())?;
    let observation_channel = writer.add_channel(
        observation_schema,
        "/truth/observation",
        "protobuf",
        &BTreeMap::new(),
    )?;
    for (sequence, state) in states.iter().enumerate() {
        write_message(
            &mut writer,
            state_channel,
            sequence as u32,
            state.truth_time_ns,
            state.truth_time_ns,
            &state.encode_to_vec(),
        )?;
    }
    for (sequence, observation) in observations.iter().enumerate() {
        write_message(
            &mut writer,
            observation_channel,
            sequence as u32,
            observation.arrival_truth_ns,
            observation.publish_truth_ns,
            &observation.encode_to_vec(),
        )?;
    }
    writer.finish()?;
    Ok(())
}

fn new_writer(path: &Path) -> Result<Writer<BufWriter<File>>> {
    let file =
        File::create(path).with_context(|| format!("failed to create {}", path.display()))?;
    Ok(Writer::new(BufWriter::new(file))?)
}

fn write_message<W: std::io::Write + std::io::Seek>(
    writer: &mut Writer<W>,
    channel_id: u16,
    sequence: u32,
    log_time_ns: i64,
    publish_time_ns: i64,
    data: &[u8],
) -> Result<()> {
    if log_time_ns < 0 || publish_time_ns < 0 {
        bail!(
            "MCAP log and publish times must be nonnegative; signed device time remains inside the payload"
        );
    }
    writer.write_to_known_channel(
        &MessageHeader {
            channel_id,
            sequence,
            log_time: log_time_ns as u64,
            publish_time: publish_time_ns as u64,
        },
        data,
    )?;
    Ok(())
}

fn file_hash(path: &Path) -> Result<String> {
    Ok(sha256(&fs::read(path).with_context(|| {
        format!("failed to hash {}", path.display())
    })?))
}

fn git_provenance() -> (String, bool) {
    let commit =
        command_output("git", &["rev-parse", "HEAD"]).unwrap_or_else(|| "unknown".to_owned());
    let dirty = Command::new("git")
        .args(["status", "--porcelain"])
        .output()
        .map(|output| !output.stdout.is_empty())
        .unwrap_or(true);
    (commit, dirty)
}

fn command_output(command: &str, args: &[&str]) -> Option<String> {
    let output = Command::new(command).args(args).output().ok()?;
    output
        .status
        .success()
        .then(|| String::from_utf8_lossy(&output.stdout).trim().to_owned())
}
