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
        CameraFrame, EgoStateEstimate, EgoTruthState, GpsFix, ImuSample, LidarScan, ObjectTrack,
        ObjectTruthState, ObservationTruth, RecordHeader, SensorCalibration,
    },
};
use mcap::{Writer, records::MessageHeader};
use prost::Message;
use serde::{Deserialize, Serialize};

use crate::{
    estimator::{BaselineAssumptions, FilterDiagnostics, TimingDiagnostics},
    eval::RunMetrics,
    scenario::{ResolvedScenario, canonical_yaml, sha256},
    tracker::TrackerDiagnostics,
};

#[derive(Debug, Clone)]
pub enum MeasurementRecord {
    Calibration(SensorCalibration),
    Imu(ImuSample),
    Gps(GpsFix),
    Camera(CameraFrame),
    Lidar(LidarScan),
}

impl MeasurementRecord {
    pub fn header(&self) -> &RecordHeader {
        match self {
            Self::Calibration(value) => value.header.as_ref(),
            Self::Imu(value) => value.header.as_ref(),
            Self::Gps(value) => value.header.as_ref(),
            Self::Camera(value) => value.header.as_ref(),
            Self::Lidar(value) => value.header.as_ref(),
        }
        .expect("generated measurements have headers")
    }

    pub fn header_mut(&mut self) -> &mut RecordHeader {
        match self {
            Self::Calibration(value) => value.header.as_mut(),
            Self::Imu(value) => value.header.as_mut(),
            Self::Gps(value) => value.header.as_mut(),
            Self::Camera(value) => value.header.as_mut(),
            Self::Lidar(value) => value.header.as_mut(),
        }
        .expect("generated measurements have headers")
    }
}

#[derive(Debug, Clone)]
pub struct GeneratedRun {
    pub measurements: Vec<MeasurementRecord>,
    pub ego_truth_states: Vec<EgoTruthState>,
    pub object_truth_states: Vec<ObjectTruthState>,
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
    pub root_seed: u64,
    pub simulator_commit: String,
    pub artifacts: BTreeMap<String, String>,
}

impl BundleManifest {
    pub fn start(scenario: &ResolvedScenario) -> Self {
        Self {
            format_version: scenario.format_version,
            run_id: scenario.run_id.clone(),
            status: "STARTED".to_owned(),
            error: None,
            warnings: vec![
                "Camera and lidar are analytic object detections, not raw images or point clouds. Association is supplied.".to_owned(),
                "The baseline estimators are planar; the measurement API carries 3D values for later platform models.".to_owned(),
            ],
            root_seed: scenario.root_seed,
            simulator_commit: command_output("git", &["rev-parse", "HEAD"]).unwrap_or_else(|| "unknown".to_owned()),
            artifacts: BTreeMap::new(),
        }
    }

    pub fn record_generated(&mut self, output: &Path) -> Result<()> {
        for relative in ["scenario.resolved.yaml", "measurements.mcap", "truth.mcap"] {
            self.record(output, relative)?;
        }
        Ok(())
    }

    pub fn record_outputs(&mut self, output: &Path) -> Result<()> {
        for relative in [
            "estimates/ego-baseline.mcap",
            "tracks/estimated-ego.mcap",
            "tracks/truth-ego.mcap",
        ] {
            self.record(output, relative)?;
        }
        Ok(())
    }

    pub fn finish(&mut self, output: &Path) -> Result<()> {
        for relative in [
            "reports/baseline/metrics.json",
            "reports/baseline/summary.md",
        ] {
            self.record(output, relative)?;
        }
        let visualization = "reports/baseline/visualization.rrd";
        if output.join(visualization).is_file() {
            self.record(output, visualization)?;
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
    if output.exists() {
        bail!("output directory {} already exists", output.display());
    }
    fs::create_dir_all(output.join("estimates"))?;
    fs::create_dir_all(output.join("tracks"))?;
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
        &generated.ego_truth_states,
        &generated.object_truth_states,
        &generated.observation_truth,
    )
}

pub fn write_ego_estimates(output: &Path, estimates: &[EgoStateEstimate]) -> Result<()> {
    write_ego_estimates_file(
        &output.join("estimates/ego-baseline.mcap"),
        "ego-baseline",
        estimates,
    )
}

pub fn write_ego_estimates_file(
    path: &Path,
    name: &str,
    estimates: &[EgoStateEstimate],
) -> Result<()> {
    let mut writer = new_writer(path)?;
    let schema = writer.add_schema("fusion.EgoStateEstimate", "protobuf", FILE_DESCRIPTOR_SET)?;
    let channel = writer.add_channel(
        schema,
        &format!("/estimate/ego/{name}"),
        "protobuf",
        &BTreeMap::new(),
    )?;
    for (sequence, estimate) in estimates.iter().enumerate() {
        write_message(
            &mut writer,
            channel,
            sequence as u32,
            estimate.emission_time_ns,
            estimate.estimate_time_ns,
            &estimate.encode_to_vec(),
        )?;
    }
    writer.finish()?;
    Ok(())
}

pub fn write_tracks(output: &Path, name: &str, tracks: &[ObjectTrack]) -> Result<()> {
    write_tracks_file(
        &output.join("tracks").join(format!("{name}.mcap")),
        name,
        tracks,
    )
}

pub fn write_tracks_file(path: &Path, name: &str, tracks: &[ObjectTrack]) -> Result<()> {
    let mut writer = new_writer(path)?;
    let schema = writer.add_schema("fusion.ObjectTrack", "protobuf", FILE_DESCRIPTOR_SET)?;
    let channel = writer.add_channel(
        schema,
        &format!("/track/object/{name}"),
        "protobuf",
        &BTreeMap::new(),
    )?;
    for (sequence, track) in tracks.iter().enumerate() {
        write_message(
            &mut writer,
            channel,
            sequence as u32,
            track.emission_time_ns,
            track.estimate_time_ns,
            &track.encode_to_vec(),
        )?;
    }
    writer.finish()?;
    Ok(())
}

pub fn read_measurements(path: &Path) -> Result<Vec<MeasurementRecord>> {
    let bytes = fs::read(path).with_context(|| format!("failed to read {}", path.display()))?;
    let mut records = Vec::new();
    let mut previous_log_time = None;
    let mut previous_delivery = None;
    for item in mcap::MessageStream::new(&bytes)? {
        let message = item?;
        if previous_log_time.is_some_and(|time| message.log_time < time) {
            bail!("measurement MCAP is not in receipt order");
        }
        previous_log_time = Some(message.log_time);
        let schema = message
            .channel
            .schema
            .as_ref()
            .map(|schema| schema.name.as_str())
            .ok_or_else(|| anyhow::anyhow!("measurement channel has no schema"))?;
        let record = match schema {
            "fusion.SensorCalibration" => {
                MeasurementRecord::Calibration(SensorCalibration::decode(message.data.as_ref())?)
            }
            "fusion.ImuSample" => MeasurementRecord::Imu(ImuSample::decode(message.data.as_ref())?),
            "fusion.GpsFix" => MeasurementRecord::Gps(GpsFix::decode(message.data.as_ref())?),
            "fusion.CameraFrame" => {
                MeasurementRecord::Camera(CameraFrame::decode(message.data.as_ref())?)
            }
            "fusion.LidarScan" => {
                MeasurementRecord::Lidar(LidarScan::decode(message.data.as_ref())?)
            }
            other => bail!("unsupported measurement schema {other}"),
        };
        if previous_delivery.is_some_and(|delivery| record.header().delivery_index <= delivery) {
            bail!("measurement delivery indices are not increasing");
        }
        previous_delivery = Some(record.header().delivery_index);
        records.push(record);
    }
    Ok(records)
}

pub fn read_ego_truth(path: &Path) -> Result<Vec<EgoTruthState>> {
    read_schema(path, "fusion.EgoTruthState")
}

pub fn read_object_truth(path: &Path) -> Result<Vec<ObjectTruthState>> {
    read_schema(path, "fusion.ObjectTruthState")
}

pub fn read_observation_truth(path: &Path) -> Result<Vec<ObservationTruth>> {
    read_schema(path, "fusion.ObservationTruth")
}

pub fn read_ego_estimates(path: &Path) -> Result<Vec<EgoStateEstimate>> {
    read_schema(path, "fusion.EgoStateEstimate")
}

pub fn read_tracks(path: &Path) -> Result<Vec<ObjectTrack>> {
    read_schema(path, "fusion.ObjectTrack")
}

fn read_schema<T: Message + Default>(path: &Path, expected: &str) -> Result<Vec<T>> {
    let bytes = fs::read(path).with_context(|| format!("failed to read {}", path.display()))?;
    let mut values = Vec::new();
    for item in mcap::MessageStream::new(&bytes)? {
        let message = item?;
        if message
            .channel
            .schema
            .as_ref()
            .map(|schema| schema.name.as_str())
            == Some(expected)
        {
            values.push(T::decode(message.data.as_ref())?);
        }
    }
    Ok(values)
}

#[allow(clippy::too_many_arguments)]
pub fn write_reports(
    output: &Path,
    metrics: &RunMetrics,
    ego_timing: &TimingDiagnostics,
    ego_diagnostics: &FilterDiagnostics,
    tracker_estimated_diagnostics: &TrackerDiagnostics,
    tracker_truth_diagnostics: &TrackerDiagnostics,
    assumptions: &BaselineAssumptions,
) -> Result<()> {
    let report_dir = output.join("reports/baseline");
    let json = serde_json::json!({
        "metrics": metrics,
        "ego_timing": ego_timing,
        "ego_updates": ego_diagnostics,
        "estimated_ego_tracker_updates": tracker_estimated_diagnostics,
        "truth_ego_tracker_updates": tracker_truth_diagnostics,
        "ego_filter_assumptions": assumptions,
    });
    fs::write(
        report_dir.join("metrics.json"),
        serde_json::to_vec_pretty(&json)?,
    )?;
    let summary = format!(
        "# Run result\n\n\
         GPS and IMU estimate the vehicle. Camera and lidar track objects.\n\n\
         ## Vehicle\n\n\
         Position RMSE: {:.3} m  \nHeading RMSE: {:.3} rad  \nGPS updates applied/rejected/invalid: {}/{}/{}\n\n\
         ## Objects\n\n\
         Truth ego position RMSE: {:.3} m  \nEstimated ego position RMSE: {:.3} m  \nCost of estimated ego: {:+.3} m\n\n\
         Estimated-ego tracker updates applied/rejected/invalid/waiting: {}/{}/{}/{}\n",
        metrics.ego.position_rmse_m,
        metrics.ego.yaw_rmse_rad,
        ego_diagnostics.applied_updates,
        ego_diagnostics.rejected_updates,
        ego_diagnostics.invalid_updates,
        metrics.tracks_with_truth_ego.position_rmse_m,
        metrics.tracks_with_estimated_ego.position_rmse_m,
        metrics.estimated_ego_position_rmse_delta_m,
        tracker_estimated_diagnostics.applied_updates,
        tracker_estimated_diagnostics.rejected_updates,
        tracker_estimated_diagnostics.invalid_updates,
        tracker_estimated_diagnostics.waiting_for_range,
    );
    fs::write(report_dir.join("summary.md"), summary)?;
    Ok(())
}

pub fn refresh_artifact(output: &Path, relative: &str) -> Result<()> {
    let path = output.join("manifest.json");
    let mut manifest: BundleManifest = serde_json::from_slice(&fs::read(&path)?)?;
    manifest.record(output, relative)?;
    manifest.write(output)
}

fn write_measurements(path: &Path, records: &[MeasurementRecord]) -> Result<()> {
    let mut writer = new_writer(path)?;
    let mut channels = BTreeMap::new();
    for (schema_name, topic) in [
        ("fusion.SensorCalibration", "/calibration/sensors"),
        ("fusion.ImuSample", "/measurement/imu/primary"),
        ("fusion.GpsFix", "/measurement/gps/primary"),
        ("fusion.CameraFrame", "/measurement/camera/primary"),
        ("fusion.LidarScan", "/measurement/lidar/primary"),
    ] {
        let schema = writer.add_schema(schema_name, "protobuf", FILE_DESCRIPTOR_SET)?;
        channels.insert(
            schema_name,
            writer.add_channel(schema, topic, "protobuf", &BTreeMap::new())?,
        );
    }
    for record in records {
        let (schema, bytes) = match record {
            MeasurementRecord::Calibration(value) => {
                ("fusion.SensorCalibration", value.encode_to_vec())
            }
            MeasurementRecord::Imu(value) => ("fusion.ImuSample", value.encode_to_vec()),
            MeasurementRecord::Gps(value) => ("fusion.GpsFix", value.encode_to_vec()),
            MeasurementRecord::Camera(value) => ("fusion.CameraFrame", value.encode_to_vec()),
            MeasurementRecord::Lidar(value) => ("fusion.LidarScan", value.encode_to_vec()),
        };
        let header = record.header();
        write_message(
            &mut writer,
            channels[schema],
            header.sensor_sequence as u32,
            header.receipt_time_ns,
            header.reported_stamp_ns,
            &bytes,
        )?;
    }
    writer.finish()?;
    Ok(())
}

fn write_truth(
    path: &Path,
    ego: &[EgoTruthState],
    objects: &[ObjectTruthState],
    observations: &[ObservationTruth],
) -> Result<()> {
    let mut writer = new_writer(path)?;
    let ego_schema = writer.add_schema("fusion.EgoTruthState", "protobuf", FILE_DESCRIPTOR_SET)?;
    let object_schema =
        writer.add_schema("fusion.ObjectTruthState", "protobuf", FILE_DESCRIPTOR_SET)?;
    let observation_schema =
        writer.add_schema("fusion.ObservationTruth", "protobuf", FILE_DESCRIPTOR_SET)?;
    let ego_channel = writer.add_channel(ego_schema, "/truth/ego", "protobuf", &BTreeMap::new())?;
    let object_channel = writer.add_channel(
        object_schema,
        "/truth/objects",
        "protobuf",
        &BTreeMap::new(),
    )?;
    let observation_channel = writer.add_channel(
        observation_schema,
        "/truth/observations",
        "protobuf",
        &BTreeMap::new(),
    )?;
    for (sequence, state) in ego.iter().enumerate() {
        write_message(
            &mut writer,
            ego_channel,
            sequence as u32,
            state.truth_time_ns,
            state.truth_time_ns,
            &state.encode_to_vec(),
        )?;
    }
    for (sequence, state) in objects.iter().enumerate() {
        write_message(
            &mut writer,
            object_channel,
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
        bail!("MCAP times must be nonnegative");
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

fn command_output(command: &str, args: &[&str]) -> Option<String> {
    let output = Command::new(command).args(args).output().ok()?;
    output
        .status
        .success()
        .then(|| String::from_utf8_lossy(&output.stdout).trim().to_owned())
}
