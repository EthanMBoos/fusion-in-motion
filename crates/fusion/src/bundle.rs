use std::{
    collections::BTreeMap,
    fs::{self, File},
    io::BufWriter,
    path::Path,
};

use anyhow::{Context, Result, bail};
use fusion_schema::{
    FILE_DESCRIPTOR_SET,
    messages::{
        CameraFrame, EgoStateEstimate, EgoTruthState, GpsFix, ImuBiasTruth, ImuSample, LidarScan,
        MeasurementTime, ObjectTrack, ObjectTruthState,
    },
};
use mcap::{Writer, records::MessageHeader};
use prost::Message;

use crate::{
    estimator::{BaselineAssumptions, GpsDiagnostics, TimingDiagnostics},
    eval::RunMetrics,
    scenario::{ResolvedScenario, canonical_yaml},
    tracker::TrackerDiagnostics,
};

#[derive(Debug, Clone)]
pub enum MeasurementRecord {
    Imu(ImuSample),
    Gps(GpsFix),
    Camera(CameraFrame),
    Lidar(LidarScan),
}

impl MeasurementRecord {
    pub fn time(&self) -> &MeasurementTime {
        match self {
            Self::Imu(value) => value.time.as_ref(),
            Self::Gps(value) => value.time.as_ref(),
            Self::Camera(value) => value.time.as_ref(),
            Self::Lidar(value) => value.time.as_ref(),
        }
        .expect("generated measurements have time")
    }
}

#[derive(Debug, Clone)]
pub struct GeneratedRun {
    pub measurements: Vec<MeasurementRecord>,
    pub ego_truth_states: Vec<EgoTruthState>,
    pub object_truth_states: Vec<ObjectTruthState>,
    pub imu_bias_truth: Vec<ImuBiasTruth>,
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
        &generated.imu_bias_truth,
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
            estimate.available_time_ns,
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
            track.available_time_ns,
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
    for item in mcap::MessageStream::new(&bytes)? {
        let message = item?;
        if previous_log_time.is_some_and(|time| message.log_time < time) {
            bail!("measurement MCAP is not in arrival order");
        }
        previous_log_time = Some(message.log_time);
        let schema = message
            .channel
            .schema
            .as_ref()
            .map(|schema| schema.name.as_str())
            .ok_or_else(|| anyhow::anyhow!("measurement channel has no schema"))?;
        records.push(match schema {
            "fusion.ImuSample" => MeasurementRecord::Imu(ImuSample::decode(message.data.as_ref())?),
            "fusion.GpsFix" => MeasurementRecord::Gps(GpsFix::decode(message.data.as_ref())?),
            "fusion.CameraFrame" => {
                MeasurementRecord::Camera(CameraFrame::decode(message.data.as_ref())?)
            }
            "fusion.LidarScan" => {
                MeasurementRecord::Lidar(LidarScan::decode(message.data.as_ref())?)
            }
            other => bail!("unsupported measurement schema {other}"),
        });
    }
    Ok(records)
}

pub fn read_ego_truth(path: &Path) -> Result<Vec<EgoTruthState>> {
    read_schema(path, "fusion.EgoTruthState")
}

pub fn read_object_truth(path: &Path) -> Result<Vec<ObjectTruthState>> {
    read_schema(path, "fusion.ObjectTruthState")
}

pub fn read_imu_bias_truth(path: &Path) -> Result<Vec<ImuBiasTruth>> {
    read_schema(path, "fusion.ImuBiasTruth")
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
    gps_diagnostics: &GpsDiagnostics,
    tracker_estimated_diagnostics: &TrackerDiagnostics,
    tracker_truth_diagnostics: &TrackerDiagnostics,
    assumptions: &BaselineAssumptions,
) -> Result<()> {
    let report_dir = output.join("reports/baseline");
    let json = serde_json::json!({
        "metrics": metrics,
        "ego_timing": ego_timing,
        "gps_fixes": gps_diagnostics,
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
         Position RMSE: {:.3} m  \nHeading RMSE: {:.3} rad  \nGPS fixes accepted/rejected/invalid: {}/{}/{}\n\n\
         ## Objects\n\n\
         Truth ego position RMSE: {:.3} m  \nEstimated ego position RMSE: {:.3} m  \nCost of estimated ego: {:+.3} m\n\n\
         Estimated-ego associations, camera/lidar: {}/{}  \nUnmatched camera/lidar detections: {}/{}  \nTracks created/confirmed/deleted: {}/{}/{}\n",
        metrics.ego.position_rmse_m,
        metrics.ego.yaw_rmse_rad,
        gps_diagnostics.accepted_fixes,
        gps_diagnostics.rejected_fixes,
        gps_diagnostics.invalid_fixes,
        metrics.tracks_with_truth_ego.position_rmse_m,
        metrics.tracks_with_estimated_ego.position_rmse_m,
        metrics.estimated_ego_position_rmse_delta_m,
        tracker_estimated_diagnostics.associated_camera_detections,
        tracker_estimated_diagnostics.associated_lidar_detections,
        tracker_estimated_diagnostics.unmatched_camera_detections,
        tracker_estimated_diagnostics.unmatched_lidar_detections,
        tracker_estimated_diagnostics.created_tracks,
        tracker_estimated_diagnostics.confirmed_tracks,
        tracker_estimated_diagnostics.deleted_tracks,
    );
    fs::write(report_dir.join("summary.md"), summary)?;
    Ok(())
}

fn write_measurements(path: &Path, records: &[MeasurementRecord]) -> Result<()> {
    let mut writer = new_writer(path)?;
    let mut channels = BTreeMap::new();
    for (schema_name, topic) in [
        ("fusion.ImuSample", "/measurement/imu"),
        ("fusion.GpsFix", "/measurement/gps"),
        ("fusion.CameraFrame", "/measurement/camera"),
        ("fusion.LidarScan", "/measurement/lidar"),
    ] {
        let schema = writer.add_schema(schema_name, "protobuf", FILE_DESCRIPTOR_SET)?;
        channels.insert(
            schema_name,
            writer.add_channel(schema, topic, "protobuf", &BTreeMap::new())?,
        );
    }
    for (sequence, record) in records.iter().enumerate() {
        let (schema, bytes) = match record {
            MeasurementRecord::Imu(value) => ("fusion.ImuSample", value.encode_to_vec()),
            MeasurementRecord::Gps(value) => ("fusion.GpsFix", value.encode_to_vec()),
            MeasurementRecord::Camera(value) => ("fusion.CameraFrame", value.encode_to_vec()),
            MeasurementRecord::Lidar(value) => ("fusion.LidarScan", value.encode_to_vec()),
        };
        let time = record.time();
        write_message(
            &mut writer,
            channels[schema],
            sequence as u32,
            time.arrival_time_ns,
            time.measurement_time_ns,
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
    imu_bias: &[ImuBiasTruth],
) -> Result<()> {
    let mut writer = new_writer(path)?;
    let ego_schema = writer.add_schema("fusion.EgoTruthState", "protobuf", FILE_DESCRIPTOR_SET)?;
    let object_schema =
        writer.add_schema("fusion.ObjectTruthState", "protobuf", FILE_DESCRIPTOR_SET)?;
    let bias_schema = writer.add_schema("fusion.ImuBiasTruth", "protobuf", FILE_DESCRIPTOR_SET)?;
    let ego_channel = writer.add_channel(ego_schema, "/truth/ego", "protobuf", &BTreeMap::new())?;
    let object_channel = writer.add_channel(
        object_schema,
        "/truth/objects",
        "protobuf",
        &BTreeMap::new(),
    )?;
    let bias_channel =
        writer.add_channel(bias_schema, "/truth/imu_bias", "protobuf", &BTreeMap::new())?;
    for (sequence, state) in ego.iter().enumerate() {
        write_message(
            &mut writer,
            ego_channel,
            sequence as u32,
            state.time_ns,
            state.time_ns,
            &state.encode_to_vec(),
        )?;
    }
    for (sequence, state) in objects.iter().enumerate() {
        write_message(
            &mut writer,
            object_channel,
            sequence as u32,
            state.time_ns,
            state.time_ns,
            &state.encode_to_vec(),
        )?;
    }
    for (sequence, state) in imu_bias.iter().enumerate() {
        write_message(
            &mut writer,
            bias_channel,
            sequence as u32,
            state.time_ns,
            state.time_ns,
            &state.encode_to_vec(),
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
