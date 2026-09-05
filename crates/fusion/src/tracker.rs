use std::collections::{BTreeMap, BTreeSet};

use anyhow::{Result, ensure};
use fusion_schema::messages::{
    CameraDetection, CameraFrame, EgoStateEstimate, EgoTruthState, LidarDetection, LidarScan,
    MeasurementTime, ObjectTrack, Vec2,
};
use nalgebra::{SMatrix, SVector, Vector2};
use serde::{Deserialize, Serialize};

use crate::{math, scenario::ObjectTrackerConfig};

type TrackVector = SVector<f64, 4>;
type TrackCovariance = SMatrix<f64, 4, 4>;
type LidarJacobian = SMatrix<f64, 2, 4>;
type LidarCovariance = SMatrix<f64, 2, 2>;

#[derive(Debug, Clone, Copy, PartialEq)]
struct TrackState {
    position_world_m: Vector2<f64>,
    velocity_world_mps: Vector2<f64>,
}

impl TrackState {
    fn new(position_world_m: Vector2<f64>, velocity_world_mps: Vector2<f64>) -> Self {
        Self {
            position_world_m,
            velocity_world_mps,
        }
    }

    fn with_correction(mut self, correction: TrackVector) -> Self {
        self.position_world_m.x += correction[TrackCoordinate::PositionX.index()];
        self.position_world_m.y += correction[TrackCoordinate::PositionY.index()];
        self.velocity_world_mps.x += correction[TrackCoordinate::VelocityX.index()];
        self.velocity_world_mps.y += correction[TrackCoordinate::VelocityY.index()];
        self
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(usize)]
enum TrackCoordinate {
    PositionX,
    PositionY,
    VelocityX,
    VelocityY,
}

impl TrackCoordinate {
    const fn index(self) -> usize {
        self as usize
    }
}

#[derive(Debug, Clone)]
pub enum PerceptionMeasurement {
    Camera(CameraFrame),
    Lidar(LidarScan),
}

impl PerceptionMeasurement {
    pub fn time(&self) -> &MeasurementTime {
        match self {
            Self::Camera(value) => value.time.as_ref(),
            Self::Lidar(value) => value.time.as_ref(),
        }
        .expect("generated perception measurements have time")
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TrackerDiagnostics {
    pub received_detections: usize,
    pub applied_updates: usize,
    pub rejected_updates: usize,
    pub invalid_updates: usize,
    pub waiting_for_range: usize,
    pub missing_ego_pose: usize,
    pub delayed_detections: usize,
    pub replayed_detections: usize,
    pub discarded_detections: usize,
    pub associated_camera_detections: usize,
    pub associated_lidar_detections: usize,
    pub unmatched_camera_detections: usize,
    pub unmatched_lidar_detections: usize,
    pub created_tracks: usize,
    pub confirmed_tracks: usize,
    pub deleted_tracks: usize,
}

#[derive(Debug)]
pub struct TrackerRun {
    pub tracks: Vec<ObjectTrack>,
    pub diagnostics: TrackerDiagnostics,
    pub processed_detections: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EgoSource {
    Estimated,
    Truth,
}

impl EgoSource {
    pub(crate) fn label(self) -> &'static str {
        match self {
            Self::Estimated => "estimated",
            Self::Truth => "truth",
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct EgoPose {
    time_ns: i64,
    position: Vector2<f64>,
    yaw: f64,
    covariance_xy_yaw: SMatrix<f64, 3, 3>,
}

#[derive(Debug)]
pub struct EgoHistory {
    samples: Vec<EgoPose>,
}

impl EgoHistory {
    pub fn from_estimates(estimates: &[EgoStateEstimate]) -> Result<Self> {
        let mut samples = Vec::with_capacity(estimates.len());
        for estimate in estimates {
            let pose = estimate
                .pose_world
                .as_ref()
                .ok_or_else(|| anyhow::anyhow!("ego estimate has no pose"))?;
            let position = pose
                .position
                .as_ref()
                .ok_or_else(|| anyhow::anyhow!("ego estimate pose has no position"))?;
            ensure!(
                matches!(estimate.state_covariance.len(), 16 | 36),
                "planar ego covariance must contain 16 or 36 values"
            );
            let dimension = if estimate.state_covariance.len() == 36 {
                6
            } else {
                4
            };
            let indices = [0, 1, 2];
            let covariance_xy_yaw = SMatrix::from_fn(|row, column| {
                estimate.state_covariance[indices[row] * dimension + indices[column]]
            });
            samples.push(EgoPose {
                time_ns: estimate.estimate_time_ns,
                position: Vector2::new(position.x, position.y),
                yaw: pose.yaw_rad,
                covariance_xy_yaw,
            });
        }
        Ok(Self { samples })
    }

    pub fn from_truth(truth: &[EgoTruthState]) -> Result<Self> {
        let mut samples = Vec::with_capacity(truth.len());
        for state in truth {
            let pose = state
                .pose_world
                .as_ref()
                .ok_or_else(|| anyhow::anyhow!("ego truth has no pose"))?;
            let position = pose
                .position
                .as_ref()
                .ok_or_else(|| anyhow::anyhow!("ego truth pose has no position"))?;
            samples.push(EgoPose {
                time_ns: state.time_ns,
                position: Vector2::new(position.x, position.y),
                yaw: pose.yaw_rad,
                covariance_xy_yaw: SMatrix::zeros(),
            });
        }
        Ok(Self { samples })
    }

    fn sample(&self, time_ns: i64) -> Option<EgoPose> {
        let index = self
            .samples
            .partition_point(|sample| sample.time_ns < time_ns);
        match (index.checked_sub(1), self.samples.get(index)) {
            (None, next) => next.copied(),
            (Some(previous), None) => self.samples.get(previous).copied(),
            (Some(previous), Some(next)) => {
                let previous = self.samples[previous];
                if next.time_ns == previous.time_ns {
                    return Some(previous);
                }
                let fraction = ((time_ns - previous.time_ns) as f64
                    / (next.time_ns - previous.time_ns) as f64)
                    .clamp(0.0, 1.0);
                Some(EgoPose {
                    time_ns,
                    position: previous.position + (next.position - previous.position) * fraction,
                    yaw: math::wrap_angle(
                        previous.yaw + math::wrap_angle(next.yaw - previous.yaw) * fraction,
                    ),
                    covariance_xy_yaw: previous.covariance_xy_yaw
                        + (next.covariance_xy_yaw - previous.covariance_xy_yaw) * fraction,
                })
            }
        }
    }
}

#[derive(Debug, Clone)]
enum Detection {
    Camera(CameraDetection),
    Lidar(LidarDetection),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SensorKind {
    Camera,
    Lidar,
}

#[derive(Debug, Clone)]
struct DetectionBatch {
    sensor: SensorKind,
    measurement_time_ns: i64,
    arrival_time_ns: i64,
    stable_id: String,
    detections: Vec<Detection>,
}

impl DetectionBatch {
    fn detection_time_ns(&self, detection: &Detection) -> i64 {
        match detection {
            Detection::Camera(_) => self.measurement_time_ns,
            Detection::Lidar(value) => value.measurement_time_ns,
        }
    }

    fn output_time_ns(&self) -> i64 {
        self.detections
            .iter()
            .map(|detection| self.detection_time_ns(detection))
            .max()
            .unwrap_or(self.measurement_time_ns)
    }
}

#[derive(Clone)]
struct Filter {
    state: TrackState,
    covariance: TrackCovariance,
    time_ns: i64,
}

struct ManagedTrack {
    filter: Filter,
    hits: usize,
    missed_lidar_scans: usize,
    confirmed: bool,
}

pub fn run(
    config: &ObjectTrackerConfig,
    measurements: &[PerceptionMeasurement],
    ego_history: &EgoHistory,
) -> Result<TrackerRun> {
    validate_delivery_order(measurements)?;
    let mut diagnostics = TrackerDiagnostics::default();
    let mut batches = flatten(measurements);
    diagnostics.received_detections = batches.iter().map(|batch| batch.detections.len()).sum();
    prepare_timing(config, &mut batches, &mut diagnostics);

    let mut filters = BTreeMap::<String, ManagedTrack>::new();
    let mut next_track_number = 1_u64;
    let mut tracks = Vec::new();
    let mut processed_detections = Vec::new();

    for batch in batches {
        let egos = batch
            .detections
            .iter()
            .map(|detection| ego_history.sample(batch.detection_time_ns(detection)))
            .collect::<Vec<_>>();
        diagnostics.missing_ego_pose += egos.iter().filter(|ego| ego.is_none()).count();

        let track_ids = filters.keys().cloned().collect::<Vec<_>>();
        let associations = associate(
            &track_ids,
            &filters,
            &batch,
            &egos,
            config.gate_sigma,
            config.acceleration_noise_stddev_mps2,
        );
        let matched_tracks = associations
            .iter()
            .map(|(track_index, _)| track_ids[*track_index].clone())
            .collect::<BTreeSet<_>>();
        let matched_detections = associations
            .iter()
            .map(|(_, detection_index)| *detection_index)
            .collect::<BTreeSet<_>>();

        for (track_index, detection_index) in associations {
            let track_id = &track_ids[track_index];
            let detection = &batch.detections[detection_index];
            let ego = egos[detection_index].expect("association requires ego pose");
            let detection_time_ns = batch.detection_time_ns(detection);
            let track = filters.get_mut(track_id).expect("associated track exists");
            propagate(
                &mut track.filter,
                detection_time_ns,
                config.acceleration_noise_stddev_mps2,
            )?;
            let result = match detection {
                Detection::Camera(value) => {
                    update_camera(&mut track.filter, value, ego, config.gate_sigma)
                }
                Detection::Lidar(value) => {
                    update_lidar(&mut track.filter, value, ego, config.gate_sigma)
                }
            };
            match result {
                TrackUpdate::Applied => {
                    diagnostics.applied_updates += 1;
                    match detection {
                        Detection::Camera(_) => {
                            diagnostics.associated_camera_detections += 1;
                            track.missed_lidar_scans = 0;
                        }
                        Detection::Lidar(_) => {
                            diagnostics.associated_lidar_detections += 1;
                            track.missed_lidar_scans = 0;
                        }
                    }
                    track.hits += 1;
                    if !track.confirmed && track.hits >= config.confirmation_hits {
                        track.confirmed = true;
                        diagnostics.confirmed_tracks += 1;
                    }
                }
                TrackUpdate::Rejected => diagnostics.rejected_updates += 1,
                TrackUpdate::Invalid => diagnostics.invalid_updates += 1,
            }
        }

        if batch.sensor == SensorKind::Lidar {
            for track_id in &track_ids {
                if !matched_tracks.contains(track_id) {
                    filters
                        .get_mut(track_id)
                        .expect("existing track")
                        .missed_lidar_scans += 1;
                }
            }
        }

        for (detection_index, detection) in batch.detections.iter().enumerate() {
            if matched_detections.contains(&detection_index) || egos[detection_index].is_none() {
                continue;
            }
            match detection {
                Detection::Camera(_) => {
                    diagnostics.unmatched_camera_detections += 1;
                    diagnostics.waiting_for_range += 1;
                }
                Detection::Lidar(value) => {
                    diagnostics.unmatched_lidar_detections += 1;
                    let track_id = format!("track-{next_track_number:03}");
                    next_track_number += 1;
                    let confirmed = config.confirmation_hits == 1;
                    filters.insert(
                        track_id,
                        ManagedTrack {
                            filter: initialize(
                                value,
                                egos[detection_index].expect("checked ego pose"),
                                batch.detection_time_ns(detection),
                            ),
                            hits: 1,
                            missed_lidar_scans: 0,
                            confirmed,
                        },
                    );
                    diagnostics.created_tracks += 1;
                    diagnostics.applied_updates += 1;
                    if confirmed {
                        diagnostics.confirmed_tracks += 1;
                    }
                }
            }
        }

        let before = filters.len();
        filters.retain(|_, track| track.missed_lidar_scans < config.max_missed_lidar_scans);
        diagnostics.deleted_tracks += before - filters.len();

        let output_time_ns = batch.output_time_ns();
        for (track_id, track) in filters.iter().filter(|(_, track)| track.confirmed) {
            let mut predicted = track.filter.clone();
            propagate(
                &mut predicted,
                output_time_ns,
                config.acceleration_noise_stddev_mps2,
            )?;
            tracks.push(to_track(&predicted, track_id, batch.arrival_time_ns));
        }

        processed_detections.extend(
            batch
                .detections
                .iter()
                .enumerate()
                .map(|(index, _)| format!("{}:{index}", batch.stable_id)),
        );
    }

    Ok(TrackerRun {
        tracks,
        diagnostics,
        processed_detections,
    })
}

fn prepare_timing(
    config: &ObjectTrackerConfig,
    batches: &mut Vec<DetectionBatch>,
    diagnostics: &mut TrackerDiagnostics,
) {
    if config.timing_compensation {
        let mut latest_measurement_time = None;
        batches.retain(|batch| {
            let age = latest_measurement_time
                .map(|time: i64| time.saturating_sub(batch.measurement_time_ns))
                .unwrap_or(0);
            let count = batch.detections.len();
            if age > 0 {
                diagnostics.delayed_detections += count;
            }
            latest_measurement_time = Some(
                latest_measurement_time.map_or(batch.measurement_time_ns, |time: i64| {
                    time.max(batch.measurement_time_ns)
                }),
            );
            if age > config.history_duration_ns {
                diagnostics.discarded_detections += count;
                false
            } else {
                if age > 0 {
                    diagnostics.replayed_detections += count;
                }
                true
            }
        });
        batches.sort_by(|left, right| {
            (
                left.measurement_time_ns,
                left.arrival_time_ns,
                &left.stable_id,
            )
                .cmp(&(
                    right.measurement_time_ns,
                    right.arrival_time_ns,
                    &right.stable_id,
                ))
        });
    } else {
        for batch in batches {
            batch.measurement_time_ns = batch.arrival_time_ns;
            for detection in &mut batch.detections {
                if let Detection::Lidar(value) = detection {
                    value.measurement_time_ns = batch.arrival_time_ns;
                }
            }
        }
    }
}

fn flatten(measurements: &[PerceptionMeasurement]) -> Vec<DetectionBatch> {
    measurements
        .iter()
        .enumerate()
        .map(|(record_index, measurement)| {
            let time = measurement.time();
            match measurement {
                PerceptionMeasurement::Camera(frame) => DetectionBatch {
                    sensor: SensorKind::Camera,
                    measurement_time_ns: time.measurement_time_ns,
                    arrival_time_ns: time.arrival_time_ns,
                    stable_id: format!("camera:{record_index}"),
                    detections: frame
                        .detections
                        .iter()
                        .cloned()
                        .map(Detection::Camera)
                        .collect(),
                },
                PerceptionMeasurement::Lidar(scan) => DetectionBatch {
                    sensor: SensorKind::Lidar,
                    measurement_time_ns: scan
                        .detections
                        .iter()
                        .map(|detection| detection.measurement_time_ns)
                        .min()
                        .unwrap_or(time.measurement_time_ns),
                    arrival_time_ns: time.arrival_time_ns,
                    stable_id: format!("lidar:{record_index}"),
                    detections: scan
                        .detections
                        .iter()
                        .cloned()
                        .map(Detection::Lidar)
                        .collect(),
                },
            }
        })
        .collect()
}

fn validate_delivery_order(measurements: &[PerceptionMeasurement]) -> Result<()> {
    let mut previous = None;
    for measurement in measurements {
        let delivery = measurement.time().arrival_time_ns;
        if let Some(previous) = previous {
            ensure!(
                delivery >= previous,
                "perception measurements are not in arrival order"
            );
        }
        previous = Some(delivery);
    }
    Ok(())
}

fn associate(
    track_ids: &[String],
    tracks: &BTreeMap<String, ManagedTrack>,
    batch: &DetectionBatch,
    egos: &[Option<EgoPose>],
    gate_sigma: f64,
    acceleration_noise_stddev_mps2: f64,
) -> Vec<(usize, usize)> {
    if track_ids.is_empty() || batch.detections.is_empty() {
        return Vec::new();
    }
    let unmatched_cost = gate_sigma.powi(2) + 1.0;
    let invalid_cost = unmatched_cost * 1.0e6;
    let mut costs = Vec::with_capacity(track_ids.len());
    for track_id in track_ids {
        let mut row = Vec::with_capacity(batch.detections.len() + track_ids.len());
        let track = &tracks[track_id];
        for (index, detection) in batch.detections.iter().enumerate() {
            let cost = egos[index]
                .and_then(|ego| {
                    let mut predicted = track.filter.clone();
                    propagate(
                        &mut predicted,
                        batch.detection_time_ns(detection),
                        acceleration_noise_stddev_mps2,
                    )
                    .ok()?;
                    normalized_innovation_squared(&predicted, detection, ego)
                })
                .filter(|cost| cost.sqrt() <= gate_sigma)
                .unwrap_or(invalid_cost);
            row.push(cost);
        }
        row.extend(std::iter::repeat_n(unmatched_cost, track_ids.len()));
        costs.push(row);
    }
    math::minimum_cost_assignment(&costs)
        .into_iter()
        .enumerate()
        .filter(|(track_index, detection_index)| {
            *detection_index < batch.detections.len()
                && costs[*track_index][*detection_index] < unmatched_cost
        })
        .collect()
}

fn normalized_innovation_squared(
    filter: &Filter,
    detection: &Detection,
    ego: EgoPose,
) -> Option<f64> {
    match detection {
        Detection::Camera(value) => {
            let (residual, jacobian, variance) = camera_innovation(filter, value, ego)?;
            Some(residual.powi(2) / innovation_variance(filter, jacobian, variance)?)
        }
        Detection::Lidar(value) => {
            let (residual, jacobian, covariance) = lidar_innovation(filter, value, ego)?;
            let innovation = jacobian * filter.covariance * jacobian.transpose() + covariance;
            let inverse = innovation.try_inverse()?;
            let cost = (residual.transpose() * inverse * residual)[0];
            cost.is_finite().then_some(cost)
        }
    }
}

fn initialize(detection: &LidarDetection, ego: EgoPose, time_ns: i64) -> Filter {
    let bearing_world = ego.yaw + detection.bearing_rad;
    let position =
        ego.position + Vector2::new(bearing_world.cos(), bearing_world.sin()) * detection.range_m;
    let tangential_variance = detection.range_m.powi(2) * detection.bearing_variance_rad2;
    let ego_position_variance = ego.covariance_xy_yaw[(0, 0)].max(ego.covariance_xy_yaw[(1, 1)]);
    let ego_yaw_variance = ego.covariance_xy_yaw[(2, 2)] * detection.range_m.powi(2);
    let position_variance = detection.range_variance_m2
        + tangential_variance
        + ego_position_variance
        + ego_yaw_variance;
    let position_x = TrackCoordinate::PositionX.index();
    let position_y = TrackCoordinate::PositionY.index();
    let velocity_x = TrackCoordinate::VelocityX.index();
    let velocity_y = TrackCoordinate::VelocityY.index();
    Filter {
        state: TrackState::new(position, Vector2::zeros()),
        covariance: {
            let mut covariance = TrackCovariance::zeros();
            covariance[(position_x, position_x)] = position_variance;
            covariance[(position_y, position_y)] = position_variance;
            covariance[(velocity_x, velocity_x)] = 4.0;
            covariance[(velocity_y, velocity_y)] = 4.0;
            covariance
        },
        time_ns,
    }
}

fn propagate(filter: &mut Filter, time_ns: i64, acceleration_noise_stddev_mps2: f64) -> Result<()> {
    let dt = (time_ns - filter.time_ns) as f64 * 1.0e-9;
    ensure!(dt >= 0.0, "tracker measurements are not time ordered");
    if dt == 0.0 {
        return Ok(());
    }
    let mut transition = TrackCovariance::identity();
    transition[(
        TrackCoordinate::PositionX.index(),
        TrackCoordinate::VelocityX.index(),
    )] = dt;
    transition[(
        TrackCoordinate::PositionY.index(),
        TrackCoordinate::VelocityY.index(),
    )] = dt;
    let q = acceleration_noise_stddev_mps2.powi(2);
    let dt2 = dt * dt;
    let dt3 = dt2 * dt;
    let dt4 = dt2 * dt2;
    let mut process_noise = TrackCovariance::zeros();
    for (position, velocity) in [
        (TrackCoordinate::PositionX, TrackCoordinate::VelocityX),
        (TrackCoordinate::PositionY, TrackCoordinate::VelocityY),
    ] {
        process_noise[(position.index(), position.index())] = 0.25 * dt4 * q;
        process_noise[(position.index(), velocity.index())] = 0.5 * dt3 * q;
        process_noise[(velocity.index(), position.index())] = 0.5 * dt3 * q;
        process_noise[(velocity.index(), velocity.index())] = dt2 * q;
    }
    filter.state.position_world_m += filter.state.velocity_world_mps * dt;
    filter.covariance = transition * filter.covariance * transition.transpose() + process_noise;
    filter.time_ns = time_ns;
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TrackUpdate {
    Applied,
    Rejected,
    Invalid,
}

fn camera_innovation(
    filter: &Filter,
    detection: &CameraDetection,
    ego: EgoPose,
) -> Option<(f64, TrackVector, f64)> {
    let displacement = filter.state.position_world_m - ego.position;
    let range_squared = displacement.norm_squared();
    if range_squared <= 1.0e-12 {
        return None;
    }
    let predicted = math::wrap_angle(displacement.y.atan2(displacement.x) - ego.yaw);
    let mut jacobian = TrackVector::zeros();
    jacobian[TrackCoordinate::PositionX.index()] = -displacement.y / range_squared;
    jacobian[TrackCoordinate::PositionY.index()] = displacement.x / range_squared;
    let ego_jacobian = SVector::<f64, 3>::new(
        displacement.y / range_squared,
        -displacement.x / range_squared,
        -1.0,
    );
    let ego_variance = (ego_jacobian.transpose() * ego.covariance_xy_yaw * ego_jacobian)[0];
    Some((
        math::wrap_angle(detection.bearing_rad - predicted),
        jacobian,
        detection.bearing_variance_rad2 + ego_variance.max(0.0),
    ))
}

fn lidar_innovation(
    filter: &Filter,
    detection: &LidarDetection,
    ego: EgoPose,
) -> Option<(Vector2<f64>, LidarJacobian, LidarCovariance)> {
    let displacement = filter.state.position_world_m - ego.position;
    let range_squared = displacement.norm_squared();
    let range = range_squared.sqrt();
    if range <= 1.0e-9 {
        return None;
    }
    let predicted_bearing = math::wrap_angle(displacement.y.atan2(displacement.x) - ego.yaw);
    let residual = Vector2::new(
        detection.range_m - range,
        math::wrap_angle(detection.bearing_rad - predicted_bearing),
    );
    let mut jacobian = LidarJacobian::zeros();
    jacobian[(0, TrackCoordinate::PositionX.index())] = displacement.x / range;
    jacobian[(0, TrackCoordinate::PositionY.index())] = displacement.y / range;
    jacobian[(1, TrackCoordinate::PositionX.index())] = -displacement.y / range_squared;
    jacobian[(1, TrackCoordinate::PositionY.index())] = displacement.x / range_squared;
    let ego_jacobian = SMatrix::<f64, 2, 3>::from_row_slice(&[
        -displacement.x / range,
        -displacement.y / range,
        0.0,
        displacement.y / range_squared,
        -displacement.x / range_squared,
        -1.0,
    ]);
    let sensor_covariance = LidarCovariance::from_diagonal(&Vector2::new(
        detection.range_variance_m2,
        detection.bearing_variance_rad2,
    ));
    let covariance =
        sensor_covariance + ego_jacobian * ego.covariance_xy_yaw * ego_jacobian.transpose();
    Some((residual, jacobian, covariance))
}

fn innovation_variance(
    filter: &Filter,
    jacobian: TrackVector,
    measurement_variance: f64,
) -> Option<f64> {
    if !measurement_variance.is_finite() || measurement_variance < 0.0 {
        return None;
    }
    let variance = (jacobian.transpose() * filter.covariance * jacobian)[0] + measurement_variance;
    (variance.is_finite() && variance > 1.0e-15).then_some(variance)
}

fn update_camera(
    filter: &mut Filter,
    detection: &CameraDetection,
    ego: EgoPose,
    gate_sigma: f64,
) -> TrackUpdate {
    let Some((residual, jacobian, measurement_variance)) =
        camera_innovation(filter, detection, ego)
    else {
        return TrackUpdate::Invalid;
    };
    apply_track_scalar(filter, residual, jacobian, measurement_variance, gate_sigma)
}

fn update_lidar(
    filter: &mut Filter,
    detection: &LidarDetection,
    ego: EgoPose,
    gate_sigma: f64,
) -> TrackUpdate {
    let Some((residual, jacobian, measurement_covariance)) =
        lidar_innovation(filter, detection, ego)
    else {
        return TrackUpdate::Invalid;
    };
    if !residual.iter().all(|value| value.is_finite())
        || !measurement_covariance.iter().all(|value| value.is_finite())
    {
        return TrackUpdate::Invalid;
    }
    let innovation = jacobian * filter.covariance * jacobian.transpose() + measurement_covariance;
    let Some(inverse) = innovation.try_inverse() else {
        return TrackUpdate::Invalid;
    };
    let normalized_squared = (residual.transpose() * inverse * residual)[0];
    if !normalized_squared.is_finite() {
        return TrackUpdate::Invalid;
    }
    if normalized_squared.sqrt() > gate_sigma {
        return TrackUpdate::Rejected;
    }
    let gain = filter.covariance * jacobian.transpose() * inverse;
    let state = filter.state.with_correction(gain * residual);
    let left = TrackCovariance::identity() - gain * jacobian;
    let covariance = left * filter.covariance * left.transpose()
        + gain * measurement_covariance * gain.transpose();
    commit_update(filter, state, covariance)
}

fn apply_track_scalar(
    filter: &mut Filter,
    residual: f64,
    jacobian: TrackVector,
    measurement_variance: f64,
    gate_sigma: f64,
) -> TrackUpdate {
    if !residual.is_finite() {
        return TrackUpdate::Invalid;
    }
    let Some(variance) = innovation_variance(filter, jacobian, measurement_variance) else {
        return TrackUpdate::Invalid;
    };
    if residual.abs() / variance.sqrt() > gate_sigma {
        return TrackUpdate::Rejected;
    }
    let gain = filter.covariance * jacobian / variance;
    let state = filter.state.with_correction(gain * residual);
    let left = TrackCovariance::identity() - gain * jacobian.transpose();
    let covariance = left * filter.covariance * left.transpose()
        + gain * measurement_variance * gain.transpose();
    commit_update(filter, state, covariance)
}

fn commit_update(
    filter: &mut Filter,
    state: TrackState,
    covariance: TrackCovariance,
) -> TrackUpdate {
    let covariance = 0.5 * (covariance + covariance.transpose());
    if !state.position_world_m.iter().all(|value| value.is_finite())
        || !state
            .velocity_world_mps
            .iter()
            .all(|value| value.is_finite())
        || !covariance.iter().all(|value| value.is_finite())
        || covariance.clone_owned().cholesky().is_none()
    {
        return TrackUpdate::Invalid;
    }
    filter.state = state;
    filter.covariance = covariance;
    TrackUpdate::Applied
}

fn to_track(filter: &Filter, track_id: &str, available_time_ns: i64) -> ObjectTrack {
    ObjectTrack {
        track_id: track_id.to_owned(),
        estimate_time_ns: filter.time_ns,
        available_time_ns,
        position_world_m: Some(Vec2 {
            x: filter.state.position_world_m.x,
            y: filter.state.position_world_m.y,
        }),
        velocity_world_mps: Some(Vec2 {
            x: filter.state.velocity_world_mps.x,
            y: filter.state.velocity_world_mps.y,
        }),
        state_covariance: (0..4)
            .flat_map(|row| (0..4).map(move |column| filter.covariance[(row, column)]))
            .collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn detection(range_m: f64, bearing_rad: f64, time_ns: i64) -> LidarDetection {
        LidarDetection {
            measurement_time_ns: time_ns,
            range_m,
            bearing_rad,
            range_variance_m2: 0.01,
            bearing_variance_rad2: 0.0001,
        }
    }

    fn ego() -> EgoPose {
        EgoPose {
            time_ns: 0,
            position: Vector2::zeros(),
            yaw: 0.0,
            covariance_xy_yaw: SMatrix::zeros(),
        }
    }

    #[test]
    fn ego_heading_error_moves_a_distant_object_sideways() {
        let correct = initialize(&detection(20.0, 0.0, 0), ego(), 0);
        let wrong = initialize(
            &detection(20.0, 0.0, 0),
            EgoPose {
                yaw: 1_f64.to_radians(),
                ..ego()
            },
            0,
        );
        assert!(
            (wrong.state.position_world_m.y - correct.state.position_world_m.y - 0.349).abs()
                < 0.002
        );
    }

    #[test]
    fn lidar_update_is_atomic_when_rejected() {
        let ego = ego();
        let mut filter = initialize(&detection(20.0, 0.0, 0), ego, 0);
        let original_state = filter.state;
        let original_covariance = filter.covariance;
        let outlier = detection(20.0, std::f64::consts::FRAC_PI_2, 0);

        assert_eq!(
            update_lidar(&mut filter, &outlier, ego, 3.0),
            TrackUpdate::Rejected
        );
        assert_eq!(filter.state, original_state);
        assert_eq!(filter.covariance, original_covariance);
    }

    #[test]
    fn association_follows_position_instead_of_detection_order() -> Result<()> {
        let time = |time_ns| MeasurementTime {
            measurement_time_ns: time_ns,
            arrival_time_ns: time_ns,
        };
        let measurements = vec![
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(0)),
                detections: vec![detection(5.0, 0.0, 0), detection(10.0, 0.0, 0)],
            }),
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(1_000_000_000)),
                detections: vec![
                    detection(10.1, 0.0, 1_000_000_000),
                    detection(5.1, 0.0, 1_000_000_000),
                ],
            }),
        ];
        let history = EgoHistory {
            samples: vec![
                ego(),
                EgoPose {
                    time_ns: 1_000_000_000,
                    ..ego()
                },
            ],
        };
        let result = run(&ObjectTrackerConfig::default(), &measurements, &history)?;
        let first = result
            .tracks
            .iter()
            .find(|track| track.track_id == "track-001")
            .unwrap();
        let second = result
            .tracks
            .iter()
            .find(|track| track.track_id == "track-002")
            .unwrap();
        assert!(first.position_world_m.as_ref().unwrap().x < 7.0);
        assert!(second.position_world_m.as_ref().unwrap().x > 8.0);
        assert_eq!(result.diagnostics.created_tracks, 2);
        assert_eq!(result.diagnostics.confirmed_tracks, 2);
        Ok(())
    }

    #[test]
    fn track_is_deleted_after_configured_unmatched_lidar_scans() -> Result<()> {
        let time = |time_ns| MeasurementTime {
            measurement_time_ns: time_ns,
            arrival_time_ns: time_ns,
        };
        let measurements = vec![
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(0)),
                detections: vec![detection(5.0, 0.0, 0)],
            }),
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(1_000_000_000)),
                detections: Vec::new(),
            }),
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(2_000_000_000)),
                detections: Vec::new(),
            }),
        ];
        let history = EgoHistory {
            samples: vec![
                ego(),
                EgoPose {
                    time_ns: 2_000_000_000,
                    ..ego()
                },
            ],
        };
        let config = ObjectTrackerConfig {
            confirmation_hits: 1,
            max_missed_lidar_scans: 2,
            ..ObjectTrackerConfig::default()
        };
        let result = run(&config, &measurements, &history)?;
        assert_eq!(result.diagnostics.created_tracks, 1);
        assert_eq!(result.diagnostics.confirmed_tracks, 1);
        assert_eq!(result.diagnostics.deleted_tracks, 1);
        assert!(
            result
                .tracks
                .iter()
                .all(|track| track.estimate_time_ns < 2_000_000_000)
        );
        Ok(())
    }
}
