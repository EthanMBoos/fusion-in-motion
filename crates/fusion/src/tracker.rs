use std::collections::BTreeMap;

use anyhow::{Result, ensure};
use fusion_schema::messages::{
    CameraDetection, CameraFrame, CovarianceKind, EgoSource, EgoStateEstimate, EgoTruthState,
    EstimateStatus, LidarDetection, LidarScan, ObjectStateModel, ObjectTrack, RecordHeader, Vec3,
};
use nalgebra::{SMatrix, SVector, Vector2};
use serde::{Deserialize, Serialize};

use crate::{
    math,
    scenario::{CameraConfig, LidarConfig, ObjectTrackerConfig, SensorMountConfig},
};

type TrackState = SVector<f64, 4>;
type TrackCovariance = SMatrix<f64, 4, 4>;

#[derive(Debug, Clone)]
pub enum PerceptionMeasurement {
    Camera(CameraFrame),
    Lidar(LidarScan),
}

impl PerceptionMeasurement {
    pub fn header(&self) -> &RecordHeader {
        match self {
            Self::Camera(value) => value.header.as_ref(),
            Self::Lidar(value) => value.header.as_ref(),
        }
        .expect("generated perception measurements have headers")
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
}

#[derive(Debug)]
pub struct TrackerRun {
    pub tracks: Vec<ObjectTrack>,
    pub diagnostics: TrackerDiagnostics,
    pub processed_detection_ids: Vec<String>,
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
                .pose_w_b
                .as_ref()
                .ok_or_else(|| anyhow::anyhow!("ego estimate has no pose"))?;
            let position = pose
                .position
                .as_ref()
                .ok_or_else(|| anyhow::anyhow!("ego estimate pose has no position"))?;
            ensure!(
                estimate.covariance.len() == 36,
                "planar ego covariance must contain 36 values"
            );
            let indices = [0, 1, 2];
            let covariance_xy_yaw = SMatrix::from_fn(|row, column| {
                estimate.covariance[indices[row] * 6 + indices[column]]
            });
            samples.push(EgoPose {
                time_ns: estimate.estimate_time_ns,
                position: Vector2::new(position.x, position.y),
                yaw: math::yaw_from_pose(pose),
                covariance_xy_yaw,
            });
        }
        Ok(Self { samples })
    }

    pub fn from_truth(truth: &[EgoTruthState]) -> Result<Self> {
        let mut samples = Vec::with_capacity(truth.len());
        for state in truth {
            let pose = state
                .pose_w_b
                .as_ref()
                .ok_or_else(|| anyhow::anyhow!("ego truth has no pose"))?;
            let position = pose
                .position
                .as_ref()
                .ok_or_else(|| anyhow::anyhow!("ego truth pose has no position"))?;
            samples.push(EgoPose {
                time_ns: state.truth_time_ns,
                position: Vector2::new(position.x, position.y),
                yaw: math::yaw_from_pose(pose),
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

#[derive(Debug, Clone)]
struct TimedDetection {
    measurement_time_ns: i64,
    receipt_time_ns: i64,
    delivery_index: u64,
    detection: Detection,
}

impl TimedDetection {
    fn id(&self) -> &str {
        match &self.detection {
            Detection::Camera(value) => &value.detection_id,
            Detection::Lidar(value) => &value.detection_id,
        }
    }

    fn association_key(&self) -> &str {
        match &self.detection {
            Detection::Camera(value) => &value.association_key,
            Detection::Lidar(value) => &value.association_key,
        }
    }
}

#[derive(Clone)]
struct Filter {
    state: TrackState,
    covariance: TrackCovariance,
    time_ns: i64,
}

pub fn run(
    config: &ObjectTrackerConfig,
    camera: &CameraConfig,
    lidar: &LidarConfig,
    measurements: &[PerceptionMeasurement],
    ego_history: &EgoHistory,
    ego_source: EgoSource,
    world_frame: &str,
) -> Result<TrackerRun> {
    validate_delivery_order(measurements)?;
    let mut diagnostics = TrackerDiagnostics::default();
    let mut detections = flatten(measurements);
    diagnostics.received_detections = detections.len();
    if config.timing_compensation {
        let mut latest_measurement_time = None;
        detections.retain(|detection| {
            let age = latest_measurement_time
                .map(|time: i64| time.saturating_sub(detection.measurement_time_ns))
                .unwrap_or(0);
            if age > 0 {
                diagnostics.delayed_detections += 1;
            }
            latest_measurement_time = Some(
                latest_measurement_time.map_or(detection.measurement_time_ns, |time: i64| {
                    time.max(detection.measurement_time_ns)
                }),
            );
            if age > config.history_duration_ns {
                diagnostics.discarded_detections += 1;
                false
            } else {
                if age > 0 {
                    diagnostics.replayed_detections += 1;
                }
                true
            }
        });
        detections.sort_by_key(|detection| {
            (
                detection.measurement_time_ns,
                detection.delivery_index,
                detection.id().to_owned(),
            )
        });
    } else {
        for detection in &mut detections {
            detection.measurement_time_ns = detection.receipt_time_ns;
        }
    }

    let mut filters = BTreeMap::<String, Filter>::new();
    let mut tracks = Vec::new();
    let mut processed_detection_ids = Vec::new();
    for detection in detections {
        let Some(ego) = ego_history.sample(detection.measurement_time_ns) else {
            diagnostics.missing_ego_pose += 1;
            continue;
        };
        let key = detection.association_key().to_owned();
        if !filters.contains_key(&key) {
            let Detection::Lidar(lidar_detection) = &detection.detection else {
                diagnostics.waiting_for_range += 1;
                continue;
            };
            filters.insert(
                key.clone(),
                initialize(
                    lidar_detection,
                    ego,
                    &lidar.mount,
                    detection.measurement_time_ns,
                ),
            );
            diagnostics.applied_updates += 1;
        } else {
            let filter = filters.get_mut(&key).expect("checked filter presence");
            propagate(
                filter,
                detection.measurement_time_ns,
                config.acceleration_noise_stddev_mps2,
            )?;
            let result = match &detection.detection {
                Detection::Camera(value) => {
                    update_camera(filter, value, ego, &camera.mount, config.gate_sigma)
                }
                Detection::Lidar(value) => {
                    update_lidar(filter, value, ego, &lidar.mount, config.gate_sigma)
                }
            };
            match result {
                TrackUpdate::Applied => diagnostics.applied_updates += 1,
                TrackUpdate::Rejected => diagnostics.rejected_updates += 1,
                TrackUpdate::Invalid => diagnostics.invalid_updates += 1,
            }
        }
        processed_detection_ids.push(detection.id().to_owned());
        let filter = filters.get(&key).expect("initialized or updated filter");
        tracks.push(to_track(
            filter,
            config,
            &key,
            detection.receipt_time_ns,
            ego_source,
            world_frame,
        ));
    }

    Ok(TrackerRun {
        tracks,
        diagnostics,
        processed_detection_ids,
    })
}

fn flatten(measurements: &[PerceptionMeasurement]) -> Vec<TimedDetection> {
    let mut detections = Vec::new();
    for measurement in measurements {
        let header = measurement.header();
        match measurement {
            PerceptionMeasurement::Camera(frame) => {
                for detection in &frame.detections {
                    detections.push(TimedDetection {
                        measurement_time_ns: header.reported_stamp_ns,
                        receipt_time_ns: header.receipt_time_ns,
                        delivery_index: header.delivery_index,
                        detection: Detection::Camera(detection.clone()),
                    });
                }
            }
            PerceptionMeasurement::Lidar(scan) => {
                let scan_start = header.reported_stamp_ns - header.acquisition_duration_ns;
                for detection in &scan.detections {
                    detections.push(TimedDetection {
                        measurement_time_ns: scan_start + detection.acquisition_offset_ns,
                        receipt_time_ns: header.receipt_time_ns,
                        delivery_index: header.delivery_index,
                        detection: Detection::Lidar(detection.clone()),
                    });
                }
            }
        }
    }
    detections
}

fn validate_delivery_order(measurements: &[PerceptionMeasurement]) -> Result<()> {
    let mut previous = None;
    for measurement in measurements {
        let delivery = measurement.header().delivery_index;
        if let Some(previous) = previous {
            ensure!(
                delivery > previous,
                "perception delivery indices are not increasing"
            );
        }
        previous = Some(delivery);
    }
    Ok(())
}

fn sensor_pose(ego: EgoPose, mount: &SensorMountConfig) -> (Vector2<f64>, f64) {
    let offset = Vector2::new(
        ego.yaw.cos() * mount.position_m.x - ego.yaw.sin() * mount.position_m.y,
        ego.yaw.sin() * mount.position_m.x + ego.yaw.cos() * mount.position_m.y,
    );
    (
        ego.position + offset,
        math::wrap_angle(ego.yaw + mount.yaw_rad),
    )
}

fn initialize(
    detection: &LidarDetection,
    ego: EgoPose,
    mount: &SensorMountConfig,
    time_ns: i64,
) -> Filter {
    let (sensor_position, sensor_yaw) = sensor_pose(ego, mount);
    let horizontal_range = detection.range_m * detection.elevation_rad.cos();
    let bearing_world = sensor_yaw + detection.azimuth_rad;
    let position =
        sensor_position + Vector2::new(bearing_world.cos(), bearing_world.sin()) * horizontal_range;
    let range_variance = detection
        .spherical_covariance
        .first()
        .copied()
        .unwrap_or(0.25);
    let bearing_variance = detection
        .spherical_covariance
        .get(4)
        .copied()
        .unwrap_or(0.01);
    let tangential_variance = horizontal_range.powi(2) * bearing_variance;
    let ego_position_variance = ego.covariance_xy_yaw[(0, 0)].max(ego.covariance_xy_yaw[(1, 1)]);
    let ego_yaw_variance = ego.covariance_xy_yaw[(2, 2)] * horizontal_range.powi(2);
    let position_variance =
        range_variance + tangential_variance + ego_position_variance + ego_yaw_variance;
    Filter {
        state: TrackState::new(position.x, position.y, 0.0, 0.0),
        covariance: TrackCovariance::from_diagonal(&TrackState::new(
            position_variance,
            position_variance,
            4.0,
            4.0,
        )),
        time_ns,
    }
}

fn propagate(filter: &mut Filter, time_ns: i64, acceleration_noise_stddev_mps2: f64) -> Result<()> {
    let dt = (time_ns - filter.time_ns) as f64 * 1.0e-9;
    ensure!(dt >= 0.0, "tracker measurements are not time ordered");
    if dt == 0.0 {
        return Ok(());
    }
    let transition = TrackCovariance::new(
        1.0, 0.0, dt, 0.0, 0.0, 1.0, 0.0, dt, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    );
    let q = acceleration_noise_stddev_mps2.powi(2);
    let dt2 = dt * dt;
    let dt3 = dt2 * dt;
    let dt4 = dt2 * dt2;
    let process_noise = TrackCovariance::new(
        0.25 * dt4 * q,
        0.0,
        0.5 * dt3 * q,
        0.0,
        0.0,
        0.25 * dt4 * q,
        0.0,
        0.5 * dt3 * q,
        0.5 * dt3 * q,
        0.0,
        dt2 * q,
        0.0,
        0.0,
        0.5 * dt3 * q,
        0.0,
        dt2 * q,
    );
    filter.state = transition * filter.state;
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

fn update_camera(
    filter: &mut Filter,
    detection: &CameraDetection,
    ego: EgoPose,
    mount: &SensorMountConfig,
    gate_sigma: f64,
) -> TrackUpdate {
    let (sensor_position, sensor_yaw) = sensor_pose(ego, mount);
    let displacement = Vector2::new(filter.state[0], filter.state[1]) - sensor_position;
    let range_squared = displacement.norm_squared();
    if range_squared <= 1.0e-12 {
        return TrackUpdate::Invalid;
    }
    let predicted = math::wrap_angle(displacement.y.atan2(displacement.x) - sensor_yaw);
    let mut jacobian = TrackState::zeros();
    jacobian[0] = -displacement.y / range_squared;
    jacobian[1] = displacement.x / range_squared;
    let base_variance = detection
        .angular_covariance
        .first()
        .copied()
        .unwrap_or(0.01);
    let ego_jacobian = SVector::<f64, 3>::new(
        displacement.y / range_squared,
        -displacement.x / range_squared,
        -1.0,
    );
    let ego_variance = (ego_jacobian.transpose() * ego.covariance_xy_yaw * ego_jacobian)[0];
    apply_track_scalar(
        filter,
        math::wrap_angle(detection.azimuth_rad - predicted),
        jacobian,
        base_variance + ego_variance.max(0.0),
        gate_sigma,
    )
}

fn update_lidar(
    filter: &mut Filter,
    detection: &LidarDetection,
    ego: EgoPose,
    mount: &SensorMountConfig,
    gate_sigma: f64,
) -> TrackUpdate {
    let before = filter.clone();
    let (sensor_position, _) = sensor_pose(ego, mount);
    let displacement = Vector2::new(filter.state[0], filter.state[1]) - sensor_position;
    let range_squared = displacement.norm_squared();
    let range = range_squared.sqrt();
    if range <= 1.0e-9 {
        return TrackUpdate::Invalid;
    }
    let mut range_jacobian = TrackState::zeros();
    range_jacobian[0] = displacement.x / range;
    range_jacobian[1] = displacement.y / range;
    let ego_range_jacobian =
        SVector::<f64, 3>::new(-displacement.x / range, -displacement.y / range, 0.0);
    let ego_range_variance =
        (ego_range_jacobian.transpose() * ego.covariance_xy_yaw * ego_range_jacobian)[0];
    let range_result = apply_track_scalar(
        filter,
        detection.range_m * detection.elevation_rad.cos() - range,
        range_jacobian,
        detection
            .spherical_covariance
            .first()
            .copied()
            .unwrap_or(0.25)
            + ego_range_variance.max(0.0),
        gate_sigma,
    );
    if range_result != TrackUpdate::Applied {
        return range_result;
    }
    let bearing_result = update_camera(
        filter,
        &CameraDetection {
            detection_id: detection.detection_id.clone(),
            association_key: detection.association_key.clone(),
            azimuth_rad: detection.azimuth_rad,
            elevation_rad: detection.elevation_rad,
            angular_covariance: vec![
                detection
                    .spherical_covariance
                    .get(4)
                    .copied()
                    .unwrap_or(0.01),
                0.0,
                0.0,
                0.0,
            ],
        },
        EgoPose {
            yaw: math::wrap_angle(ego.yaw),
            ..ego
        },
        &SensorMountConfig {
            yaw_rad: mount.yaw_rad,
            ..mount.clone()
        },
        gate_sigma,
    );
    if bearing_result != TrackUpdate::Applied {
        *filter = before;
    }
    bearing_result
}

fn apply_track_scalar(
    filter: &mut Filter,
    residual: f64,
    jacobian: TrackState,
    measurement_variance: f64,
    gate_sigma: f64,
) -> TrackUpdate {
    if !residual.is_finite() || !measurement_variance.is_finite() || measurement_variance < 0.0 {
        return TrackUpdate::Invalid;
    }
    let innovation_variance =
        (jacobian.transpose() * filter.covariance * jacobian)[0] + measurement_variance;
    if !innovation_variance.is_finite() || innovation_variance <= 1.0e-15 {
        return TrackUpdate::Invalid;
    }
    if residual.abs() / innovation_variance.sqrt() > gate_sigma {
        return TrackUpdate::Rejected;
    }
    let gain = filter.covariance * jacobian / innovation_variance;
    let state = filter.state + gain * residual;
    let left = TrackCovariance::identity() - gain * jacobian.transpose();
    let covariance = left * filter.covariance * left.transpose()
        + gain * measurement_variance * gain.transpose();
    let covariance = 0.5 * (covariance + covariance.transpose());
    if !state.iter().all(|value| value.is_finite())
        || !covariance.iter().all(|value| value.is_finite())
        || covariance.clone_owned().cholesky().is_none()
    {
        return TrackUpdate::Invalid;
    }
    filter.state = state;
    filter.covariance = covariance;
    TrackUpdate::Applied
}

fn to_track(
    filter: &Filter,
    config: &ObjectTrackerConfig,
    association_key: &str,
    emission_time_ns: i64,
    ego_source: EgoSource,
    world_frame: &str,
) -> ObjectTrack {
    ObjectTrack {
        tracker_id: config.id.clone(),
        track_id: format!("{}:{association_key}", config.id),
        association_key: association_key.to_owned(),
        estimate_time_ns: filter.time_ns,
        emission_time_ns,
        position_world_m: Some(Vec3 {
            x: filter.state[0],
            y: filter.state[1],
            z: 0.0,
        }),
        velocity_world_mps: Some(Vec3 {
            x: filter.state[2],
            y: filter.state[3],
            z: 0.0,
        }),
        status: EstimateStatus::Valid as i32,
        covariance_kind: CovarianceKind::Full as i32,
        covariance: (0..4)
            .flat_map(|row| (0..4).map(move |column| filter.covariance[(row, column)]))
            .collect(),
        revision: 0,
        state_model: ObjectStateModel::PlanarConstantVelocity as i32,
        ego_source: ego_source as i32,
        frame_id: world_frame.to_owned(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scenario::Vec3Config;

    fn mount() -> SensorMountConfig {
        SensorMountConfig {
            frame: "lidar".to_owned(),
            position_m: Vec3Config::default(),
            roll_rad: 0.0,
            pitch_rad: 0.0,
            yaw_rad: 0.0,
        }
    }

    fn detection() -> LidarDetection {
        LidarDetection {
            detection_id: "d".to_owned(),
            association_key: "a".to_owned(),
            range_m: 20.0,
            azimuth_rad: 0.0,
            elevation_rad: 0.0,
            acquisition_offset_ns: 0,
            spherical_covariance: vec![0.01, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001],
        }
    }

    #[test]
    fn ego_heading_error_moves_a_distant_object_sideways() {
        let correct = initialize(
            &detection(),
            EgoPose {
                time_ns: 0,
                position: Vector2::zeros(),
                yaw: 0.0,
                covariance_xy_yaw: SMatrix::zeros(),
            },
            &mount(),
            0,
        );
        let wrong = initialize(
            &detection(),
            EgoPose {
                time_ns: 0,
                position: Vector2::zeros(),
                yaw: 1_f64.to_radians(),
                covariance_xy_yaw: SMatrix::zeros(),
            },
            &mount(),
            0,
        );
        assert!((wrong.state[1] - correct.state[1] - 0.349).abs() < 0.002);
    }

    #[test]
    fn lidar_update_is_atomic_when_bearing_is_rejected() {
        let ego = EgoPose {
            time_ns: 0,
            position: Vector2::zeros(),
            yaw: 0.0,
            covariance_xy_yaw: SMatrix::zeros(),
        };
        let mut filter = initialize(&detection(), ego, &mount(), 0);
        let original_state = filter.state;
        let original_covariance = filter.covariance;
        let mut outlier = detection();
        outlier.azimuth_rad = std::f64::consts::FRAC_PI_2;

        assert_eq!(
            update_lidar(&mut filter, &outlier, ego, &mount(), 3.0),
            TrackUpdate::Rejected
        );
        assert_eq!(filter.state, original_state);
        assert_eq!(filter.covariance, original_covariance);
    }
}
