use std::collections::BTreeMap;

use anyhow::{Result, ensure};
use fusion_schema::messages::{
    CameraDetection, CameraFrame, EgoStateEstimate, EgoTruthState, LidarDetection, LidarScan,
    MeasurementTime, ObjectTrack, Vec2,
};
use nalgebra::{SMatrix, SVector, Vector2};
use serde::{Deserialize, Serialize};

use crate::{math, scenario::ObjectTrackerConfig};

type TrackState = SVector<f64, 4>;
type TrackCovariance = SMatrix<f64, 4, 4>;

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

#[derive(Debug, Clone)]
struct TimedDetection {
    measurement_time_ns: i64,
    arrival_time_ns: i64,
    stable_id: String,
    detection: Detection,
}

impl TimedDetection {
    fn id(&self) -> &str {
        &self.stable_id
    }

    fn track_key(&self) -> &str {
        match &self.detection {
            Detection::Camera(value) => &value.track_key,
            Detection::Lidar(value) => &value.track_key,
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
    measurements: &[PerceptionMeasurement],
    ego_history: &EgoHistory,
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
                detection.arrival_time_ns,
                detection.id().to_owned(),
            )
        });
    } else {
        for detection in &mut detections {
            detection.measurement_time_ns = detection.arrival_time_ns;
        }
    }

    let mut filters = BTreeMap::<String, Filter>::new();
    let mut tracks = Vec::new();
    let mut processed_detections = Vec::new();
    for detection in detections {
        let Some(ego) = ego_history.sample(detection.measurement_time_ns) else {
            diagnostics.missing_ego_pose += 1;
            continue;
        };
        let key = detection.track_key().to_owned();
        if !filters.contains_key(&key) {
            let Detection::Lidar(lidar_detection) = &detection.detection else {
                diagnostics.waiting_for_range += 1;
                continue;
            };
            filters.insert(
                key.clone(),
                initialize(lidar_detection, ego, detection.measurement_time_ns),
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
                Detection::Camera(value) => update_camera(filter, value, ego, config.gate_sigma),
                Detection::Lidar(value) => update_lidar(filter, value, ego, config.gate_sigma),
            };
            match result {
                TrackUpdate::Applied => diagnostics.applied_updates += 1,
                TrackUpdate::Rejected => diagnostics.rejected_updates += 1,
                TrackUpdate::Invalid => diagnostics.invalid_updates += 1,
            }
        }
        processed_detections.push(detection.id().to_owned());
        let filter = filters.get(&key).expect("initialized or updated filter");
        tracks.push(to_track(filter, &key, detection.arrival_time_ns));
    }

    Ok(TrackerRun {
        tracks,
        diagnostics,
        processed_detections,
    })
}

fn flatten(measurements: &[PerceptionMeasurement]) -> Vec<TimedDetection> {
    let mut detections = Vec::new();
    for (record_index, measurement) in measurements.iter().enumerate() {
        let time = measurement.time();
        match measurement {
            PerceptionMeasurement::Camera(frame) => {
                for (detection_index, detection) in frame.detections.iter().enumerate() {
                    detections.push(TimedDetection {
                        measurement_time_ns: time.measurement_time_ns,
                        arrival_time_ns: time.arrival_time_ns,
                        stable_id: format!("camera:{record_index}:{detection_index}"),
                        detection: Detection::Camera(detection.clone()),
                    });
                }
            }
            PerceptionMeasurement::Lidar(scan) => {
                for (detection_index, detection) in scan.detections.iter().enumerate() {
                    detections.push(TimedDetection {
                        measurement_time_ns: detection.measurement_time_ns,
                        arrival_time_ns: time.arrival_time_ns,
                        stable_id: format!("lidar:{record_index}:{detection_index}"),
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
    gate_sigma: f64,
) -> TrackUpdate {
    let displacement = Vector2::new(filter.state[0], filter.state[1]) - ego.position;
    let range_squared = displacement.norm_squared();
    if range_squared <= 1.0e-12 {
        return TrackUpdate::Invalid;
    }
    let predicted = math::wrap_angle(displacement.y.atan2(displacement.x) - ego.yaw);
    let mut jacobian = TrackState::zeros();
    jacobian[0] = -displacement.y / range_squared;
    jacobian[1] = displacement.x / range_squared;
    let ego_jacobian = SVector::<f64, 3>::new(
        displacement.y / range_squared,
        -displacement.x / range_squared,
        -1.0,
    );
    let ego_variance = (ego_jacobian.transpose() * ego.covariance_xy_yaw * ego_jacobian)[0];
    apply_track_scalar(
        filter,
        math::wrap_angle(detection.bearing_rad - predicted),
        jacobian,
        detection.bearing_variance_rad2 + ego_variance.max(0.0),
        gate_sigma,
    )
}

fn update_lidar(
    filter: &mut Filter,
    detection: &LidarDetection,
    ego: EgoPose,
    gate_sigma: f64,
) -> TrackUpdate {
    let before = filter.clone();
    let displacement = Vector2::new(filter.state[0], filter.state[1]) - ego.position;
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
        detection.range_m - range,
        range_jacobian,
        detection.range_variance_m2 + ego_range_variance.max(0.0),
        gate_sigma,
    );
    if range_result != TrackUpdate::Applied {
        return range_result;
    }
    let bearing_result = update_camera(
        filter,
        &CameraDetection {
            track_key: detection.track_key.clone(),
            bearing_rad: detection.bearing_rad,
            bearing_variance_rad2: detection.bearing_variance_rad2,
        },
        EgoPose {
            yaw: math::wrap_angle(ego.yaw),
            ..ego
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

fn to_track(filter: &Filter, track_key: &str, available_time_ns: i64) -> ObjectTrack {
    ObjectTrack {
        track_key: track_key.to_owned(),
        estimate_time_ns: filter.time_ns,
        available_time_ns,
        position_world_m: Some(Vec2 {
            x: filter.state[0],
            y: filter.state[1],
        }),
        velocity_world_mps: Some(Vec2 {
            x: filter.state[2],
            y: filter.state[3],
        }),
        state_covariance: (0..4)
            .flat_map(|row| (0..4).map(move |column| filter.covariance[(row, column)]))
            .collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn detection() -> LidarDetection {
        LidarDetection {
            track_key: "a".to_owned(),
            measurement_time_ns: 0,
            range_m: 20.0,
            bearing_rad: 0.0,
            range_variance_m2: 0.01,
            bearing_variance_rad2: 0.0001,
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
        let mut filter = initialize(&detection(), ego, 0);
        let original_state = filter.state;
        let original_covariance = filter.covariance;
        let mut outlier = detection();
        outlier.bearing_rad = std::f64::consts::FRAC_PI_2;

        assert_eq!(
            update_lidar(&mut filter, &outlier, ego, 3.0),
            TrackUpdate::Rejected
        );
        assert_eq!(filter.state, original_state);
        assert_eq!(filter.covariance, original_covariance);
    }
}
