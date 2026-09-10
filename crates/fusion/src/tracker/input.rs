use anyhow::{Result, ensure};
use fusion_schema::messages::{
    CameraDetection, CameraFrame, EgoStateEstimate, EgoTruthState, LidarDetection, LidarScan,
    MeasurementTime,
};
use nalgebra::{SMatrix, Vector2};

use crate::{
    math,
    scenario::{CameraConfig, LidarConfig, ObjectTrackerConfig},
};

#[derive(Debug, Clone)]
pub enum PerceptionMeasurement {
    Camera(CameraFrame),
    Lidar(LidarScan),
}

impl PerceptionMeasurement {
    pub fn time(&self) -> Result<&MeasurementTime> {
        let time = match self {
            Self::Camera(value) => value.time.as_ref(),
            Self::Lidar(value) => value.time.as_ref(),
        };
        time.ok_or_else(|| anyhow::anyhow!("perception measurement has no time"))
    }
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
pub(super) struct EgoPose {
    pub(super) time_ns: i64,
    pub(super) position_world_m: Vector2<f64>,
    pub(super) yaw_world_from_body_rad: f64,
    /// Row and column order: world x position, world y position, yaw.
    pub(super) pose_covariance_world: SMatrix<f64, 3, 3>,
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
            let position_world_m = pose
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
            let pose_covariance_world = SMatrix::from_fn(|row, column| {
                estimate.state_covariance[indices[row] * dimension + indices[column]]
            });
            samples.push(EgoPose {
                time_ns: estimate.estimate_time_ns,
                position_world_m: Vector2::new(position_world_m.x, position_world_m.y),
                yaw_world_from_body_rad: pose.yaw_rad,
                pose_covariance_world,
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
            let position_world_m = pose
                .position
                .as_ref()
                .ok_or_else(|| anyhow::anyhow!("ego truth pose has no position"))?;
            samples.push(EgoPose {
                time_ns: state.time_ns,
                position_world_m: Vector2::new(position_world_m.x, position_world_m.y),
                yaw_world_from_body_rad: pose.yaw_rad,
                pose_covariance_world: SMatrix::zeros(),
            });
        }
        Ok(Self { samples })
    }

    pub(super) fn sample(&self, time_ns: i64) -> Option<EgoPose> {
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
                    position_world_m: previous.position_world_m
                        + (next.position_world_m - previous.position_world_m) * fraction,
                    yaw_world_from_body_rad: math::wrap_angle(
                        previous.yaw_world_from_body_rad
                            + math::wrap_angle(
                                next.yaw_world_from_body_rad - previous.yaw_world_from_body_rad,
                            ) * fraction,
                    ),
                    pose_covariance_world: previous.pose_covariance_world
                        + (next.pose_covariance_world - previous.pose_covariance_world) * fraction,
                })
            }
        }
    }

    #[cfg(test)]
    pub(super) fn from_samples(samples: Vec<EgoPose>) -> Self {
        Self { samples }
    }
}

#[derive(Debug, Clone)]
pub(super) enum Detection {
    Camera(CameraDetection),
    Lidar(LidarDetection),
}

#[derive(Debug, Clone, Copy)]
pub(super) struct DetectionContext {
    pub(super) ego_pose: Option<EgoPose>,
}

#[derive(Debug, Clone, Copy)]
pub(super) struct DetectionScanContext {
    pub(super) sensor: super::SensorKind,
    pub(super) ego_pose: Option<EgoPose>,
    pub(super) horizontal_fov_rad: f64,
    pub(super) max_range_m: f64,
    pub(super) detection_probability: f64,
}

#[derive(Debug, Clone)]
pub(super) struct DetectionBatch {
    pub(super) sensor: super::SensorKind,
    pub(super) measurement_time_ns: i64,
    pub(super) arrival_time_ns: i64,
    pub(super) stable_id: String,
    pub(super) horizontal_fov_rad: f64,
    pub(super) max_range_m: f64,
    pub(super) detection_probability: f64,
    pub(super) detections: Vec<Detection>,
}

impl DetectionBatch {
    pub(super) fn detection_time_ns(&self, detection: &Detection) -> i64 {
        match detection {
            Detection::Camera(_) => self.measurement_time_ns,
            Detection::Lidar(value) => value.measurement_time_ns,
        }
    }
}

pub(super) fn prepare_timing(
    config: &ObjectTrackerConfig,
    batches: &mut Vec<DetectionBatch>,
    diagnostics: &mut super::TrackerDiagnostics,
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

pub(super) fn flatten(
    camera_config: &CameraConfig,
    lidar_config: &LidarConfig,
    measurements: &[PerceptionMeasurement],
) -> Result<Vec<DetectionBatch>> {
    measurements
        .iter()
        .enumerate()
        .map(|(record_index, measurement)| {
            let time = measurement.time()?;
            let batch = match measurement {
                PerceptionMeasurement::Camera(frame) => DetectionBatch {
                    sensor: super::SensorKind::Camera,
                    measurement_time_ns: time.measurement_time_ns,
                    arrival_time_ns: time.arrival_time_ns,
                    stable_id: format!("camera:{record_index}"),
                    horizontal_fov_rad: camera_config.horizontal_fov_rad,
                    max_range_m: camera_config.max_range_m,
                    detection_probability: camera_config.detection_probability,
                    detections: frame
                        .detections
                        .iter()
                        .cloned()
                        .map(Detection::Camera)
                        .collect(),
                },
                PerceptionMeasurement::Lidar(scan) => DetectionBatch {
                    sensor: super::SensorKind::Lidar,
                    measurement_time_ns: time.measurement_time_ns,
                    arrival_time_ns: time.arrival_time_ns,
                    stable_id: format!("lidar:{record_index}"),
                    horizontal_fov_rad: lidar_config.horizontal_fov_rad,
                    max_range_m: lidar_config.max_range_m,
                    detection_probability: lidar_config.detection_probability,
                    detections: scan
                        .detections
                        .iter()
                        .cloned()
                        .map(Detection::Lidar)
                        .collect(),
                },
            };
            Ok(batch)
        })
        .collect()
}

pub(super) fn validate_delivery_order(measurements: &[PerceptionMeasurement]) -> Result<()> {
    let mut previous = None;
    for measurement in measurements {
        let delivery_time_ns = measurement.time()?.arrival_time_ns;
        if let Some(previous) = previous {
            ensure!(
                delivery_time_ns >= previous,
                "perception measurements are not in arrival order"
            );
        }
        previous = Some(delivery_time_ns);
    }
    Ok(())
}
