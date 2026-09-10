use std::error::Error;

use fusion_schema::messages::{CameraDetection, LidarDetection};
use fusion_tracking::{
    DetectionOpportunity, GateDecision, HypothesisModel, HypothesisOutcome, ObservationHypothesis,
    TimedObservation,
};
use nalgebra::{SMatrix, SVector, Vector2};

use crate::math;

use super::{
    input::{Detection, DetectionContext, DetectionScanContext, EgoPose},
    planar_state::{
        PlanarTrackFilter, TrackCoordinate, TrackState, TrackStateCovariance, TrackStateVector,
        propagate,
    },
};

type LidarMeasurementJacobian = SMatrix<f64, 2, 4>;
type LidarMeasurementCovariance = SMatrix<f64, 2, 2>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum UpdateResult {
    Applied,
    Rejected,
    Invalid,
}

struct CameraInnovation {
    residual_bearing_rad: f64,
    measurement_jacobian_h: TrackStateVector,
    measurement_variance_rad2: f64,
}

struct LidarInnovation {
    residual_range_m: f64,
    residual_bearing_rad: f64,
    measurement_jacobian_h: LidarMeasurementJacobian,
    measurement_covariance_r: LidarMeasurementCovariance,
}

impl LidarInnovation {
    fn residual_vector(&self) -> Vector2<f64> {
        Vector2::new(self.residual_range_m, self.residual_bearing_rad)
    }
}

#[derive(Debug, Clone, Copy)]
pub(super) struct PlanarModel {
    pub(super) gate_sigma: f64,
    pub(super) acceleration_noise_stddev_mps2: f64,
}

#[derive(Debug, Clone)]
pub(super) struct PlanarModelError(String);

impl std::fmt::Display for PlanarModelError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(formatter)
    }
}

impl Error for PlanarModelError {}

impl HypothesisModel<PlanarTrackFilter, Detection, DetectionContext, DetectionScanContext>
    for PlanarModel
{
    type Error = PlanarModelError;

    fn predict(
        &self,
        state: &PlanarTrackFilter,
        time_ns: i64,
    ) -> Result<PlanarTrackFilter, Self::Error> {
        let mut predicted = state.clone();
        propagate(&mut predicted, time_ns, self.acceleration_noise_stddev_mps2)
            .map_err(|error| PlanarModelError(error.to_string()))?;
        Ok(predicted)
    }

    fn hypothesize(
        &self,
        predicted: &PlanarTrackFilter,
        observation: &TimedObservation<Detection, DetectionContext>,
    ) -> Result<ObservationHypothesis<PlanarTrackFilter>, Self::Error> {
        let Some(ego_pose) = observation.context.ego_pose else {
            return Ok(ObservationHypothesis {
                normalized_innovation_squared: None,
                log_likelihood: None,
                outcome: HypothesisOutcome::Invalid,
            });
        };
        let normalized_innovation_squared =
            normalized_innovation_squared(predicted, &observation.payload, ego_pose);
        let gate = match normalized_innovation_squared {
            Some(value) if value >= 0.0 && value.sqrt() <= self.gate_sigma => GateDecision::Inside,
            Some(value) if value >= 0.0 => GateDecision::Outside,
            _ => GateDecision::Invalid,
        };
        let outcome = match gate {
            GateDecision::Inside => {
                let mut posterior = predicted.clone();
                let update_result = match &observation.payload {
                    Detection::Camera(value) => {
                        update_camera(&mut posterior, value, ego_pose, self.gate_sigma)
                    }
                    Detection::Lidar(value) => {
                        update_lidar(&mut posterior, value, ego_pose, self.gate_sigma)
                    }
                };
                match update_result {
                    UpdateResult::Applied => HypothesisOutcome::Inside { posterior },
                    UpdateResult::Rejected => HypothesisOutcome::Outside,
                    UpdateResult::Invalid => HypothesisOutcome::Invalid,
                }
            }
            GateDecision::Outside => HypothesisOutcome::Outside,
            GateDecision::Invalid => HypothesisOutcome::Invalid,
        };
        Ok(ObservationHypothesis {
            normalized_innovation_squared,
            // NIS alone omits the innovation-covariance normalization, so do
            // not mislabel the hard-assignment score as a log likelihood.
            log_likelihood: None,
            outcome,
        })
    }

    fn detection_opportunity(
        &self,
        predicted: &PlanarTrackFilter,
        _measurement_time_ns: i64,
        context: &DetectionScanContext,
    ) -> Result<DetectionOpportunity, Self::Error> {
        let Some(ego_pose) = context.ego_pose else {
            return Ok(DetectionOpportunity::NotObservable);
        };
        let displacement_world_m = predicted.state.position_world_m - ego_pose.position_world_m;
        let range_m = displacement_world_m.norm();
        let bearing_body_rad = math::wrap_angle(
            displacement_world_m.y.atan2(displacement_world_m.x) - ego_pose.yaw_world_from_body_rad,
        );
        let inside_field_of_view = bearing_body_rad.abs() <= context.horizontal_fov_rad / 2.0;
        if range_m <= context.max_range_m && inside_field_of_view {
            Ok(DetectionOpportunity::Observable {
                detection_probability: context.detection_probability,
            })
        } else {
            Ok(DetectionOpportunity::NotObservable)
        }
    }
}

pub(super) fn initialize(
    detection: &LidarDetection,
    ego_pose: EgoPose,
    time_ns: i64,
) -> PlanarTrackFilter {
    let bearing_world_rad = ego_pose.yaw_world_from_body_rad + detection.bearing_rad;
    let position_world_m = ego_pose.position_world_m
        + Vector2::new(bearing_world_rad.cos(), bearing_world_rad.sin()) * detection.range_m;
    let tangential_variance_m2 = detection.range_m.powi(2) * detection.bearing_variance_rad2;
    let ego_position_variance_m2 =
        ego_pose.pose_covariance_world[(0, 0)].max(ego_pose.pose_covariance_world[(1, 1)]);
    let ego_yaw_position_variance_m2 =
        ego_pose.pose_covariance_world[(2, 2)] * detection.range_m.powi(2);
    let position_variance_m2 = detection.range_variance_m2
        + tangential_variance_m2
        + ego_position_variance_m2
        + ego_yaw_position_variance_m2;
    let position_x = TrackCoordinate::PositionX.index();
    let position_y = TrackCoordinate::PositionY.index();
    let velocity_x = TrackCoordinate::VelocityX.index();
    let velocity_y = TrackCoordinate::VelocityY.index();
    PlanarTrackFilter {
        state: TrackState::new(position_world_m, Vector2::zeros()),
        state_covariance: {
            let mut state_covariance = TrackStateCovariance::zeros();
            state_covariance[(position_x, position_x)] = position_variance_m2;
            state_covariance[(position_y, position_y)] = position_variance_m2;
            state_covariance[(velocity_x, velocity_x)] = 4.0;
            state_covariance[(velocity_y, velocity_y)] = 4.0;
            state_covariance
        },
        time_ns,
    }
}

fn normalized_innovation_squared(
    filter: &PlanarTrackFilter,
    detection: &Detection,
    ego_pose: EgoPose,
) -> Option<f64> {
    match detection {
        Detection::Camera(value) => {
            let innovation = camera_innovation(filter, value, ego_pose)?;
            let innovation_variance_s_rad2 = innovation_variance(
                filter,
                innovation.measurement_jacobian_h,
                innovation.measurement_variance_rad2,
            )?;
            Some(innovation.residual_bearing_rad.powi(2) / innovation_variance_s_rad2)
        }
        Detection::Lidar(value) => {
            let innovation = lidar_innovation(filter, value, ego_pose)?;
            let residual_range_bearing = innovation.residual_vector();
            let innovation_covariance_s = innovation.measurement_jacobian_h
                * filter.state_covariance
                * innovation.measurement_jacobian_h.transpose()
                + innovation.measurement_covariance_r;
            let innovation_covariance_inverse = innovation_covariance_s.try_inverse()?;
            let normalized_innovation_squared = (residual_range_bearing.transpose()
                * innovation_covariance_inverse
                * residual_range_bearing)[0];
            normalized_innovation_squared
                .is_finite()
                .then_some(normalized_innovation_squared)
        }
    }
}

fn camera_innovation(
    filter: &PlanarTrackFilter,
    detection: &CameraDetection,
    ego_pose: EgoPose,
) -> Option<CameraInnovation> {
    let displacement_world_m = filter.state.position_world_m - ego_pose.position_world_m;
    let range_squared_m2 = displacement_world_m.norm_squared();
    if range_squared_m2 <= 1.0e-12 {
        return None;
    }
    let predicted_bearing_rad = math::wrap_angle(
        displacement_world_m.y.atan2(displacement_world_m.x) - ego_pose.yaw_world_from_body_rad,
    );
    let mut measurement_jacobian_h = TrackStateVector::zeros();
    measurement_jacobian_h[TrackCoordinate::PositionX.index()] =
        -displacement_world_m.y / range_squared_m2;
    measurement_jacobian_h[TrackCoordinate::PositionY.index()] =
        displacement_world_m.x / range_squared_m2;
    let ego_pose_jacobian_h = SVector::<f64, 3>::new(
        displacement_world_m.y / range_squared_m2,
        -displacement_world_m.x / range_squared_m2,
        -1.0,
    );
    let ego_bearing_variance_rad2 =
        (ego_pose_jacobian_h.transpose() * ego_pose.pose_covariance_world * ego_pose_jacobian_h)[0];
    Some(CameraInnovation {
        residual_bearing_rad: math::wrap_angle(detection.bearing_rad - predicted_bearing_rad),
        measurement_jacobian_h,
        measurement_variance_rad2: detection.bearing_variance_rad2
            + ego_bearing_variance_rad2.max(0.0),
    })
}

fn lidar_innovation(
    filter: &PlanarTrackFilter,
    detection: &LidarDetection,
    ego_pose: EgoPose,
) -> Option<LidarInnovation> {
    let displacement_world_m = filter.state.position_world_m - ego_pose.position_world_m;
    let range_squared_m2 = displacement_world_m.norm_squared();
    let predicted_range_m = range_squared_m2.sqrt();
    if predicted_range_m <= 1.0e-9 {
        return None;
    }
    let predicted_bearing_rad = math::wrap_angle(
        displacement_world_m.y.atan2(displacement_world_m.x) - ego_pose.yaw_world_from_body_rad,
    );
    let mut measurement_jacobian_h = LidarMeasurementJacobian::zeros();
    measurement_jacobian_h[(0, TrackCoordinate::PositionX.index())] =
        displacement_world_m.x / predicted_range_m;
    measurement_jacobian_h[(0, TrackCoordinate::PositionY.index())] =
        displacement_world_m.y / predicted_range_m;
    measurement_jacobian_h[(1, TrackCoordinate::PositionX.index())] =
        -displacement_world_m.y / range_squared_m2;
    measurement_jacobian_h[(1, TrackCoordinate::PositionY.index())] =
        displacement_world_m.x / range_squared_m2;
    let ego_pose_jacobian_h = SMatrix::<f64, 2, 3>::from_row_slice(&[
        -displacement_world_m.x / predicted_range_m,
        -displacement_world_m.y / predicted_range_m,
        0.0,
        displacement_world_m.y / range_squared_m2,
        -displacement_world_m.x / range_squared_m2,
        -1.0,
    ]);
    let sensor_covariance_r = LidarMeasurementCovariance::from_diagonal(&Vector2::new(
        detection.range_variance_m2,
        detection.bearing_variance_rad2,
    ));
    let measurement_covariance_r = sensor_covariance_r
        + ego_pose_jacobian_h * ego_pose.pose_covariance_world * ego_pose_jacobian_h.transpose();
    Some(LidarInnovation {
        residual_range_m: detection.range_m - predicted_range_m,
        residual_bearing_rad: math::wrap_angle(detection.bearing_rad - predicted_bearing_rad),
        measurement_jacobian_h,
        measurement_covariance_r,
    })
}

fn innovation_variance(
    filter: &PlanarTrackFilter,
    measurement_jacobian_h: TrackStateVector,
    measurement_variance_rad2: f64,
) -> Option<f64> {
    if !measurement_variance_rad2.is_finite() || measurement_variance_rad2 < 0.0 {
        return None;
    }
    let innovation_variance_s_rad2 =
        (measurement_jacobian_h.transpose() * filter.state_covariance * measurement_jacobian_h)[0]
            + measurement_variance_rad2;
    (innovation_variance_s_rad2.is_finite() && innovation_variance_s_rad2 > 1.0e-15)
        .then_some(innovation_variance_s_rad2)
}

pub(super) fn update_camera(
    filter: &mut PlanarTrackFilter,
    detection: &CameraDetection,
    ego_pose: EgoPose,
    gate_sigma: f64,
) -> UpdateResult {
    let Some(innovation) = camera_innovation(filter, detection, ego_pose) else {
        return UpdateResult::Invalid;
    };
    apply_camera_innovation(filter, innovation, gate_sigma)
}

pub(super) fn update_lidar(
    filter: &mut PlanarTrackFilter,
    detection: &LidarDetection,
    ego_pose: EgoPose,
    gate_sigma: f64,
) -> UpdateResult {
    let Some(innovation) = lidar_innovation(filter, detection, ego_pose) else {
        return UpdateResult::Invalid;
    };
    let residual_range_bearing = innovation.residual_vector();
    if !residual_range_bearing.iter().all(|value| value.is_finite())
        || !innovation
            .measurement_covariance_r
            .iter()
            .all(|value| value.is_finite())
    {
        return UpdateResult::Invalid;
    }
    let innovation_covariance_s = innovation.measurement_jacobian_h
        * filter.state_covariance
        * innovation.measurement_jacobian_h.transpose()
        + innovation.measurement_covariance_r;
    let Some(innovation_covariance_inverse) = innovation_covariance_s.try_inverse() else {
        return UpdateResult::Invalid;
    };
    let normalized_innovation_squared = (residual_range_bearing.transpose()
        * innovation_covariance_inverse
        * residual_range_bearing)[0];
    if !normalized_innovation_squared.is_finite() {
        return UpdateResult::Invalid;
    }
    if normalized_innovation_squared.sqrt() > gate_sigma {
        return UpdateResult::Rejected;
    }
    let kalman_gain_k = filter.state_covariance
        * innovation.measurement_jacobian_h.transpose()
        * innovation_covariance_inverse;
    let corrected_state = filter
        .state
        .with_correction(kalman_gain_k * residual_range_bearing);
    let covariance_update_factor =
        TrackStateCovariance::identity() - kalman_gain_k * innovation.measurement_jacobian_h;
    let corrected_state_covariance =
        covariance_update_factor * filter.state_covariance * covariance_update_factor.transpose()
            + kalman_gain_k * innovation.measurement_covariance_r * kalman_gain_k.transpose();
    commit_update(filter, corrected_state, corrected_state_covariance)
}

fn apply_camera_innovation(
    filter: &mut PlanarTrackFilter,
    innovation: CameraInnovation,
    gate_sigma: f64,
) -> UpdateResult {
    if !innovation.residual_bearing_rad.is_finite() {
        return UpdateResult::Invalid;
    }
    let Some(innovation_variance_s_rad2) = innovation_variance(
        filter,
        innovation.measurement_jacobian_h,
        innovation.measurement_variance_rad2,
    ) else {
        return UpdateResult::Invalid;
    };
    if innovation.residual_bearing_rad.abs() / innovation_variance_s_rad2.sqrt() > gate_sigma {
        return UpdateResult::Rejected;
    }
    let kalman_gain_k =
        filter.state_covariance * innovation.measurement_jacobian_h / innovation_variance_s_rad2;
    let corrected_state = filter
        .state
        .with_correction(kalman_gain_k * innovation.residual_bearing_rad);
    let covariance_update_factor = TrackStateCovariance::identity()
        - kalman_gain_k * innovation.measurement_jacobian_h.transpose();
    let corrected_state_covariance =
        covariance_update_factor * filter.state_covariance * covariance_update_factor.transpose()
            + kalman_gain_k * innovation.measurement_variance_rad2 * kalman_gain_k.transpose();
    commit_update(filter, corrected_state, corrected_state_covariance)
}

fn commit_update(
    filter: &mut PlanarTrackFilter,
    corrected_state: TrackState,
    corrected_state_covariance: TrackStateCovariance,
) -> UpdateResult {
    let corrected_state_covariance =
        0.5 * (corrected_state_covariance + corrected_state_covariance.transpose());
    if !corrected_state
        .position_world_m
        .iter()
        .all(|value| value.is_finite())
        || !corrected_state
            .velocity_world_mps
            .iter()
            .all(|value| value.is_finite())
        || !corrected_state_covariance
            .iter()
            .all(|value| value.is_finite())
        || corrected_state_covariance.cholesky().is_none()
    {
        return UpdateResult::Invalid;
    }
    filter.state = corrected_state;
    filter.state_covariance = corrected_state_covariance;
    UpdateResult::Applied
}
