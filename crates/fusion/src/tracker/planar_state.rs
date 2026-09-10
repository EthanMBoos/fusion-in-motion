use anyhow::{Result, ensure};
use nalgebra::{SMatrix, SVector, Vector2};

use super::TrackSnapshot;

pub(super) type TrackStateVector = SVector<f64, 4>;
pub(super) type TrackStateCovariance = SMatrix<f64, 4, 4>;

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct TrackState {
    pub(super) position_world_m: Vector2<f64>,
    pub(super) velocity_world_mps: Vector2<f64>,
}

impl TrackState {
    pub(super) fn new(position_world_m: Vector2<f64>, velocity_world_mps: Vector2<f64>) -> Self {
        Self {
            position_world_m,
            velocity_world_mps,
        }
    }

    pub(super) fn with_correction(mut self, state_correction: TrackStateVector) -> Self {
        self.position_world_m.x += state_correction[TrackCoordinate::PositionX.index()];
        self.position_world_m.y += state_correction[TrackCoordinate::PositionY.index()];
        self.velocity_world_mps.x += state_correction[TrackCoordinate::VelocityX.index()];
        self.velocity_world_mps.y += state_correction[TrackCoordinate::VelocityY.index()];
        self
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(usize)]
pub(super) enum TrackCoordinate {
    PositionX,
    PositionY,
    VelocityX,
    VelocityY,
}

impl TrackCoordinate {
    pub(super) const fn index(self) -> usize {
        self as usize
    }
}

#[derive(Debug, Clone)]
pub(super) struct PlanarTrackFilter {
    pub(super) state: TrackState,
    pub(super) state_covariance: TrackStateCovariance,
    pub(super) time_ns: i64,
}

pub(super) fn propagate(
    filter: &mut PlanarTrackFilter,
    target_time_ns: i64,
    acceleration_noise_stddev_mps2: f64,
) -> Result<()> {
    let dt_s = (target_time_ns - filter.time_ns) as f64 * 1.0e-9;
    ensure!(dt_s >= 0.0, "tracker measurements are not time ordered");
    if dt_s == 0.0 {
        return Ok(());
    }
    let mut state_transition_f = TrackStateCovariance::identity();
    state_transition_f[(
        TrackCoordinate::PositionX.index(),
        TrackCoordinate::VelocityX.index(),
    )] = dt_s;
    state_transition_f[(
        TrackCoordinate::PositionY.index(),
        TrackCoordinate::VelocityY.index(),
    )] = dt_s;
    let acceleration_variance_m2ps4 = acceleration_noise_stddev_mps2.powi(2);
    let dt_s_squared = dt_s * dt_s;
    let dt_s_cubed = dt_s_squared * dt_s;
    let dt_s_fourth = dt_s_squared * dt_s_squared;
    let mut process_noise_q = TrackStateCovariance::zeros();
    for (position_coordinate, velocity_coordinate) in [
        (TrackCoordinate::PositionX, TrackCoordinate::VelocityX),
        (TrackCoordinate::PositionY, TrackCoordinate::VelocityY),
    ] {
        process_noise_q[(position_coordinate.index(), position_coordinate.index())] =
            0.25 * dt_s_fourth * acceleration_variance_m2ps4;
        process_noise_q[(position_coordinate.index(), velocity_coordinate.index())] =
            0.5 * dt_s_cubed * acceleration_variance_m2ps4;
        process_noise_q[(velocity_coordinate.index(), position_coordinate.index())] =
            0.5 * dt_s_cubed * acceleration_variance_m2ps4;
        process_noise_q[(velocity_coordinate.index(), velocity_coordinate.index())] =
            dt_s_squared * acceleration_variance_m2ps4;
    }
    filter.state.position_world_m += filter.state.velocity_world_mps * dt_s;
    filter.state_covariance =
        state_transition_f * filter.state_covariance * state_transition_f.transpose()
            + process_noise_q;
    filter.time_ns = target_time_ns;
    Ok(())
}

pub(super) fn snapshot(filter: &PlanarTrackFilter) -> TrackSnapshot {
    TrackSnapshot {
        position_world_m: [
            filter.state.position_world_m.x,
            filter.state.position_world_m.y,
        ],
        velocity_world_mps: [
            filter.state.velocity_world_mps.x,
            filter.state.velocity_world_mps.y,
        ],
        state_covariance: (0..4)
            .flat_map(|row| (0..4).map(move |column| filter.state_covariance[(row, column)]))
            .collect(),
    }
}
