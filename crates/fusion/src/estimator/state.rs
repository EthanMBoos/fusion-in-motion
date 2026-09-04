use nalgebra::{SMatrix, SVector, Vector2};

use crate::{math, scenario::EgoEstimatorConfig};

pub const STATE_DIMENSION: usize = 6;
pub const STATE_NAMES: [&str; STATE_DIMENSION] = [
    "position_world_x_m",
    "position_world_y_m",
    "yaw_world_from_body_rad",
    "forward_speed_mps",
    "gyro_bias_z_radps",
    "accel_bias_x_mps2",
];

pub type StateCorrection = SVector<f64, STATE_DIMENSION>;
pub type StateCovariance = SMatrix<f64, STATE_DIMENSION, STATE_DIMENSION>;

#[derive(Debug, Clone, Copy)]
#[repr(usize)]
pub enum StateIndex {
    PositionWorldX = 0,
    PositionWorldY = 1,
    YawWorldFromBody = 2,
    ForwardSpeed = 3,
    GyroBias = 4,
    AccelBias = 5,
}

impl StateIndex {
    pub const fn index(self) -> usize {
        self as usize
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct PlanarState {
    pub position_world_m: Vector2<f64>,
    pub yaw_world_from_body_rad: f64,
    pub forward_speed_mps: f64,
    pub gyro_bias_radps: f64,
    pub accel_bias_mps2: f64,
}

impl PlanarState {
    pub fn apply_correction(&mut self, correction: &StateCorrection) {
        self.position_world_m.x += correction[StateIndex::PositionWorldX.index()];
        self.position_world_m.y += correction[StateIndex::PositionWorldY.index()];
        self.yaw_world_from_body_rad = math::wrap_angle(
            self.yaw_world_from_body_rad + correction[StateIndex::YawWorldFromBody.index()],
        );
        self.forward_speed_mps += correction[StateIndex::ForwardSpeed.index()];
        self.gyro_bias_radps += correction[StateIndex::GyroBias.index()];
        self.accel_bias_mps2 += correction[StateIndex::AccelBias.index()];
    }
}

pub fn initial_covariance(config: &EgoEstimatorConfig) -> StateCovariance {
    let stddevs = [
        config.initial_position_stddev_m,
        config.initial_position_stddev_m,
        config.initial_yaw_stddev_rad,
        config.initial_speed_stddev_mps,
        if config.estimate_imu_bias {
            config.initial_gyro_bias_stddev_radps
        } else {
            1.0e-9
        },
        if config.estimate_imu_bias {
            config.initial_accel_bias_stddev_mps2
        } else {
            1.0e-9
        },
    ];
    StateCovariance::from_diagonal(&SVector::from_fn(|index, _| stddevs[index].powi(2)))
}

pub fn set_variance(covariance: &mut StateCovariance, state: StateIndex, variance: f64) {
    covariance[(state.index(), state.index())] = variance;
}
