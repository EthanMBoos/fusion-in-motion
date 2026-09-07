#[cxx::bridge(namespace = "fusion")]
pub mod ffi {
    struct PlanarConfig {
        initial_position_variance_m2: f64,
        initial_yaw_variance_rad2: f64,
        initial_speed_variance_m2ps2: f64,
        initial_gyro_bias_variance_rad2ps2: f64,
        initial_accel_bias_variance_m2ps4: f64,
        gyro_white_noise_density_radps_sqrt_hz: f64,
        accel_white_noise_density_mps2_sqrt_hz: f64,
        gyro_bias_random_walk_radps_sqrt_s: f64,
        accel_bias_random_walk_mps2_sqrt_s: f64,
        gps_gate_sigma: f64,
    }

    #[derive(Debug, PartialEq)]
    enum GpsUpdate {
        Applied,
        Rejected,
        Invalid,
    }

    struct GpsUpdateResult {
        update: GpsUpdate,
        normalized_residual: f64,
    }

    struct PlanarEstimate {
        position_world_x_m: f64,
        position_world_y_m: f64,
        yaw_world_from_body_rad: f64,
        forward_speed_mps: f64,
        gyro_bias_z_radps: f64,
        accel_bias_x_mps2: f64,
        state_covariance: [f64; 36],
    }

    unsafe extern "C++" {
        include!("fusion-gtsam/gtsam_estimator.hpp");

        type GtsamEkfEstimator;

        fn new_gtsam_ekf_estimator(config: &PlanarConfig) -> Result<UniquePtr<GtsamEkfEstimator>>;
        fn process_imu(
            self: Pin<&mut GtsamEkfEstimator>,
            measurement_time_ns: i64,
            yaw_rate_radps: f64,
            forward_acceleration_mps2: f64,
        ) -> Result<()>;
        fn process_gps(
            self: Pin<&mut GtsamEkfEstimator>,
            position_world_x_m: f64,
            position_world_y_m: f64,
            horizontal_position_variance_m2: f64,
        ) -> Result<GpsUpdateResult>;
        fn estimate(self: &GtsamEkfEstimator) -> Result<PlanarEstimate>;
        fn gtsam_version() -> &'static CxxString;
    }
}

pub use ffi::{GpsUpdate, GpsUpdateResult, PlanarConfig, PlanarEstimate};

pub struct GtsamEkfEstimator {
    estimator: cxx::UniquePtr<ffi::GtsamEkfEstimator>,
}

impl GtsamEkfEstimator {
    pub fn new(config: &PlanarConfig) -> Result<Self, cxx::Exception> {
        Ok(Self {
            estimator: ffi::new_gtsam_ekf_estimator(config)?,
        })
    }

    pub fn version() -> String {
        ffi::gtsam_version().to_string_lossy().into_owned()
    }

    pub fn process_imu(
        &mut self,
        measurement_time_ns: i64,
        yaw_rate_radps: f64,
        forward_acceleration_mps2: f64,
    ) -> Result<(), cxx::Exception> {
        self.estimator.pin_mut().process_imu(
            measurement_time_ns,
            yaw_rate_radps,
            forward_acceleration_mps2,
        )
    }

    pub fn process_gps(
        &mut self,
        position_world_x_m: f64,
        position_world_y_m: f64,
        horizontal_position_variance_m2: f64,
    ) -> Result<GpsUpdateResult, cxx::Exception> {
        self.estimator.pin_mut().process_gps(
            position_world_x_m,
            position_world_y_m,
            horizontal_position_variance_m2,
        )
    }

    pub fn estimate(&self) -> Result<PlanarEstimate, cxx::Exception> {
        self.estimator.estimate()
    }
}
