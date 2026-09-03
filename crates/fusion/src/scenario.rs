use std::{collections::BTreeSet, fs, path::Path};

use anyhow::{Context, Result, bail, ensure};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ResolvedScenario {
    pub format_version: u32,
    pub run_id: String,
    pub duration_s: f64,
    #[serde(default = "default_motion_speed_factor")]
    pub motion_speed_factor: f64,
    pub local_epoch: String,
    pub root_seed: u64,
    pub platform_profile: String,
    pub platform: Platform,
    pub world: WorldConfig,
    pub trajectory: Vec<MotionSegment>,
    pub imu: ImuConfig,
    pub camera: CameraConfig,
    pub lidar: LidarConfig,
    pub estimator: EstimatorConfig,
    pub metrics: MetricsConfig,
}

impl ResolvedScenario {
    pub fn effective_duration_s(&self) -> f64 {
        self.duration_s / self.motion_speed_factor
    }
}

fn default_motion_speed_factor() -> f64 {
    1.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Platform {
    pub body_frame: String,
    pub world_frame: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WorldConfig {
    pub landmarks: Vec<LandmarkConfig>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LandmarkConfig {
    pub id: String,
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotionSegment {
    pub id: String,
    pub duration_s: f64,
    pub longitudinal_acceleration_mps2: f64,
    pub yaw_rate_radps: f64,
}

#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Vec3Config {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImuConfig {
    pub instance_id: String,
    pub frame: String,
    pub rate_hz: f64,
    pub clock_offset_ns: i64,
    pub latency_ns: i64,
    pub gyro_white_noise_density_radps_sqrt_hz: f64,
    pub accel_white_noise_density_mps2_sqrt_hz: f64,
    pub gyro_turn_on_bias_radps: Vec3Config,
    pub accel_turn_on_bias_mps2: Vec3Config,
    pub gyro_bias_random_walk_radps_sqrt_s: f64,
    pub accel_bias_random_walk_mps2_sqrt_s: f64,
    pub gyro_saturation_radps: f64,
    pub accel_saturation_mps2: f64,
    pub quantization_step: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CameraConfig {
    pub instance_id: String,
    pub frame: String,
    pub rate_hz: f64,
    pub latency_ns: i64,
    pub horizontal_fov_rad: f64,
    pub max_range_m: f64,
    pub bearing_noise_stddev_rad: f64,
    pub detection_probability: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LidarConfig {
    pub instance_id: String,
    pub frame: String,
    pub rate_hz: f64,
    pub latency_ns: i64,
    pub horizontal_fov_rad: f64,
    pub max_range_m: f64,
    pub scan_duration_ns: i64,
    pub range_noise_stddev_m: f64,
    pub bearing_noise_stddev_rad: f64,
    pub detection_probability: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EstimatorConfig {
    pub id: String,
    pub output_world_frame: String,
    pub output_body_frame: String,
    pub timing_compensation: bool,
    pub history_duration_ns: i64,
    pub camera_bearing_stddev_rad: f64,
    pub lidar_range_stddev_m: f64,
    pub lidar_bearing_stddev_rad: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MetricsConfig {
    pub alignment: String,
    pub divergence_position_error_m: f64,
}

pub fn load_and_resolve(path: &Path) -> Result<ResolvedScenario> {
    let source = fs::read_to_string(path)
        .with_context(|| format!("failed to read scenario {}", path.display()))?;
    let scenario: ResolvedScenario = serde_yaml_ng::from_str(&source)
        .with_context(|| format!("invalid scenario {}", path.display()))?;
    validate(&scenario)?;
    Ok(scenario)
}

pub fn validate(s: &ResolvedScenario) -> Result<()> {
    ensure!(
        s.format_version == 1,
        "unsupported format_version {}; expected 1",
        s.format_version
    );
    ensure!(!s.run_id.trim().is_empty(), "run_id must not be empty");
    ensure!(
        s.platform_profile == "planar_sensor_fusion",
        "only planar_sensor_fusion is implemented"
    );
    ensure!(
        s.duration_s.is_finite() && s.duration_s > 0.0,
        "duration_s must be positive and finite"
    );
    ensure!(
        s.motion_speed_factor.is_finite() && s.motion_speed_factor > 0.0,
        "motion_speed_factor must be positive and finite"
    );
    ensure!(
        s.platform.body_frame != s.platform.world_frame,
        "body and world frames must differ"
    );
    let sensor_frames = [
        ("imu", s.imu.frame.as_str()),
        ("camera", s.camera.frame.as_str()),
        ("lidar", s.lidar.frame.as_str()),
    ];
    for (name, frame) in sensor_frames {
        ensure!(
            frame == s.platform.body_frame,
            "the planar simulator requires the {name} frame to equal the body frame"
        );
    }
    let sensor_ids = [
        s.imu.instance_id.as_str(),
        s.camera.instance_id.as_str(),
        s.lidar.instance_id.as_str(),
    ];
    let unique_sensor_ids: BTreeSet<_> = sensor_ids.iter().copied().collect();
    ensure!(
        unique_sensor_ids.len() == sensor_ids.len()
            && unique_sensor_ids.iter().all(|id| !id.is_empty()),
        "sensor instance IDs must be nonempty and unique"
    );
    validate_rate("imu.rate_hz", s.imu.rate_hz)?;
    validate_rate("camera.rate_hz", s.camera.rate_hz)?;
    validate_rate("lidar.rate_hz", s.lidar.rate_hz)?;
    ensure!(
        s.imu.latency_ns >= 0 && s.camera.latency_ns >= 0 && s.lidar.latency_ns >= 0,
        "sensor latencies must be nonnegative"
    );
    validate_imu(&s.imu)?;
    validate_detection_sensor(
        "camera",
        s.camera.horizontal_fov_rad,
        s.camera.max_range_m,
        s.camera.detection_probability,
    )?;
    validate_detection_sensor(
        "lidar",
        s.lidar.horizontal_fov_rad,
        s.lidar.max_range_m,
        s.lidar.detection_probability,
    )?;
    ensure!(
        s.lidar.scan_duration_ns >= 0,
        "lidar scan duration must be nonnegative"
    );
    for (name, value) in [
        ("camera bearing noise", s.camera.bearing_noise_stddev_rad),
        ("lidar range noise", s.lidar.range_noise_stddev_m),
        ("lidar bearing noise", s.lidar.bearing_noise_stddev_rad),
    ] {
        ensure!(
            value.is_finite() && value >= 0.0,
            "{name} must be finite and nonnegative"
        );
    }
    validate_world(s)?;
    validate_estimator(s)?;
    ensure!(
        s.metrics.alignment == "NONE",
        "the initial example implements only NONE alignment"
    );
    ensure!(
        s.metrics.divergence_position_error_m > 0.0,
        "divergence threshold must be positive"
    );
    validate_trajectory(s)?;
    Ok(())
}

fn validate_imu(imu: &ImuConfig) -> Result<()> {
    for (name, value) in [
        (
            "gyro noise density",
            imu.gyro_white_noise_density_radps_sqrt_hz,
        ),
        (
            "accel noise density",
            imu.accel_white_noise_density_mps2_sqrt_hz,
        ),
        (
            "gyro bias random walk",
            imu.gyro_bias_random_walk_radps_sqrt_s,
        ),
        (
            "accel bias random walk",
            imu.accel_bias_random_walk_mps2_sqrt_s,
        ),
        ("quantization step", imu.quantization_step),
    ] {
        ensure!(
            value.is_finite() && value >= 0.0,
            "{name} must be finite and nonnegative"
        );
    }
    ensure!(
        imu.gyro_saturation_radps > 0.0 && imu.accel_saturation_mps2 > 0.0,
        "IMU saturation limits must be positive"
    );
    Ok(())
}

fn validate_detection_sensor(
    name: &str,
    horizontal_fov_rad: f64,
    max_range_m: f64,
    detection_probability: f64,
) -> Result<()> {
    ensure!(
        horizontal_fov_rad > 0.0 && horizontal_fov_rad <= 2.0 * std::f64::consts::PI,
        "{name} horizontal FOV must be in (0, 2pi]"
    );
    ensure!(
        max_range_m > 0.0 && max_range_m.is_finite(),
        "{name} max range must be positive"
    );
    ensure!(
        (0.0..=1.0).contains(&detection_probability),
        "{name} detection probability must be in [0, 1]"
    );
    Ok(())
}

fn validate_world(s: &ResolvedScenario) -> Result<()> {
    ensure!(
        !s.world.landmarks.is_empty(),
        "world must contain at least one landmark"
    );
    let mut ids = BTreeSet::new();
    for landmark in &s.world.landmarks {
        ensure!(
            !landmark.id.is_empty() && ids.insert(&landmark.id),
            "landmark IDs must be nonempty and unique"
        );
        ensure!(
            landmark.x_m.is_finite() && landmark.y_m.is_finite() && landmark.z_m.is_finite(),
            "landmark {} coordinates must be finite",
            landmark.id
        );
    }
    Ok(())
}

fn validate_estimator(s: &ResolvedScenario) -> Result<()> {
    ensure!(
        s.estimator.output_world_frame == s.platform.world_frame
            && s.estimator.output_body_frame == s.platform.body_frame,
        "estimator output frames must match the platform frames"
    );
    ensure!(
        s.estimator.history_duration_ns >= 0,
        "estimator history duration must be nonnegative"
    );
    if s.estimator.timing_compensation {
        ensure!(
            s.estimator.history_duration_ns > 0,
            "timing compensation requires a positive estimator history duration"
        );
    }
    for (name, value) in [
        (
            "estimator camera bearing standard deviation",
            s.estimator.camera_bearing_stddev_rad,
        ),
        (
            "estimator lidar range standard deviation",
            s.estimator.lidar_range_stddev_m,
        ),
        (
            "estimator lidar bearing standard deviation",
            s.estimator.lidar_bearing_stddev_rad,
        ),
    ] {
        ensure!(
            value > 0.0 && value.is_finite(),
            "{name} must be positive and finite"
        );
    }
    Ok(())
}

fn validate_trajectory(s: &ResolvedScenario) -> Result<()> {
    ensure!(
        !s.trajectory.is_empty(),
        "trajectory must contain at least one segment"
    );
    let mut ids = BTreeSet::new();
    let mut duration = 0.0;
    let mut speed = 0.0;
    for segment in &s.trajectory {
        ensure!(
            !segment.id.is_empty() && ids.insert(&segment.id),
            "trajectory segment IDs must be nonempty and unique"
        );
        ensure!(
            segment.duration_s.is_finite() && segment.duration_s > 0.0,
            "segment {} duration must be positive",
            segment.id
        );
        ensure!(
            segment.longitudinal_acceleration_mps2.is_finite()
                && segment.yaw_rate_radps.is_finite(),
            "segment {} motion values must be finite",
            segment.id
        );
        duration += segment.duration_s;
        speed += segment.longitudinal_acceleration_mps2 * segment.duration_s;
        ensure!(
            speed >= -1.0e-9,
            "segment {} produces a negative forward speed",
            segment.id
        );
    }
    if (duration - s.duration_s).abs() > 1.0e-9 {
        bail!(
            "duration_s ({}) does not match trajectory duration ({duration})",
            s.duration_s
        );
    }
    Ok(())
}

fn validate_rate(name: &str, value: f64) -> Result<()> {
    ensure!(
        value.is_finite() && value > 0.0,
        "{name} must be positive and finite"
    );
    let period_ns = 1.0e9 / value;
    ensure!(
        (period_ns - period_ns.round()).abs() < 1.0e-6,
        "{name} must map to an integer nanosecond period"
    );
    Ok(())
}

pub fn canonical_yaml(scenario: &ResolvedScenario) -> Result<String> {
    Ok(serde_yaml_ng::to_string(scenario)?)
}

pub fn sha256(bytes: &[u8]) -> String {
    hex::encode(Sha256::digest(bytes))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unknown_fields_are_rejected() {
        let yaml = "format_version: 1\nunknown: true\n";
        assert!(serde_yaml_ng::from_str::<ResolvedScenario>(yaml).is_err());
    }
}
