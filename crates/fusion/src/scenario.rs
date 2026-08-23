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
    pub platform: PlatformConfig,
    pub world: WorldConfig,
    pub trajectory: Vec<MotionSegment>,
    pub imu: ImuConfig,
    pub gps: GpsConfig,
    pub camera: CameraConfig,
    pub lidar: LidarConfig,
    pub ego_estimator: EgoEstimatorConfig,
    pub object_tracker: ObjectTrackerConfig,
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
pub struct PlatformConfig {
    pub model: String,
    pub body_frame: String,
    pub world_frame: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WorldConfig {
    pub objects: Vec<ObjectConfig>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ObjectConfig {
    pub id: String,
    pub association_key: String,
    pub initial_position_m: Vec3Config,
    pub velocity_world_mps: Vec3Config,
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

impl Vec3Config {
    pub fn finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SensorMountConfig {
    pub frame: String,
    pub position_m: Vec3Config,
    pub roll_rad: f64,
    pub pitch_rad: f64,
    pub yaw_rad: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImuConfig {
    pub enabled: bool,
    pub instance_id: String,
    pub mount: SensorMountConfig,
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
pub struct GpsConfig {
    pub enabled: bool,
    pub instance_id: String,
    pub mount: SensorMountConfig,
    pub rate_hz: f64,
    pub latency_ns: i64,
    pub horizontal_position_stddev_m: f64,
    pub vertical_position_stddev_m: f64,
    pub altitude_valid: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CameraConfig {
    pub enabled: bool,
    pub instance_id: String,
    pub mount: SensorMountConfig,
    pub rate_hz: f64,
    pub latency_ns: i64,
    pub horizontal_fov_rad: f64,
    pub vertical_fov_rad: f64,
    pub max_range_m: f64,
    pub bearing_noise_stddev_rad: f64,
    pub detection_probability: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LidarConfig {
    pub enabled: bool,
    pub instance_id: String,
    pub mount: SensorMountConfig,
    pub rate_hz: f64,
    pub latency_ns: i64,
    pub horizontal_fov_rad: f64,
    pub vertical_fov_rad: f64,
    pub max_range_m: f64,
    pub scan_duration_ns: i64,
    pub range_noise_stddev_m: f64,
    pub bearing_noise_stddev_rad: f64,
    pub detection_probability: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EgoEstimatorConfig {
    pub id: String,
    pub output_world_frame: String,
    pub output_body_frame: String,
    pub timing_compensation: bool,
    pub history_duration_ns: i64,
    pub gps_gate_sigma: f64,
    pub initial_position_stddev_m: f64,
    pub initial_yaw_stddev_rad: f64,
    pub initial_speed_stddev_mps: f64,
    pub initial_gyro_bias_stddev_radps: f64,
    pub initial_accel_bias_stddev_mps2: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ObjectTrackerConfig {
    pub id: String,
    pub timing_compensation: bool,
    pub history_duration_ns: i64,
    pub acceleration_noise_stddev_mps2: f64,
    pub gate_sigma: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MetricsConfig {
    pub max_truth_match_gap_ns: i64,
    pub ego_divergence_position_error_m: f64,
    pub track_divergence_position_error_m: f64,
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
        "unsupported format_version {}",
        s.format_version
    );
    ensure!(!s.run_id.trim().is_empty(), "run_id must not be empty");
    ensure!(
        s.platform.model == "planar_ground",
        "only planar_ground is implemented"
    );
    ensure!(
        s.duration_s.is_finite() && s.duration_s > 0.0,
        "duration_s must be positive"
    );
    ensure!(
        s.motion_speed_factor.is_finite() && s.motion_speed_factor > 0.0,
        "motion_speed_factor must be positive"
    );
    ensure!(
        !s.platform.body_frame.is_empty()
            && !s.platform.world_frame.is_empty()
            && s.platform.body_frame != s.platform.world_frame,
        "body and world frames must be nonempty and different"
    );

    let sensors = [
        (
            s.imu.enabled,
            "imu",
            &s.imu.instance_id,
            &s.imu.mount,
            s.imu.rate_hz,
            s.imu.latency_ns,
        ),
        (
            s.gps.enabled,
            "gps",
            &s.gps.instance_id,
            &s.gps.mount,
            s.gps.rate_hz,
            s.gps.latency_ns,
        ),
        (
            s.camera.enabled,
            "camera",
            &s.camera.instance_id,
            &s.camera.mount,
            s.camera.rate_hz,
            s.camera.latency_ns,
        ),
        (
            s.lidar.enabled,
            "lidar",
            &s.lidar.instance_id,
            &s.lidar.mount,
            s.lidar.rate_hz,
            s.lidar.latency_ns,
        ),
    ];
    let mut ids = BTreeSet::new();
    let mut frames = BTreeSet::new();
    for (enabled, name, id, mount, rate, latency) in sensors {
        ensure!(
            !id.is_empty() && ids.insert(id),
            "sensor instance IDs must be nonempty and unique"
        );
        ensure!(
            !mount.frame.is_empty() && frames.insert(&mount.frame),
            "sensor frames must be nonempty and unique"
        );
        validate_mount(name, mount)?;
        if enabled {
            validate_rate(&format!("{name}.rate_hz"), rate)?;
        }
        ensure!(latency >= 0, "{name}.latency_ns must be nonnegative");
    }
    ensure!(
        s.imu.enabled,
        "the planar baseline requires imu.enabled: true"
    );

    validate_nonnegative("GPS horizontal noise", s.gps.horizontal_position_stddev_m)?;
    validate_nonnegative("GPS vertical noise", s.gps.vertical_position_stddev_m)?;
    validate_detection_sensor(
        "camera",
        s.camera.horizontal_fov_rad,
        s.camera.vertical_fov_rad,
        s.camera.max_range_m,
        s.camera.detection_probability,
    )?;
    validate_detection_sensor(
        "lidar",
        s.lidar.horizontal_fov_rad,
        s.lidar.vertical_fov_rad,
        s.lidar.max_range_m,
        s.lidar.detection_probability,
    )?;
    ensure!(
        s.lidar.scan_duration_ns >= 0,
        "lidar scan duration must be nonnegative"
    );
    validate_nonnegative("camera bearing noise", s.camera.bearing_noise_stddev_rad)?;
    validate_nonnegative("lidar range noise", s.lidar.range_noise_stddev_m)?;
    validate_nonnegative("lidar bearing noise", s.lidar.bearing_noise_stddev_rad)?;
    validate_imu(&s.imu)?;
    validate_world(s)?;
    validate_trajectory(s)?;
    validate_estimators(s)?;
    ensure!(
        s.metrics.max_truth_match_gap_ns >= 0,
        "maximum truth match gap must be nonnegative"
    );
    ensure!(
        s.metrics.ego_divergence_position_error_m > 0.0,
        "ego divergence threshold must be positive"
    );
    ensure!(
        s.metrics.track_divergence_position_error_m > 0.0,
        "track divergence threshold must be positive"
    );
    Ok(())
}

fn validate_mount(name: &str, mount: &SensorMountConfig) -> Result<()> {
    ensure!(
        mount.position_m.finite(),
        "{name} mount position must be finite"
    );
    for value in [mount.roll_rad, mount.pitch_rad, mount.yaw_rad] {
        ensure!(value.is_finite(), "{name} mount angles must be finite");
    }
    ensure!(
        mount.position_m.z.abs() < 1.0e-12
            && mount.roll_rad.abs() < 1.0e-12
            && mount.pitch_rad.abs() < 1.0e-12,
        "planar_ground currently supports x/y/yaw sensor mounts only"
    );
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
        validate_nonnegative(name, value)?;
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
    vertical_fov_rad: f64,
    max_range_m: f64,
    detection_probability: f64,
) -> Result<()> {
    ensure!(
        horizontal_fov_rad > 0.0 && horizontal_fov_rad <= std::f64::consts::TAU,
        "{name} horizontal FOV must be in (0, 2pi]"
    );
    ensure!(
        vertical_fov_rad > 0.0 && vertical_fov_rad <= std::f64::consts::PI,
        "{name} vertical FOV must be in (0, pi]"
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
        !s.world.objects.is_empty(),
        "world must contain at least one object"
    );
    let mut ids = BTreeSet::new();
    let mut associations = BTreeSet::new();
    for object in &s.world.objects {
        ensure!(
            !object.id.is_empty() && ids.insert(&object.id),
            "object IDs must be nonempty and unique"
        );
        ensure!(
            !object.association_key.is_empty() && associations.insert(&object.association_key),
            "association keys must be nonempty and unique"
        );
        ensure!(
            object.initial_position_m.finite() && object.velocity_world_mps.finite(),
            "object {} state must be finite",
            object.id
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

fn validate_estimators(s: &ResolvedScenario) -> Result<()> {
    ensure!(
        s.ego_estimator.output_world_frame == s.platform.world_frame
            && s.ego_estimator.output_body_frame == s.platform.body_frame,
        "ego estimator output frames must match platform frames"
    );
    ensure!(
        !s.ego_estimator.id.is_empty() && !s.object_tracker.id.is_empty(),
        "estimator IDs must not be empty"
    );
    ensure!(
        s.ego_estimator.history_duration_ns >= 0 && s.object_tracker.history_duration_ns >= 0,
        "history durations must be nonnegative"
    );
    for (name, value) in [
        ("GPS gate", s.ego_estimator.gps_gate_sigma),
        (
            "initial position uncertainty",
            s.ego_estimator.initial_position_stddev_m,
        ),
        (
            "initial yaw uncertainty",
            s.ego_estimator.initial_yaw_stddev_rad,
        ),
        (
            "initial speed uncertainty",
            s.ego_estimator.initial_speed_stddev_mps,
        ),
        (
            "initial gyro bias uncertainty",
            s.ego_estimator.initial_gyro_bias_stddev_radps,
        ),
        (
            "initial accel bias uncertainty",
            s.ego_estimator.initial_accel_bias_stddev_mps2,
        ),
        (
            "tracker acceleration noise",
            s.object_tracker.acceleration_noise_stddev_mps2,
        ),
        ("tracker gate", s.object_tracker.gate_sigma),
    ] {
        ensure!(
            value.is_finite() && value > 0.0,
            "{name} must be positive and finite"
        );
    }
    Ok(())
}

fn validate_nonnegative(name: &str, value: f64) -> Result<()> {
    ensure!(
        value.is_finite() && value >= 0.0,
        "{name} must be finite and nonnegative"
    );
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
