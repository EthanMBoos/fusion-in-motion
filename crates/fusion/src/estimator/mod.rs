mod camera;
mod lidar;
mod observation;
mod propagation;
mod state;

use std::collections::BTreeMap;

use anyhow::{Result, ensure};
use fusion_schema::messages::{CovarianceKind, EstimateStatus, LandmarkMap, StateEstimate, Vec3};
use nalgebra::Vector2;
use serde::{Deserialize, Serialize};

use crate::{
    bundle::MeasurementRecord,
    math,
    scenario::{EstimatorConfig, ImuConfig},
};

use self::observation::ScalarUpdateResult;
pub use self::propagation::ImuProcessNoise;
use self::state::{PlanarState, StateCovariance};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimingDiagnostics {
    pub timing_compensation: bool,
    pub history_duration_ns: i64,
    pub received_measurements: usize,
    pub delayed_measurements: usize,
    pub replayed_measurements: usize,
    pub discarded_measurements: usize,
    pub revised_estimates: usize,
    pub deskewed_lidar_scans: usize,
    pub deskewed_lidar_returns: usize,
    pub maximum_delivery_age_ns: i64,
}

#[derive(Debug)]
pub struct EstimatorRun {
    pub estimates: Vec<StateEstimate>,
    pub timing: TimingDiagnostics,
    pub diagnostics: FilterDiagnostics,
    pub assumptions: BaselineAssumptions,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FilterDiagnostics {
    pub attempted_scalar_updates: usize,
    pub applied_scalar_updates: usize,
    pub invalid_scalar_updates: usize,
}

impl FilterDiagnostics {
    fn record(&mut self, result: ScalarUpdateResult) {
        self.attempted_scalar_updates += 1;
        match result {
            ScalarUpdateResult::Applied => self.applied_scalar_updates += 1,
            ScalarUpdateResult::Invalid => self.invalid_scalar_updates += 1,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BaselineAssumptions {
    pub state_order: Vec<String>,
    pub initial_covariance_diagonal: [f64; 6],
    pub initial_cross_covariances_zero: bool,
    pub imu_process_noise_source: String,
    pub imu_process_noise: ImuProcessNoise,
    pub uses_additional_process_noise: bool,
    pub camera_bearing_stddev_rad: f64,
    pub lidar_range_stddev_m: f64,
    pub lidar_bearing_stddev_rad: f64,
}

impl BaselineAssumptions {
    fn new(config: &EstimatorConfig, imu: &ImuConfig) -> Self {
        let initial_covariance = state::initial_covariance();
        Self {
            state_order: state::STATE_NAMES.map(str::to_owned).to_vec(),
            initial_covariance_diagonal: std::array::from_fn(|index| {
                initial_covariance[(index, index)]
            }),
            initial_cross_covariances_zero: true,
            imu_process_noise_source: "scenario.imu".to_owned(),
            imu_process_noise: ImuProcessNoise::from(imu),
            uses_additional_process_noise: false,
            camera_bearing_stddev_rad: config.camera_bearing_stddev_rad,
            lidar_range_stddev_m: config.lidar_range_stddev_m,
            lidar_bearing_stddev_rad: config.lidar_bearing_stddev_rad,
        }
    }
}

pub fn run_baseline(
    config: &EstimatorConfig,
    imu: &ImuConfig,
    measurements: &[MeasurementRecord],
) -> Result<EstimatorRun> {
    validate_delivery_order(measurements)?;
    let assumptions = BaselineAssumptions::new(config, imu);
    if config.timing_compensation {
        run_timing_compensated(config, measurements, assumptions)
    } else {
        run_at_arrival(config, measurements, assumptions)
    }
}

fn run_at_arrival(
    config: &EstimatorConfig,
    measurements: &[MeasurementRecord],
    assumptions: BaselineAssumptions,
) -> Result<EstimatorRun> {
    let mut filter = BaselineEkf::new();
    let mut estimates = Vec::new();
    let mut timing = TimingDiagnostics::new(config, measurements);
    let mut diagnostics = FilterDiagnostics::default();
    let mut latest_imu_stamp_ns = None;

    for measurement in measurements {
        let header = measurement.header();
        if let Some(current_time_ns) = latest_imu_stamp_ns
            && header.reported_stamp_ns < current_time_ns
        {
            timing.delayed_measurements += 1;
        }

        match measurement {
            MeasurementRecord::Map(map) => filter.handle_map(map)?,
            MeasurementRecord::Imu(imu) => {
                propagation::propagate_imu(
                    &mut filter.state,
                    &mut filter.covariance,
                    &mut filter.last_imu_stamp_ns,
                    imu,
                    &assumptions.imu_process_noise,
                )?;
                latest_imu_stamp_ns = Some(header.reported_stamp_ns);
                estimates.push(filter.estimate(
                    &config.id,
                    header.reported_stamp_ns,
                    header.receipt_time_ns,
                    &config.output_world_frame,
                    &config.output_body_frame,
                ));
            }
            MeasurementRecord::Camera(frame) => camera::update(
                &mut filter.state,
                &mut filter.covariance,
                &filter.landmarks,
                config,
                frame,
                &mut diagnostics,
            )?,
            MeasurementRecord::Lidar(scan) => lidar::update(
                &mut filter.state,
                &mut filter.covariance,
                &filter.landmarks,
                config,
                scan,
                &mut diagnostics,
            )?,
        }
    }

    Ok(EstimatorRun {
        estimates,
        timing,
        diagnostics,
        assumptions,
    })
}

fn run_timing_compensated(
    config: &EstimatorConfig,
    measurements: &[MeasurementRecord],
    assumptions: BaselineAssumptions,
) -> Result<EstimatorRun> {
    let mut timing = TimingDiagnostics::new(config, measurements);
    let mut diagnostics = FilterDiagnostics::default();
    let mut accepted = Vec::with_capacity(measurements.len());
    let mut latest_imu_stamp_ns = None;

    for measurement in measurements {
        let header = measurement.header();
        let oldest_required_ns = match measurement {
            MeasurementRecord::Lidar(_) => {
                header.reported_stamp_ns - header.acquisition_duration_ns
            }
            _ => header.reported_stamp_ns,
        };
        let delayed_by_ns = latest_imu_stamp_ns
            .map(|time_ns: i64| time_ns.saturating_sub(header.reported_stamp_ns))
            .unwrap_or(0);
        if delayed_by_ns > 0 {
            timing.delayed_measurements += 1;
        }
        let required_history_ns = latest_imu_stamp_ns
            .map(|time_ns: i64| time_ns.saturating_sub(oldest_required_ns))
            .unwrap_or(0);
        let discard = !matches!(
            measurement,
            MeasurementRecord::Map(_) | MeasurementRecord::Imu(_)
        ) && required_history_ns > config.history_duration_ns;
        if discard {
            timing.discarded_measurements += 1;
        } else {
            if delayed_by_ns > 0 {
                timing.replayed_measurements += 1;
            }
            accepted.push(measurement);
        }
        if matches!(measurement, MeasurementRecord::Imu(_)) {
            latest_imu_stamp_ns = Some(
                latest_imu_stamp_ns.map_or(header.reported_stamp_ns, |current: i64| {
                    current.max(header.reported_stamp_ns)
                }),
            );
        }
    }

    accepted.sort_by_key(|measurement| {
        (
            measurement.header().reported_stamp_ns,
            measurement_priority(measurement),
            measurement.header().delivery_index,
        )
    });

    let mut filter = BaselineEkf::new();
    let mut state_history = Vec::new();
    let mut estimates = Vec::new();
    let mut index = 0;
    while index < accepted.len() {
        let time_ns = accepted[index].header().reported_stamp_ns;
        let end = accepted[index..]
            .partition_point(|measurement| measurement.header().reported_stamp_ns == time_ns)
            + index;
        let mut imu_receipt_time_ns = None;

        for measurement in &accepted[index..end] {
            let header = measurement.header();
            match measurement {
                MeasurementRecord::Map(map) => filter.handle_map(map)?,
                MeasurementRecord::Imu(imu) => {
                    propagation::propagate_imu(
                        &mut filter.state,
                        &mut filter.covariance,
                        &mut filter.last_imu_stamp_ns,
                        imu,
                        &assumptions.imu_process_noise,
                    )?;
                    imu_receipt_time_ns = Some(header.receipt_time_ns);
                }
                MeasurementRecord::Camera(frame) => camera::update(
                    &mut filter.state,
                    &mut filter.covariance,
                    &filter.landmarks,
                    config,
                    frame,
                    &mut diagnostics,
                )?,
                MeasurementRecord::Lidar(scan) => {
                    let state_at = |query_ns| {
                        interpolate_state(&state_history, time_ns, filter.state, query_ns)
                    };
                    let scan = lidar::deskew(scan, state_at)?;
                    if header.acquisition_duration_ns > 0 {
                        timing.deskewed_lidar_scans += 1;
                        timing.deskewed_lidar_returns += scan.returns.len();
                    }
                    lidar::update(
                        &mut filter.state,
                        &mut filter.covariance,
                        &filter.landmarks,
                        config,
                        &scan,
                        &mut diagnostics,
                    )?;
                }
            }
        }

        state_history.push(TimedState {
            time_ns,
            state: filter.state,
        });
        trim_state_history(
            &mut state_history,
            time_ns.saturating_sub(config.history_duration_ns),
        );
        if let Some(emission_time_ns) = imu_receipt_time_ns {
            estimates.push(filter.estimate(
                &config.id,
                time_ns,
                emission_time_ns,
                &config.output_world_frame,
                &config.output_body_frame,
            ));
        }
        index = end;
    }

    for estimate in &mut estimates {
        let initial_emission_ns = estimate.emission_time_ns;
        let mut revision = 0_u64;
        for measurement in &accepted {
            let header = measurement.header();
            if !matches!(measurement, MeasurementRecord::Map(_))
                && header.reported_stamp_ns <= estimate.estimate_time_ns
                && header.receipt_time_ns > initial_emission_ns
            {
                estimate.emission_time_ns = estimate.emission_time_ns.max(header.receipt_time_ns);
                revision += 1;
            }
        }
        estimate.revision = revision;
    }
    timing.revised_estimates = estimates
        .iter()
        .filter(|estimate| estimate.revision > 0)
        .count();

    Ok(EstimatorRun {
        estimates,
        timing,
        diagnostics,
        assumptions,
    })
}

impl TimingDiagnostics {
    fn new(config: &EstimatorConfig, measurements: &[MeasurementRecord]) -> Self {
        Self {
            timing_compensation: config.timing_compensation,
            history_duration_ns: config.history_duration_ns,
            received_measurements: measurements.len(),
            delayed_measurements: 0,
            replayed_measurements: 0,
            discarded_measurements: 0,
            revised_estimates: 0,
            deskewed_lidar_scans: 0,
            deskewed_lidar_returns: 0,
            maximum_delivery_age_ns: measurements
                .iter()
                .map(|measurement| {
                    let header = measurement.header();
                    header
                        .receipt_time_ns
                        .saturating_sub(header.reported_stamp_ns)
                })
                .max()
                .unwrap_or(0),
        }
    }
}

fn validate_delivery_order(measurements: &[MeasurementRecord]) -> Result<()> {
    let mut last_delivery_index = None;
    for measurement in measurements {
        let delivery_index = measurement.header().delivery_index;
        if let Some(previous_delivery_index) = last_delivery_index {
            ensure!(
                delivery_index > previous_delivery_index,
                "measurement delivery indices are not strictly increasing"
            );
        }
        last_delivery_index = Some(delivery_index);
    }
    Ok(())
}

fn measurement_priority(measurement: &MeasurementRecord) -> u8 {
    match measurement {
        MeasurementRecord::Map(_) => 0,
        MeasurementRecord::Imu(_) => 10,
        MeasurementRecord::Camera(_) => 20,
        MeasurementRecord::Lidar(_) => 30,
    }
}

#[derive(Debug, Clone, Copy)]
struct TimedState {
    time_ns: i64,
    state: PlanarState,
}

fn interpolate_state(
    history: &[TimedState],
    current_time_ns: i64,
    current_state: PlanarState,
    query_ns: i64,
) -> Option<PlanarState> {
    if query_ns == current_time_ns {
        return Some(current_state);
    }
    if query_ns > current_time_ns {
        return None;
    }
    let index = history.partition_point(|sample| sample.time_ns < query_ns);
    match (index.checked_sub(1), history.get(index)) {
        (None, Some(next)) => Some(next.state),
        (Some(previous), None) => {
            let previous = history[previous];
            (previous.time_ns == query_ns).then_some(previous.state)
        }
        (Some(previous), Some(next)) => {
            let previous = history[previous];
            if next.time_ns == query_ns {
                return Some(next.state);
            }
            let span_ns = next.time_ns - previous.time_ns;
            if span_ns <= 0 {
                return None;
            }
            let fraction = (query_ns - previous.time_ns) as f64 / span_ns as f64;
            Some(interpolate_planar_state(
                previous.state,
                next.state,
                fraction,
            ))
        }
        _ => None,
    }
}

fn interpolate_planar_state(start: PlanarState, end: PlanarState, fraction: f64) -> PlanarState {
    let fraction = fraction.clamp(0.0, 1.0);
    PlanarState {
        position_world_m: start.position_world_m
            + (end.position_world_m - start.position_world_m) * fraction,
        yaw_world_from_body_rad: math::wrap_angle(
            start.yaw_world_from_body_rad
                + math::wrap_angle(end.yaw_world_from_body_rad - start.yaw_world_from_body_rad)
                    * fraction,
        ),
        forward_speed_mps: start.forward_speed_mps
            + (end.forward_speed_mps - start.forward_speed_mps) * fraction,
        gyro_bias_radps: start.gyro_bias_radps
            + (end.gyro_bias_radps - start.gyro_bias_radps) * fraction,
        accel_bias_mps2: start.accel_bias_mps2
            + (end.accel_bias_mps2 - start.accel_bias_mps2) * fraction,
    }
}

fn trim_state_history(history: &mut Vec<TimedState>, cutoff_ns: i64) {
    let samples_through_cutoff = history.partition_point(|sample| sample.time_ns <= cutoff_ns);
    let first_to_keep = samples_through_cutoff.saturating_sub(1);
    if first_to_keep > 0 {
        history.drain(..first_to_keep);
    }
}

struct BaselineEkf {
    state: PlanarState,
    covariance: StateCovariance,
    landmarks: BTreeMap<String, Vector2<f64>>,
    last_imu_stamp_ns: Option<i64>,
}

impl BaselineEkf {
    fn new() -> Self {
        Self {
            state: PlanarState::default(),
            covariance: state::initial_covariance(),
            landmarks: BTreeMap::new(),
            last_imu_stamp_ns: None,
        }
    }

    fn handle_map(&mut self, map: &LandmarkMap) -> Result<()> {
        ensure!(!map.landmarks.is_empty(), "landmark map is empty");
        self.landmarks.clear();

        for landmark in &map.landmarks {
            let position_world_m = landmark
                .position_world_m
                .as_ref()
                .ok_or_else(|| anyhow::anyhow!("landmark {} has no position", landmark.id))?;
            ensure!(
                self.landmarks
                    .insert(
                        landmark.id.clone(),
                        Vector2::new(position_world_m.x, position_world_m.y),
                    )
                    .is_none(),
                "duplicate landmark {} in estimator map",
                landmark.id
            );
        }

        Ok(())
    }

    fn estimate(
        &self,
        estimator_id: &str,
        estimate_time_ns: i64,
        emission_time_ns: i64,
        world_frame: &str,
        body_frame: &str,
    ) -> StateEstimate {
        let yaw_rad = self.state.yaw_world_from_body_rad;
        StateEstimate {
            estimator_id: estimator_id.to_owned(),
            estimate_time_ns,
            emission_time_ns,
            pose_w_b: Some(math::yaw_pose(
                self.state.position_world_m.x,
                self.state.position_world_m.y,
                yaw_rad,
                world_frame,
                body_frame,
            )),
            velocity_world_mps: Some(Vec3 {
                x: self.state.forward_speed_mps * yaw_rad.cos(),
                y: self.state.forward_speed_mps * yaw_rad.sin(),
                z: 0.0,
            }),
            status: EstimateStatus::Valid as i32,
            covariance_kind: CovarianceKind::Full as i32,
            covariance: (0..state::STATE_DIMENSION)
                .flat_map(|row| {
                    (0..state::STATE_DIMENSION).map(move |column| self.covariance[(row, column)])
                })
                .collect(),
            revision: 0,
        }
    }
}
