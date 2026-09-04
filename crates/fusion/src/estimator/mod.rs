mod gps;
mod observation;
mod propagation;
pub mod state;

use anyhow::{Result, ensure};
use fusion_schema::messages::{EgoStateEstimate, GpsFix, ImuSample, MeasurementTime};
use serde::{Deserialize, Serialize};

use crate::{
    math,
    scenario::{EgoEstimatorConfig, GpsConfig, ImuConfig},
};

use self::{
    observation::UpdateResult,
    propagation::ImuProcessNoise,
    state::{PlanarState, StateCovariance},
};

#[derive(Debug, Clone, PartialEq)]
pub enum EgoMeasurement {
    Imu(ImuSample),
    Gps(GpsFix),
}

impl EgoMeasurement {
    pub fn time(&self) -> &MeasurementTime {
        match self {
            Self::Imu(value) => value.time.as_ref(),
            Self::Gps(value) => value.time.as_ref(),
        }
        .expect("generated ego measurements have time")
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct FilterDiagnostics {
    pub attempted_updates: usize,
    pub applied_updates: usize,
    pub rejected_updates: usize,
    pub invalid_updates: usize,
    pub maximum_absolute_normalized_residual: f64,
}

impl FilterDiagnostics {
    pub fn record(&mut self, result: UpdateResult) {
        self.attempted_updates += 1;
        match result {
            UpdateResult::Applied {
                normalized_residual,
            } => {
                self.applied_updates += 1;
                self.maximum_absolute_normalized_residual = self
                    .maximum_absolute_normalized_residual
                    .max(normalized_residual.abs());
            }
            UpdateResult::Rejected {
                normalized_residual,
            } => {
                self.rejected_updates += 1;
                self.maximum_absolute_normalized_residual = self
                    .maximum_absolute_normalized_residual
                    .max(normalized_residual.abs());
            }
            UpdateResult::Invalid => self.invalid_updates += 1,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimingDiagnostics {
    pub timing_compensation: bool,
    pub history_duration_ns: i64,
    pub received_measurements: usize,
    pub delayed_measurements: usize,
    pub replayed_measurements: usize,
    pub discarded_measurements: usize,
    pub revised_estimates: usize,
    pub maximum_delivery_age_ns: i64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BaselineAssumptions {
    pub state_order: Vec<String>,
    pub initial_covariance_diagonal: Vec<f64>,
    pub imu_process_noise: ImuProcessNoise,
    pub gps_gate_sigma: f64,
}

#[derive(Debug)]
pub struct EstimatorRun {
    pub estimates: Vec<EgoStateEstimate>,
    pub timing: TimingDiagnostics,
    pub diagnostics: FilterDiagnostics,
    pub assumptions: BaselineAssumptions,
}

pub fn run_baseline(
    config: &EgoEstimatorConfig,
    imu: &ImuConfig,
    gps_config: &GpsConfig,
    measurements: &[EgoMeasurement],
) -> Result<EstimatorRun> {
    validate_delivery_order(measurements)?;
    let initial_covariance = state::initial_covariance(config);
    let assumptions = BaselineAssumptions {
        state_order: state::STATE_NAMES[..if config.estimate_imu_bias { 6 } else { 4 }]
            .iter()
            .map(|name| (*name).to_owned())
            .collect(),
        initial_covariance_diagonal: (0..if config.estimate_imu_bias { 6 } else { 4 })
            .map(|index| initial_covariance[(index, index)])
            .collect(),
        imu_process_noise: ImuProcessNoise::from(imu),
        gps_gate_sigma: config.gps_gate_sigma,
    };
    if config.timing_compensation {
        run_at_measurement_time(config, gps_config, measurements, assumptions)
    } else {
        run_at_arrival(config, gps_config, measurements, assumptions)
    }
}

fn run_at_arrival(
    config: &EgoEstimatorConfig,
    gps_config: &GpsConfig,
    measurements: &[EgoMeasurement],
    assumptions: BaselineAssumptions,
) -> Result<EstimatorRun> {
    let mut filter = BaselineEkf::new(config);
    let mut diagnostics = FilterDiagnostics::default();
    let mut estimates = Vec::new();
    let mut latest_imu_stamp_ns = None;
    let mut delayed = 0;
    for measurement in measurements {
        let time = measurement.time();
        if latest_imu_stamp_ns.is_some_and(|latest| time.measurement_time_ns < latest) {
            delayed += 1;
        }
        match measurement {
            EgoMeasurement::Imu(imu) => {
                filter.propagate(imu, &assumptions.imu_process_noise)?;
                latest_imu_stamp_ns = Some(time.measurement_time_ns);
                estimates.push(filter.estimate(
                    config,
                    time.measurement_time_ns,
                    time.arrival_time_ns,
                ));
            }
            EgoMeasurement::Gps(fix) => gps::update(
                &mut filter.state,
                &mut filter.covariance,
                config,
                gps_config,
                fix,
                &mut diagnostics,
            )?,
        }
    }
    Ok(EstimatorRun {
        estimates,
        timing: timing(config, measurements, delayed, 0, 0, 0),
        diagnostics,
        assumptions,
    })
}

fn run_at_measurement_time(
    config: &EgoEstimatorConfig,
    gps_config: &GpsConfig,
    measurements: &[EgoMeasurement],
    assumptions: BaselineAssumptions,
) -> Result<EstimatorRun> {
    let mut accepted = Vec::new();
    let mut latest_imu_stamp_ns = None;
    let mut delayed = 0;
    let mut replayed = 0;
    let mut discarded = 0;
    for measurement in measurements {
        let time = measurement.time();
        let age = latest_imu_stamp_ns
            .map(|latest: i64| latest.saturating_sub(time.measurement_time_ns))
            .unwrap_or(0);
        if age > 0 {
            delayed += 1;
        }
        if matches!(measurement, EgoMeasurement::Gps(_)) && age > config.history_duration_ns {
            discarded += 1;
        } else {
            if age > 0 {
                replayed += 1;
            }
            accepted.push(measurement);
        }
        if matches!(measurement, EgoMeasurement::Imu(_)) {
            latest_imu_stamp_ns = Some(
                latest_imu_stamp_ns.map_or(time.measurement_time_ns, |time: i64| {
                    time.max(measurement.time().measurement_time_ns)
                }),
            );
        }
    }
    accepted.sort_by_key(|measurement| {
        (
            measurement.time().measurement_time_ns,
            measurement_priority(measurement),
            measurement.time().arrival_time_ns,
        )
    });

    let mut filter = BaselineEkf::new(config);
    let mut diagnostics = FilterDiagnostics::default();
    let mut estimates = Vec::new();
    let mut revised = 0;
    let mut index = 0;
    while index < accepted.len() {
        let stamp = accepted[index].time().measurement_time_ns;
        let end = index
            + accepted[index..]
                .partition_point(|measurement| measurement.time().measurement_time_ns == stamp);
        let mut emission = None;
        for measurement in &accepted[index..end] {
            match measurement {
                EgoMeasurement::Imu(imu) => {
                    filter.propagate(imu, &assumptions.imu_process_noise)?;
                    emission = Some(measurement.time().arrival_time_ns);
                }
                EgoMeasurement::Gps(fix) => gps::update(
                    &mut filter.state,
                    &mut filter.covariance,
                    config,
                    gps_config,
                    fix,
                    &mut diagnostics,
                )?,
            }
        }
        if let Some(initial_emission) = emission {
            let mut final_emission = initial_emission;
            let mut was_revised = false;
            for measurement in &accepted {
                let time = measurement.time();
                if matches!(measurement, EgoMeasurement::Gps(_))
                    && time.measurement_time_ns <= stamp
                    && time.arrival_time_ns > initial_emission
                {
                    final_emission = final_emission.max(time.arrival_time_ns);
                    was_revised = true;
                }
            }
            revised += usize::from(was_revised);
            estimates.push(filter.estimate(config, stamp, final_emission));
        }
        index = end;
    }
    Ok(EstimatorRun {
        estimates,
        timing: timing(config, measurements, delayed, replayed, discarded, revised),
        diagnostics,
        assumptions,
    })
}

fn timing(
    config: &EgoEstimatorConfig,
    measurements: &[EgoMeasurement],
    delayed_measurements: usize,
    replayed_measurements: usize,
    discarded_measurements: usize,
    revised_estimates: usize,
) -> TimingDiagnostics {
    TimingDiagnostics {
        timing_compensation: config.timing_compensation,
        history_duration_ns: config.history_duration_ns,
        received_measurements: measurements.len(),
        delayed_measurements,
        replayed_measurements,
        discarded_measurements,
        revised_estimates,
        maximum_delivery_age_ns: measurements
            .iter()
            .map(|measurement| {
                let time = measurement.time();
                time.arrival_time_ns
                    .saturating_sub(time.measurement_time_ns)
            })
            .max()
            .unwrap_or(0),
    }
}

fn measurement_priority(measurement: &EgoMeasurement) -> u8 {
    match measurement {
        EgoMeasurement::Imu(_) => 10,
        EgoMeasurement::Gps(_) => 20,
    }
}

fn validate_delivery_order(measurements: &[EgoMeasurement]) -> Result<()> {
    let mut previous = None;
    for measurement in measurements {
        let delivery = measurement.time().arrival_time_ns;
        if let Some(previous) = previous {
            ensure!(
                delivery >= previous,
                "ego measurements are not in arrival order"
            );
        }
        previous = Some(delivery);
    }
    Ok(())
}

struct BaselineEkf {
    state: PlanarState,
    covariance: StateCovariance,
    last_imu_stamp_ns: Option<i64>,
}

impl BaselineEkf {
    fn new(config: &EgoEstimatorConfig) -> Self {
        Self {
            state: PlanarState::default(),
            covariance: state::initial_covariance(config),
            last_imu_stamp_ns: None,
        }
    }

    fn propagate(&mut self, imu: &ImuSample, noise: &ImuProcessNoise) -> Result<()> {
        propagation::propagate_imu(
            &mut self.state,
            &mut self.covariance,
            &mut self.last_imu_stamp_ns,
            imu,
            noise,
        )
    }

    fn estimate(
        &self,
        config: &EgoEstimatorConfig,
        estimate_time_ns: i64,
        available_time_ns: i64,
    ) -> EgoStateEstimate {
        let yaw = self.state.yaw_world_from_body_rad;
        EgoStateEstimate {
            estimate_time_ns,
            available_time_ns,
            pose_world: Some(math::pose2(
                self.state.position_world_m.x,
                self.state.position_world_m.y,
                yaw,
            )),
            forward_speed_mps: self.state.forward_speed_mps,
            state_covariance: (0..if config.estimate_imu_bias { 6 } else { 4 })
                .flat_map(|row| {
                    (0..if config.estimate_imu_bias { 6 } else { 4 })
                        .map(move |column| self.covariance[(row, column)])
                })
                .collect(),
            gyro_bias_z_radps: config
                .estimate_imu_bias
                .then_some(self.state.gyro_bias_radps),
            accel_bias_x_mps2: config
                .estimate_imu_bias
                .then_some(self.state.accel_bias_mps2),
        }
    }
}
