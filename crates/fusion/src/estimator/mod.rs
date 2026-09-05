mod basic;
mod imu_bias;

use anyhow::{Result, ensure};
use fusion_schema::messages::{EgoStateEstimate, GpsFix, ImuSample, MeasurementTime};
use serde::{Deserialize, Serialize};

use crate::scenario::{EgoEstimatorAlgorithm, EgoEstimatorConfig, ImuConfig};

use self::{basic::BasicEkf, imu_bias::ImuBiasEkf};

#[derive(Debug, Clone, Copy, PartialEq)]
enum UpdateResult {
    Applied { normalized_residual: f64 },
    Rejected { normalized_residual: f64 },
    Invalid,
}

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
pub struct GpsDiagnostics {
    pub attempted_fixes: usize,
    pub accepted_fixes: usize,
    pub rejected_fixes: usize,
    pub invalid_fixes: usize,
    pub maximum_normalized_residual: f64,
}

impl GpsDiagnostics {
    fn record(&mut self, result: UpdateResult) {
        self.attempted_fixes += 1;
        match result {
            UpdateResult::Applied {
                normalized_residual,
            } => {
                self.accepted_fixes += 1;
                self.maximum_normalized_residual =
                    self.maximum_normalized_residual.max(normalized_residual);
            }
            UpdateResult::Rejected {
                normalized_residual,
            } => {
                self.rejected_fixes += 1;
                self.maximum_normalized_residual =
                    self.maximum_normalized_residual.max(normalized_residual);
            }
            UpdateResult::Invalid => self.invalid_fixes += 1,
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
    pub algorithm: EgoEstimatorAlgorithm,
    pub state_order: Vec<String>,
    pub initial_covariance_diagonal: Vec<f64>,
    pub imu_process_noise: ImuProcessNoise,
    pub gps_gate_sigma: f64,
}

#[derive(Debug)]
pub struct EstimatorRun {
    pub estimates: Vec<EgoStateEstimate>,
    pub timing: TimingDiagnostics,
    pub gps_diagnostics: GpsDiagnostics,
    pub assumptions: BaselineAssumptions,
}

pub fn run_baseline(
    config: &EgoEstimatorConfig,
    imu: &ImuConfig,
    measurements: &[EgoMeasurement],
) -> Result<EstimatorRun> {
    validate_delivery_order(measurements)?;
    let filter = ActiveEkf::new(config);
    let assumptions = filter.assumptions(config, imu);
    if config.timing_compensation {
        run_at_measurement_time(config, measurements, assumptions, filter)
    } else {
        run_at_arrival(config, measurements, assumptions, filter)
    }
}

fn run_at_arrival(
    config: &EgoEstimatorConfig,
    measurements: &[EgoMeasurement],
    assumptions: BaselineAssumptions,
    mut filter: ActiveEkf,
) -> Result<EstimatorRun> {
    let mut gps_diagnostics = GpsDiagnostics::default();
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
                estimates.push(filter.estimate(time.measurement_time_ns, time.arrival_time_ns));
            }
            EgoMeasurement::Gps(fix) => gps_diagnostics.record(filter.update_gps(config, fix)?),
        }
    }
    Ok(EstimatorRun {
        estimates,
        timing: timing(config, measurements, delayed, 0, 0, 0),
        gps_diagnostics,
        assumptions,
    })
}

fn run_at_measurement_time(
    config: &EgoEstimatorConfig,
    measurements: &[EgoMeasurement],
    assumptions: BaselineAssumptions,
    mut filter: ActiveEkf,
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

    let mut gps_diagnostics = GpsDiagnostics::default();
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
                EgoMeasurement::Gps(fix) => gps_diagnostics.record(filter.update_gps(config, fix)?),
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
            estimates.push(filter.estimate(stamp, final_emission));
        }
        index = end;
    }
    Ok(EstimatorRun {
        estimates,
        timing: timing(config, measurements, delayed, replayed, discarded, revised),
        gps_diagnostics,
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

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ImuProcessNoise {
    pub gyro_white_noise_density_radps_sqrt_hz: f64,
    pub accel_white_noise_density_mps2_sqrt_hz: f64,
    pub gyro_bias_random_walk_radps_sqrt_s: f64,
    pub accel_bias_random_walk_mps2_sqrt_s: f64,
}

impl ImuProcessNoise {
    fn for_algorithm(config: &ImuConfig, algorithm: EgoEstimatorAlgorithm) -> Self {
        let estimates_bias = algorithm == EgoEstimatorAlgorithm::ImuBias;
        Self {
            gyro_white_noise_density_radps_sqrt_hz: config.gyro_white_noise_density_radps_sqrt_hz,
            accel_white_noise_density_mps2_sqrt_hz: config.accel_white_noise_density_mps2_sqrt_hz,
            gyro_bias_random_walk_radps_sqrt_s: if estimates_bias {
                config.gyro_bias_random_walk_radps_sqrt_s
            } else {
                0.0
            },
            accel_bias_random_walk_mps2_sqrt_s: if estimates_bias {
                config.accel_bias_random_walk_mps2_sqrt_s
            } else {
                0.0
            },
        }
    }
}

enum ActiveEkf {
    Basic(BasicEkf),
    ImuBias(ImuBiasEkf),
}

impl ActiveEkf {
    fn new(config: &EgoEstimatorConfig) -> Self {
        match config.algorithm {
            EgoEstimatorAlgorithm::Basic => Self::Basic(BasicEkf::new(config)),
            EgoEstimatorAlgorithm::ImuBias => Self::ImuBias(ImuBiasEkf::new(config)),
        }
    }

    fn assumptions(&self, config: &EgoEstimatorConfig, imu: &ImuConfig) -> BaselineAssumptions {
        let (state_names, initial_covariance_diagonal): (&[&str], Vec<f64>) = match self {
            Self::Basic(filter) => (&basic::STATE_NAMES, filter.covariance_diagonal()),
            Self::ImuBias(filter) => (&imu_bias::STATE_NAMES, filter.covariance_diagonal()),
        };
        BaselineAssumptions {
            algorithm: config.algorithm,
            state_order: state_names.iter().map(|name| (*name).to_owned()).collect(),
            initial_covariance_diagonal,
            imu_process_noise: ImuProcessNoise::for_algorithm(imu, config.algorithm),
            gps_gate_sigma: config.gps_gate_sigma,
        }
    }

    fn propagate(&mut self, imu: &ImuSample, noise: &ImuProcessNoise) -> Result<()> {
        match self {
            Self::Basic(filter) => filter.propagate(imu, noise),
            Self::ImuBias(filter) => filter.propagate(imu, noise),
        }
    }

    fn update_gps(&mut self, config: &EgoEstimatorConfig, fix: &GpsFix) -> Result<UpdateResult> {
        match self {
            Self::Basic(filter) => filter.update_gps(config, fix),
            Self::ImuBias(filter) => filter.update_gps(config, fix),
        }
    }

    fn estimate(&self, estimate_time_ns: i64, available_time_ns: i64) -> EgoStateEstimate {
        match self {
            Self::Basic(filter) => filter.estimate(estimate_time_ns, available_time_ns),
            Self::ImuBias(filter) => filter.estimate(estimate_time_ns, available_time_ns),
        }
    }
}
