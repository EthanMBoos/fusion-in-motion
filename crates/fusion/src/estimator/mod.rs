mod camera;
mod lidar;
mod observation;
mod propagation;
mod state;

use std::collections::BTreeMap;

use anyhow::{Result, ensure};
use fusion_schema::messages::{CovarianceKind, EstimateStatus, LandmarkMap, StateEstimate, Vec3};
use nalgebra::Vector2;

use crate::{bundle::MeasurementRecord, math, scenario::EstimatorConfig};

use self::state::{PlanarState, StateCovariance};

pub fn run_baseline(
    config: &EstimatorConfig,
    measurements: &[MeasurementRecord],
) -> Result<Vec<StateEstimate>> {
    let mut filter = BaselineEkf::new();
    let mut estimates = Vec::new();
    let mut last_delivery_index = None;

    for measurement in measurements {
        let header = measurement.header();
        if let Some(previous_delivery_index) = last_delivery_index {
            ensure!(
                header.delivery_index > previous_delivery_index,
                "measurement delivery indices are not strictly increasing"
            );
        }
        last_delivery_index = Some(header.delivery_index);

        match measurement {
            MeasurementRecord::Map(map) => filter.handle_map(map)?,
            MeasurementRecord::Imu(imu) => {
                propagation::propagate_imu(
                    &mut filter.state,
                    &mut filter.covariance,
                    &mut filter.last_imu_stamp_ns,
                    imu,
                )?;
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
            )?,
            MeasurementRecord::Lidar(scan) => lidar::update(
                &mut filter.state,
                &mut filter.covariance,
                &filter.landmarks,
                config,
                scan,
            )?,
        }
    }

    Ok(estimates)
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
