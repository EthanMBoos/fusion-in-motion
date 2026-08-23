use anyhow::{Result, ensure};
use fusion_schema::messages::GpsFix;

use crate::scenario::{EgoEstimatorConfig, GpsConfig};

use super::{
    FilterDiagnostics,
    observation::apply_scalar_update,
    state::{PlanarState, StateCorrection, StateCovariance, StateIndex},
};

pub fn update(
    state: &mut PlanarState,
    covariance: &mut StateCovariance,
    estimator: &EgoEstimatorConfig,
    config: &GpsConfig,
    fix: &GpsFix,
    diagnostics: &mut FilterDiagnostics,
) -> Result<()> {
    ensure!(
        fix.frame_id == estimator.output_world_frame,
        "GPS frame does not match ego estimator world frame"
    );
    let position = fix
        .position_world_m
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("GPS fix has no position"))?;
    ensure!(
        fix.position_covariance.len() == 9,
        "GPS covariance must contain 9 values"
    );
    let yaw = state.yaw_world_from_body_rad;
    let mount = &config.mount.position_m;
    let predicted_x = state.position_world_m.x + yaw.cos() * mount.x - yaw.sin() * mount.y;

    let mut x_jacobian = StateCorrection::zeros();
    x_jacobian[StateIndex::PositionWorldX.index()] = 1.0;
    x_jacobian[StateIndex::YawWorldFromBody.index()] = -yaw.sin() * mount.x - yaw.cos() * mount.y;
    diagnostics.record(apply_scalar_update(
        state,
        covariance,
        position.x - predicted_x,
        x_jacobian,
        fix.position_covariance[0],
        estimator.gps_gate_sigma,
    ));

    let yaw = state.yaw_world_from_body_rad;
    let predicted_y = state.position_world_m.y + yaw.sin() * mount.x + yaw.cos() * mount.y;
    let mut y_jacobian = StateCorrection::zeros();
    y_jacobian[StateIndex::PositionWorldY.index()] = 1.0;
    y_jacobian[StateIndex::YawWorldFromBody.index()] = yaw.cos() * mount.x - yaw.sin() * mount.y;
    diagnostics.record(apply_scalar_update(
        state,
        covariance,
        position.y - predicted_y,
        y_jacobian,
        fix.position_covariance[4],
        estimator.gps_gate_sigma,
    ));
    Ok(())
}
