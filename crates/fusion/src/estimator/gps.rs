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
    _config: &GpsConfig,
    fix: &GpsFix,
    diagnostics: &mut FilterDiagnostics,
) -> Result<()> {
    let position = fix
        .position_world_m
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("GPS fix has no position"))?;
    ensure!(
        fix.horizontal_position_variance_m2.is_finite()
            && fix.horizontal_position_variance_m2 >= 0.0,
        "GPS variance must be finite and nonnegative"
    );
    let predicted_x = state.position_world_m.x;

    let mut x_jacobian = StateCorrection::zeros();
    x_jacobian[StateIndex::PositionWorldX.index()] = 1.0;
    diagnostics.record(apply_scalar_update(
        state,
        covariance,
        position.x - predicted_x,
        x_jacobian,
        fix.horizontal_position_variance_m2,
        estimator.gps_gate_sigma,
    ));

    let predicted_y = state.position_world_m.y;
    let mut y_jacobian = StateCorrection::zeros();
    y_jacobian[StateIndex::PositionWorldY.index()] = 1.0;
    diagnostics.record(apply_scalar_update(
        state,
        covariance,
        position.y - predicted_y,
        y_jacobian,
        fix.horizontal_position_variance_m2,
        estimator.gps_gate_sigma,
    ));
    Ok(())
}
