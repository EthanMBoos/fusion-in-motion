use anyhow::{Result, ensure};
use fusion_schema::messages::GpsFix;
use nalgebra::{Matrix2, Vector2};

use crate::scenario::EgoEstimatorConfig;

use super::{
    observation::{UpdateResult, apply_position_update},
    state::{PlanarState, StateCovariance},
};

pub fn update(
    state: &mut PlanarState,
    covariance: &mut StateCovariance,
    estimator: &EgoEstimatorConfig,
    fix: &GpsFix,
) -> Result<UpdateResult> {
    let position = fix
        .position_world_m
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("GPS fix has no position"))?;
    ensure!(
        fix.horizontal_position_variance_m2.is_finite()
            && fix.horizontal_position_variance_m2 >= 0.0,
        "GPS variance must be finite and nonnegative"
    );
    Ok(apply_position_update(
        state,
        covariance,
        Vector2::new(position.x, position.y) - state.position_world_m,
        Matrix2::identity() * fix.horizontal_position_variance_m2,
        estimator.gps_gate_sigma,
    ))
}
