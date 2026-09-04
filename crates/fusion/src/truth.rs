use anyhow::{Result, ensure};
use fusion_schema::messages::{EgoTruthState, ObjectTruthState, Vec2};

use crate::{math, scenario::ResolvedScenario};

#[derive(Debug, Clone, Copy)]
pub struct Sample {
    pub time_s: f64,
    pub x_world_m: f64,
    pub y_world_m: f64,
    pub yaw_world_from_body_rad: f64,
    pub speed_mps: f64,
    pub path_distance_m: f64,
    pub longitudinal_acceleration_mps2: f64,
    pub yaw_rate_radps: f64,
}

#[derive(Debug, Clone)]
struct SegmentState {
    start_s: f64,
    end_s: f64,
    start_x_world_m: f64,
    start_y_world_m: f64,
    start_yaw_world_from_body_rad: f64,
    start_forward_speed_mps: f64,
    start_path_distance_m: f64,
    longitudinal_acceleration_mps2: f64,
    yaw_rate_radps: f64,
}

#[derive(Debug, Clone)]
pub struct Trajectory {
    segments: Vec<SegmentState>,
    duration_s: f64,
}

impl Trajectory {
    pub fn new(scenario: &ResolvedScenario) -> Result<Self> {
        let speed_factor = scenario.motion_speed_factor;
        let mut segments = Vec::with_capacity(scenario.trajectory.len());
        let (mut time, mut x, mut y, mut yaw, mut speed, mut distance) =
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        for configured in &scenario.trajectory {
            let duration_s = configured.duration_s / speed_factor;
            let state = SegmentState {
                start_s: time,
                end_s: time + duration_s,
                start_x_world_m: x,
                start_y_world_m: y,
                start_yaw_world_from_body_rad: yaw,
                start_forward_speed_mps: speed,
                start_path_distance_m: distance,
                longitudinal_acceleration_mps2: configured.longitudinal_acceleration_mps2
                    * speed_factor.powi(2),
                yaw_rate_radps: configured.yaw_rate_radps * speed_factor,
            };
            let end = integrate(&state, duration_s);
            time = state.end_s;
            x = end.x_world_m;
            y = end.y_world_m;
            yaw = end.yaw_world_from_body_rad;
            speed = end.speed_mps;
            distance = end.path_distance_m;
            ensure!(
                speed >= -1.0e-9,
                "segment {} produces a negative forward speed; use a positive-speed reverse profile in a later version",
                configured.id
            );
            segments.push(state);
        }
        Ok(Self {
            segments,
            duration_s: scenario.effective_duration_s(),
        })
    }

    pub fn sample_ns(&self, time_ns: i64) -> Sample {
        self.sample_s(time_ns as f64 * 1.0e-9)
    }

    pub fn sample_s(&self, time_s: f64) -> Sample {
        let clamped = time_s.clamp(0.0, self.duration_s);
        let state = self
            .segments
            .iter()
            .find(|segment| clamped < segment.end_s)
            .unwrap_or_else(|| self.segments.last().expect("validated nonempty trajectory"));
        integrate(
            state,
            (clamped - state.start_s).clamp(0.0, state.end_s - state.start_s),
        )
    }

    pub fn truth_state(&self, time_ns: i64) -> EgoTruthState {
        let sample = self.sample_ns(time_ns);
        EgoTruthState {
            time_ns,
            pose_world: Some(math::pose2(
                sample.x_world_m,
                sample.y_world_m,
                sample.yaw_world_from_body_rad,
            )),
            forward_speed_mps: sample.speed_mps,
        }
    }
}

pub fn object_truth_states(scenario: &ResolvedScenario, period_ns: i64) -> Vec<ObjectTruthState> {
    let end_ns = (scenario.effective_duration_s() * 1.0e9).round() as i64;
    let mut states = Vec::new();
    let mut time_ns = 0;
    while time_ns <= end_ns {
        let time_s = time_ns as f64 * 1.0e-9;
        for object in &scenario.world.objects {
            states.push(ObjectTruthState {
                track_key: object.id.clone(),
                time_ns,
                position_world_m: Some(Vec2 {
                    x: object.initial_position_m.x + object.velocity_world_mps.x * time_s,
                    y: object.initial_position_m.y + object.velocity_world_mps.y * time_s,
                }),
                velocity_world_mps: Some(Vec2 {
                    x: object.velocity_world_mps.x,
                    y: object.velocity_world_mps.y,
                }),
            });
        }
        time_ns += period_ns;
    }
    states
}

fn integrate(segment: &SegmentState, dt_s: f64) -> Sample {
    let start_speed_mps = segment.start_forward_speed_mps;
    let acceleration_mps2 = segment.longitudinal_acceleration_mps2;
    let yaw_rate_radps = segment.yaw_rate_radps;
    let start_yaw_rad = segment.start_yaw_world_from_body_rad;

    let yaw_rad = start_yaw_rad + yaw_rate_radps * dt_s;
    let distance_m = start_speed_mps * dt_s + 0.5 * acceleration_mps2 * dt_s * dt_s;
    let (dx_world_m, dy_world_m) = if yaw_rate_radps.abs() < 1.0e-10 {
        (
            distance_m * start_yaw_rad.cos(),
            distance_m * start_yaw_rad.sin(),
        )
    } else {
        let yaw_rate_squared = yaw_rate_radps * yaw_rate_radps;
        let dx_world_m = start_speed_mps / yaw_rate_radps * (yaw_rad.sin() - start_yaw_rad.sin())
            + acceleration_mps2
                * (dt_s * yaw_rad.sin() / yaw_rate_radps
                    + (yaw_rad.cos() - start_yaw_rad.cos()) / yaw_rate_squared);
        let dy_world_m = start_speed_mps / yaw_rate_radps * (start_yaw_rad.cos() - yaw_rad.cos())
            + acceleration_mps2
                * (-dt_s * yaw_rad.cos() / yaw_rate_radps
                    + (yaw_rad.sin() - start_yaw_rad.sin()) / yaw_rate_squared);
        (dx_world_m, dy_world_m)
    };

    Sample {
        time_s: segment.start_s + dt_s,
        x_world_m: segment.start_x_world_m + dx_world_m,
        y_world_m: segment.start_y_world_m + dy_world_m,
        yaw_world_from_body_rad: yaw_rad,
        speed_mps: start_speed_mps + acceleration_mps2 * dt_s,
        path_distance_m: segment.start_path_distance_m + distance_m,
        longitudinal_acceleration_mps2: acceleration_mps2,
        yaw_rate_radps,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn straight_and_circle_are_closed_form() {
        let straight = SegmentState {
            start_s: 0.0,
            end_s: 2.0,
            start_x_world_m: 0.0,
            start_y_world_m: 0.0,
            start_yaw_world_from_body_rad: 0.0,
            start_forward_speed_mps: 1.0,
            start_path_distance_m: 0.0,
            longitudinal_acceleration_mps2: 0.0,
            yaw_rate_radps: 0.0,
        };
        let sample = integrate(&straight, 2.0);
        assert!((sample.x_world_m - 2.0).abs() < 1.0e-12);
        assert!(sample.y_world_m.abs() < 1.0e-12);

        let circle = SegmentState {
            yaw_rate_radps: 1.0,
            ..straight
        };
        let quarter = integrate(&circle, std::f64::consts::FRAC_PI_2);
        assert!((quarter.x_world_m - 1.0).abs() < 1.0e-12);
        assert!((quarter.y_world_m - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn speed_factor_preserves_path_and_scales_time() {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../examples/initial.yaml");
        let nominal_scenario = crate::scenario::load_and_resolve(&path).unwrap();
        let mut fast_scenario = nominal_scenario.clone();
        fast_scenario.motion_speed_factor = 2.0;
        let nominal = Trajectory::new(&nominal_scenario).unwrap();
        let fast = Trajectory::new(&fast_scenario).unwrap();

        let nominal_mid = nominal.sample_s(4.0);
        let fast_mid = fast.sample_s(2.0);
        assert!((nominal_mid.x_world_m - fast_mid.x_world_m).abs() < 1.0e-12);
        assert!((nominal_mid.y_world_m - fast_mid.y_world_m).abs() < 1.0e-12);
        assert!(
            (nominal_mid.yaw_world_from_body_rad - fast_mid.yaw_world_from_body_rad).abs()
                < 1.0e-12
        );
        assert!((2.0 * nominal_mid.speed_mps - fast_mid.speed_mps).abs() < 1.0e-12);

        let nominal_end = nominal.sample_s(nominal_scenario.duration_s());
        let fast_end = fast.sample_s(fast_scenario.effective_duration_s());
        assert!((nominal_end.x_world_m - fast_end.x_world_m).abs() < 1.0e-12);
        assert!((nominal_end.y_world_m - fast_end.y_world_m).abs() < 1.0e-12);
        assert!(
            (nominal_end.yaw_world_from_body_rad - fast_end.yaw_world_from_body_rad).abs()
                < 1.0e-12
        );
    }
}
