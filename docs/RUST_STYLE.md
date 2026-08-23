# State-estimation-oriented Rust style

Standard Rust conventions, `rustfmt`, and Clippy are the baseline. The rules
below adapt that baseline so sensor and estimator code can be read in physical
and algorithmic order by an engineer who is not a Rust specialist.

## Optimize for an engineering review

The most important code should make these questions easy to answer:

- What physical quantity is this?
- Which frame is it expressed in?
- What unit does it use?
- Is it measured, predicted, corrected, or hidden truth?
- Which stage of the sensor or estimator produced it?

Prefer a few explicit intermediate values over a compact iterator chain or a
deeply nested expression when the intermediate values answer those questions.

## Names carry frames and units

- Put the frame before the unit: `position_world_m`, `velocity_body_mps`, and
  `yaw_world_from_body_rad`.
- Use unit suffixes on physical scalars, including private fields and function
  arguments: `_m`, `_mps`, `_mps2`, `_rad`, `_radps`, `_s`, and `_ns`.
- Use `world` and `body` rather than a bare `w` or `b` in public interfaces.
- Short names such as `dt_s`, `dx_m`, and `dy_m` are appropriate inside a small
  equation function when their meaning is established immediately.
- Do not use Unicode identifiers. ASCII names are easier to type, search, and
  discuss.

## Named physical state, matrix coordinates

Represent the nominal physical state with a struct whose fields have names.
Do not access position, attitude, speed, or bias through unexplained vector
indices.

Fixed-size `nalgebra` vectors and matrices are appropriate for covariance,
Jacobians, gains, and state corrections. Define the matrix-coordinate order
once with a named enum and use those indices when constructing matrices. Do not
describe the baseline as an error-state filter unless it actually adopts an
error-state formulation.

```rust
struct PlanarState {
    position_world_m: Vector2<f64>,
    yaw_world_from_body_rad: f64,
    forward_speed_mps: f64,
    gyro_bias_radps: f64,
    accel_bias_mps2: f64,
}
```

Use named structs for groups of physically different values. In particular, do
not return geometry as a tuple of anonymous `f64` values.

## Write estimator operations in stages

An IMU propagation function should read in this order:

```text
measurement and elapsed time
-> bias correction
-> nominal-state propagation
-> state-transition and process-noise construction
-> covariance propagation
```

A measurement update should read in this order:

```text
predict measurement
-> observed minus predicted residual
-> measurement Jacobian and variance
-> innovation uncertainty and Kalman gain
-> state and covariance correction
```

Keep orchestration, physical models, stochastic effects, and serialization out
of these equation-level functions.

## Connect notation to meaning

Use descriptive names with a conventional estimation suffix when it helps an
engineer compare code with a reference:

```rust
let state_transition_f = ...;
let process_noise_q = ...;
let measurement_jacobian_h = ...;
let innovation_variance_s = ...;
let kalman_gain_k = ...;
```

This retains recognizable `F`, `Q`, `H`, `S`, and `K` without requiring the
reader to remember a single-letter variable. Public interfaces always use
descriptive names.

Comments should explain the physical model, convention, approximation, or
reason for a step. They should not merely translate the following Rust line
into English. A compact equation in a comment is useful only after the plain
meaning is established.

## Sensor models have explicit boundaries

Keep these stages separate, even when they live in the same sensor module:

```text
trajectory sample
-> ideal geometry or ideal inertial signal
-> field of view and detection
-> bias, noise, saturation, and quantization
-> timing and delivery metadata
```

Random draws use stable engineering identities rather than collection order.
Hidden ideal values and applied effects remain available for diagnosis, but
estimator code must only receive the visible measurement.

## Runtime and dependency boundary

Estimator propagation and update functions are deterministic and free of file
I/O, visualization, wall-clock access, and random draws. Configuration parsing,
MCAP handling, command-line behavior, and Rerun output stay at the boundary.

Prefer the existing fixed-size `nalgebra` types over adding an estimator DSL or
unit framework. Add stronger unit or frame wrapper types only where repeated
mistakes show that naming and tests are insufficient.

## Tests express engineering invariants

Equation changes should include the smallest applicable checks:

- an analytically solvable or zero-effect case;
- a convention test for frames, signs, quaternion ordering, or angle wrapping;
- a deterministic fixed-seed sensor case;
- a covariance symmetry and nonnegative-diagonal check; and
- an end-to-end check that estimator-visible inputs remain separated from
  hidden truth.

Name tests after the physical behavior they protect. Use tolerances with an
explicit scale rather than exact floating-point equality unless equality is the
property under test.
