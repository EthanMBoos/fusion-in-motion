# Experiments

Fusion in Motion is a playground for questions such as:

- What happens when the platform moves twice as fast?
- Does a higher sensor rate compensate for latency?
- How sensitive is an estimator to noise, bias, or missed detections?
- Which sensor configuration gives a useful accuracy tradeoff?

It generates repeatable analytic measurements, replays them through an
estimator, scores the estimate against hidden truth, and saves the result for
inspection.

## Initial example

The initial example is a planar ground robot moving through a sparse landmark
map.
It has:

- an IMU measuring angular rate and specific force;
- a camera producing landmark bearings;
- a lidar producing timed landmark range and bearing returns.

These are lightweight analytic observations for experimenting with a fusion
back end. They are not raw images or dense point clouds. Landmark association
is supplied to the estimator.

Run it with:

```sh
fusion run examples/initial.yaml --view
```

## Scenario settings

Scenario YAML contains the motion, landmarks, sensor settings, random seed,
baseline-estimator assumptions, and evaluation thresholds. Physical units are
included in field names. Unknown fields are rejected so a misspelling does not
silently create a different experiment.

Useful settings include:

- `motion_speed_factor`;
- trajectory duration, acceleration, and yaw rate;
- sensor rate, range, field of view, and latency;
- IMU noise, bias, saturation, and quantization;
- camera and lidar noise and detection probability;
- the measurement noise assumed by the baseline estimator.

### Motion speed

`motion_speed_factor` changes how quickly the configured path is traversed. A
factor of `2.0` uses half the time, twice the velocity and yaw rate, and four
times the longitudinal acceleration. The geometric path stays the same.

Sensor rates remain samples per second. A faster traversal therefore receives
fewer measurements along the path, which is usually the behavior an experiment
wants to expose.

## Sweeps

A sweep starts from one scenario and replaces selected fields with lists of
values. The runner evaluates their Cartesian product for every requested seed.

```yaml
name: Speed and lidar-rate comparison
base_scenario: initial.yaml
seeds: [10, 11, 12]
parameters:
  motion_speed_factor: [0.5, 1.0, 2.0]
  lidar.rate_hz: [2.0, 5.0, 10.0]
  lidar.latency_ns: [10000000, 50000000]
```

Run a sweep with:

```sh
fusion sweep examples/speed_sensor_sweep.yaml --output runs/speed-sensor-sweep
```

Dotted paths refer directly to scenario fields. Sequence indices are also
accepted, for example `trajectory.3.yaw_rate_radps`. Sweeps are capped at 10,000
cases to catch accidentally enormous grids.

The root report contains:

```text
reports/summary.md
reports/results.csv
reports/results.json
```

Every `case-NNNN` directory is a complete run. Failed cases remain in the
aggregate report. Rerun recordings are skipped during a sweep; open an
interesting case afterward:

```sh
fusion view runs/speed-sensor-sweep/case-0004
```

## Run folders

A normal run contains:

```text
manifest.json
scenario.resolved.yaml
measurements.mcap
truth.mcap
estimates/baseline.mcap
reports/baseline/metrics.json
reports/baseline/summary.md
reports/baseline/visualization.rrd
```

`measurements.mcap` is what the estimator sees. `truth.mcap` is opened only
after estimation for scoring and visualization. The manifest records the
scenario seed, software information, warnings, run status, and artifact hashes.

With no `--output`, each run uses the next free `runs/runNNN` folder. An
explicit output path must be new. Existing runs are not replaced.

## Randomness

Sensor noise and missed detections are generated from the scenario's root seed
and stable names. Repeating a resolved scenario produces the same logical
measurements. Changing one landmark or sensor does not intentionally advance a
shared random stream used by everything else.

One seed is useful for debugging. Use several paired seeds before drawing a
conclusion from a comparison.

## Results

The evaluator reports position and yaw error, final drift, availability, time
coverage, divergence, and time to first valid output. For estimators that emit
a full covariance, it also validates the matrix and reports ANEES plus marginal
95% coverage for x, y, yaw, and forward speed. Sweep reports retain the
parameter values and root seed for paired comparisons.

The covariance is row-major for
`[x, y, yaw, forward_speed, gyro_bias_z, accel_bias_x]`. Consistency errors use
additive world-frame x/y, wrapped world-from-body yaw, and signed body-forward
speed. Bias blocks are validated but not scored because realized bias truth is
not stored. The dashboard's position uncertainty line is the outer radius of
the 2D 95% covariance ellipse, not a marginal x or y bound.

The values describe this analytic experiment. They are not claims about the
performance of physical sensors.

The planned distinction between estimator behavior and runtime throughput is
described in [Sensor pipelines and estimator performance](SENSOR_PIPELINES.md).

## Current boundaries

The project currently focuses on analytic planar experiments. It does not try
to provide vehicle physics, raw perception, photorealistic rendering,
production data association, live ROS execution, or permanently frozen file
formats. Those can be added later if a specific experiment needs them.
