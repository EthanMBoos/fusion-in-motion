# Experiments

Each scenario defines vehicle motion, objects, sensors, baseline settings, a
random seed, and scoring limits. Unknown YAML fields are rejected so a typo
does not silently change the experiment.

Run a scenario with:

```sh
fusion run examples/initial.yaml
```

The default output is the next free `runs/runNNN` folder. An explicit folder is
also accepted with `--output`.

## Useful changes

Localization settings live under `imu`, `gps`, and `ego_estimator`. Perception
and tracking settings live under `camera`, `lidar`, and `object_tracker`.
Changing camera or lidar must not change the vehicle estimate.

The fastest useful experiments are GPS on versus off, low versus high GPS
noise, camera versus lidar versus both, and truth ego versus estimated ego in
the object tracker. The last comparison is generated automatically from one
detection stream.

`motion_speed_factor` changes how quickly the configured vehicle path is
traversed while keeping its shape. Sensor rates remain samples per second, so a
faster run gets fewer observations per meter.

## Sweeps

A sweep replaces selected scenario fields with lists and runs their Cartesian
product over paired seeds:

```yaml
name: GPS noise and rate
base_scenario: initial.yaml
seeds: [10, 11, 12, 13, 14]
parameters:
  gps.rate_hz: [1.0, 2.0, 5.0]
  gps.horizontal_position_stddev_m: [0.1, 0.5, 1.0]
```

Run it with:

```sh
fusion sweep examples/localization_sweep.yaml --output runs/localization-sweep
```

The report includes every case, group means, sample standard deviation, and a
warning when a group has fewer than three successful seeds. Object results
include the paired error difference between estimated-ego and truth-ego
tracking.

## Run contents

```text
manifest.json
scenario.resolved.yaml
measurements.mcap
truth.mcap
estimates/ego-baseline.mcap
tracks/estimated-ego.mcap
tracks/truth-ego.mcap
reports/baseline/metrics.json
reports/baseline/summary.md
reports/baseline/visualization.rrd
```

`measurements.mcap` is estimator-visible. Normal localization and tracking do
not receive `truth.mcap`. The truth-ego tracker is a labeled scoring control,
not the normal data path.

Metrics match truth only within `metrics.max_truth_match_gap_ns`. Reported
`DIVERGED` status and exceeding an error threshold are counted separately.
Time coverage describes the part of the run covered by valid output; it does
not assume an estimator runs at IMU rate.

One seed is useful for debugging. Use paired seeds before making a general
claim.
