# Experiments

Start with `examples/initial.yaml`. It exposes the sensor rates, basic noise,
field of view, range, objects, and vehicle motion. Unknown YAML fields are
rejected so typos do not silently change a run.

The examples add complexity in this order:

| Example | Adds | Watch |
| --- | --- | --- |
| `initial.yaml` | GPS/IMU localization and camera/lidar tracking | vehicle and object error |
| `imu_bias.yaml` | sensor bias, drift, and bias estimation | the two bias plots |
| `outliers.yaml` | bad GPS fixes and observation gating | accepted and rejected update counts |
| `timing.yaml` | latency, lidar scan time, and delayed-data handling | GPS age and error during turns |
| `association.yaml` | unlabeled detections, track IDs, and crossing objects | track IDs before and after the crossing |
| `localization_sweep.yaml` | paired seeds over GPS settings | mean and spread of vehicle error |
| `perception_sweep.yaml` | paired seeds over perception settings | object error and ego cost |

The bias, outlier, and timing examples have short guides in
[BIAS_EXPERIMENT.md](BIAS_EXPERIMENT.md),
[OUTLIER_EXPERIMENT.md](OUTLIER_EXPERIMENT.md), and
[TIMING_EXPERIMENT.md](TIMING_EXPERIMENT.md). The association example is
explained in [ASSOCIATION_EXPERIMENT.md](ASSOCIATION_EXPERIMENT.md).

Run an example with:

```sh
fusion run examples/imu_bias.yaml --view
```

The default output is the next free `runs/runNNN` directory. Use `--output` only
when a named location is useful.

## Sweeps

A sweep replaces selected scenario fields with lists and runs every combination
over the same seeds:

```yaml
name: GPS noise and rate
base_scenario: initial.yaml
seeds: [10, 11, 12, 13, 14]
parameters:
  gps.rate_hz: [1.0, 2.0, 5.0]
  gps.horizontal_position_stddev_m: [0.1, 0.5, 1.0]
```

```sh
fusion sweep examples/localization_sweep.yaml --output runs/localization-sweep
```

The report contains every case, group means, sample standard deviation, and a
warning for groups with fewer than three successful seeds.

## Run files

```text
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

The resolved scenario records the defaults that were omitted from the example.
One seed is useful for debugging. Use several paired seeds before making a
general claim.
