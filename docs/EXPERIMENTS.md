# Experiments

Each study is a YAML file. Its opening comments state the question and what to
watch. Comments beside individual settings suggest edits.

Planned demos are in [`DEMOS.md`](DEMOS.md).

Work through them in this order:

1. [`initial.yaml`](../experiments/initial.yaml)
2. [`imu_bias.yaml`](../experiments/imu_bias.yaml)
3. [`outliers.yaml`](../experiments/outliers.yaml)
4. [`timing.yaml`](../experiments/timing.yaml)
5. [`tracker_update.yaml`](../experiments/tracker_update.yaml)
6. [`perception.yaml`](../experiments/perception.yaml)
7. [`association.yaml`](../experiments/association.yaml)

Run one with:

```sh
fusion run experiments/imu_bias.yaml --view
```

The default output is the next free `runs/runNNN` directory. Reopen it with
`fusion view runs/run001`, or compare two runs with
`fusion compare runs/run001 runs/run002`.

## Sweeps

[`localization_sweep.yaml`](../experiments/localization_sweep.yaml) runs a
parameter grid over paired random seeds.
[`perception.yaml`](../experiments/perception.yaml) defines the camera/lidar
study, and [`perception_sweep.yaml`](../experiments/perception_sweep.yaml) runs
its three sensor cases:

```sh
fusion sweep experiments/perception_sweep.yaml --output runs/perception-sweep
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
reports/baseline/tracker-history-estimated-ego.json
reports/baseline/tracker-history-truth-ego.json
reports/baseline/visualization.rrd
```

The resolved scenario records the defaults omitted from the experiment. One
seed is useful for debugging. Use several paired seeds before making a general
claim.
