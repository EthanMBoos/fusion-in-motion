# Experiments

Each study is a YAML file. Its opening comments state the question and what to
watch. Comments beside individual settings suggest edits.

Work through them in this order:

1. [`initial.yaml`](../experiments/initial.yaml)
2. [`imu_bias.yaml`](../experiments/imu_bias.yaml)
3. [`outliers.yaml`](../experiments/outliers.yaml)
4. [`timing.yaml`](../experiments/timing.yaml)
5. [`association.yaml`](../experiments/association.yaml)

Run one with:

```sh
fusion run experiments/imu_bias.yaml --view
```

The default output is the next free `runs/runNNN` directory. Reopen it with
`fusion view runs/run001`, or compare two runs with
`fusion compare runs/run001 runs/run002`.

## Sweeps

[`localization_sweep.yaml`](../experiments/localization_sweep.yaml) and
[`perception_sweep.yaml`](../experiments/perception_sweep.yaml) run parameter
grids over paired random seeds:

```sh
fusion sweep experiments/localization_sweep.yaml --output runs/localization-sweep
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

The resolved scenario records the defaults omitted from the experiment. One
seed is useful for debugging. Use several paired seeds before making a general
claim.
