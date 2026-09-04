# Start here

This walkthrough takes about 45 minutes. Run the baseline, read the dashboard,
then change speed, camera latency, lidar scan duration, and estimator timing
compensation.

The first demo is 2D. The platform is a ground robot on a planar landmark map.
It covers propagation, partial observations, sensor rate, latency, acquisition
time, hidden truth, and estimation error without adding roll, pitch, gravity
alignment, or 3D calibration. Add 3D later when an experiment needs it.

## Set up a working scenario

Keep your edits under `runs/` so they do not show up as source changes:

```sh
mkdir -p runs
cp examples/initial.yaml runs/my_first_experiment.yaml
fusion check runs/my_first_experiment.yaml > /dev/null
```

The guide uses these fields:

| Experiment | Field | Baseline | Try |
| --- | --- | ---: | ---: |
| Faster motion | `motion_speed_factor` | `1.0` | `2.0` |
| Late camera data | `camera.latency_ns` | `18000000` | `500000000` |
| Timed lidar scan | `lidar.scan_duration_ns` | `0` | `80000000` |
| Timing handling | `estimator.timing_compensation` | `true` | `false` |

Timing fields are in nanoseconds. One millisecond is 1,000,000 ns. One second
is 1,000,000,000 ns.

## Run the baseline

```sh
fusion run runs/my_first_experiment.yaml
```

Fusion prints the run directory. New runs use the next free name under
`runs/`: `run001`, `run002`, and so on. Existing directories are not replaced.

The commands below assume this is a fresh `runs/` directory. Use the directories
printed by `fusion run` if other runs already exist.

```sh
fusion view runs/run001
```

## Read the dashboard

Pause playback and scrub through a straight segment and a turn.

### Map

Yellow points are landmarks. Green is truth. Pink is the estimate. The
estimator does not read truth; truth is loaded after estimation for scoring and
visualization.

### Sensors

- Camera rays show bearing only. Ray length does not represent depth.
- Lidar returns show range and bearing. Color runs from early to late within
  the scan. The green line shows platform motion during that scan. The baseline
  scan is instantaneous, so its returns share one acquisition time; Experiment
  3 turns scan timing on.
- The observation plot counts camera features and lidar returns.

### Timing

- Measurement age is receipt time minus reported timestamp.
- Acquisition duration is the interval covered by one record.

Latency means an observation is already old when the estimator receives it. A
lidar scan can also cover more than one platform pose.

The reference EKF accepts observations up to one second late. With
`timing_compensation: true`, it applies them at acquisition time, propagates the
correction forward, and deskews lidar returns using its estimated motion.
Measurements that need more history than the configured window are discarded.
The run report records replay, discard, revision, and deskew counts.

### Error

The error plot compares position and yaw error with reported 95% covariance.
The position line is the outer radius of the 2D covariance ellipse; the yaw
line is the marginal 95% bound.

The run report lists error, estimator status, and covariance metrics. See
[Results](EXPERIMENTS.md#results) for how they are calculated.

## Experiment 1: speed

Change this line in `runs/my_first_experiment.yaml`:

```yaml
motion_speed_factor: 2.0
```

Before running, predict what happens to path shape, traversal time, sensor
samples along the path, acceleration, and position error.

```sh
fusion check runs/my_first_experiment.yaml > /dev/null
fusion run runs/my_first_experiment.yaml
fusion compare runs/run001 runs/run002
fusion view runs/run002
```

The path shape stays fixed. At `2.0`, traversal takes half as long, velocity and
yaw rate double, and longitudinal acceleration is four times larger. Sensor
rates are still samples per second, so fewer samples cover the path.

## Experiment 2: camera latency

Reset `motion_speed_factor` to `1.0`. Then change:

```yaml
camera:
  latency_ns: 500000000  # 500 ms
```

Leave `estimator.timing_compensation: true`, run again, and check camera age in
the timing plot. The estimate should remain close because 500 ms fits inside
the one-second history.

```sh
fusion check runs/my_first_experiment.yaml > /dev/null
fusion run runs/my_first_experiment.yaml
fusion compare runs/run001 runs/run003
fusion view runs/run003
```

Now change:

```yaml
estimator:
  timing_compensation: false
```

Run once more. The uncompensated EKF applies the old camera bearing to the state
that exists when it arrives.

```sh
fusion run runs/my_first_experiment.yaml
fusion compare runs/run003 runs/run004
fusion view runs/run004
```

## Experiment 3: lidar scan duration

Reset camera latency to `18000000`, restore
`estimator.timing_compensation: true`, then change:

```yaml
lidar:
  scan_duration_ns: 80000000  # 80 ms
```

Run again. Scrub through a turn in the lidar panel. The return colors show when
each point was acquired. The green line shows how far the platform moved while
the scan was collected.

```sh
fusion check runs/my_first_experiment.yaml > /dev/null
fusion run runs/my_first_experiment.yaml
fusion compare runs/run001 runs/run005
fusion view runs/run005
```

Set `estimator.timing_compensation: false` and run the same scan again. The raw
lidar panel is unchanged, but the uncompensated estimate treats every return as
if it were acquired at the scan timestamp.

```sh
fusion run runs/my_first_experiment.yaml
fusion compare runs/run005 runs/run006
fusion view runs/run006
```

Then try `scan_duration_ns: 400000000`. After running each variable alone, use
that 400 ms scan with `motion_speed_factor: 4.0`. Keep the one-second history
when compensation is enabled.

## Compare and repeat

Compare metrics:

```sh
fusion compare runs/run005 runs/run006
```

Compare the full resolved scenarios:

```sh
diff -u runs/run001/scenario.resolved.yaml runs/run002/scenario.resolved.yaml
```

One seed is enough to debug an experiment. Use several seeds before making a
claim:

```sh
fusion sweep examples/beginner_sweep.yaml --output runs/beginner-sweep
```

Read `runs/beginner-sweep/reports/summary.md`. See
[Experiments](EXPERIMENTS.md) for trajectory changes, sensor noise, missed
detections, estimator assumptions, and larger sweeps.

Continue with the [IMU bias experiment](BIAS_EXPERIMENT.md) to inspect the two
bias states and their uncertainty across independent runs.
