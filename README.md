# Fusion in Motion

https://github.com/user-attachments/assets/0f9fa725-449f-4467-9a2e-a6477f6f2ba8

**Fusion in Motion** is a playground for learning when a sensor-fusion setup
works and when it falls apart. It generates repeatable motion and sensor data,
runs an estimator without exposing the hidden truth, scores the result, and
saves an animated Rerun dashboard.

The initial example is a planar ground robot with an IMU, camera landmark
bearings, and timed lidar returns. A small EKF is included as a working
baseline. Change the platform speed, sensor rates, latency, noise, or missed
detections to see how the estimate responds.

Start with [three guided experiments](docs/START_HERE.md): run the baseline,
read the dashboard, then change speed, camera latency, and lidar scan duration
one at a time.

The longer-term goal includes testing the path from larger sensor payloads to
the estimator. That makes it possible to separate a bad fusion technique from
a system that cannot move, preprocess, and deliver its measurements quickly
enough. The reasoning and proposed experiment structure are described in
[Sensor pipelines and estimator performance](docs/SENSOR_PIPELINES.md).

## Repository layout

- `crates/fusion/` contains the scenario runner, truth and sensor generation,
  replay, baseline EKF, evaluation, and visualization.
- `crates/fusion-schema/` generates the shared Protobuf messages.
- `proto/` contains the measurement, truth, and estimate definitions.
- `examples/` contains runnable scenarios and parameter sweeps.
- `docs/` contains installation, experiment, estimator, implementation style,
  and background notes.
- `runs/` is the ignored location for generated experiment results.

## Run the initial example

Complete the [installation guide](docs/INSTALL.md), then run this from the
repository root:

```sh
fusion run examples/initial.yaml --view
```

Each run gets the next free folder: `runs/run001`, `runs/run002`, and so on.
`--output` selects a different new folder. Existing folders are not replaced.

Rerun opens a looping dashboard with the true and estimated path, ego-centric
camera and lidar views, estimation error, IMU data, and observation counts. The
run folder also contains the sensor measurements, hidden truth, estimates, and
metrics needed to inspect or repeat the result.

[Experiments](docs/EXPERIMENTS.md) explains the scenario settings, optional
radar example, parameter sweeps, and run folders. To evaluate another estimator
against the same measurements, see [Using another estimator](docs/ESTIMATORS.md).

## What is simulated today

The current sensors produce lightweight analytic observations rather than raw
images, dense point clouds, or raw radar data. Landmark association is supplied
to the estimator. This keeps the first example understandable while exercising
sensor timing, noise, motion, replay, estimation, scoring, and visualization.

These experiments characterize the configured model. They are not claims
about the performance of a physical sensor or a production runtime.

## Core simulator development

Run the local checks after changing the simulator:

```sh
cargo fmt --all -- --check
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo build --release --workspace
```

## License

Project code and documentation use the [MIT License](LICENSE).
