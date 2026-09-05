# Fusion in Motion

Fusion in Motion is a simulator for testing sensor fusion and object tracking.
It models two connected systems:

```text
GPS + IMU -> vehicle state
camera + lidar + vehicle state -> object tracks
```

GPS and IMU measurements feed the vehicle estimator. Camera and lidar
detections feed the object tracker, which also uses the estimated vehicle state.

A scenario defines vehicle and object motion over time. The simulator calculates
what each sensor would report, then adds configured noise, bias, missed
detections, and delay. It produces measurements, not camera images or lidar
point clouds.

The current simulator is planar. Vehicle motion follows acceleration and turn
rate segments, while objects move at configured velocities. GPS reports vehicle
position, the IMU reports forward acceleration and rotation, the camera reports
direction to an object, and lidar reports direction and distance.

## Run it

Install the command using [docs/INSTALL.md](docs/INSTALL.md), then run:

```sh
fusion run experiments/initial.yaml --view
```

Runs are saved as `runs/run001`, `runs/run002`, and so on. Start with
[docs/START_HERE.md](docs/START_HERE.md) for the dashboard and the first edits
to try.

The starter keeps latency, IMU bias, missed detections, and outlier gating out
of the way. The files under [`experiments/`](experiments/) add those effects one
at a time. See [docs/EXPERIMENTS.md](docs/EXPERIMENTS.md) for the sequence.

[docs/REFERENCES.md](docs/REFERENCES.md) contains the simulator and benchmark
review, literature, and recommended demo roadmap.

## Development

```sh
cargo fmt --all -- --check
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo build --release --workspace
```

Run every experiment and sweep and compare the results with the
[committed baselines](crates/fusion/tests/fixtures/experiment_baselines.json):

```sh
cargo test -p fusion-in-motion --test experiment_regressions
```

Selected metrics may differ by 1%; counts must match. Failures print the
changed value.

Dashboard and experiment changes also require the screenshot workflow in
[AGENTS.md](AGENTS.md).

## License

Project code and documentation use the [MIT License](LICENSE).
