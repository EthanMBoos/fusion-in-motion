# Fusion in Motion

This repo simulates two connected problems:

```text
GPS + IMU -> estimate the vehicle
camera + lidar + vehicle estimate -> track other objects
```

Camera and lidar do not correct the vehicle estimate. They detect objects. The
object tracker needs the vehicle estimate to tell vehicle motion from object
motion.

The simulator is planar. GPS reports vehicle position, the IMU reports forward
acceleration and rotation, the camera reports direction to an object, and lidar
reports direction and distance. Measurements are analytic rather than rendered
images or point clouds.

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

Dashboard and experiment changes also require the screenshot workflow in
[AGENTS.md](AGENTS.md).

## License

Project code and documentation use the [MIT License](LICENSE).
