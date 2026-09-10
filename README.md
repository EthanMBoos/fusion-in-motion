# Fusion in Motion

Fusion in Motion is a Rust workbench for sensor fusion and target tracking. It
is both:

- a runnable reference repo for simulation, estimation, tracking, evaluation,
  and visualization; and
- a reusable target-tracking engine for applications kept in other
  repositories.

The reference system connects two estimation problems:

```text
GPS + IMU -> vehicle state
camera + lidar + vehicle state -> object tracks
```

GPS and IMU estimate the moving vehicle. Camera and lidar detections, combined
with that vehicle estimate, produce object tracks. A second tracker uses the
true vehicle pose as a control, so the dashboard shows how vehicle error moves
the object tracks.

A YAML scenario defines motion, sensors, noise, bias, missed detections, delay,
filters, association, and track lifecycle. Each run writes the measurements,
truth, estimates, tracks, metrics, tracker history, and a Rerun dashboard. The
checked-in experiments change one effect at a time and keep numerical
baselines.

`fusion-tracking` is the reusable part. It runs measurement-time prediction,
hypothesis generation, association, posterior updates, initiation,
confirmation, coasting, deletion, and diagnostics. It includes gated global
nearest-neighbor assignment and accepts other models and association methods
through its API. Application state, sensor math, schemas, data, and evaluation
stay in the application repository.

The checked-in simulator is planar and generates sensor detections rather than
camera images or lidar point clouds. It provides the complete example for the
engine: GPS position, IMU acceleration and rotation, camera direction, and
lidar range and direction.

Run the reference experiments to study the full system. Use
[`fusion-tracking`](docs/TRACKING_ENGINE.md) from another repository to build a
different tracker.

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

Development plans are in [docs/ROADMAP.md](docs/ROADMAP.md).
External comparisons are documented in [docs/GTSAM.md](docs/GTSAM.md) and
[docs/STONE_SOUP.md](docs/STONE_SOUP.md).

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
