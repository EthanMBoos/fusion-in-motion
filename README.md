# Fusion in Motion

Fusion in Motion is a small simulator for two connected problems:

```text
GPS + IMU -> estimate the vehicle
camera + lidar + vehicle estimate -> track other objects
```

Camera and lidar never correct the vehicle-position filter. They detect other
objects. The object tracker uses the vehicle estimate to separate vehicle
motion from object motion.

The first demo is a planar ground vehicle with one stationary object and one
moving object. Its messages already carry 3D positions, orientations, bearings,
and velocities so later demos can add drones and camera-only tracking without
replacing the API. The included filters are still 2D.

## Run it

Install the command as described in [docs/INSTALL.md](docs/INSTALL.md), then:

```sh
fusion run examples/initial.yaml --view
```

Runs go to `runs/run001`, `runs/run002`, and so on. The dashboard shows the
vehicle truth and GPS/IMU estimate, camera and lidar detections, object truth,
and two copies of the object tracker:

- orange uses the estimated vehicle pose;
- purple uses the true vehicle pose as a comparison.

Both trackers receive the same detections. The gap between them is object-track
error caused by an imperfect vehicle estimate.

Start with [docs/START_HERE.md](docs/START_HERE.md). It gives a short sequence
of YAML edits that show GPS drift correction, vehicle error entering object
tracks, and what camera and lidar each add.

## What is simulated

The camera produces directions to detected objects. It does not produce fake
distance. The lidar produces range and direction at a particular time within a
scan. GPS produces noisy positions in the local `world_enu` frame. The IMU
produces angular rate and acceleration with noise and drifting bias.

These are analytic measurements, not rendered images or point clouds. Object
association is supplied in the first tracker so the demo stays focused on
localization and sensor fusion.

The estimator-visible measurements are stored separately from hidden truth in
each run. See [docs/API.md](docs/API.md) for the data boundary and
[docs/EXPERIMENTS.md](docs/EXPERIMENTS.md) for run and sweep details.

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
