# GTSAM EKF

The GTSAM backend runs the same six-state planar EKF as the Rust `imu_bias`
backend. Fusion in Motion still owns measurement order, timing, scoring,
tracking, and Rerun output. Only the vehicle filter changes.

```text
IMU and GPS in arrival order
        |
        v
Rust estimator runner
        |
        v
cxx bridge -> GTSAM 4.2.2 EKF
        |
        v
vehicle estimate -> scoring, Rerun, and object tracker
```

## Set up GTSAM

Install CMake and Boost once if they are not already available:

```sh
brew install cmake boost                 # macOS
sudo apt install cmake libboost-all-dev  # Ubuntu
```

Then run:

```sh
./scripts/setup-gtsam.sh
```

The script clones GTSAM 4.2.2 into the ignored `third_party/gtsam` directory,
builds it with two compiler jobs, and installs it there. The Rust build finds
that copy automatically.

Install Fusion in Motion with the optional backend:

```sh
cargo install --path crates/fusion --features gtsam
```

The normal workspace build does not compile the C++ adapter. `GTSAM_ROOT` can
still point to another GTSAM 4.2.2 install if needed.

## Run the comparison

```sh
fusion run experiments/imu_bias.yaml
fusion run experiments/gtsam_ekf.yaml
fusion compare runs/run001 runs/run002
```

The two experiment files use the same seed, sensor settings, and filter tuning.
Their vehicle and bias results should be nearly identical. Each report names
the estimator, and the GTSAM report records the linked GTSAM version.

## What crosses the bridge

Rust passes plain numbers for one IMU or GPS update. C++ returns the six state
values, the row-major 6x6 covariance, and the GPS gate decision. GTSAM, Eigen,
Protobuf, MCAP, and Rerun objects do not cross the bridge.

The state order is:

```text
x, y, yaw, forward speed, gyro bias, accelerometer bias
```

The C++ adapter owns the GTSAM filter for the run. Rust checks every returned
state and covariance before it reaches scoring or tracking. C++ exceptions
come back as normal Rust errors with the sensor and both timestamps.

GTSAM's standard 3D IMU factor does not match the current planar IMU, which
only reports yaw rate and forward acceleration. The adapter therefore defines
the same planar motion and GPS factors used by the Rust EKF. It uses GTSAM's
`ExtendedKalmanFilter` to solve them.

## Checks

Run the direct math and bridge checks with:

```sh
cargo test -p fusion-in-motion --features gtsam estimator::
```

They check:

- a hand-calculated propagation step;
- conversion of a C++ exception into a Rust error; and
- Rust/GTSAM state, covariance, residual, and gate parity after each input.

Run the GTSAM experiment through normal scoring, tracking, and Rerun generation
with:

```sh
cargo test -p fusion-in-motion --features gtsam --test experiment_regressions
```

## Later

An iSAM2 backend can reuse this bridge and planar model. It should be a separate
algorithm because it keeps a graph and may revise earlier states. Once the
simulator has a 3D IMU, use GTSAM's normal pose, velocity, bias, GPS, and IMU
preintegration types instead of extending the planar factor.

References: [GTSAM 4.2.2](https://github.com/borglab/gtsam/releases/tag/4.2.2),
[`ExtendedKalmanFilter`](https://borglab.github.io/gtsam/extendedkalmanfilter/),
and [`cxx`](https://cxx.rs/).
