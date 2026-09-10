# Simulator API

This page describes the planar simulator API. `fusion-tracking` contains the
whole-tracker API, the point-target reference manager, and no serialization
dependency. A scan carries scan-wide sensor context even when it has no
detections. See
[`TRACKING_ENGINE.md`](TRACKING_ENGINE.md) to use it from another repository.

The messages in `proto/fusion.proto` match the planar simulator:

```text
ImuSample + GpsFix -> EgoStateEstimate
CameraFrame + LidarScan + EgoStateEstimate -> ObjectTrackFrame
EgoTruthState + ObjectTruthState -> scoring and display only
```

Camera and lidar detections do not contain object IDs. The baseline predicts
each existing track, rejects implausible detection-to-track pairs, and finds a
one-to-one assignment for the remaining pairs. Lidar can create a track because
it measures range. Camera can update a track but cannot create one from a
single direction measurement. `ObjectTrack.track_id` belongs to the tracker.
Truth object IDs are read only by scoring and display code.

The built-in `TrackManager` is one `Tracker` backend. Another backend can own
its complete state machine and return the same timestamped track reports
without using the built-in association or lifecycle components.

`ObjectTrackFrame` contains the full tracker output at one estimate time. An
empty frame means the tracker ran and produced no confirmed tracks. Individual
tracks inherit the frame's estimate and availability times.

Positions use the fixed local world frame. Vehicle x points forward, y points
left, and yaw is positive counterclockwise. All sensors are at the vehicle
origin in this version.

Each sensor record has a measurement time and an arrival time. They are equal
in the starter. `experiments/timing.yaml` delays GPS and can process the
completed GPS/IMU run in measurement-time order. This is an offline reference,
not an online rewind/replay filter. Each lidar detection has its own measurement
time because one scan can collect returns at different times.

`ego_estimator.algorithm` selects `basic`, `imu_bias`, or `gtsam_ekf_planar`.
The GTSAM backend requires the optional build in [GTSAM.md](GTSAM.md). The
basic EKF assumes the IMU is correct and uses a row-major 4×4 covariance
ordered as:

```text
x, y, yaw, forward speed
```

The two bias-aware EKFs estimate gyro and accelerometer bias and use a 6×6
matrix:

```text
x, y, yaw, forward speed, gyro bias, accelerometer bias
```

Object-track covariance is a row-major 4×4 matrix ordered as:

```text
x, y, velocity x, velocity y
```

`measurements.mcap` contains only data available to the estimators. `truth.mcap`
contains vehicle truth, object truth, and simulated IMU bias. Normal estimation
does not read truth. The purple truth-ego tracker is run separately as a scoring
control.

The tracker history JSON under `reports/baseline` records predictions,
association decisions, updates, and track lifecycle events from the built-in
tracker. It is not part of the external result API.

External track CSV uses these columns:

```text
estimate_time_ns,available_time_ns,track_id,x_m,y_m,vx_mps,vy_mps
```

Write one row per track. To record a frame with no tracks, write its two times
and leave the remaining fields empty.
