# Simulator API

The messages in `proto/fusion.proto` match the planar simulator:

```text
ImuSample + GpsFix -> EgoStateEstimate
CameraFrame + LidarScan + EgoStateEstimate -> ObjectTrack
EgoTruthState + ObjectTruthState -> scoring and display only
```

Camera and lidar detections do not contain object IDs. The baseline predicts
each existing track, rejects implausible detection-to-track pairs, and finds a
one-to-one assignment for the remaining pairs. Lidar can create a track because
it measures range. Camera can update a track but cannot create one from a
single direction measurement. `ObjectTrack.track_id` belongs to the tracker.
Truth object IDs are read only by scoring and display code.

Positions use the fixed local world frame. Vehicle x points forward, y points
left, and yaw is positive counterclockwise. All sensors are at the vehicle
origin in this version.

Each sensor record has a measurement time and an arrival time. They are equal
in the starter. `examples/timing.yaml` adds latency and uses the measurement
time when replaying delayed data. Each lidar detection has its own measurement
time because one scan can collect returns at different times.

The starter vehicle covariance is a row-major 4×4 matrix ordered as:

```text
x, y, yaw, forward speed
```

The bias experiment adds gyro and accelerometer bias, producing a 6×6 matrix:

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
