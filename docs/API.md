# Simulator API

The API separates the vehicle from the objects it sees.

```text
ImuSample + GpsFix -> EgoStateEstimate
CameraFrame + LidarScan + EgoStateEstimate -> ObjectTrack
EgoTruthState + ObjectTruthState -> scoring only
```

All public messages are defined in `proto/fusion.proto`. Positions and
velocities use three components, orientations use xyzw quaternions, and camera
and lidar detections include elevation. The first implementation uses only
x/y/yaw and planar velocity.

## Time

Every sensor record contains two times that matter:

- `reported_stamp_ns` is when the measurement describes the world;
- `receipt_time_ns` is when the application receives it.

Camera frames have one acquisition time. Each lidar detection also has an
offset within its scan. The baseline can apply delayed data at measurement time
and replay the later work, or apply it at arrival time for a timing comparison.

## Frames

`world_enu` is fixed at the start of a run. The vehicle body frame uses x
forward, y left, and z up. Each sensor has a mount pose from its sensor frame to
the body frame. The planar implementation accepts x/y/yaw mounts; the message
types do not impose that limit.

GPS fixes are already expressed in the local-world frame. Camera and lidar
detections are expressed in their sensor frames. The object tracker combines
them with the vehicle pose. That dependency is one way: an object detection
cannot update `EgoStateEstimate`.

## Baseline state layouts

`EGO_STATE_MODEL_PLANAR` uses this covariance order:

```text
x, y, yaw, forward speed, gyro-z bias, accelerometer-x bias
```

`OBJECT_STATE_MODEL_PLANAR_CONSTANT_VELOCITY` uses:

```text
object x, object y, object velocity x, object velocity y
```

The model enum must be checked before reading a covariance matrix. A later 3D
estimator will use a new model value and state layout.

## Object identity

The first lesson supplies `association_key`, which tells the tracker which
detections belong together. `detection_id` identifies one measurement. The
hidden truth file maps that detection to the simulated object for scoring.

Supplying association avoids mixing track management into the first fusion
exercise. A later experiment can remove that help and add false detections,
gating, and track creation/deletion.

## Stored files

`measurements.mcap` contains sensor calibration, IMU, GPS, camera, and lidar
records. `truth.mcap` contains vehicle truth, object truth, sensor-effect truth,
and detection-to-object truth. Normal localization and tracking read only the
measurement file. Truth is opened afterward for scoring and for the explicitly
labeled truth-ego tracker.
