# Simulation profiles

`platform_profile` selects the motion, frame, measurement, and estimate API.
Experiment settings such as speed, rate, noise, and latency do not change that
API.

## `planar_sensor_fusion`

This is the only implemented profile.

| Part | API |
| --- | --- |
| World frame | Right-handed ENU: x east, y north, z up |
| Body frame | Right-handed: x forward, y left, z up |
| Motion | x, y, and yaw; z, roll, pitch, and vertical velocity are zero |
| Sensor frames | Equal to the body frame; no sensor extrinsics |
| Association | Camera and lidar observations include known landmark IDs |

Positive yaw turns left. Poses use normalized xyzw quaternions even though only
yaw changes.

| Sensor | Generated measurement | Baseline use |
| --- | --- | --- |
| IMU | Three-axis body angular rate and specific force | Gyro z and accelerometer x |
| Camera | Landmark azimuth and elevation | Azimuth only |
| Lidar | Horizontal range and azimuth | Range and azimuth |

Landmark height affects camera elevation. It does not affect lidar, whose range
is measured in the x-y plane. Accelerometer z includes the support force
opposing gravity, even though the baseline does not use that channel.

The baseline state and full covariance order are:

```text
[x, y, yaw, forward_speed, gyro_bias_z, accel_bias_x]
```

## Future SE(3) profile

SE(3) is not implemented. It can share record headers, timestamps, vectors, and
poses with the planar profile. Its API will need to define motion, gravity,
sensor extrinsics, measurement geometry, estimate state, covariance, and
evaluation. The planar 6x6 covariance will not be reused as an SE(3) covariance.
