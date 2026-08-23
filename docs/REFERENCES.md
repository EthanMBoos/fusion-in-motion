# References and neighboring tools

Fusion in Motion is not a vehicle-physics or rendering simulator. It creates
small, controlled measurement streams for localization and object-tracking
experiments.

The [Kalman filter](https://doi.org/10.1115/1.3662552),
[*Probabilistic Robotics*](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/),
and Bar-Shalom, Li, and Kirubarajan's
[*Estimation with Applications to Tracking and Navigation*](https://doi.org/10.1002/0471221279)
cover the estimation and tracking foundations used here. Solà's
[quaternion error-state notes](https://arxiv.org/abs/1711.02508) are a useful
reference when the planar ego filter becomes a 3D filter.

[robot_localization](https://github.com/cra-ros-pkg/robot_localization) is a
useful comparison for GPS/IMU-style robot state estimation. It reinforces the
boundary between localization measurements and raw perception. For tracking,
[Stone Soup](https://github.com/dstl/Stone-Soup) shows a broad set of motion,
measurement, association, and track-management components that this repository
should not try to reproduce all at once.

[Gazebo](https://gazebosim.org/), [CARLA](https://carla.org/), and
[AirSim](https://github.com/microsoft/AirSim) are better fits when an experiment
needs vehicle dynamics, scene rendering, or raw camera/lidar data. Fusion in
Motion should consume representative outputs from tools like those rather than
grow into another general simulator.

The first analytic models make several limits explicit: GPS is a noisy local
position rather than satellite geometry; the camera reports object direction
rather than pixels; lidar reports object range and direction rather than a
point cloud; association is supplied; and the filters are planar. These limits
keep the first experiment readable while leaving the public messages ready for
3D positions, elevation, and sensor mounts.

Timing and consistency are not bookkeeping. Out-of-sequence measurements,
innovation checks, covariance coverage, and paired multi-seed comparisons are
part of evaluating whether a filter or tracker is behaving honestly. They are
reported separately for vehicle localization and object tracking.
