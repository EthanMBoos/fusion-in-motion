# GPS/IMU positioning and camera/lidar object tracking

## The problem

The current demo uses the camera and lidar to locate the robot. Both sensors
observe fixed landmarks whose map positions and identities are known. Those
observations update the same EKF that processes the IMU.

That is not the intended system.

The robot should use GPS and IMU to estimate its own position. Camera and lidar
should detect and track other objects. The object tracker needs the robot's
position so it can tell the difference between robot motion and object motion,
but object detections should not correct the robot's position.

The intended data flow is:

```text
GPS + IMU ───────────────> robot position filter ─────> robot state
                                                            │
camera + lidar ──────────> object detections                 │
             object detections + robot state ───────────────> object tracker

hidden robot and object truth ──────────────────────────────> scoring only
```

There is no GPS sensor, moving-object truth, object detection API, or object
track API in the repository today. The current camera and lidar messages refer
to known landmarks and feed the robot-position EKF.

## What each sensor should do

| Sensor | Job |
| --- | --- |
| IMU | Follow short-term changes in the robot's speed and direction. It runs quickly but drifts over time. |
| GPS | Pull the robot position back toward the correct place. It is slower and noisier than the IMU but does not drift in the same way. |
| Camera | Detect the direction of an object. The first analytic camera model does not provide distance. |
| Lidar | Detect the distance and direction of an object. A scan can span several robot poses. |

The robot-position filter consumes only GPS and IMU. The object tracker
consumes camera and lidar detections plus the robot state produced by that
filter.

The dependency is one way. Lidar deskew uses the robot's motion to place all
returns at one time. Camera and lidar detections use the robot pose to move
measurements from a sensor frame into a fixed local frame. Neither operation
feeds a correction back into the robot-position filter.

## Local position is enough

The first version does not need latitude, longitude, or an Earth-wide frame.
Simulated GPS can produce noisy x/y positions in the existing `world_enu`
frame, which starts at the robot's initial location.

| Frame | Meaning | Initial use |
| --- | --- | --- |
| Body | Moves and turns with the robot. x is forward, y is left, and z is up. | IMU data and nearby object detections |
| Local world | Fixed when the run starts. | Robot paths and object tracks |
| Global map | Shared geographic or site-wide coordinates. | Later work when runs, maps, or robots must agree |

For immediate collision avoidance, the range and direction of an object in the
body frame may be enough. Tracking the object across time still requires the
robot's motion. A stationary object appears to move in the sensor view whenever
the robot moves or turns.

To place a lidar detection into the local world frame, the tracker applies the
robot position and heading:

```text
object position in local frame
    = robot position
    + robot heading applied to the object position seen by the sensor
```

Any error in the robot state enters the object track. Heading error is
especially visible at long range. A one-degree heading error moves an object
at 20 meters sideways by about 35 centimeters.

## Robot-position filter

The current planar EKF state can remain:

```text
[x, y, yaw, forward speed, gyroscope z bias, accelerometer x bias]
```

The IMU continues to move this state forward. The new GPS observation updates
x and y. Motion over time allows those GPS corrections to improve speed,
heading, and IMU bias estimates through the EKF covariance.

Camera-bearing and lidar-range updates must be removed from this filter. The
known landmark map is no longer part of the initial positioning problem.

GPS needs the same timing fields as the other sensors: measurement time,
arrival time, rate, latency, and noise. Fixed-lag replay remains useful because
a late GPS position should correct the state at the time it was measured, then
carry that correction forward.

The initial GPS API should contain a header, a position in `world_enu`, and the
reported position uncertainty. Raw latitude, longitude, satellite geometry,
and multipath can wait until an experiment needs them.

## Object simulation and tracking

The world needs objects with their own paths. At minimum, each hidden object
truth record needs an object ID, time, local position, and local velocity.

Camera and lidar should produce object detections rather than landmark
observations. A camera detection needs its acquisition time and direction from
the camera. A lidar detection needs its acquisition time, distance, and
direction from the lidar. Hidden truth records connect detections to simulated
objects for scoring.

The first tracking lesson should supply the object association. This keeps the
lesson about sensor fusion and robot-motion error. A later lesson can remove
that help and cover association mistakes, missed detections, and false
detections.

The first object tracker can use a planar constant-velocity state:

```text
[object x, object y, object velocity x, object velocity y]
```

An object track record needs the track ID, estimate time, position, velocity,
status, frame, and covariance. Object truth stays hidden from the tracker.

The tracker should support two robot-state sources without changing the sensor
data. One source uses perfect robot truth. The other uses the GPS/IMU estimate.
That comparison isolates the object-tracking error caused by the robot-position
filter.

## First experiments

### 1. See what GPS adds to the IMU

Run the robot-position filter with GPS and IMU, then run the same IMU data with
GPS disabled. The IMU-only position should drift. GPS updates should keep the
robot near the correct path.

This replaces the current use of fixed camera and lidar landmarks. It should be
the first student experiment because it shows why the sensors are fused.

### 2. Learn IMU bias

Keep the current bias truth, estimates, plots, RMSE, coverage, and multi-seed
scoring. Replace camera/lidar landmark corrections with GPS corrections.

The lesson should still answer whether the filter learns the initial sensor
error, follows a drifting error, and reports believable uncertainty.

### 3. Show robot error entering an object track

Simulate one moving object and one stationary object. Generate one camera and
lidar detection stream. Run it through two otherwise identical trackers:

```text
tracker A: detections + true robot motion
tracker B: detections + GPS/IMU robot estimate
```

The difference between their errors is the cost of imperfect robot positioning.
The stationary object makes false motion easy to see. The moving object shows
how robot and object motion can be confused.

### 4. Camera and lidar together

The camera provides a good direction but no distance in the first analytic
model. Lidar provides distance and direction at a lower rate. The experiment
should show what happens with camera only, lidar only, and both sensors using
the same robot-state input.

### 5. Sensor timing

Camera latency and lidar scan duration should affect object tracks, not the
robot-position EKF. A delayed camera detection must be applied at its
measurement time. Lidar returns must be corrected for robot motion during the
scan before they update an object track.

## Dashboard

The existing Rerun setup, saved timelines, and screenshot workflow are useful.
The panels need to reflect the new separation between robot positioning and
object tracking.

The main map should show robot truth, robot estimate, object truth, and object
tracks in the fixed local frame. The sensor views should remain centered on the
robot and show current camera and lidar detections.

The robot plots should show position error, heading error, IMU bias, and GPS
updates. The object plots should show object position error and velocity error.
For the first tracking experiment, the same plot should compare the tracker
using true robot motion with the tracker using estimated robot motion.

A useful turn checkpoint should place robot heading error beside sideways
object error. This makes the relationship visible instead of leaving it as a
formula in the documentation.

Uncertainty comes after the basic error comparison is clear. When enabled, the
object tracker must account for robot-position uncertainty. Treating the robot
pose as exact will make object tracks look more certain than the inputs justify.

## Evaluation

Robot-position metrics remain separate from object-track metrics. The existing
position error, heading error, availability, covariance checks, and multi-seed
summaries remain useful for the GPS/IMU filter.

Object scoring should report error in two forms. Relative error compares the
object with truth as seen from the robot. Local-frame error compares the full
object track with object truth in `world_enu`. The gap between those results
shows how much robot-position error entered the track.

The first object report should include position RMSE, velocity RMSE, final
error, matched track samples, and track availability. Covariance coverage can
be added once the tracker carries both detection uncertainty and robot-position
uncertainty correctly.

Every comparison must reuse the same truth, detections, timestamps, and random
seed. Only the robot-state source or tracker setting should change.

## Recent commit audit

This review covers the 14 commits after `19e9868` (`Release v0.1.0`). Seven can
stay, six contain useful work that must move to the right part of the system,
and one roadmap commit should be replaced.

| Commit | Current work | Decision |
| --- | --- | --- |
| `14f9ab8` | Invalid camera/lidar EKF update counts | Rework. Keep update diagnostics, but report GPS updates for the robot filter and detection/track updates separately. |
| `e762000` | Lidar deskew uncertainty note | Rework. Deskew remains important, but it belongs before object tracking and must not update robot position. |
| `b62e50a` | IMU noise drives EKF process noise | Keep. This is required for GPS/IMU positioning. |
| `dd0f6a9` | Evaluation and sweep semantics | Keep. The same rules apply to robot and object estimates. |
| `2fe8085` | Planar API documentation | Rework. Document GPS/IMU positioning and object-detection frames instead of landmark localization. |
| `05fb444` | Delayed camera updates, state replay, and lidar deskew | Rework. Use state replay for delayed GPS; use camera timing and lidar deskew in the object path. |
| `8026eed` | Starter lidar timing settings | Rework. Keep the timing values but judge their effect on object tracks. |
| `727662d` | Covariance consistency evaluation | Keep. Apply it to GPS/IMU positioning and later to object tracks. |
| `8d6d3d5` | Dashboard readability | Keep. Replace landmark panels with object panels. |
| `acc7df0` | Visual verification workflow | Keep. Multiple time checkpoints are still required. |
| `48393a5` | Radar removal | Keep. The intended perception sensors are camera and lidar. |
| `2cd9797` | Starter guide and CLI workflow | Rework. Keep the commands and run layout; replace the lesson sequence. |
| `7df3849` | Numbered run folders | Keep. It is unrelated to sensor roles. |
| `231e65e` | Sequenced localization and SLAM demos | Replace. The roadmap should lead from GPS/IMU positioning into camera/lidar object tracking. |

The larger architecture problem began in v0.1.0, where camera and lidar were
first wired to known landmarks. The recent commits mostly added timing,
evaluation, and teaching around that existing choice.

## Current uncommitted bias work

Most of the uncommitted bias implementation remains useful. Keep the typed IMU
bias truth, typed bias estimates, exact timestamp matching, bias RMSE, final
error, covariance coverage, sweep summaries, dashboard plots, and external
estimate fields.

The lesson and baseline run must change. GPS should provide the outside position
information that lets the EKF correct IMU drift and learn bias. Camera and lidar
must no longer update those states.

The current bias exercise scenario and screenshots should be regenerated after
GPS is added. Results produced by camera/lidar landmark localization should not
be presented as the intended baseline.

## Code changes

The cleanup does not need backward compatibility. The known-landmark path can
be removed instead of preserved beside the new design.

### Positioning

Add `GpsFix` to the Protobuf API, `GpsConfig` to scenario YAML, GPS generation
to the sensor layer, and GPS records to measurement bundles. Replace camera and
lidar update calls in the baseline EKF with GPS position updates. Keep IMU
propagation, fixed-lag history, estimator timing reports, state covariance, and
bias states.

### Perception

Replace `Landmark`, `LandmarkMap`, `CameraFeature.landmark_id`, and
`LidarReturn.landmark_id` in the initial path with object truth and detection
types. Keep camera bearing noise, lidar range and bearing noise, detection
probability, acquisition time, latency, and lidar scan duration.

Move lidar deskew out of the robot-position estimator. It should consume an
ego-pose history and produce corrected lidar detections for the object tracker.

### Tracking

Add a small planar object tracker with known association first. Save its output
beside the robot estimate rather than overloading `StateEstimate`. Add object
evaluation and Rerun logging as separate modules.

### Documentation

Rewrite the README so it states the two jobs plainly. Rewrite `START_HERE.md`
around GPS+IMU positioning, then camera/lidar object tracking. Update the bias
lesson to use GPS. Replace the demo order in `DEMOS.md`. Update the literature
review because it currently says target tracking is outside the repository's
scope.

## Suggested implementation order

1. Change the README and API description so the intended split is fixed before
   more code is added.
2. Add GPS truth-free measurements and feed them into the robot EKF.
3. Remove camera, lidar, and known landmarks from the robot EKF.
4. Rerun the positioning and bias experiments with GPS+IMU.
5. Add moving-object truth and camera/lidar object detections.
6. Add the first known-association object tracker.
7. Add the true-ego versus estimated-ego comparison and dashboard panels.
8. Move the timing experiments onto GPS latency and object-detection timing.

## Acceptance checks

The architecture is corrected when changing camera or lidar noise cannot change
the robot-position estimate. Disabling both sensors must leave GPS/IMU
positioning unchanged.

Disabling GPS should produce visible IMU drift. Restoring GPS should reduce that
drift and allow the bias states to improve.

The object tracker must never read robot truth in its normal mode. The
true-robot mode exists only as a comparison baseline and must be labeled in its
output.

The same camera and lidar detections must feed both tracker modes. Their object
error difference must therefore come from robot-state error, not new random
measurements.

At least one test must prove that camera and lidar records are never passed to
the robot-position EKF. Another must prove that changing a robot heading
estimate changes a local-frame object track in the expected direction.

The dashboard must show the GPS/IMU robot result and the camera/lidar object
result as separate outputs. A student should be able to say which sensor affects
which estimate without reading the Rust code.
