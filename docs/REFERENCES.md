# Simulator and benchmark review

Reviewed September 2026.

## Bottom line

Fusion in Motion sits between a Kalman-filter notebook and a renderer such as
CARLA or Gazebo. It isolates estimation behavior:

```text
GPS + IMU -> vehicle state
camera + lidar + vehicle state -> object tracks
```

The planar starter is the right first demo. A 3D filter adds gravity,
roll and pitch, three-axis sensor errors, camera projection, and harder-to-read
covariance without changing the first lesson. Add 3D as a later drone track,
after the planar path.

The baseline now uses unlabeled camera and lidar detections. It gates possible
matches, makes a one-to-one global nearest-neighbor assignment, and manages
track birth, confirmation, coasting, and deletion. The next tracking work is to
score identity continuity and add false detections and harder occlusions.

The camera/lidar study uses one object so association cannot hide the sensor
effect. Slow, intermittent lidar starts the metric track and supplies distance.
Camera bearings update direction between scans. The association experiment
adds a second object separately.

There is no single public benchmark for this exact two-stage system. Validate
it in three ways:

1. deterministic and multi-seed simulation tests for the math;
2. focused scenarios for failure modes and teaching; and
3. separate real-data replays for GPS/IMU localization and object tracking.

## Scope

The core covers:

- planar vehicle and object motion;
- sensor rate, noise, bias, field of view, missed observations, and time;
- GPS/IMU vehicle estimation;
- camera/lidar object estimation, association, and track life cycle;
- truth excluded from algorithm input;
- repeatable scenarios, sweeps, scoring, and a dashboard that explains a run.

Image formation, lidar ray tracing, tire dynamics, traffic behavior, weather,
and photorealism come from external simulators or recorded data.

## Simulation techniques

| Technique | What it is good for | What it cannot establish | Place here |
| --- | --- | --- | --- |
| Closed-form measurements | Filter equations, observability, noise, timing, and repeatable sweeps | Performance on pixels, point clouds, or real scenes | Core of this repo |
| Kinematic scenario simulation | Controlled vehicle and target maneuvers | Tire, suspension, actuator, and contact behavior | Core until dynamics become the lesson |
| Physics simulation | Vehicle response, vibration, mounting motion, control loops | Photorealistic perception by itself | Gazebo or another upstream source |
| Ray casting and rendering | FOV, occlusion, point density, image and scan geometry | Real sensor artifacts without calibration against data | CARLA, Gazebo, or Isaac Sim upstream |
| Log replay | Real noise, clutter, missed detections, and scene complexity | Counterfactual scenarios outside the recording | Required external validation layer |
| Monte Carlo simulation | Statistical error, consistency, and sensitivity | Realism of an incorrect model | Core evaluation method |
| Software/hardware in the loop | Scheduling, transport, compute limits, drivers, and flight controllers | Algorithm understanding at low setup cost | Later integration work |

Start with analytic measurements. Replay the same algorithm on real logs. Use
high-fidelity simulation only when the test needs it.

## Neighboring tools

| Tool | What it sets the standard for | What to borrow | Role here |
| --- | --- | --- | --- |
| [Stone Soup 1.9](https://stonesoup.readthedocs.io/) | Composable tracking models, predictors, updaters, association, initiators, deleters, metrics, and teaching examples | One concept per example and explicit tracker parts | Curriculum and algorithm reference |
| [MATLAB Sensor Fusion and Tracking Toolbox](https://www.mathworks.com/help/fusion/) | Polished scenario-to-sensor-to-tracker examples and quantitative tracker analysis | Focused scenarios, coverage plots, track life-cycle lessons | Example and dashboard reference |
| [robot_localization](https://github.com/cra-ros-pkg/robot_localization) | Practical asynchronous robot state estimation and ROS frame/message conventions | Clear separation of localization inputs and perception; later ROS adapters | ROS integration reference |
| [GTSAM](https://gtsam.org/) | Factor graphs, IMU preintegration, batch optimization, and fixed-lag smoothing in C++ and Python | A later 3D GPS/IMU comparison between filtering and smoothing | External estimator; its standard GPS/IMU state is 3D |
| [OpenVINS evaluation tools](https://docs.openvins.com/evaluation.html) | ATE, RPE, RMSE, NEES, timing, and multi-run estimator analysis | Error plus uncertainty plots and trajectory-scale evaluation | Evaluation reference |
| [CARLA 0.9.16](https://carla.org/2025/09/16/release-0.9.16/) and [ScenarioRunner](https://scenario-runner.readthedocs.io/) | Driving scenes, traffic actors, RGB/depth/segmentation cameras, ray-cast lidar, GNSS/IMU, synchronous stepping, and OpenSCENARIO workflows | An optional adapter when occlusion, raw perception, or traffic matters | Raw driving-scene source |
| [Gazebo Jetty/Harmonic](https://gazebosim.org/docs/latest/releases/) | Robot physics, models, sensor mounts, ROS integration, IMU, NavSat, cameras, and lidar | An optional source for robot or drone dynamics and mounted sensor data | Robot and drone dynamics source |
| [NVIDIA Isaac Sim 6.0](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/overview/release_notes.html) | RTX camera/lidar simulation, synthetic data, GPU robotics workflows, SIL and HIL | A future raw-perception or learned-model validation path | GPU raw-perception source |
| [Project AirSim](https://github.com/iamaisim/ProjectAirSim) | Drone and ground-vehicle dynamics, a fast non-rendered runtime, an Unreal rendering host, and PX4/ArduPilot SIL and HIL | Its split between fast algorithm tests and rendered integration tests | Revisit for 3D; its [open-source 1.0 release](https://github.com/iamaisim/ProjectAirSim/releases/tag/v1.0.0) is new |
| [AirSim](https://github.com/microsoft/AirSim) | Historically important drone/car simulation with cameras, lidar, GPS/IMU, and flight-controller integration | Its explicit sensor rate, startup delay, and latency configuration | Historical reference; no longer actively developed |

High fidelity does not remove timing problems. CARLA's
[sensor documentation](https://carla.readthedocs.io/en/latest/ref_sensors/)
describes ray-cast lidar collecting a rotation interval while the simulated
physics is held fixed, and its GPU cameras can arrive several frames late. This
repo models those effects with separate measurement and arrival times and an
explicit lidar scan time.

## External data and algorithms

There are three separate paths: replace an estimator or tracker, replay
recorded detections, or turn raw sensor data into detections. The first has
basic support today.

### Replace an estimator or tracker

An external Python or C++ algorithm can read a generated measurement bundle and
return final estimates:

```text
measurements.mcap -> external algorithm -> estimate or track CSV -> fusion score
```

Truth stays in `truth.mcap` and out of the algorithm. `fusion score ego` and
`fusion score tracks` write the submitted result into the run and produce
metrics. The dashboard does not discover external results yet.

Add a small Python example that decodes `measurements.mcap`, runs a basic
GPS/IMU estimator, and writes the supported CSV. The
[MCAP Python Protobuf reader](https://mcap.dev/docs/python/mcap-protobuf-apidoc/mcap_protobuf.reader)
provides the file interface.

### Replay recorded detections

The object tracker should also accept detections produced outside this
simulator:

```text
ROS bag, MCAP, or dataset
        -> format adapter
        -> CameraFrame and LidarScan measurements
        -> Rust object tracker
        -> tracks, metrics, and dashboard
```

An MCAP file is only a container. A ROS recording or dataset will use different
topics, message schemas, coordinate frames, and timestamps from
`fusion.proto`. An adapter must translate those details into this API.

For the current planar tracker, a camera detection contains bearing and bearing
uncertainty. A lidar detection contains range, bearing, and both uncertainties.
The adapter applies sensor calibration, converts observations into the vehicle
frame, keeps measurement and arrival time, and does not attach object names. If
a recording has no useful arrival timestamp, set arrival time equal to
measurement time unless latency is part of the test.

Tracking also needs the vehicle state. It can come from GPS and IMU measurements
in the same recording or from an imported external vehicle estimate. Reference
vehicle and object states belong in a separate truth file used only for scoring
and display.

The current CLI cannot import a detection recording or rerun the Rust tracker
from one. The first replay integration should use a small MCAP containing
already-computed camera and lidar detections. This tests the adapter, timing,
tracking, scoring, and dashboard before raw perception is added.

### Raw camera and lidar frontends

Raw images and point clouds add a perception step before the same replay path:

```text
camera frames -> detector -> bearing observations
lidar points  -> detector -> range and bearing observations
observations + vehicle state -> Rust object tracker
```

[OpenCV](https://docs.opencv.org/4.x/d6/d0f/group__dnn.html) can handle image
input, calibration, pixel geometry, and model inference.
[TorchVision](https://docs.pytorch.org/tutorials/intermediate/torchvision_tutorial.html)
provides reference object-detection models and training examples, and
[ONNX Runtime](https://onnxruntime.ai/inference) provides a lighter
model-independent inference option. The simulator API is detector-independent.

The frontend owns decoding, calibration, and model inference. Camera
calibration turns an image detection into a direction; it does not supply object
distance. Detector confidence is not covariance and should not be copied into
the measurement-uncertainty field.

Detections are unlabeled. Truth IDs remain in the truth recording and evaluator;
the tracker creates its own IDs.

Keep two output choices. A perception frontend emits observations for the Rust
tracker. A complete external tracker emits track CSV for `fusion score tracks`.
Build the observation path first because it keeps association and track life
cycle inside this repo.

Implementation order:

1. Add the small Python MCAP reader and external GPS/IMU estimator example.
2. Import a small recorded-detection MCAP and run the Rust tracker on it.
3. Add adapters for selected ROS or dataset recordings.
4. Add a camera detector, keeping both frame time and result-arrival time.
5. Add a lidar point-cloud frontend when raw lidar becomes a lesson.
6. Add GTSAM for the 3D GPS/IMU smoother comparison.

### GTSAM

Run GTSAM as an external estimator when the 3D GPS/IMU layer is added. Its
standard [IMU factor](https://borglab.github.io/gtsam/imufactor/) estimates a 3D pose,
velocity, and IMU bias from preintegrated measurements, while its
[GPS factors](https://borglab.github.io/gtsam/gpsfactor/) constrain position.
Compare three estimators on the same measurement bundle:

```text
3D GPS + IMU -> online EKF
             -> fixed-lag GTSAM smoother
             -> offline full-batch GTSAM smoother
```

Use delayed GPS, bias drift, and GPS outages to expose the differences. Label
each result as online, fixed-lag, or offline; a batch solver can revise the
entire trajectory.

Start with the Python bindings and export the existing score format. A planar
version would need [custom factors](https://borglab.github.io/gtsam/customfactor/)
and is not part of the starter.

## What public benchmarks actually measure

### Vehicle localization

Common measures are position and orientation RMSE, final and maximum error,
absolute trajectory error (ATE), and relative pose error (RPE). KITTI computes
translation and rotation error over subsequences of different path lengths,
which prevents one endpoint from standing in for the whole run.

ATE often aligns the estimated path to truth first. Use that for SLAM or visual
odometry, where the arbitrary starting frame should not count as error. Do not
use it as the primary GPS score: alignment can erase the global offset the
estimator was meant to recover. Use direct world-frame position and yaw error,
with distance-binned RPE as a drift diagnostic.

An estimator must also report believable uncertainty. Use:

- innovation and normalized innovation squared (NIS) for each measurement
  type;
- normalized estimation error squared (NEES) where simulation truth and full
  covariance are available; and
- component error against the claimed uncertainty for dashboard diagnosis.

NIS and NEES are statistical tests. Draw conclusions from independent seeds
and chi-squared bounds, not from many correlated samples along one path.

### Object state estimation

For the current API, report per-object position and velocity RMSE, maximum and
final error, time coverage, innovation statistics, and covariance consistency.
Report both world-frame and vehicle-relative error. The truth-ego tracker is a
good control: the difference between truth-ego and estimated-ego tracking
shows how much vehicle error reached the object result.

### Multi-object tracking

Full tracking starts with unlabeled detections and includes missed detections,
false detections, association, track birth, confirmation, coasting, deletion,
and identity continuity.

The major benchmark families report:

| Metric | Question it answers | Use here |
| --- | --- | --- |
| Position/velocity RMSE | How accurate is a correctly matched state? | Use at every stage |
| GOSPA | How much error came from localization, missed objects, and false objects as a set? | Add with the clutter lesson |
| HOTA and its DetA/AssA/LocA parts | How well are detection, association, and localization balanced over a sequence? | Primary full-tracker teaching metric |
| MOTA/MOTP | What is the combined miss, false-positive, and identity-switch cost; how precise are matches? | Report for comparison, not alone |
| IDF1 and ID switches | Does an object retain the same identity? | Essential for crossings and occlusion |
| Track initialization delay, longest gap, fragments, mostly tracked/lost | Does the track exist when it is needed? | Best dashboard/report diagnostics for life-cycle lessons |

[KITTI tracking](https://www.cvlibs.net/datasets/kitti/eval_tracking.php)
now ranks by HOTA and also reports CLEAR MOT and track coverage. The
[nuScenes tracking benchmark](https://www.nuscenes.org/tracking) uses 3D
ground-plane center-distance matching and reports AMOTA/AMOTP plus false
positives, misses, identity switches, fragments, initialization duration, and
longest gap. [Waymo](https://waymo.com/open/) evaluates 2D and 3D tracking on
real sensor sequences. Their leaderboard numbers are not useful pass thresholds
for an analytic point-target simulator; their task definitions and error
breakdowns are useful.

### Detection

KITTI, nuScenes, Waymo, and Argoverse also benchmark the detector that turns an
image or point cloud into boxes, classes, and confidence scores. The current
simulator starts after that step. Report average precision only after adding
raw sensor data or recorded detector outputs.

## Relevant datasets

No one dataset is ideal for both halves of the repo.

| Dataset | Useful part | Limitation for this repo |
| --- | --- | --- |
| [KITTI raw](https://www.cvlibs.net/datasets/kitti/raw_data.php) | Synchronized camera, lidar, GPS/IMU, calibration, and 3D object tracklets in compact driving sequences | OXTS is already a high-grade integrated navigation system; benchmark tasks are mostly perception and visual/lidar odometry |
| [UrbanNav](https://github.com/IPNL-POLYU/UrbanNavDataset) | Raw GNSS, IMU, accurate reference, and difficult urban-canyon multipath/NLOS | Large and aimed at navigation; object annotations are not its main benchmark |
| [NCLT](https://deepblue.lib.umich.edu/data/concern/data_sets/h128nf37h) | Consumer GPS, IMU, wheel sensing, RTK reference, and repeated indoor/outdoor routes | Large and less focused than a curated UrbanNav segment |
| [Oxford RobotCar](https://robotcar-dataset.robots.ox.ac.uk/) | Repeated real routes, GPS/INS, cameras, lidar, weather, and RTK reference for selected runs | Very large and not organized as an object-tracking benchmark |
| [nuScenes](https://www.nuscenes.org/) | Calibrated 360-degree camera/lidar/radar scenes, ego poses, 3D boxes, track IDs, and official tracking evaluation | Keyframes are sparse for fast filter lessons; raw GPS/IMU localization is not the main task |
| [Waymo Open Dataset](https://waymo.com/open/) | Large real camera/lidar 2D and 3D perception/tracking benchmark with official code | Heavy data and tooling; not a beginner path |
| [Argoverse 2 Sensor Dataset](https://argoverse.github.io/user-guide/datasets/sensor.html) | 1,000 short scenes, 10 Hz lidar, nine cameras, calibrated ego poses, and 3D annotations | Best suited to later perception/tracking replay, not the first estimator demo |

Use UrbanNav or selected KITTI/Oxford data for GPS/IMU validation. Use a
nuScenes mini or small KITTI tracking subset for the first real object-tracking
adapter. The localization and tracking replays can use different datasets.

## What the current repo validates

| Capability | Current state | Supported conclusion |
| --- | --- | --- |
| GPS/IMU planar estimation | Present | Compare estimator accuracy, bias behavior, outlier handling, timing, and sensitivity under the stated model |
| Camera/lidar object updates | Present | Compare bearing/range fusion after association |
| Ego error propagation | Present through estimated-ego and truth-ego runs | Isolate the cost of vehicle pose error on object state |
| Statistical sweeps | Present | Compare configured variants across paired seeds |
| Multi-object association | Gated global nearest neighbor | Baseline association behavior; identity metrics still needed |
| Track life cycle | Birth, confirmation, coasting, and deletion present | Basic track management without clutter |
| Camera/lidar detector quality | Not present; measurements are analytic object observations | No AP or raw perception claim |
| Sensor realism | White noise, random-walk bias, delay, FOV, range, and simple misses | Model-level behavior, not device fidelity |
| 3D or drone tracking | Not present | Planar only |

## Validation stack for this repo

| Layer | Required checks |
| --- | --- |
| Simulator | Zero-noise geometry; exact rates and timestamps; measured noise and bias statistics; independent sensor random streams; camera/lidar changes never affect ego estimation. Match IMU rate scaling to the [Kalibr noise model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model). |
| Estimator | Error, coverage, NIS/NEES, uncertainty bounds, accepted and rejected measurements, gaps, failures, and runtime when relevant. Use paired seeds and report spread. Scenario-specific metrics define failure; there is no universal divergence threshold. |
| Controls | Truth versus estimated ego, perfect versus noisy observations, camera versus lidar versus both, and timestamp-naive versus timestamp-aware processing. |
| Model mismatch | Correlated GPS error, target maneuvers, long delay, unmodeled bias, clutter, and crossings. Each lesson needs at least one mismatch between the simulator and estimator assumptions. |
| Real replay | GPS/IMU for navigation; camera/lidar detections and ego pose for tracking; truth held separately. Recorded detections are enough to test association and tracking. |

Use the [SDFormat sensor terms](https://sdformat.org/spec/1.11/sensor/) when
adding sample noise, startup bias, slow bias, correlation time, or quantization.

## Recommended demo sequence

The current starter is the quick tour. Each later demo isolates one new idea.

| Order | Demo | Main knobs | Evidence | State |
| ---: | --- | --- | --- | --- |
| 0 | Measurement geometry | vehicle/object pose, camera bearing, lidar range | body/world sketch and zero-noise values | New, very small |
| 1 | Full planar tour | existing starter rates and noise | dashboard shows both pipelines and truth-ego control | Present |
| 2 | GPS rate and noise | GPS rate, position noise, GPS on/off | vehicle RMSE and orange-versus-purple object error | Present through starter edits and sweep |
| 3 | IMU bias | initial bias, bias walk, estimation on/off | bias truth/estimate/bounds, vehicle error, multi-seed coverage | Present |
| 4 | Bad GPS fixes | outlier rate/size, gate | innovation, accepted/rejected fixes, vehicle error | Present |
| 5 | Delayed GPS reference | GPS latency, measurement-time order on/off | GPS age and turn error | Present; offline |
| 6 | GPS interruption and urban error | outage interval, position drift, correlated bias, recovery | drift growth, recovery, global versus relative object error | New, highest-priority localization demo |
| 7 | Lidar track from known ego | target speed, lidar rate/noise, process noise | position/velocity error and covariance | New focused tracker base |
| 8 | Camera plus lidar | camera/lidar enable flags | lidar initializes range; camera sharpens direction between scans | Present |
| 9 | Maneuvering target | target acceleration/turn, CV process noise, later CA/CTRV/IMM | lag, NIS, position/velocity error around maneuver | New |
| 10 | Lost detections and track life | FOV, occlusion interval, detection probability, confirmation/deletion rules | initialization delay, coasting, gaps, deletion/reacquisition | New |
| 11 | Association and clutter | object spacing, crossing angle, false detections, gate, assignment method | HOTA parts, GOSPA, ID switches, track timeline | Crossing baseline present; clutter and metrics remain |
| 12 | Ego uncertainty cost | GPS condition, object range, vehicle turn | truth-ego control, estimated-ego result, global and relative error | Current mechanism; add focused scenario |
| 13 | Mount and calibration error | camera/lidar position and yaw offset, time offset | biased object track with unchanged vehicle estimate | New |
| 14 | Real log replay | dataset sequence and detector source | same reports plus dataset-specific metrics | New integration layer |
| 15 | 3D drone, camera only for objects | drone maneuver, object depth, FOV, bearing noise | angular error, depth convergence, covariance, observability | New 3D track |

### Implementation notes

| Area | Decision |
| --- | --- |
| Urban GPS | Start with an outage, then add slowly varying position error and honest versus overconfident covariance. |
| Camera and lidar | Use one object and truth ego first. Lidar supplies range; camera updates direction between slower lidar scans. Camera-only range needs useful relative motion. |
| Association | Start with unlabeled detections, Mahalanobis gating, and one-to-one global nearest-neighbor assignment. Add JPDA or multiple-hypothesis tracking later. |
| Ego uncertainty | Propagate ego covariance through the sensor-to-world transform. Per-object propagation is still an approximation because all objects share the same ego error. |
| 3D camera tracking | Image-plane tracking and metric world tracking are separate tasks. Compare poor-parallax and sideways-motion cases. Add the full 3D state, IMU, camera, truth, and covariance API together. [UAVDT](https://sites.google.com/view/daweidu/projects/uavdt) and [VisDrone](https://github.com/VisDrone/VisDrone-Dataset) cover image-plane tracking. |

## Reading list

### Estimation and inertial sensing

- R. E. Kalman, ["A New Approach to Linear Filtering and Prediction
  Problems"](https://doi.org/10.1115/1.3662552), 1960 — the base recursion.
- S. Thrun, W. Burgard, and D. Fox,
  [*Probabilistic Robotics*](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/)
  — accessible probabilistic robotics foundation.
- Y. Bar-Shalom, X. R. Li, and T. Kirubarajan,
  [*Estimation with Applications to Tracking and Navigation*](https://doi.org/10.1002/0471221279)
  — estimation, consistency, tracking, and navigation in one reference.
- J. Solà,
  ["Quaternion kinematics for the error-state Kalman filter"](https://arxiv.org/abs/1711.02508),
  2017 — use when the 3D estimator is implemented.
- P. Groves,
  [*Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*](https://us.artechhouse.com/Principles-of-GNSS-Inertial-and-Multisensor-Integrated-Navigation-Systems-Second-Edition-P1609.aspx)
  — deeper GNSS/INS engineering reference.
- [PX4 EKF2 tuning guide](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf)
  — production examples of bias states, innovation checks, delayed fusion, and
  estimator health.

### Tracking

- Y. Bar-Shalom and T. Fortmann,
  [*Tracking and Data Association*](https://books.google.com/books?id=B_FQAAAAMAAJ),
  1988 — the classic association foundation.
- A. Bewley et al.,
  ["Simple Online and Realtime Tracking"](https://arxiv.org/abs/1602.00763),
  2016 — a clear baseline combining a Kalman model with one-to-one assignment.
- X. R. Li and V. Jilkov,
  ["Survey of Maneuvering Target Tracking. Part I: Dynamic Models"](https://doi.org/10.1109/TAES.2003.1261132),
  2003 — CV, acceleration, and turn-model background.
- H. Blom and Y. Bar-Shalom,
  ["The Interacting Multiple Model Algorithm for Systems with Markovian Switching Coefficients"](https://doi.org/10.1109/9.1299),
  1988 — the later IMM lesson.
- V. Aidala and S. Hammel,
  ["Utilization of Modified Polar Coordinates for Bearings-Only Tracking"](https://doi.org/10.1109/TAC.1983.1103230),
  1983 — why bearing-only initialization and coordinates need care.
- S. Nardone and V. Aidala,
  ["Observability Criteria for Bearings-Only Target Motion Analysis"](https://doi.org/10.1109/TAES.1981.309141),
  1981 — why the observer's motion determines whether range can be learned.
- J. Montiel, J. Civera, and A. Davison,
  ["Unified Inverse Depth Parametrization for Monocular SLAM"](https://doi.org/10.15607/RSS.2006.II.011),
  2006 — a useful representation for initially unknown monocular depth, even
  though this repo is tracking objects rather than landmarks.
- Y. Bar-Shalom,
  ["Update with out-of-sequence measurements in tracking: exact solution"](https://doi.org/10.1109/TAES.2002.1039398),
  2002 — background for the delayed-measurement lesson.

### Evaluation

- K. Bernardin and R. Stiefelhagen,
  ["Evaluating Multiple Object Tracking Performance: The CLEAR MOT Metrics"](https://doi.org/10.1155/2008/246309),
  2008.
- J. Luiten et al.,
  ["HOTA: A Higher Order Metric for Evaluating Multi-object Tracking"](https://arxiv.org/abs/2009.07736),
  2020 — separates detection, association, and localization quality.
- A. S. Rahmathullah, Á. F. García-Fernández, and L. Svensson,
  ["Generalized optimal sub-pattern assignment metric"](https://doi.org/10.23919/ICIF.2017.8009645),
  2017 — a principled set error for localization, missed targets, and false
  targets.
- E. Ristani et al.,
  ["Performance Measures and a Data Set for Multi-Target, Multi-Camera Tracking"](https://arxiv.org/abs/1609.01775),
  2016 — the identity precision, recall, and IDF1 metrics.
