# Demo plan

[`EXPERIMENTS.md`](EXPERIMENTS.md) lists what runs today. This is the intended
order as more demos are added. **Ready now** means YAML and dashboard work.
**Core work** means Rust changes.

Reviewed: Stone Soup 1.9.1
[tutorials](https://stonesoup.readthedocs.io/en/stable/auto_tutorials/index.html),
examples, and [source](https://github.com/dstl/Stone-Soup/commit/ec032a9f7ba73c9cc34d1c4f7824af8cab8fc0f1).
Once a demo has a YAML file, put its setup and expected result there and shorten
its entry here.

Comparisons use the same truth and measurements. Random studies use paired
seeds.

## GPS and IMU — ready now

1. Add a short straight drive with clean IMU and noisy GPS. Show the GPS fixes,
   estimate, and position error.
2. [`imu_bias.yaml`](../experiments/imu_bias.yaml): learn fixed and drifting IMU
   bias.
3. [`outliers.yaml`](../experiments/outliers.yaml): reject bad GPS fixes without
   rejecting useful ones.
4. [`timing.yaml`](../experiments/timing.yaml): delayed GPS in arrival order and
   offline measurement-time order.

Reference: [Stone Soup Kalman filter tutorial](https://stonesoup.readthedocs.io/en/stable/auto_tutorials/01_KalmanFilterTutorial.html).

## Camera and lidar object tracking

Use truth ego for demos 1–8. Add GPS/IMU position error in demo 9.

### 1. Predict and update one track — ready now

[`tracker_update.yaml`](../experiments/tracker_update.yaml) shows the prediction,
correction, error, covariance, and NIS for one lidar track.

References: [Kalman filter](https://stonesoup.readthedocs.io/en/stable/auto_tutorials/01_KalmanFilterTutorial.html)
and [EKF](https://stonesoup.readthedocs.io/en/stable/auto_tutorials/02_ExtendedKalmanFilterTutorial.html).

### 2. Show what camera and lidar measure — ready now

Simplify [`perception.yaml`](../experiments/perception.yaml) to perfect
detection, immediate confirmation, and no deletion. Compare:

- camera direction with no range;
- lidar distance and direction at each scan; and
- lidar initialization with camera updates between scans.

The current tracker cannot start an absolute-position track from camera alone.
Several bearings from a moving camera can provide range later. Camera-only runs
should show rays and no world-position track. Never seed the range from truth.

Reference: [Stone Soup bearing-only example](https://stonesoup.readthedocs.io/en/stable/auto_examples/filters/bearing_only_example.html).

### 3. Break the motion model — core work

Make the object accelerate or turn while the tracker assumes constant
velocity. Measure peak error during the maneuver and recovery time afterward.
Then run an acceleration or turn model against the same detections.

Reference: [Stone Soup multi-filter example](https://stonesoup.readthedocs.io/en/stable/auto_examples/filters/Multi_Tracker_Example.html).

### 4. Gate clutter around one track — core work

Add false lidar returns and a short outage around one established track. Stop
unmatched detections from starting tracks with an explicit initiation setting.
Compare a loose and tight gate.

Use the tracker history to inspect each candidate's NIS, gate result, selected
detection, rejected updates, and missed updates. `rejected_updates` does not
include pairs removed during assignment gating.

Reference: [Stone Soup single-target clutter tutorial](https://stonesoup.readthedocs.io/en/stable/auto_tutorials/05_DataAssociation-Clutter.html).

### 5. Assign detections at a crossing — core work

Simplify [`association.yaml`](../experiments/association.yaml) to lidar only,
perfect detection, immediate confirmation, and no deletion. Add a greedy
nearest-neighbor option and compare it with the current global assignment on
the same detections. Show the chosen links and costs through the crossing.

Use the time-local truth matching when reporting identity switches. Add missed
detections and false returns only after the clean crossing works.

References: [crossing tutorial](https://stonesoup.readthedocs.io/en/stable/auto_tutorials/06_DataAssociation-MultiTargetTutorial.html),
[greedy assignment](https://github.com/dstl/Stone-Soup/blob/main/stonesoup/dataassociator/neighbour.py#L58-L98),
and [global assignment](https://github.com/dstl/Stone-Soup/blob/main/stonesoup/dataassociator/neighbour.py#L189-L305).

### 6. Create and delete tracks — core work

Start with lidar only. Include an object entering and leaving view, a few false
returns at the same location, a short outage the track survives, and a longer
outage followed by re-entry. Re-entry after deletion creates a new track ID.

Use the recorded tentative, confirmed, missed, and deleted events to report
confirmation delay, false-track time, deletion delay, continuity through the
short outage, and new-track delay after re-entry.

Choose and test the lifecycle order first. Stone Soup uses update, delete, then
initiate in its [tracker loop](https://github.com/dstl/Stone-Soup/blob/main/stonesoup/tracker/simple.py#L175-L222)
and waits for several observations in its
[multi-measurement initiator](https://github.com/dstl/Stone-Soup/blob/main/stonesoup/initiator/simple.py#L233-L309).

### 7. Compare delayed object detections offline — ready now

Run the same case with no delay, arrival-order processing, and the current
offline measurement-time reorder. Show both timestamps, track error, and the
reordered-detection count.

Reference: [Stone Soup fixed-lag example](https://stonesoup.readthedocs.io/en/stable/auto_examples/oosm/example_simple_oosm.html).

### 8. Rewind and replay delayed detections online — core work

Save filter, association, and lifecycle history. When a late detection arrives,
rerun from the saved state and replace the affected outputs. Report the replayed
interval, correction size, revised outputs, discarded events, and output delay.

### 9. Combine vehicle and object estimation — ready now

Move [`initial.yaml`](../experiments/initial.yaml) to the end after the smaller
demos exist. Run the same detections with truth ego and GPS/IMU ego. Compare
object error, association, lifecycle, and track count. Camera and lidar settings
must not change the vehicle estimate.

Reference: [Stone Soup combined tutorial](https://stonesoup.readthedocs.io/en/stable/auto_tutorials/10_Simulation_%26_Tracking_Components.html).

## Measures to add — core work

| Measure | First use |
| --- | --- |
| Position and velocity error; covariance | One-track filter |
| 95% uncertainty coverage over several seeds | One-track sweep |
| Gated pairs, accepted updates, and missed updates | Clutter |
| Identity switches and track fragments | Crossing |
| Confirmation, deletion, false-track, and new-track delays | Lifecycle |
| GOSPA localization, missed-object, and false-object parts | Multi-target clutter or capstone |
| Processing time and output delay | Algorithm and timing comparisons |

GOSPA results must include the cutoff, order, alpha, units, and time
aggregation. References: [metrics example](https://stonesoup.readthedocs.io/en/stable/auto_examples/metrics/Metrics.html),
[GOSPA](https://github.com/dstl/Stone-Soup/blob/main/stonesoup/metricgenerator/ospametric.py),
and [CLEAR MOT](https://github.com/dstl/Stone-Soup/blob/main/stonesoup/metricgenerator/clearmotmetrics.py).

## Add after a baseline fails — core work

- Compare nearest-neighbor with PDA on the one-target clutter data. Report
  association probability, missed updates, error, consistency, and runtime.
  [PDA tutorial](https://stonesoup.readthedocs.io/en/stable/auto_tutorials/07_PDATutorial.html)
- Add JPDA if global assignment fails in the crossing study. Report identity
  switches, fragments, GOSPA, and runtime.
  [JPDA tutorial](https://stonesoup.readthedocs.io/en/stable/auto_tutorials/08_JPDATutorial.html)
- Add a UKF for a nonlinear case that separates it from the EKF.
  [UKF tutorial](https://stonesoup.readthedocs.io/en/stable/auto_tutorials/03_UnscentedKalmanFilterTutorial.html)
- Add a particle filter for a non-Gaussian result with more than one likely
  state. Compare it on the same prior and measurements, then vary particle
  count and measure error, consistency, resampling, runtime, and memory.
  [Particle-filter tutorial](https://stonesoup.readthedocs.io/en/stable/auto_tutorials/04_ParticleFilter.html)
- Add offline smoothing to the same forward-filter result. Compare filtered and
  smoothed error. The final states should match. The smoother uses future
  measurements. [Kalman smoother](https://github.com/dstl/Stone-Soup/blob/main/stonesoup/smoother/kalman.py)

## 3D and recorded data — core work

The build steps are in [`ROADMAP.md`](ROADMAP.md). Stone Soup references:

- [3D multi-target example](https://stonesoup.readthedocs.io/en/stable/auto_examples/simulation/MTT_3D_Platform.html):
  reuse the scenario structure, but score x, y, and z. Its displayed SIAP
  measures omit z.
- [Video-processing demo](https://stonesoup.readthedocs.io/en/stable/auto_demos/Video_Processing.html):
  use the reader, detector, and tracker boundary. Record the sequence, frame
  range, truth source, coordinate and timestamp conversions, adapter version,
  and input checksum. Keep fixed-detection results separate from full
  detector-to-track results.

## Hold for later

MHT, EHM, multi-frame assignment, loopy belief propagation, PMHT, GM-PHD,
expected-likelihood particle tracking, track-to-track fusion, track stitching,
extended-object tracking, road constraints, classification, distributed
fusion, and sensor management. Radar models, static-landmark localization, and
Stone Soup's plotting code are outside this plan.

## Core implementation todo

### Simulator and baseline comparisons

- [ ] Generate false camera and lidar detections.
- [ ] Add turning and accelerating object truth.
- [ ] Compare greedy and global assignment on the same detections.
- [ ] Compare constant-velocity and maneuvering motion models on the same
  detections.

### Later core work

- [ ] Initialize range from several camera bearings and known camera motion.
- [ ] Add causal fixed-history rewind and replay for delayed measurements.
