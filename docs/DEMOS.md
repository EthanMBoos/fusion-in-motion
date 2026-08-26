# Fusion in Motion demos

Fusion in Motion should grow through a sequence of demonstrations. Each demo
adds one platform capability, reproduces a recognizable result from the
state-estimation literature, and provides the foundation for the next demo.

The project already has the necessary core: deterministic planar motion,
analytic IMU/camera/lidar/radar measurements, explicit acquisition and receipt
times, per-return lidar timing, hidden truth, MCAP/Protobuf bundles, a baseline
EKF, parameter sweeps, scoring, and Rerun visualization.

The demos below turn that core into evidence that the tool is useful.

## 1. Accurate is not the same as consistent

Run the baseline EKF with three measurement-noise configurations:

- matched to the simulated sensor noise;
- deliberately underestimated; and
- deliberately overestimated.

Use identical generated measurements and paired random seeds for all three.
Add covariance validation, confidence coverage, and ANEES for the state
components with matching truth. Plot estimation error with the reported
uncertainty bounds.

### What it demonstrates

Two estimates can have similar trajectory error while one is badly
overconfident. A visually good path is not sufficient evidence that a filter is
working correctly.

### Literature match

- OpenVINS uses trajectory error, NEES, timing, and resource use together in its
  [evaluation guidance](https://docs.openvins.com/evaluation.html).
- Huang, Mourikis, and Roumeliotis connect EKF inconsistency to incorrect
  linearized observability in [Observability-based Rules for Designing
  Consistent EKF SLAM
  Estimators](https://doi.org/10.1177/0278364909353640).

### What this adds

- covariance and consistency scoring;
- paired multi-estimator, multi-seed reports; and
- uncertainty plots reusable by every later demo.

With trustworthy uncertainty metrics in place, the next demo can show what
happens when otherwise valid measurements arrive late.

## 2. Delayed measurements need state history

The measurement bundle is already ordered by receipt time and preserves the
original acquisition time. The current EKF applies delayed camera, lidar, and
radar observations to its current state.

Compare three policies:

1. apply the measurement immediately to the current state;
2. discard measurements older than a configured limit; and
3. restore the state at acquisition time, apply the update, and repropagate to
   the present inside a fixed-lag window.

Sweep sensor latency and latency jitter while keeping all sensor draws paired.
Plot trajectory error, consistency, measurement age, discarded measurements,
revisions, and processing cost.

### What it demonstrates

Applying an old observation to the current state creates a modeling error. A
fixed-lag method recovers the information while the observation remains inside
its history window. Dropping old data avoids the bad update but loses useful
information.

### Literature match

- Bar-Shalom's out-of-sequence measurement work establishes delayed-data
  processing as part of the estimator rather than transport bookkeeping.
- [iSAM2](https://doi.org/10.1177/0278364911430419) and systems such as
  [Graph-MSF](https://github.com/leggedrobotics/graph_msf) show why revisable
  smoothing windows coexist with high-rate propagated estimates.

### What this adds

- a bounded state and IMU history;
- fixed-lag repropagation and revised estimates;
- explicit causal, fixed-lag, and offline estimator labels; and
- latency and revision metrics.

The same state-history mechanism can now correct measurements acquired across
an interval rather than at one timestamp.

## 3. One lidar scan is many poses

Fusion in Motion already generates lidar returns with acquisition offsets
inside a scan. Compare:

- an uncompensated estimator that treats the complete scan as instantaneous;
  and
- a time-aware estimator that evaluates each return at its acquisition time.

Sweep platform speed, yaw rate, lidar rate, and scan duration. In Rerun, show
the platform motion during the scan and the compensated and uncompensated
return endpoints.

### What it demonstrates

The two methods agree during slow motion and short scans. During fast turns or
long scans, the uncompensated geometry bends or smears and the state estimate
degrades.

### Literature match

[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM#prepare-lidar-data) requires a
timestamp for each point so that IMU motion can deskew a rotating lidar scan.
Continuous-time estimation work by [Furgale, Barfoot, and
Sibley](https://doi.org/10.1109/ICRA.2012.6225005) provides the broader basis for
querying platform state at heterogeneous acquisition times.

### What this adds

- a reusable per-element acquisition-time interface;
- interpolation across the state history; and
- visualization of interval observations.

This timing model is also the basis for rolling-shutter cameras and recorded
video playback later in the sequence.

## 4. Run the same evidence through mature estimators

Use one saved measurement bundle with:

- the built-in Rust EKF;
- a Python estimator implemented with
  [navlie](https://github.com/decargroup/navlie); and
- a C++ Pose2 factor graph implemented with
  [GTSAM](https://github.com/borglab/gtsam).

Start with known-map localization using the existing IMU and landmark
range/bearing observations. The adapters consume `measurements.mcap`, never
open `truth.mcap`, and emit the existing estimate format with covariance and
timing. Record dependency versions, tuning, initialization, estimator access
class, runtime, and memory.

The first GTSAM result can be offline. Then use the fixed-lag work from Demo 2
to make a properly labeled online comparison.

### What it demonstrates

The useful product is not another estimator implementation. It is the ability
to generate, replay, score, and visualize the same experiment across estimator
families without rebuilding the experiment around each library.

### Literature match

- GTSAM's [factor-graph tutorial](https://gtsam.org/tutorials/intro.html)
  includes Pose2 localization, landmark SLAM, and incremental smoothing.
- [Factor Graphs for Robot
  Perception](https://doi.org/10.1561/2300000043) provides the underlying
  formulation.
- navlie provides EKF, iterated and sigma-point filters, batch estimation, and
  Monte Carlo analysis behind one model interface.

### What this adds

- a reference external-estimator contract;
- direct C++ and Python integration examples;
- side-by-side reports and Rerun views; and
- comparable causal, fixed-lag, and offline results without conflating them.

The factor-graph adapter makes it possible to reproduce the most recognizable
robust-estimation failure: one bad constraint distorting the solution.

## 5. One false constraint can break the solution

Add deterministic fault schedules for:

- a landmark identity swap;
- a range or bearing bias burst; and
- a false relative-pose or loop-closure constraint.

Keep the fault type, onset, duration, and magnitude in hidden observation truth.
Compare ordinary Gaussian updates with innovation gating, a robust loss such as
Huber, and one graph method such as switchable constraints.

Report faulty measurements accepted, healthy measurements rejected, peak and
integrated error, time outside bounds, recovery time, and terminal failures.
Show the bad constraint and the recovered trajectory in Rerun.

### What it demonstrates

One plausible false association can dominate ordinary least squares. Robust
methods can isolate it, but aggressive rejection also has a cost during normal
operation.

### Literature match

- [Switchable Constraints for Robust Pose Graph
  SLAM](https://doi.org/10.1109/IROS.2012.6385590) makes constraint validity part
  of the optimization.
- [Max-mixtures](https://doi.org/10.1109/ICRA.2012.6224699) and
  [dynamic covariance scaling](https://doi.org/10.1109/ICRA.2013.6630557)
  provide alternative robust graph formulations.

### What this adds

- scheduled and hidden sensor faults;
- estimator gating or weighting decisions in the result record; and
- failure and recovery metrics rather than final RMSE alone.

With comparison, timing, and failure behavior established synthetically, the
next step is to pass a compatible real dataset through the same path.

## 6. Repeat the planar experiment on real data

Import one robot from the [UTIAS Multi-Robot Cooperative Localization and
Mapping dataset](https://doi.org/10.1177/0278364911398404). It provides planar
odometry, range-bearing landmark observations, a known landmark map, and
motion-capture ground truth. That is a closer match to the current platform
than jumping directly to EuRoC, KITTI, or a raw lidar dataset.

Convert visible measurements into `measurements.mcap` and keep robot pose truth
in `truth.mcap`. Preserve source files, checksums, frame and time conversions,
excluded intervals, and dataset citation in the run manifest. Run the built-in
and external estimators without dataset-specific evaluation code.

### What it demonstrates

The bundle, estimator adapters, metrics, and visualization are not tied to the
analytic generator. Differences between synthetic and real results expose
model mismatch instead of hiding it behind separate tooling.

### Literature match

The [UTIAS MR.CLAM
dataset](https://www.stars.utias.utoronto.ca/datasets/mrclam/index.html) was
published specifically for known-map cooperative localization and SLAM with
odometry and range-bearing observations.

### What this adds

- a provenance-aware real-data importer;
- real sensor timing and noise behavior; and
- one direct synthetic-to-real comparison on the existing planar problem.

The data-import boundary can now be extended from compact observations to real
camera frames without adding an in-core renderer.

## 7. Drive simulator playback with real video

Use calibrated recorded video as the photoreal camera source. Fusion in Motion
controls the playback timeline: it selects frames at simulated acquisition
times, applies configured rate, latency, jitter, and drops, and records the
source frame and timestamp. A real vision front end processes those frames and
returns tracks, poses, or other derived observations to the estimator bundle.

Compare the same video and front end under increasing processing latency and
frame-drop schedules. Show the video frame, acquisition and delivery times,
front-end result, and fused trajectory together in Rerun.

The recording must have a defensible pose relationship: its original calibrated
trajectory, a registered replay path, or an explicitly throughput-only mode.
Arbitrary video paired with unrelated synthetic truth cannot support an
accuracy claim.

### What it demonstrates

Photoreal input can be added through playback rather than by building a renderer
or vehicle-physics engine. The same experiment can separate front-end failure,
delivery delay, and fusion behavior.

### Literature match

- Visual-inertial systems such as [OpenVINS](https://docs.openvins.com/) make
  feature tracking, camera calibration, IMU timing, and estimator consistency
  separate but connected concerns.
- The [EuRoC MAV dataset](https://projects.asl.ethz.ch/datasets/euroc-mav/)
  provides a later canonical calibrated stereo/IMU playback target once the
  platform moves beyond planar observations.

### What this adds

- timestamped external media playback;
- a raw-frame-to-derived-observation boundary;
- front-end latency and drop experiments; and
- a path to photoreal demonstrations without owning image synthesis.

## End state

After these demos, Fusion in Motion has a clear claim: it makes controlled
sensor-fusion results easier to reproduce across synthetic data, real logs,
real video, and multiple estimation libraries while keeping time, truth access,
uncertainty, and evaluation explicit.

Only then is it useful to choose the next direction: SE(3) visual-inertial
estimation, runtime pipeline benchmarking, or multi-robot and robust-association
experiments. The choice should follow whichever completed demos prove most
useful, not a general feature checklist.

For the broader technical background and source survey, see the [literature and
design review](LITERATURE_REVIEW.md).
