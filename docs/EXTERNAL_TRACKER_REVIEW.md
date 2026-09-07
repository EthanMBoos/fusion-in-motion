# External tracker review

Can Fusion in Motion include a respected external tracker implementation, like
GTSAM provides for ego estimation?

Yes, but there is no single tracker library that fits every experiment. The
current simulator gives the tracker camera bearings and lidar range/bearing
measurements. Automotive trackers usually expect localized 2D or 3D objects.
Video trackers expect image boxes, scores, and sometimes pixels or appearance
features. Those are different tracking problems, not interchangeable input
formats.

The practical choices are:

- use [Stone Soup](https://github.com/dstl/Stone-Soup) as the independent
  reference for the tracker that exists now;
- run the official [ByteTrack](https://github.com/FoundationVision/ByteTrack)
  implementation after image-box detections exist, then consider
  [ByteTrack-cpp](https://github.com/Vertical-Beach/ByteTrack-cpp) or
  [Similari](https://github.com/insight-platform/Similari) as a linked backend;
  and
- use [Autoware's multi-object tracker](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_multi_object_tracker)
  later for detections expressed as Autoware 3D objects.

Do not start by adding a universal tracker plugin API. Run one direct comparison
at each measurement boundary.

Reviewed September 2026.

## What has to match

An external tracker should receive the same observations as the Rust tracker and
own all of these decisions:

```text
prediction
gating
association
state update
track creation and confirmation
miss handling and deletion
track IDs
```

Passing it associations already selected by Rust would only compare filters.
Changing a camera bearing into a made-up bounding box or position would not be a
fair tracker comparison.

Fusion in Motion currently has this boundary:

```text
camera: bearing + variance
lidar: range + bearing + variances
ego: world pose + covariance
    -> tracker
world track: x, y, vx, vy + covariance
```

`fusion score tracks` already imports external world tracks from CSV, scores
them, and saves the result. That is enough for an initial state comparison. It
cannot represent output frames with no tracks, so it will undercount misses. It
also does not add external tracks to the Rerun view. Those are the two required
fixes for a full comparison.

Covariance is optional unless the comparison checks filter uncertainty.
Lifecycle and association diagnostics can stay in backend-specific files.

## Shortlist

| Project | Normal input | Language / license | Fit | Use here |
| --- | --- | --- | --- | --- |
| [Stone Soup](https://github.com/dstl/Stone-Soup) | Configurable measurements and models, including nonlinear bearing/range | Python / MIT | Best match for the current simulator | First independent result comparison |
| [NRL Tracker Component Library](https://github.com/USNavalResearchLaboratory/TrackerComponentLibrary) | Components assembled into a tracker | MATLAB / mostly U.S. public domain; bundled files keep their own licenses | Strong mathematical reference, poor dependency | Check assignment, gating, filtering, and later JIPDA behavior |
| [García-Fernández MTT](https://github.com/Agarciafernandez/MTT) | Paper-specific multi-target models | MATLAB / BSD-2-Clause | Strong advanced algorithm reference | Reproduce later PMBM, PHD, and trajectory-filter cases |
| [Autoware multi-object tracker](https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_multi_object_tracker/) | Objects in a known frame with covariance, class, and shape | C++ / Apache-2.0 | Good industry tracker; wrong input for current camera bearings | Later external ROS 2 comparison with Autoware 3D objects |
| [Apollo perception](https://github.com/ApolloAuto/apollo/tree/master/modules/perception) | Segmented 3D objects in Apollo frames and messages | C++ / Apache-2.0 | Credible but tightly coupled to Apollo | Architecture and behavior reference only |
| [SPENCER people tracking](https://github.com/spencer-project/spencer_people_tracking) | Localized person detections with covariance | C++ / mostly BSD, ROS 1 | Useful IMM and lifecycle reference; old ROS boundary | Source for a maneuvering-target lesson |
| [Similari](https://github.com/insight-platform/Similari) | Boxes, points, and feature vectors | Rust with Python bindings / Apache-2.0 | Easy Rust dependency; does not accept the present nonlinear sensor models | Future Rust box or point tracker |
| [ByteTrack-cpp](https://github.com/Vertical-Beach/ByteTrack-cpp) | Image boxes, class, and confidence | C++17 / MIT | Small unofficial port of a recognized algorithm | Linked camera-box candidate after a wider parity check |
| [Official ByteTrack](https://github.com/FoundationVision/ByteTrack) | Image boxes and confidence | Python with deployment C++ / MIT | Author implementation and benchmark reference; large as a dependency | Check ByteTrack-cpp parity and cite results |
| [OC-SORT](https://github.com/noahcao/OC_SORT) | Image boxes and confidence | Python with contributed C++ / MIT | Good second motion-only video tracker | Run official Python first; audit a C++ port later |
| [Smorodov Multitarget-tracker](https://github.com/Smorodov/Multitarget-tracker) | Image points or boxes | C++ / Apache-2.0 | Mature embeddable library with many choices | Broader C++ camera-tracker comparisons |
| [Norfair](https://github.com/tryolabs/norfair) | Arbitrary image points and a custom distance | Python / BSD-3-Clause | Flexible and approachable; not covariance-driven sensor fusion | Camera and drone experiments |
| [BoxMOT](https://github.com/mikel-brostrom/boxmot) | Image boxes, frames, and optional appearance features | Python and C++ / AGPL-3.0 | Strong multi-algorithm harness; license is a poor fit for the MIT core | Separate benchmark tool |
| [NVIDIA DeepStream tracker](https://docs.nvidia.com/metropolis/deepstream/9.0/text/DS_plugin_gst-nvtracker.html) | Batched video boxes and sometimes image buffers | Proprietary C/C++ SDK and NVIDIA runtime | Real industry implementation; tied to NVIDIA and GStreamer | External recorded-video comparison |
| [CenterPoint](https://github.com/tianweiy/CenterPoint) | Detected 3D boxes | Python/CUDA / MIT | Useful public 3D detector and simple tracker | Later dataset benchmark, not a linked tracker |
| [btrack](https://github.com/quantumjot/btrack) | Point observations and domain-specific hypotheses | C++ core with Python API / MIT | Real reusable code, but built for cell tracking | Interop reference, not the first baseline |
| [MathWorks trackers](https://www.mathworks.com/help/fusion/multiple-object-tracking.html) | Configurable object detections and filter initialization | MATLAB / proprietary | Useful tracker API and behavior reference | Commercial comparison when a license is available |

## Best fit for the tracker we have now

### Stone Soup

Stone Soup is the strongest first comparison because its measurement models and
tracker pieces can represent the current problem without inventing data. It has
nonlinear filters, bearing and range-bearing models, moving sensor platforms,
gating, global nearest-neighbor assignment, PDA, JPDA, track initiation, and
deletion. Its [multi-target tutorial](https://stonesoup.readthedocs.io/en/stable/auto_tutorials/06_DataAssociation-MultiTargetTutorial.html)
shows the predictor, updater, association, and track loop separately. The
[sensor-platform example](https://stonesoup.readthedocs.io/en/latest/auto_examples/simulation/Sensor_Platform_Simulation.html)
and [3D platform example](https://stonesoup.readthedocs.io/en/stable/auto_examples/simulation/MTT_3D_Platform.html)
cover moving sensors and nonlinear observations.

Run Stone Soup as a separate reference program over the completed measurement
file. There is little value in embedding Python inside the Rust process for this
comparison.

Use two small checks. First seed one established track with the same state,
motion model, process noise, measurement covariance, gate, and timestamps. This
checks filter math; include covariance if uncertainty is under test. Then give
both complete trackers the same detections and let each one own initialization,
association, lifecycle, and IDs. Export the Stone Soup world tracks and score
them with `fusion score tracks`.

Start with truth ego, one object, lidar only, no missed detections, and no delay.
Add camera bearings next. Add clutter, crossings, and lifecycle cases as those
simulator features are built.

This does not prove that either implementation is correct by itself. Agreement
on hand-checkable cases, followed by cases that separate the algorithms, is
useful evidence.

### NRL and published RFS implementations

The [NRL Tracker Component Library](https://github.com/USNavalResearchLaboratory/TrackerComponentLibrary)
contains assignment, gating, dynamic estimation, coordinate conversion,
tracking measures, and complete sample trackers. Its
`demo2DIntegratedDataAssociation.m` builds a GNN-JIPDAF tracker from those
pieces. Some routines have C or C++ implementations, but the library is not a
standalone C++ tracker to link from Rust.

The [García-Fernández MTT repository](https://github.com/Agarciafernandez/MTT)
contains author implementations for PMBM, PMB, MBM, PHD, CPHD, trajectory
filters, out-of-sequence measurements, and GOSPA variants. These are useful
when a demo reaches one of those algorithms. They are paper implementations,
not a generic runtime backend.

Use both as numerical references for a specific lesson. Do not pull either into
the normal build.

This review did not find a mature general-purpose C++ PMBM, GLMB, or other
random-finite-set tracker that fits a GTSAM-style backend. The strongest public
implementations are mainly MATLAB code tied to particular papers.

### MathWorks

MathWorks supplies configurable
[`trackerGNN`](https://www.mathworks.com/help/fusion/ref/trackergnn-system-object.html),
[`trackerJPDA`](https://www.mathworks.com/help/fusion/ref/trackerjpda-system-object.html),
[`trackerTOMHT`](https://www.mathworks.com/help/fusion/ref/trackertomht-system-object.html),
and [`trackerPHD`](https://www.mathworks.com/help/fusion/ref/trackerphd-system-object.html)
implementations. Their detection, filter-initialization, lifecycle, assignment,
and out-of-sequence controls are useful API references. They require commercial
toolboxes, so they cannot be this project's public baseline.

## Industry world-space trackers

### Autoware

Autoware is the best public industry comparison found in this review. Its
tracker uses global assignment, class- and shape-dependent gates, several EKF
motion models, and explicit track lifecycle. The package has unit, performance,
simulation, and rosbag tests. Its current build still depends on ROS 2,
Autoware messages and utilities, transforms, odometry, diagnostics, Eigen, and
the muSSP assignment solver; the dependency list is visible in its
[CMake file](https://github.com/autowarefoundation/autoware_universe/blob/main/perception/autoware_multi_object_tracker/CMakeLists.txt).

Autoware consumes detections in a known coordinate frame and uses transforms and
odometry to track them. Its 3D object message carries position, covariance,
class, shape, and orientation. A lidar range/bearing observation can provide a
Cartesian point with propagated covariance, but not all of those object
properties. A single camera bearing cannot provide them either. Supplying fake
positions, shapes, or classes would weaken the comparison.

Wait until Fusion in Motion has a real localized-object input. Then run
Autoware externally in a pinned ROS 2 environment or container. Extracting its
tracker into a small C++ library would create a fork that this repo has to
maintain.

### Apollo and SPENCER

[Apollo's lidar tracker](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/lidar_tracking/lidar_tracking_component.cc)
uses segmented 3D objects, Apollo frame types, configuration protobufs, and its
runtime. The code is worth reading, but the adapter would have to reproduce too
much of Apollo's perception stack.

[SPENCER](https://github.com/spencer-project/spencer_people_tracking) combines
nearest-neighbor association with an interacting multiple-model filter for
people moving around a robot. It is a useful independent source for the future
maneuvering-target demo. It is tied to an older ROS 1 stack and is not a good
dependency.

## Camera and recorded-video trackers

These trackers need image detections. The current `CameraDetection` contains a
bearing and variance, not a box.

### ByteTrack-cpp

ByteTrack is a well-known published tracking-by-detection algorithm. The
[official repository](https://github.com/FoundationVision/ByteTrack) is the
algorithm and benchmark reference. It also includes detector and deployment
code that Fusion in Motion does not need.

[ByteTrack-cpp](https://github.com/Vertical-Beach/ByteTrack-cpp) extracts the
tracker into a small C++17 library. Its input is a rectangle, class, and
confidence. It depends on Eigen, uses CMake, has an MIT license, and includes
tests against the official ncnn C++ output. This is the closest future match to
the GTSAM integration: one pinned dependency, one narrow `cxx` bridge, and one
optional crate.

Caveats:

- the extraction is not maintained by the ByteTrack paper authors;
- its last visible update was in 2024 and it has no tagged releases;
- it advances by frame number rather than accepting a timestamp;
- it returns boxes and IDs, not world state or covariance.

The adapter must call it once for every camera frame, including empty frames.
Start with regular-rate frames because its frame-rate setting controls how long
lost tracks are kept. Its output needs image-space scoring rather than the
current meter-based track scoring.

The first image comparison also needs real box detections and box truth. Use a
short annotated recording rather than adding object sizes, camera projection,
visibility, and occlusion to the analytic simulator just to create boxes. Keep
low-confidence detections; discarding them before ByteTrack would remove the
main behavior being tested.

### Similari

[Similari](https://github.com/insight-platform/Similari) is easy to miss because
it is a Rust framework rather than a named paper implementation. It provides
SORT and VisualSORT trackers, track lifecycle, axis-aligned and rotated boxes,
2D point Kalman filters, feature-vector matching, Python bindings, benchmarks,
and a published crate. It is Apache-2.0 and can be used without a C++ bridge.

It is a good fit if the goal is to let Rust users build and inspect box or point
trackers. It is a weaker external authority than an official algorithm
implementation, and its ready-made trackers still do not consume the present
bearing/range likelihoods. A Similari integration would test a useful Rust
library, not independently validate the current geometric tracker.

### OC-SORT, Smorodov, Norfair, and BoxMOT

[OC-SORT](https://github.com/noahcao/OC_SORT) is a good second video algorithm
for missed observations, occlusion, and nonlinear image motion. Start with the
official Python implementation. Its contributed C++ path should be checked
against official output before it is linked.

[Smorodov's Multitarget-tracker](https://github.com/Smorodov/Multitarget-tracker)
is a mature C++ library with Hungarian and LAPJV assignment, linear and
unscented Kalman filters, constant-velocity and constant-acceleration models,
ByteTrack, and several visual tracking modes. It is a practical choice if one
C++ dependency should expose several camera trackers. That breadth also makes
it less clear as a single reference result.

[Norfair](https://github.com/tryolabs/norfair) accepts arbitrary image point
sets and custom distance functions. It is useful for quick camera or drone
experiments, but it does not preserve the covariance-aware nonlinear sensor
model used by the current tracker.

[BoxMOT](https://github.com/mikel-brostrom/boxmot) runs several modern trackers
behind a common box interface and includes evaluation and tuning support. Its
AGPL-3.0 license makes it a separate research tool rather than a dependency of
this MIT repository.

### DeepStream

[NVIDIA DeepStream](https://docs.nvidia.com/metropolis/deepstream/9.0/text/DS_plugin_gst-nvtracker.html)
provides IOU, NvSORT, NvDeepSORT, NvDCF, and other trackers through a shared
low-level interface. NVIDIA publishes the plugin source and API, not all of the
tracker implementations as a portable open-source library. This is a useful
industry comparison for recorded video, but it assumes DeepStream batches,
GStreamer, NVIDIA buffer types, Ubuntu or Jetson, and often CUDA/TensorRT.

Run it as a separate offline pipeline and import its result. Do not wrap the SDK
with `cxx` as the first video integration.

## 3D tracking repositories

[CenterPoint](https://github.com/tianweiy/CenterPoint) and
[SimpleTrack](https://github.com/tusen-ai/SimpleTrack) are useful public
references once a dataset or frontend produces 3D detection boxes. They are
tied to autonomous-driving datasets and detector output, not raw lidar returns.

[AB3DMOT](https://github.com/xinshuoweng/AB3DMOT) is widely cited but its
license restricts use to noncommercial research. Do not vendor it into this MIT
project.

These projects should be evaluated on the dataset representation they were
designed for. They do not help validate the current planar bearing/range
tracker.

## Repositories not worth integrating first

- [openMHT](https://github.com/SyllogismRXS/openmht) is a very small,
  lightly maintained C++ project with a fixed time step and narrow 2D point
  input. The algorithm name does not make it a strong reference.
- [khmot](https://github.com/r7vme/khmot) is an approachable C++
  Kalman/Hungarian tracker, but its own documentation calls it an MVP and notes
  covariance and process-noise limits.
- [multi_target_kf](https://github.com/mzahana/multi_target_kf) has clean small
  motion-model pieces, but little independent benchmark evidence and a ROS 2
  Cartesian-point boundary.
- [SORT](https://github.com/abewley/sort) and
  [Deep SORT](https://github.com/nwojke/deep_sort) are canonical teaching
  references, but their official repositories are GPL-3.0 and Python-based.
- [BoT-SORT](https://github.com/NirAharon/BoT-SORT), FairMOT, and Tracktor bring
  detector, appearance-network, and model-weight stacks. They compare a larger
  vision system rather than just tracker logic.
- OpenCV's individual object trackers do not provide multi-object association,
  initiation, deletion, or re-entry handling.

## Three kinds of integration

### External reference run

```text
measurements.mcap -> outside tracker -> common track result
                                      -> Rust scoring and Rerun
```

This is the right starting point for Stone Soup, NRL cases, published MATLAB
code, Autoware, and DeepStream. Fusion in Motion still owns the simulated or
recorded input, truth, scoring, and display. The outside project keeps its own
runtime.

### Linked optional backend

This matches the current GTSAM setup: one optional crate, a narrow Rust or C++
adapter, and a YAML algorithm choice. It only makes sense when the library has a
small callable API and uses the same input and output as the experiment.
ByteTrack-cpp and Similari meet the software requirement for future image-box
tracking. Autoware and Apollo do not. A ByteTrack-cpp bridge should pass plain
owned values, not its C++ `shared_ptr` or STL types.

### Full-system benchmark

Some repositories include detection, tracking, runtime, and hardware-specific
processing. Run those on recorded data and import the final tracks. Keep their
end-to-end result separate from a fixed-detection tracker comparison.

## Keep the data boundaries separate

Add new messages when the corresponding experiments arrive. Do not grow the
current bearing/range messages into a union of every possible tracker input.

### Current geometric tracking

```text
camera bearing or lidar range/bearing + ego pose
    -> world x/y position and velocity + covariance
```

Stone Soup can compare this path now.

### Image tracking

```text
sequence ID and camera ID
integer frame index
measurement and availability time
image width and height
box in pixels with one stated coordinate convention
detection confidence and class
optional relative reference to the recorded video
optional appearance vector and the model that produced it
    -> image box + track ID
```

ByteTrack-cpp, Similari, OC-SORT, Smorodov, and DeepStream belong here. The
video stays on disk; the measurement file carries frame identity, timestamps,
detections, and a portable reference to the source when pixels are needed.
Frames with no detections still need a record. State whether boxes use `xyxy`
or `xywh` and whether their upper bounds are inclusive or half-open.

### Localized object tracking

```text
measurement and availability time
2D or 3D position or box
coordinate frame and covariance
class and shape when actually provided
    -> world tracks
```

Autoware, Apollo, CenterPoint's tracker, and other automotive systems belong
here.

Image tracks and world tracks should not share one result type until there is a
real conversion between them. Do not require covariance from a tracker that
does not report it.

## How to judge a comparison

Using a known repository is not enough. Record:

- the exact upstream commit or release and license;
- the input sequence and checksum;
- every coordinate, unit, covariance order, and timestamp conversion;
- the tracker settings and initial state;
- whether the result used fixed detections or included a detector;
- output at each frame, including empty frames; and
- runtime separately from decoding, detector work, and Rerun recording.

For the present world tracker, keep the existing position error, misses, false
tracks, identity switches, fragments, and later GOSPA work. Stone Soup provides
independent CLEAR MOT and OSPA/GOSPA implementations.

For image tracks, use [TrackEval](https://github.com/JonathonLuiten/TrackEval).
It is the official evaluation code for MOTChallenge and KITTI 2D Tracking and
reports HOTA, CLEAR MOT, and identity metrics. Export MOTChallenge-format truth
and tracks rather than adapting meter-based scoring to boxes. Map every frame
to a deterministic integer and disable MOTChallenge preprocessing unless the
experiment intentionally supplies its class, distractor, and ignore-region
rules.

## Recommended order

### 1. Validate the current Rust tracker with Stone Soup

This is a post-run validation, not a selectable tracker backend. Run the normal
experiment first, then have a small Python program read its
`measurements.mcap` and `truth.mcap`, run Stone Soup with the same lidar
observations and truth ego, and write world tracks for `fusion score tracks`.

Extend the external result import to preserve frames with no tracks and show
the imported tracks in Rerun. Add covariance only if the comparison includes
uncertainty.

There is no embedded Python or live subprocess. Stone Soup and the Rust tracker
process the same saved measurements, and the existing Rust code scores both
results.

Repeat this as the Rust tracker grows: add one small behavior, compare it with
the matching Stone Soup implementation, inspect the first difference, and keep
the reviewed result as a regression check.

### 2. Add recognized metrics before another tracker dependency

Finish the measures in [`DEMOS.md`](DEMOS.md) for world-space tracking. Add
TrackEval only with the first image-box experiment. Metrics and test cases make
the comparison credible; the number of integrated trackers does not.

### 3. Run ByteTrack with the first image-box experiment

Use a short annotated recording with fixed detections, including low-confidence
ones, and run the official ByteTrack implementation first. Record its revision,
detections, thresholds, per-sequence settings, and interpolation settings.

Add ByteTrack-cpp only if an in-process backend is useful. Pin a reviewed
commit, keep it in an optional crate, pass plain box values through `cxx`, and
check more than one fixture against the official result. Include empty frames
and lifecycle transitions. That backend can run during `fusion run` and be
selected in YAML, like GTSAM.

Similari is the alternative if native Rust and easy experimentation matter more
than matching a named external implementation.

### 4. Use Autoware after localized detections exist

Start with an external ROS 2 run over a lidar-derived or recorded Cartesian
detection sequence. Only consider a linked C++ extraction if repeated use shows
that maintaining it is worthwhile.
