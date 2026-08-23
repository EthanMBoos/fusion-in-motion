# Fusion in Motion: literature and design review

This optional background review collects foundational estimation work, current
systems, sensor-model guidance, timing and calibration research, robust and
non-Gaussian methods, experimental-design principles, related software, and
datasets. It predates the project's smaller playground scope and should be read
as a map of possible experiments, not as a roadmap or list of promised
features.

## Review method and limits

The original repository review inspected every file and all 39 unique external
URLs then present. Literature discovery was coverage-driven across estimation
theory, Lie-group state representation, filtering and smoothing,
continuous-time trajectories, calibration and observability, asynchronous and
out-of-sequence fusion, robust/non-Gaussian methods, data association, sensor
error models, experiment design, metrics, ground/UAV datasets, maintained
software, log formats, and external simulators. Claims were checked
preferentially against papers, publisher/proceedings pages, official
documentation, and primary repositories current on the review date.

This is an engineering literature review, not a PRISMA-style systematic review
or bibliometric proof that no paper exists outside the set. “Exhaustive” means
that every architecture-relevant topic is represented by foundational and/or
current primary sources, not that every sensor-fusion paper is listed.
Perception-only work is included where it determines an interface, failure
mode, sensor model, dataset, or evaluation issue relevant to navigation.

## Executive synthesis

The literature supports the project's central boundary: a compact experiment
core should own continuous truth, acquisition timing, explicit sensor effects,
causal delivery, hidden fault truth, reproducible logging, and scoring. It
should connect to mature perception and estimation systems rather than
reimplement all of them, and to full simulators only when an experiment truly
needs physics or rendered raw data.

Several conclusions recur across otherwise different systems:

- motion, excitation, clocks, and acquisition intervals are part of the
  estimation problem, not simulator bookkeeping;
- filters and factor graphs solve different operational problems and are often
  combined;
- raw sensor simulation, perception front ends, fusion back ends, and
  evaluation are separable layers;
- controlled synthetic faults and real logs answer complementary questions;
- data association, calibration knowledge, future-data access, and information
  level can dominate an algorithm-label comparison;
- deterministic replay is valuable, but statistical conclusions require paired
  multi-seed experiments; and
- consistency, failure, recovery, and resource use matter alongside trajectory
  accuracy.

No reviewed work argues for broadening the project into a general robot
simulator. The useful lesson is narrower: an experiment should be clear about
time, frames, truth visibility, sensor abstraction, and estimator fairness.

---

## 1. Scope and neighboring problem classes

“Sensor fusion” may describe navigation, SLAM, target tracking, semantic map
fusion, multimodal representation learning, occupancy prediction, or decision
fusion. Fusion in Motion is about moving-platform navigation state and SLAM
back ends. This scope admits pose, attitude, velocity, inertial biases,
calibration, landmarks, maps, and reference-frame alignments without implying a
benchmark for object detection or planning.

A camera or lidar front end usually turns raw samples into tracks, detections,
registrations, poses, or residuals. A recursive filter or graph then fuses those
results with inertial, wheel, GNSS, range, or other constraints. The distinction
matters because oracle landmark identity, a production feature tracker, and a
precomputed relative pose represent very different information even when all
are casually called “camera input.” [Cadena et al.'s SLAM
survey](https://doi.org/10.1109/TRO.2016.2624754) remains a useful overview of
front ends, back ends, observability, mapping, and evaluation.

The intended system is therefore not a smaller Gazebo and not another fusion
library. Its closest role is a controlled, moving measurement experiment whose
artifacts can feed several libraries and whose hidden truth can score them.

## 2. Probabilistic estimation foundations

### Bayesian and recursive estimation

- **Kalman (1960), [“A New Approach to Linear Filtering and Prediction
  Problems”](https://doi.org/10.1115/1.3662552)** establishes the linear-Gaussian
  recursive filter and covariance propagation against which implementations can
  be sanity-tested.
- Thrun, Burgard, and Fox, [*Probabilistic
  Robotics*](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/),
  covers Bayesian filtering, localization, mapping, particles, and common robot
  motion/measurement models.
- Bar-Shalom, Li, and Kirubarajan, [*Estimation with Applications to Tracking
  and Navigation*](https://doi.org/10.1002/0471221279), provides important
  foundations for consistency, innovations, delayed data, tracking, and
  navigation.
- Julier and Uhlmann, [“Unscented Filtering and Nonlinear
  Estimation”](https://doi.org/10.1109/JPROC.2003.823141), is the canonical
  unscented-filter reference. Arasaratnam and Haykin's [cubature Kalman
  filter](https://doi.org/10.1109/TAC.2009.2019800) is a related sigma-point
  alternative.
- Mahony, Hamel, and Pflimlin, [nonlinear complementary filters on
  SO(3)](https://doi.org/10.1109/TAC.2008.923738), represent a low-cost attitude
  family that a broad harness should be able to score without requiring a full
  navigation state.
- Solà's [quaternion ESKF
  notes](https://arxiv.org/abs/1711.02508) are a practical convention reference.
  They reinforce that quaternion storage is not the covariance coordinate and
  that perturbation side, ordering, and reset Jacobians must be declared.

These sources justify including linear, EKF/error-state, sigma-point,
information/square-root/UD, complementary, and consider-filter variants in the
technique space. They do not imply that the project core should implement them
all.

### Lie groups and invariant estimation

Mobile state estimation lives on rotation and rigid-transform manifolds.
Barfoot's [*State Estimation for
Robotics*](https://www.cambridge.org/highereducation/books/state-estimation-for-robotics/7B3E9741465F5E550A91E4242F4B18EE)
connects Lie groups, batch estimation, Gaussian processes, and practical
derivations. Solà, Deray, and Atchuthan's [micro Lie theory
tutorial](https://arxiv.org/abs/1812.01537) is a concise implementation reference.

Barrau and Bonnabel's [invariant EKF as a stable
observer](https://doi.org/10.1109/TAC.2016.2594085) and their [invariant-EKF SLAM
work](https://arxiv.org/abs/1510.06263) show how symmetry-aware error definitions
can preserve properties lost by ordinary linearization. The engineering lesson
is not to canonize the IEKF; it is to make the truth, tangent, perturbation, and
covariance conventions rich enough to compare ordinary and invariant methods
correctly.

### Factor graphs, smoothing, and preintegration

- Dellaert and Kaess, [“Factor Graphs for Robot
  Perception”](https://doi.org/10.1561/2300000043), gives the compact mathematical
  foundation for expressing measurements as factors and solving MAP problems.
- Kaess et al., [iSAM2](https://doi.org/10.1177/0278364911430419), establishes
  incremental smoothing and efficient relinearization through the Bayes tree.
- Forster et al., [IMU preintegration on
  manifold](https://arxiv.org/abs/1512.02363), explains how high-rate inertial
  samples become a bias-correctable factor with propagated uncertainty.
- Strasdat et al., [“Visual SLAM: Why
  Filter?”](https://doi.org/10.1016/j.imavis.2012.02.009), remains important
  context for filtering versus keyframe optimization tradeoffs.

Recursive filters remain useful when memory and computation must stay bounded
and a state is needed at every inertial update. Graphs and smoothers are useful
when delayed observations, loop closures, calibration states, or multiple map
frames revise the past. Neither family supersedes the other, which is why a fair
harness must label causal, fixed-lag, and full-batch access separately.

### Continuous-time estimation

Furgale, Barfoot, and Sibley, [continuous-time batch estimation with temporal
basis functions](https://doi.org/10.1109/ICRA.2012.6225005), and Barfoot, Tong,
and Särkkä, [exactly sparse GP trajectory
estimation](https://doi.org/10.15607/RSS.2014.X.003), provide two major ways to
query a trajectory at arbitrary acquisition times. Johnson et al.'s [modern GP
versus spline comparison](https://arxiv.org/abs/2402.00399) is useful when
selecting a representation.

For this project, continuous-time truth is more important than committing the
estimators to continuous time. It lets an IMU, rolling camera, spinning lidar,
and delayed aiding sensor observe one consistent moving body at different
instants and intervals.

## 3. Representative estimation systems

No maintained package spans raw perception, calibration, all estimator
families, simulation, deployment, visualization, and real data equally. The
following systems clarify useful boundaries.

| System | Primary job | Architectural lesson for Fusion in Motion |
| --- | --- | --- |
| [robot_localization](https://github.com/cra-ros-pkg/robot_localization) | ROS EKF/UKF fusion of pose, twist, odometry, IMU, and GNSS-derived inputs | Standard derived messages are useful, while raw perception remains outside the estimator. |
| [OpenVINS](https://docs.openvins.com/) and its [ICRA paper](https://doi.org/10.1109/ICRA40945.2020.9196524) | Visual-inertial estimation, calibration, simulation, and consistency analysis | Continuous SE(3) truth and separate noise/init/calibration streams are directly relevant. |
| [MINS](https://github.com/rpng/MINS) / [paper](https://arxiv.org/abs/2309.15390) | Tightly coupled IMU, camera, lidar, GNSS, and wheel estimation | High-order interpolation and dynamic cloning address asynchronous sensors without unbounded filter state. |
| [MaRS](https://github.com/aau-cns/mars_lib) / [paper](https://doi.org/10.1109/LRA.2020.3043195) | Modular recursive filtering for constrained computers | Sensor-specific covariance blocks and a chronological delay buffer make costs modular, with assumptions about neglected cross-correlation that must be reported. |
| [GTSAM](https://gtsam.org/) | Lie geometry, factors, smoothing, and optimization back ends | A mature C++ core and Python/MATLAB interfaces are compatibility targets; the experiment should not force a rewrite. |
| [Holistic Fusion](https://doi.org/10.1109/TRO.2026.3714645) | Setup-independent local/global estimation | Treating local/global/relative measurements and reference-frame alignments as graph variables motivates explicit frame provenance. |
| [Ground-Fusion++ and M3DGR](https://arxiv.org/abs/2507.08364) | Ground fusion and benchmarking under degraded sensing | Controlled low light, lidar degeneracy, and GNSS denial show why sensor degradation belongs in the benchmark, not a demo appendix. |
| [Stone Soup](https://stonesoup.readthedocs.io/en/v1.9/) | Python tracking and fusion research | Composable sensors, trackers, links, and latency are adjacent design inspiration, though its main domain is tracking. |
| [navlie](https://github.com/decargroup/navlie) | Lie-group filter/batch experiments and Monte Carlo analysis | A strong Python baseline and analysis partner without becoming the runtime. |
| [fact.rs](https://github.com/rpl-cmu/fact-rs) | Typed factor graphs in Rust | Compile-time factor/noise checks are attractive, but sparse performance and ecosystem maturity still favor GTSAM/Ceres for many external examples. |

Many successful systems combine a high-rate propagated local estimate with a
lower-rate smoothing back end. [Graph-MSF](https://github.com/leggedrobotics/graph_msf)
publishes at IMU frequency while measurement insertion and graph optimization
run separately inside a fixed smoothing window. [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
and its [paper](https://doi.org/10.1109/IROS45743.2020.9341176) use a persistent
mapping graph plus a periodically reset IMU/lidar graph; point timestamps are
required for deskewing. [FAST-LIO2](https://doi.org/10.1109/TRO.2022.3141876)
illustrates a tightly coupled iterated-filter alternative.

[VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/Vins-Fusion),
[VINS-Mono](https://doi.org/10.1109/TRO.2018.2853729),
[OKVIS](https://doi.org/10.1177/0278364914554813),
[Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO), and
[ORB-SLAM3](https://doi.org/10.1109/TRO.2021.3075644) are reality checks on the
front-end and language boundary. They depend on mature C++ vision, geometry,
and optimization components. Recorded or streaming language-neutral messages
preserve access to them without embedding their build systems in the Rust core.

The [MSCKF](https://doi.org/10.1109/ICRA.2007.364024) is the important
foundation behind OpenVINS-style filtering: it constrains a sliding set of
camera poses using feature tracks while eliminating feature states. It belongs
in the reference set because “EKF” alone does not describe the structure of a
visual-inertial estimator.

## 4. Motion, observability, calibration, and time

### Motion and excitation

A static rig can demonstrate measurement noise but not most mobile-fusion
failures. Bias, scale, extrinsic, temporal, gravity, and velocity variables may
be unobservable or weakly observable under particular trajectories.

Huang, Mourikis, and Roumeliotis, [observability-based rules for consistent EKF
SLAM](https://doi.org/10.1177/0278364909353640), show how estimator
linearization can spuriously add information along unobservable directions.
Bailey et al., [EKF-SLAM consistency](https://doi.org/10.1109/IROS.2006.281644),
connect Monte Carlo NEES behavior to these issues. Yang et al., [online IMU
intrinsic calibration](https://doi.org/10.15607/RSS.2020.XVI.026), characterize
degenerate motions and necessary excitation.

The practical consequence is two-sided: reference scenarios need rich motion,
and the suite also needs deliberately degenerate trajectories. A framework that
only generates “good” exciting motion cannot test observability failures or the
false confidence they cause.

The vehicle taxonomy should remain a trajectory-provider concern rather than a
second set of frames and messages. Useful ground profiles include differential
drive, skid steer with turn- or terrain-dependent slip, Ackermann/car-like,
omnidirectional, uneven-terrain SE(3), and arbitrary prescribed SE(3). Useful
aerial profiles include multirotor hover/translation/yaw/roll-pitch/vertical
excitation and, later, fixed-wing forward flight, banked turns, climb/descent,
wind, and air-data aiding. A kinematic profile is scientifically legitimate
when its limits and missing dynamics are explicit.

### Spatial and temporal calibration

Furgale, Rehder, and Siegwart, [unified temporal/spatial
calibration](https://doi.org/10.1109/IROS.2013.6696514), use a continuous-time
trajectory to calibrate heterogeneous asynchronous sensors. Li and Mourikis,
[online camera–IMU temporal
calibration](https://doi.org/10.1177/0278364913515286), address identifiability
and state augmentation; Qin and Shen provide an [optimization-based temporal
calibration approach](https://arxiv.org/abs/1808.00692). Rehder et al.'s
[Kalibr camera–IMU work](https://doi.org/10.1109/ICRA.2016.7487628) is the
practical multi-camera/inertial calibration foundation.

[iKalibr](https://arxiv.org/abs/2407.11420) extends targetless spatiotemporal
calibration across IMUs, cameras, lidars, depth cameras, and radars and requires
dynamically excited collection. [GLIC-Calib](https://doi.org/10.1109/IROS60139.2025.11247264)
addresses ground-vehicle lidar–IMU and camera–IMU calibration using vehicle
motion and ground constraints. The [OpenVINS calibration
guide](https://docs.openvins.com/gs-calibration.html) gives complementary
collection guidance emphasizing smooth translation and orientation changes.

Calibration knowledge has to be labeled as fixed truth, perturbed nominal,
estimated online, marginalized nuisance, or oracle. Otherwise an “algorithm”
comparison may actually be a comparison of initialization privilege.

### Clock semantics and rolling acquisition

Guo et al., [rolling-shutter VINS with inaccurate
timestamps](https://doi.org/10.15607/RSS.2014.X.057), show that acquisition
timing and clock offset couple directly to motion estimation. A spinning lidar
samples rays at different poses; a camera timestamp can refer to trigger,
exposure start, midpoint, or end.

The [Oxford Spires dataset](https://doi.org/10.1177/02783649251369905) is an
especially concrete example: its cameras/IMU use hardware synchronization,
devices use PTP, images are timestamped at mid-exposure, and lidar sweeps are
motion-corrected to camera time. The published material shows visible
camera–lidar disagreement under modest motion when that correction is omitted.

Bund et al., [“Alignment Sets for Sensor Fusion Against Temporal
Misalignment”](https://doi.org/10.4230/LIPIcs.ECRTS.2026.8), distinguish samples
that should represent one instant from samples intentionally acquired at
different times. Their clock drift, jitter, network delay, and alignment-set
taxonomy supports treating acquisition, reported device time, production, and
receipt as separate concepts rather than one generic timestamp error.

Bar-Shalom's [out-of-sequence measurement
update](https://doi.org/10.1109/9.981722) supplies the classical estimation side
of delayed data. It also underscores why an estimator must consume causal
arrival order rather than a log sorted by sensor stamp.

## 5. Estimator families the toolkit may explore

Breadth comes from preserving information and execution semantics, not shipping
one implementation of every row.

| Family | Useful experiment support | Representative basis/target |
| --- | --- | --- |
| Linear KF and complementary attitude | event/fixed-rate updates and simple covariance | Kalman; Mahony et al. |
| EKF/error-state EKF | high-rate propagation, asynchronous updates, reset convention | Solà; OpenVINS/MSCKF |
| Information, square-root, and UD filters | covariance or information output with numerically stable alternatives | standard estimation texts/external references |
| Iterated/invariant filters | repeated linearization or group error conventions | FAST-LIO2; Barrau/Bonnabel |
| UKF/cubature | deterministic measurement sequence and manifold sigma-point rules | Julier/Uhlmann; Arasaratnam/Haykin |
| Particles, RBPF, and mixtures | estimator random-source provenance and non-Gaussian outputs | Gordon et al.; FastSLAM; max-mixtures |
| IMM/multiple model | model probabilities and switching ground truth | Blom/Bar-Shalom |
| H-infinity, set-membership, zonotopic | bounded sets and metrics beyond covariance | later robust adapters |
| Batch MAP/factor graph | full visible log, declared offline access | GTSAM |
| Incremental/fixed-lag smoothing | delayed input, window, revisions of past states | iSAM2; Graph-MSF |
| Continuous-time estimation | acquisition intervals and per-element offsets | Furgale; Barfoot GP/splines |
| Robust/adaptive estimation | outliers, gates, weights, switches, adaptive covariance | switchable constraints, DCS, GNC |
| Moving-horizon estimation | constraints and bounded window | external optimization packages |
| Learned/hybrid fusion | tensor/front-end input, model/training provenance, uncertainty | TLIO, RoNIN, learned IMU correction |
| Decentralized/track-to-track | source-estimate provenance and unknown correlation | covariance intersection; later work |

Independent architecture dimensions include centralized/federated/decentralized
organization, loose/tight/raw coupling, recursive/fixed-lag/batch temporal
scope, state choice, pseudo-measurements, calibration treatment, association
policy, uncertainty representation, outlier handling, clock-state treatment,
and odometry/localization/SLAM mode. Recording only “EKF” or “factor graph” would
erase most of what makes two systems different.

## 6. Robust, non-Gaussian, multiple-model, and association methods

Independent Gaussian noise plus dropout is insufficient for comparing a wide
range of fusion techniques.

### Robust estimation and false constraints

- Huber and Ronchetti, [*Robust
  Statistics*](https://doi.org/10.1002/9780470434697), provides the statistical
  foundation for M-estimation, influence, and contamination.
- Sünderhauf and Protzel, [switchable
  constraints](https://doi.org/10.1109/IROS.2012.6385590), represent false graph
  constraints with latent switches.
- Olson and Agarwal, [max-mixtures](https://doi.org/10.1109/ICRA.2012.6224699),
  introduce multimodal robust factor models.
- Agarwal et al., [dynamic covariance
  scaling](https://doi.org/10.1109/ICRA.2013.6630557), provide an efficient
  downweighting approach for bad pose-graph constraints.
- Yang et al., [graduated non-convexity for robust spatial
  perception](https://doi.org/10.1109/LRA.2020.2965893), address high-outlier
  robust estimation.

These methods motivate false loop closures, multipath/NLOS, heavy-tailed
mixtures, burst faults, association swaps, and quality-dependent noise in the
experiment model, together with metrics for false rejection and recovery.

### Unknown correlation and decentralized fusion

Julier and Uhlmann, [covariance intersection for
SLAM](https://doi.org/10.1016/j.robot.2006.06.011), and Noack et al., [inverse
covariance intersection](https://doi.org/10.1016/j.automatica.2017.01.019),
address conservative fusion when cross-correlation is unknown. They are
important future references for track-to-track and multi-agent work. That work
should wait because it adds source identity, communication, clock domains, and
correlation bookkeeping to an already broad single-platform design.

### Data association and multimodality

Neira and Tardós, [joint compatibility branch and
bound](https://doi.org/10.1109/70.938381), is the canonical geometric
data-association consistency test. Bar-Shalom and Fortmann's [*Tracking and Data
Association*](https://www.sciencedirect.com/book/9780120797608/tracking-and-data-association)
covers PDA/JPDA foundations relevant when ambiguous or dynamic detections enter
scope.

Blom and Bar-Shalom, [interacting multiple-model
estimation](https://doi.org/10.1109/9.1299), support switching motion or slip
regimes. Gordon, Salmond, and Smith, [bootstrap particle
filter](https://doi.org/10.1049/ip-f-2.1993.0015), Dellaert et al., [Monte Carlo
localization](https://doi.org/10.1109/ROBOT.1999.772544), and Montemerlo et al.,
[FastSLAM](https://www.cs.cmu.edu/~mmv/papers/02aaai-fastslam.pdf), cover major
non-Gaussian and Rao–Blackwellized examples. Schmidt's [separate-bias/consider
filter](https://ntrs.nasa.gov/citations/19660006077) is relevant when uncertain
calibration should affect covariance without becoming a fully estimated state.

The resulting benchmark requirement is explicit association and information
provenance: oracle identity, noisy hard tracks, probabilistic association, and
joint inference cannot occupy an unlabeled leaderboard.

## 7. Learned and hybrid fusion

[IONet](https://ojs.aaai.org/index.php/AAAI/article/view/11355) is an early
learned inertial-odometry reference. [RoNIN](https://doi.org/10.1109/ICRA40945.2020.9196860)
addresses robust neural inertial navigation across users and devices.
[TLIO](https://doi.org/10.15607/RSS.2020.XVI.066) predicts displacement and
uncertainty then fuses them in an EKF, illustrating a hybrid rather than a pure
end-to-end replacement. Brossard et al., [learned gyroscope
denoising](https://doi.org/10.1109/LRA.2020.2966419), treat learned correction as
a sensor front end.

The core need not run neural networks. It does need reproducible tensor or
feature inputs, explicit learned uncertainty, model and training-data hashes,
determinism settings, and a distinction between pretrained components and
estimators tuned on the evaluation scenario.

## 8. Sensor modalities and model evidence

The useful abstraction is a fidelity ladder: a summary/constraint, an analytic
sensor, or external/raw data. That avoids false claims while allowing many
sensor configurations.

| Modality | Useful analytic or summary representation | Important effects and relevant evidence |
| --- | --- | --- |
| IMU | angular rate and specific force | white-noise density, bias random walk/Gauss–Markov, turn-on bias, scale/non-orthogonality, lever arm, vibration, saturation, quantization, temperature, coning/sculling |
| Wheel/odometry | encoder ticks or per-wheel increments; derived twist separately | radius/baseline, quantization, missed ticks, backlash, kinematic model, state/terrain-correlated slip |
| GNSS solution | geodetic/local position and velocity with status/covariance | correlated horizontal/vertical error, fix transitions, outage, latency, multipath/common bias |
| Raw GNSS | pseudorange, Doppler, carrier phase, C/N0 | satellite/clock/orbit, atmosphere, visibility, cycle slip, integer ambiguity, multipath |
| Camera geometric | bearing/pixel/track or relative constraint | intrinsics/distortion, exposure interval, rolling shutter, blur/visibility, missed/false tracks, association policy |
| Camera raw | image plus calibration and exposure metadata | photometry, auto-exposure, lens effects, weather; external renderer or real log |
| Lidar analytic | timed ray/range or point | scan pattern, FOV/range, per-ray pose, beam/noise approximation, occlusion, return/dropout policy |
| Lidar raw | packet or point cloud | materials, intensity, weather, interference, multipath, waveform/return selection; external provider |
| Radar analytic | range, angle, radial velocity, covariance/SNR detection | probability of detection, range/angle/range-rate uncertainty, clutter, ghosts, multipath, RCS proxy, chirp/scan time |
| Radar raw | heatmap, ADC, or cube | waveform, antenna array, materials/RCS, interference and multipath; specialized external simulation |
| Magnetometer | sensor-frame field vector | Earth/local field, hard/soft iron, scale/bias, spatial and motor/current disturbance |
| Barometer | pressure or declared derived altitude | reference/weather pressure, bias/drift, vertical correlation, dynamic pressure/rotor wash |
| Rangefinder | timed beam range | surface intersection, tilt/FOV, invalid returns, limits, ground clutter |
| Optical flow | integrated angular displacement and quality | integration period, gyro compensation, texture/height dependence, scale/range coupling, dropout |
| UWB | one-/two-way range and anchor ID | anchor clocks, antenna delay, NLOS positive bias/heavy tail, geometry/GDOP |
| Air data | differential pressure/airspeed; optional AoA/sideslip | wind, density, scale/bias, blockage, latency; useful for fixed-wing extension |
| External navigation | pose/velocity/heading with covariance and frame | reference alignment, correlated output, latency, reset/jump, relocalization |
| Event/thermal/depth/ultrasonic | raw or front-end constraint | modality-specific transfer, timing, environment, and quality; adapter/specialized scope |

### Inertial modeling

El-Sheimy, Hou, and Niu, [Allan-variance inertial-sensor
modeling](https://doi.org/10.1109/TIM.2007.908635), and Farrell et al., [IMU
error-modeling tutorial](https://doi.org/10.1109/MCS.2022.3209059), are direct
guidance for translating datasheet noise, bias processes, and continuous-time
models into discrete samples. They justify requiring written equations and
units rather than a generic “noise sigma.”

An IMU model must include specific force rather than world acceleration, the
sensor lever-arm rotational acceleration, explicitly framed angular rate,
continuous-density/discrete covariance conversion, acquisition averaging, and
clearly enabled or disabled Earth-rotation/coning/sculling effects. Plausible
but dimensionally wrong IMU simulation is one of the project's highest risks.

### Wheel, GNSS, and radar

Borenstein and Feng, [systematic mobile-robot odometry error measurement and
correction](https://doi.org/10.1109/70.544770), is the classic wheel-radius and
baseline calibration reference. It should be paired with non-systematic slip
models related to turns, terrain, and vehicle type.

Groves, [*Principles of GNSS, Inertial, and Multisensor Integrated Navigation
Systems*](https://us.artechhouse.com/Principles-of-GNSS-Inertial-and-Multisensor-Integrated-Navigation-Systems-Second-Edition-P2043.aspx),
and Teunissen and Montenbruck's [*Springer Handbook of Global Navigation
Satellite Systems*](https://doi.org/10.1007/978-3-319-42928-1) provide the
reference-frame, solution-level, and raw-observable foundation. A simple GNSS
solution model is appropriate for initial experiments; raw carrier-phase ambiguity and satellite
physics warrant a specialized module.

Patole et al.'s [automotive radar
survey](https://doi.org/10.1109/SPM.2016.2628914) covers FMCW architecture,
measurements, and challenges. It is the clearest warning against treating radar
as a renamed lidar: Doppler/radial velocity, probability of detection, angular
uncertainty, clutter, RCS/material behavior, ghosts, and multipath are central.

### UAV aiding sensors

Current [PX4 EKF2](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf)
and [ArduPilot EKF3](https://ardupilot.org/dev/docs/extended-kalman-filter.html)
documentation are valuable practice checks for combinations of IMU, GNSS,
magnetometer, barometer, rangefinder, optical flow, airspeed, and external
vision. They do not substitute for scientific sensor models, but they ensure the
UAV reference profile reflects real navigation-stack inputs and switching
behavior.

## 9. World representation, perception, and rendering

A useful synthetic world separates:

1. metric surfaces for range, visibility, and free space;
2. landmarks/objects with hidden identity and optional semantics;
3. appearance for camera rendering; and
4. truth frames, trajectories, mounts, and object motion.

A camera and lidar can observe one physical object without emitting the same
type. Hidden shared identity is useful for scoring, but it becomes an oracle if
given to the estimator without an explicit known-association declaration.

[REP-103](https://reps.openrobotics.org/rep-0103/) and
[REP-105](https://reps.openrobotics.org/rep-0105/) provide the established SI,
right-handed, body/sensor, and local/global frame baseline. They are not enough
by themselves: transform direction, quaternion storage, perturbation side,
velocity expression, covariance order, gauge, and unknown covariance still need
project-specific rules.

### Why Gaussian splats are not geometric truth

Ordinary 3D Gaussian splatting is an appearance representation optimized for
novel-view rendering. It does not inherently guarantee metric scale, a
watertight surface, or physically correct range away from training views.

- [GeomGS](https://arxiv.org/abs/2501.13417) responds to geometric/scale weakness
  with lidar constraints and geometric confidence.
- [SplatAD](https://openaccess.thecvf.com/content/CVPR2025/html/Hess_SplatAD_Real-Time_Lidar_and_Camera_Rendering_with_3D_Gaussian_Splatting_CVPR_2025_paper.html)
  models camera rolling shutter, lidar intensity/dropout, and moving actors
  separately rather than treating splatting as a generic sensor.
- [Uni-Gaussians](https://arxiv.org/abs/2503.08317) rasterizes camera output but
  ray-traces lidar because an active ranging sensor is not a cylindrical camera.
- [SimULi](https://research.nvidia.com/labs/sil/projects/simuli/) uses
  factorized camera/lidar Gaussians with geometric anchoring to reduce
  cross-sensor inconsistency.
- [NVIDIA NuRec](https://docs.nvidia.com/nurec/basics/how-nurec-works.html)
  reconstructs captured camera/lidar data and packages neural rendering,
  trajectories, and scene information for OpenUSD simulators; reconstruction is
  not itself the whole robot simulator.
- A 2026 [mesh plus geometry-consistent 3DGS sim-to-real
  study](https://doi.org/10.1016/j.engappai.2026.114372) similarly combines a
  metric surface extracted from registered lidar with a photorealistic
  appearance representation.

The defensible captured-world bundle therefore pairs a registered mesh, lidar
map, or metric surface with a splat/textured appearance asset. An external
renderer may provide pixels later, but appearance must not silently become
range truth.

### Full simulators

[Gazebo](https://gazebosim.org/docs/harmonic/sensors/),
[Webots](https://cyberbotics.com/doc/guide/introduction-to-webots),
[CARLA](https://carla.readthedocs.io/en/latest/start_introduction/), and
[Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/index.html)
own world assets, physics, actors, rendering, sensors, communication, and
interactive execution. That integration is valuable for collision, traction,
photometric effects, material response, radar physics, or a complete autonomy
stack, but couples the experiment to an engine, update loop, asset system, and
hardware profile.

Fusion in Motion can answer its first questions with a continuous kinematic
trajectory and analytic geometry. When an experiment needs rendered RGB,
material-aware radar/lidar, or contact dynamics, an external simulator should
consume the declared truth timeline and return an L2 observation. This keeps
the core experiment loop understandable.

UAV-specific external options include [Pegasus
Simulator](https://pegasussimulator.github.io/PegasusSimulator/) and
[Flightmare](https://github.com/uzh-rpg/flightmare). [AirSim](https://github.com/microsoft/AirSim)
is historically important but its maintenance state should be evaluated before
an adapter is committed. Versions matter for all simulator APIs, particularly
Isaac Sim's evolving RTX sensor paths.

## 10. Language, runtime, logs, and interoperability

### Why a small Rust core is reasonable

Rust does not improve estimator accuracy. It can make event scheduling,
ownership of mutable simulation state, typed boundaries, large-log processing,
and reproducible native builds easier to maintain without a garbage collector.

[Copper](https://github.com/copper-project/copper-rs) uses a compile-time task
graph, avoids allocation on its real-time path, and records replay logs.
[Dora](https://github.com/dora-rs/dora) combines a Rust runtime with Arrow/shared
memory and native Rust, Python, C, and C++ nodes. [fact.rs](https://github.com/rpl-cmu/fact-rs)
uses Rust types to detect factor arity, variable type, and noise-dimension
mismatches.

These are infrastructure examples, not evidence that Rust makes frames,
covariances, or noise equations correct. Rust also does not guarantee identical
floating-point bytes across every compiler, library, target, compression
setting, or parallel execution.

The ECRTS study [“A First Look at ROS 2 Applications Written in Asynchronous
Rust”](https://arxiv.org/abs/2505.21323) found that bounded response required an
explicit mapping of callbacks, priorities, and threads. An async runtime is not
automatically deterministic or real time. This supports beginning with an
offline event clock rather than promising deployment-runtime guarantees.

### Other language boundaries

Python remains the natural home for NumPy/SciPy, OpenCV, PyTorch, navlie,
notebooks, plotting, and GTSAM bindings. C++ remains necessary for GTSAM, Ceres,
OpenCV, PCL, Kalibr-derived tools, and much of the ROS SLAM ecosystem.
[Ceres](https://ceres-solver.org/), [manif](https://github.com/artivis/manif),
and [Sophus](https://github.com/strasdat/Sophus) are useful compatibility
targets. [GeographicLib](https://geographiclib.sourceforge.io/) is a strong
geodesy implementation rather than a calculation to reproduce casually.

ROS 2 belongs first at an offline conversion boundary. The
[rosbag2 MCAP plugin](https://github.com/ros-tooling/rosbag2_storage_mcap),
Foxglove schemas, and `rosbags` form the relevant converter ecosystem. MATLAB
can use file export where a controls group needs an existing implementation.

### MCAP, Protobuf, and Rerun

[MCAP](https://mcap.dev/) is a defensible primary container: append-only,
self-describing, indexed, and implemented in Rust, C++, and Python with support
for arbitrary schemas, channels, attachments, chunks, compression, record and
publish/log times. It is not itself a domain schema, causal replay guarantee, or
experiment bundle. The project must still define time mapping, record order,
visibility, descriptors, hashes, topics, chunking, compression, CRC behavior,
and conversion loss.

Protobuf provides generated readers across the target languages. Its own
compatibility rules do not decide physical units, signed run-relative clocks,
covariance semantics, frame conventions, or truth visibility. Including the
transitive descriptor set is important for reflection and long-lived logs.

[Rerun](https://rerun.io/docs/concepts/logging-and-ingestion/mcap/message-formats)
can ingest supported ROS 2 and Foxglove Protobuf images, clouds, poses,
transforms, and timestamps and reflect unknown messages. Custom messages do not
automatically acquire rich sensor visualization; a small lens/converter remains
necessary. Rerun should be a viewer, never the canonical results store.

[gnss-ins-sim](https://github.com/Aceinna/gnss-ins-sim) is worth consulting for
specialized inertial/GNSS simulation concepts, but need not become a core
dependency.

## 11. Evaluation, consistency, and experiment design

### Trajectory metrics and alignment

Sturm et al., [TUM RGB-D benchmark and
ATE/RPE](https://doi.org/10.1109/IROS.2012.6385773), is the canonical source for
absolute and relative trajectory measures. Zhang and Scaramuzza's [trajectory
evaluation tutorial](https://doi.org/10.1109/IROS.2018.8593941) explains
alignment and metric subtleties. [evo](https://github.com/MichaelGrupp/evo) is a
widely used implementation reference, but a pinned program is not a substitute
for stating the metric.

Translation-only, yaw-plus-translation, SE(3), and Sim(3) alignment answer
different questions. Sim(3) can hide scale error; SE(3) can hide an expected
global-frame error. A benchmark must tie alignment to estimator gauge and score
velocity, attitude, biases, calibration, initialization, availability, drift,
and failures where applicable—not just one ATE number.

### NEES, NIS, and consistency

NIS uses innovation and innovation covariance; NEES uses truth error and the
matching estimated covariance in a precisely defined local coordinate. They
require correct degrees of freedom and chi-square bounds. ANEES/ANIS require
multiple independent trials. Gauge freedoms must be projected or anchored, and
a pose marginal cannot be scored as the full state.

Bailey et al.'s [consistency study](https://doi.org/10.1109/IROS.2006.281644)
and Huang et al.'s [observability-based
rules](https://doi.org/10.1177/0278364909353640) are essential context.
Coverage, innovation mean, autocorrelation/whiteness, and temporal runs help
expose structure a scalar average hides. A visually close trajectory can remain
statistically inconsistent.

### Controlled faults and failure policy

Every fault study should preserve onset/offset, hidden magnitude, estimator
detection/gating response, peak and integrated error, time outside bounds,
recovery, healthy-data rejection, faulty-data acceptance, and terminal outcomes
such as NaN, crash, timeout, or invalid covariance. Thresholds should be set
before the sweep, and failed or missing runs must remain in results to avoid
survivorship bias.

### Reproducible stochastic experiments

Salmon et al., [counter-based parallel random
numbers](https://doi.org/10.1145/2063384.2063405), support random draws keyed by
component, event, effect, and index. This makes enabling one effect or sensor
less likely to perturb unrelated streams than a mutable sequential generator.

Heidelberger and Iglehart, [common random
numbers](https://doi.org/10.2307/1426860), motivate paired comparisons using the
same stochastic trials. McKay, Beckman, and Conover, [Latin hypercube
sampling](https://doi.org/10.1080/00401706.1979.10489755), and Saltelli et al.,
[*Global Sensitivity Analysis*](https://doi.org/10.1002/9780470725184), support
multi-parameter designs and interaction analysis beyond one-factor sweeps.

Pineau et al., [reproducibility in machine learning
research](https://jmlr.org/papers/v22/20-303.html), supplies a useful artifact
and provenance checklist, especially when learned estimators enter scope.
Reproducibility requires resolved configuration, code and dependency versions,
assets, model/training provenance, machine settings, failed trials, hashes, and
metric parameters in addition to a seed.

## 12. Dataset coverage

Datasets serve two purposes: they test import paths and expose the
gap between analytic assumptions and real sensors. They are not proof that a
synthetic model is realistic. Each importer should document sensor semantics,
calibration, synchronization, reference-trajectory quality and valid intervals,
license/registration, and conversion loss.

| Dataset | Platform/domain | Sensors and challenges | Relevance |
| --- | --- | --- | --- |
| [EuRoC MAV](https://projects.asl.ethz.ch/datasets/euroc-mav/) | micro aerial vehicle | stereo, IMU, mocap/laser truth, aggressive sequences | canonical compact VIO import and synchronization case |
| [TUM VI](https://doi.org/10.1177/0278364919881687) | handheld/moving rig | fisheye stereo, IMU, long runs, partial mocap truth | calibration and drift evaluation |
| [Blackbird](https://arxiv.org/abs/1810.01987) | aggressive quadrotor | stereo/virtual cameras, IMU, motor speed, high-rate mocap | fast aerial motion and dynamics-adjacent data |
| [UZH FPV](https://doi.org/10.1177/0278364918801965) | racing quadrotor | event/frame cameras, IMU, aggressive flight | event and high-speed extension |
| [NTU VIRAL](https://doi.org/10.1177/02783649211052312) | UAV | dual lidar, synchronized cameras, multiple IMUs, UWB, laser-tracker truth | strongest broad aerial multisensor match |
| [Hilti SLAM Challenge](https://arxiv.org/abs/2109.11316) | handheld/robot construction sites | cameras, lidar, IMU, calibration, sparse high-grade truth | difficult geometry and degradation |
| [Hilti–Trimble–Oxford 2026](https://arxiv.org/abs/2607.06464) | construction mapping | 360 visual-inertial, rolling shutter, floor-plan priors | current timing and prior-aided localization case |
| [M2DGR](https://arxiv.org/abs/2112.13659) | ground robot | camera, thermal, event, IMU, lidar, GNSS/RTK | broad ground sensor constellation |
| [M3DGR/Ground-Fusion++](https://arxiv.org/abs/2507.08364) | ground robot | controlled visual/lidar/wheel/GNSS degradation | direct robustness match |
| [M2UD](https://arxiv.org/abs/2503.12387) | uneven-terrain ground robot | aggressive motion, weather/smoke/darkness, mapping truth | SE(3) ground and degradation |
| [Oxford RobotCar](https://doi.org/10.1177/0278364916679498) | road vehicle | repeated seasons, camera, lidar, GPS/INS | long-term environmental change |
| [Oxford Radar RobotCar](https://arxiv.org/abs/1909.01300) | road vehicle | FMCW scanning radar, lidar, camera, GPS/INS | radar experiments and long-range sensing |
| [Boreas](https://doi.org/10.1177/02783649231160195) | road vehicle | lidar, radar, camera, centimeter GNSS/INS, seasons | all-weather long-term localization |
| [Boreas Road Trip](https://arxiv.org/abs/2602.16870) | road vehicle | Doppler radar/lidar, camera, dual IMU, wheel, GNSS/INS | current rich driving combination |
| [MulRan](https://sites.google.com/view/mulran-pr) | road vehicle | radar/lidar repeated routes | radar/lidar place recognition and timing |
| [KITTI](https://doi.org/10.1177/0278364913491297) / [KITTI-360](https://doi.org/10.1109/TPAMI.2022.3179507) | road vehicle | stereo, lidar, GPS/IMU, semantics | canonical compatibility; less suitable for controlled faults |
| [Newer College](https://doi.org/10.1177/0278364921988297) | handheld/ground-like mapping | lidar, cameras, IMU, detailed map truth | lidar-inertial/visual mapping and motion distortion |
| [Oxford Spires](https://doi.org/10.1177/02783649251369905) | large-scale mapping rig | synchronized camera/IMU, raw/corrected lidar, survey truth | timing and motion-correction reference |
| [SEW Multimodal AMR](https://github.com/SEW-Eurodrive-Open-Source/Multimodal_AMR_dataset) | industrial AMR | RGB, thermal, radar, ultrasonic, ToF, laser | unusual modalities and calibration documentation |
| [TartanGround](https://arxiv.org/abs/2505.10696) | simulated wheeled/legged | stereo, depth, flow, lidar, semantics, occupancy | external synthetic front-end testing, not real validation |

A sensible import order is:

1. EuRoC or TUM VI for a small, well-understood camera/IMU path;
2. NTU VIRAL for the aerial multisensor claim; and
3. M2DGR/M3DGR or Boreas for ground multimodality, degradation, and radar.

One deep importer is more scientifically useful than many superficial format
claims.

## 13. Reference and claim audit

The earlier design notes used mostly accurate and unusually current references.
Their weakness was not unreliable citations; it was disproportionate attention to
recent systems/rendering and insufficient coverage of older mathematical,
statistical, sensor, and experimental foundations. The expanded review above
corrects that balance.

| Item/group | Disposition |
| --- | --- |
| robot_localization | Keep as a representative loosely coupled ROS system; pin software version in experiments. |
| OpenVINS | Keep; pair documentation with the ICRA paper, MSCKF, and observability work. |
| MINS | Keep; cite its paper as well as the GPL repository. |
| MaRS | Keep; describe covariance-block assumptions carefully and inspect the pinned license terms. |
| GTSAM | Essential target; pair with factor-graph, iSAM2, and preintegration foundations. |
| Holistic Fusion | Keep the accepted T-RO DOI rather than only a mutable preprint link. |
| Ground-Fusion++/M3DGR | Keep and distinguish the method from the dataset/benchmark. |
| Stone Soup | Keep as adjacent tracking research infrastructure, not central navigation evidence. |
| navlie | Keep as a strong manifold/Monte Carlo baseline; pin a release. |
| fact.rs | Keep as a young Rust comparison, not proof of mature Rust graph performance. |
| Graph-MSF, LIO-SAM | Keep as hybrid/fixed-lag and deskewing examples; cite papers/versions. |
| VINS-Fusion, Kimera-VIO | Keep as ecosystem examples; balance with VINS-Mono, OKVIS, OpenVINS, and MSCKF papers. |
| iKalibr, GLIC-Calib, OpenVINS guide | Keep; pair operational guidance with foundational calibration/observability work. |
| Oxford Spires, Alignment Sets | Keep and integrate their timing distinctions into experiment design. |
| REP-103/105 | Keep as baseline, supplemented by project transform/tangent/covariance rules. |
| GeomGS, SplatAD, Uni-Gaussians, SimULi, NuRec | Keep in optional rendering rationale; do not let them displace estimation foundations. |
| Gazebo, Webots, CARLA, Isaac Sim | Keep as boundary comparisons and pin versions for adapters. |
| Copper, Dora, async Rust/ROS 2 | Keep briefly as runtime evidence, not scientific estimation evidence. |
| MCAP, Protobuf, Rerun | Keep, while distinguishing stored data from a derived viewer. |
| M3DGR, M2UD, SEW, TartanGround | Keep as recent ground additions and balance with aerial, radar, and canonical datasets. |

Claims should retain these qualifications:

- “Recurring patterns in representative maintained systems” is better supported
  than “architectural patterns that survived.”
- Selected 2025–2026 systems emphasize degradation, calibration, asynchronous
  data, and changing frames; that is not a comprehensive claim about the whole
  field.
- Quaternions are a storage/propagation choice, not a covariance coordinate.
- An analytic trajectory supplies exact derivatives only with respect to its
  configured representation.
- Deterministic logical messages do not imply cross-platform byte-identical
  compressed files.
- An idealized radar detection model is not raw radar simulation.
- A fixed seed gives replay, not statistically general performance.

## 14. Licensing, provenance, and reproducibility cautions

The repository's MIT license covers original project code, not every integration
or artifact:

- OpenVINS and MINS are GPL-3.0; invoking an external executable is different
  from copying or linking its source into an MIT crate.
- MaRS repository terms include an additional no-commercial-use condition
  despite BSD-like wording; the exact pinned license needs inspection before
  reuse.
- Robotics datasets may require registration, restrict use to research or
  non-commercial work, prohibit redistribution, or mandate citation.
- trained weights, Gaussian splats, meshes, maps, vehicle assets, camera models,
  simulator SDKs, and vendored/generated schemas have independent terms.

The eventual repository should maintain a third-party notice and registry with
source, version/commit, license, required citation, redistribution permission,
and content hash. Download scripts should send users to upstream terms and
verify hashes rather than commit restricted data. Dataset reference trajectories
also need uncertainty, provenance, validity intervals, and known limitations;
“ground truth” is not always exact.

## 15. Design traceability and residual risks

| Evidence/theme | Resulting design choice | Residual risk |
| --- | --- | --- |
| Continuous-time, rolling acquisition, timing papers | acquisition intervals, reported stamps, production and receipt time, per-element offsets | schemas can still expose hidden true time accidentally |
| Observability and calibration literature | exciting and deliberately degenerate profiles; truth/nominal/estimated calibration labels | impressive nominal paths may hide unobservable states |
| Filter/smoother coexistence | causal, fixed-lag, and offline result classes | full-file “online” adapters can peek ahead |
| Robust/non-Gaussian work | outlier, mixture, association, common-mode, and recovery experiments | simple early models may overstate modality coverage |
| IMU/GNSS/wheel/radar references | explicit equations, units, and fidelity levels | plausible but incorrect sensor physics |
| Factor-graph and Lie-group foundations | one SE(3) truth and explicit tangent/covariance conventions | cross-language convention drift |
| Deterministic random generation and common random numbers | keyed named draws and paired multi-seed trials | byte determinism can still vary by platform/library |
| Metric/consistency literature | declared alignment/gauge, NIS/NEES math, failures retained | leaderboards may still collapse incompatible tasks |
| Current ground/UAV systems and datasets | possible future examples | broad sensor coverage may outrun deeply tested models |
| Full simulators and neural rendering | external L2 providers; geometry separate from appearance | adapter complexity or engine/version churn |
| Mature C++/Python ecosystem | process/file protocol instead of a universal Rust ABI | schema/converter maintenance burden |
| Dataset/software licenses | provenance and third-party registry | use restrictions can limit distributed examples |

The principal scope risk is becoming a general simulator before producing one
credible comparison. The principal scientific risk is the opposite: producing
attractive plots from wrong frame, time, covariance, or sensor semantics. The
The practical response is to keep the implementation small while testing
frames, timing, and sensor equations carefully.

## Final assessment

The reviewed literature supports proceeding. A moving, deterministic,
language-neutral experiment runner with explicit clocks, hidden truth, sensor
fidelity levels, estimator access classes, multi-seed scoring, and real-log
validation is a sensible and useful project. It can cover a wide variety of
ground and aerial sensor configurations without pretending to be a universal
physics engine or estimator implementation.

The important constraint is architectural honesty: each record must say what
information it contains, each estimator must say what it was allowed to see,
each sensor must say how faithful its model is, and each score must say how
gauge, alignment, stochastic trials, failures, and resources were handled. With
those rules, the project can add techniques and modalities over time without
changing the scientific meaning of an experiment.
