# Build on the tracking engine

`fusion-tracking` provides a small tracking API and a point-target reference
manager. Your repository supplies the state, sensor models, data formats,
large custom scenarios, metrics, and visualization.

`fusion-in-motion` shows one setup using planar camera bearings, lidar range
and bearing, and planar ego poses.

## Code layout

```text
fusion-tracking
  timestamped scans, scan context, and IDs
  whole-tracker input and output API
  point-target hypothesis and association APIs
  reference track manager, events, and diagnostics

fusion-in-motion
  planar sensor and motion models
  Protobuf and MCAP records
  scenarios, scoring, and Rerun dashboard

your repository
  large custom scenarios and sensor configurations
  tracker backends, models, schemas, data, metrics, and displays
```

Your repository depends on `fusion-tracking`. `fusion-tracking` does not load
or register project code.

## Why the API stops here

`fusion-tracking` has no dependencies. Using it does not pull in the simulator,
Protobuf, MCAP, Rerun, scenario YAML, or the planar camera and lidar code.

The whole-tracker API is the boundary between a runner and a tracker:

```text
timestamped scan + scan context
    -> tracker-owned processing
    -> timestamped tracks + lifecycle events + backend diagnostics
```

Implement `Tracker<Scan>` when the backend owns a state machine that does not
fit the reference manager. Its births, deaths, identities, history, and other
retained state stay behind the API. `TrackingOutput` keeps diagnostics generic,
so the common output does not force every backend to expose pair hypotheses or
association details that it may not have.

Keep large custom scenarios and sensor configurations in their own
repositories. Those projects implement the API with their own types. The
engine does not need a feature, registry entry, or branch for each setup.

## Shared scan and output API

`ObservationBatch<D, C, B>` is the point-detection scan used by the reference
manager:

- `D` is one observation payload;
- `C` is context tied to one observation, such as an ego pose at that
  observation's measurement time; and
- `B` is scan-wide context, such as sensor identity, platform pose,
  calibration, field of view, coverage, detection probability, or clutter
  parameters.

The batch retains `B` when it has no observations. An empty frame can therefore
mean either that an observable target was missed or that the sensor could not
see it. Without scan context, those cases are indistinguishable and track
deletion becomes dependent on where the platform points.

`Tracker<Scan>` accepts one scan and returns `TrackingOutput<S, Diagnostics>`.
The output contains measurement and arrival times, `TrackReport<S>` values,
lifecycle events, and diagnostics selected by the backend. A report is an
extracted target estimate; it does not imply that the backend stores one state
per report internally.

`TrackIdentity::Labeled` carries a stable tracker ID. `Unlabeled` represents a
target set with no persistent identity. It is not a missing or temporary ID.
An unlabeled backend can leave identity-specific lifecycle events empty and
put set-level information in its diagnostics.

The integration test in
[`downstream_api.rs`](../crates/fusion-tracking/tests/downstream_api.rs) shows
both integration levels: assembling the reference manager and implementing the
outer API with backend-owned state.

## Reference point-target manager

`TrackManager` covers online, scan-by-scan point-target processing. It retains
one reduced state for each live track and resolves the current batch before the
next call.

One batch is one mutual-exclusion domain: normally one sensor scan at one
effective time, with no more than one observation from each target and no more
than one target behind each observation. Process sensors as separate batches
when they do not share one joint association problem. A same-time update that
must assign several sensor observations to the same track needs a different
scan representation or manager; putting those observations in one batch would
violate this API's one-to-one rule.

For each batch, `TrackManager`:

1. checks the observation IDs;
2. predicts each live track to every observation time;
3. builds every track/observation hypothesis;
4. runs hard or marginal association;
5. validates probability mass and selected hypotheses;
6. reduces selected and missed-detection branches at one time;
7. asks the model whether each predicted track was observable in this scan;
8. passes association probability and detection opportunity to lifecycle;
9. passes each observation's unassigned probability to initiation;
10. returns confirmed tracks at the batch output time.

The manager stages the complete batch before changing its tracks, initiator,
or lifecycle policy. An error leaves them at the end of the previous successful
batch. This makes a failed scan retryable, but it also means each call clones
all live tracks and the mutable policy state.

## Point-target component APIs

| API | What your code provides |
| --- | --- |
| `HypothesisModel<S, D, C, B>` | State prediction, pair gating and correction, and scan-level detection opportunity |
| `AssociationEngine<S, D, C, B>` | Hard assignments or per-track marginal probabilities using the typed scan, pair hypotheses, and per-track detection opportunities |
| `PosteriorReducer<S>` | A hard posterior or a common-time reduction of weighted detection and miss branches |
| `Initiator<S, D, C, B>` | New states selected from observations and their unassigned probabilities |
| `LifecyclePolicy<S>` | Confirmation, misses, coasting, and deletion from structured association evidence |

The initiator and lifecycle policy implement `Clone` because the manager runs
them on batch-local copies and commits them only after every stage succeeds.

The crate includes gated global-nearest-neighbor assignment using the Hungarian
algorithm, a reducer for hard assignments and hard misses, and
probability-thresholded lifecycle with detection-opportunity-aware coasting.

`HypothesisModel` returns `Inside` with a posterior, `Outside`, or `Invalid`, so
an inside gate cannot exist without a state that can be selected. The model
also reports `DetectionOpportunity` for each predicted track. `NotObservable`
coasts a track; an observable scan with insufficient association probability
is a miss. The supplied lifecycle measures deletion age only across observable
time, so an out-of-coverage scan does not move a track toward deletion.

`AssociationPlan::Hard` assigns each observation to no more than one track.
`AssociationPlan::Marginal` describes every live track with a missed-detection
probability and observation probabilities. The manager checks that each
track's probabilities sum to one, that no observation receives more than total
probability one, and that every selected pair has an inside-gate posterior.

The associator receives the full typed batch and one
`TrackDetectionOpportunity` per track. Sensor conditions such as coverage,
clutter, and detection probability are properties of the scan; they should not
be hidden in a pair score.

Marginal probability has two separate consumers. Lifecycle receives a track's
total association probability. Initiation receives each observation's
unassigned probability:

```text
1 - sum(probability assigned to the observation by existing tracks)
```

The application chooses the hit and birth thresholds. Any positive marginal
must not automatically become a hit or permanently block a plausible birth.

## Time, state, and scale limits

Measurement time and arrival time are separate. Prediction and correction use
measurement time; arrival time records when the scan became available. The
manager does not enforce time order and does not retain snapshots for replay.
A caller handling late data must reject it, reorder it, or own the history
needed to revise prior state before calling the manager.

Each managed track owns an independent `S`. Passing the same platform pose or
calibration through scan context conditions every track on that value, but it
does not preserve cross-covariance caused by shared platform, calibration,
bias, or map uncertainty. A tracker that estimates shared state must own that
joint state outside `TrackManager` and implement the outer `Tracker` API.

The reference manager builds the complete track/observation hypothesis matrix.
It predicts separately for each pair, stores candidate states, and clones all
tracks for transactional processing. This is easy to inspect and works well
for the reference setup. At larger sizes or with expensive state objects,
sparse gating, prediction caching, shared state storage, or a different commit
strategy may be needed before runtime comparisons are meaningful.

Deleting a managed track removes its ID and state. The reference manager has no
archive, revival, genealogy, public snapshot, or tentative-track inspection
API. Put those responsibilities in backend-owned state when a scenario needs
them.

## Add the dependency

Pin a reviewed commit:

```toml
[dependencies]
fusion-tracking = {
  git = "https://github.com/ethanboos/fusion-in-motion",
  rev = "<reviewed-commit>"
}
```

For local work, put a Cargo `[patch]` in your repository and point it at a
sibling checkout.

## Choose the integration level

Use `TrackManager` when one reduced state per live track and the point-detection
batch rules match the tracker. Supply its model, associator, reducer, initiator,
and lifecycle policy.

Both integration levels are called the same way:

```rust
use fusion_tracking::Tracker as _;

let output = tracker.process_scan(&scan)?;
```

Implement `Tracker<YourScan>` directly when the backend needs different input,
retained state, identity, history, birth/death logic, or output extraction.
Return labeled or unlabeled `TrackReport` values so the runner can apply the
metrics that match those outputs.

## Planar reference files

| Part | File |
| --- | --- |
| input conversion and ego context | `crates/fusion/src/tracker/input.rs` and `tracker.rs` |
| state prediction | `crates/fusion/src/tracker/planar_state.rs` |
| pair hypotheses and sensor coverage | `crates/fusion/src/tracker/planar_model.rs` |
| initiation | `crates/fusion/src/tracker/planar_initiator.rs` |
| association, reduction, lifecycle, and manager | `crates/fusion-tracking/src/` |
| MCAP and history output | `crates/fusion/src/tracker.rs` and `bundle.rs` |
| scoring and display | `crates/fusion/src/eval.rs` and `viz.rs` |

The planar runner calls the whole-tracker API. Its built-in backend is
`TrackManager`; the run loop does not call manager internals. Manager-specific
pair hypotheses remain available for the reference dashboard and history
files.
