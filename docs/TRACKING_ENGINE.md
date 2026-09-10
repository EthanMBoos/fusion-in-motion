# Use the tracking engine from another repository

`fusion-tracking` runs the tracking cycle. Your code supplies the state,
observations, models, data formats, metrics, and visualization.

`fusion-in-motion` shows one working setup using planar camera bearings, lidar
range and bearing, and planar ego poses.

## Code layout

```text
fusion-tracking
  timestamps and IDs
  hypothesis and association interfaces
  posterior reduction
  track initiation and lifecycle
  track manager, events, and diagnostics

fusion-in-motion
  planar sensor and motion models
  Protobuf and MCAP records
  scenarios, scoring, and Rerun dashboard

your repository
  large custom scenarios and sensor configurations
  your models, schemas, data, metrics, and displays
```

Your repository depends on `fusion-tracking`. `fusion-tracking` does not load
or register project code.

## Why the API stops here

`fusion-tracking` has no dependencies. Using it does not pull in the simulator,
Protobuf, MCAP, Rerun, scenario YAML, or the planar camera and lidar code.

Keep large custom scenarios and sensor configurations in their own
repositories. Those projects depend on the tracking engine and implement the
API with their own types. The engine does not need a feature, registry entry,
or branch for each sensor setup.

The state, measurement, and context types are generic. The same manager can run
with bearing, range, point, box, or other measurement models. Each project owns
the sensor math and decides what a track state contains.

## What happens for each batch

`TrackManager::process_batch` does this:

1. Check the observation IDs.
2. Predict each track to each observation's measurement time.
3. Build every track/observation hypothesis with `HypothesisModel`. The manager
   adds the track ID, observation ID, and predicted state.
4. Run `AssociationEngine`.
5. Reject plans that use missing, gated, or invalid hypotheses.
6. Propagate every selected posterior and the missed-detection prediction to
   one reduction time, then call `PosteriorReducer`.
7. Record track hits and misses.
8. Pass unused observations to `Initiator`.
9. Confirm and delete tracks through `LifecyclePolicy`.
10. Return confirmed tracks at the batch output time.

The manager stages the complete batch before changing its tracks, initiator,
or lifecycle policy. An error leaves those values at the end of the previous
successful batch, so the caller can inspect the error or retry.

Measurement time and arrival time are separate. Prediction and update use
measurement time. Arrival time is available for latency and replay logic. The
batch also has a measurement time so an empty scan can advance track deletion.

## Interfaces to implement

| Interface | What your code provides |
| --- | --- |
| `TimedObservation<D, C>` | Measurement `D`, context `C`, ID, and measurement time |
| `HypothesisModel<S, D, C>` | State prediction, gating, and one possible posterior |
| `AssociationEngine<S>` | Hard assignments or per-track marginal probabilities |
| `PosteriorReducer<S>` | One selected posterior or a common-time reduction of weighted posteriors |
| `Initiator<S, D, C>` | New states from unused observations |
| `LifecyclePolicy<S>` | Confirmation and deletion rules |

The initiator and lifecycle policy implement `Clone`. The manager runs them on
batch-local copies and keeps the originals when a batch returns an error.

The crate includes:

- gated global-nearest-neighbor assignment using the Hungarian algorithm;
- a reducer for hard assignments; and
- hit-count confirmation with time-since-update deletion.

`HypothesisModel` returns an `ObservationHypothesis`. Its `HypothesisOutcome`
is `Inside` with a posterior, `Outside`, or `Invalid`, so an inside gate cannot
exist without a posterior. `TrackManager` combines that result with its own
track ID, observation ID, and predicted state to form a `PairHypothesis` for
association and diagnostics.

An `AssociationPlan::Hard` assigns each observation to at most one track. An
`AssociationPlan::Marginal` carries missed-detection and observation
probabilities. `TrackManager` checks that each track's probabilities sum to one
and that an observation's total probability does not exceed one. Selected
posteriors may come from different measurement times. Before reduction, the
manager propagates every branch to the latest selected measurement time and
passes that time to `PosteriorReducer`.

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

## Build a downstream project

1. Decode your records into `ObservationBatch` values.
2. Construct the model, associator, reducer, initiator, and lifecycle policy.
3. Feed batches to one `TrackManager` in delivery order.
4. Save or display each `BatchResult` with your own output code.
5. Record the engine commit, scenario configuration, and input checksum.

Keep general changes to time handling, assignment, diagnostics, and performance
in `fusion-tracking`. Keep sensor equations, schemas, data, scenarios, and
results in the downstream repository.

## Planar example

| Part | File |
| --- | --- |
| input conversion and ego context | `crates/fusion/src/tracker/input.rs` and `tracker.rs` |
| state prediction | `crates/fusion/src/tracker/planar_state.rs` |
| pair hypotheses | `crates/fusion/src/tracker/planar_model.rs` |
| initiation | `crates/fusion/src/tracker/planar_initiator.rs` |
| association, reduction, lifecycle, and manager | `crates/fusion-tracking/src/` |
| MCAP and history output | `crates/fusion/src/tracker.rs` and `bundle.rs` |
| scoring and display | `crates/fusion/src/eval.rs` and `viz.rs` |

`fusion-tracking` has no dependencies. The planar Protobuf stays in
`fusion-in-motion`.
