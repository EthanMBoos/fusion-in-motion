# Remaining student-readiness work

Reviewed against the current working tree. Completed findings have been removed.

## Priority

Before students use results for graded work:

1. Add a complete external-estimator example if students will write their own
   estimators.
2. Run the starter guide with students who did not build the repository.

Add the outlier lesson when the course reaches residuals, gating, and fault
handling.

## 1. Add one external-estimator example

Writing estimate CSV is straightforward. Reading `measurements.mcap` requires
MCAP and its embedded Protobuf schemas, but the repository has no reader,
generated bindings, or complete example.

- Provide a small Python package or self-contained reader.
- Include one estimator that reads `measurements.mcap` without opening
  `truth.mcap`.
- Show the run, estimate, score, and view commands.
- Add covariance-capable output if external estimators need consistency
  scoring; the CSV importer cannot carry covariance.

## 2. Add the outlier and gating lesson

The baseline reports attempted, applied, and invalid scalar updates. The
remaining fault injection, normalized innovations, gating decisions, and
recovery metrics are specified in [Demo 5](DEMOS.md#5-one-false-constraint-can-break-the-solution).

## 3. Run a student pilot

Run the starter guide with at least three students who did not build the
repository. Record incorrect interpretations of the experiment and dashboard,
not only command failures, then revise the guide from those observations.

## Relevant references

| Work | Reference |
| --- | --- |
| Consistency and runtime evaluation | [OpenVINS evaluation guide](https://docs.openvins.com/evaluation.html) |
| Multi-seed consistency analysis | [navlie](https://github.com/decargroup/navlie) |
| External planar estimator | [GTSAM Pose2 tutorial](https://gtsam.org/tutorials/intro.html) |

## Repository evidence

| Work | Current implementation |
| --- | --- |
| Metrics and sweeps | [`eval.rs`](../crates/fusion/src/eval.rs), [`sweep.rs`](../crates/fusion/src/sweep.rs) |
| External estimates | [`ESTIMATORS.md`](ESTIMATORS.md), [`external.rs`](../crates/fusion/src/external.rs) |
