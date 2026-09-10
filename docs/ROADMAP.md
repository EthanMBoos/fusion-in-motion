# Roadmap

Build a fast Rust engine for state-estimation and tracking experiments. Run
simulated or recorded data and inspect the included planar reference in Rerun.
The goal is a Rust sister project to
[Stone Soup](https://github.com/dstl/Stone-Soup).

## 1. Add recorded inputs

Run the built-in Rust estimator and tracker from a completed measurement file.
Create a run from recorded data without a simulated scenario.

Start with a short sequence of detections from a ROS bag, MCAP file, or public
dataset. Check several timestamps, positions, and coordinate conversions by
hand before running the full sequence.

Add Python camera and lidar frontends afterward. Python handles video and
point-cloud decoding, model inference, and calibration. Rust receives the
observations. Report fixed-detection tracking separately from the full
detector-to-track result.

## 2. Harden the reusable tracking engine

Keep schemas, sensor names, coordinate systems, and visualization out of
`fusion-tracking`. Test the interfaces through the planar implementation. Put
large custom scenarios, sensor configurations, and their models in their own
repositories.

- Document the observation, hypothesis, association, reduction, initiation,
  and lifecycle interfaces with one small downstream example.
- Add public API tests for alternative caller-owned state and observation
  types without adding another built-in domain.
- Stabilize diagnostics and history ordering needed for reproducible studies.
- Publish reviewed revisions that downstream repositories can pin.

## 3. Finish evaluation and benchmarks

Implement the tracking measures listed in [`DEMOS.md`](DEMOS.md). Give every
measure a small case with an answer that can be checked by hand.

Run one recorded navigation benchmark and one recorded tracking benchmark.
Record the sequence, frame range, coordinate and timestamp conversions, adapter
version, and input checksum.

Compare the Rust core with equivalent Stone Soup cases at small, medium, and
large sizes. Report update time, full-run time, events per second, and peak
memory. Measure Rerun recording separately.

Measurement-file replay and tracking measures come next. Add recorded
benchmarks after those work. Expand domain-specific models in downstream
repositories once the run and result formats are stable.
