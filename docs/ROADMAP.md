# Roadmap

Build a fast Rust simulator for 2D and 3D state-estimation and tracking
experiments. Run simulated or recorded data and inspect the results in Rerun.
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

## 2. Add 3D

Leave the planar API small. Add separate 3D types for pose, motion,
observations, and covariance.

- Define and test the coordinates and camera geometry.
- Add a small 3D truth path and camera observations.
- Add 3D scoring and a Rerun view.
- Build the first 3D lesson.

Start with a drone and camera. State whether the demo estimates the drone, an
object, or both. A monocular camera does not provide absolute distance from one
image. Get scale from motion, known geometry, altitude, or a prior.

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
benchmarks after those work. Start 3D once the run and result formats are
stable.
