# IMU bias experiment

This simulator is meant for both finding the robot's own position and tracking
other objects. This experiment only covers the first: how well the robot
estimates its own motion and sensor errors. It does not track another moving
object.

An IMU combines an accelerometer, which measures how the robot's motion is
changing, and a gyroscope, which measures how the robot is turning.

The sensor can be wrong, and its error can drift over time. That error is its
bias.

The baseline filter estimates gyroscope z bias, which affects left or right
turning, and accelerometer x bias, which affects forward acceleration.

The filter starts with both offsets set to zero. The simulator gives the IMU
nonzero offsets, then the filter uses camera and lidar measurements to learn
them.

Run the same experiment ten times with different noise:

```sh
fusion sweep examples/bias_sweep.yaml --output runs/bias-sweep
```

Read `runs/bias-sweep/reports/summary.md`, then inspect one run:

```sh
fusion view runs/bias-sweep/case-0000
```

## What to look for

Green is the actual simulated offset, which the filter cannot see. Pink is the
filter's estimate. The estimate starts at zero and should move toward the green
line, then follow its slow changes.

In `case-0000`, look at 0.5 s to see the filter begin learning both offsets. By
4 s, the green and pink lines should be close. From 5–7 s, the robot turns and
the gyroscope estimate should keep tracking without a jump. At 15.5 s, both
estimates should remain close during the final rest.

The normalized-error panels compare the filter's error with its own uncertainty.
The line should usually stay between `-1.96` and `+1.96`. An occasional crossing
is normal. Frequent crossings mean the filter is more confident than it should
be.

## Read the results

The sweep runs the experiment with ten different noise patterns. One run can
look good by luck, so use the sweep report for the main conclusion. RMSE is the
typical size of the bias error, and smaller is better. Final error says how far
off the estimate is at the end. Coverage says how often the error stayed inside
the filter's expected range; the target is about 95%. Normalized ANEES is
another check of the filter's uncertainty; the target is about 1.

Each estimate is compared with the simulated bias from the same IMU timestamp.
The hidden values remain in `truth.mcap`; the filter never reads them.

The simulator and filter currently use the same setting for how quickly bias
can drift. A later exercise should separate those settings so students can see
what happens when the filter expects too little or too much drift.
