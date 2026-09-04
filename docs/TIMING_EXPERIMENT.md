# Timing experiment

Run `fusion run examples/timing.yaml --view`.

Each measurement records when it observed the world and when it arrived. The
starter makes those times equal. This example delays every sensor and gives a
lidar scan a duration.

Set `timing_compensation` to `false` for both estimators and run it again. Look
during the turns, where using an old measurement as if it were current causes
the largest visible error. The report counts delayed, replayed, and discarded
measurements.
