# Stone Soup tracker comparison

This runs Stone Soup 1.9.1 over the lidar measurements from a completed
experiment. Stone Soup uses the true vehicle pose, then the normal Rust scorer
compares its tracks with object truth.

Install [`uv`](https://docs.astral.sh/uv/getting-started/installation/) and run:

```sh
fusion run experiments/tracker_update.yaml
# Use the new run folder created above.
./scripts/run-stone-soup.sh runs/runNNN
fusion view runs/runNNN --force
```

The first Stone Soup run creates its Python environment from the committed
lockfile. The result is saved as `tracks/stone-soup.mcap`, its metrics are under
`reports/stone-soup`, and the rebuilt dashboard shows the track in red.

This first comparison is lidar only, uses the true vehicle pose, creates tracks
on the first detection, and does not cover delayed or rolling scans. The two
trackers use the same measurements, motion noise, and gate. Their initialization
and EKF implementations are independent, so their results should be close but
not identical.

Stone Soup reads vehicle truth to place each relative lidar return in the world.
It does not read object truth. Object truth is only used later by `fusion score
tracks`.
