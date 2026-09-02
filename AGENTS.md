# Development instructions

## Visual verification

Changes to the Rerun dashboard, scenario behavior, sensor timing, or beginner
experiments require visual verification. Passing tests and reading the `.rrd`
schema are not substitutes for looking at the dashboard.

Keep visual QA artifacts under `runs/visual-qa/`. This directory is ignored by
Git. Do not commit screenshots or recordings unless the user asks for them.

### Tools

Use the Rerun Viewer MCP for experiment QA. It can control the timeline,
interact with views, and save screenshots from a running native or headless
viewer.

Register it with Codex if it is not already available:

```sh
codex mcp add rerun -- rerun viewer-mcp
```

Restart Codex after registering it. Start the recording in a separate viewer:

```sh
rerun runs/run001/reports/baseline/visualization.rrd \
  --headless \
  --window-size 1920x1200
```

The viewer needs access to a working graphics adapter, even in headless mode.
Request the required execution permission if the sandbox blocks it.

For a quick layout smoke test, a single screenshot is acceptable:

```sh
rerun runs/run001/reports/baseline/visualization.rrd \
  --headless \
  --screenshot-to runs/visual-qa/layout.png \
  --window-size 1920x1200
```

This command usually captures the viewer near startup. It does not select a
stable experiment timestamp and may include loading notifications. Do not use
it as the only evidence for a temporal experiment.

If Viewer MCP is unavailable, the Rerun web viewer plus browser automation is
an acceptable fallback. The fallback must support selecting exact timeline
positions. If neither tool can control time, report that temporal visual QA was
not completed.

Before taking evidence screenshots:

- Check `rerun --version` against the Rerun SDK version in `Cargo.lock`.
- Wait for the recording and blueprint to finish loading.
- Pause playback.
- Select the `time` timeline.
- Dismiss or wait out notifications unless notifications are under test.
- Use the same window size for every comparison.

### Layout verification

Capture one full-dashboard screenshot at 1920x1200 for a layout-only change.
Inspect it at full resolution. Capture focused views as well if important
details are too small in the full dashboard.

Check all of the following:

- The map frames the full path and landmarks.
- Truth and estimate remain distinguishable where they overlap.
- Scenario, motion, and guide text is formatted, readable, and not clipped.
- Camera bearings are visible and are not presented as depth.
- Lidar returns, scan-time color, and platform motion are large enough to read.
- Error, timing, IMU, and observation plots have readable legends and scales.
- The timeline does not consume enough height to make the plots unusable.
- No loading warning, version warning, empty panel, or debug UI obscures content.

A screenshot is evidence, not a checkbox. Record visible problems even when
they are outside the code changed in the current task.

### Temporal experiment verification

One screenshot is never enough for a motion or timing experiment. Capture a
minimum of five ordered checkpoints:

1. early straight motion;
2. entry into a turn;
3. middle of the turn;
4. exit from the turn; and
5. the final or recovery segment.

Choose checkpoints from `scenario.resolved.yaml` and the trajectory segment
boundaries. Record exact Rerun timeline times. Do not choose frames only because
they look interesting.

Use filenames that preserve order and context:

```text
runs/visual-qa/camera-latency/
  baseline_t002000ms_straight.png
  baseline_t004500ms_turn-entry.png
  baseline_t005500ms_mid-turn.png
  variant_t002000ms_straight.png
  variant_t004500ms_turn-entry.png
  variant_t005500ms_mid-turn.png
```

Inspect every screenshot individually at full resolution. A contact sheet is
useful for finding broad differences, but it is not a substitute for inspecting
the source images.

Compare a baseline and variant at matched physical checkpoints:

- For speed experiments, match trajectory segment and path progress, not wall
  clock time. A 2x run reaches the same pose in half the time.
- For camera-latency experiments, include a frame before receipt, at or just
  after receipt, and after the estimator response when possible. Check the
  timing plot as well as the map error.
- For lidar scan-duration experiments, include a turning segment. Maximize the
  lidar view for at least one frame so scan order and platform motion during the
  scan are readable.

Use metrics and resolved scenarios for numeric claims. Use screenshots for
layout, visibility, timing relationships, motion progression, and unexpected
visual behavior. Do not estimate a precise RMSE or latency from plot pixels.

### Reasoning over several images

Treat screenshots as an ordered sequence of observations:

1. Verify the timestamp and trajectory segment shown in each image.
2. Describe what changed since the previous checkpoint.
3. Compare the same panel and physical checkpoint between baseline and variant.
4. Separate expected changes from unrelated layout or scaling changes.
5. Check whether the final frame shows recovery, accumulated error, or neither.

Do not rely on memory of an earlier image when a direct side-by-side or contact
sheet can be created. Keep filenames and captions explicit so images cannot be
silently compared at different path phases.

### Video

Video is secondary evidence. It is useful for spotting transitions, flicker,
autoscale jumps, and short-lived failures, but an agent should not base its
conclusion on watching a video once.

When a video is available:

- note its playback rate and the Rerun timeline it represents;
- extract or recapture frames at the planned checkpoints;
- add extra frames around any transient found in the video;
- inspect those frames individually; and
- cite the frame times in the report.

Do not treat video playback time as simulation time unless they are explicitly
mapped. Prefer screenshots selected directly on the Rerun timeline.

### Visual QA report

Write `runs/visual-qa/<experiment>/REPORT.md` with:

- the run directories and scenario difference;
- Rerun SDK and viewer versions;
- window size;
- a table of screenshot path, exact timeline time, trajectory segment, and why
  that checkpoint was selected;
- expected behavior;
- what was actually visible in each dashboard panel;
- discrepancies and follow-up work; and
- the metric comparison output.

Do not conclude with only “looks good.” State what was checked and what the
images showed.

