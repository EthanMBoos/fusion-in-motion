# Development instructions

## Backward compatibility

This is a prototype and teaching repo. Do not preserve backward compatibility
unless the user asks for it. Keep the current API and code easy to understand.

When an API, schema, or configuration changes, update the checked-in code,
experiments, docs, and tests together. Remove replaced fields and behavior. Do
not add aliases, migration paths, fallback parsers, or Serde defaults for old
formats. Existing generated runs do not need to keep working.

## Visual verification

Changes to the dashboard, sensor behavior, timing, localization, tracking, or
starter experiment require visual verification. Tests and metrics do not show
whether the dashboard tells the right story.

Store recordings, screenshots, and notes under `runs/visual-qa/`. The directory
is ignored by Git.

### Viewer MCP

Check the Rerun SDK version in `Cargo.lock` and the installed viewer version:

```sh
rerun --version
codex mcp list
```

This repo registers the viewer in `.codex/config.toml`:

```toml
[mcp_servers.rerun]
command = "rerun"
args = ["viewer-mcp"]
startup_timeout_sec = 20
required = true
default_tools_approval_mode = "approve"
```

`approve` lets Codex use the viewer controls without asking before every call.
`auto` can still prompt. `.codex/rules/rerun.rules` separately allows the local
`rerun` shell command used to launch the viewer.

On a new computer or checkout, trust the repository when Codex asks and start a
new session. Codex ignores project `.codex` settings until the repository is
trusted, and MCP changes do not affect a session that is already running. If
`codex mcp list` still does not show `rerun`, check project trust and restart
Codex before changing the checked-in configuration.

Launch the recording in a separate viewer for MCP control:

```sh
rerun runs/run001/reports/baseline/visualization.rrd \
  --headless \
  --window-size 1920x1200
```

The viewer still needs a working graphics adapter in headless mode. If Viewer
MCP is unavailable, use a browser-controlled Rerun web viewer that can select
exact timeline positions. If neither path can control time, report that the
temporal visual check was not completed.

### Before screenshots

- Wait for the recording and blueprint to load.
- Pause playback and select the `time` timeline.
- Dismiss or wait out notifications.
- Use 1920x1200 for matched comparisons.
- Confirm the timestamp shown by the viewer before saving each image.

One screenshot is enough only for a layout-only change. A motion, timing,
localization, or tracking change needs at least five checkpoints. For the
starter scenario use:

| Time | Segment | What to inspect |
| ---: | --- | --- |
| 2.0 s | acceleration | GPS/IMU estimate and first object tracks |
| 5.0 s | turn entry | heading and sideways object error beginning |
| 6.0 s | middle of left turn | largest visible difference between ego sources |
| 7.5 s | turn exit | whether the tracks recover or retain error |
| 16.0 s | final rest | accumulated vehicle and object error |

Add checkpoints immediately before and after a delayed measurement when timing
is under test. For a speed change, match path segment and path progress rather
than wall-clock time.

Use ordered names such as:

```text
runs/visual-qa/initial/
  t002000ms_acceleration.png
  t005000ms_turn-entry.png
  t006000ms_mid-turn.png
  t007500ms_turn-exit.png
  t016000ms_final.png
```

Inspect every source screenshot at full resolution. A contact sheet helps with
side-by-side comparison but does not replace opening each image.

### What must be visible

The full dashboard should show:

- vehicle truth, GPS fixes, and the GPS/IMU estimate;
- stationary and moving object truth;
- orange tracks using estimated ego and purple tracks using truth ego;
- camera direction rays without implied distance;
- lidar range and direction;
- vehicle and object error plots;
- IMU data and detection counts; and
- a readable timeline and guide.

The bias experiment must also show bias truth, estimate, and both uncertainty
bounds. The timing experiment must show GPS age. These panels should not appear
in the starter dashboard.

Camera or lidar changes must not move the vehicle estimate. In matched images,
check that the truth-ego object track remains the control while estimated-ego
track error follows vehicle error. Look closely during turns.

Do not accept empty panels, clipped guide text, unreadable legends, loading
warnings, or auto-scaling that hides the comparison.

### QA report

Write `runs/visual-qa/<experiment>/REPORT.md` with the run path, scenario
changes, Rerun versions, window size, screenshot times, expected behavior,
what each panel showed, metrics, and any remaining problem. Do not conclude
with only "looks good."
