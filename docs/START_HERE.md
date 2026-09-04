# Start here

Run the starter:

```sh
fusion run experiments/initial.yaml --view
```

## What the dashboard shows

The large map follows the vehicle and two objects. Green is truth. Pink is the
vehicle position estimated from GPS and IMU. Yellow dots are GPS fixes.

Orange and purple are estimates of the objects. Orange uses the estimated
vehicle pose. Purple uses the true vehicle pose as a control. Both receive the
same detections. The gap between them shows how an error in the vehicle pose
moves an object track.

The detections do not say which object they came from. The tracker assigns them
to its own `track-001`, `track-002`, and so on.

The camera view uses equal-length cyan rays because the camera gives direction,
not distance. Lidar gives both, so its blue rays end at the measured distance.
The plots show vehicle error, object error, IMU readings, and detection counts.

Drag through the left turn from 5 to 7 seconds. A small heading error moves a
distant object sideways.

The comments in [`initial.yaml`](../experiments/initial.yaml) give you the first
sensor settings to change. Run after each edit and compare the dashboards.

Reopen any run with:

```sh
fusion view runs/run001
```

Use `fusion compare runs/run001 runs/run002` when you want the metric difference.
