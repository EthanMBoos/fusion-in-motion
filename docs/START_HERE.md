# Start here

Run the starter:

```sh
fusion run examples/initial.yaml --view
```

## What the dashboard shows

The large map follows the vehicle and two objects. Green is truth. Pink is the
vehicle position estimated from GPS and IMU. Yellow dots are GPS fixes.

Orange and purple are estimates of the objects. Orange uses the estimated
vehicle pose. Purple uses the true vehicle pose as a control. Both receive the
same detections. The gap between them shows how an error in the vehicle pose
moves an object track.

The camera view uses equal-length cyan rays because the camera gives direction,
not distance. Lidar gives both, so its blue rays end at the measured distance.
The plots show vehicle error, object error, IMU readings, and detection counts.

Drag through the left turn from 5 to 7 seconds. A small heading error moves a
distant object sideways.

## Three useful edits

Edit `examples/initial.yaml`, run it again, and compare the dashboard with the
previous run.

First set `gps.enabled` to `false`. The IMU can follow short motion, but position
drifts without GPS corrections. The orange object tracks move with that error.

Restore GPS, then change `gps.horizontal_position_stddev_m` from `0.25` to
`1.0`. The vehicle estimate and orange tracks become noisier. Purple remains the
control.

Restore the GPS noise, then disable either camera or lidar. Lidar can create a
track because it measures distance. A single camera direction cannot create a
track by itself, but camera measurements can update a track that lidar already
started.

Reopen any run with:

```sh
fusion view runs/run001
```

Use `fusion compare runs/run001 runs/run002` when you want the metric difference.
