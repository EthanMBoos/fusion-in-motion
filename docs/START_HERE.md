# Start here

Run the starter scenario:

```sh
fusion run examples/initial.yaml --view
```

The run is saved as `runs/run001`. Future runs use `run002`, `run003`, and so
on. Reopen one with:

```sh
fusion view runs/run001
```

## Read the dashboard

The large map contains two separate results. Pink is the GPS/IMU estimate of
the vehicle. Orange and purple are estimates of the other objects.

Purple object tracks use the true vehicle pose. Orange tracks use the GPS/IMU
vehicle estimate. Both receive the same camera and lidar detections. A gap
between orange and purple shows how vehicle-position error entered the object
track.

The camera panel draws equal-length rays because the camera reports direction,
not distance. The lidar panel shows range and direction. The plots below the
sensor views show vehicle error, object error, IMU readings, GPS delay, and the
number of detections. The two bias plots compare the IMU's simulated error with
the error learned by the vehicle filter.

Drag the timeline through the first left turn from 5 to 7 seconds. Heading
error is easiest to see there, and a small heading error moves distant object
tracks sideways.

## Try the important changes

Make one edit in `examples/initial.yaml`, run it again, and compare the new run
with the previous one:

```sh
fusion compare runs/run001 runs/run002
fusion view runs/run002
```

First set `gps.enabled` to `false`. The IMU still follows short-term motion, but
its position drifts because nothing pulls it back to the local-world position.
The orange object tracks should get worse with it. Set GPS back to `true` before
continuing.

Next change `gps.horizontal_position_stddev_m` from `0.25` to `1.0`. The vehicle
estimate and orange object tracks should become noisier. Purple is the useful
control: its detections did not change and it does not use the GPS/IMU result.

Set `camera.enabled` to `false`. Lidar can still start and update tracks because
it measures distance. Put the camera back, then set `lidar.enabled` to `false`.
The current camera-only path reports detections but does not start a track from
a single direction measurement. Multi-view camera initialization is future
drone work.

Finally, restore both sensors and change their noise or detection probability.
Only the object tracks should move. The vehicle estimate must stay exactly the
same because camera and lidar do not enter its filter.

For repeatable multi-seed comparisons, use the examples in
[EXPERIMENTS.md](EXPERIMENTS.md).
