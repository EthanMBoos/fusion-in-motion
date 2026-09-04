# Object association

Run:

```sh
fusion run examples/association.yaml --view
```

The two objects pass close to each other at 5 seconds. Camera and lidar
detections contain measurements, not object names. The tracker must decide
which detection belongs to each track.

At each camera frame or lidar scan, the tracker predicts where its tracks should
be. It removes pairings that are too far from those predictions, then chooses
the lowest-cost one-to-one assignment across the whole frame. This is global
nearest-neighbor association using each track's uncertainty.

Lidar starts a track because it measures direction and distance. A track is
shown after two matching detections. Camera supplies direction updates to
existing tracks. A track is removed after it remains unmatched through three
lidar scans.

Scrub from 4 to 6 seconds. `track-001` and `track-002` should pass each other
without swapping paths. The run report shows associated and unmatched
detections plus tracks created, confirmed, and deleted.

Try lowering `camera.detection_probability` and
`lidar.detection_probability`. Then reduce `object_tracker.gate_sigma`. A narrow
gate rejects more possible matches; a wide gate makes incorrect matches easier
when objects are close.
