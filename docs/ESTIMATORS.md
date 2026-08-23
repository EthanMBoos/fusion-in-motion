# Using another estimator or tracker

The measurement bundle lets another program replace either baseline without
receiving hidden truth.

## Vehicle estimator

Read `measurements.mcap`, consume the IMU and GPS channels, and export:

```text
estimate_time_ns,x_m,y_m,yaw_rad
```

Optional columns are `emission_time_ns`, `vx_mps`, `vy_mps`, `status`,
`gyro_bias_z_radps`, and `accel_bias_x_mps2`.

```sh
fusion score ego runs/run001 my-ego.csv --id my-filter
```

Camera and lidar are present in the bundle for the object tracker. A vehicle
estimator should not consume them in this project.

## Object tracker

Read camera and lidar detections plus an ego-state stream and export:

```text
estimate_time_ns,track_id,association_key,x_m,y_m,vx_mps,vy_mps
```

`emission_time_ns` and `status` are optional.

```sh
fusion score tracks runs/run001 my-tracks.csv --id my-tracker --ego-source estimated
```

Use `--ego-source truth` only when the tracker intentionally used the truth-ego
comparison stream. The normal mode is `estimated`.

Status values are `INITIALIZING`, `VALID`, and `DIVERGED`. CSV times are integer
nanoseconds. Numeric fields must be finite.
