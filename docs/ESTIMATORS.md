# Using your estimator

An estimator can run in any language or process. Give it a run's
`measurements.mcap`, export its planar estimates as CSV, and ask Fusion in
Motion to score the result against the hidden truth.

Required columns:

```text
estimate_time_ns,x_m,y_m,yaw_rad
```

Optional columns:

```text
emission_time_ns,vx_mps,vy_mps,status
```

`status` may be `VALID`, `INITIALIZING`, or `DIVERGED` and defaults to `VALID`.
`emission_time_ns` defaults to `estimate_time_ns`, and velocity defaults to
zero. Times must be strictly increasing. The file uses simple unquoted numeric
fields rather than a general CSV dialect.

Example:

```csv
estimate_time_ns,emission_time_ns,x_m,y_m,yaw_rad,vx_mps,vy_mps,status
10000000,12000000,0.0,0.0,0.0,0.0,0.0,INITIALIZING
20000000,23000000,0.001,0.0,0.0002,0.10,0.0,VALID
```

Score it with:

```sh
fusion score runs/initial my-estimates.csv --id my-filter
```

This writes `estimates/my-filter.mcap` and a report under
`reports/my-filter/`. The estimator is never given `truth.mcap`; the scoring
command opens truth only after the estimate CSV exists.

Open the synchronized dashboard with the imported estimate:

```sh
fusion view runs/initial --estimator my-filter
```

Availability currently assumes an output cadence comparable to the IMU/truth
cadence. Accuracy and time coverage remain useful for lower-rate estimators,
but their availability number will be conservative.
