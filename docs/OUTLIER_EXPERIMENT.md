# GPS outlier experiment

Run `fusion run examples/outliers.yaml --view`.

Most GPS fixes have ordinary noise. Some are deliberately moved several meters
away. `ego_estimator.gps_gate_sigma` limits how surprising a fix may be before
the filter skips it.

Compare the yellow GPS dots with the pink vehicle estimate, then open the run
summary and check the applied and rejected GPS counts. Raise the gate to admit
more bad fixes; lower it far enough and useful fixes will also be skipped.
