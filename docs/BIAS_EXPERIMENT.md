# IMU bias experiment

This experiment is about the vehicle knowing its own position. Better vehicle
motion also gives the object tracker a better reference.

An IMU contains an accelerometer, which measures how motion is changing, and a
gyroscope, which measures rotation. Either sensor can be wrong, and that error
can drift. The GPS gives the filter an outside position reference, so it can
correct the vehicle path and learn some of the IMU error.

The baseline estimates the gyroscope z-axis error and accelerometer x-axis
error. The dashboard and report compare those estimates with the simulated
sensor error and show their uncertainty.

Increase `imu.gyro_turn_on_bias_radps.z` or
`imu.accel_turn_on_bias_mps2.x`, then run several seeds. Look for three things:
whether the estimate moves toward the starting error, whether it follows slow
drift, and whether the truth usually stays inside the reported uncertainty.

Bias is not fully observable in every motion. Stops, acceleration, turns, GPS
rate, and GPS noise all change what the filter can learn. Do not judge coverage
from one run; use a multi-seed sweep.
