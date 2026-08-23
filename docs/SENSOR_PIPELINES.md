# Sensor pipeline work

The current simulator models measurement time, receipt time, sensor latency,
and lidar scan duration. It processes analytic observations offline. It does
not measure thread scheduling, queue pressure, decoding time, memory bandwidth,
or real sensor payload throughput.

Those runtime questions belong around the two existing paths:

```text
GPS + IMU -> vehicle estimator
camera + lidar -> perception frontend -> object tracker
```

A future runtime experiment should replay the same saved measurements through
bounded queues and configurable workers. It should record queue time,
processing time, measurement age, dropped data, and deadline misses alongside
vehicle and object error.

Camera and lidar frontend load must never become a reason to send their object
detections into the vehicle EKF. Runtime coupling and estimator data flow are
different questions.

Start with a deterministic queue model. Add real multithreaded replay only when
the experiment needs operating-system and memory effects. Raw images and point
clouds can be attached later without changing the compact detection API used by
the tracker.
