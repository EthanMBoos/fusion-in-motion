# Sensor pipelines and estimator performance

Fusion results depend on more than the estimator equations. A technique can be
mathematically appropriate and still perform poorly because measurements reach
it late, arrive out of order, or are dropped under load.

This matters when a platform carries high-throughput sensors such as cameras,
lidars, and imaging radars. A 20 Hz lidar may publish several megabytes per
scan, while multiple cameras publish images at 30 Hz or faster. The estimator
may only need compact features from those sensors, but the system must first
receive, move, decode, and preprocess the raw data.

The playground should make it possible to distinguish two failures:

1. the estimator does not use the available measurements well; and
2. the runtime does not deliver usable measurements to the estimator in time.

The current implementation only tests the first case. It generates all
measurements, writes them to MCAP, reads them back, and processes them
sequentially. Sensor latency changes the simulated receipt timestamp, but no
thread actually blocks and no queue becomes full. The current results therefore
do not characterize transport throughput or runtime architecture.

## Keep truth separate from execution

Sensor generation should remain deterministic. A scenario should produce the
same timestamped workload for every pipeline being compared:

```text
ground truth
    |
    v
sensor generation
    |
    v
fixed camera, lidar, radar, and IMU workload
    |
    +---> shared single-thread pipeline
    +---> shared worker pool
    +---> dedicated sensor workers
    +---> separate frontends and ordered fusion
```

Keeping the input fixed makes the comparison understandable. If estimator
accuracy changes between runs, the experiment can connect that change to
delivery latency, queueing, dropped messages, or processing order rather than
to a different random sensor realization.

## Where concurrency belongs

A practical sensor-fusion system often resembles:

```text
lidar receiver  -> lidar queue  -> point-cloud frontend --+
camera receiver -> camera queue -> vision frontend --------+-> ordered fusion
radar receiver  -> radar queue  -> radar frontend ----------+
IMU receiver    -> priority queue --------------------------+
```

Receiving and preprocessing independent sensors are natural concurrent stages.
The final estimator update is commonly serialized because every update changes
the same state and covariance.

This does not imply that every sensor needs its own thread. Useful alternatives
include:

- one shared queue and worker;
- one shared bounded worker pool;
- separate sensor queues feeding a shared pool;
- dedicated workers for expensive frontends; and
- separate processes or hardware-accelerated frontends.

The topology, queue sizes, and worker counts are experiment settings. They
should not be baked into the sensor or estimator math.

Different choices expose different failure modes. A shared queue can cause a
large lidar job to delay camera and IMU work. A shared worker pool can use CPU
efficiently but allow large jobs to occupy every worker. Dedicated workers
provide isolation but may leave cores idle. Adding too many threads can make
performance worse through scheduling, synchronization, cache pressure, and
memory contention.

## Two useful execution modes

The project should eventually support a deterministic runtime model and a real
multithreaded replay. They answer different questions.

### Deterministic runtime model

A single-threaded event simulation can model:

- sensor publication rate and payload size;
- fixed transport latency and link bandwidth;
- serialization and deserialization time;
- queue capacity and queueing policy;
- frontend processing time and worker availability;
- estimator processing time; and
- blocking, deadline misses, and message drops.

For example, a stage that takes 60 ms to process every 50 ms lidar scan cannot
keep up indefinitely. An event simulation can expose the growing queue and
measurement age without waiting in real time. It is fast, repeatable, and well
suited to large parameter sweeps.

This mode predicts behavior from configured costs. It cannot measure operating
system scheduling, memory allocation, cache effects, lock contention, or the
actual performance of a Rust channel or serializer.

### Multithreaded replay

A real-time backend should replay the same saved workload through actual
bounded queues and configurable workers:

```text
timed sensor playback
        |
        v
bounded input queues
        |
        v
decode and preprocessing workers
        |
        v
ordered estimator input
        |
        v
state estimate
```

This mode should move real payload bytes, run according to wall-clock time, and
repeat each case several times. Thread scheduling is nondeterministic, so one
run is not enough to support a performance conclusion.

The first version only needs configurable worker counts. CPU affinity,
real-time priorities, NUMA placement, GPU execution, and distributed middleware
can wait until simpler experiments show that they matter.

## Raw payloads and fusion observations

The current analytic camera, lidar, and radar messages are small. They resemble
the observations that a perception frontend would hand to the estimator, not
the large raw data that arrived from the sensor.

The pipeline experiment should make this boundary visible:

```text
raw image       -> detection and association -> camera observation
raw point cloud -> filtering and association -> lidar observation
raw radar data  -> detection and tracking    -> radar observation
```

Initially, a raw message can contain a synthetic byte buffer of a configured
size. Moving and serializing that buffer exercises allocation, copying, memory
bandwidth, and queue pressure without requiring a photorealistic camera or a
complete point-cloud renderer. Configurable frontend work can stand in for
decoding and feature extraction.

Later, representative images and point clouds can replace synthetic payloads
without changing the surrounding pipeline experiment.

Merely attaching a `payload_size_bytes` number to a tiny analytic observation
is still useful in the deterministic model, but it is not sufficient for the
multithreaded benchmark. A label does not create real memory or serialization
pressure.

## What to record

Each message should carry enough timing information to reconstruct its path:

- measurement time at the sensor;
- publication time;
- transport arrival time;
- queue entry time;
- processing start and finish times;
- time applied by the estimator;
- payload size;
- queue depth; and
- whether and why the message was dropped.

The resulting reports should include:

- end-to-end latency percentiles;
- measurement age when fused;
- time spent waiting in queues;
- deadline-miss and drop rates;
- sustained message and byte throughput;
- frontend and estimator processing time; and
- estimation error alongside the runtime measurements.

The important connection is:

```text
pipeline architecture
        -> delivery timing and lost data
        -> estimator accuracy and availability
```

A useful result might say:

> On the tested eight-core machine, a shared two-worker frontend delayed camera
> observations when 4 MB lidar scans arrived at 20 Hz. Separate camera and
> lidar workers eliminated camera deadline misses and reduced position error.

That is a statement about a particular workload, implementation, and machine.
It should not be generalized to every platform without another measurement.

## Suggested implementation order

1. Add a deterministic queue and processing-time model around the existing
   generated measurement stream.
2. Report queue delay, measurement age, drops, deadline misses, and estimator
   error together.
3. Add synthetic raw payload sizes and a wall-clock replay using bounded Rust
   channels.
4. Make queue topology and preprocessing worker counts configurable.
5. Compare shared and isolated sensor pipelines using the same saved workload.
6. Add representative image or point-cloud processing only when synthetic data
   no longer answers the experiment.

This approach adds genuine multithreaded measurements where they are useful
while keeping truth generation, scoring, and the estimator equations simple
and reproducible.
