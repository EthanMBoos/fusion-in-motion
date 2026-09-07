#!/usr/bin/env python3

import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory
from ruamel.yaml import YAML
from stonesoup.base import Property
from stonesoup.dataassociator.neighbour import GNNWith2DAssignment
from stonesoup.deleter.time import UpdateTimeDeleter
from stonesoup.hypothesiser.distance import DistanceHypothesiser
from stonesoup.initiator.simple import SimpleMeasurementInitiator
from stonesoup.measures import Mahalanobis
from stonesoup.models.base import TimeVariantModel
from stonesoup.models.measurement.nonlinear import CartesianToBearingRange
from stonesoup.models.transition.linear import LinearGaussianTransitionModel
from stonesoup.predictor.kalman import KalmanPredictor
from stonesoup.types.angle import Bearing
from stonesoup.types.array import CovarianceMatrix
from stonesoup.types.detection import Detection
from stonesoup.types.state import GaussianState
from stonesoup.updater.kalman import ExtendedKalmanUpdater


EPOCH = datetime(2000, 1, 1, tzinfo=timezone.utc)


class DiscreteAccelerationTransition(LinearGaussianTransitionModel, TimeVariantModel):
    acceleration_noise_stddev_mps2: float = Property(
        doc="Standard deviation of one acceleration sample"
    )

    @property
    def ndim_state(self):
        return 4

    def matrix(self, time_interval, **kwargs):
        dt = time_interval.total_seconds()
        return np.array(
            [
                [1.0, dt, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, dt],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

    def covar(self, time_interval, **kwargs):
        dt = abs(time_interval.total_seconds())
        variance = self.acceleration_noise_stddev_mps2**2
        block = np.array(
            [
                [0.25 * dt**4, 0.5 * dt**3],
                [0.5 * dt**3, dt**2],
            ]
        ) * variance
        covariance = np.zeros((4, 4))
        covariance[0:2, 0:2] = block
        covariance[2:4, 2:4] = block
        return CovarianceMatrix(covariance)


@dataclass(frozen=True)
class EgoPose:
    time_ns: int
    x_m: float
    y_m: float
    yaw_rad: float


def read_messages(path: Path, schema_name: str):
    with path.open("rb") as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        for record in reader.iter_decoded_messages():
            if record.schema is not None and record.schema.name == schema_name:
                yield record.decoded_message


def read_ego_truth(path: Path):
    poses = []
    for state in read_messages(path, "fusion.EgoTruthState"):
        poses.append(
            EgoPose(
                time_ns=state.time_ns,
                x_m=state.pose_world.position.x,
                y_m=state.pose_world.position.y,
                yaw_rad=state.pose_world.yaw_rad,
            )
        )
    if not poses:
        raise ValueError(f"{path} contains no vehicle truth")
    return poses


def interpolate_ego(poses, time_ns):
    times = [pose.time_ns for pose in poses]
    index = bisect.bisect_left(times, time_ns)
    if index == 0:
        return poses[0]
    if index == len(poses):
        return poses[-1]
    after = poses[index]
    before = poses[index - 1]
    if after.time_ns == time_ns:
        return after
    fraction = (time_ns - before.time_ns) / (after.time_ns - before.time_ns)
    yaw_delta = math.remainder(after.yaw_rad - before.yaw_rad, 2.0 * math.pi)
    return EgoPose(
        time_ns=time_ns,
        x_m=before.x_m + (after.x_m - before.x_m) * fraction,
        y_m=before.y_m + (after.y_m - before.y_m) * fraction,
        yaw_rad=math.remainder(before.yaw_rad + yaw_delta * fraction, 2.0 * math.pi),
    )


def timestamp(time_ns):
    if time_ns % 1_000 != 0:
        raise ValueError("Stone Soup 1.9 timestamps are limited to whole microseconds")
    return EPOCH + timedelta(microseconds=time_ns // 1_000)


def measurement(detection, ego, time):
    model = CartesianToBearingRange(
        ndim_state=4,
        mapping=(0, 2),
        noise_covar=np.diag(
            [detection.bearing_variance_rad2, detection.range_variance_m2]
        ),
        translation_offset=np.array([[ego.x_m], [ego.y_m]]),
        rotation_offset=np.array([[0.0], [0.0], [ego.yaw_rad]]),
    )
    return Detection(
        [[Bearing(detection.bearing_rad)], [detection.range_m]],
        timestamp=time,
        measurement_model=model,
    )


def check_scenario(run):
    yaml = YAML(typ="safe")
    with (run / "scenario.resolved.yaml").open() as stream:
        scenario = yaml.load(stream)
    checks = [
        (not scenario["camera"]["enabled"], "camera must be disabled"),
        (scenario["lidar"]["enabled"], "lidar must be enabled"),
        (scenario["lidar"]["latency_ns"] == 0, "lidar latency must be zero"),
        (scenario["lidar"]["scan_duration_ns"] == 0, "lidar scans must be instantaneous"),
        (
            not scenario["object_tracker"]["timing_compensation"],
            "tracker timing compensation must be disabled",
        ),
        (
            scenario["object_tracker"]["confirmation_hits"] == 1,
            "tracker confirmation_hits must be 1",
        ),
    ]
    for valid, message in checks:
        if not valid:
            raise ValueError(f"unsupported comparison scenario: {message}")
    return scenario["object_tracker"]


def run_tracker(run):
    config = check_scenario(run)
    ego_poses = read_ego_truth(run / "truth.mcap")
    scans = list(read_messages(run / "measurements.mcap", "fusion.LidarScan"))
    if not scans:
        raise ValueError(f"{run / 'measurements.mcap'} contains no lidar scans")

    transition = DiscreteAccelerationTransition(
        config["acceleration_noise_stddev_mps2"]
    )
    predictor = KalmanPredictor(transition)
    updater = ExtendedKalmanUpdater(
        measurement_model=None,
        force_symmetric_covariance=True,
        use_joseph_cov=True,
    )
    hypothesiser = DistanceHypothesiser(
        predictor,
        updater,
        measure=Mahalanobis(),
        missed_distance=config["gate_sigma"],
    )
    associator = GNNWith2DAssignment(hypothesiser)
    deleter = UpdateTimeDeleter(
        time_since_update=timedelta(seconds=config["max_time_without_update_s"])
    )
    initiator = SimpleMeasurementInitiator(
        prior_state=GaussianState(
            [[0.0], [0.0], [0.0], [0.0]],
            np.diag([0.0, 4.0, 0.0, 4.0]),
        )
    )

    tracks = set()
    track_ids = {}
    next_track_number = 1
    frames = []
    previous_time_ns = None

    for scan in scans:
        measurement_time_ns = scan.time.measurement_time_ns
        available_time_ns = scan.time.arrival_time_ns
        if previous_time_ns is not None and measurement_time_ns < previous_time_ns:
            raise ValueError("lidar measurement times must not go backward")
        previous_time_ns = measurement_time_ns
        time = timestamp(measurement_time_ns)
        detections = set()
        for detection in scan.detections:
            if detection.measurement_time_ns != measurement_time_ns:
                raise ValueError("all lidar returns must use the scan measurement time")
            ego = interpolate_ego(ego_poses, detection.measurement_time_ns)
            detections.add(measurement(detection, ego, time))

        associations = associator.associate(tracks, detections, time)
        associated_detections = set()
        for track in tracks:
            hypothesis = associations[track]
            if hypothesis:
                track.append(updater.update(hypothesis))
                associated_detections.add(hypothesis.measurement)
            else:
                track.append(hypothesis.prediction)

        tracks -= deleter.delete_tracks(tracks, timestamp=time)
        new_tracks = initiator.initiate(detections - associated_detections, time)
        for track in sorted(
            new_tracks,
            key=lambda value: (float(value.state_vector[0]), float(value.state_vector[2])),
        ):
            track_ids[track.id] = f"track-{next_track_number:03}"
            next_track_number += 1
        tracks |= new_tracks

        output_tracks = []
        for track in sorted(tracks, key=lambda value: track_ids[value.id]):
            state = np.asarray(track.state_vector).reshape(-1)
            output_tracks.append(
                {
                    "track_id": track_ids[track.id],
                    "x_m": state[0],
                    "y_m": state[2],
                    "vx_mps": state[1],
                    "vy_mps": state[3],
                }
            )
        frames.append((measurement_time_ns, available_time_ns, output_tracks))
    return frames


def write_tracks(path, frames):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "estimate_time_ns",
                "available_time_ns",
                "track_id",
                "x_m",
                "y_m",
                "vx_mps",
                "vy_mps",
            ]
        )
        for estimate_time_ns, available_time_ns, tracks in frames:
            if not tracks:
                writer.writerow([estimate_time_ns, available_time_ns, "", "", "", "", ""])
                continue
            for track in tracks:
                writer.writerow(
                    [
                        estimate_time_ns,
                        available_time_ns,
                        track["track_id"],
                        track["x_m"],
                        track["y_m"],
                        track["vx_mps"],
                        track["vy_mps"],
                    ]
                )


def main():
    parser = argparse.ArgumentParser(
        description="Run Stone Soup over a completed Fusion in Motion lidar experiment"
    )
    parser.add_argument("run", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    frames = run_tracker(args.run)
    write_tracks(args.output, frames)
    print(f"Stone Soup wrote {len(frames)} frames to {args.output}")


if __name__ == "__main__":
    main()
