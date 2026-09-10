mod input;
mod planar_initiator;
mod planar_model;
mod planar_state;

use std::collections::{BTreeMap, BTreeSet};

use anyhow::Result;
use fusion_schema::messages::{ObjectTrack, ObjectTrackFrame, Vec2};
use fusion_tracking::{
    AssociationPlan, GateDecision, GlobalNearestNeighbor, HitCountLifecycle, ObservationBatch,
    SinglePosteriorReducer, TimedObservation, TrackManager, TrackManagerDiagnostics, Tracker,
    TrackingOutput,
};
use serde::{Deserialize, Serialize};

pub use input::{EgoHistory, EgoSource, PerceptionMeasurement};

use crate::scenario::{CameraConfig, LidarConfig, ObjectTrackerConfig};
use input::{Detection, DetectionBatch, DetectionContext, DetectionScanContext};
use planar_initiator::PlanarInitiator;
use planar_model::PlanarModel;
use planar_state::{PlanarTrackFilter, snapshot};

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct TrackerDiagnostics {
    pub received_detections: usize,
    pub applied_updates: usize,
    pub waiting_for_range: usize,
    pub missing_ego_pose: usize,
    pub missing_scan_ego_pose: usize,
    pub delayed_detections: usize,
    pub replayed_detections: usize,
    pub discarded_detections: usize,
    pub associated_camera_detections: usize,
    pub associated_lidar_detections: usize,
    pub unmatched_camera_detections: usize,
    pub unmatched_lidar_detections: usize,
    pub candidate_pairs: usize,
    pub gated_out_pairs: usize,
    pub invalid_candidate_pairs: usize,
    pub selected_associations: usize,
    pub missed_updates: usize,
    pub coasted_updates: usize,
    pub created_tracks: usize,
    pub confirmed_tracks: usize,
    pub deleted_tracks: usize,
}

#[derive(Debug)]
pub struct TrackerRun {
    pub frames: Vec<ObjectTrackFrame>,
    pub diagnostics: TrackerDiagnostics,
    pub processed_detections: Vec<String>,
    pub(crate) history: TrackerHistory,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub(crate) enum SensorKind {
    Camera,
    Lidar,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub(crate) struct TrackSnapshot {
    pub position_world_m: [f64; 2],
    pub velocity_world_mps: [f64; 2],
    pub state_covariance: Vec<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "sensor", rename_all = "snake_case")]
pub(crate) enum DetectionRecord {
    Camera {
        bearing_rad: f64,
        bearing_variance_rad2: f64,
    },
    Lidar {
        range_m: f64,
        bearing_rad: f64,
        range_variance_m2: f64,
        bearing_variance_rad2: f64,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub(crate) enum GateResult {
    Inside,
    Outside,
    Invalid,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub(crate) struct AssociationRecord {
    pub batch_id: String,
    pub measurement_time_ns: i64,
    pub arrival_time_ns: i64,
    pub detection_index: usize,
    pub detection: DetectionRecord,
    pub track_id: String,
    pub predicted: Option<TrackSnapshot>,
    pub normalized_innovation_squared: Option<f64>,
    pub gate_threshold_squared: f64,
    pub gate_result: GateResult,
    pub selected: bool,
    pub corrected: Option<TrackSnapshot>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub(crate) enum LifecycleEventKind {
    Created,
    Confirmed,
    Missed,
    Coasted,
    Deleted,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub(crate) struct LifecycleRecord {
    pub measurement_time_ns: i64,
    pub arrival_time_ns: i64,
    pub sensor: SensorKind,
    pub track_id: String,
    pub event: LifecycleEventKind,
    pub detection: Option<DetectionRecord>,
    pub state: TrackSnapshot,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub(crate) struct TrackerHistory {
    pub associations: Vec<AssociationRecord>,
    pub lifecycle: Vec<LifecycleRecord>,
}

pub fn run(
    config: &ObjectTrackerConfig,
    camera_config: &CameraConfig,
    lidar_config: &LidarConfig,
    measurements: &[PerceptionMeasurement],
    ego_history: &EgoHistory,
) -> Result<TrackerRun> {
    input::validate_delivery_order(measurements)?;
    let mut diagnostics = TrackerDiagnostics::default();
    let mut batches = input::flatten(camera_config, lidar_config, measurements)?;
    diagnostics.received_detections = batches.iter().map(|batch| batch.detections.len()).sum();
    input::prepare_timing(config, &mut batches, &mut diagnostics);

    let model = PlanarModel {
        gate_sigma: config.gate_sigma,
        acceleration_noise_stddev_mps2: config.acceleration_noise_stddev_mps2,
    };
    let mut manager = TrackManager::new(
        model,
        GlobalNearestNeighbor {
            missed_assignment_cost: config.gate_sigma.powi(2) + 1.0,
        },
        SinglePosteriorReducer,
        PlanarInitiator {
            minimum_unassigned_probability: 0.5,
        },
        HitCountLifecycle {
            confirmation_hits: config.confirmation_hits,
            max_time_without_update_ns: (config.max_time_without_update_s * 1.0e9).round() as i64,
            hit_probability_threshold: 0.5,
        },
    );
    let mut frames = Vec::with_capacity(batches.len());
    let mut processed_detections = Vec::new();
    let mut history = TrackerHistory::default();
    process_scans(
        batches,
        ego_history,
        &mut manager,
        &mut diagnostics,
        |source, input, output, diagnostics| {
            collect_diagnostics(&input, &output, diagnostics);
            collect_history(&source, &input, &output, config, &mut history);
            frames.push(ObjectTrackFrame {
                estimate_time_ns: output.measurement_time_ns,
                available_time_ns: output.arrival_time_ns,
                tracks: output
                    .tracks
                    .iter()
                    .map(|track| {
                        let track_id = track
                            .identity
                            .track_id()
                            .expect("TrackManager reports labeled tracks");
                        to_track(&track.state, track_id.as_str())
                    })
                    .collect(),
            });
            processed_detections.extend(
                source
                    .detections
                    .iter()
                    .enumerate()
                    .map(|(index, _)| format!("{}:{index}", source.stable_id)),
            );
            Ok(())
        },
    )?;

    Ok(TrackerRun {
        frames,
        diagnostics,
        processed_detections,
        history,
    })
}

fn process_scans<T, F>(
    batches: Vec<DetectionBatch>,
    ego_history: &EgoHistory,
    tracker: &mut T,
    diagnostics: &mut TrackerDiagnostics,
    mut handle_output: F,
) -> Result<()>
where
    T: Tracker<
            ObservationBatch<Detection, DetectionContext, DetectionScanContext>,
            State = PlanarTrackFilter,
        >,
    F: FnMut(
        DetectionBatch,
        ObservationBatch<Detection, DetectionContext, DetectionScanContext>,
        TrackingOutput<PlanarTrackFilter, T::Diagnostics>,
        &mut TrackerDiagnostics,
    ) -> Result<()>,
{
    for source in batches {
        let input = to_core_batch(&source, ego_history, diagnostics);
        let output = tracker.process_scan(&input).map_err(anyhow::Error::new)?;
        handle_output(source, input, output, diagnostics)?;
    }
    Ok(())
}

fn to_core_batch(
    batch: &DetectionBatch,
    ego_history: &EgoHistory,
    diagnostics: &mut TrackerDiagnostics,
) -> ObservationBatch<Detection, DetectionContext, DetectionScanContext> {
    let scan_ego_pose = ego_history.sample(batch.measurement_time_ns);
    if scan_ego_pose.is_none() {
        diagnostics.missing_scan_ego_pose += 1;
    }
    let observations = batch
        .detections
        .iter()
        .enumerate()
        .map(|(index, detection)| {
            let measurement_time_ns = batch.detection_time_ns(detection);
            let ego_pose = ego_history.sample(measurement_time_ns);
            if ego_pose.is_none() {
                diagnostics.missing_ego_pose += 1;
            }
            TimedObservation {
                id: format!("{}:{index}", batch.stable_id).into(),
                measurement_time_ns,
                payload: detection.clone(),
                context: DetectionContext { ego_pose },
            }
        })
        .collect();
    ObservationBatch {
        id: batch.stable_id.clone().into(),
        measurement_time_ns: batch.measurement_time_ns,
        arrival_time_ns: batch.arrival_time_ns,
        context: DetectionScanContext {
            sensor: batch.sensor,
            ego_pose: scan_ego_pose,
            horizontal_fov_rad: batch.horizontal_fov_rad,
            max_range_m: batch.max_range_m,
            detection_probability: batch.detection_probability,
        },
        observations,
    }
}

fn collect_diagnostics(
    core_batch: &ObservationBatch<Detection, DetectionContext, DetectionScanContext>,
    result: &TrackingOutput<PlanarTrackFilter, TrackManagerDiagnostics<PlanarTrackFilter>>,
    diagnostics: &mut TrackerDiagnostics,
) {
    let batch_diagnostics = result.diagnostics.summary;
    diagnostics.candidate_pairs += batch_diagnostics.candidate_pairs;
    diagnostics.gated_out_pairs += batch_diagnostics.gated_out_pairs;
    diagnostics.invalid_candidate_pairs += batch_diagnostics.invalid_candidate_pairs;
    diagnostics.selected_associations += batch_diagnostics.selected_associations;
    diagnostics.missed_updates += batch_diagnostics.missed_updates;
    diagnostics.coasted_updates += batch_diagnostics.coasted_updates;
    diagnostics.created_tracks += batch_diagnostics.created_tracks;
    diagnostics.confirmed_tracks += batch_diagnostics.confirmed_tracks;
    diagnostics.deleted_tracks += batch_diagnostics.deleted_tracks;
    diagnostics.applied_updates +=
        batch_diagnostics.selected_associations + batch_diagnostics.created_tracks;

    let selected = selected_observation_ids(&result.diagnostics.association);
    for observation in &core_batch.observations {
        if selected.contains(observation.id.as_str()) {
            match observation.payload {
                Detection::Camera(_) => diagnostics.associated_camera_detections += 1,
                Detection::Lidar(_) => diagnostics.associated_lidar_detections += 1,
            }
        } else if observation.context.ego_pose.is_some() {
            match observation.payload {
                Detection::Camera(_) => {
                    diagnostics.unmatched_camera_detections += 1;
                    diagnostics.waiting_for_range += 1;
                }
                Detection::Lidar(_) => diagnostics.unmatched_lidar_detections += 1,
            }
        }
    }
}

fn collect_history(
    batch: &DetectionBatch,
    core_batch: &ObservationBatch<Detection, DetectionContext, DetectionScanContext>,
    result: &TrackingOutput<PlanarTrackFilter, TrackManagerDiagnostics<PlanarTrackFilter>>,
    config: &ObjectTrackerConfig,
    history: &mut TrackerHistory,
) {
    let observations = core_batch
        .observations
        .iter()
        .enumerate()
        .map(|(index, observation)| (observation.id.as_str(), (index, observation)))
        .collect::<BTreeMap<_, _>>();
    let selected = selected_pairs(&result.diagnostics.association);
    history
        .associations
        .extend(result.diagnostics.hypotheses.iter().map(|hypothesis| {
            let (detection_index, observation) = observations[hypothesis.observation_id.as_str()];
            let is_selected = selected.contains(&(
                hypothesis.track_id.as_str(),
                hypothesis.observation_id.as_str(),
            ));
            AssociationRecord {
                batch_id: batch.stable_id.clone(),
                measurement_time_ns: observation.measurement_time_ns,
                arrival_time_ns: batch.arrival_time_ns,
                detection_index,
                detection: detection_record(&observation.payload),
                track_id: hypothesis.track_id.to_string(),
                predicted: Some(snapshot(&hypothesis.predicted)),
                normalized_innovation_squared: hypothesis.observation.normalized_innovation_squared,
                gate_threshold_squared: config.gate_sigma.powi(2),
                gate_result: match hypothesis.observation.outcome.gate_decision() {
                    GateDecision::Inside => GateResult::Inside,
                    GateDecision::Outside => GateResult::Outside,
                    GateDecision::Invalid => GateResult::Invalid,
                },
                selected: is_selected,
                corrected: is_selected
                    .then(|| hypothesis.observation.outcome.posterior().map(snapshot))
                    .flatten(),
            }
        }));
    history
        .lifecycle
        .extend(result.lifecycle.iter().map(|event| {
            let detection = event
                .observation_id
                .as_ref()
                .and_then(|id| observations.get(id.as_str()))
                .map(|(_, observation)| detection_record(&observation.payload));
            LifecycleRecord {
                measurement_time_ns: event.time_ns,
                arrival_time_ns: batch.arrival_time_ns,
                sensor: core_batch.context.sensor,
                track_id: event.track_id.to_string(),
                event: match event.kind {
                    fusion_tracking::LifecycleEventKind::Created => LifecycleEventKind::Created,
                    fusion_tracking::LifecycleEventKind::Confirmed => LifecycleEventKind::Confirmed,
                    fusion_tracking::LifecycleEventKind::Missed => LifecycleEventKind::Missed,
                    fusion_tracking::LifecycleEventKind::Coasted => LifecycleEventKind::Coasted,
                    fusion_tracking::LifecycleEventKind::Deleted => LifecycleEventKind::Deleted,
                },
                detection,
                state: snapshot(&event.state),
            }
        }));
}

fn selected_pairs(plan: &AssociationPlan) -> BTreeSet<(&str, &str)> {
    match plan {
        AssociationPlan::Hard(assignments) => assignments
            .iter()
            .map(|assignment| {
                (
                    assignment.track_id.as_str(),
                    assignment.observation_id.as_str(),
                )
            })
            .collect(),
        AssociationPlan::Marginal(marginals) => marginals
            .iter()
            .flat_map(|marginal| {
                marginal
                    .observation_probabilities
                    .iter()
                    .filter(|(_, probability)| *probability > 0.0)
                    .map(|(observation_id, _)| {
                        (marginal.track_id.as_str(), observation_id.as_str())
                    })
            })
            .collect(),
    }
}

fn selected_observation_ids(plan: &AssociationPlan) -> BTreeSet<&str> {
    selected_pairs(plan)
        .into_iter()
        .map(|(_, observation_id)| observation_id)
        .collect()
}

fn detection_record(detection: &Detection) -> DetectionRecord {
    match detection {
        Detection::Camera(value) => DetectionRecord::Camera {
            bearing_rad: value.bearing_rad,
            bearing_variance_rad2: value.bearing_variance_rad2,
        },
        Detection::Lidar(value) => DetectionRecord::Lidar {
            range_m: value.range_m,
            bearing_rad: value.bearing_rad,
            range_variance_m2: value.range_variance_m2,
            bearing_variance_rad2: value.bearing_variance_rad2,
        },
    }
}

fn to_track(filter: &PlanarTrackFilter, track_id: &str) -> ObjectTrack {
    ObjectTrack {
        track_id: track_id.to_owned(),
        position_world_m: Some(Vec2 {
            x: filter.state.position_world_m.x,
            y: filter.state.position_world_m.y,
        }),
        velocity_world_mps: Some(Vec2 {
            x: filter.state.velocity_world_mps.x,
            y: filter.state.velocity_world_mps.y,
        }),
        state_covariance: (0..4)
            .flat_map(|row| (0..4).map(move |column| filter.state_covariance[(row, column)]))
            .collect(),
    }
}

#[cfg(test)]
mod tests {
    use fusion_schema::messages::{
        CameraDetection, CameraFrame, LidarDetection, LidarScan, MeasurementTime,
    };
    use nalgebra::{SMatrix, Vector2};

    use super::*;
    use input::EgoPose;
    use planar_model::{UpdateResult, initialize, update_lidar};

    fn detection(range_m: f64, bearing_rad: f64, time_ns: i64) -> LidarDetection {
        LidarDetection {
            measurement_time_ns: time_ns,
            range_m,
            bearing_rad,
            range_variance_m2: 0.01,
            bearing_variance_rad2: 0.0001,
        }
    }

    fn ego() -> EgoPose {
        EgoPose {
            time_ns: 0,
            position_world_m: Vector2::zeros(),
            yaw_world_from_body_rad: 0.0,
            pose_covariance_world: SMatrix::zeros(),
        }
    }

    fn time(time_ns: i64) -> MeasurementTime {
        MeasurementTime {
            measurement_time_ns: time_ns,
            arrival_time_ns: time_ns,
        }
    }

    fn history(end_time_ns: i64) -> EgoHistory {
        EgoHistory::from_samples(vec![
            ego(),
            EgoPose {
                time_ns: end_time_ns,
                ..ego()
            },
        ])
    }

    #[test]
    fn ego_heading_error_moves_a_distant_object_sideways() {
        let correct = initialize(&detection(20.0, 0.0, 0), ego(), 0);
        let wrong = initialize(
            &detection(20.0, 0.0, 0),
            EgoPose {
                yaw_world_from_body_rad: 1_f64.to_radians(),
                ..ego()
            },
            0,
        );
        assert!(
            (wrong.state.position_world_m.y - correct.state.position_world_m.y - 0.349).abs()
                < 0.002
        );
    }

    #[test]
    fn lidar_update_is_atomic_when_rejected() {
        let ego = ego();
        let mut filter = initialize(&detection(20.0, 0.0, 0), ego, 0);
        let original_state = filter.state;
        let original_state_covariance = filter.state_covariance;
        let outlier = detection(20.0, std::f64::consts::FRAC_PI_2, 0);
        assert_eq!(
            update_lidar(&mut filter, &outlier, ego, 3.0),
            UpdateResult::Rejected
        );
        assert_eq!(filter.state, original_state);
        assert_eq!(filter.state_covariance, original_state_covariance);
    }

    #[test]
    fn association_follows_position_instead_of_detection_order() -> Result<()> {
        let measurements = vec![
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(0)),
                detections: vec![detection(5.0, 0.0, 0), detection(10.0, 0.0, 0)],
            }),
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(1_000_000_000)),
                detections: vec![
                    detection(10.1, 0.0, 1_000_000_000),
                    detection(5.1, 0.0, 1_000_000_000),
                ],
            }),
        ];
        let result = run(
            &ObjectTrackerConfig::default(),
            &CameraConfig::default(),
            &LidarConfig::default(),
            &measurements,
            &history(1_000_000_000),
        )?;
        let tracks = &result.frames.last().unwrap().tracks;
        let first = tracks
            .iter()
            .find(|track| track.track_id == "track-001")
            .unwrap();
        let second = tracks
            .iter()
            .find(|track| track.track_id == "track-002")
            .unwrap();
        assert!(first.position_world_m.as_ref().unwrap().x < 7.0);
        assert!(second.position_world_m.as_ref().unwrap().x > 8.0);
        assert_eq!(result.diagnostics.created_tracks, 2);
        assert_eq!(result.diagnostics.confirmed_tracks, 2);
        Ok(())
    }

    #[test]
    fn history_records_prediction_gate_and_correction() -> Result<()> {
        let measurements = vec![
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(0)),
                detections: vec![detection(5.0, 0.0, 0)],
            }),
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(1_000_000_000)),
                detections: vec![detection(5.5, 0.0, 1_000_000_000)],
            }),
        ];
        let result = run(
            &ObjectTrackerConfig {
                confirmation_hits: 1,
                ..ObjectTrackerConfig::default()
            },
            &CameraConfig::default(),
            &LidarConfig::default(),
            &measurements,
            &history(1_000_000_000),
        )?;
        let association = result.history.associations.last().unwrap();
        assert_eq!(association.gate_result, GateResult::Inside);
        assert!(association.selected);
        assert!(
            association
                .normalized_innovation_squared
                .unwrap()
                .is_finite()
        );
        assert!(association.predicted.is_some());
        assert!(association.corrected.is_some());
        Ok(())
    }

    #[test]
    fn history_records_a_gated_candidate_and_missed_update() -> Result<()> {
        let measurements = vec![
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(0)),
                detections: vec![detection(5.0, 0.0, 0)],
            }),
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(1_000_000_000)),
                detections: vec![detection(5.0, std::f64::consts::FRAC_PI_2, 1_000_000_000)],
            }),
        ];
        let result = run(
            &ObjectTrackerConfig {
                confirmation_hits: 1,
                max_time_without_update_s: 2.0,
                gate_sigma: 0.1,
                ..ObjectTrackerConfig::default()
            },
            &CameraConfig::default(),
            &LidarConfig::default(),
            &measurements,
            &history(1_000_000_000),
        )?;
        let candidate = result.history.associations.first().unwrap();
        assert_eq!(candidate.gate_result, GateResult::Outside);
        assert!(!candidate.selected);
        assert!(candidate.corrected.is_none());
        assert!(result.history.lifecycle.iter().any(|record| {
            record.track_id == "track-001" && record.event == LifecycleEventKind::Missed
        }));
        Ok(())
    }

    #[test]
    fn camera_update_refreshes_track_lifetime() -> Result<()> {
        let measurements = vec![
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(0)),
                detections: vec![detection(5.0, 0.0, 0)],
            }),
            PerceptionMeasurement::Camera(CameraFrame {
                time: Some(time(1_000_000_000)),
                detections: vec![CameraDetection {
                    bearing_rad: 0.0,
                    bearing_variance_rad2: 0.0001,
                }],
            }),
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(2_000_000_000)),
                detections: Vec::new(),
            }),
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(2_500_000_000)),
                detections: Vec::new(),
            }),
        ];
        let result = run(
            &ObjectTrackerConfig {
                confirmation_hits: 1,
                max_time_without_update_s: 1.5,
                ..ObjectTrackerConfig::default()
            },
            &CameraConfig::default(),
            &LidarConfig::default(),
            &measurements,
            &history(2_500_000_000),
        )?;
        assert_eq!(result.frames[2].tracks.len(), 1);
        assert!(result.frames[3].tracks.is_empty());
        assert_eq!(result.diagnostics.deleted_tracks, 1);
        Ok(())
    }

    #[test]
    fn empty_camera_frame_outside_coverage_does_not_delete_track() -> Result<()> {
        let measurements = vec![
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(0)),
                detections: vec![detection(5.0, 0.0, 0)],
            }),
            PerceptionMeasurement::Camera(CameraFrame {
                time: Some(time(2_000_000_000)),
                detections: Vec::new(),
            }),
        ];
        let result = run(
            &ObjectTrackerConfig {
                confirmation_hits: 1,
                max_time_without_update_s: 1.0,
                ..ObjectTrackerConfig::default()
            },
            &CameraConfig {
                max_range_m: 1.0,
                ..CameraConfig::default()
            },
            &LidarConfig::default(),
            &measurements,
            &history(2_000_000_000),
        )?;

        assert_eq!(result.diagnostics.coasted_updates, 1);
        assert_eq!(result.diagnostics.missed_updates, 0);
        assert_eq!(result.diagnostics.deleted_tracks, 0);
        assert_eq!(result.frames.last().unwrap().tracks.len(), 1);
        Ok(())
    }

    #[test]
    fn track_is_deleted_after_configured_time_without_an_update() -> Result<()> {
        let measurements = vec![
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(0)),
                detections: vec![detection(5.0, 0.0, 0)],
            }),
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(1_000_000_000)),
                detections: Vec::new(),
            }),
            PerceptionMeasurement::Lidar(LidarScan {
                time: Some(time(2_000_000_000)),
                detections: Vec::new(),
            }),
        ];
        let result = run(
            &ObjectTrackerConfig {
                confirmation_hits: 1,
                max_time_without_update_s: 2.0,
                ..ObjectTrackerConfig::default()
            },
            &CameraConfig::default(),
            &LidarConfig::default(),
            &measurements,
            &history(2_000_000_000),
        )?;
        assert_eq!(result.diagnostics.created_tracks, 1);
        assert_eq!(result.diagnostics.confirmed_tracks, 1);
        assert_eq!(result.diagnostics.deleted_tracks, 1);
        assert!(result.frames.last().unwrap().tracks.is_empty());
        Ok(())
    }

    #[test]
    fn missing_perception_time_is_an_input_error() {
        let measurements = vec![PerceptionMeasurement::Camera(CameraFrame {
            time: None,
            detections: Vec::new(),
        })];
        let error = run(
            &ObjectTrackerConfig::default(),
            &CameraConfig::default(),
            &LidarConfig::default(),
            &measurements,
            &history(0),
        )
        .unwrap_err();
        assert_eq!(error.to_string(), "perception measurement has no time");
    }
}
