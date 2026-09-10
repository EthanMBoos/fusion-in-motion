use std::{convert::Infallible, error::Error};

use fusion_tracking::{
    AssociationEngine, AssociationPlan, DetectionOpportunity, HitCountLifecycle, HypothesisModel,
    HypothesisOutcome, InitiatedTrack, InitiationCandidate, Initiator, LifecycleEvent,
    ObservationBatch, ObservationHypothesis, ObservationId, PairHypothesis, PosteriorReducer,
    TimedObservation, TrackDetectionOpportunity, TrackId, TrackIdentity, TrackManager,
    TrackMarginal, TrackReport, TrackStatus, Tracker, TrackingOutput, WeightedPosterior,
};

#[derive(Debug, Clone)]
struct State {
    value: f64,
    time_ns: i64,
}

struct Model;

impl HypothesisModel<State, f64, (), &'static str> for Model {
    type Error = Infallible;

    fn predict(&self, state: &State, time_ns: i64) -> Result<State, Self::Error> {
        Ok(State {
            value: state.value,
            time_ns,
        })
    }

    fn hypothesize(
        &self,
        _predicted: &State,
        observation: &TimedObservation<f64, ()>,
    ) -> Result<ObservationHypothesis<State>, Self::Error> {
        Ok(ObservationHypothesis {
            normalized_innovation_squared: Some(1.0),
            log_likelihood: Some(-0.5),
            outcome: HypothesisOutcome::Inside {
                posterior: State {
                    value: observation.payload,
                    time_ns: observation.measurement_time_ns,
                },
            },
        })
    }

    fn detection_opportunity(
        &self,
        _predicted: &State,
        _measurement_time_ns: i64,
        context: &&'static str,
    ) -> Result<DetectionOpportunity, Self::Error> {
        assert_eq!(*context, "sensor-a");
        Ok(DetectionOpportunity::Observable {
            detection_probability: 0.9,
        })
    }
}

#[derive(Clone)]
struct FirstObservationInitiator;

impl Initiator<State, f64, (), &'static str> for FirstObservationInitiator {
    type Error = Infallible;

    fn initiate(
        &mut self,
        context: &&'static str,
        candidates: &[InitiationCandidate<'_, f64, ()>],
    ) -> Result<Vec<InitiatedTrack<State>>, Self::Error> {
        assert_eq!(*context, "sensor-a");
        Ok(candidates
            .first()
            .filter(|candidate| candidate.unassigned_probability >= 0.5)
            .map(|candidate| InitiatedTrack {
                observation_id: candidate.observation.id.clone(),
                state: State {
                    value: candidate.observation.payload,
                    time_ns: candidate.observation.measurement_time_ns,
                },
            })
            .into_iter()
            .collect())
    }
}

struct MarginalAssociator;

impl AssociationEngine<State, f64, (), &'static str> for MarginalAssociator {
    type Error = Infallible;

    fn associate(
        &self,
        batch: &ObservationBatch<f64, (), &'static str>,
        track_ids: &[TrackId],
        observation_ids: &[ObservationId],
        _hypotheses: &[PairHypothesis<State>],
        detection_opportunities: &[TrackDetectionOpportunity],
    ) -> Result<AssociationPlan, Self::Error> {
        assert_eq!(batch.context, "sensor-a");
        assert_eq!(detection_opportunities.len(), track_ids.len());
        Ok(AssociationPlan::Marginal(
            track_ids
                .iter()
                .map(|track_id| TrackMarginal {
                    track_id: track_id.clone(),
                    missed_probability: 0.25,
                    observation_probabilities: vec![(observation_ids[0].clone(), 0.75)],
                })
                .collect(),
        ))
    }
}

struct WeightedReducer;

impl PosteriorReducer<State> for WeightedReducer {
    type Error = Infallible;

    fn reduce(
        &self,
        reduction_time_ns: i64,
        predicted: &State,
        candidates: &[WeightedPosterior<'_, State>],
        missed_probability: f64,
    ) -> Result<State, Self::Error> {
        Ok(State {
            value: missed_probability * predicted.value
                + candidates
                    .iter()
                    .map(|candidate| candidate.probability * candidate.state.value)
                    .sum::<f64>(),
            time_ns: reduction_time_ns,
        })
    }
}

fn batch(
    id: &str,
    measurement_time_ns: i64,
    value: f64,
) -> ObservationBatch<f64, (), &'static str> {
    ObservationBatch {
        id: id.into(),
        measurement_time_ns,
        arrival_time_ns: measurement_time_ns + 100,
        context: "sensor-a",
        observations: vec![TimedObservation {
            id: format!("{id}:0").into(),
            measurement_time_ns,
            payload: value,
            context: (),
        }],
    }
}

#[test]
fn external_crate_can_supply_every_application_component() -> Result<(), Box<dyn Error>> {
    let mut manager = TrackManager::new(
        Model,
        MarginalAssociator,
        WeightedReducer,
        FirstObservationInitiator,
        HitCountLifecycle {
            confirmation_hits: 1,
            max_time_without_update_ns: 1_000,
            hit_probability_threshold: 0.5,
        },
    );

    manager.process_scan(&batch("first", 5, 0.0))?;
    let result = manager.process_scan(&batch("second", 10, 10.0))?;

    assert_eq!(result.tracks.len(), 1);
    assert_eq!(result.tracks[0].state.value, 7.5);
    assert_eq!(result.tracks[0].state.time_ns, 10);
    assert_eq!(result.arrival_time_ns, 110);
    Ok(())
}

struct PopulationTracker {
    scan_count: usize,
}

impl Tracker<ObservationBatch<f64, (), &'static str>> for PopulationTracker {
    type State = State;
    type Diagnostics = usize;
    type Error = Infallible;

    fn process_scan(
        &mut self,
        scan: &ObservationBatch<f64, (), &'static str>,
    ) -> Result<TrackingOutput<State, usize>, Self::Error> {
        self.scan_count += 1;
        Ok(TrackingOutput {
            measurement_time_ns: scan.output_time_ns(),
            arrival_time_ns: scan.arrival_time_ns,
            tracks: vec![TrackReport {
                identity: TrackIdentity::Unlabeled,
                state: State {
                    value: scan.observations[0].payload,
                    time_ns: scan.output_time_ns(),
                },
                status: TrackStatus::Confirmed,
            }],
            lifecycle: Vec::<LifecycleEvent<State>>::new(),
            diagnostics: self.scan_count,
        })
    }
}

#[test]
fn downstream_tracker_can_own_non_track_manager_state() -> Result<(), Box<dyn Error>> {
    let mut tracker = PopulationTracker { scan_count: 0 };
    let output = tracker.process_scan(&batch("first", 5, 7.0))?;

    assert_eq!(output.tracks[0].identity, TrackIdentity::Unlabeled);
    assert_eq!(output.tracks[0].state.value, 7.0);
    assert_eq!(output.diagnostics, 1);
    Ok(())
}
