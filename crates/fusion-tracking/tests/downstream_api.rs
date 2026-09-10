use std::{convert::Infallible, error::Error};

use fusion_tracking::{
    AssociationEngine, AssociationPlan, HitCountLifecycle, HypothesisModel, HypothesisOutcome,
    InitiatedTrack, Initiator, ObservationBatch, ObservationHypothesis, ObservationId,
    PairHypothesis, PosteriorReducer, TimedObservation, TrackId, TrackManager, TrackMarginal,
    WeightedPosterior,
};

#[derive(Debug, Clone)]
struct State {
    value: f64,
    time_ns: i64,
}

struct Model;

impl HypothesisModel<State, f64, ()> for Model {
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
}

#[derive(Clone)]
struct FirstObservationInitiator;

impl Initiator<State, f64, ()> for FirstObservationInitiator {
    type Error = Infallible;

    fn initiate(
        &mut self,
        observations: &[&TimedObservation<f64, ()>],
    ) -> Result<Vec<InitiatedTrack<State>>, Self::Error> {
        Ok(observations
            .first()
            .map(|observation| InitiatedTrack {
                observation_id: observation.id.clone(),
                state: State {
                    value: observation.payload,
                    time_ns: observation.measurement_time_ns,
                },
            })
            .into_iter()
            .collect())
    }
}

struct MarginalAssociator;

impl AssociationEngine<State> for MarginalAssociator {
    type Error = Infallible;

    fn associate(
        &self,
        track_ids: &[TrackId],
        observation_ids: &[ObservationId],
        _hypotheses: &[PairHypothesis<State>],
    ) -> Result<AssociationPlan, Self::Error> {
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

fn batch(id: &str, measurement_time_ns: i64, value: f64) -> ObservationBatch<f64, ()> {
    ObservationBatch {
        id: id.into(),
        measurement_time_ns,
        arrival_time_ns: measurement_time_ns + 100,
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
        },
    );

    manager.process_batch(&batch("first", 5, 0.0))?;
    let result = manager.process_batch(&batch("second", 10, 10.0))?;

    assert_eq!(result.tracks.len(), 1);
    assert_eq!(result.tracks[0].state.value, 7.5);
    assert_eq!(result.tracks[0].state.time_ns, 10);
    assert_eq!(result.arrival_time_ns, 110);
    Ok(())
}
