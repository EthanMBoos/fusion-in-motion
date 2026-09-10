use std::collections::{BTreeMap, BTreeSet};

use crate::{
    AssociationEngine, AssociationPlan, BatchDiagnostics, GateDecision, HypothesisModel, Initiator,
    LifecycleEvent, LifecycleEventKind, LifecyclePolicy, ManagedTrack, ObservationBatch,
    ObservationId, PairHypothesis, PosteriorReducer, TimedObservation, TrackId, TrackMarginal,
    TrackStatus, WeightedPosterior,
};

#[derive(Debug, Clone)]
pub struct BatchResult<S> {
    pub measurement_time_ns: i64,
    pub arrival_time_ns: i64,
    pub tracks: Vec<ManagedTrack<S>>,
    pub hypotheses: Vec<PairHypothesis<S>>,
    pub association: AssociationPlan,
    pub lifecycle: Vec<LifecycleEvent<S>>,
    pub diagnostics: BatchDiagnostics,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TrackManagerError(String);

impl TrackManagerError {
    fn new(message: impl Into<String>) -> Self {
        Self(message.into())
    }
}

impl std::fmt::Display for TrackManagerError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(formatter)
    }
}

impl std::error::Error for TrackManagerError {}

pub struct TrackManager<S, D, C, M, A, R, I, L> {
    model: M,
    association: A,
    reducer: R,
    initiator: I,
    lifecycle: L,
    tracks: BTreeMap<TrackId, ManagedTrack<S>>,
    next_track_number: u64,
    marker: std::marker::PhantomData<(D, C)>,
}

impl<S, D, C, M, A, R, I, L> TrackManager<S, D, C, M, A, R, I, L>
where
    S: Clone,
    M: HypothesisModel<S, D, C>,
    A: AssociationEngine<S>,
    R: PosteriorReducer<S>,
    I: Initiator<S, D, C> + Clone,
    L: LifecyclePolicy<S> + Clone,
{
    pub fn new(model: M, association: A, reducer: R, initiator: I, lifecycle: L) -> Self {
        Self {
            model,
            association,
            reducer,
            initiator,
            lifecycle,
            tracks: BTreeMap::new(),
            next_track_number: 1,
            marker: std::marker::PhantomData,
        }
    }

    pub fn process_batch(
        &mut self,
        batch: &ObservationBatch<D, C>,
    ) -> Result<BatchResult<S>, TrackManagerError> {
        validate_batch(batch)?;
        // Commit these batch-local copies only after every fallible stage succeeds.
        let mut staged_tracks = self.tracks.clone();
        let mut staged_initiator = self.initiator.clone();
        let mut staged_lifecycle = self.lifecycle.clone();
        let mut staged_next_track_number = self.next_track_number;
        let output_time_ns = batch.output_time_ns();
        let track_ids = staged_tracks.keys().cloned().collect::<Vec<_>>();
        let observation_ids = batch
            .observations
            .iter()
            .map(|observation| observation.id.clone())
            .collect::<Vec<_>>();
        let mut hypotheses = Vec::with_capacity(track_ids.len() * observation_ids.len());

        for track_id in &track_ids {
            let track = &staged_tracks[track_id];
            for observation in &batch.observations {
                let predicted = self
                    .model
                    .predict(&track.state, observation.measurement_time_ns)
                    .map_err(|error| {
                        TrackManagerError::new(format!(
                            "prediction failed for track {track_id} at {} ns: {error}",
                            observation.measurement_time_ns
                        ))
                    })?;
                let observation_hypothesis = self
                    .model
                    .hypothesize(&predicted, observation)
                    .map_err(|error| {
                        TrackManagerError::new(format!(
                            "hypothesis failed for track {track_id}, observation {}: {error}",
                            observation.id
                        ))
                    })?;
                hypotheses.push(PairHypothesis {
                    track_id: track_id.clone(),
                    observation_id: observation.id.clone(),
                    predicted,
                    observation: observation_hypothesis,
                });
            }
        }

        let association = self
            .association
            .associate(&track_ids, &observation_ids, &hypotheses)
            .map_err(|error| TrackManagerError::new(format!("association failed: {error}")))?;
        validate_plan(&association, &track_ids, &observation_ids, &hypotheses)?;

        let mut diagnostics = BatchDiagnostics {
            candidate_pairs: hypotheses.len(),
            gated_out_pairs: hypotheses
                .iter()
                .filter(|hypothesis| {
                    hypothesis.observation.outcome.gate_decision() == GateDecision::Outside
                })
                .count(),
            invalid_candidate_pairs: hypotheses
                .iter()
                .filter(|hypothesis| {
                    hypothesis.observation.outcome.gate_decision() == GateDecision::Invalid
                })
                .count(),
            selected_associations: selected_pair_count(&association),
            ..BatchDiagnostics::default()
        };
        let mut lifecycle = Vec::new();
        let marginals = plan_marginals(&association, &track_ids);
        let lookup = hypotheses
            .iter()
            .map(|hypothesis| {
                (
                    (
                        hypothesis.track_id.clone(),
                        hypothesis.observation_id.clone(),
                    ),
                    hypothesis,
                )
            })
            .collect::<BTreeMap<_, _>>();

        for marginal in &marginals {
            if marginal.observation_probabilities.is_empty() {
                let track = &staged_tracks[&marginal.track_id];
                let predicted = self
                    .model
                    .predict(&track.state, output_time_ns)
                    .map_err(|error| {
                        TrackManagerError::new(format!(
                            "missed-track prediction failed for track {} at {output_time_ns} ns: {error}",
                            marginal.track_id
                        ))
                    })?;
                let track = staged_tracks
                    .get_mut(&marginal.track_id)
                    .expect("validated association references a live track");
                staged_lifecycle.after_miss(track, output_time_ns);
                diagnostics.missed_updates += 1;
                lifecycle.push(event(
                    track,
                    LifecycleEventKind::Missed,
                    output_time_ns,
                    None,
                    predicted,
                ));
                continue;
            }

            let reduction_time_ns = marginal
                .observation_probabilities
                .iter()
                .map(|(observation_id, _)| observation(batch, observation_id).measurement_time_ns)
                .max()
                .expect("nonempty marginal has a measurement time");
            let track = &staged_tracks[&marginal.track_id];
            let predicted_at_reduction_time = self
                .model
                .predict(&track.state, reduction_time_ns)
                .map_err(|error| {
                TrackManagerError::new(format!(
                    "reduction prediction failed for track {} at {reduction_time_ns} ns: {error}",
                    marginal.track_id
                ))
            })?;
            let candidate_states = marginal
                .observation_probabilities
                .iter()
                .map(|(observation_id, probability)| {
                    let hypothesis = lookup[&(marginal.track_id.clone(), observation_id.clone())];
                    let posterior = hypothesis
                        .observation
                        .outcome
                        .posterior()
                        .expect("validated selected hypothesis has a posterior");
                    let state = self
                        .model
                        .predict(posterior, reduction_time_ns)
                        .map_err(|error| {
                            TrackManagerError::new(format!(
                                "posterior prediction failed for track {}, observation {observation_id} at {reduction_time_ns} ns: {error}",
                                marginal.track_id
                            ))
                        })?;
                    Ok((observation_id, state, *probability))
                })
                .collect::<Result<Vec<_>, TrackManagerError>>()?;
            let candidates = candidate_states
                .iter()
                .map(|(observation_id, state, probability)| WeightedPosterior {
                    observation_id,
                    state,
                    probability: *probability,
                })
                .collect::<Vec<_>>();
            let reduced_state = self
                .reducer
                .reduce(
                    reduction_time_ns,
                    &predicted_at_reduction_time,
                    &candidates,
                    marginal.missed_probability,
                )
                .map_err(|error| {
                    TrackManagerError::new(format!(
                        "posterior reduction failed for track {} at {reduction_time_ns} ns: {error}",
                        marginal.track_id
                    ))
                })?;
            let track = staged_tracks
                .get_mut(&marginal.track_id)
                .expect("validated association references a live track");
            track.state = reduced_state;
            let was_confirmed = track.status == TrackStatus::Confirmed;
            staged_lifecycle.after_hit(track, reduction_time_ns);
            if !was_confirmed && track.status == TrackStatus::Confirmed {
                diagnostics.confirmed_tracks += 1;
                lifecycle.push(event(
                    track,
                    LifecycleEventKind::Confirmed,
                    reduction_time_ns,
                    None,
                    track.state.clone(),
                ));
            }
        }

        let used_observations = association_observations(&association);
        let unused = batch
            .observations
            .iter()
            .filter(|observation| !used_observations.contains(&observation.id))
            .collect::<Vec<_>>();
        let initiated = staged_initiator
            .initiate(&unused)
            .map_err(|error| TrackManagerError::new(format!("initiation failed: {error}")))?;
        let unused_ids = unused
            .iter()
            .map(|observation| &observation.id)
            .collect::<BTreeSet<_>>();
        let mut initiated_ids = BTreeSet::new();
        for initiated_track in initiated {
            if !unused_ids.contains(&initiated_track.observation_id)
                || !initiated_ids.insert(initiated_track.observation_id.clone())
            {
                return Err(TrackManagerError::new(
                    "initiator returned an assigned, unknown, or duplicate observation",
                ));
            }
            let observation = observation(batch, &initiated_track.observation_id);
            let track_id = TrackId::new(format!("track-{staged_next_track_number:03}"));
            staged_next_track_number = staged_next_track_number
                .checked_add(1)
                .ok_or_else(|| TrackManagerError::new("track identifier counter overflowed"))?;
            let mut track = ManagedTrack {
                id: track_id.clone(),
                state: initiated_track.state,
                status: TrackStatus::Tentative,
                hit_count: 0,
                miss_count: 0,
                last_update_time_ns: observation.measurement_time_ns,
            };
            lifecycle.push(event(
                &track,
                LifecycleEventKind::Created,
                observation.measurement_time_ns,
                Some(initiated_track.observation_id.clone()),
                track.state.clone(),
            ));
            diagnostics.created_tracks += 1;
            staged_lifecycle.after_hit(&mut track, observation.measurement_time_ns);
            if track.status == TrackStatus::Confirmed {
                lifecycle.push(event(
                    &track,
                    LifecycleEventKind::Confirmed,
                    observation.measurement_time_ns,
                    None,
                    track.state.clone(),
                ));
                diagnostics.confirmed_tracks += 1;
            }
            staged_tracks.insert(track_id, track);
        }

        let stale = staged_tracks
            .iter()
            .filter(|(_, track)| staged_lifecycle.should_delete(track, output_time_ns))
            .map(|(track_id, _)| track_id.clone())
            .collect::<Vec<_>>();
        for track_id in stale {
            let track = &staged_tracks[&track_id];
            let predicted = self
                .model
                .predict(&track.state, output_time_ns)
                .map_err(|error| {
                    TrackManagerError::new(format!(
                        "deletion prediction failed for track {track_id} at {output_time_ns} ns: {error}"
                    ))
                })?;
            lifecycle.push(event(
                track,
                LifecycleEventKind::Deleted,
                output_time_ns,
                None,
                predicted,
            ));
            staged_tracks.remove(&track_id).expect("stale track exists");
            diagnostics.deleted_tracks += 1;
        }

        let output_tracks = staged_tracks
            .values()
            .filter(|track| track.status == TrackStatus::Confirmed)
            .map(|track| {
                let mut output = track.clone();
                output.state = self.model.predict(&track.state, output_time_ns).map_err(
                    |error| {
                        TrackManagerError::new(format!(
                            "output prediction failed for track {} at {output_time_ns} ns: {error}",
                            track.id
                        ))
                    },
                )?;
                Ok(output)
            })
            .collect::<Result<Vec<_>, TrackManagerError>>()?;

        self.tracks = staged_tracks;
        self.initiator = staged_initiator;
        self.lifecycle = staged_lifecycle;
        self.next_track_number = staged_next_track_number;

        Ok(BatchResult {
            measurement_time_ns: output_time_ns,
            arrival_time_ns: batch.arrival_time_ns,
            tracks: output_tracks,
            hypotheses,
            association,
            lifecycle,
            diagnostics,
        })
    }
}

fn validate_batch<D, C>(batch: &ObservationBatch<D, C>) -> Result<(), TrackManagerError> {
    let mut ids = BTreeSet::new();
    if batch
        .observations
        .iter()
        .any(|observation| !ids.insert(&observation.id))
    {
        return Err(TrackManagerError::new(
            "observation identifiers must be unique within a batch",
        ));
    }
    Ok(())
}

fn validate_plan<S>(
    plan: &AssociationPlan,
    track_ids: &[TrackId],
    observation_ids: &[ObservationId],
    hypotheses: &[PairHypothesis<S>],
) -> Result<(), TrackManagerError> {
    let known_tracks = track_ids.iter().collect::<BTreeSet<_>>();
    let known_observations = observation_ids.iter().collect::<BTreeSet<_>>();
    let candidates = hypotheses
        .iter()
        .map(|hypothesis| {
            (
                (&hypothesis.track_id, &hypothesis.observation_id),
                hypothesis,
            )
        })
        .collect::<BTreeMap<_, _>>();
    if let AssociationPlan::Hard(assignments) = plan {
        let unique_tracks = assignments
            .iter()
            .map(|assignment| &assignment.track_id)
            .collect::<BTreeSet<_>>();
        let unique_observations = assignments
            .iter()
            .map(|assignment| &assignment.observation_id)
            .collect::<BTreeSet<_>>();
        if unique_tracks.len() != assignments.len()
            || unique_observations.len() != assignments.len()
        {
            return Err(TrackManagerError::new(
                "hard association must be one-to-one",
            ));
        }
    }
    let marginals = plan_marginals(plan, track_ids);
    if marginals.len() != track_ids.len() {
        return Err(TrackManagerError::new(
            "association plan must describe every live track",
        ));
    }
    let mut seen_tracks = BTreeSet::new();
    let mut hard_observations = BTreeSet::new();
    let mut marginal_observation_totals = BTreeMap::<&ObservationId, f64>::new();
    for marginal in &marginals {
        if !known_tracks.contains(&marginal.track_id) || !seen_tracks.insert(&marginal.track_id) {
            return Err(TrackManagerError::new(
                "association plan contains an unknown or duplicate track",
            ));
        }
        if !marginal.missed_probability.is_finite()
            || !(0.0..=1.0).contains(&marginal.missed_probability)
        {
            return Err(TrackManagerError::new(
                "invalid missed-detection probability",
            ));
        }
        let mut total = marginal.missed_probability;
        let mut seen_observations = BTreeSet::new();
        for (observation_id, probability) in &marginal.observation_probabilities {
            if !known_observations.contains(observation_id)
                || !seen_observations.insert(observation_id)
                || !probability.is_finite()
                || *probability <= 0.0
                || *probability > 1.0
            {
                return Err(TrackManagerError::new(
                    "association plan contains an invalid observation probability",
                ));
            }
            let Some(hypothesis) = candidates.get(&(&marginal.track_id, observation_id)) else {
                return Err(TrackManagerError::new(
                    "association plan references a missing hypothesis",
                ));
            };
            if hypothesis.observation.outcome.gate_decision() != GateDecision::Inside {
                return Err(TrackManagerError::new(
                    "association plan selected an invalid or gated-out hypothesis",
                ));
            }
            total += probability;
            *marginal_observation_totals
                .entry(observation_id)
                .or_default() += probability;
            if matches!(plan, AssociationPlan::Hard(_)) && !hard_observations.insert(observation_id)
            {
                return Err(TrackManagerError::new(
                    "hard association assigned one observation more than once",
                ));
            }
        }
        if (total - 1.0).abs() > 1.0e-9 {
            return Err(TrackManagerError::new(
                "association probabilities must sum to one per track",
            ));
        }
    }
    if seen_tracks.len() != track_ids.len() {
        return Err(TrackManagerError::new(
            "association plan omitted a live track",
        ));
    }
    if marginal_observation_totals
        .values()
        .any(|probability| *probability > 1.0 + 1.0e-9)
    {
        return Err(TrackManagerError::new(
            "association probabilities assign an observation more than once",
        ));
    }
    Ok(())
}

fn plan_marginals(plan: &AssociationPlan, track_ids: &[TrackId]) -> Vec<TrackMarginal> {
    match plan {
        AssociationPlan::Hard(assignments) => {
            let assigned = assignments
                .iter()
                .map(|assignment| (&assignment.track_id, &assignment.observation_id))
                .collect::<BTreeMap<_, _>>();
            track_ids
                .iter()
                .map(|track_id| match assigned.get(track_id) {
                    Some(observation_id) => TrackMarginal {
                        track_id: track_id.clone(),
                        missed_probability: 0.0,
                        observation_probabilities: vec![((*observation_id).clone(), 1.0)],
                    },
                    None => TrackMarginal {
                        track_id: track_id.clone(),
                        missed_probability: 1.0,
                        observation_probabilities: Vec::new(),
                    },
                })
                .collect()
        }
        AssociationPlan::Marginal(marginals) => marginals.clone(),
    }
}

fn selected_pair_count(plan: &AssociationPlan) -> usize {
    match plan {
        AssociationPlan::Hard(assignments) => assignments.len(),
        AssociationPlan::Marginal(marginals) => marginals
            .iter()
            .map(|marginal| marginal.observation_probabilities.len())
            .sum(),
    }
}

fn association_observations(plan: &AssociationPlan) -> BTreeSet<ObservationId> {
    match plan {
        AssociationPlan::Hard(assignments) => assignments
            .iter()
            .map(|assignment| assignment.observation_id.clone())
            .collect(),
        AssociationPlan::Marginal(marginals) => marginals
            .iter()
            .flat_map(|marginal| {
                marginal
                    .observation_probabilities
                    .iter()
                    .filter(|(_, probability)| *probability > 0.0)
                    .map(|(observation_id, _)| observation_id.clone())
            })
            .collect(),
    }
}

fn observation<'a, D, C>(
    batch: &'a ObservationBatch<D, C>,
    id: &ObservationId,
) -> &'a TimedObservation<D, C> {
    batch
        .observations
        .iter()
        .find(|observation| observation.id == *id)
        .expect("validated observation exists")
}

fn event<S>(
    track: &ManagedTrack<S>,
    kind: LifecycleEventKind,
    time_ns: i64,
    observation_id: Option<ObservationId>,
    state: S,
) -> LifecycleEvent<S> {
    LifecycleEvent {
        time_ns,
        track_id: track.id.clone(),
        kind,
        observation_id,
        status: track.status,
        state,
    }
}

#[cfg(test)]
mod tests {
    use std::{convert::Infallible, error::Error};

    use super::*;
    use crate::{
        GlobalNearestNeighbor, HitCountLifecycle, HypothesisOutcome, InitiatedTrack,
        ObservationHypothesis, SinglePosteriorReducer,
    };

    #[derive(Clone, Debug, PartialEq)]
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
            predicted: &State,
            observation: &TimedObservation<f64, ()>,
        ) -> Result<ObservationHypothesis<State>, Self::Error> {
            let cost = (observation.payload - predicted.value).powi(2);
            Ok(ObservationHypothesis {
                normalized_innovation_squared: Some(cost),
                log_likelihood: None,
                outcome: if cost <= 25.0 {
                    HypothesisOutcome::Inside {
                        posterior: State {
                            value: observation.payload,
                            time_ns: observation.measurement_time_ns,
                        },
                    }
                } else {
                    HypothesisOutcome::Outside
                },
            })
        }
    }

    #[derive(Clone, Default)]
    struct TestInitiator;

    impl Initiator<State, f64, ()> for TestInitiator {
        type Error = Infallible;

        fn initiate(
            &mut self,
            observations: &[&TimedObservation<f64, ()>],
        ) -> Result<Vec<InitiatedTrack<State>>, Self::Error> {
            Ok(observations
                .iter()
                .map(|observation| InitiatedTrack {
                    observation_id: observation.id.clone(),
                    state: State {
                        value: observation.payload,
                        time_ns: observation.measurement_time_ns,
                    },
                })
                .collect())
        }
    }

    fn batch(
        id: &str,
        measurement: i64,
        arrival: i64,
        values: &[f64],
    ) -> ObservationBatch<f64, ()> {
        ObservationBatch {
            id: id.into(),
            measurement_time_ns: measurement,
            arrival_time_ns: arrival,
            observations: values
                .iter()
                .enumerate()
                .map(|(index, value)| TimedObservation {
                    id: format!("{id}:{index}").into(),
                    measurement_time_ns: measurement,
                    payload: *value,
                    context: (),
                })
                .collect(),
        }
    }

    fn manager() -> TrackManager<
        State,
        f64,
        (),
        Model,
        GlobalNearestNeighbor,
        SinglePosteriorReducer,
        TestInitiator,
        HitCountLifecycle,
    > {
        TrackManager::new(
            Model,
            GlobalNearestNeighbor {
                missed_assignment_cost: 26.0,
            },
            SinglePosteriorReducer,
            TestInitiator,
            HitCountLifecycle {
                confirmation_hits: 2,
                max_time_without_update_ns: 20,
            },
        )
    }

    #[test]
    fn prediction_uses_measurement_time_not_arrival_time() -> Result<(), Box<dyn Error>> {
        let mut manager = manager();
        manager.process_batch(&batch("first", 10, 100, &[2.0]))?;
        let result = manager.process_batch(&batch("second", 15, 200, &[2.0]))?;
        assert_eq!(result.tracks[0].state.time_ns, 15);
        assert_ne!(result.tracks[0].state.time_ns, result.arrival_time_ns);
        Ok(())
    }

    #[test]
    fn lifecycle_orders_confirmation_miss_and_deletion() -> Result<(), Box<dyn Error>> {
        let mut manager = manager();
        let created = manager.process_batch(&batch("first", 0, 0, &[2.0]))?;
        assert_eq!(created.lifecycle[0].kind, LifecycleEventKind::Created);
        assert!(created.tracks.is_empty());

        let confirmed = manager.process_batch(&batch("second", 5, 5, &[2.0]))?;
        assert_eq!(confirmed.lifecycle[0].kind, LifecycleEventKind::Confirmed);
        assert_eq!(confirmed.tracks[0].status, TrackStatus::Confirmed);

        let missed = manager.process_batch(&batch("third", 10, 10, &[]))?;
        assert_eq!(missed.lifecycle[0].kind, LifecycleEventKind::Missed);
        assert_eq!(missed.diagnostics.deleted_tracks, 0);

        let deleted = manager.process_batch(&batch("fourth", 25, 25, &[]))?;
        assert_eq!(deleted.lifecycle[0].kind, LifecycleEventKind::Missed);
        assert_eq!(deleted.lifecycle[1].kind, LifecycleEventKind::Deleted);
        assert!(deleted.tracks.is_empty());
        Ok(())
    }

    #[test]
    fn output_and_events_are_deterministically_ordered() -> Result<(), Box<dyn Error>> {
        let mut manager = manager();
        manager.process_batch(&batch("first", 0, 0, &[8.0, 2.0]))?;
        let result = manager.process_batch(&batch("second", 1, 1, &[2.0, 8.0]))?;
        assert_eq!(result.tracks[0].id.as_str(), "track-001");
        assert_eq!(result.tracks[1].id.as_str(), "track-002");
        assert_eq!(result.hypotheses[0].track_id.as_str(), "track-001");
        assert_eq!(result.hypotheses[0].observation_id.as_str(), "second:0");
        Ok(())
    }

    #[test]
    fn marginal_plan_cannot_reuse_more_than_one_observation_probability() {
        let track_ids = vec![TrackId::new("track-001"), TrackId::new("track-002")];
        let observation_ids = vec![ObservationId::new("observation-1")];
        let hypotheses = track_ids
            .iter()
            .map(|track_id| PairHypothesis {
                track_id: track_id.clone(),
                observation_id: observation_ids[0].clone(),
                predicted: (),
                observation: ObservationHypothesis {
                    normalized_innovation_squared: Some(1.0),
                    log_likelihood: None,
                    outcome: HypothesisOutcome::Inside { posterior: () },
                },
            })
            .collect::<Vec<_>>();
        let plan = AssociationPlan::Marginal(
            track_ids
                .iter()
                .map(|track_id| TrackMarginal {
                    track_id: track_id.clone(),
                    missed_probability: 0.25,
                    observation_probabilities: vec![(observation_ids[0].clone(), 0.75)],
                })
                .collect(),
        );

        assert!(validate_plan(&plan, &track_ids, &observation_ids, &hypotheses).is_err());
    }

    struct AllObservationsMarginal;

    impl AssociationEngine<State> for AllObservationsMarginal {
        type Error = Infallible;

        fn associate(
            &self,
            track_ids: &[TrackId],
            observation_ids: &[ObservationId],
            _hypotheses: &[PairHypothesis<State>],
        ) -> Result<AssociationPlan, Self::Error> {
            let probability = 1.0 / observation_ids.len() as f64;
            Ok(AssociationPlan::Marginal(
                track_ids
                    .iter()
                    .map(|track_id| TrackMarginal {
                        track_id: track_id.clone(),
                        missed_probability: 0.0,
                        observation_probabilities: observation_ids
                            .iter()
                            .cloned()
                            .map(|observation_id| (observation_id, probability))
                            .collect(),
                    })
                    .collect(),
            ))
        }
    }

    struct MovingModel;

    impl HypothesisModel<State, f64, ()> for MovingModel {
        type Error = Infallible;

        fn predict(&self, state: &State, time_ns: i64) -> Result<State, Self::Error> {
            Ok(State {
                value: state.value + (time_ns - state.time_ns) as f64,
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
                log_likelihood: None,
                outcome: HypothesisOutcome::Inside {
                    posterior: State {
                        value: observation.payload,
                        time_ns: observation.measurement_time_ns,
                    },
                },
            })
        }
    }

    struct MeanReducer;

    impl PosteriorReducer<State> for MeanReducer {
        type Error = Infallible;

        fn reduce(
            &self,
            reduction_time_ns: i64,
            _predicted: &State,
            candidates: &[WeightedPosterior<'_, State>],
            _missed_probability: f64,
        ) -> Result<State, Self::Error> {
            Ok(State {
                value: candidates
                    .iter()
                    .map(|candidate| candidate.state.value * candidate.probability)
                    .sum(),
                time_ns: reduction_time_ns,
            })
        }
    }

    #[test]
    fn marginal_posteriors_are_reduced_at_one_time() -> Result<(), Box<dyn Error>> {
        let mut manager = TrackManager::new(
            MovingModel,
            AllObservationsMarginal,
            MeanReducer,
            TestInitiator,
            HitCountLifecycle {
                confirmation_hits: 1,
                max_time_without_update_ns: 100,
            },
        );
        manager.process_batch(&batch("first", 0, 0, &[0.0]))?;
        let second = ObservationBatch {
            id: "second".into(),
            measurement_time_ns: 10,
            arrival_time_ns: 30,
            observations: vec![
                TimedObservation {
                    id: "second:0".into(),
                    measurement_time_ns: 10,
                    payload: 10.0,
                    context: (),
                },
                TimedObservation {
                    id: "second:1".into(),
                    measurement_time_ns: 20,
                    payload: 20.0,
                    context: (),
                },
            ],
        };
        let result = manager.process_batch(&second)?;
        assert_eq!(result.tracks[0].state.value, 20.0);
        assert_eq!(result.tracks[0].state.time_ns, 20);
        Ok(())
    }

    #[derive(Clone, Default)]
    struct FailingInitiator;

    impl Initiator<State, f64, ()> for FailingInitiator {
        type Error = std::io::Error;

        fn initiate(
            &mut self,
            observations: &[&TimedObservation<f64, ()>],
        ) -> Result<Vec<InitiatedTrack<State>>, Self::Error> {
            if observations
                .iter()
                .any(|observation| observation.payload == 999.0)
            {
                return Err(std::io::Error::other("test initiation failure"));
            }
            Ok(observations
                .iter()
                .map(|observation| InitiatedTrack {
                    observation_id: observation.id.clone(),
                    state: State {
                        value: observation.payload,
                        time_ns: observation.measurement_time_ns,
                    },
                })
                .collect())
        }
    }

    #[test]
    fn failed_batch_does_not_change_tracks() -> Result<(), Box<dyn Error>> {
        let mut manager = TrackManager::new(
            Model,
            GlobalNearestNeighbor {
                missed_assignment_cost: 26.0,
            },
            SinglePosteriorReducer,
            FailingInitiator,
            HitCountLifecycle {
                confirmation_hits: 1,
                max_time_without_update_ns: 100,
            },
        );
        manager.process_batch(&batch("first", 0, 0, &[0.0]))?;
        assert!(
            manager
                .process_batch(&batch("failed", 1, 1, &[1.0, 999.0]))
                .is_err()
        );

        let result = manager.process_batch(&batch("after", 2, 2, &[]))?;
        assert_eq!(result.tracks[0].state.value, 0.0);
        assert_eq!(result.tracks[0].hit_count, 1);
        Ok(())
    }
}
