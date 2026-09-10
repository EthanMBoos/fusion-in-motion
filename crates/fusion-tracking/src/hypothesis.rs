use std::error::Error;

use crate::{ObservationId, TimedObservation, TrackId};

/// Whether the current scan could have detected a predicted track.
///
/// This is evaluated per track because field of view, range, occlusion, and
/// detection probability can differ across the same scan. `NotObservable`
/// lets lifecycle distinguish lack of coverage from an observable miss.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetectionOpportunity {
    Observable { detection_probability: f64 },
    NotObservable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateDecision {
    Inside,
    Outside,
    Invalid,
}

#[derive(Debug, Clone)]
pub enum HypothesisOutcome<S> {
    Inside { posterior: S },
    Outside,
    Invalid,
}

impl<S> HypothesisOutcome<S> {
    pub fn gate_decision(&self) -> GateDecision {
        match self {
            Self::Inside { .. } => GateDecision::Inside,
            Self::Outside => GateDecision::Outside,
            Self::Invalid => GateDecision::Invalid,
        }
    }

    pub fn posterior(&self) -> Option<&S> {
        match self {
            Self::Inside { posterior } => Some(posterior),
            Self::Outside | Self::Invalid => None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct ObservationHypothesis<S> {
    pub normalized_innovation_squared: Option<f64>,
    pub log_likelihood: Option<f64>,
    pub outcome: HypothesisOutcome<S>,
}

#[derive(Debug, Clone)]
pub struct PairHypothesis<S> {
    pub track_id: TrackId,
    pub observation_id: ObservationId,
    pub predicted: S,
    pub observation: ObservationHypothesis<S>,
}

pub trait HypothesisModel<S, D, C, B> {
    type Error: Error + Send + Sync + 'static;

    fn predict(&self, state: &S, time_ns: i64) -> Result<S, Self::Error>;

    fn hypothesize(
        &self,
        predicted: &S,
        observation: &TimedObservation<D, C>,
    ) -> Result<ObservationHypothesis<S>, Self::Error>;

    fn detection_opportunity(
        &self,
        predicted: &S,
        measurement_time_ns: i64,
        context: &B,
    ) -> Result<DetectionOpportunity, Self::Error>;
}

/// One observation-conditioned state at the reducer's common output time.
#[derive(Debug, Clone, Copy)]
pub struct WeightedPosterior<'a, S> {
    pub observation_id: &'a ObservationId,
    pub state: &'a S,
    pub probability: f64,
}

/// Reduces observation-conditioned and missed-detection branches to one state.
///
/// The manager advances every candidate to `reduction_time_ns` before calling
/// this API. A probabilistic reducer should include `missed_probability` in
/// the state and uncertainty update instead of treating a nonempty candidate
/// list as a certain hit.
pub trait PosteriorReducer<S> {
    type Error: Error + Send + Sync + 'static;

    fn reduce(
        &self,
        reduction_time_ns: i64,
        predicted: &S,
        candidates: &[WeightedPosterior<'_, S>],
        missed_probability: f64,
    ) -> Result<S, Self::Error>;
}

#[derive(Debug, Clone, Copy, Default)]
pub struct SinglePosteriorReducer;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SinglePosteriorError;

impl std::fmt::Display for SinglePosteriorError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str("hard assignment requires one selected posterior or a complete miss")
    }
}

impl Error for SinglePosteriorError {}

impl<S: Clone> PosteriorReducer<S> for SinglePosteriorReducer {
    type Error = SinglePosteriorError;

    fn reduce(
        &self,
        _reduction_time_ns: i64,
        _predicted: &S,
        candidates: &[WeightedPosterior<'_, S>],
        missed_probability: f64,
    ) -> Result<S, Self::Error> {
        match candidates {
            [] if missed_probability == 1.0 => Ok(_predicted.clone()),
            [candidate] if missed_probability == 0.0 && candidate.probability == 1.0 => {
                Ok(candidate.state.clone())
            }
            _ => Err(SinglePosteriorError),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hard_reducer_selects_only_weight_one_posterior() {
        let observation_id = ObservationId::new("observation-1");
        let posterior = 11;
        let result = SinglePosteriorReducer
            .reduce(
                5,
                &3,
                &[WeightedPosterior {
                    observation_id: &observation_id,
                    state: &posterior,
                    probability: 1.0,
                }],
                0.0,
            )
            .unwrap();
        assert_eq!(result, posterior);
    }

    #[test]
    fn hard_reducer_rejects_probabilistic_input() {
        let observation_id = ObservationId::new("observation-1");
        let posterior = 11;
        assert!(
            SinglePosteriorReducer
                .reduce(
                    5,
                    &3,
                    &[WeightedPosterior {
                        observation_id: &observation_id,
                        state: &posterior,
                        probability: 0.75,
                    }],
                    0.25,
                )
                .is_err()
        );
    }

    #[test]
    fn hard_reducer_keeps_prediction_after_a_miss() {
        let result = SinglePosteriorReducer.reduce(5, &3, &[], 1.0).unwrap();
        assert_eq!(result, 3);
    }
}
