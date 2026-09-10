use std::{collections::BTreeMap, error::Error};

use crate::{
    DetectionOpportunity, GateDecision, ObservationBatch, ObservationId, PairHypothesis, TrackId,
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Assignment {
    pub track_id: TrackId,
    pub observation_id: ObservationId,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TrackMarginal {
    pub track_id: TrackId,
    pub missed_probability: f64,
    pub observation_probabilities: Vec<(ObservationId, f64)>,
}

/// Association result for one scan.
///
/// A hard plan is one-to-one. A marginal plan describes every live track,
/// gives each track total probability one after including its miss branch, and
/// cannot assign more than total probability one to any observation. The
/// manager validates these conditions before changing tracker state.
#[derive(Debug, Clone, PartialEq)]
pub enum AssociationPlan {
    Hard(Vec<Assignment>),
    Marginal(Vec<TrackMarginal>),
}

#[derive(Debug, Clone, PartialEq)]
pub struct TrackDetectionOpportunity {
    pub track_id: TrackId,
    pub opportunity: DetectionOpportunity,
}

/// Selects or weights the validated track/observation hypotheses for one scan.
///
/// The typed batch and per-track detection opportunities are supplied in
/// addition to pair hypotheses because clutter, coverage, and detection
/// probability are scan-level inputs rather than pair-update outputs.
pub trait AssociationEngine<S, D, C, B> {
    type Error: Error + Send + Sync + 'static;

    fn associate(
        &self,
        batch: &ObservationBatch<D, C, B>,
        track_ids: &[TrackId],
        observation_ids: &[ObservationId],
        hypotheses: &[PairHypothesis<S>],
        detection_opportunities: &[TrackDetectionOpportunity],
    ) -> Result<AssociationPlan, Self::Error>;
}

#[derive(Debug, Clone, Copy)]
pub struct GlobalNearestNeighbor {
    pub missed_assignment_cost: f64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AssociationError {
    InvalidMissedAssignmentCost,
    IncompleteHypothesisMatrix,
    InvalidCostMatrix(MinimumCostAssignmentError),
}

impl std::fmt::Display for AssociationError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidMissedAssignmentCost => {
                formatter.write_str("missed-assignment cost must be finite and positive")
            }
            Self::IncompleteHypothesisMatrix => {
                formatter.write_str("hypothesis matrix is incomplete")
            }
            Self::InvalidCostMatrix(error) => write!(formatter, "invalid cost matrix: {error}"),
        }
    }
}

impl Error for AssociationError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::InvalidCostMatrix(error) => Some(error),
            Self::InvalidMissedAssignmentCost | Self::IncompleteHypothesisMatrix => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MinimumCostAssignmentError {
    TooFewColumns { rows: usize, columns: usize },
    RaggedRows,
    NonFiniteCost,
}

impl std::fmt::Display for MinimumCostAssignmentError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::TooFewColumns { rows, columns } => write!(
                formatter,
                "cost matrix has {rows} rows but only {columns} columns"
            ),
            Self::RaggedRows => formatter.write_str("cost matrix rows have different lengths"),
            Self::NonFiniteCost => formatter.write_str("cost matrix contains a non-finite value"),
        }
    }
}

impl Error for MinimumCostAssignmentError {}

impl<S, D, C, B> AssociationEngine<S, D, C, B> for GlobalNearestNeighbor {
    type Error = AssociationError;

    fn associate(
        &self,
        _batch: &ObservationBatch<D, C, B>,
        track_ids: &[TrackId],
        observation_ids: &[ObservationId],
        hypotheses: &[PairHypothesis<S>],
        _detection_opportunities: &[TrackDetectionOpportunity],
    ) -> Result<AssociationPlan, Self::Error> {
        if !self.missed_assignment_cost.is_finite() || self.missed_assignment_cost <= 0.0 {
            return Err(AssociationError::InvalidMissedAssignmentCost);
        }
        if track_ids.is_empty() || observation_ids.is_empty() {
            return Ok(AssociationPlan::Hard(Vec::new()));
        }

        let lookup = hypotheses
            .iter()
            .map(|hypothesis| {
                (
                    (&hypothesis.track_id, &hypothesis.observation_id),
                    hypothesis,
                )
            })
            .collect::<BTreeMap<_, _>>();
        let invalid_cost = self.missed_assignment_cost;
        let mut costs = Vec::with_capacity(track_ids.len());
        for track_id in track_ids {
            let mut row = Vec::with_capacity(observation_ids.len() + track_ids.len());
            for observation_id in observation_ids {
                let hypothesis = lookup
                    .get(&(track_id, observation_id))
                    .ok_or(AssociationError::IncompleteHypothesisMatrix)?;
                let cost = match (
                    hypothesis.observation.outcome.gate_decision(),
                    hypothesis.observation.normalized_innovation_squared,
                ) {
                    (GateDecision::Inside, Some(cost))
                        if cost.is_finite()
                            && cost >= 0.0
                            && cost < self.missed_assignment_cost =>
                    {
                        cost
                    }
                    _ => invalid_cost,
                };
                row.push(cost);
            }
            row.extend(std::iter::repeat_n(
                self.missed_assignment_cost,
                track_ids.len(),
            ));
            costs.push(row);
        }

        let assignments = minimum_cost_assignment(&costs)
            .map_err(AssociationError::InvalidCostMatrix)?
            .into_iter()
            .enumerate()
            .filter(|(track_index, observation_index)| {
                *observation_index < observation_ids.len()
                    && costs[*track_index][*observation_index] < self.missed_assignment_cost
            })
            .map(|(track_index, observation_index)| Assignment {
                track_id: track_ids[track_index].clone(),
                observation_id: observation_ids[observation_index].clone(),
            })
            .collect();
        Ok(AssociationPlan::Hard(assignments))
    }
}

/// Returns the selected column for each row of a finite rectangular cost matrix.
///
/// The number of columns must be at least the number of rows.
pub fn minimum_cost_assignment(
    costs: &[Vec<f64>],
) -> Result<Vec<usize>, MinimumCostAssignmentError> {
    let rows = costs.len();
    if rows == 0 {
        return Ok(Vec::new());
    }
    let columns = costs[0].len();
    if columns < rows {
        return Err(MinimumCostAssignmentError::TooFewColumns { rows, columns });
    }
    if costs.iter().any(|row| row.len() != columns) {
        return Err(MinimumCostAssignmentError::RaggedRows);
    }
    if costs.iter().flatten().any(|cost| !cost.is_finite()) {
        return Err(MinimumCostAssignmentError::NonFiniteCost);
    }

    let mut row_potential = vec![0.0; rows + 1];
    let mut column_potential = vec![0.0; columns + 1];
    let mut matched_row = vec![0_usize; columns + 1];
    let mut previous_column = vec![0_usize; columns + 1];

    for row in 1..=rows {
        matched_row[0] = row;
        let mut minimum = vec![f64::INFINITY; columns + 1];
        let mut used = vec![false; columns + 1];
        let mut column = 0;
        loop {
            used[column] = true;
            let current_row = matched_row[column];
            let mut delta = f64::INFINITY;
            let mut next_column = 0;
            for candidate in 1..=columns {
                if used[candidate] {
                    continue;
                }
                let reduced = costs[current_row - 1][candidate - 1]
                    - row_potential[current_row]
                    - column_potential[candidate];
                if reduced < minimum[candidate] {
                    minimum[candidate] = reduced;
                    previous_column[candidate] = column;
                }
                if minimum[candidate] < delta {
                    delta = minimum[candidate];
                    next_column = candidate;
                }
            }
            for candidate in 0..=columns {
                if used[candidate] {
                    row_potential[matched_row[candidate]] += delta;
                    column_potential[candidate] -= delta;
                } else {
                    minimum[candidate] -= delta;
                }
            }
            column = next_column;
            if matched_row[column] == 0 {
                break;
            }
        }
        loop {
            let previous = previous_column[column];
            matched_row[column] = matched_row[previous];
            column = previous;
            if column == 0 {
                break;
            }
        }
    }

    let mut assignment = vec![usize::MAX; rows];
    for column in 1..=columns {
        if matched_row[column] != 0 {
            assignment[matched_row[column] - 1] = column - 1;
        }
    }
    Ok(assignment)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hypothesis(track: &str, observation: &str, cost: f64) -> PairHypothesis<()> {
        PairHypothesis {
            track_id: track.into(),
            observation_id: observation.into(),
            predicted: (),
            observation: crate::ObservationHypothesis {
                normalized_innovation_squared: Some(cost),
                log_likelihood: None,
                outcome: crate::HypothesisOutcome::Inside { posterior: () },
            },
        }
    }

    fn batch() -> ObservationBatch<(), (), ()> {
        ObservationBatch {
            id: "batch".into(),
            measurement_time_ns: 0,
            arrival_time_ns: 0,
            context: (),
            observations: Vec::new(),
        }
    }

    fn opportunities(track_ids: &[TrackId]) -> Vec<TrackDetectionOpportunity> {
        track_ids
            .iter()
            .map(|track_id| TrackDetectionOpportunity {
                track_id: track_id.clone(),
                opportunity: DetectionOpportunity::Observable {
                    detection_probability: 1.0,
                },
            })
            .collect()
    }

    #[test]
    fn assignment_is_globally_optimal() {
        let costs = vec![vec![1.0, 2.0, 9.0], vec![1.1, 100.0, 9.0]];
        assert_eq!(minimum_cost_assignment(&costs).unwrap(), vec![1, 0]);
    }

    #[test]
    fn global_assignment_never_reuses_an_observation() {
        let tracks = vec![TrackId::new("a"), TrackId::new("b")];
        let observations = vec![ObservationId::new("x"), ObservationId::new("y")];
        let hypotheses = vec![
            hypothesis("a", "x", 1.0),
            hypothesis("a", "y", 2.0),
            hypothesis("b", "x", 1.1),
            hypothesis("b", "y", 100.0),
        ];
        let plan = GlobalNearestNeighbor {
            missed_assignment_cost: 9.0,
        }
        .associate(
            &batch(),
            &tracks,
            &observations,
            &hypotheses,
            &opportunities(&tracks),
        )
        .unwrap();
        let AssociationPlan::Hard(assignments) = plan else {
            unreachable!()
        };
        assert_eq!(assignments.len(), 2);
        assert_eq!(assignments[0].observation_id.as_str(), "y");
        assert_eq!(assignments[1].observation_id.as_str(), "x");
    }

    #[test]
    fn gated_candidate_cannot_be_selected() {
        let tracks = vec![TrackId::new("a")];
        let observations = vec![ObservationId::new("x")];
        let mut candidate = hypothesis("a", "x", 1.0);
        candidate.observation.outcome = crate::HypothesisOutcome::Outside;
        let plan = GlobalNearestNeighbor {
            missed_assignment_cost: 9.0,
        }
        .associate(
            &batch(),
            &tracks,
            &observations,
            &[candidate],
            &opportunities(&tracks),
        )
        .unwrap();
        assert_eq!(plan, AssociationPlan::Hard(Vec::new()));
    }

    #[test]
    fn malformed_cost_matrix_returns_an_error() {
        let ragged = vec![vec![1.0, 2.0], vec![3.0]];
        assert_eq!(
            minimum_cost_assignment(&ragged),
            Err(MinimumCostAssignmentError::RaggedRows)
        );
        assert_eq!(
            minimum_cost_assignment(&[vec![], vec![]]),
            Err(MinimumCostAssignmentError::TooFewColumns {
                rows: 2,
                columns: 0,
            })
        );
        assert_eq!(
            minimum_cost_assignment(&[vec![f64::NAN]]),
            Err(MinimumCostAssignmentError::NonFiniteCost)
        );
    }

    #[test]
    fn largest_finite_missed_cost_does_not_overflow() {
        let tracks = vec![TrackId::new("a"), TrackId::new("b")];
        let observations = vec![ObservationId::new("x"), ObservationId::new("y")];
        let candidates =
            [("a", "x"), ("a", "y"), ("b", "x"), ("b", "y")].map(|(track_id, observation_id)| {
                let mut candidate = hypothesis(track_id, observation_id, 1.0);
                candidate.observation.outcome = crate::HypothesisOutcome::Outside;
                candidate
            });
        let plan = GlobalNearestNeighbor {
            missed_assignment_cost: f64::MAX,
        }
        .associate(
            &batch(),
            &tracks,
            &observations,
            &candidates,
            &opportunities(&tracks),
        )
        .unwrap();
        assert_eq!(plan, AssociationPlan::Hard(Vec::new()));
    }
}
