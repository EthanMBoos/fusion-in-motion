//! Reusable target-tracking interfaces and manager.
//!
//! Applications define the state, observations, models, serialization,
//! metrics, and visualization.

mod association;
mod diagnostics;
mod hypothesis;
mod lifecycle;
mod manager;
mod observation;

pub use association::{
    Assignment, AssociationEngine, AssociationError, AssociationPlan, GlobalNearestNeighbor,
    MinimumCostAssignmentError, TrackMarginal, minimum_cost_assignment,
};
pub use diagnostics::{BatchDiagnostics, LifecycleEvent, LifecycleEventKind};
pub use hypothesis::{
    GateDecision, HypothesisModel, HypothesisOutcome, ObservationHypothesis, PairHypothesis,
    PosteriorReducer, SinglePosteriorReducer, WeightedPosterior,
};
pub use lifecycle::{
    HitCountLifecycle, InitiatedTrack, Initiator, LifecyclePolicy, ManagedTrack, TrackStatus,
};
pub use manager::{BatchResult, TrackManager, TrackManagerError};
pub use observation::{BatchId, ObservationBatch, ObservationId, TimedObservation, TrackId};
