//! Reusable target-tracking API and point-target reference manager.
//!
//! Applications can implement [`Tracker`] directly or build a conventional
//! point-target backend from [`TrackManager`] and the component APIs. They
//! define the state, observations, models, serialization, metrics, and
//! visualization.

mod association;
mod diagnostics;
mod hypothesis;
mod lifecycle;
mod manager;
mod observation;
mod tracker;

pub use association::{
    Assignment, AssociationEngine, AssociationError, AssociationPlan, GlobalNearestNeighbor,
    MinimumCostAssignmentError, TrackDetectionOpportunity, TrackMarginal, minimum_cost_assignment,
};
pub use diagnostics::{BatchDiagnostics, LifecycleEvent, LifecycleEventKind};
pub use hypothesis::{
    DetectionOpportunity, GateDecision, HypothesisModel, HypothesisOutcome, ObservationHypothesis,
    PairHypothesis, PosteriorReducer, SinglePosteriorReducer, WeightedPosterior,
};
pub use lifecycle::{
    AssociationEvidence, HitCountLifecycle, InitiatedTrack, InitiationCandidate, Initiator,
    LifecyclePolicy, ManagedTrack, TrackStatus, TrackUpdate,
};
pub use manager::{TrackManager, TrackManagerDiagnostics, TrackManagerError};
pub use observation::{BatchId, ObservationBatch, ObservationId, TimedObservation, TrackId};
pub use tracker::{TrackIdentity, TrackReport, Tracker, TrackingOutput};
