use std::error::Error;

use crate::{LifecycleEvent, TrackId, TrackStatus};

/// Identity attached to an extracted target estimate.
///
/// `Unlabeled` is not a temporary or missing ID. It represents a backend whose
/// output is a target set without persistent identity.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TrackIdentity {
    Labeled(TrackId),
    Unlabeled,
}

impl TrackIdentity {
    pub fn track_id(&self) -> Option<&TrackId> {
        match self {
            Self::Labeled(track_id) => Some(track_id),
            Self::Unlabeled => None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct TrackReport<S> {
    pub identity: TrackIdentity,
    pub state: S,
    pub status: TrackStatus,
}

#[derive(Debug, Clone)]
pub struct TrackingOutput<S, Diagnostics> {
    pub measurement_time_ns: i64,
    pub arrival_time_ns: i64,
    pub tracks: Vec<TrackReport<S>>,
    pub lifecycle: Vec<LifecycleEvent<S>>,
    pub diagnostics: Diagnostics,
}

/// The common scan-to-output boundary for a complete tracker.
///
/// Implement this trait directly when the tracker owns state that cannot be
/// represented as one reduced state per live track. Backend-specific state and
/// diagnostics remain behind this API.
pub trait Tracker<Scan> {
    type State;
    type Diagnostics;
    type Error: Error + Send + Sync + 'static;

    fn process_scan(
        &mut self,
        scan: &Scan,
    ) -> Result<TrackingOutput<Self::State, Self::Diagnostics>, Self::Error>;
}
