use crate::{ObservationId, TrackId, TrackStatus};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LifecycleEventKind {
    Created,
    Confirmed,
    Missed,
    Coasted,
    Deleted,
}

#[derive(Debug, Clone)]
pub struct LifecycleEvent<S> {
    pub time_ns: i64,
    pub track_id: TrackId,
    pub kind: LifecycleEventKind,
    pub observation_id: Option<ObservationId>,
    pub status: TrackStatus,
    pub state: S,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BatchDiagnostics {
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
