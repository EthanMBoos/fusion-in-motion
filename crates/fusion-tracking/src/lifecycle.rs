use std::error::Error;

use crate::{ObservationId, TimedObservation, TrackId};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackStatus {
    Tentative,
    Confirmed,
}

#[derive(Debug, Clone)]
pub struct ManagedTrack<S> {
    pub id: TrackId,
    pub state: S,
    pub status: TrackStatus,
    pub hit_count: usize,
    pub miss_count: usize,
    pub last_update_time_ns: i64,
}

#[derive(Debug, Clone)]
pub struct InitiatedTrack<S> {
    pub observation_id: ObservationId,
    pub state: S,
}

pub trait Initiator<S, D, C> {
    type Error: Error + Send + Sync + 'static;

    fn initiate(
        &mut self,
        observations: &[&TimedObservation<D, C>],
    ) -> Result<Vec<InitiatedTrack<S>>, Self::Error>;
}

pub trait LifecyclePolicy<S> {
    fn after_hit(&mut self, track: &mut ManagedTrack<S>, time_ns: i64);
    fn after_miss(&mut self, track: &mut ManagedTrack<S>, time_ns: i64);
    fn should_delete(&self, track: &ManagedTrack<S>, time_ns: i64) -> bool;
}

#[derive(Debug, Clone, Copy)]
pub struct HitCountLifecycle {
    pub confirmation_hits: usize,
    pub max_time_without_update_ns: i64,
}

impl<S> LifecyclePolicy<S> for HitCountLifecycle {
    fn after_hit(&mut self, track: &mut ManagedTrack<S>, time_ns: i64) {
        track.hit_count += 1;
        track.miss_count = 0;
        track.last_update_time_ns = time_ns;
        if track.hit_count >= self.confirmation_hits {
            track.status = TrackStatus::Confirmed;
        }
    }

    fn after_miss(&mut self, track: &mut ManagedTrack<S>, _time_ns: i64) {
        track.miss_count += 1;
    }

    fn should_delete(&self, track: &ManagedTrack<S>, time_ns: i64) -> bool {
        time_ns.saturating_sub(track.last_update_time_ns) >= self.max_time_without_update_ns
    }
}
