use std::error::Error;

use crate::{DetectionOpportunity, ObservationId, TimedObservation, TrackId};

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
    /// Latest time the sensor had a detection opportunity for this track.
    /// Out-of-coverage scans do not advance this value toward deletion.
    pub last_observable_time_ns: i64,
}

#[derive(Debug, Clone)]
pub struct InitiatedTrack<S> {
    pub observation_id: ObservationId,
    pub state: S,
}

#[derive(Debug, Clone, Copy)]
pub struct InitiationCandidate<'a, D, C> {
    pub observation: &'a TimedObservation<D, C>,
    /// Probability mass not assigned to any existing track.
    ///
    /// This is `1 - sum(track association probabilities)` for the observation.
    /// The initiator chooses the threshold appropriate for the application.
    pub unassigned_probability: f64,
}

pub trait Initiator<S, D, C, B> {
    type Error: Error + Send + Sync + 'static;

    fn initiate(
        &mut self,
        context: &B,
        candidates: &[InitiationCandidate<'_, D, C>],
    ) -> Result<Vec<InitiatedTrack<S>>, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AssociationEvidence {
    /// Sum of this track's observation-association probabilities.
    pub associated_probability: f64,
    /// Probability assigned to this track's missed-detection branch.
    pub missed_probability: f64,
    pub detection_opportunity: DetectionOpportunity,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackUpdate {
    Hit,
    Miss,
    Coast,
}

pub trait LifecyclePolicy<S> {
    fn update(
        &mut self,
        track: &mut ManagedTrack<S>,
        time_ns: i64,
        evidence: AssociationEvidence,
    ) -> TrackUpdate;

    fn should_delete(&self, track: &ManagedTrack<S>, time_ns: i64) -> bool;
}

#[derive(Debug, Clone, Copy)]
pub struct HitCountLifecycle {
    pub confirmation_hits: usize,
    pub max_time_without_update_ns: i64,
    pub hit_probability_threshold: f64,
}

impl<S> LifecyclePolicy<S> for HitCountLifecycle {
    fn update(
        &mut self,
        track: &mut ManagedTrack<S>,
        time_ns: i64,
        evidence: AssociationEvidence,
    ) -> TrackUpdate {
        let update = match evidence.detection_opportunity {
            _ if evidence.associated_probability >= self.hit_probability_threshold => {
                TrackUpdate::Hit
            }
            DetectionOpportunity::NotObservable => TrackUpdate::Coast,
            DetectionOpportunity::Observable {
                detection_probability: 0.0,
            } => TrackUpdate::Coast,
            DetectionOpportunity::Observable { .. } => TrackUpdate::Miss,
        };
        match update {
            TrackUpdate::Hit => {
                track.hit_count += 1;
                track.miss_count = 0;
                track.last_update_time_ns = time_ns;
                track.last_observable_time_ns = time_ns;
                if track.hit_count >= self.confirmation_hits {
                    track.status = TrackStatus::Confirmed;
                }
            }
            TrackUpdate::Miss => {
                track.miss_count += 1;
                track.last_observable_time_ns = time_ns;
            }
            TrackUpdate::Coast => {}
        }
        update
    }

    fn should_delete(&self, track: &ManagedTrack<S>, _time_ns: i64) -> bool {
        track
            .last_observable_time_ns
            .saturating_sub(track.last_update_time_ns)
            >= self.max_time_without_update_ns
    }
}
