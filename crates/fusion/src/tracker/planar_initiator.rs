use std::convert::Infallible;

use fusion_tracking::{InitiatedTrack, InitiationCandidate, Initiator};

use super::{
    input::{Detection, DetectionContext, DetectionScanContext},
    planar_model::initialize,
    planar_state::PlanarTrackFilter,
};

#[derive(Debug, Clone, Copy)]
pub(super) struct PlanarInitiator {
    pub(super) minimum_unassigned_probability: f64,
}

impl Initiator<PlanarTrackFilter, Detection, DetectionContext, DetectionScanContext>
    for PlanarInitiator
{
    type Error = Infallible;

    fn initiate(
        &mut self,
        _context: &DetectionScanContext,
        candidates: &[InitiationCandidate<'_, Detection, DetectionContext>],
    ) -> Result<Vec<InitiatedTrack<PlanarTrackFilter>>, Self::Error> {
        Ok(candidates
            .iter()
            .filter(|candidate| {
                candidate.unassigned_probability >= self.minimum_unassigned_probability
            })
            .filter_map(|candidate| {
                let observation = candidate.observation;
                let Detection::Lidar(detection) = &observation.payload else {
                    return None;
                };
                let ego_pose = observation.context.ego_pose?;
                Some(InitiatedTrack {
                    observation_id: observation.id.clone(),
                    state: initialize(detection, ego_pose, observation.measurement_time_ns),
                })
            })
            .collect())
    }
}
