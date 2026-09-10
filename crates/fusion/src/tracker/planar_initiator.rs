use std::convert::Infallible;

use fusion_tracking::{InitiatedTrack, Initiator, TimedObservation};

use super::{
    input::{Detection, DetectionContext},
    planar_model::initialize,
    planar_state::PlanarTrackFilter,
};

#[derive(Debug, Clone, Copy, Default)]
pub(super) struct PlanarInitiator;

impl Initiator<PlanarTrackFilter, Detection, DetectionContext> for PlanarInitiator {
    type Error = Infallible;

    fn initiate(
        &mut self,
        observations: &[&TimedObservation<Detection, DetectionContext>],
    ) -> Result<Vec<InitiatedTrack<PlanarTrackFilter>>, Self::Error> {
        Ok(observations
            .iter()
            .filter_map(|observation| {
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
