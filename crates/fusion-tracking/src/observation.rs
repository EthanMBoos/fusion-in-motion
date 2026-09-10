use std::fmt;

macro_rules! string_id {
    ($name:ident) => {
        #[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(String);

        impl $name {
            pub fn new(value: impl Into<String>) -> Self {
                Self(value.into())
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
                self.0.fmt(formatter)
            }
        }

        impl From<&str> for $name {
            fn from(value: &str) -> Self {
                Self::new(value)
            }
        }

        impl From<String> for $name {
            fn from(value: String) -> Self {
                Self::new(value)
            }
        }
    };
}

string_id!(BatchId);
string_id!(ObservationId);
string_id!(TrackId);

#[derive(Debug, Clone)]
pub struct TimedObservation<D, C> {
    pub id: ObservationId,
    pub measurement_time_ns: i64,
    pub payload: D,
    pub context: C,
}

#[derive(Debug, Clone)]
pub struct ObservationBatch<D, C> {
    pub id: BatchId,
    /// Event time used for an empty batch and for the output snapshot.
    pub measurement_time_ns: i64,
    pub arrival_time_ns: i64,
    pub observations: Vec<TimedObservation<D, C>>,
}

impl<D, C> ObservationBatch<D, C> {
    pub fn output_time_ns(&self) -> i64 {
        self.observations
            .iter()
            .map(|observation| observation.measurement_time_ns)
            .max()
            .unwrap_or(self.measurement_time_ns)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identifiers_keep_caller_supplied_values() {
        let batch = BatchId::new("source:0007");
        let observation = ObservationId::new("source:0007:3");
        let track = TrackId::new("track-042");

        assert_eq!(batch.as_str(), "source:0007");
        assert_eq!(observation.as_str(), "source:0007:3");
        assert_eq!(track.as_str(), "track-042");
    }

    #[test]
    fn empty_batch_keeps_its_measurement_time() {
        let batch = ObservationBatch::<(), ()> {
            id: "empty".into(),
            measurement_time_ns: 17,
            arrival_time_ns: 29,
            observations: Vec::new(),
        };
        assert_eq!(batch.output_time_ns(), 17);
    }
}
