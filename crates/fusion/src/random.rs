use sha2::{Digest, Sha256};

/// A stateless named random source. Every draw is independently keyed so adding
/// unrelated effects cannot perturb an existing sequence.
#[derive(Debug, Clone, Copy)]
pub struct DeterministicRandom {
    root_seed: u64,
}

impl DeterministicRandom {
    pub fn new(root_seed: u64) -> Self {
        Self { root_seed }
    }

    pub fn uniform(&self, component: &str, event: u64, effect: &str, draw: u64) -> f64 {
        self.uniform_named(component, event, effect, "", draw)
    }

    pub fn uniform_named(
        &self,
        component: &str,
        event: u64,
        effect: &str,
        subject: &str,
        draw: u64,
    ) -> f64 {
        let mut hash = Sha256::new();
        hash.update(b"fusion-deterministic-random-1");
        hash.update(self.root_seed.to_le_bytes());
        hash.update((component.len() as u64).to_le_bytes());
        hash.update(component.as_bytes());
        hash.update(event.to_le_bytes());
        hash.update((effect.len() as u64).to_le_bytes());
        hash.update(effect.as_bytes());
        hash.update((subject.len() as u64).to_le_bytes());
        hash.update(subject.as_bytes());
        hash.update(draw.to_le_bytes());
        let bytes: [u8; 8] = hash.finalize()[..8].try_into().expect("fixed digest slice");
        let value = u64::from_le_bytes(bytes);
        ((value as f64) + 0.5) / ((u64::MAX as f64) + 1.0)
    }

    pub fn normal(&self, component: &str, event: u64, effect: &str, draw: u64) -> f64 {
        self.normal_named(component, event, effect, "", draw)
    }

    pub fn normal_named(
        &self,
        component: &str,
        event: u64,
        effect: &str,
        subject: &str,
        draw: u64,
    ) -> f64 {
        let u1 = self
            .uniform_named(component, event, effect, subject, draw * 2)
            .max(f64::MIN_POSITIVE);
        let u2 = self.uniform_named(component, event, effect, subject, draw * 2 + 1);
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn draws_are_repeatable_and_named() {
        let random = DeterministicRandom::new(42);
        assert_eq!(
            random.normal("imu", 3, "noise", 1),
            random.normal("imu", 3, "noise", 1)
        );
        assert_ne!(
            random.normal("imu", 3, "noise", 1),
            random.normal("imu", 3, "bias", 1)
        );
        assert_ne!(
            random.normal_named("camera", 3, "noise", "landmark-a", 0),
            random.normal_named("camera", 3, "noise", "landmark-b", 0)
        );
    }
}
