//! Fail-closed admission requirements for the first playable milestone.
use std::collections::BTreeSet;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Capability {
    LiveLocalWorld,
    AtomicOwnershipAndSave,
    TransformedAuthoritativeActions,
    LiveLocalRendering,
    MovingContactAndSupport,
    CoherentTickPublication,
}

pub const REQUIRED: [Capability; 6] = [
    Capability::LiveLocalWorld,
    Capability::AtomicOwnershipAndSave,
    Capability::TransformedAuthoritativeActions,
    Capability::LiveLocalRendering,
    Capability::MovingContactAndSupport,
    Capability::CoherentTickPublication,
];

/// A report supplied by the integration layer. Setting a flag does not verify
/// an SDK declaration, actual host wiring, runtime behavior or source identity.
/// In particular, a positive report cannot turn these contracts into a host.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct CapabilityReport {
    available: BTreeSet<Capability>,
}

impl CapabilityReport {
    pub fn reported(available: impl IntoIterator<Item = Capability>) -> Self {
        Self {
            available: available.into_iter().collect(),
        }
    }

    pub fn missing(&self) -> Vec<Capability> {
        REQUIRED
            .iter()
            .filter(|capability| !self.available.contains(*capability))
            .copied()
            .collect()
    }

    pub fn require_playable_contracts(&self) -> std::result::Result<(), Vec<Capability>> {
        let missing = self.missing();
        if missing.is_empty() {
            Ok(())
        } else {
            Err(missing)
        }
    }
}
