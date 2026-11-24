//! Derived invalidation decisions. Live contents remain exclusively host-owned.
use crate::{Error, Result, registry::ResolvedState};
use sable_core::{Config, Id, Pos, mass::Properties};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CellIdentity {
    pub owner: Id,
    pub generation: u64,
    pub local: Pos,
}

/// Retained derived facts, without a block/BE/inventory payload. The host's
/// monotonically increasing revision covers state and live content changes.
#[derive(Debug, Clone, PartialEq)]
pub struct CellStamp {
    identity: CellIdentity,
    registry_epoch: u64,
    state: String,
    properties: Properties,
    revision: u64,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Invalidation {
    pub geometry: bool,
    pub mass: bool,
    pub render: bool,
    pub content: bool,
}

impl CellStamp {
    pub fn new(
        config: &Config,
        identity: CellIdentity,
        registry_epoch: u64,
        state: &ResolvedState,
        revision: u64,
    ) -> Result<Self> {
        config.validate().map_err(|_| Error::InvalidConfiguration)?;
        if !config.local_valid(identity.local) {
            return Err(Error::InvalidCoordinate);
        }
        if identity.owner.0 == [0; 16] || identity.generation == 0 {
            return Err(Error::StaleOwner);
        }
        if registry_epoch == 0 || registry_epoch != state.epoch() {
            return Err(Error::StaleRegistry);
        }
        Ok(Self {
            identity,
            registry_epoch,
            state: state.state().key().to_owned(),
            properties: state.properties().clone(),
            revision,
        })
    }

    pub fn classify(&self, next: &Self) -> Result<Invalidation> {
        if self.identity != next.identity {
            return Err(Error::StaleOwner);
        }
        if next.registry_epoch < self.registry_epoch {
            return Err(Error::StaleRegistry);
        }
        if next.revision < self.revision
            || (next.revision == self.revision && next.state != self.state)
        {
            return Err(Error::InvalidRevision);
        }
        let before = &self.properties;
        let after = &next.properties;
        let geometry =
            before.solid != after.solid || before.full != after.full || before.boxes != after.boxes;
        let mass = before.solid != after.solid
            || before.mass != after.mass
            || before.center != after.center
            || before.inertia != after.inertia;
        let content = self.revision != next.revision;
        Ok(Invalidation {
            geometry,
            mass,
            render: content || geometry || self.state != next.state,
            content,
        })
    }
}
