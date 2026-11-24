//! Conversion of explicitly supplied host registry facts, never a replacement
//! registry or a fallback to the Phase 2 pinned_full_blocks test profile.
use crate::{Error, Result};
use sable_core::mass::{MassRegistry, Properties};
use std::collections::{BTreeMap, BTreeSet};

fn token(value: &str) -> bool {
    !value.is_empty()
        && value.bytes().all(|c| {
            c.is_ascii_lowercase() || c.is_ascii_digit() || matches!(c, b'_' | b'-' | b'.')
        })
}

fn block_name(value: &str) -> bool {
    let Some((namespace, path)) = value.split_once(':') else {
        return false;
    };
    token(namespace) && !path.is_empty() && path.split('/').all(token)
}

/// The complete host-supplied state definition, including every property and
/// its legal serialized values. This type does not query the SDK by itself.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StateDefinition {
    block: String,
    properties: BTreeMap<String, BTreeSet<String>>,
}

impl StateDefinition {
    pub fn new(block: String, definitions: Vec<(String, Vec<String>)>) -> Result<Self> {
        if !block_name(&block) {
            return Err(Error::InvalidState);
        }
        let mut properties = BTreeMap::new();
        for (name, values) in definitions {
            if !token(&name) || values.is_empty() || values.iter().any(|value| !token(value)) {
                return Err(Error::InvalidState);
            }
            let unique: BTreeSet<_> = values.iter().cloned().collect();
            if unique.len() != values.len() || properties.insert(name, unique).is_some() {
                return Err(Error::InvalidState);
            }
        }
        Ok(Self { block, properties })
    }

    /// All properties must be present exactly once. Partial constraints or
    /// missing defaults would collapse distinct live states and are refused.
    pub fn resolve(&self, values: Vec<(String, String)>) -> Result<CanonicalState> {
        if values.len() != self.properties.len() {
            return Err(Error::InvalidState);
        }
        let mut properties = BTreeMap::new();
        for (name, value) in values {
            if !self
                .properties
                .get(&name)
                .is_some_and(|allowed| allowed.contains(&value))
                || properties.insert(name, value).is_some()
            {
                return Err(Error::InvalidState);
            }
        }
        let mut key = self.block.clone();
        if !properties.is_empty() {
            key.push('[');
            for (index, (name, value)) in properties.iter().enumerate() {
                if index != 0 {
                    key.push(',');
                }
                key.push_str(name);
                key.push('=');
                key.push_str(value);
            }
            key.push(']');
        }
        Ok(CanonicalState { key })
    }
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct CanonicalState {
    key: String,
}

impl CanonicalState {
    pub fn key(&self) -> &str {
        &self.key
    }
}

/// Only verified state-invariant empty-context geometry is cacheable by state
/// alone. A level-dependent shape needs a future cell/context-aware route.
#[derive(Debug, Clone, PartialEq)]
pub struct HostGeometry {
    pub state_invariant: bool,
    pub solid: bool,
    pub full: bool,
    pub boxes: Vec<[f64; 6]>,
}

/// Explicit per-state Sable policy supplied by the adapter's property resolver.
/// There is deliberately no Default implementation or implicit mass of one.
#[derive(Debug, Clone, PartialEq)]
pub struct PhysicsPolicy {
    pub rule: String,
    pub mass: f64,
    pub center: [f64; 3],
    pub inertia: Option<[f64; 3]>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedState {
    epoch: u64,
    state: CanonicalState,
    properties: Properties,
    rule: String,
}

impl ResolvedState {
    pub fn epoch(&self) -> u64 {
        self.epoch
    }

    pub fn state(&self) -> &CanonicalState {
        &self.state
    }

    pub fn properties(&self) -> &Properties {
        &self.properties
    }

    pub fn rule(&self) -> &str {
        &self.rule
    }
}

/// A derived immutable-facts cache for one host registry epoch. It owns no live
/// blocks, BEs, inventory bytes, entities or scheduled ticks. Build a replacement
/// for a registry reload and reject callers still carrying the previous epoch.
#[derive(Debug, Clone)]
pub struct PropertyCatalog {
    epoch: u64,
    states: BTreeMap<String, ResolvedState>,
}

impl PropertyCatalog {
    pub fn new(epoch: u64) -> Result<Self> {
        if epoch == 0 {
            return Err(Error::StaleRegistry);
        }
        Ok(Self {
            epoch,
            states: BTreeMap::new(),
        })
    }

    pub fn epoch(&self) -> u64 {
        self.epoch
    }

    pub fn register(
        &mut self,
        state: CanonicalState,
        geometry: HostGeometry,
        policy: PhysicsPolicy,
    ) -> Result<()> {
        if self.states.contains_key(state.key()) {
            return Err(Error::DuplicateState);
        }
        if !geometry.state_invariant {
            return Err(Error::ContextDependentShape);
        }
        if geometry.boxes.iter().any(|b| {
            !b.iter()
                .all(|value| value.is_finite() && (*value as f32).is_finite())
                || (0..3).any(|axis| b[axis] >= b[axis + 3] || b[axis] as f32 >= b[axis + 3] as f32)
        }) || geometry.solid == geometry.boxes.is_empty()
            || (geometry.full && !geometry.solid)
        {
            return Err(Error::InvalidShape);
        }
        if policy.rule.trim().is_empty() {
            return Err(Error::InvalidPolicy);
        }
        let properties = Properties {
            solid: geometry.solid,
            full: geometry.full,
            mass: policy.mass,
            center: policy.center,
            inertia: policy.inertia,
            boxes: geometry.boxes,
        };
        // Reuse the accepted core's finite/mass/inertia validation before the
        // catalog changes. This temporary registry contains no live contents.
        let mut validation = MassRegistry::default();
        validation
            .register(state.key.clone(), properties.clone())
            .map_err(|_| Error::InvalidPolicy)?;
        self.states.insert(
            state.key.clone(),
            ResolvedState {
                epoch: self.epoch,
                state,
                properties,
                rule: policy.rule,
            },
        );
        Ok(())
    }

    pub fn get(&self, epoch: u64, key: &str) -> Result<&ResolvedState> {
        if epoch != self.epoch {
            return Err(Error::StaleRegistry);
        }
        self.states.get(key).ok_or(Error::UnknownState)
    }

    /// Resolve only requested canonical keys. This result supplies physics
    /// facts; it does not authorize live transfer or claim payload codec support.
    pub fn mass_registry<'a>(
        &self,
        epoch: u64,
        keys: impl IntoIterator<Item = &'a str>,
    ) -> Result<MassRegistry> {
        if epoch != self.epoch {
            return Err(Error::StaleRegistry);
        }
        let mut resolved = MassRegistry::default();
        for key in keys {
            let state = self.get(epoch, key)?;
            resolved
                .register(key.to_owned(), state.properties.clone())
                .map_err(|_| Error::InvalidPolicy)?;
        }
        Ok(resolved)
    }
}
