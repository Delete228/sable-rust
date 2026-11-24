//! Lifecycle policy for a host which owns every live block, inventory and ticker.
//!
//! The private registry below contains only derived canonical state keys and
//! logical metadata. It never executes a transfer or stores a live payload.
//! Each operation starts from fresh bounded host observations; the adapter binds
//! its plan to the real host's prepared token, revalidates inside that transaction
//! and supplies a receipt only after the actual host commit succeeds.
use crate::{
    Block, Config, Contents, Error, Handle, Id, Level, Motion, Payload, Pos, Registry, Result,
    gather, mass::MassRegistry,
};
use std::collections::{BTreeMap, BTreeSet};

/// An actual host observation of the bounded transfer domain. Omitted cells
/// in that domain are air; missing/unloaded observations must refuse upstream.
/// No BE data, schedules, entities or attachment payloads enter this cache.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StateSnapshot {
    pub revision: u64,
    pub cells: BTreeMap<Pos, String>,
}

/// Revisions returned by the actual successful host transfer, not an approval
/// token. The adapter keeps the host owner/generation/prepared-handle identity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CommitReceipt {
    pub source_revision: u64,
    pub destination_revision: u64,
}

/// Derived from a real completed owned-source checkpoint after retiring its
/// binding and persisting the caller tombstone. This is not a moved-cell receipt.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RetirementReceipt {
    pub barrier_epoch: u64,
    pub source_revision: u64,
    pub retired_generation: u64,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RecordedReceipt {
    Transfer(CommitReceipt),
    Retirement(RetirementReceipt),
    Batch(BatchReceipt),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PendingPhase {
    Prepared,
    Unconfirmed,
    Committed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Operation {
    Capture,
    Return,
    RetireEmpty,
    Split,
}

/// Actual atomic host batch outcome, with destination order matching the
/// prepared child list. It is queried after the real root transaction closes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BatchReceipt {
    pub barrier_epoch: u64,
    pub source_revision: u64,
    pub destination_revisions: Vec<u64>,
}

pub struct SplitDestination {
    pub id: Id,
    pub state: StateSnapshot,
}

/// The caller obtains `split` from the existing accepted SplitTracker, then
/// checks its current epoch again while reserving the lifecycle operation.
pub struct SplitRequest {
    pub parent: Handle,
    pub source: StateSnapshot,
    pub split: crate::split::SplitPlan,
    pub tracker_epoch: u64,
    pub children: Vec<SplitDestination>,
    pub motion: Motion,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SplitChild {
    pub id: Id,
    pub plot: u32,
    pub generation: u64,
    pub mapping: Vec<(Pos, Pos)>,
    pub motion: Motion,
}

/// Owned policy description usable after releasing the adapter's state borrow.
/// It authorizes no host action by itself. Only one plan may be pending.
#[derive(Debug)]
pub struct LifecyclePlan {
    serial: u64,
    epoch: u64,
    operation: Operation,
    handle: Handle,
    plot: u32,
    mapping: Vec<(Pos, Pos)>,
    motion: Motion,
    split_children: Vec<SplitChild>,
    tracker_epoch: Option<u64>,
}
impl LifecyclePlan {
    pub fn operation(&self) -> Operation {
        self.operation
    }
    pub fn id(&self) -> Id {
        self.handle.id()
    }
    pub fn plot(&self) -> u32 {
        self.plot
    }
    pub fn generation(&self) -> u64 {
        self.handle.generation
    }
    /// Capture/return mapping. Splits use the explicitly routed child mappings.
    pub fn mapping(&self) -> &[(Pos, Pos)] {
        &self.mapping
    }
    pub fn split_children(&self) -> &[SplitChild] {
        &self.split_children
    }
    pub fn motion(&self) -> &Motion {
        &self.motion
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Metadata {
    pub handle: Handle,
    pub plot: u32,
    pub generation: u64,
    /// Last adopted derived motion. Physical publication/persistence must use
    /// the actual current native/logical motion supplied by the adapter.
    pub motion: Motion,
    /// Last adopted host observation epoch. The host may also advance it for
    /// content-only changes; no inventory data is inferred or retained here.
    pub host_revision: u64,
}

/// Logical identity only. Physical motion is stored once in the caller's
/// canonical host-journal metadata, not duplicated in this identity record.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogicalIdentity {
    pub id: Id,
    pub plot: u32,
    pub generation: u64,
    pub parent: Option<Id>,
}

/// Bounded canonical metadata; zero generation floors are omitted. The caller
/// persists this alongside its durable lifecycle phase in the real host journal.
/// No geometry, inventory or cached host observation epoch is serialized here.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PersistentState {
    pub owners: Vec<LogicalIdentity>,
    pub plot_generations: BTreeMap<u32, u64>,
}

/// Supplied from the real current owner after validating its durable phase,
/// binding identity and canonical physical metadata. State keys are re-read
/// from the actual host, never loaded from this module's former geometry cache.
#[derive(Clone)]
pub struct RestoredState {
    pub state: StateSnapshot,
    pub motion: Motion,
}

struct Pending {
    serial: u64,
    epoch: u64,
    operation: Operation,
    handle: Handle,
    before_source: StateSnapshot,
    before_destination: Vec<StateSnapshot>,
    after_source: BTreeMap<Pos, String>,
    after_destination: Vec<BTreeMap<Pos, String>>,
    staged: Registry,
    receipt: Option<RecordedReceipt>,
    unconfirmed: bool,
    retired_generation: Option<u64>,
    split_handles: Vec<Handle>,
}

/// A failed readback after a successful host commit is explicit pending work.
/// It cannot be cancelled or treated as if the old host ownership survived.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PendingCommit {
    pub operation: Operation,
    pub id: Id,
    pub receipt: RecordedReceipt,
}

pub struct HostLifecycle {
    registry: Registry,
    host_revisions: BTreeMap<Id, u64>,
    epoch: u64,
    next_serial: u64,
    pending: Option<Pending>,
}
impl HostLifecycle {
    pub fn new(config: Config) -> Result<Self> {
        Ok(Self {
            registry: Registry::new(config)?,
            host_revisions: BTreeMap::new(),
            epoch: 0,
            next_serial: 0,
            pending: None,
        })
    }
    pub fn restore(
        config: Config,
        saved: PersistentState,
        mut actual: BTreeMap<Id, RestoredState>,
    ) -> Result<Self> {
        let mut restored = Self::new(config)?;
        if saved.owners.len() > restored.registry.config().slots() as usize
            || saved.plot_generations.len() > restored.registry.config().slots() as usize
            || actual.len() != saved.owners.len()
        {
            return Err(Error::CorruptSave);
        }
        let mut generations = vec![0u64; restored.registry.config().slots() as usize];
        for (&plot, &generation) in &saved.plot_generations {
            let target = generations
                .get_mut(plot as usize)
                .ok_or(Error::CorruptSave)?;
            if generation == 0 {
                return Err(Error::CorruptSave);
            }
            *target = generation;
        }
        let mut plots = BTreeSet::new();
        let mut identities = BTreeMap::new();
        let mut levels = Vec::new();
        for logical in saved.owners {
            if logical.id.0 == [0; 16]
                || logical.generation == 0
                || generations.get(logical.plot as usize) != Some(&logical.generation)
                || !plots.insert(logical.plot)
                || identities.insert(logical.id, logical.generation).is_some()
            {
                return Err(Error::CorruptSave);
            }
            let observed = actual.remove(&logical.id).ok_or(Error::CorruptSave)?;
            let mut level = Level::new(logical.id);
            level.plot = logical.plot;
            level.parent = logical.parent;
            level.motion = observed.motion;
            level.contents = restored.checked_contents(&observed.state, true)?;
            restored
                .host_revisions
                .insert(logical.id, observed.state.revision);
            levels.push((level, crate::Residency::Live));
        }
        if !actual.is_empty() {
            return Err(Error::CorruptSave);
        }
        // Registry restoration supplies a new ephemeral registry owner, so all
        // pre-restart handles remain stale even when logical generations match.
        let mut registry = Registry::restore(restored.registry.config().clone(), levels)?;
        registry.generations = generations;
        for (id, entry) in &mut registry.entries {
            entry.handle.generation = identities[id];
        }
        let _ = registry.drain_changes();
        registry.validate()?;
        restored.registry = registry;
        Ok(restored)
    }
    fn state_of(registry: &Registry) -> PersistentState {
        PersistentState {
            owners: registry
                .entries
                .values()
                .map(|entry| LogicalIdentity {
                    id: entry.level.id,
                    plot: entry.level.plot,
                    generation: entry.handle.generation,
                    parent: entry.level.parent,
                })
                .collect(),
            plot_generations: registry
                .generations
                .iter()
                .enumerate()
                .filter(|(_, generation)| **generation != 0)
                .map(|(plot, &generation)| (plot as u32, generation))
                .collect(),
        }
    }
    pub fn persistent_state(&self) -> Result<PersistentState> {
        if self.host_unresolved() {
            return Err(Error::Interrupted);
        }
        Ok(Self::state_of(&self.registry))
    }
    /// Encode this staged logical identity in the caller's OwnedUnprojected
    /// journal phase before the real host root transaction closes. This is
    /// planned metadata, not a claim that the transfer already committed.
    pub fn planned_persistent_state(&self, plan: &LifecyclePlan) -> Result<PersistentState> {
        Ok(Self::state_of(&self.pending_for(plan)?.staged))
    }
    pub fn metadata(&self, handle: Handle) -> Result<Metadata> {
        if self.host_unresolved() {
            return Err(Error::Interrupted);
        }
        let level = self.registry.get(handle)?;
        Ok(Metadata {
            handle,
            plot: level.plot,
            generation: handle.generation,
            motion: level.motion.clone(),
            host_revision: *self
                .host_revisions
                .get(&level.id)
                .ok_or(Error::StaleHandle)?,
        })
    }
    pub fn handle(&self, id: Id) -> Result<Handle> {
        if self.host_unresolved() {
            return Err(Error::Interrupted);
        }
        self.registry.handle(id)
    }
    pub fn pending_commit(&self) -> Option<PendingCommit> {
        let pending = self.pending.as_ref()?;
        Some(PendingCommit {
            operation: pending.operation,
            id: pending.handle.id(),
            receipt: pending.receipt.clone()?,
        })
    }
    fn host_unresolved(&self) -> bool {
        self.pending
            .as_ref()
            .is_some_and(|pending| pending.unconfirmed || pending.receipt.is_some())
    }
    pub fn pending_phase(&self) -> Option<PendingPhase> {
        self.pending.as_ref().map(|pending| {
            if pending.receipt.is_some() {
                PendingPhase::Committed
            } else if pending.unconfirmed {
                PendingPhase::Unconfirmed
            } else {
                PendingPhase::Prepared
            }
        })
    }
    /// Preserve an uncertain/fenced host outcome. No rollback is inferred from
    /// a closure error or missing receipt, and old metadata cannot remain live.
    pub fn mark_unconfirmed(&mut self, plan: &LifecyclePlan) -> Result<()> {
        if self.pending_for(plan)?.receipt.is_some() {
            return Err(Error::HostRefused);
        }
        self.pending.as_mut().unwrap().unconfirmed = true;
        Ok(())
    }
    /// Only after the real host reports its durable NotCommitted result. A
    /// caller timeout, void transaction close or guessed post-state is not that
    /// result. A recorded successful receipt can never take this path.
    pub fn confirm_not_committed(&mut self, plan: &LifecyclePlan) -> Result<()> {
        let pending = self.pending_for(plan)?;
        if !pending.unconfirmed || pending.receipt.is_some() {
            return Err(Error::HostRefused);
        }
        self.pending.as_mut().unwrap().unconfirmed = false;
        self.cancel(plan)
    }
    fn available(&self) -> Result<u64> {
        if self.pending.is_some() {
            return Err(Error::HostRefused);
        }
        // Leave capacity for cancellation/adoption before a host can mutate.
        self.epoch.checked_add(1).ok_or(Error::Capacity)?;
        self.next_serial.checked_add(1).ok_or(Error::Capacity)
    }
    /// Refresh this derived cache only after the real host has accepted the
    /// observation and pose. Before global lifecycle policy checks the adapter
    /// refreshes ALL affected/current other owners; cached others are never a
    /// substitute for fresh authoritative host observations.
    pub fn publish_observation(
        &mut self,
        handle: Handle,
        state: StateSnapshot,
        motion: Motion,
        masses: &MassRegistry,
    ) -> Result<()> {
        if self.pending.is_some() {
            return Err(Error::HostRefused);
        }
        let current = self.registry.get(handle)?;
        let old_revision = *self
            .host_revisions
            .get(&handle.id())
            .ok_or(Error::StaleHandle)?;
        if state.revision < old_revision {
            return Err(Error::StaleHandle);
        }
        let geometry = self.checked_contents(&state, true)?;
        let changed = current.contents.blocks != geometry.blocks;
        if changed && state.revision == old_revision {
            return Err(Error::StaleHandle);
        }
        if !geometry.blocks.is_empty() {
            masses.compute(&geometry)?;
        }
        let mut candidate = current.clone();
        candidate.contents = geometry;
        candidate.motion = motion;
        self.registry.validate_level(&candidate)?;
        if !changed && state.revision == old_revision && candidate.motion == current.motion {
            return Ok(());
        }
        let epoch = self.epoch.checked_add(1).ok_or(Error::Capacity)?;
        if changed {
            let mut staged = self.registry.staged();
            staged
                .entries
                .get_mut(&handle.id())
                .ok_or(Error::StaleHandle)?
                .level = candidate;
            staged.changed(handle.id(), crate::ChangeKind::Edited)?;
            staged.validate()?;
            let _ = staged.drain_changes();
            self.registry = staged;
        } else {
            self.registry
                .entries
                .get_mut(&handle.id())
                .ok_or(Error::StaleHandle)?
                .level
                .motion = candidate.motion;
        }
        self.host_revisions.insert(handle.id(), state.revision);
        self.epoch = epoch;
        Ok(())
    }
    fn checked_contents(&self, snapshot: &StateSnapshot, local: bool) -> Result<Contents> {
        if snapshot.cells.len() > self.registry.config().max_records {
            return Err(Error::Capacity);
        }
        let mut contents = Contents::default();
        for (position, state) in &snapshot.cells {
            if !gather::in_domain(*position)
                || (local && !self.registry.config().local_valid(*position))
            {
                return Err(Error::InvalidCoordinate);
            }
            if state.is_empty()
                || matches!(
                    state.as_str(),
                    "minecraft:air" | "minecraft:cave_air" | "minecraft:void_air"
                )
            {
                return Err(Error::InvalidInput);
            }
            contents.blocks.insert(
                *position,
                Block {
                    state: state.clone(),
                    block_entity: None,
                    extension: Payload::default(),
                },
            );
        }
        Ok(contents)
    }
    fn mapping(
        &self,
        source: &StateSnapshot,
        destination: &StateSnapshot,
        positions: &[Pos],
        offset: Pos,
        local_destination: bool,
    ) -> Result<Vec<(Pos, Pos)>> {
        if positions.is_empty() {
            return Err(Error::InvalidInput);
        }
        if positions.len() > self.registry.config().max_records
            || destination
                .cells
                .len()
                .checked_add(positions.len())
                .is_none_or(|n| n > self.registry.config().max_records)
        {
            return Err(Error::Capacity);
        }
        let mut selected = BTreeSet::new();
        let mut mapping = Vec::with_capacity(positions.len());
        for &from in positions {
            if !source.cells.contains_key(&from) || !selected.insert(from) {
                return Err(Error::InvalidInput);
            }
            let mut to = [0; 3];
            for (axis, target) in to.iter_mut().enumerate() {
                *target = from[axis]
                    .checked_add(offset[axis])
                    .ok_or(Error::InvalidCoordinate)?;
            }
            if !gather::in_domain(to)
                || (local_destination && !self.registry.config().local_valid(to))
            {
                return Err(Error::InvalidCoordinate);
            }
            if destination.cells.contains_key(&to) {
                return Err(Error::Occupied);
            }
            mapping.push((from, to));
        }
        Ok(mapping)
    }
    fn after(
        source: &StateSnapshot,
        destination: &StateSnapshot,
        mapping: &[(Pos, Pos)],
    ) -> (BTreeMap<Pos, String>, BTreeMap<Pos, String>) {
        let mut after_source = source.cells.clone();
        let mut after_destination = destination.cells.clone();
        for &(from, to) in mapping {
            let state = after_source.remove(&from).expect("checked selection");
            after_destination.insert(to, state);
        }
        (after_source, after_destination)
    }
    /// Select with existing `gather::connected` policy, admit each actual state
    /// and host shape, then pass the resulting ordered selection here. The
    /// destination is a new empty host-owned local world.
    pub fn prepare_capture(
        &mut self,
        source: StateSnapshot,
        destination: StateSnapshot,
        id: Id,
        anchor: Pos,
        positions: &[Pos],
        masses: &MassRegistry,
    ) -> Result<LifecyclePlan> {
        let serial = self.available()?;
        self.checked_contents(&source, false)?;
        self.checked_contents(&destination, true)?;
        if !destination.cells.is_empty() || !gather::in_domain(anchor) {
            return Err(Error::InvalidInput);
        }
        let mut offset = [0; 3];
        for (axis, translation) in offset.iter_mut().enumerate() {
            *translation = self.registry.config().anchor()[axis]
                .checked_sub(anchor[axis])
                .ok_or(Error::InvalidCoordinate)?;
        }
        let mapping = self.mapping(&source, &destination, positions, offset, true)?;
        let (after_source, after_destination) = Self::after(&source, &destination, &mapping);
        let geometry = self.checked_contents(
            &StateSnapshot {
                revision: destination.revision,
                cells: after_destination.clone(),
            },
            true,
        )?;
        let mass = masses.compute(&geometry)?;
        let mut level = Level::new(id);
        level.contents = geometry;
        // Same translation-only assembly convention as Registry::assemble.
        level.motion.pose.pivot = mass.center;
        level.motion.pose.position =
            std::array::from_fn(|axis| mass.center[axis] - f64::from(offset[axis]));
        let mut staged = self.registry.staged();
        let handle = staged.insert(level)?;
        // Registry events belong to its standalone owned-store consumers.
        // This private derived cache has no such consumer or event history.
        let _ = staged.drain_changes();
        let staged_level = staged.get(handle)?;
        let plan = LifecyclePlan {
            serial,
            epoch: self.epoch,
            operation: Operation::Capture,
            handle,
            plot: staged_level.plot,
            mapping,
            motion: staged_level.motion.clone(),
            split_children: Vec::new(),
            tracker_epoch: None,
        };
        self.pending = Some(Pending {
            serial,
            epoch: self.epoch,
            operation: Operation::Capture,
            handle,
            before_source: source,
            before_destination: vec![destination],
            after_source,
            after_destination: vec![after_destination],
            staged,
            receipt: None,
            unconfirmed: false,
            retired_generation: None,
            split_handles: Vec::new(),
        });
        self.next_serial = serial;
        Ok(plan)
    }
    /// Plan an explicit translation-only return of ALL currently observed local
    /// cells. The caller supplies the current actual native/logical motion,
    /// never a cached initial pose. Rotated/scaled returns need a real host state
    /// transform codec and are refused by this bounded slice.
    pub fn prepare_return(
        &mut self,
        handle: Handle,
        source: StateSnapshot,
        destination: StateSnapshot,
        offset: Pos,
        motion: Motion,
        masses: &MassRegistry,
    ) -> Result<LifecyclePlan> {
        let serial = self.available()?;
        let existing = self.registry.get(handle)?;
        let plot = existing.plot;
        let geometry = self.checked_contents(&source, true)?;
        self.checked_contents(&destination, false)?;
        masses.compute(&geometry)?;
        if !motion.pose.valid()
            || motion.pose.rotation != [0., 0., 0., 1.]
            || motion.pose.scale != [1.; 3]
            || motion
                .linear
                .iter()
                .chain(&motion.angular)
                .any(|v| !v.is_finite())
        {
            return Err(Error::InvalidInput);
        }
        let positions = source.cells.keys().copied().collect::<Vec<_>>();
        let mapping = self.mapping(&source, &destination, &positions, offset, false)?;
        let (after_source, after_destination) = Self::after(&source, &destination, &mapping);
        let mut staged = self.registry.staged();
        // Only derived metadata is removed. No contents returned by the old
        // owned-store API leave this wrapper or reach a real host writer.
        let _ = staged.delete(handle)?;
        staged.validate()?;
        let _ = staged.drain_changes();
        let plan = LifecyclePlan {
            serial,
            epoch: self.epoch,
            operation: Operation::Return,
            handle,
            plot,
            mapping,
            motion,
            split_children: Vec::new(),
            tracker_epoch: None,
        };
        self.pending = Some(Pending {
            serial,
            epoch: self.epoch,
            operation: Operation::Return,
            handle,
            before_source: source,
            before_destination: vec![destination],
            after_source,
            after_destination: vec![after_destination],
            staged,
            receipt: None,
            unconfirmed: false,
            retired_generation: None,
            split_handles: Vec::new(),
        });
        self.next_serial = serial;
        Ok(plan)
    }
    /// Stage all R5 child identities and geometry from ONE frozen current
    /// parent observation. This performs no host write and emits no transfer
    /// event. The actual backend must stage every child in one atomic batch.
    pub fn prepare_split(
        &mut self,
        request: SplitRequest,
        masses: &MassRegistry,
    ) -> Result<LifecyclePlan> {
        let serial = self.available()?;
        self.registry.get(request.parent)?;
        if request.split.source_epoch != request.tracker_epoch
            || request.split.children.is_empty()
            || request.split.children.len() != request.children.len()
            || request.children.len() >= self.registry.config().slots() as usize
            || !request.motion.pose.valid()
            || request
                .motion
                .linear
                .iter()
                .chain(&request.motion.angular)
                .any(|v| !v.is_finite())
        {
            return Err(Error::InvalidInput);
        }
        let original = self.checked_contents(&request.source, true)?;
        let original_mass = masses.compute(&original)?;
        let original_com = request.motion.pose.transform(original_mass.center);
        let mut remaining = original.clone();
        let mut selected = BTreeSet::new();
        let mut staged = self.registry.staged();
        let mut before_destination = Vec::new();
        let mut after_destination = Vec::new();
        let mut split_handles = Vec::new();
        let mut children = Vec::new();
        for (component, destination) in request.split.children.iter().zip(request.children) {
            self.checked_contents(&destination.state, true)?;
            if component.is_empty() || !destination.state.cells.is_empty() {
                return Err(Error::InvalidInput);
            }
            let mut child = Level::new(destination.id);
            child.parent = Some(request.parent.id());
            child.motion = request.motion.clone();
            let mut mapping = Vec::new();
            for &position in component {
                if !selected.insert(position) {
                    return Err(Error::InvalidInput);
                }
                let block = remaining
                    .blocks
                    .remove(&position)
                    .ok_or(Error::StaleHandle)?;
                child.contents.blocks.insert(position, block);
                mapping.push((position, position));
            }
            let mass = masses.compute(&child.contents)?;
            let world_com = child.motion.pose.transform(mass.center);
            // Accepted R5 arithmetic: all children sample original_com above;
            // never recompute it after removing an earlier component.
            child.motion.linear = crate::add(
                request.motion.linear,
                crate::cross(request.motion.angular, crate::sub(world_com, original_com)),
            );
            after_destination.push(
                child
                    .contents
                    .blocks
                    .iter()
                    .map(|(p, b)| (*p, b.state.clone()))
                    .collect(),
            );
            // Match the accepted R5 staging order: removed cells no longer
            // count in the retained parent's derived geometry before inserting
            // a child. This avoids a false aggregate-capacity double count.
            let parent = staged
                .entries
                .get_mut(&request.parent.id())
                .ok_or(Error::StaleHandle)?;
            parent.level.contents = remaining.clone();
            parent.level.motion = request.motion.clone();
            let handle = staged.insert(child)?;
            let level = staged.get(handle)?;
            children.push(SplitChild {
                id: level.id,
                plot: level.plot,
                generation: handle.generation,
                mapping,
                motion: level.motion.clone(),
            });
            split_handles.push(handle);
            before_destination.push(destination.state);
        }
        if remaining.blocks.is_empty() {
            return Err(Error::InvalidInput);
        }
        masses.compute(&remaining)?;
        let after_source = remaining
            .blocks
            .iter()
            .map(|(p, b)| (*p, b.state.clone()))
            .collect::<BTreeMap<_, _>>();
        if request.split.retained.as_ref().is_some_and(|retained| {
            *retained != after_source.keys().copied().collect::<BTreeSet<_>>()
        }) {
            return Err(Error::StaleHandle);
        }
        let parent = staged
            .entries
            .get_mut(&request.parent.id())
            .ok_or(Error::StaleHandle)?;
        parent.level.contents = remaining;
        parent.level.motion = request.motion.clone();
        staged.changed(request.parent.id(), crate::ChangeKind::Edited)?;
        staged.validate()?;
        let _ = staged.drain_changes();
        let plot = staged.get(request.parent)?.plot;
        let plan = LifecyclePlan {
            serial,
            epoch: self.epoch,
            operation: Operation::Split,
            handle: request.parent,
            plot,
            mapping: Vec::new(),
            motion: request.motion,
            split_children: children,
            tracker_epoch: Some(request.tracker_epoch),
        };
        self.pending = Some(Pending {
            serial,
            epoch: self.epoch,
            operation: Operation::Split,
            handle: request.parent,
            before_source: request.source,
            before_destination,
            after_source,
            after_destination,
            staged,
            receipt: None,
            unconfirmed: false,
            retired_generation: None,
            split_handles,
        });
        self.next_serial = serial;
        Ok(plan)
    }
    pub fn validate_split(
        &self,
        plan: &LifecyclePlan,
        source: &StateSnapshot,
        destinations: &[StateSnapshot],
        tracker_epoch: u64,
    ) -> Result<()> {
        let pending = self.pending_for(plan)?;
        // The actual host token also validates every owner/generation/content
        // revision. The caller rechecks its tracker epoch at this boundary.
        if pending.operation != Operation::Split
            || pending.receipt.is_some()
            || pending.unconfirmed
            || pending.before_source != *source
            || pending.before_destination != destinations
            || Some(tracker_epoch) != plan.tracker_epoch
        {
            return Err(Error::StaleHandle);
        }
        Ok(())
    }
    pub fn record_split_committed(
        &mut self,
        plan: &LifecyclePlan,
        receipt: BatchReceipt,
    ) -> Result<()> {
        let pending = self.pending_for(plan)?;
        if pending.operation != Operation::Split {
            return Err(Error::InvalidInput);
        }
        let recorded = RecordedReceipt::Batch(receipt);
        if pending.receipt.as_ref().is_some_and(|old| *old != recorded) {
            return Err(Error::StaleHandle);
        }
        let pending = self.pending.as_mut().unwrap();
        pending.receipt = Some(recorded);
        pending.unconfirmed = false;
        Ok(())
    }
    pub fn adopt_split_committed(
        &mut self,
        plan: &LifecyclePlan,
        receipt: BatchReceipt,
        source: &StateSnapshot,
        destinations: &[StateSnapshot],
    ) -> Result<Vec<Handle>> {
        self.record_split_committed(plan, receipt)?;
        self.reconcile_split(plan, source, destinations)
    }
    pub fn reconcile_split(
        &mut self,
        plan: &LifecyclePlan,
        source: &StateSnapshot,
        destinations: &[StateSnapshot],
    ) -> Result<Vec<Handle>> {
        let pending = self.pending_for(plan)?;
        let Some(RecordedReceipt::Batch(receipt)) = pending.receipt.as_ref() else {
            return Err(Error::HostRefused);
        };
        if pending.operation != Operation::Split
            || receipt.barrier_epoch == 0
            || receipt.source_revision <= pending.before_source.revision
            || source.revision < receipt.source_revision
            || source.cells != pending.after_source
            || destinations.len() != pending.before_destination.len()
            || receipt.destination_revisions.len() != destinations.len()
            || destinations.len() != pending.after_destination.len()
            || destinations.iter().enumerate().any(|(index, state)| {
                receipt.destination_revisions[index] <= pending.before_destination[index].revision
                    || state.revision < receipt.destination_revisions[index]
                    || state.cells != pending.after_destination[index]
            })
        {
            return Err(Error::StaleHandle);
        }
        let epoch = self.epoch.checked_add(1).ok_or(Error::Capacity)?;
        let pending = self.pending.take().unwrap();
        self.registry = pending.staged;
        self.host_revisions
            .insert(pending.handle.id(), source.revision);
        for (handle, state) in pending.split_handles.iter().zip(destinations) {
            self.host_revisions.insert(handle.id(), state.revision);
        }
        self.epoch = epoch;
        Ok(pending.split_handles)
    }

    /// Stage retirement only from the COMPLETE current actual owner membership.
    /// The adapter must prove no live member remains, retire the actual binding,
    /// persist its tombstone and obtain a real owned-source save receipt.
    pub fn prepare_empty_retirement(
        &mut self,
        handle: Handle,
        source: StateSnapshot,
        host_generation: u64,
    ) -> Result<LifecyclePlan> {
        let serial = self.available()?;
        let level = self.registry.get(handle)?;
        if !source.cells.is_empty() || host_generation == 0 {
            return Err(Error::InvalidInput);
        }
        let plan = LifecyclePlan {
            serial,
            epoch: self.epoch,
            operation: Operation::RetireEmpty,
            handle,
            plot: level.plot,
            mapping: Vec::new(),
            motion: level.motion.clone(),
            split_children: Vec::new(),
            tracker_epoch: None,
        };
        let mut staged = self.registry.staged();
        let _ = staged.delete(handle)?;
        staged.validate()?;
        let _ = staged.drain_changes();
        self.pending = Some(Pending {
            serial,
            epoch: self.epoch,
            operation: Operation::RetireEmpty,
            handle,
            before_source: source,
            before_destination: Vec::new(),
            after_source: BTreeMap::new(),
            after_destination: Vec::new(),
            staged,
            receipt: None,
            unconfirmed: false,
            retired_generation: Some(host_generation),
            split_handles: Vec::new(),
        });
        self.next_serial = serial;
        Ok(plan)
    }
    pub fn validate_empty_retirement(
        &self,
        plan: &LifecyclePlan,
        source: &StateSnapshot,
        host_generation: u64,
    ) -> Result<()> {
        let pending = self.pending_for(plan)?;
        if pending.operation != Operation::RetireEmpty
            || pending.receipt.is_some()
            || pending.unconfirmed
            || pending.retired_generation != Some(host_generation)
            || pending.before_source != *source
            || !source.cells.is_empty()
        {
            return Err(Error::StaleHandle);
        }
        Ok(())
    }
    pub fn record_retired(
        &mut self,
        plan: &LifecyclePlan,
        receipt: RetirementReceipt,
    ) -> Result<()> {
        let pending = self.pending_for(plan)?;
        if pending.operation != Operation::RetireEmpty {
            return Err(Error::InvalidInput);
        }
        let recorded = RecordedReceipt::Retirement(receipt);
        if pending.receipt.as_ref().is_some_and(|old| *old != recorded) {
            return Err(Error::StaleHandle);
        }
        let pending = self.pending.as_mut().unwrap();
        pending.receipt = Some(recorded);
        pending.unconfirmed = false;
        Ok(())
    }
    pub fn adopt_retired(
        &mut self,
        plan: &LifecyclePlan,
        receipt: RetirementReceipt,
        source: &StateSnapshot,
    ) -> Result<()> {
        self.record_retired(plan, receipt)?;
        self.reconcile_retired(plan, source)
    }
    pub fn reconcile_retired(
        &mut self,
        plan: &LifecyclePlan,
        source: &StateSnapshot,
    ) -> Result<()> {
        let pending = self.pending_for(plan)?;
        let Some(RecordedReceipt::Retirement(receipt)) = pending.receipt.as_ref() else {
            return Err(Error::HostRefused);
        };
        if pending.operation != Operation::RetireEmpty
            || receipt.barrier_epoch == 0
            || pending.retired_generation != Some(receipt.retired_generation)
            || receipt.source_revision < pending.before_source.revision
            || source.revision < receipt.source_revision
            || !source.cells.is_empty()
        {
            return Err(Error::StaleHandle);
        }
        let epoch = self.epoch.checked_add(1).ok_or(Error::Capacity)?;
        let pending = self.pending.take().unwrap();
        self.registry = pending.staged;
        self.host_revisions.remove(&pending.handle.id());
        self.epoch = epoch;
        Ok(())
    }
    fn pending_for(&self, plan: &LifecyclePlan) -> Result<&Pending> {
        self.pending
            .as_ref()
            .filter(|pending| {
                pending.serial == plan.serial
                    && pending.epoch == plan.epoch
                    && pending.epoch == self.epoch
                    && pending.handle == plan.handle
                    && pending.operation == plan.operation
            })
            .ok_or(Error::StaleHandle)
    }
    /// Recheck this inside the actual host's precommit callback after all other
    /// callbacks, while its host prepared token still validates. No mutation.
    pub fn validate_pending(
        &self,
        plan: &LifecyclePlan,
        source: &StateSnapshot,
        destination: &StateSnapshot,
    ) -> Result<()> {
        let pending = self.pending_for(plan)?;
        if pending.receipt.is_some()
            || pending.unconfirmed
            || pending.before_destination.len() != 1
            || pending.before_source != *source
            || pending.before_destination[0] != *destination
        {
            return Err(Error::StaleHandle);
        }
        Ok(())
    }
    /// Cancellation is valid only before the actual host commit. The adapter
    /// must immediately latch successful host completion with record_committed,
    /// record_split_committed or record_retired (or its corresponding adopt
    /// convenience) before any fallible post-state read.
    pub fn cancel(&mut self, plan: &LifecyclePlan) -> Result<()> {
        let pending = self.pending_for(plan)?;
        if pending.receipt.is_some() || pending.unconfirmed {
            return Err(Error::HostRefused);
        }
        let epoch = self.epoch.checked_add(1).ok_or(Error::Capacity)?;
        self.pending = None;
        self.epoch = epoch;
        Ok(())
    }
    /// Record actual host success BEFORE validating readback. If validation
    /// refuses, pending_commit remains visible and all new operations stay
    /// blocked; reconciliation never rewrites or rolls back the host.
    /// Latch a real successful receipt even when post-state reads are unavailable.
    /// Reconciliation is separate and no fabricated snapshot is required.
    pub fn record_committed(&mut self, plan: &LifecyclePlan, receipt: CommitReceipt) -> Result<()> {
        let pending = self.pending_for(plan)?;
        if !matches!(pending.operation, Operation::Capture | Operation::Return) {
            return Err(Error::InvalidInput);
        }
        let receipt = RecordedReceipt::Transfer(receipt);
        if pending.receipt.as_ref().is_some_and(|old| *old != receipt) {
            return Err(Error::StaleHandle);
        }
        let pending = self.pending.as_mut().unwrap();
        pending.receipt = Some(receipt);
        pending.unconfirmed = false;
        Ok(())
    }
    pub fn adopt_committed(
        &mut self,
        plan: &LifecyclePlan,
        receipt: CommitReceipt,
        source: &StateSnapshot,
        destination: &StateSnapshot,
    ) -> Result<Option<Handle>> {
        self.record_committed(plan, receipt)?;
        self.reconcile_committed(plan, source, destination)
    }
    /// Retry the exact recorded successful transfer's actual bounded readback.
    /// The caller must revalidate its current host owner/generation. This does
    /// not bless changed state keys/membership or invent success after an unknown
    /// result. Unrelated monotonic host revisions may advance while state keys
    /// remain identical; readback must never precede the recorded commit.
    pub fn reconcile_committed(
        &mut self,
        plan: &LifecyclePlan,
        source: &StateSnapshot,
        destination: &StateSnapshot,
    ) -> Result<Option<Handle>> {
        let pending = self.pending_for(plan)?;
        let Some(RecordedReceipt::Transfer(receipt)) = pending.receipt.as_ref() else {
            return Err(Error::HostRefused);
        };
        if receipt.source_revision > source.revision
            || receipt.destination_revision > destination.revision
            || receipt.source_revision <= pending.before_source.revision
            || pending.before_destination.len() != 1
            || receipt.destination_revision <= pending.before_destination[0].revision
            || source.cells != pending.after_source
            || destination.cells != pending.after_destination[0]
        {
            return Err(Error::StaleHandle);
        }
        let epoch = self.epoch.checked_add(1).ok_or(Error::Capacity)?;
        let pending = self.pending.take().unwrap();
        self.registry = pending.staged;
        self.epoch = epoch;
        match pending.operation {
            Operation::Capture => {
                self.host_revisions
                    .insert(pending.handle.id(), destination.revision);
                Ok(Some(pending.handle))
            }
            Operation::Return => {
                self.host_revisions.remove(&pending.handle.id());
                Ok(None)
            }
            Operation::RetireEmpty | Operation::Split => {
                unreachable!("separate complete operation receipt required")
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn repeated_host_episodes_do_not_retain_unconsumable_registry_events() {
        let mut policy = HostLifecycle::new(Config::default()).unwrap();
        let masses = MassRegistry::pinned_full_blocks();
        let mut ordinary = StateSnapshot {
            revision: 0,
            cells: BTreeMap::from([([0, 64, 0], "minecraft:blackstone".into())]),
        };
        let mut local = StateSnapshot {
            revision: 0,
            cells: BTreeMap::new(),
        };
        for _ in 0..128 {
            let capture = policy
                .prepare_capture(
                    ordinary.clone(),
                    local.clone(),
                    Id([1; 16]),
                    [0, 64, 0],
                    &[[0, 64, 0]],
                    &masses,
                )
                .unwrap();
            assert!(policy.pending.as_ref().unwrap().staged.changes.is_empty());
            let (from, to) = capture.mapping()[0];
            let state = ordinary.cells.remove(&from).unwrap();
            local.cells.insert(to, state);
            ordinary.revision += 1;
            local.revision += 1;
            let handle = policy
                .adopt_committed(
                    &capture,
                    CommitReceipt {
                        source_revision: ordinary.revision,
                        destination_revision: local.revision,
                    },
                    &ordinary,
                    &local,
                )
                .unwrap()
                .unwrap();
            assert!(policy.registry.changes.is_empty());
            let offset = std::array::from_fn(|axis| from[axis] - to[axis]);
            let returning = policy
                .prepare_return(
                    handle,
                    local.clone(),
                    ordinary.clone(),
                    offset,
                    capture.motion().clone(),
                    &masses,
                )
                .unwrap();
            assert!(policy.pending.as_ref().unwrap().staged.changes.is_empty());
            let state = local.cells.remove(&to).unwrap();
            ordinary.cells.insert(from, state);
            local.revision += 1;
            ordinary.revision += 1;
            policy
                .adopt_committed(
                    &returning,
                    CommitReceipt {
                        source_revision: local.revision,
                        destination_revision: ordinary.revision,
                    },
                    &local,
                    &ordinary,
                )
                .unwrap();
            assert!(policy.registry.changes.is_empty());
            assert_eq!(policy.registry.entries.len(), 0);
            assert_eq!(policy.host_revisions.len(), 0);
        }
    }
}
