//! Transaction boundaries joining the sole owned contents with gather/split.
use crate::{
    mass::MassRegistry,
    split::SplitPlan,
    transfer::{self, TransferEvent, TransferHost},
    *,
};

struct SplitWork {
    handle: Handle,
    revision: u64,
    awaiting: bool,
    solids: BTreeSet<Pos>,
    tracker: split::SplitTracker,
}
/// Queues are derived runtime work. Save/load and identity reuse restart them.
#[derive(Default)]
pub struct SplitService {
    work: BTreeMap<Id, SplitWork>,
}
#[derive(Debug, Clone)]
pub struct PendingSplit {
    pub handle: Handle,
    pub revision: u64,
    pub plan: SplitPlan,
}
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SplitOutcome {
    pub children: Vec<Handle>,
    pub events: Vec<(Handle, Vec<TransferEvent>)>,
}
impl SplitService {
    /// Call at every edit publication boundary. Topology edits arriving during
    /// propagation enter the original add/remove queues rather than instant BFS.
    pub fn observe(&mut self, r: &Registry, h: Handle, masses: &MassRegistry) -> Result<()> {
        let l = r.get(h)?;
        if r.residency(h)? != Residency::Live {
            self.work.remove(&h.id);
            return Ok(());
        }
        let solids = l
            .contents
            .blocks
            .iter()
            .map(|(p, b)| masses.get(&b.state).map(|_| *p))
            .collect::<Result<BTreeSet<_>>>()?;
        let w = self.work.entry(h.id).or_insert_with(|| SplitWork {
            handle: h,
            revision: l.revision,
            awaiting: false,
            solids: BTreeSet::new(),
            tracker: split::SplitTracker::new(),
        });
        if w.handle != h {
            *w = SplitWork {
                handle: h,
                revision: l.revision,
                awaiting: false,
                solids: BTreeSet::new(),
                tracker: split::SplitTracker::new(),
            };
        }
        if w.awaiting && w.revision != l.revision {
            w.tracker.reconcile(&solids);
            w.solids = solids;
            w.revision = l.revision;
            w.awaiting = false;
            return Ok(());
        }
        for p in w.solids.difference(&solids) {
            w.tracker.on_removed(*p)
        }
        for p in solids.difference(&w.solids) {
            w.tracker.on_added(*p)
        }
        w.solids = solids;
        w.revision = l.revision;
        Ok(())
    }
    pub fn tick(
        &mut self,
        r: &Registry,
        h: Handle,
        masses: &MassRegistry,
        budget: usize,
        enabled: bool,
    ) -> Result<Option<PendingSplit>> {
        self.observe(r, h, masses)?;
        let revision = r.get(h)?.revision;
        Ok(self
            .work
            .get_mut(&h.id)
            .and_then(|w| {
                let plan = w
                    .tracker
                    .tick(&w.solids, budget, enabled)
                    .into_iter()
                    .next();
                if plan.is_some() {
                    w.awaiting = true;
                }
                plan
            })
            .map(|plan| PendingSplit {
                handle: h,
                revision,
                plan,
            }))
    }
    pub fn cancel(&mut self, id: Id) {
        self.work.remove(&id);
    }
    pub fn commit(
        &mut self,
        r: &mut Registry,
        pending: PendingSplit,
        ids: &[Id],
        host: &mut impl TransferHost,
        masses: &MassRegistry,
    ) -> Result<SplitOutcome> {
        let w = self
            .work
            .get(&pending.handle.id)
            .ok_or(Error::StaleHandle)?;
        if w.handle != pending.handle {
            return Err(Error::StaleHandle);
        }
        if w.tracker.epoch() != pending.plan.source_epoch {
            self.cancel(pending.handle.id);
            return Err(Error::StaleHandle);
        }
        let result = r.apply_split(
            pending.handle,
            pending.revision,
            &pending.plan,
            ids,
            host,
            masses,
        );
        self.cancel(pending.handle.id);
        result
    }
}
impl Registry {
    pub fn transfer(
        &mut self,
        source: Handle,
        target: Handle,
        positions: &[Pos],
        offset: Pos,
        host: &mut impl TransferHost,
    ) -> Result<Vec<TransferEvent>> {
        if source == target {
            return Err(Error::InvalidInput);
        }
        if self.residency(source)? != Residency::Live || self.residency(target)? != Residency::Live
        {
            return Err(Error::NotLive);
        }
        let mut from = self.get(source)?.contents.clone();
        let mut to = self.get(target)?.contents.clone();
        let plan = transfer::plan(&from, &to, positions, offset, self.config.max_records)?;
        let events = transfer::move_contents(&mut from, &mut to, &plan, host)?;
        let mut next = self.staged();
        next.entries.get_mut(&source.id).unwrap().level.contents = from;
        next.entries.get_mut(&target.id).unwrap().level.contents = to;
        next.changed(source.id, ChangeKind::Edited)?;
        next.changed(target.id, ChangeKind::Edited)?;
        next.validate()?;
        *self = next;
        Ok(events)
    }
    /// Assembly is translation-only. Rotation/state transforms require a host
    /// codec and are deliberately outside this supported path.
    pub fn assemble(
        &mut self,
        world: &mut Contents,
        id: Id,
        anchor: Pos,
        positions: &[Pos],
        host: &mut impl TransferHost,
        masses: &MassRegistry,
    ) -> Result<(Handle, Vec<TransferEvent>)> {
        if !gather::in_domain(anchor) || positions.iter().any(|p| !gather::in_domain(*p)) {
            return Err(Error::InvalidCoordinate);
        }
        let dst = self.config.anchor();
        let offset: Pos = std::array::from_fn(|i| dst[i] - anchor[i]);
        let mut from = world.clone();
        let mut level = Level::new(id);
        let plan = transfer::plan(
            &from,
            &level.contents,
            positions,
            offset,
            self.config.max_records,
        )?;
        let events = transfer::move_contents(&mut from, &mut level.contents, &plan, host)?;
        let m = masses.compute(&level.contents)?;
        level.motion.pose.pivot = m.center;
        level.motion.pose.position = std::array::from_fn(|i| m.center[i] - f64::from(offset[i]));
        let mut next = self.staged();
        let h = next.insert(level)?;
        *self = next;
        *world = from;
        Ok((h, events))
    }
    /// Apply an incremental split result against the exact runtime generation
    /// and edit revision that produced it. Parent keeps its identity and pose.
    /// Every new child samples the same pre-transaction parent COM and motion.
    /// This atomic-batch policy deliberately differs from sequential Java
    /// assembly, whose immediate COM updates can make later children depend on
    /// iteration order. The retained parent's velocity stays unchanged. See
    /// docs/phase-2-r5-motion-policy.md; full game split parity is not claimed.
    pub fn apply_split(
        &mut self,
        parent: Handle,
        revision: u64,
        plan: &SplitPlan,
        ids: &[Id],
        host: &mut impl TransferHost,
        masses: &MassRegistry,
    ) -> Result<SplitOutcome> {
        let original = self.get(parent)?;
        if self.residency(parent)? != Residency::Live {
            return Err(Error::NotLive);
        }
        if revision != original.revision {
            return Err(Error::StaleHandle);
        }
        if ids.len() != plan.children.len() {
            return Err(Error::InvalidInput);
        }
        let original_mass = masses.compute(&original.contents)?;
        let old_com = original.motion.pose.transform(original_mass.center);
        let mut selected = BTreeSet::new();
        for component in &plan.children {
            if component.is_empty() {
                return Err(Error::InvalidInput);
            }
            for p in component {
                if !original.contents.blocks.contains_key(p) || !selected.insert(*p) {
                    return Err(Error::InvalidInput);
                }
            }
        }
        let mut next = self.staged();
        let mut from = original.contents.clone();
        let mut children = Vec::new();
        let mut events = Vec::new();
        for (component, id) in plan.children.iter().zip(ids) {
            let mut child = Level::new(*id);
            child.parent = Some(original.id);
            child.motion = original.motion.clone();
            let positions: Vec<_> = component.iter().copied().collect();
            let p = transfer::plan(
                &from,
                &child.contents,
                &positions,
                [0; 3],
                self.config.max_records,
            )?;
            let child_events = transfer::move_contents(&mut from, &mut child.contents, &p, host)?;
            // A massless component remains owned; a caller must explicitly
            // destroy it rather than silently discard inventories as Java can.
            let mass = masses.compute(&child.contents)?;
            let world_com = child.motion.pose.transform(mass.center);
            child.motion.linear = add(
                original.motion.linear,
                cross(original.motion.angular, sub(world_com, old_com)),
            );
            next.entries.get_mut(&parent.id).unwrap().level.contents = from.clone();
            let handle = next.insert(child)?;
            children.push(handle);
            events.push((handle, child_events));
        }
        if from.blocks.is_empty() {
            return Err(Error::InvalidInput);
        }
        masses.compute(&from)?;
        let level = &mut next.entries.get_mut(&parent.id).unwrap().level;
        level.contents = from;
        next.changed(parent.id, ChangeKind::Edited)?;
        next.validate()?;
        *self = next;
        Ok(SplitOutcome { children, events })
    }
}
