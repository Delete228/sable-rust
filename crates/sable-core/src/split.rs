//! Incremental, 18-neighbour heat propagation derived from pinned Java Sable.
//!
//! Original: `SubLevelHeatMapManager.java`, Sable
//! `9b1a3d1555d35db9fb7b1103701e7490f0a968b9`. See the crate provenance ledger.
//! The owned block store is supplied by the caller; heat is derived metadata.
//! Equal-size selection uses coordinate order, rather than Fastutil table order.
//! A returned plan is a transaction boundary: apply it before advancing again,
//! then reconcile heat from the sole authoritative retained block store.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use crate::Pos;

/// The order in the original heat manager, distinct from gather loop order.
const DIRECTIONS: [Pos; 18] = [
    [1, 0, 0],
    [-1, 0, 0],
    [0, 1, 0],
    [0, -1, 0],
    [0, 0, 1],
    [0, 0, -1],
    [1, 1, 0],
    [-1, -1, 0],
    [1, -1, 0],
    [-1, 1, 0],
    [1, 0, 1],
    [-1, 0, -1],
    [1, 0, -1],
    [-1, 0, 1],
    [0, 1, 1],
    [0, -1, -1],
    [0, -1, 1],
    [0, 1, -1],
];

/// Java's default counts heat steps, not blocks visited or components found.
pub const DEFAULT_HEAT_STEPS: usize = 200;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HeatState {
    Filling,
    Clearing,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SplitPlan {
    /// Tracker-local edit epoch. The registry also validates its own generation.
    pub source_epoch: u64,
    pub children: Vec<BTreeSet<Pos>>,
    /// Set only when all components were candidates and one was kept as parent.
    pub retained: Option<BTreeSet<Pos>>,
}

/// Diagnostic view for bounded original-source comparison, not saved authority.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct HeatSnapshot {
    pub state: HeatState,
    pub initialized: bool,
    pub split_complete: bool,
    pub solid_count: i64,
    pub heat: BTreeMap<Pos, i16>,
    pub floodfill: Vec<Pos>,
    pub removed: Vec<Pos>,
    pub new_starts: Vec<Pos>,
    pub split_indices: BTreeMap<Pos, usize>,
    pub index_map: Vec<usize>,
    pub steps_executed: u64,
}

#[derive(Clone, Debug)]
pub struct SplitTracker {
    heat: BTreeMap<Pos, i16>,
    splits: BTreeMap<Pos, usize>,
    floodfill: VecDeque<Pos>,
    removed: Vec<Pos>,
    new_starts: Vec<Pos>,
    index_map: Vec<usize>,
    state: HeatState,
    initialized: bool,
    split_complete: bool,
    solid_count: i64,
    epoch: u64,
    steps_executed: u64,
    awaiting_commit: bool,
}

impl Default for SplitTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl SplitTracker {
    pub fn new() -> Self {
        Self {
            heat: BTreeMap::new(),
            splits: BTreeMap::new(),
            floodfill: VecDeque::new(),
            removed: Vec::new(),
            new_starts: Vec::new(),
            index_map: Vec::new(),
            state: HeatState::Filling,
            initialized: false,
            split_complete: false,
            solid_count: 0,
            epoch: 0,
            steps_executed: 0,
            awaiting_commit: false,
        }
    }

    pub fn epoch(&self) -> u64 {
        self.epoch
    }

    /// Call once for an air-to-nonair transition after the authoritative edit.
    pub fn on_added(&mut self, pos: Pos) {
        self.epoch = self.epoch.wrapping_add(1);
        self.add(pos);
    }

    fn add(&mut self, pos: Pos) {
        self.solid_count += 1;
        if !self.initialized {
            self.initialized = true;
            self.heat.insert(pos, 1);
            self.floodfill.push_back(pos);
            self.index_map.push(0);
            return;
        }

        if let Some(i) = self.removed.iter().position(|p| *p == pos) {
            self.removed.remove(i);
            return;
        }

        let min = neighbours(pos)
            .filter_map(|p| self.heat.get(&p).copied())
            .min();
        if let Some(heat) = min {
            self.set_heat(pos, heat.wrapping_add(1));
            if self.state == HeatState::Filling {
                self.floodfill.push_back(pos);
            } else {
                self.new_starts.push(pos);
            }
        } else if !self.split_complete {
            // Original source does not register thin-air placements after the
            // first completed flood-fill. Preserve that observable decision.
            self.splits.insert(pos, 0);
        }
    }

    /// Call once for a nonair-to-air transition after the authoritative edit.
    pub fn on_removed(&mut self, pos: Pos) {
        self.epoch = self.epoch.wrapping_add(1);
        self.solid_count -= 1;
        self.removed.push(pos);
    }

    /// Advances exactly the source heat-step budget until idle or a transfer
    /// plan. A source step can process all queued removals; it is not a CPU-time
    /// bound. Disabled splitting pauses state without consuming work.
    pub fn tick(&mut self, solids: &BTreeSet<Pos>, budget: usize, enabled: bool) -> Vec<SplitPlan> {
        if !enabled || self.awaiting_commit {
            return Vec::new();
        }
        for _ in 0..budget {
            self.steps_executed += 1;
            let (idle, plan) = self.step(solids);
            if let Some(plan) = plan {
                self.awaiting_commit = true;
                return vec![plan];
            }
            if idle {
                break;
            }
        }
        Vec::new()
    }

    /// Rebuild derived work after a committed or refused plan, load, or explicit
    /// cancellation. Runtime queues never survive identity/plot reuse. The
    /// ordering policy is documented separately from Java hash iteration.
    pub fn reconcile(&mut self, solids: &BTreeSet<Pos>) {
        let epoch = self.epoch.wrapping_add(1);
        let steps = self.steps_executed;
        *self = Self::new();
        self.epoch = epoch;
        self.steps_executed = steps;
        for &pos in solids {
            self.add(pos);
        }
    }

    pub fn snapshot(&self) -> HeatSnapshot {
        HeatSnapshot {
            state: self.state,
            initialized: self.initialized,
            split_complete: self.split_complete,
            solid_count: self.solid_count,
            heat: self.heat.clone(),
            floodfill: self.floodfill.iter().copied().collect(),
            removed: self.removed.clone(),
            new_starts: self.new_starts.clone(),
            split_indices: self.splits.clone(),
            index_map: self.index_map.clone(),
            steps_executed: self.steps_executed,
        }
    }

    fn step(&mut self, solids: &BTreeSet<Pos>) -> (bool, Option<SplitPlan>) {
        if self.state == HeatState::Filling {
            if let Some(pos) = self.floodfill.pop_front()
                && let Some(&heat) = self.heat.get(&pos)
            {
                for neighbour in neighbours(pos) {
                    if solids.contains(&neighbour) && !self.heat.contains_key(&neighbour) {
                        self.set_heat(neighbour, heat.wrapping_add(1));
                        self.splits.remove(&neighbour);
                        self.floodfill.push_back(neighbour);
                    }
                }
            }
            if self.floodfill.is_empty() {
                self.split_complete = true;
                self.state = HeatState::Clearing;
                if !self.splits.is_empty()
                    && let Some(plan) = self.split()
                {
                    return (false, Some(plan));
                }
            }
        }

        // Intentionally a second `if`: the transition above can perform a
        // CLEARING action during the same original step.
        if self.state == HeatState::Clearing {
            if let Some(pos) = self.floodfill.pop_front() {
                if let Some(&heat) = self.heat.get(&pos) {
                    let current = self.index_map[self.splits.get(&pos).copied().unwrap_or(0)];
                    for neighbour in neighbours(pos) {
                        if solids.contains(&neighbour) {
                            if let Some(&other) = self.splits.get(&neighbour)
                                && current != self.index_map[other]
                            {
                                self.index_map[other] = current;
                            }
                            if let Some(&other_heat) = self.heat.get(&neighbour) {
                                if other_heat > heat {
                                    self.floodfill.push_back(neighbour);
                                    self.splits.insert(neighbour, current);
                                } else {
                                    self.new_starts.push(neighbour);
                                }
                            }
                        }
                    }
                    self.heat.remove(&pos);
                }
            } else if !self.removed.is_empty() {
                // Java treats the entire removal list as one step and removes
                // heat in list order, which matters for concurrent edits.
                let removed = std::mem::take(&mut self.removed);
                for pos in removed {
                    if let Some(&heat) = self.heat.get(&pos) {
                        for (direction, neighbour) in offsets(pos) {
                            if solids.contains(&neighbour)
                                && self.heat.get(&neighbour).is_some_and(|h| *h > heat)
                            {
                                let neighbour_heat = self.heat[&neighbour];
                                let can_remove = offsets(neighbour).all(|(other_dir, next)| {
                                    other_dir == direction.map(|x| -x)
                                        || !(solids.contains(&next)
                                            && self
                                                .heat
                                                .get(&next)
                                                .is_some_and(|h| *h < neighbour_heat))
                                });
                                if can_remove {
                                    self.floodfill.push_back(neighbour);
                                    let new_index = self.index_map.len();
                                    self.splits.insert(neighbour, new_index);
                                    self.index_map.push(new_index);
                                }
                            }
                        }
                    }
                    self.heat.remove(&pos);
                }
            } else if !self.new_starts.is_empty() {
                self.floodfill.extend(std::mem::take(&mut self.new_starts));
                self.state = HeatState::Filling;
            } else if !self.splits.is_empty() {
                self.split_complete = true;
                return (false, self.split());
            } else {
                self.split_complete = true;
                return (true, None);
            }
        }
        (false, None)
    }

    fn split(&mut self) -> Option<SplitPlan> {
        let mut groups: BTreeMap<usize, BTreeSet<Pos>> = BTreeMap::new();
        for (&pos, &partial) in &self.splits {
            groups
                .entry(self.index_map[partial])
                .or_default()
                .insert(pos);
        }

        let whole = groups.len() == 1
            && groups
                .values()
                .next()
                .is_some_and(|v| v.len() as i64 == self.solid_count);
        let mut retained = None;
        if whole {
            let blocks = groups.values().next().expect("one component");
            self.rebuild_heat(blocks);
            groups.clear();
        }

        let total: usize = groups.values().map(BTreeSet::len).sum();
        if !whole && total != 0 && total as i64 == self.solid_count {
            // Java descending stable sort keeps the first Fastutil entry on
            // ties. Here the smallest coordinate of equal-largest groups wins.
            let key = groups
                .iter()
                .min_by(|(_, a), (_, b)| {
                    b.len()
                        .cmp(&a.len())
                        .then_with(|| a.first().cmp(&b.first()))
                })
                .map(|(&key, _)| key)
                .expect("nonempty candidate groups");
            let keep = groups.remove(&key).expect("selected group");
            self.rebuild_heat(&keep);
            retained = Some(keep);
        }

        self.splits.clear();
        self.index_map.clear();
        self.index_map.push(0);

        if groups.is_empty() {
            None
        } else {
            Some(SplitPlan {
                source_epoch: self.epoch,
                children: groups.into_values().collect(),
                retained,
            })
        }
    }

    fn rebuild_heat(&mut self, blocks: &BTreeSet<Pos>) {
        self.state = HeatState::Filling;
        self.initialized = false;
        self.split_complete = false;
        self.solid_count = 0;
        self.new_starts.clear();
        self.floodfill.clear();
        self.removed.clear();
        for pos in blocks {
            self.heat.remove(pos);
        }
        for &pos in blocks {
            self.add(pos);
        }
    }

    fn set_heat(&mut self, pos: Pos, heat: i16) {
        if heat == 0 {
            self.heat.remove(&pos);
        } else {
            self.heat.insert(pos, heat);
        }
    }
}

fn offsets(pos: Pos) -> impl Iterator<Item = (Pos, Pos)> {
    DIRECTIONS.into_iter().filter_map(move |direction| {
        Some((
            direction,
            [
                pos[0].checked_add(direction[0])?,
                pos[1].checked_add(direction[1])?,
                pos[2].checked_add(direction[2])?,
            ],
        ))
    })
}

fn neighbours(pos: Pos) -> impl Iterator<Item = Pos> {
    offsets(pos).map(|(_, neighbour)| neighbour)
}
