//! Host-independent Sable contents and lifecycle. Opaque payloads are preserved,
//! never executed; adapters must supply real ticking and payload codecs.
pub mod gather;
pub mod host_lifecycle;
pub mod lifecycle;
pub mod mass;
pub mod persistence;
#[cfg(feature = "native-physics")]
pub mod physics;
pub mod split;
pub mod transfer;

use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicU64, Ordering};

pub type Pos = [i32; 3];
pub type Result<T> = std::result::Result<T, Error>;
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    InvalidCoordinate,
    InvalidInput,
    Capacity,
    DuplicateId,
    Occupied,
    StaleHandle,
    NotLive,
    MissingDependency,
    CyclicDependency,
    HostRefused,
    CorruptSave,
    UnsupportedVersion,
    Io,
    Interrupted,
}
/// UUID bytes supplied by the host's UUID generator, persisted exactly.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Id(pub [u8; 16]);
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Payload {
    pub format: String,
    pub bytes: Vec<u8>,
}
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Block {
    /// Registry key plus canonical properties; no numeric cross-version IDs.
    pub state: String,
    pub block_entity: Option<Payload>,
    pub extension: Payload,
}
#[derive(Debug, Clone, PartialEq)]
pub struct Entity {
    pub id: Id,
    pub anchor: Pos,
    pub position: [f64; 3],
    pub payload: Payload,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TickKind {
    Block,
    Fluid,
}
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ScheduledTick {
    pub id: Id,
    pub position: Pos,
    pub kind: TickKind,
    pub due: i64,
    pub priority: i32,
    pub payload: Payload,
}
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Contents {
    pub blocks: BTreeMap<Pos, Block>,
    pub entities: BTreeMap<Id, Entity>,
    pub ticks: Vec<ScheduledTick>,
    pub extension: Payload,
}
impl Contents {
    pub fn bounds(&self) -> Option<[i32; 6]> {
        let mut keys = self.blocks.keys();
        let first = *keys.next()?;
        let mut b = [first[0], first[1], first[2], first[0], first[1], first[2]];
        for p in keys {
            for i in 0..3 {
                b[i] = b[i].min(p[i]);
                b[i + 3] = b[i + 3].max(p[i]);
            }
        }
        Some(b)
    }
    /// Derived sparse section membership; blocks remain the sole authority.
    pub fn sections(&self) -> BTreeSet<Pos> {
        self.blocks
            .keys()
            .map(|p| p.map(|v| v.div_euclid(16)))
            .collect()
    }
    pub fn record_count(&self) -> usize {
        self.blocks
            .len()
            .saturating_add(self.entities.len())
            .saturating_add(self.ticks.len())
    }
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub position: [f64; 3],
    pub rotation: [f64; 4],
    pub pivot: [f64; 3],
    pub scale: [f64; 3],
}
impl Default for Pose {
    fn default() -> Self {
        Self {
            position: [0.; 3],
            rotation: [0., 0., 0., 1.],
            pivot: [0.; 3],
            scale: [1.; 3],
        }
    }
}
pub fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] + b[i])
}
pub fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}
pub fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
pub fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    rotate_matrix(q, v, false)
}
// JOML 1.10.5 arithmetic order, captured default useMathFma=false.
// See LICENSE-JOML.txt and the provenance ledger. Squared-norm normalization
// matters even for a quaternion whose norm rounds almost to one.
fn rotate_matrix(q: [f64; 4], v: [f64; 3], inverse: bool) -> [f64; 3] {
    let [x, y, z, w] = q;
    let xx = x * x;
    let yy = y * y;
    let zz = z * z;
    let ww = w * w;
    let xy = x * y;
    let xz = x * z;
    let yz = y * z;
    let xw = x * w;
    let zw = z * w;
    let yw = y * w;
    let k = 1. / (((xx + yy) + zz) + ww);
    let a = ((xx - yy - zz + ww) * k) * v[0];
    let b = ((yy - xx - zz + ww) * k) * v[1];
    let c = ((zz - xx - yy + ww) * k) * v[2];
    if inverse {
        [
            a + ((2. * (xy + zw) * k) * v[1] + (2. * (xz - yw) * k) * v[2]),
            (2. * (xy - zw) * k) * v[0] + (b + (2. * (yz + xw) * k) * v[2]),
            (2. * (xz + yw) * k) * v[0] + ((2. * (yz - xw) * k) * v[1] + c),
        ]
    } else {
        [
            a + ((2. * (xy - zw) * k) * v[1] + (2. * (xz + yw) * k) * v[2]),
            (2. * (xy + zw) * k) * v[0] + (b + (2. * (yz - xw) * k) * v[2]),
            (2. * (xz - yw) * k) * v[0] + ((2. * (yz + xw) * k) * v[1] + c),
        ]
    }
}
impl Pose {
    pub fn transform(&self, p: [f64; 3]) -> [f64; 3] {
        add(
            rotate(
                self.rotation,
                std::array::from_fn(|i| (p[i] - self.pivot[i]) * self.scale[i]),
            ),
            self.position,
        )
    }
    pub fn inverse(&self, p: [f64; 3]) -> [f64; 3] {
        let [x, y, z, w] = self.rotation;
        let inv = 1. / (x * x + (y * y + (z * z + w * w)));
        let v = rotate_matrix(self.rotation.map(|a| a * inv), sub(p, self.position), true);
        std::array::from_fn(|i| v[i] * (1. / self.scale[i]) + self.pivot[i])
    }
    pub fn world_bounds(&self, b: [i32; 6]) -> [f64; 6] {
        let mut out = [
            f64::INFINITY,
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ];
        for mask in 0..8 {
            let p = std::array::from_fn(|i| {
                if mask & (1 << i) == 0 {
                    b[i] as f64
                } else {
                    b[i + 3] as f64 + 1.
                }
            });
            let w = self.transform(p);
            for i in 0..3 {
                out[i] = out[i].min(w[i]);
                out[i + 3] = out[i + 3].max(w[i]);
            }
        }
        out
    }
    pub fn valid(&self) -> bool {
        self.position
            .iter()
            .chain(self.rotation.iter())
            .chain(self.pivot.iter())
            .chain(self.scale.iter())
            .all(|x| x.is_finite())
            && (self.rotation.iter().map(|x| x * x).sum::<f64>() - 1.).abs() < 1e-6
            && self.scale.iter().all(|x| *x > 0.)
    }
}
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Motion {
    pub pose: Pose,
    pub linear: [f64; 3],
    pub angular: [f64; 3],
}
impl Motion {
    /// Serializer pose normalization preserves visible geometry while moving
    /// the saved rotation point to the structure's own center of mass.
    pub fn at_self_com(&self, center: [f64; 3]) -> Self {
        let mut motion = self.clone();
        motion.pose.position = self.pose.transform(center);
        motion.pose.pivot = center;
        motion
    }
}
#[derive(Debug, Clone, PartialEq)]
pub struct Level {
    pub id: Id,
    pub plot: u32,
    pub revision: u64,
    pub contents: Contents,
    pub motion: Motion,
    pub parent: Option<Id>,
    pub dependencies: BTreeSet<Id>,
    pub user_data: Payload,
    pub name: Option<String>,
}
impl Level {
    pub fn new(id: Id) -> Self {
        Self {
            id,
            plot: 0,
            revision: 0,
            contents: Contents::default(),
            motion: Motion::default(),
            parent: None,
            dependencies: BTreeSet::new(),
            user_data: Payload::default(),
            name: None,
        }
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Residency {
    Live,
    Holding,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Handle {
    owner: u64,
    pub(crate) slot: u32,
    generation: u64,
    pub(crate) id: Id,
}
impl Handle {
    pub fn id(self) -> Id {
        self.id
    }
}
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Config {
    pub log_grid: u8,
    pub log_plot: u8,
    pub origin: [i32; 2],
    pub min_y: i32,
    pub max_y: i32,
    pub max_records: usize,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            log_grid: 7,
            log_plot: 7,
            origin: [10000; 2],
            min_y: -64,
            max_y: 320,
            max_records: 1_000_000,
        }
    }
}
impl Config {
    pub fn validate(&self) -> Result<()> {
        if self.log_grid > 7
            || self.log_plot == 0
            || self.log_plot > 7
            || self.min_y >= self.max_y
            || self.min_y < -2048
            || self.max_y > 2048
            || self.max_records == 0
            || self.max_records > 25_600_000
        {
            return Err(Error::InvalidInput);
        }
        for plot in [0, self.slots() - 1] {
            for p in [
                [0, self.min_y, 0],
                [self.side() - 1, self.max_y - 1, self.side() - 1],
            ] {
                self.reserved(plot, p)?;
            }
        }
        Ok(())
    }
    pub fn slots(&self) -> u32 {
        1u32 << (2 * self.log_grid)
    }
    pub fn side(&self) -> i32 {
        16 << self.log_plot
    }
    pub fn local_valid(&self, p: Pos) -> bool {
        p[0] >= 0
            && p[0] < self.side()
            && p[2] >= 0
            && p[2] < self.side()
            && p[1] >= self.min_y
            && p[1] < self.max_y
    }
    pub fn anchor(&self) -> Pos {
        let c = self.side() / 2 + 8;
        [
            c,
            ((i64::from(self.min_y) + i64::from(self.max_y)) / 2) as i32,
            c,
        ]
    }
    pub fn reserved(&self, plot: u32, p: Pos) -> Result<Pos> {
        if plot >= self.slots() || !self.local_valid(p) {
            return Err(Error::InvalidCoordinate);
        }
        let n = 1i64 << self.log_grid;
        let x = (i64::from(self.origin[0]) + i64::from(plot) % n) * i64::from(self.side())
            + i64::from(p[0]);
        let z = (i64::from(self.origin[1]) + i64::from(plot) / n) * i64::from(self.side())
            + i64::from(p[2]);
        if !(-(1 << 25)..(1 << 25)).contains(&x)
            || !(-(1 << 25)..(1 << 25)).contains(&z)
            || !(-(1 << 23)..(1 << 23)).contains(&p[1])
        {
            return Err(Error::InvalidCoordinate);
        }
        Ok([x as i32, p[1], z as i32])
    }
}
#[derive(Debug, Clone)]
pub(crate) struct Entry {
    pub(crate) handle: Handle,
    pub(crate) level: Level,
    pub(crate) residency: Residency,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChangeKind {
    Created,
    Edited,
    Unloaded,
    Loaded,
    Removed,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Change {
    pub id: Id,
    pub revision: u64,
    pub kind: ChangeKind,
}
static NEXT_REGISTRY: AtomicU64 = AtomicU64::new(1);
#[derive(Debug)]
pub struct Registry {
    pub(crate) config: Config,
    pub(crate) entries: BTreeMap<Id, Entry>,
    pub(crate) generations: Vec<u64>,
    owner: u64,
    pub(crate) changes: Vec<Change>,
}
impl Registry {
    pub(crate) fn staged(&self) -> Self {
        Self {
            config: self.config.clone(),
            entries: self.entries.clone(),
            generations: self.generations.clone(),
            owner: self.owner,
            changes: self.changes.clone(),
        }
    }
    pub fn new(config: Config) -> Result<Self> {
        config.validate()?;
        Ok(Self {
            generations: vec![0; config.slots() as usize],
            config,
            entries: BTreeMap::new(),
            owner: NEXT_REGISTRY.fetch_add(1, Ordering::Relaxed),
            changes: Vec::new(),
        })
    }
    pub fn config(&self) -> &Config {
        &self.config
    }
    pub fn levels(&self) -> impl Iterator<Item = (&Level, Residency)> {
        self.entries.values().map(|e| (&e.level, e.residency))
    }
    pub fn handle(&self, id: Id) -> Result<Handle> {
        self.entries
            .get(&id)
            .map(|e| e.handle)
            .ok_or(Error::StaleHandle)
    }
    pub fn get(&self, h: Handle) -> Result<&Level> {
        Ok(&self.entry(h)?.level)
    }
    pub fn residency(&self, h: Handle) -> Result<Residency> {
        Ok(self.entry(h)?.residency)
    }
    pub(crate) fn entry(&self, h: Handle) -> Result<&Entry> {
        self.entries
            .get(&h.id)
            .filter(|e| e.handle == h && h.owner == self.owner)
            .ok_or(Error::StaleHandle)
    }
    fn allocate(&self) -> Result<u32> {
        let n = 1u32 << self.config.log_grid;
        for x in 0..n {
            for z in 0..n {
                let slot = x + z * n;
                if !self.entries.values().any(|e| e.level.plot == slot) {
                    return Ok(slot);
                }
            }
        }
        Err(Error::Capacity)
    }
    pub fn insert(&mut self, mut level: Level) -> Result<Handle> {
        if self.entries.contains_key(&level.id) {
            return Err(Error::DuplicateId);
        }
        level.plot = self.allocate()?;
        level.revision = level.revision.checked_add(1).ok_or(Error::Capacity)?;
        let mut staged = self.staged();
        let h = staged.install(level, Residency::Live)?;
        staged.validate()?;
        *self = staged;
        Ok(h)
    }
    /// Install an already chosen revision. Restoration is not a mutation of
    /// saved content and must not consume revision capacity or publish a
    /// revision different from the state installed below.
    fn install(&mut self, level: Level, residency: Residency) -> Result<Handle> {
        self.validate_level(&level)?;
        if self.entries.values().any(|e| e.level.plot == level.plot) {
            return Err(Error::Occupied);
        }
        let g = self.generations[level.plot as usize]
            .checked_add(1)
            .ok_or(Error::Capacity)?;
        let h = Handle {
            owner: self.owner,
            slot: level.plot,
            generation: g,
            id: level.id,
        };
        self.generations[level.plot as usize] = g;
        self.changes.push(Change {
            id: level.id,
            revision: level.revision,
            kind: ChangeKind::Created,
        });
        self.entries.insert(
            level.id,
            Entry {
                handle: h,
                level,
                residency,
            },
        );
        Ok(h)
    }
    pub fn restore(config: Config, levels: Vec<(Level, Residency)>) -> Result<Self> {
        let mut r = Self::new(config)?;
        for (l, s) in levels {
            if r.entries.contains_key(&l.id) {
                return Err(Error::DuplicateId);
            }
            r.install(l, s)?;
        }
        r.validate()?;
        Ok(r)
    }
    pub fn validate_level(&self, l: &Level) -> Result<()> {
        if l.id.0 == [0; 16]
            || l.plot >= self.config.slots()
            || !l.motion.pose.valid()
            || !l
                .motion
                .linear
                .iter()
                .chain(l.motion.angular.iter())
                .all(|v| v.is_finite())
        {
            return Err(Error::InvalidInput);
        }
        if l.contents.record_count() > self.config.max_records {
            return Err(Error::Capacity);
        }
        if l.contents
            .bounds()
            .is_some_and(|b| l.motion.pose.world_bounds(b).iter().any(|v| !v.is_finite()))
        {
            return Err(Error::InvalidInput);
        }
        for (p, b) in &l.contents.blocks {
            self.config.reserved(l.plot, *p)?;
            if b.state.is_empty()
                || matches!(
                    b.state.as_str(),
                    "minecraft:air" | "minecraft:cave_air" | "minecraft:void_air"
                )
            {
                return Err(Error::InvalidInput);
            }
        }
        let mut ids = BTreeSet::new();
        for (id, e) in &l.contents.entities {
            if id != &e.id
                || id.0 == [0; 16]
                || !ids.insert(*id)
                || !self.config.local_valid(e.anchor)
                || !e.position.iter().all(|v| v.is_finite())
            {
                return Err(Error::InvalidInput);
            }
        }
        for t in &l.contents.ticks {
            if t.id.0 == [0; 16] || !ids.insert(t.id) || !self.config.local_valid(t.position) {
                return Err(Error::InvalidInput);
            }
        }
        Ok(())
    }
    pub fn validate(&self) -> Result<()> {
        let mut records = BTreeSet::new();
        let mut total = 0usize;
        for (l, _) in self.levels() {
            self.validate_level(l)?;
            total = total
                .checked_add(l.contents.record_count())
                .ok_or(Error::Capacity)?;
            if total > self.config.max_records {
                return Err(Error::Capacity);
            }
            for id in l
                .contents
                .entities
                .keys()
                .chain(l.contents.ticks.iter().map(|t| &t.id))
            {
                if !records.insert(*id) {
                    return Err(Error::DuplicateId);
                }
            }
            for dep in &l.dependencies {
                if !self.entries.contains_key(dep) {
                    return Err(Error::MissingDependency);
                }
            }
        }
        // Iterative topological validation bounds work and avoids recursive stack overflow.
        let mut done = BTreeSet::new();
        while done.len() < self.entries.len() {
            let before = done.len();
            for (id, e) in &self.entries {
                if e.level.dependencies.iter().all(|d| done.contains(d)) {
                    done.insert(*id);
                }
            }
            if before == done.len() {
                return Err(Error::CyclicDependency);
            }
        }
        let mut ancestry_done = BTreeSet::new();
        for id in self.entries.keys() {
            let mut path = BTreeSet::new();
            let mut current = Some(*id);
            while let Some(id) = current {
                if ancestry_done.contains(&id) {
                    break;
                }
                if !path.insert(id) {
                    return Err(Error::CyclicDependency);
                }
                current = self.entries.get(&id).and_then(|e| e.level.parent);
            }
            ancestry_done.extend(path);
        }
        Ok(())
    }
    pub(crate) fn changed(&mut self, id: Id, kind: ChangeKind) -> Result<()> {
        let e = self.entries.get_mut(&id).ok_or(Error::StaleHandle)?;
        e.level.revision = e.level.revision.checked_add(1).ok_or(Error::Capacity)?;
        self.changes.push(Change {
            id,
            revision: e.level.revision,
            kind,
        });
        Ok(())
    }
    pub fn edit(&mut self, h: Handle, p: Pos, block: Option<Block>) -> Result<Option<Block>> {
        if self.residency(h)? != Residency::Live {
            return Err(Error::NotLive);
        }
        self.config.reserved(h.slot, p)?;
        let mut staged = self.staged();
        let l = &mut staged.entries.get_mut(&h.id).unwrap().level;
        let old = if let Some(b) = block {
            l.contents.blocks.insert(p, b)
        } else {
            l.contents.blocks.remove(&p)
        };
        staged.validate()?;
        staged.changed(h.id, ChangeKind::Edited)?;
        *self = staged;
        Ok(old)
    }
    /// Unload the connected dependency group at one publication boundary.
    /// Holding entries retain UUID, contents and plot reservation.
    pub fn unload(&mut self, h: Handle) -> Result<()> {
        self.entry(h)?;
        self.validate()?;
        let mut adjacent: BTreeMap<Id, BTreeSet<Id>> = BTreeMap::new();
        for (id, e) in &self.entries {
            for dep in &e.level.dependencies {
                adjacent.entry(*id).or_default().insert(*dep);
                adjacent.entry(*dep).or_default().insert(*id);
            }
        }
        let mut group = BTreeSet::from([h.id]);
        let mut queue = vec![h.id];
        while let Some(id) = queue.pop() {
            if let Some(neighbors) = adjacent.get(&id) {
                for next in neighbors {
                    if group.insert(*next) {
                        queue.push(*next);
                    }
                }
            }
        }
        let mut staged = self.staged();
        for id in group {
            if staged.entries[&id].residency == Residency::Live {
                staged.changed(id, ChangeKind::Unloaded)?;
                staged.entries.get_mut(&id).unwrap().residency = Residency::Holding;
            }
        }
        *self = staged;
        Ok(())
    }
    pub(crate) fn make_live(&mut self, h: Handle) -> Result<()> {
        self.entry(h)?;
        self.changed(h.id, ChangeKind::Loaded)?;
        self.entries.get_mut(&h.id).unwrap().residency = Residency::Live;
        Ok(())
    }
    /// Permanent deletion returns sole ownership of removed contents to the caller.
    pub fn delete(&mut self, h: Handle) -> Result<Level> {
        self.entry(h)?;
        if self
            .entries
            .values()
            .any(|e| e.level.dependencies.contains(&h.id))
        {
            return Err(Error::MissingDependency);
        }
        let e = self.entries.remove(&h.id).unwrap();
        self.changes.push(Change {
            id: h.id,
            revision: e.level.revision,
            kind: ChangeKind::Removed,
        });
        Ok(e.level)
    }
    pub fn drain_changes(&mut self) -> Vec<Change> {
        std::mem::take(&mut self.changes)
    }
}
