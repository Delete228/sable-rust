//! Versioned Rust saves. This is not Java NBT/file compatibility.
//! Snapshot construction requires the caller's completed host/physics epoch.

use crate::{
    Block, Config, Contents, Entity, Error, Handle, Id, Level, Motion, Payload, Pos, Pose,
    Registry, Residency, Result, ScheduledTick, TickKind,
};
use std::collections::{BTreeMap, BTreeSet};
use std::fs::{self, File, OpenOptions};
use std::io::{Read, Write};
#[cfg(unix)]
use std::os::unix::fs::OpenOptionsExt;
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};

const MAGIC: &[u8; 8] = b"SABLEP2\0";
const VERSION: u32 = 1;

#[derive(Clone, Copy, Debug)]
pub struct DecodeLimits {
    pub max_bytes: usize,
    pub max_records: usize,
    pub max_levels: usize,
    pub max_blob: usize,
}

impl Default for DecodeLimits {
    fn default() -> Self {
        Self {
            max_bytes: 64 * 1024 * 1024,
            max_records: 1_000_000,
            max_levels: 16_384,
            max_blob: 16 * 1024 * 1024,
        }
    }
}

pub type SavedVelocities = (Option<[f64; 3]>, Option<[f64; 3]>);

/// Original SubLevelSerializer optional velocity fields, with JOML 1.10.5's
/// default `joml.useMathFma=false`. Its lengthSquared evaluates x*x+(y*y+z*z),
/// not a left-associated sum. This helper does not produce a Java NBT file.
pub fn java_saved_velocities(linear: [f64; 3], angular: [f64; 3]) -> Result<SavedVelocities> {
    if !linear.iter().chain(angular.iter()).all(|v| v.is_finite()) {
        return Err(Error::InvalidInput);
    }
    let length_squared = |v: [f64; 3]| v[0] * v[0] + (v[1] * v[1] + v[2] * v[2]);
    let saved_linear = (length_squared(linear) > 0.1 * 0.1).then_some(linear);
    // Preserve the source's dimensionally questionable angular comparison.
    let saved_angular = (length_squared(angular) > 1.0_f64.to_radians()).then_some(angular);
    Ok((saved_linear, saved_angular))
}

/// Exact Rust saves retain all motion. Java-compatible *field policy* preserves
/// the source's asymmetric thresholds, including squared angular magnitude
/// compared directly to radians(1). It does not write Java's NBT format.
#[derive(Clone, Copy, Debug)]
pub enum VelocityPolicy {
    Exact,
    JavaFields { retained_on_load: f64 },
}

impl VelocityPolicy {
    pub fn apply(self, motion: &mut Motion) -> Result<()> {
        if !motion
            .linear
            .iter()
            .chain(motion.angular.iter())
            .all(|v| v.is_finite())
        {
            return Err(Error::InvalidInput);
        }
        let Self::JavaFields { retained_on_load } = self else {
            return Ok(());
        };
        if !retained_on_load.is_finite() || !(0.0..=1.0).contains(&retained_on_load) {
            return Err(Error::InvalidInput);
        }
        let (linear, angular) = java_saved_velocities(motion.linear, motion.angular)?;
        motion.linear = linear.map_or([0.0; 3], |v| v.map(|n| n * retained_on_load));
        motion.angular = angular.map_or([0.0; 3], |v| v.map(|n| n * retained_on_load));
        Ok(())
    }
}

fn hash(bytes: &[u8]) -> u64 {
    // FNV-1a detects accidental damage; it is not an authenticity mechanism.
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |h, b| {
        (h ^ u64::from(*b)).wrapping_mul(0x100_0000_01b3)
    })
}

struct Writer {
    bytes: Vec<u8>,
}
impl Writer {
    fn u8(&mut self, v: u8) {
        self.bytes.push(v);
    }
    fn u32(&mut self, v: u32) {
        self.bytes.extend(v.to_le_bytes());
    }
    fn i32(&mut self, v: i32) {
        self.bytes.extend(v.to_le_bytes());
    }
    fn u64(&mut self, v: u64) {
        self.bytes.extend(v.to_le_bytes());
    }
    fn i64(&mut self, v: i64) {
        self.bytes.extend(v.to_le_bytes());
    }
    fn f64(&mut self, v: f64) {
        self.u64(v.to_bits());
    }
    fn len(&mut self, n: usize) -> Result<()> {
        self.u32(u32::try_from(n).map_err(|_| Error::Capacity)?);
        Ok(())
    }
    fn blob(&mut self, v: &[u8]) -> Result<()> {
        self.len(v.len())?;
        self.bytes.extend(v);
        Ok(())
    }
    fn string(&mut self, v: &str) -> Result<()> {
        self.blob(v.as_bytes())
    }
    fn id(&mut self, v: Id) {
        self.bytes.extend(v.0);
    }
    fn pos(&mut self, v: Pos) {
        for n in v {
            self.i32(n);
        }
    }
    fn vector(&mut self, v: [f64; 3]) {
        for n in v {
            self.f64(n);
        }
    }
    fn payload(&mut self, v: &Payload) -> Result<()> {
        self.string(&v.format)?;
        self.blob(&v.bytes)
    }
    fn optional_payload(&mut self, v: &Option<Payload>) -> Result<()> {
        self.u8(u8::from(v.is_some()));
        if let Some(p) = v {
            self.payload(p)?;
        }
        Ok(())
    }
    fn motion(&mut self, v: &Motion) {
        self.vector(v.pose.position);
        for n in v.pose.rotation {
            self.f64(n);
        }
        self.vector(v.pose.pivot);
        self.vector(v.pose.scale);
        self.vector(v.linear);
        self.vector(v.angular);
    }
    fn contents(&mut self, v: &Contents) -> Result<()> {
        self.len(v.blocks.len())?;
        for (&position, block) in &v.blocks {
            self.pos(position);
            self.string(&block.state)?;
            self.optional_payload(&block.block_entity)?;
            self.payload(&block.extension)?;
        }
        self.len(v.entities.len())?;
        for entity in v.entities.values() {
            self.id(entity.id);
            self.pos(entity.anchor);
            self.vector(entity.position);
            self.payload(&entity.payload)?;
        }
        self.len(v.ticks.len())?;
        for tick in &v.ticks {
            self.id(tick.id);
            self.pos(tick.position);
            self.u8(match tick.kind {
                TickKind::Block => 0,
                TickKind::Fluid => 1,
            });
            self.i64(tick.due);
            self.i32(tick.priority);
            self.payload(&tick.payload)?;
        }
        self.payload(&v.extension)
    }
    fn level(&mut self, v: &Level, residency: Residency) -> Result<()> {
        self.id(v.id);
        self.u32(v.plot);
        self.u64(v.revision);
        self.u8(match residency {
            Residency::Live => 0,
            Residency::Holding => 1,
        });
        self.u8(u8::from(v.parent.is_some()));
        if let Some(id) = v.parent {
            self.id(id);
        }
        self.len(v.dependencies.len())?;
        for id in &v.dependencies {
            self.id(*id);
        }
        self.u8(u8::from(v.name.is_some()));
        if let Some(name) = &v.name {
            self.string(name)?;
        }
        self.motion(&v.motion);
        self.contents(&v.contents)?;
        self.payload(&v.user_data)
    }
}

/// This consumes a coherent immutable registry snapshot, including holding
/// entries. Removed entries are absent, so a subsequently committed save cannot
/// resurrect deleted contents.
pub fn encode(registry: &Registry) -> Result<Vec<u8>> {
    registry.validate()?;
    let mut w = Writer { bytes: Vec::new() };
    w.bytes.extend(MAGIC);
    w.u32(VERSION);
    let config = registry.config();
    w.u8(config.log_grid);
    w.u8(config.log_plot);
    w.i32(config.origin[0]);
    w.i32(config.origin[1]);
    w.i32(config.min_y);
    w.i32(config.max_y);
    w.u64(u64::try_from(config.max_records).map_err(|_| Error::Capacity)?);
    let levels: Vec<_> = registry.levels().collect();
    w.len(levels.len())?;
    for (level, residency) in levels {
        w.level(level, residency)?;
    }
    let checksum = hash(&w.bytes);
    w.u64(checksum);
    Ok(w.bytes)
}

struct Reader<'a> {
    bytes: &'a [u8],
    cursor: usize,
    limits: DecodeLimits,
    records_left: usize,
}
impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Result<&'a [u8]> {
        let end = self.cursor.checked_add(n).ok_or(Error::CorruptSave)?;
        let result = self.bytes.get(self.cursor..end).ok_or(Error::CorruptSave)?;
        self.cursor = end;
        Ok(result)
    }
    fn u8(&mut self) -> Result<u8> {
        Ok(self.take(1)?[0])
    }
    fn flag(&mut self) -> Result<bool> {
        match self.u8()? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(Error::CorruptSave),
        }
    }
    fn u32(&mut self) -> Result<u32> {
        Ok(u32::from_le_bytes(
            self.take(4)?.try_into().map_err(|_| Error::CorruptSave)?,
        ))
    }
    fn i32(&mut self) -> Result<i32> {
        Ok(i32::from_le_bytes(
            self.take(4)?.try_into().map_err(|_| Error::CorruptSave)?,
        ))
    }
    fn u64(&mut self) -> Result<u64> {
        Ok(u64::from_le_bytes(
            self.take(8)?.try_into().map_err(|_| Error::CorruptSave)?,
        ))
    }
    fn i64(&mut self) -> Result<i64> {
        Ok(i64::from_le_bytes(
            self.take(8)?.try_into().map_err(|_| Error::CorruptSave)?,
        ))
    }
    fn f64(&mut self) -> Result<f64> {
        let v = f64::from_bits(self.u64()?);
        if v.is_finite() {
            Ok(v)
        } else {
            Err(Error::CorruptSave)
        }
    }
    fn count(&mut self) -> Result<usize> {
        let n = self.u32()? as usize;
        if n > self.records_left {
            return Err(Error::Capacity);
        }
        self.records_left -= n;
        Ok(n)
    }
    fn metadata_count(&mut self, maximum: usize) -> Result<usize> {
        let n = self.u32()? as usize;
        if n > maximum {
            return Err(Error::Capacity);
        }
        Ok(n)
    }
    fn blob(&mut self) -> Result<Vec<u8>> {
        let n = self.u32()? as usize;
        if n > self.limits.max_blob {
            return Err(Error::Capacity);
        }
        Ok(self.take(n)?.to_vec())
    }
    fn string(&mut self) -> Result<String> {
        String::from_utf8(self.blob()?).map_err(|_| Error::CorruptSave)
    }
    fn id(&mut self) -> Result<Id> {
        Ok(Id(self
            .take(16)?
            .try_into()
            .map_err(|_| Error::CorruptSave)?))
    }
    fn pos(&mut self) -> Result<Pos> {
        Ok([self.i32()?, self.i32()?, self.i32()?])
    }
    fn vector(&mut self) -> Result<[f64; 3]> {
        Ok([self.f64()?, self.f64()?, self.f64()?])
    }
    fn payload(&mut self) -> Result<Payload> {
        Ok(Payload {
            format: self.string()?,
            bytes: self.blob()?,
        })
    }
    fn optional_payload(&mut self) -> Result<Option<Payload>> {
        if self.flag()? {
            Ok(Some(self.payload()?))
        } else {
            Ok(None)
        }
    }
    fn motion(&mut self) -> Result<Motion> {
        Ok(Motion {
            pose: Pose {
                position: self.vector()?,
                rotation: [self.f64()?, self.f64()?, self.f64()?, self.f64()?],
                pivot: self.vector()?,
                scale: self.vector()?,
            },
            linear: self.vector()?,
            angular: self.vector()?,
        })
    }
    fn contents(&mut self) -> Result<Contents> {
        let mut blocks = BTreeMap::new();
        for _ in 0..self.count()? {
            let pos = self.pos()?;
            let block = Block {
                state: self.string()?,
                block_entity: self.optional_payload()?,
                extension: self.payload()?,
            };
            if blocks.insert(pos, block).is_some() {
                return Err(Error::DuplicateId);
            }
        }
        let mut entities = BTreeMap::new();
        for _ in 0..self.count()? {
            let id = self.id()?;
            let entity = Entity {
                id,
                anchor: self.pos()?,
                position: self.vector()?,
                payload: self.payload()?,
            };
            if entities.insert(id, entity).is_some() {
                return Err(Error::DuplicateId);
            }
        }
        let mut ticks = Vec::new();
        for _ in 0..self.count()? {
            let id = self.id()?;
            let position = self.pos()?;
            let kind = match self.u8()? {
                0 => TickKind::Block,
                1 => TickKind::Fluid,
                _ => return Err(Error::CorruptSave),
            };
            ticks.push(ScheduledTick {
                id,
                position,
                kind,
                due: self.i64()?,
                priority: self.i32()?,
                payload: self.payload()?,
            });
        }
        Ok(Contents {
            blocks,
            entities,
            ticks,
            extension: self.payload()?,
        })
    }
    fn level(&mut self) -> Result<(Level, Residency)> {
        let id = self.id()?;
        let plot = self.u32()?;
        let revision = self.u64()?;
        let residency = match self.u8()? {
            0 => Residency::Live,
            1 => Residency::Holding,
            _ => return Err(Error::CorruptSave),
        };
        let parent = if self.flag()? { Some(self.id()?) } else { None };
        let mut dependencies = BTreeSet::new();
        for _ in 0..self.metadata_count(self.limits.max_levels)? {
            if !dependencies.insert(self.id()?) {
                return Err(Error::DuplicateId);
            }
        }
        let name = if self.flag()? {
            Some(self.string()?)
        } else {
            None
        };
        Ok((
            Level {
                id,
                plot,
                revision,
                parent,
                dependencies,
                name,
                motion: self.motion()?,
                contents: self.contents()?,
                user_data: self.payload()?,
            },
            residency,
        ))
    }
}

/// Every field and the entire ownership/dependency graph are staged and
/// validated before a new Registry is returned. Existing registries are untouched.
pub fn decode(bytes: &[u8], limits: DecodeLimits) -> Result<Registry> {
    if bytes.len() > limits.max_bytes {
        return Err(Error::Capacity);
    }
    if bytes.len() < 20 || bytes.get(..8) != Some(MAGIC) {
        return Err(Error::CorruptSave);
    }
    let end = bytes.len() - 8;
    let checksum = u64::from_le_bytes(bytes[end..].try_into().map_err(|_| Error::CorruptSave)?);
    if hash(&bytes[..end]) != checksum {
        return Err(Error::CorruptSave);
    }
    let mut r = Reader {
        bytes: &bytes[..end],
        cursor: 8,
        limits,
        records_left: limits.max_records,
    };
    if r.u32()? != VERSION {
        return Err(Error::UnsupportedVersion);
    }
    let config = Config {
        log_grid: r.u8()?,
        log_plot: r.u8()?,
        origin: [r.i32()?, r.i32()?],
        min_y: r.i32()?,
        max_y: r.i32()?,
        max_records: usize::try_from(r.u64()?).map_err(|_| Error::Capacity)?,
    };
    if config.max_records > limits.max_records {
        return Err(Error::Capacity);
    }
    let count = r.metadata_count(limits.max_levels)?;
    let mut levels = Vec::new();
    for _ in 0..count {
        levels.push(r.level()?);
    }
    if r.cursor != r.bytes.len() {
        return Err(Error::CorruptSave);
    }
    Registry::restore(config, levels)
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum WriteFault {
    #[default]
    None,
    BeforeWrite,
    AfterWrite,
    AfterFileSync,
    BeforeRename,
}

static TEMP_SEQUENCE: AtomicU64 = AtomicU64::new(0);

/// Single-writer durable replacement: write a same-directory private temporary
/// file, fsync it, rename atomically, then fsync the directory. Failures before
/// rename leave the last save untouched; interrupted temporary files are ignored.
/// An I/O error after rename has an explicitly uncertain durability outcome.
/// Unix temporaries start at mode 0600; after writing, an existing destination's
/// permissions are applied before the final file sync and rename. This preserves
/// Unix mode bits, not ownership or platform-specific ACLs. Other platforms use
/// their default creation permissions and std's supported Permissions metadata.
pub fn write_atomic(path: &Path, bytes: &[u8], fault: WriteFault) -> Result<()> {
    let parent = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .ok_or(Error::InvalidInput)?;
    let name = path
        .file_name()
        .ok_or(Error::InvalidInput)?
        .to_string_lossy();
    let serial = TEMP_SEQUENCE.fetch_add(1, Ordering::Relaxed);
    let temp = parent.join(format!(".{name}.pending-{}-{serial}", std::process::id()));
    let permissions = match fs::metadata(path) {
        Ok(metadata) => Some(metadata.permissions()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => None,
        Err(_) => return Err(Error::Io),
    };
    let mut owns_temp = false;
    let result = (|| {
        let mut options = OpenOptions::new();
        options.write(true).create_new(true);
        #[cfg(unix)]
        options.mode(0o600);
        let mut file = options.open(&temp).map_err(|_| Error::Io)?;
        owns_temp = true;
        if fault == WriteFault::BeforeWrite {
            return Err(Error::Interrupted);
        }
        file.write_all(bytes).map_err(|_| Error::Io)?;
        if fault == WriteFault::AfterWrite {
            return Err(Error::Interrupted);
        }
        if let Some(permissions) = permissions {
            file.set_permissions(permissions).map_err(|_| Error::Io)?;
        }
        file.sync_all().map_err(|_| Error::Io)?;
        if fault == WriteFault::AfterFileSync || fault == WriteFault::BeforeRename {
            return Err(Error::Interrupted);
        }
        fs::rename(&temp, path).map_err(|_| Error::Io)?;
        owns_temp = false;
        File::open(parent)
            .and_then(|dir| dir.sync_all())
            .map_err(|_| Error::Io)
    })();
    if owns_temp {
        let _ = fs::remove_file(&temp);
    }
    result
}

pub fn read(path: &Path, limits: DecodeLimits) -> Result<Registry> {
    let file = File::open(path).map_err(|_| Error::Io)?;
    let size = usize::try_from(file.metadata().map_err(|_| Error::Io)?.len())
        .map_err(|_| Error::Capacity)?;
    if size > limits.max_bytes {
        return Err(Error::Capacity);
    }
    let mut bytes = Vec::new();
    file.take((limits.max_bytes as u64).saturating_add(1))
        .read_to_end(&mut bytes)
        .map_err(|_| Error::Io)?;
    decode(&bytes, limits)
}

/// The host decides readiness; the core never acquires terrain tickets or polls
/// indefinitely. Bounds use the source's one-block margin and inclusive XZ
/// chunk endpoints. `ready` returns once, preserving pending entries.
pub trait ResidencyHost {
    fn chunk_ready(&self, x: i32, z: i32) -> bool;
}

pub fn ready_chunks(
    bounds: ([f64; 3], [f64; 3]),
    host: &impl ResidencyHost,
    budget: usize,
) -> Result<bool> {
    let (min, max) = bounds;
    if min.iter().chain(max.iter()).any(|n| !n.is_finite()) || (0..3).any(|i| min[i] > max[i]) {
        return Err(Error::InvalidInput);
    }
    let chunk = |n: f64| -> Result<i32> {
        let block = n.floor();
        if block < f64::from(i32::MIN) || block > f64::from(i32::MAX) {
            return Err(Error::InvalidCoordinate);
        }
        Ok((block as i32).div_euclid(16))
    };
    let x0 = chunk(min[0] - 1.0)?;
    let x1 = chunk(max[0] + 1.0)?;
    let z0 = chunk(min[2] - 1.0)?;
    let z1 = chunk(max[2] + 1.0)?;
    let count = i64::from(x1) - i64::from(x0) + 1;
    let count = count
        .checked_mul(i64::from(z1) - i64::from(z0) + 1)
        .ok_or(Error::Capacity)?;
    if count > i64::try_from(budget).map_err(|_| Error::Capacity)? {
        return Err(Error::Capacity);
    }
    for x in x0..=x1 {
        for z in z0..=z1 {
            if !host.chunk_ready(x, z) {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

/// Attempts one bounded load of a dependency closure. Missing terrain returns
/// `false` without changing ownership, velocity, generations or queued changes.
/// Dependencies are made live before dependents, at one atomic core boundary.
pub fn reload_with(
    registry: &mut Registry,
    handle: Handle,
    host: &impl ResidencyHost,
    chunk_budget: usize,
    velocity_policy: VelocityPolicy,
) -> Result<bool> {
    registry.entry(handle)?;
    registry.validate()?;
    if registry.residency(handle)? == Residency::Live {
        return Ok(true);
    }
    let mut closure = BTreeSet::from([handle.id()]);
    let mut pending = vec![handle.id()];
    while let Some(id) = pending.pop() {
        let entry = registry.entries.get(&id).ok_or(Error::MissingDependency)?;
        for dep in &entry.level.dependencies {
            if closure.insert(*dep) {
                pending.push(*dep);
            }
        }
    }
    let mut remaining = chunk_budget;
    for id in &closure {
        let entry = registry.entries.get(id).ok_or(Error::MissingDependency)?;
        if entry.residency == Residency::Live {
            continue;
        }
        let local = entry.level.contents.bounds().ok_or(Error::InvalidInput)?;
        let b = entry.level.motion.pose.world_bounds(local);
        let min = [b[0], b[1], b[2]];
        let max = [b[3], b[4], b[5]];
        // Same endpoints as ready_chunks; checked there before any cast here.
        if !ready_chunks((min, max), host, remaining)? {
            return Ok(false);
        }
        let dx = ((max[0] + 1.0).floor() as i64).div_euclid(16)
            - ((min[0] - 1.0).floor() as i64).div_euclid(16)
            + 1;
        let dz = ((max[2] + 1.0).floor() as i64).div_euclid(16)
            - ((min[2] - 1.0).floor() as i64).div_euclid(16)
            + 1;
        let used = usize::try_from(dx.checked_mul(dz).ok_or(Error::Capacity)?)
            .map_err(|_| Error::Capacity)?;
        remaining = remaining.checked_sub(used).ok_or(Error::Capacity)?;
    }
    let mut staged = registry.staged();
    let mut loaded = BTreeSet::new();
    while loaded.len() < closure.len() {
        let before = loaded.len();
        for id in &closure {
            if loaded.contains(id) {
                continue;
            }
            let entry = staged.entries.get(id).ok_or(Error::MissingDependency)?;
            if !entry
                .level
                .dependencies
                .iter()
                .all(|dep| loaded.contains(dep))
            {
                continue;
            }
            if entry.residency == Residency::Holding {
                let h = entry.handle;
                velocity_policy.apply(
                    &mut staged
                        .entries
                        .get_mut(id)
                        .ok_or(Error::StaleHandle)?
                        .level
                        .motion,
                )?;
                staged.make_live(h)?;
            }
            loaded.insert(*id);
        }
        if before == loaded.len() {
            return Err(Error::CyclicDependency);
        }
    }
    staged.validate()?;
    *registry = staged;
    Ok(true)
}

#[cfg(test)]
mod decoder_tests {
    use super::*;

    fn id(n: u8) -> Id {
        Id([n; 16])
    }
    fn level(n: u8, plot: u32) -> Level {
        let mut l = Level::new(id(n));
        l.plot = plot;
        l.contents.blocks.insert(
            [0; 3],
            Block {
                state: "minecraft:stone".into(),
                block_entity: None,
                extension: Payload::default(),
            },
        );
        l
    }
    fn unchecked(levels: &[(Level, Residency)]) -> Vec<u8> {
        let c = Config::default();
        let mut w = Writer { bytes: Vec::new() };
        w.bytes.extend(MAGIC);
        w.u32(VERSION);
        w.u8(c.log_grid);
        w.u8(c.log_plot);
        for n in c.origin {
            w.i32(n);
        }
        w.i32(c.min_y);
        w.i32(c.max_y);
        w.u64(c.max_records as u64);
        w.len(levels.len()).unwrap();
        for (l, s) in levels {
            w.level(l, *s).unwrap();
        }
        let sum = hash(&w.bytes);
        w.u64(sum);
        w.bytes
    }

    #[test]
    fn decoder_validates_all_ownership_coordinates_and_dependency_graphs_before_returning() {
        let a = level(1, 0);
        let mut b = level(2, 1);
        for levels in [
            vec![
                (a.clone(), Residency::Live),
                (a.clone(), Residency::Holding),
            ],
            vec![(a.clone(), Residency::Live), (level(2, 0), Residency::Live)],
        ] {
            assert!(decode(&unchecked(&levels), DecodeLimits::default()).is_err());
        }
        b.contents.blocks.clear();
        b.contents
            .blocks
            .insert([-1, 0, 0], a.contents.blocks[&[0; 3]].clone());
        assert!(matches!(
            decode(
                &unchecked(&[(b.clone(), Residency::Holding)]),
                DecodeLimits::default()
            ),
            Err(Error::InvalidCoordinate)
        ));
        b = level(2, 1);
        b.dependencies.insert(id(99));
        assert!(matches!(
            decode(
                &unchecked(&[(b.clone(), Residency::Holding)]),
                DecodeLimits::default()
            ),
            Err(Error::MissingDependency)
        ));
        let mut a = a;
        a.dependencies.insert(id(2));
        b.dependencies = BTreeSet::from([id(1)]);
        assert!(matches!(
            decode(
                &unchecked(&[(a, Residency::Holding), (b, Residency::Holding)]),
                DecodeLimits::default()
            ),
            Err(Error::CyclicDependency)
        ));
    }

    #[test]
    fn decoder_record_limit_counts_contents_separately_from_metadata() {
        let c = Config {
            max_records: 1,
            ..Config::default()
        };
        let mut r = Registry::new(c).unwrap();
        r.insert(level(1, 0)).unwrap();
        let bytes = encode(&r).unwrap();
        assert!(
            decode(
                &bytes,
                DecodeLimits {
                    max_records: 1,
                    ..DecodeLimits::default()
                }
            )
            .is_ok()
        );
    }
}
