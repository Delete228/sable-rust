//! Derived physics replicas across the explicit compiler ABI. Live blocks,
//! inventories, entities and scheduled ticks belong exclusively to the host.
use sable_bridge::{Body, Bridge, Mass, Material, MaterialId, Scene, abi::Cell};
use sable_core::{
    Block, Config, Contents, Id, Motion, Payload, Pos,
    mass::{MassRegistry, Properties},
    rotate, sub,
};
use std::cell::RefCell;
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;
use std::rc::Rc;

#[derive(Debug)]
pub enum SimulationError {
    Core(sable_core::Error),
    Native(sable_bridge::Error),
    Stale,
    UnsupportedScale,
}
impl From<sable_core::Error> for SimulationError {
    fn from(value: sable_core::Error) -> Self {
        Self::Core(value)
    }
}
impl From<sable_bridge::Error> for SimulationError {
    fn from(value: sable_bridge::Error) -> Self {
        Self::Native(value)
    }
}
pub type Result<T> = std::result::Result<T, SimulationError>;

/// A short-lived, read-only host geometry observation. It carries no inventory,
/// block-entity bytes or ticker and cannot authorize a host ownership change.
#[derive(Debug, Clone, PartialEq)]
pub struct GeometrySnapshot {
    pub id: Id,
    pub generation: u64,
    pub revision: u64,
    pub plot: u32,
    pub cells: BTreeMap<Pos, String>,
    pub motion: Motion,
}
/// Derived ordinary-world collision inputs, addressed by native section/cell.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TerrainSection {
    pub section: Pos,
    pub cells: BTreeMap<[u8; 3], String>,
}
struct Seen {
    generation: u64,
    revision: u64,
    retired: bool,
}
struct Realized {
    body: Body,
    generation: u64,
    revision: u64,
    plot: u32,
    geometry: BTreeMap<Pos, String>,
    physical: BTreeMap<Pos, Properties>,
    sections: BTreeSet<Pos>,
    published: Motion,
}
struct Input<'a> {
    snapshot: &'a GeometrySnapshot,
    mass: Mass,
    bounds: [i32; 6],
    pose: [f64; 7],
    physical: BTreeMap<Pos, Properties>,
}
#[derive(Clone)]
struct NativeMaterial {
    boxes: Vec<[f64; 6]>,
    id: MaterialId,
}
type TerrainShapes = BTreeMap<Pos, BTreeMap<[u8; 3], Vec<[f64; 6]>>>;

/// One server-thread physics owner, for a bounded initial parent world. Multiple
/// live sublevels share its scene. Bridge and Engine remain non-Send/non-Sync.
pub struct PhysicsReplica {
    bridge: Rc<RefCell<Bridge>>,
    scene: Scene,
    config: Config,
    bodies: BTreeMap<Id, Realized>,
    materials: BTreeMap<String, NativeMaterial>,
    shape_materials: Vec<NativeMaterial>,
    properties: Option<MassRegistry>,
    seen: BTreeMap<Id, Seen>,
    terrain: BTreeMap<Pos, BTreeMap<[u8; 3], String>>,
    terrain_properties: Option<MassRegistry>,
    terrain_shapes: Option<TerrainShapes>,
}
impl PhysicsReplica {
    pub fn open(library: impl AsRef<Path>, config: Config, gravity: [f64; 3]) -> Result<Self> {
        config.validate()?;
        let bridge = Bridge::open(library)?;
        let replica = Self::scene(Rc::new(RefCell::new(bridge)), config, gravity)?;
        // Explicit pinned Java default profile; no SDK/runtime solver participates.
        replica
            .bridge
            .borrow_mut()
            .configure([18, 2, 2], [40., 5.], 128)?;
        Ok(replica)
    }
    /// A distinct gravity scenario under the same actual native Engine owner.
    /// No second thread owner or competing process-global configuration exists.
    pub fn new_scene(&self, config: Config, gravity: [f64; 3]) -> Result<Self> {
        Self::scene(Rc::clone(&self.bridge), config, gravity)
    }
    fn scene(bridge: Rc<RefCell<Bridge>>, config: Config, gravity: [f64; 3]) -> Result<Self> {
        config.validate()?;
        let scene = bridge.borrow_mut().scene(gravity, 0.)?;
        Ok(Self {
            bridge,
            scene,
            config,
            bodies: BTreeMap::new(),
            materials: BTreeMap::new(),
            shape_materials: Vec::new(),
            properties: None,
            seen: BTreeMap::new(),
            terrain: BTreeMap::new(),
            terrain_properties: None,
            terrain_shapes: None,
        })
    }
    pub fn body(&self, id: Id) -> Option<Body> {
        self.bodies.get(&id).map(|b| b.body)
    }
    pub fn counts(&self) -> (usize, usize) {
        (
            self.bodies.len(),
            self.bodies.values().map(|b| b.sections.len()).sum(),
        )
    }
    /// Distinct native shape registrations owned by this replica. The original
    /// process-global registry persists; this is not its global element count.
    pub fn material_count(&self) -> usize {
        self.shape_materials.len()
    }
    /// Explicit native impulse, with the accepted solver's impulse units. Read
    /// back and publish motion before a later geometry synchronization.
    pub fn impulse(
        &mut self,
        id: Id,
        linear: [f64; 3],
        angular: [f64; 3],
        wake: bool,
    ) -> Result<()> {
        let body = self.bodies.get(&id).ok_or(SimulationError::Stale)?.body;
        self.bridge
            .borrow_mut()
            .impulse(body, linear, angular, wake)?;
        Ok(())
    }
    /// Drain actual native 15-channel contact records; not host/player contacts.
    pub fn collisions(&mut self) -> Result<Vec<f64>> {
        Ok(self.bridge.borrow_mut().collisions(self.scene)?)
    }
    /// Actual native world-COM pose and velocities, distinct from the logical
    /// block pose and the host's canonical publication receipt. No cache update.
    pub fn native_sample(&self, id: Id) -> Result<([f64; 7], [f64; 6])> {
        let body = self.bodies.get(&id).ok_or(SimulationError::Stale)?.body;
        let mut bridge = self.bridge.borrow_mut();
        let pose = bridge.pose(body)?;
        let velocity = bridge.velocity(body)?;
        if !pose.iter().chain(&velocity).all(|value| value.is_finite()) {
            return Err(sable_core::Error::InvalidInput.into());
        }
        Ok((pose, velocity))
    }

    /// Validate prospective host geometry before committing a host transfer.
    /// This performs no native or host mutation and grants no transfer authority.
    pub fn validate_snapshots(
        &self,
        snapshots: &[GeometrySnapshot],
        registry: &MassRegistry,
    ) -> Result<()> {
        self.preflight(snapshots, registry).map(|_| ())
    }

    fn preflight<'a>(
        &self,
        snapshots: &'a [GeometrySnapshot],
        registry: &MassRegistry,
    ) -> Result<Vec<Input<'a>>> {
        let mut ids = BTreeSet::new();
        let mut plots = BTreeSet::new();
        let mut count = 0usize;
        let mut inputs = Vec::new();
        for snapshot in snapshots {
            if snapshot.id.0 == [0; 16]
                || snapshot.generation == 0
                || !ids.insert(snapshot.id)
                || !plots.insert(snapshot.plot)
                || snapshot.cells.is_empty()
                || !snapshot.motion.pose.valid()
                || !snapshot
                    .motion
                    .linear
                    .iter()
                    .chain(&snapshot.motion.angular)
                    .all(|v| v.is_finite())
            {
                return Err(sable_core::Error::InvalidInput.into());
            }
            if snapshot.motion.pose.scale != [1.; 3] {
                return Err(SimulationError::UnsupportedScale);
            }
            if self.seen.get(&snapshot.id).is_some_and(|seen| {
                snapshot.generation < seen.generation
                    || (snapshot.generation == seen.generation
                        && (seen.retired || snapshot.revision < seen.revision))
            }) {
                return Err(SimulationError::Stale);
            }
            if let Some(old) = self.bodies.get(&snapshot.id)
                && old.generation == snapshot.generation
                && (snapshot.revision < old.revision
                    || (snapshot.revision == old.revision && snapshot.cells != old.geometry))
            {
                return Err(SimulationError::Stale);
            }
            count = count
                .checked_add(snapshot.cells.len())
                .ok_or(sable_core::Error::Capacity)?;
            if count > self.config.max_records {
                return Err(sable_core::Error::Capacity.into());
            }
            let mut contents = Contents::default();
            let mut physical = BTreeMap::new();
            for (position, state) in &snapshot.cells {
                self.config.reserved(snapshot.plot, *position)?;
                let properties = registry.get(state)?;
                if properties.boxes.len() > sable_bridge::abi::MAX_BOXES
                    || properties
                        .boxes
                        .iter()
                        .flatten()
                        .any(|value| !(*value as f32).is_finite())
                {
                    return Err(sable_core::Error::InvalidInput.into());
                }
                physical.insert(*position, properties.clone());
                contents.blocks.insert(
                    *position,
                    Block {
                        state: state.clone(),
                        block_entity: None,
                        extension: Payload::default(),
                    },
                );
            }
            let mass = registry.compute(&contents)?;
            let bounds = contents.bounds().ok_or(sable_core::Error::InvalidInput)?;
            let min = self
                .config
                .reserved(snapshot.plot, [bounds[0], bounds[1], bounds[2]])?;
            let max = self
                .config
                .reserved(snapshot.plot, [bounds[3], bounds[4], bounds[5]])?;
            let origin = self
                .config
                .reserved(snapshot.plot, [0, self.config.min_y, 0])?;
            let center = [
                mass.center[0] + f64::from(origin[0]),
                mass.center[1],
                mass.center[2] + f64::from(origin[2]),
            ];
            let com = snapshot.motion.pose.transform(mass.center);
            let q = snapshot.motion.pose.rotation;
            let pose = [com[0], com[1], com[2], q[0], q[1], q[2], q[3]];
            if pose
                .iter()
                .chain(&center)
                .chain(&mass.inertia)
                .chain(&snapshot.motion.linear)
                .chain(&snapshot.motion.angular)
                .chain(std::iter::once(&mass.mass))
                .any(|v| !(*v as f32).is_finite())
            {
                return Err(sable_core::Error::InvalidInput.into());
            }
            inputs.push(Input {
                snapshot,
                mass: Mass {
                    mass: mass.mass,
                    center,
                    inertia: mass.inertia,
                },
                bounds: [min[0], min[1], min[2], max[0], max[1], max[2]],
                pose,
                physical,
            });
        }
        let new_identities = ids.iter().filter(|id| !self.seen.contains_key(id)).count();
        if self
            .seen
            .len()
            .checked_add(new_identities)
            .ok_or(sable_core::Error::Capacity)?
            > self.config.max_records
        {
            return Err(sable_core::Error::Capacity.into());
        }
        Ok(inputs)
    }
    fn remove(&mut self, id: Id) -> Result<()> {
        if let Some(old) = self.bodies.remove(&id) {
            for section in old.sections {
                self.bridge
                    .borrow_mut()
                    .remove_chunk(self.scene, section, false)?;
            }
            self.bridge.borrow_mut().remove_body(old.body)?;
        }
        Ok(())
    }
    fn register_material(&mut self, key: &str, registry: &MassRegistry) -> Result<NativeMaterial> {
        let properties = registry.get(key)?;
        if let Some(old) = self
            .shape_materials
            .iter()
            .find(|old| old.boxes == properties.boxes)
        {
            return Ok(old.clone());
        }
        let material = self.bridge.borrow_mut().material(Material {
            friction: 1.,
            volume: 1.,
            restitution: 0.,
            fluid: false,
            dynamic: false,
            boxes: properties.boxes.clone(),
            contact: None,
        })?;
        if material.index >= 65535 {
            return Err(sable_core::Error::Capacity.into());
        }
        let material = NativeMaterial {
            boxes: properties.boxes.clone(),
            id: material,
        };
        self.shape_materials.push(material.clone());
        Ok(material)
    }
    /// The input is the COMPLETE authoritative live owner set, not a view-culling
    /// subset: omissions retire that generation, and reappearance requires a
    /// newer host generation. Identity history is bounded by config.max_records.
    /// All geometry/property validation precedes stale-body disposal. The host
    /// transaction must already have committed; this cache never transfers blocks.
    pub fn sync(&mut self, snapshots: &[GeometrySnapshot], registry: &MassRegistry) -> Result<()> {
        let inputs = self.preflight(snapshots, registry)?;
        let reusable: BTreeSet<_> = inputs
            .iter()
            .filter_map(|input| {
                let snapshot = input.snapshot;
                self.bodies
                    .get(&snapshot.id)
                    .is_some_and(|old| {
                        old.generation == snapshot.generation
                            && old.plot == snapshot.plot
                            && old.physical == input.physical
                            && old.published == snapshot.motion
                    })
                    .then_some(snapshot.id)
            })
            .collect();
        // Register/validate every required material before releasing a body.
        // Global registration is intentionally persistent; allocation failure
        // does not pretend to roll back the original process-global registry.
        let mut materials = self.materials.clone();
        for input in &inputs {
            for key in input.snapshot.cells.values() {
                let boxes = &registry.get(key)?.boxes;
                if !materials.get(key).is_some_and(|old| old.boxes == *boxes) {
                    materials.insert(key.clone(), self.register_material(key, registry)?);
                }
            }
        }
        // Release all replaced native plot IDs before realizing any replacement.
        let removed: Vec<_> = self
            .bodies
            .keys()
            .filter(|id| !reusable.contains(id))
            .copied()
            .collect();
        for id in removed {
            self.remove(id)?;
        }
        self.materials = materials;
        for input in inputs {
            let snapshot = input.snapshot;
            if reusable.contains(&snapshot.id) {
                let old = self.bodies.get_mut(&snapshot.id).unwrap();
                old.revision = snapshot.revision;
                old.geometry = snapshot.cells.clone();
                continue;
            }
            let body = self.bridge.borrow_mut().create_sublevel(
                self.scene,
                snapshot.plot as i32,
                input.pose,
                input.bounds,
                input.mass,
            )?;
            self.bodies.insert(
                snapshot.id,
                Realized {
                    body,
                    generation: snapshot.generation,
                    revision: snapshot.revision,
                    plot: snapshot.plot,
                    geometry: snapshot.cells.clone(),
                    physical: input.physical,
                    sections: BTreeSet::new(),
                    published: snapshot.motion.clone(),
                },
            );
            let realization = (|| -> Result<()> {
                let mut sections: BTreeMap<Pos, Vec<Cell>> = BTreeMap::new();
                for (position, key) in &snapshot.cells {
                    let material = self.materials[key].id;
                    let reserved = self.config.reserved(snapshot.plot, *position)?;
                    sections
                        .entry(reserved.map(|v| v.div_euclid(16)))
                        .or_default()
                        .push(Cell {
                            position: reserved.map(|v| v.rem_euclid(16) as u8),
                            state: 3,
                            reserved: 0,
                            material: material.handle,
                        });
                }
                for (section, cells) in sections {
                    self.bridge.borrow_mut().upload_chunk(
                        self.scene,
                        section,
                        false,
                        Some(body),
                        &cells,
                    )?;
                    self.bodies
                        .get_mut(&snapshot.id)
                        .unwrap()
                        .sections
                        .insert(section);
                }
                self.bridge.borrow_mut().add_velocity(
                    body,
                    snapshot.motion.linear,
                    snapshot.motion.angular,
                    true,
                )?;
                Ok(())
            })();
            if let Err(error) = realization {
                self.remove(snapshot.id)?;
                self.properties = None;
                return Err(error);
            }
        }
        for seen in self.seen.values_mut() {
            seen.retired = true;
        }
        for snapshot in snapshots {
            self.seen.insert(
                snapshot.id,
                Seen {
                    generation: snapshot.generation,
                    revision: snapshot.revision,
                    retired: false,
                },
            );
        }
        let used: BTreeSet<_> = snapshots
            .iter()
            .flat_map(|snapshot| snapshot.cells.values())
            .chain(self.terrain.values().flat_map(|cells| cells.values()))
            .cloned()
            .collect();
        self.materials.retain(|key, _| used.contains(key));
        self.properties = Some(registry.clone());
        Ok(())
    }

    /// Publish actual parent-world shapes before stepping. Empty input means an
    /// observed empty world region. Chunk tickets/residency and the sampled
    /// region remain the host caller's responsibility; missing terrain is not air.
    pub fn sync_terrain(
        &mut self,
        sections: &[TerrainSection],
        registry: &MassRegistry,
    ) -> Result<()> {
        if self.properties.as_ref() != Some(registry) {
            return Err(SimulationError::Stale);
        }
        let mut staged = BTreeMap::new();
        let mut shapes = BTreeMap::new();
        let mut records = sections.len();
        if records > self.config.max_records {
            return Err(sable_core::Error::Capacity.into());
        }
        for section in sections {
            if !section
                .section
                .into_iter()
                .zip([1 << 21, 1 << 19, 1 << 21])
                .all(|(value, half)| (-half..half).contains(&value))
                || staged.contains_key(&section.section)
            {
                return Err(sable_core::Error::InvalidCoordinate.into());
            }
            records = records
                .checked_add(section.cells.len())
                .ok_or(sable_core::Error::Capacity)?;
            if records > self.config.max_records {
                return Err(sable_core::Error::Capacity.into());
            }
            let mut section_shapes = BTreeMap::new();
            for (position, key) in &section.cells {
                if position.iter().any(|v| *v >= 16) {
                    return Err(sable_core::Error::InvalidCoordinate.into());
                }
                let properties = registry.get(key)?;
                if properties.boxes.len() > sable_bridge::abi::MAX_BOXES
                    || properties
                        .boxes
                        .iter()
                        .flatten()
                        .any(|value| !(*value as f32).is_finite())
                {
                    return Err(sable_core::Error::InvalidInput.into());
                }
                section_shapes.insert(*position, properties.boxes.clone());
            }
            staged.insert(section.section, section.cells.clone());
            shapes.insert(section.section, section_shapes);
        }
        let mut materials = self.materials.clone();
        for cells in staged.values() {
            for key in cells.values() {
                let boxes = &registry.get(key)?.boxes;
                if !materials.get(key).is_some_and(|old| old.boxes == *boxes) {
                    materials.insert(key.clone(), self.register_material(key, registry)?);
                }
            }
        }
        let geometry_changed = self.terrain_shapes.as_ref() != Some(&shapes);
        for (section, cells) in &staged {
            if self
                .terrain_shapes
                .as_ref()
                .and_then(|old| old.get(section))
                == shapes.get(section)
            {
                continue;
            }
            let cells: Vec<_> = cells
                .iter()
                .map(|(position, key)| Cell {
                    position: *position,
                    state: 3,
                    reserved: 0,
                    material: materials[key].id.handle,
                })
                .collect();
            self.bridge
                .borrow_mut()
                .upload_chunk(self.scene, *section, true, None, &cells)?;
        }
        let removed: Vec<_> = self
            .terrain
            .keys()
            .filter(|section| !staged.contains_key(*section))
            .copied()
            .collect();
        for section in removed {
            self.bridge
                .borrow_mut()
                .remove_chunk(self.scene, section, true)?;
        }
        // Conservative initial-profile invalidation. A real terrain edit must
        // release sleeping support; unchanged observations do not wake bodies.
        if geometry_changed {
            for old in self.bodies.values() {
                self.bridge.borrow_mut().wake(old.body)?;
            }
        }
        self.materials = materials;
        self.terrain = staged;
        self.terrain_properties = Some(registry.clone());
        self.terrain_shapes = Some(shapes);
        Ok(())
    }
    /// Execute the caller's configured game cadence. Immediate native callbacks
    /// stay synchronous; no Host borrow or asynchronous event queue is retained.
    pub fn tick(&mut self, substeps: u32) -> Result<()> {
        if !(1..=10).contains(&substeps) {
            return Err(sable_core::Error::InvalidInput.into());
        }
        if self.terrain_properties.is_none() || self.terrain_properties != self.properties {
            return Err(SimulationError::Stale);
        }
        self.bridge.borrow_mut().tick(self.scene)?;
        for _ in 0..substeps {
            self.bridge
                .borrow_mut()
                .step(self.scene, 0.05 / f64::from(substeps))?;
        }
        Ok(())
    }
    /// Read every pose before changing any publication metadata. The returned
    /// world motion is for host pose publication, never a host save barrier.
    pub fn capture(&mut self, registry: &MassRegistry) -> Result<BTreeMap<Id, Motion>> {
        if self.properties.as_ref() != Some(registry) {
            return Err(SimulationError::Stale);
        }
        let mut result = BTreeMap::new();
        for (id, old) in &self.bodies {
            let pose = self.bridge.borrow_mut().pose(old.body)?;
            let velocity = self.bridge.borrow_mut().velocity(old.body)?;
            let mut contents = Contents::default();
            for (position, state) in &old.geometry {
                contents.blocks.insert(
                    *position,
                    Block {
                        state: state.clone(),
                        block_entity: None,
                        extension: Payload::default(),
                    },
                );
            }
            let center = registry.compute(&contents)?.center;
            let mut motion = old.published.clone();
            motion.pose.rotation = [pose[3], pose[4], pose[5], pose[6]];
            motion.pose.position = sub(
                [pose[0], pose[1], pose[2]],
                rotate(motion.pose.rotation, sub(center, motion.pose.pivot)),
            );
            motion.linear = velocity[..3].try_into().unwrap();
            motion.angular = velocity[3..].try_into().unwrap();
            if !motion.pose.valid()
                || !motion
                    .linear
                    .iter()
                    .chain(&motion.angular)
                    .all(|v| v.is_finite())
            {
                return Err(sable_core::Error::InvalidInput.into());
            }
            result.insert(*id, motion);
        }
        for (id, motion) in &result {
            self.bodies.get_mut(id).unwrap().published = motion.clone();
        }
        Ok(result)
    }
}
impl Drop for PhysicsReplica {
    fn drop(&mut self) {
        if let Ok(mut bridge) = self.bridge.try_borrow_mut() {
            let _ = bridge.dispose_scene(self.scene);
        }
    }
}
