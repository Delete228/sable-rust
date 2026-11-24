//! Derived physics realization of the core, using the accepted Engine unchanged.
//! This owner is a cache of bodies/sections, never a second block authority.
use crate::{mass::MassRegistry, *};
use sable_physics::{BodyHandle, BodyMass, Engine, Material, MaterialHandle, SceneHandle};
#[derive(Debug)]
pub enum PhysicsError {
    Core(Error),
    Engine(sable_physics::Error),
    UnsupportedScale,
}
impl From<Error> for PhysicsError {
    fn from(e: Error) -> Self {
        Self::Core(e)
    }
}
impl From<sable_physics::Error> for PhysicsError {
    fn from(e: sable_physics::Error) -> Self {
        Self::Engine(e)
    }
}
pub type PhysicsResult<T> = std::result::Result<T, PhysicsError>;
struct Realized {
    handle: BodyHandle,
    revision: u64,
    sections: BTreeSet<Pos>,
    generation: Handle,
    // Derived collision identity only; payloads remain exclusively in Contents.
    geometry: Vec<(Pos, String)>,
    published_motion: Motion,
}
pub struct PhysicsOwner {
    pub engine: Engine,
    pub scene: SceneHandle,
    bodies: BTreeMap<Id, Realized>,
    materials: BTreeMap<String, MaterialHandle>,
    properties: Option<MassRegistry>,
}
impl PhysicsOwner {
    pub fn new(gravity: [f64; 3]) -> PhysicsResult<Self> {
        let mut engine = Engine::try_new()?;
        let scene = engine.create_scene(gravity, 0.)?;
        Ok(Self {
            engine,
            scene,
            bodies: BTreeMap::new(),
            materials: BTreeMap::new(),
            properties: None,
        })
    }
    pub fn body(&self, id: Id) -> Option<BodyHandle> {
        self.bodies.get(&id).map(|b| b.handle)
    }
    pub fn counts(&self) -> (usize, usize) {
        (
            self.bodies.len(),
            self.bodies.values().map(|b| b.sections.len()).sum(),
        )
    }
    fn remove(&mut self, id: Id) -> PhysicsResult<()> {
        if let Some(b) = self.bodies.remove(&id) {
            for p in b.sections {
                self.engine.remove_chunk(self.scene, p, false)?;
            }
            self.engine.remove_body(b.handle)?;
        }
        Ok(())
    }
    pub fn sync(&mut self, r: &Registry, masses: &MassRegistry) -> PhysicsResult<()> {
        r.validate()?;
        // Preflight every active input before removing any existing body.
        let mut inputs = Vec::new();
        for (l, s) in r.levels() {
            if s != Residency::Live || l.contents.blocks.is_empty() {
                continue;
            }
            if l.motion.pose.scale != [1.; 3] {
                return Err(PhysicsError::UnsupportedScale);
            }
            let m = masses.compute(&l.contents)?;
            let b = l.contents.bounds().unwrap();
            let min = r.config.reserved(l.plot, [b[0], b[1], b[2]])?;
            let max = r.config.reserved(l.plot, [b[3], b[4], b[5]])?;
            let origin = r.config.reserved(l.plot, [0, r.config.min_y, 0])?;
            let center = [
                m.center[0] + origin[0] as f64,
                m.center[1],
                m.center[2] + origin[2] as f64,
            ];
            let world = l.motion.pose.transform(m.center);
            let q = l.motion.pose.rotation;
            let pose = [world[0], world[1], world[2], q[0], q[1], q[2], q[3]];
            if pose
                .iter()
                .chain(center.iter())
                .chain(m.inertia.iter())
                .chain(l.motion.linear.iter())
                .chain(l.motion.angular.iter())
                .chain(std::iter::once(&m.mass))
                .any(|v| !(*v as f32).is_finite())
            {
                return Err(PhysicsError::Core(Error::InvalidInput));
            }
            for block in l.contents.blocks.values() {
                let p = masses.get(&block.state)?;
                if p.boxes.iter().flatten().any(|v| !(*v as f32).is_finite()) {
                    return Err(PhysicsError::Core(Error::InvalidInput));
                }
            }
            inputs.push((
                l,
                r.handle(l.id)?,
                BodyMass {
                    mass: m.mass,
                    center,
                    inertia: m.inertia,
                },
                [min[0], min[1], min[2], max[0], max[1], max[2]],
                pose,
            ));
        }
        let properties_changed = self.properties.as_ref() != Some(masses);
        if properties_changed {
            self.materials.clear();
        }
        // Native bodies use plot slots, not UUIDs. Release every replacement
        // before creating any body: retained UUIDs can exchange slots in a
        // valid batch, including cycles with no intermediate empty sync.
        let reusable: BTreeSet<_> = inputs
            .iter()
            .filter_map(|(l, h, _, _, _)| {
                (!properties_changed
                    && self.bodies.get(&l.id).is_some_and(|b| {
                        b.generation == *h
                            && b.published_motion == l.motion
                            && b.geometry.iter().map(|(p, state)| (p, state.as_str())).eq(l
                                .contents
                                .blocks
                                .iter()
                                .map(|(p, block)| (p, block.state.as_str())))
                    }))
                .then_some(l.id)
            })
            .collect();
        let removed: Vec<_> = self
            .bodies
            .keys()
            .filter(|id| !reusable.contains(id))
            .copied()
            .collect();
        for id in removed {
            self.remove(id)?;
        }
        for (l, h, mass, bounds, pose) in inputs {
            if reusable.contains(&l.id) {
                // Content-only edits still advance the capture/save boundary,
                // while preserving the actual body's contacts, sleep and motion.
                self.bodies.get_mut(&l.id).unwrap().revision = l.revision;
                continue;
            }
            let body =
                self.engine
                    .create_sublevel(self.scene, l.plot as i32, pose, bounds, mass)?;
            self.bodies.insert(
                l.id,
                Realized {
                    handle: body,
                    revision: l.revision,
                    sections: BTreeSet::new(),
                    generation: h,
                    geometry: l
                        .contents
                        .blocks
                        .iter()
                        .map(|(p, b)| (*p, b.state.clone()))
                        .collect(),
                    published_motion: l.motion.clone(),
                },
            );
            let result = (|| -> PhysicsResult<()> {
                let mut sections: BTreeMap<Pos, Vec<([u8; 3], MaterialHandle, u8)>> =
                    BTreeMap::new();
                for (p, b) in &l.contents.blocks {
                    let props = masses.get(&b.state)?;
                    let material = if let Some(m) = self.materials.get(&b.state) {
                        *m
                    } else {
                        let m = self.engine.register_material(Material {
                            friction: 1.,
                            volume: 1.,
                            restitution: 0.,
                            fluid: false,
                            dynamic: false,
                            boxes: props.boxes.clone(),
                            contact: None,
                        })?;
                        self.materials.insert(b.state.clone(), m);
                        m
                    };
                    let p = r.config.reserved(l.plot, *p)?;
                    let section = p.map(|v| v.div_euclid(16));
                    let cell = p.map(|v| v.rem_euclid(16) as u8);
                    // Conservative CORNER (3) keeps all supplied collision shapes;
                    // interior acceleration is not a separate occupancy authority.
                    sections
                        .entry(section)
                        .or_default()
                        .push((cell, material, 3));
                }
                for (section, cells) in sections {
                    self.engine
                        .upload_chunk(self.scene, section, false, Some(body), cells)?;
                    self.bodies.get_mut(&l.id).unwrap().sections.insert(section);
                }
                self.engine
                    .add_velocity(body, l.motion.linear, l.motion.angular, true)?;
                Ok(())
            })();
            if let Err(error) = result {
                self.remove(l.id)?;
                self.properties = None;
                return Err(error);
            }
        }
        self.properties = Some(masses.clone());
        Ok(())
    }
    /// Publish actual solver state at a consistent boundary before edits/save.
    pub fn capture(&mut self, r: &mut Registry, masses: &MassRegistry) -> PhysicsResult<()> {
        if self.properties.as_ref() != Some(masses) {
            return Err(PhysicsError::Core(Error::StaleHandle));
        }
        let mut motions = Vec::new();
        for (id, b) in &self.bodies {
            let e = r.entries.get(id).ok_or(Error::StaleHandle)?;
            if e.handle != b.generation || e.level.revision != b.revision {
                return Err(PhysicsError::Core(Error::StaleHandle));
            }
            let p = self.engine.pose(b.handle)?;
            let v = self.engine.velocity(b.handle)?;
            let com = masses.compute(&e.level.contents)?.center;
            let mut motion = e.level.motion.clone();
            let pose = &mut motion.pose;
            pose.rotation = [p[3], p[4], p[5], p[6]];
            pose.position = sub(
                [p[0], p[1], p[2]],
                rotate(pose.rotation, sub(com, pose.pivot)),
            );
            motion.linear = [v[0], v[1], v[2]];
            motion.angular = [v[3], v[4], v[5]];
            motions.push((*id, motion));
        }
        let mut staged = r.staged();
        for (id, motion) in motions {
            staged.entries.get_mut(&id).unwrap().level.motion = motion;
        }
        staged.validate()?;
        for (id, body) in &mut self.bodies {
            body.published_motion = staged.entries[id].level.motion.clone();
        }
        *r = staged;
        Ok(())
    }
}
