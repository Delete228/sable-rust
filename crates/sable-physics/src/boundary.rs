//! Owned single-owner boundary around the preserved Sable algorithms.
use super::*;
pub use marten::level::{
    ContactPolicy, ContactRequest, ContactResponse, ContactWorld, ContactWorldError,
};
use std::marker::PhantomData;
use std::rc::Rc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Mutex, MutexGuard};
static OWNER: Mutex<()> = Mutex::new(());
static NEXT_OWNER: AtomicU64 = AtomicU64::new(1);
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    ProcessBusy,
    ReentrantOperation,
    StaleScene,
    StaleBody,
    WrongScene,
    AlreadyExists,
    InvalidInput,
    StaleJoint,
}
pub type Result<T> = std::result::Result<T, Error>;
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SceneHandle {
    owner: u64,
    index: u64,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BodyHandle {
    scene: SceneHandle,
    id: i32,
    generation: u64,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MaterialHandle(usize);
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct JointHandle {
    scene: SceneHandle,
    raw: i64,
}
struct OwnedScene {
    scene: PhysicsScene,
    bodies: HashMap<i32, (u64, bool)>,
    generations: HashMap<i32, u64>,
}
/// A process-wide owner, deliberately neither Send nor Sync. Materials and
/// integration configuration survive all scene disposal, as in the original.
pub struct Engine {
    _guard: MutexGuard<'static, ()>,
    owner: u64,
    next_scene: u64,
    scenes: HashMap<u64, OwnedScene>,
    _thread: PhantomData<Rc<()>>,
}
#[derive(Debug, Clone)]
pub struct Material {
    pub friction: f64,
    pub volume: f64,
    pub restitution: f64,
    pub fluid: bool,
    pub dynamic: bool,
    pub boxes: Vec<[f64; 6]>,
    pub contact: Option<Arc<dyn ContactPolicy>>,
}
#[derive(Debug, Clone, Copy)]
pub struct BodyMass {
    pub mass: f64,
    pub center: [f64; 3],
    pub inertia: [f64; 9],
}
fn finite(v: &[f64]) -> bool {
    v.iter().all(|x| x.is_finite() && (*x as f32).is_finite())
}
fn pose_valid(p: &[f64; 7]) -> bool {
    finite(p) && (p[3..].iter().map(|x| x * x).sum::<f64>() - 1.).abs() < 1e-6
}
fn box_valid(b: &[f64; 6]) -> bool {
    finite(b) && (0..3).all(|i| b[i] <= b[i + 3])
}
/// Domain of the original signed 22/20/22-bit packed section key. Validate
/// before packing: masking an out-of-domain value would alias a real section.
pub(crate) fn section_in_domain(position: [i32; 3]) -> bool {
    const HALF_RANGES: [i32; 3] = [1 << 21, 1 << 19, 1 << 21];
    position
        .into_iter()
        .zip(HALF_RANGES)
        .all(|(coordinate, half)| (-half..half).contains(&coordinate))
}

/// Arithmetic section conversion gives the corresponding signed 26/24/26-bit
/// block domain, including the last cell of each valid boundary section.
pub(crate) fn block_in_domain(position: [i32; 3]) -> bool {
    section_in_domain(position.map(|coordinate| coordinate >> CHUNK_SHIFT))
}
impl Engine {
    pub fn try_new() -> Result<Self> {
        if marten::level::in_contact_callback() {
            return Err(Error::ReentrantOperation);
        }
        let guard = OWNER.try_lock().map_err(|_| Error::ProcessBusy)?;
        Ok(Self {
            _guard: guard,
            owner: NEXT_OWNER.fetch_add(1, Ordering::Relaxed),
            next_scene: 0,
            scenes: HashMap::new(),
            _thread: PhantomData,
        })
    }
    fn scene(&self, h: SceneHandle) -> Result<&OwnedScene> {
        if h.owner != self.owner {
            return Err(Error::StaleScene);
        }
        self.scenes.get(&h.index).ok_or(Error::StaleScene)
    }
    fn body(&self, h: BodyHandle) -> Result<&PhysicsScene> {
        let s = self.scene(h.scene)?;
        if s.bodies.get(&h.id).map(|v| v.0) != Some(h.generation) {
            return Err(Error::StaleBody);
        }
        Ok(&s.scene)
    }
    fn insert_body(&mut self, s: SceneHandle, id: i32, level: bool) -> Result<BodyHandle> {
        self.scene(s)?;
        if id < 0 {
            return Err(Error::InvalidInput);
        }
        let scene = self.scenes.get_mut(&s.index).unwrap();
        if scene.bodies.contains_key(&id) {
            return Err(Error::AlreadyExists);
        }
        let generation = scene.generations.entry(id).or_insert(0);
        *generation += 1;
        scene.bodies.insert(id, (*generation, level));
        Ok(BodyHandle {
            scene: s,
            id,
            generation: *generation,
        })
    }
    pub fn create_scene(&mut self, gravity: [f64; 3], drag: f64) -> Result<SceneHandle> {
        if !finite(&gravity) || !finite(&[drag]) || drag < 0. {
            return Err(Error::InvalidInput);
        }
        let s = initialize(gravity[0], gravity[1], gravity[2], drag);
        self.next_scene += 1;
        let h = SceneHandle {
            owner: self.owner,
            index: self.next_scene,
        };
        self.scenes.insert(
            h.index,
            OwnedScene {
                scene: s,
                bodies: HashMap::new(),
                generations: HashMap::new(),
            },
        );
        Ok(h)
    }
    pub fn dispose_scene(&mut self, h: SceneHandle) -> Result<()> {
        self.scene(h)?;
        self.scenes.remove(&h.index);
        Ok(())
    }
    pub fn configure(
        &mut self,
        iterations: [i32; 3],
        frequency: [f64; 2],
        min_island: i32,
    ) -> Result<()> {
        if PHYSICS_STATE.get().is_none()
            || iterations.iter().any(|&x| x < 0)
            || min_island < 0
            || !finite(&frequency)
        {
            return Err(Error::InvalidInput);
        }
        config::configSolverIterations(iterations[0], iterations[1], iterations[2]);
        config::configFrequencyAndDamping(frequency[0], frequency[1]);
        config::configMinIslandSize(min_island);
        Ok(())
    }
    pub fn register_material(&mut self, m: Material) -> Result<MaterialHandle> {
        if PHYSICS_STATE.get().is_none()
            || !finite(&[m.friction, m.volume, m.restitution])
            || m.boxes.iter().any(|b| !box_valid(b))
        {
            return Err(Error::InvalidInput);
        }
        let mut state = get_physics_state_mut();
        let index = state.voxel_collider_map.voxel_colliders.len();
        state
            .voxel_collider_map
            .voxel_colliders
            .push(Some(marten::level::VoxelColliderData {
                collision_boxes: m
                    .boxes
                    .into_iter()
                    .map(|b| {
                        (
                            b[0] as f32,
                            b[1] as f32,
                            b[2] as f32,
                            b[3] as f32,
                            b[4] as f32,
                            b[5] as f32,
                        )
                    })
                    .collect(),
                is_fluid: m.fluid,
                friction: m.friction as f32,
                volume: m.volume as f32,
                restitution: m.restitution as f32,
                contact_events: m.contact,
                dynamic: m.dynamic,
            }));
        Ok(MaterialHandle(index))
    }
    pub fn clear_material_boxes(&mut self, m: MaterialHandle) -> Result<()> {
        let mut s = get_physics_state_mut();
        let d = s
            .voxel_collider_map
            .voxel_colliders
            .get_mut(m.0)
            .and_then(Option::as_mut)
            .ok_or(Error::InvalidInput)?;
        d.collision_boxes.clear();
        Ok(())
    }
    pub fn add_material_box(&mut self, m: MaterialHandle, b: [f64; 6]) -> Result<()> {
        if !box_valid(&b) {
            return Err(Error::InvalidInput);
        }
        let mut s = get_physics_state_mut();
        let d = s
            .voxel_collider_map
            .voxel_colliders
            .get_mut(m.0)
            .and_then(Option::as_mut)
            .ok_or(Error::InvalidInput)?;
        d.collision_boxes.push((
            b[0] as f32,
            b[1] as f32,
            b[2] as f32,
            b[3] as f32,
            b[4] as f32,
            b[5] as f32,
        ));
        Ok(())
    }
    pub fn material_index(&self, m: MaterialHandle) -> usize {
        m.0
    }
    pub fn create_box(
        &mut self,
        s: SceneHandle,
        id: i32,
        mass: f64,
        half: [f64; 3],
        pose: [f64; 7],
    ) -> Result<BodyHandle> {
        if !pose_valid(&pose)
            || !finite(&half)
            || half.iter().any(|&x| x <= 0.)
            || !finite(&[mass])
            || mass <= 0.
        {
            return Err(Error::InvalidInput);
        }
        let h = self.insert_body(s, id, false)?;
        boxes::createBox(
            &self.scene(s)?.scene,
            id,
            mass,
            half[0],
            half[1],
            half[2],
            pose,
        );
        Ok(h)
    }
    pub fn create_sublevel(
        &mut self,
        s: SceneHandle,
        id: i32,
        pose: [f64; 7],
        bounds: [i32; 6],
        mass: BodyMass,
    ) -> Result<BodyHandle> {
        if !pose_valid(&pose)
            || !finite(&mass.center)
            || !finite(&mass.inertia)
            || !finite(&[mass.mass])
            || mass.mass <= 0.
            || (0..3).any(|i| {
                bounds[i] > bounds[i + 3]
                    || (bounds[i + 3] as i64 - bounds[i] as i64) > ((1i64 << 30) - 1)
            })
        {
            return Err(Error::InvalidInput);
        }
        let h = self.insert_body(s, id, true)?;
        let scene = &self.scene(s)?.scene;
        createSubLevel(scene, id, pose);
        setCenterOfMass(scene, id, mass.center[0], mass.center[1], mass.center[2]);
        setLocalBounds(
            scene, id, bounds[0], bounds[1], bounds[2], bounds[3], bounds[4], bounds[5],
        );
        setMassProperties(scene, id, mass.mass, mass.center, mass.inertia);
        Ok(h)
    }
    pub fn pose(&self, h: BodyHandle) -> Result<[f64; 7]> {
        let mut a = [0.; 7];
        getPose(self.body(h)?, h.id, &mut a);
        Ok(a)
    }
    pub fn velocity(&self, h: BodyHandle) -> Result<[f64; 6]> {
        let mut l = [0.; 3];
        let mut a = [0.; 3];
        getLinearVelocity(self.body(h)?, h.id, &mut l);
        getAngularVelocity(self.body(h)?, h.id, &mut a);
        Ok([l[0], l[1], l[2], a[0], a[1], a[2]])
    }
    pub fn wake(&mut self, h: BodyHandle) -> Result<()> {
        wakeUpObject(self.body(h)?, h.id);
        Ok(())
    }
    pub fn impulse(&mut self, h: BodyHandle, l: [f64; 3], a: [f64; 3], wake: bool) -> Result<()> {
        if !finite(&l) || !finite(&a) {
            return Err(Error::InvalidInput);
        }
        applyForceAndTorque(
            self.body(h)?,
            h.id,
            l[0],
            l[1],
            l[2],
            a[0],
            a[1],
            a[2],
            wake as u8,
        );
        Ok(())
    }
    pub fn add_velocity(
        &mut self,
        h: BodyHandle,
        l: [f64; 3],
        a: [f64; 3],
        wake: bool,
    ) -> Result<()> {
        if !finite(&l) || !finite(&a) {
            return Err(Error::InvalidInput);
        }
        addLinearAngularVelocities(
            self.body(h)?,
            h.id,
            l[0],
            l[1],
            l[2],
            a[0],
            a[1],
            a[2],
            wake as u8,
        );
        Ok(())
    }
    pub fn step(&mut self, s: SceneHandle, dt: f64) -> Result<()> {
        if !dt.is_finite() || dt <= 0. || dt > 1. {
            return Err(Error::InvalidInput);
        }
        step(&self.scene(s)?.scene, dt);
        Ok(())
    }
    pub fn tick(&mut self, s: SceneHandle) -> Result<()> {
        tick(&self.scene(s)?.scene);
        Ok(())
    }
    pub fn collisions(&mut self, s: SceneHandle) -> Result<Vec<f64>> {
        Ok(clearCollisions(&self.scene(s)?.scene))
    }
    pub fn upload_chunk(
        &mut self,
        s: SceneHandle,
        coord: [i32; 3],
        global: bool,
        body: Option<BodyHandle>,
        cells: Vec<([u8; 3], MaterialHandle, u8)>,
    ) -> Result<()> {
        self.scene(s)?;
        if !section_in_domain(coord) {
            return Err(Error::InvalidInput);
        }
        if global && body.is_some() {
            return Err(Error::InvalidInput);
        }
        if !global {
            let h = body.ok_or(Error::InvalidInput)?;
            self.body(h)?;
            if !self
                .scene(s)?
                .bodies
                .get(&h.id)
                .is_some_and(|(_, level)| *level)
            {
                return Err(Error::InvalidInput);
            }
        }
        let id = if let Some(h) = body {
            if h.scene != s {
                return Err(Error::WrongScene);
            }
            self.body(h)?;
            h.id
        } else {
            -1
        };
        let mut packed = [0i32; 4096];
        let state = get_physics_state();
        for (p, m, n) in cells {
            if p.iter().any(|&x| x >= 16)
                || n > 4
                || m.0 >= state.voxel_collider_map.voxel_colliders.len()
                || m.0 >= 65535
            {
                return Err(Error::InvalidInput);
            }
            packed[p[0] as usize + ((p[2] as usize) << 4) + ((p[1] as usize) << 8)] =
                (((m.0 + 1) as i32) << 16) | n as i32;
        }
        drop(state);
        addChunk(
            &self.scene(s)?.scene,
            coord[0],
            coord[1],
            coord[2],
            packed,
            global as u8,
            id,
        );
        Ok(())
    }
    pub fn remove_chunk(&mut self, s: SceneHandle, coord: [i32; 3], global: bool) -> Result<()> {
        self.scene(s)?;
        if !section_in_domain(coord) {
            return Err(Error::InvalidInput);
        }
        removeChunk(
            &self.scene(s)?.scene,
            coord[0],
            coord[1],
            coord[2],
            global as u8,
        );
        Ok(())
    }
    pub fn remove_body(&mut self, h: BodyHandle) -> Result<()> {
        self.body(h)?;
        let owned = self.scenes.get_mut(&h.scene.index).unwrap();
        let (_, level) = owned.bodies.remove(&h.id).unwrap();
        if level {
            removeSubLevel(&owned.scene, h.id)
        } else {
            boxes::removeBox(&owned.scene, h.id)
        }
        Ok(())
    }
    pub fn fixed_joint(
        &mut self,
        s: SceneHandle,
        a: Option<BodyHandle>,
        b: Option<BodyHandle>,
        anchors: [f64; 6],
        q: [f64; 4],
    ) -> Result<JointHandle> {
        if !finite(&anchors) || !finite(&q) {
            return Err(Error::InvalidInput);
        }
        for h in [a, b].into_iter().flatten() {
            if h.scene != s {
                return Err(Error::WrongScene);
            }
            self.body(h)?;
        }
        let raw = joints::addFixedConstraint(
            &self.scene(s)?.scene,
            a.map_or(-1, |h| h.id),
            b.map_or(-1, |h| h.id),
            anchors[0],
            anchors[1],
            anchors[2],
            anchors[3],
            anchors[4],
            anchors[5],
            q[0],
            q[1],
            q[2],
            q[3],
        );
        Ok(JointHandle { scene: s, raw })
    }
    pub fn joint_sample(&self, h: JointHandle) -> Result<(bool, [f64; 6])> {
        let s = &self.scene(h.scene)?.scene;
        let valid = joints::isConstraintValid(s, h.raw) != 0 && joints::backend_valid(s, h.raw);
        if !valid {
            return Ok((false, [0.; 6]));
        }
        let mut a = [0.; 6];
        joints::getConstraintImpulses(s, h.raw, &mut a);
        Ok((true, a))
    }
    pub fn remove_joint(&mut self, h: JointHandle) -> Result<()> {
        joints::removeConstraint(&self.scene(h.scene)?.scene, h.raw);
        Ok(())
    }
}
