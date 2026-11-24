// Derived from Sable 9b1a3d1555d35db9fb7b1103701e7490f0a968b9.
// PolyForm Shield 1.0.0; see ../LICENSE-SABLE.md and provenance/origin.json.
mod algo;
mod boxes;
mod buoyancy;
mod collider;
mod config;

mod dispatcher;
mod event_handler;
mod groups;
mod hooks;
mod joints;
mod rope;
mod scene;
mod voxel_collider;

use rapier3d::glamx::{DVec3, Quat};
use std::collections::HashMap;

use std::sync::{Arc, OnceLock, RwLock};

use crate::buoyancy::compute_buoyancy;
use crate::collider::{LevelCollider, update_collider_aabb};
use crate::dispatcher::SableDispatcher;
use crate::event_handler::SableEventHandler;
use crate::glamx::IVec3;
use crate::groups::LEVEL_GROUP;
use crate::joints::SableJointSet;
use crate::rope::RopeMap;
use crate::scene::{
    ChunkAccess, ChunkMap, SableManifoldInfoMap, SableSceneData, SimulationSceneData,
    pack_section_pos,
};
use crate::voxel_collider::VoxelColliderMap;
use hooks::SablePhysicsHooks;
use marten::Real;
use marten::level::VoxelPhysicsState::Interior;
use marten::level::{
    ALL_VOXEL_PHYSICS_STATES, BlockState, CHUNK_SHIFT, ChunkSection, OCTREE_CHUNK_SHIFT,
    OCTREE_CHUNK_SIZE, OctreeChunkSection, VoxelPhysicsState,
};
use marten::octree::SubLevelOctree;
use rapier3d::parry::query::{DefaultQueryDispatcher, QueryDispatcher};
use rapier3d::prelude::*;
use scene::{LevelColliderID, PhysicsScene, ReportedCollisionBuffer};

#[derive(Debug)]
pub(crate) struct ActiveLevelColliderInfo {
    pub collider: ColliderHandle,
    pub static_mount: Option<RigidBodyHandle>,
    pub fake_velocities: Option<RigidBodyVelocity<Real>>,
    pub local_bounds_min: Option<IVec3>,
    pub local_bounds_max: Option<IVec3>,
    pub center_of_mass: Option<DVec3>,
    pub octree: Option<SubLevelOctree>,
    pub chunk_map: Option<ChunkMap>,
}

impl ChunkAccess for ActiveLevelColliderInfo {
    fn get_chunk_mut(&mut self, x: i32, y: i32, z: i32) -> Option<&mut ChunkSection> {
        self.chunk_map
            .as_mut()
            .unwrap()
            .get_mut(&pack_section_pos(x, y, z))
    }

    fn get_chunk(&self, x: i32, y: i32, z: i32) -> Option<&ChunkSection> {
        self.chunk_map
            .as_ref()
            .unwrap()
            .get(&pack_section_pos(x, y, z))
    }
}

impl ActiveLevelColliderInfo {
    /// Creates a new handle for a sable object with rigidbody and collider handles
    #[must_use]
    pub fn new(collider: ColliderHandle) -> Self {
        Self {
            collider,
            static_mount: None,
            fake_velocities: None,
            chunk_map: None,
            local_bounds_min: None,
            local_bounds_max: None,
            center_of_mass: None,
            octree: None,
        }
    }

    pub fn has_own_chunks(&self) -> bool {
        self.chunk_map.is_some()
    }

    /// Sets the local bounds for the object
    pub fn set_local_bounds(
        &mut self,
        min: IVec3,
        max: IVec3,
        level_chunks: &ChunkMap,
        collider_map: &VoxelColliderMap,
    ) {
        if Some(min) != self.local_bounds_min || Some(max) != self.local_bounds_max {
            self.local_bounds_min = Some(min);
            self.local_bounds_max = Some(max);

            let max_axis = (max - min).max_element() as u32 + 1;
            let smallest_pow_2_above = max_axis.next_power_of_two();

            let chunk_min = min >> CHUNK_SHIFT;
            let chunk_max = max >> CHUNK_SHIFT;

            self.octree = Some(SubLevelOctree::new(
                smallest_pow_2_above.trailing_zeros() as i32
            ));

            let has_own_chunks = self.has_own_chunks();

            for cx in chunk_min.x..=chunk_max.x {
                for cy in chunk_min.y..=chunk_max.y {
                    for cz in chunk_min.z..=chunk_max.z {
                        let chunk = if has_own_chunks {
                            self.chunk_map
                                .as_ref()
                                .unwrap()
                                .get(&pack_section_pos(cx, cy, cz))
                        } else {
                            level_chunks.get(&pack_section_pos(cx, cy, cz))
                        };

                        if let Some(chunk_section) = chunk {
                            for x in 0..16 {
                                for y in 0..16 {
                                    for z in 0..16 {
                                        let block_owned = chunk_section.get_block(x, y, z);
                                        if block_owned.1 == VoxelPhysicsState::Empty {
                                            continue;
                                        }

                                        insert_block_octree(
                                            collider_map,
                                            self.octree.as_mut().unwrap(),
                                            &block_owned,
                                            false,
                                            (x + (cx << CHUNK_SHIFT)) - min.x,
                                            (y + (cy << CHUNK_SHIFT)) - min.y,
                                            (z + (cz << CHUNK_SHIFT)) - min.z,
                                        );
                                    }
                                }
                            }
                        }
                        // let chunk = scene.main_level_chunks.get(&pack_section_pos(cx, cy, cz));

                        // if let Some(chunk) = chunk {
                        //     self.insert_chunk(chunk, cx, cy, cz);
                        // }
                    }
                }
            }
        }
        self.local_bounds_min = Some(min);
        self.local_bounds_max = Some(max);
    }

    fn insert_chunk(
        &mut self,
        chunk_section: &ChunkSection,
        cx: i32,
        cy: i32,
        cz: i32,
        collider_map: &VoxelColliderMap,
    ) {
        for x in 0..16 {
            for y in 0..16 {
                for z in 0..16 {
                    self.insert_block(
                        x + (cx << CHUNK_SHIFT),
                        y + (cy << CHUNK_SHIFT),
                        z + (cz << CHUNK_SHIFT),
                        &chunk_section.get_block(x, y, z),
                        false,
                        collider_map,
                    );
                }
            }
        }
    }

    fn insert_block(
        &mut self,
        x: i32,
        y: i32,
        z: i32,
        state: &BlockState,
        remove: bool,
        collider_map: &VoxelColliderMap,
    ) {
        let local_min = self.local_bounds_min.unwrap();
        let x = x - local_min.x;
        let y = y - local_min.y;
        let z = z - local_min.z;

        let Some(octree) = &mut self.octree else {
            panic!("No octree!");
        };
        insert_block_octree(collider_map, octree, state, remove, x, y, z);
    }

    fn contains(&self, x: i32, y: i32, z: i32) -> bool {
        if self.local_bounds_min.is_none() || self.local_bounds_max.is_none() {
            return false;
        }

        let local_min = self.local_bounds_min.unwrap();
        let local_max = self.local_bounds_max.unwrap();

        x >= local_min.x
            && x <= local_max.x
            && y >= local_min.y
            && y <= local_max.y
            && z >= local_min.z
            && z <= local_max.z
    }
}

/// Global physics engine state shared across all scenes.
pub(crate) struct PhysicsState {
    /// The integration parameters, updated every time-step
    integration_parameters: IntegrationParameters,

    /// An array of i32 IDs -> block collider entries
    voxel_collider_map: VoxelColliderMap,
}

/// A collision to report to the Java side.
#[derive(Debug, Clone)]
pub(crate) struct ReportedCollision {
    body_a: Option<LevelColliderID>,
    body_b: Option<LevelColliderID>,
    local_point_a: DVec3,
    local_point_b: DVec3,
    local_normal_a: DVec3,
    local_normal_b: DVec3,
    force_amount: f64,
}

static PHYSICS_STATE: OnceLock<RwLock<PhysicsState>> = OnceLock::new();

fn with_handle<F, R>(scene: &PhysicsScene, f: F) -> R
where
    F: FnOnce(&PhysicsScene) -> R,
{
    f(scene)
}

#[inline(always)]
fn get_physics_state() -> std::sync::RwLockReadGuard<'static, PhysicsState> {
    PHYSICS_STATE
        .get()
        .expect("No physics state!")
        .read()
        .unwrap()
}

#[inline(always)]
fn get_physics_state_mut() -> std::sync::RwLockWriteGuard<'static, PhysicsState> {
    PHYSICS_STATE
        .get()
        .expect("No physics state!")
        .write()
        .unwrap()
}

#[inline(always)]
fn get_rigid_body_mut<'a>(
    sim: &'a mut SimulationSceneData,
    sable_data: &SableSceneData,
    id: LevelColliderID,
) -> &'a mut RigidBody {
    let handle = sable_data
        .rigid_bodies
        .get(&id)
        .expect("No rigid body for id");
    &mut sim.rigid_body_set[*handle]
}

#[inline(always)]
fn get_rigid_body<'a>(
    sim: &'a SimulationSceneData,
    sable_data: &SableSceneData,
    id: LevelColliderID,
) -> &'a RigidBody {
    let handle = sable_data
        .rigid_bodies
        .get(&id)
        .expect("No rigid body for id");
    &sim.rigid_body_set[*handle]
}

fn initialize(x: f64, y: f64, z: f64, universal_drag: f64) -> PhysicsScene {
    PHYSICS_STATE.get_or_init(|| {
        RwLock::new(PhysicsState {
            integration_parameters: IntegrationParameters {
                dt: 1.0 / 20.0,

                max_ccd_substeps: 3,
                normalized_prediction_distance: 0.005,

                contact_softness: SpringCoefficients {
                    natural_frequency: 30.0,
                    damping_ratio: 5.0,
                },

                normalized_max_corrective_velocity: 50.0,
                normalized_allowed_linear_error: 0.0025,

                ..IntegrationParameters::default()
            },
            voxel_collider_map: VoxelColliderMap::new(),
        })
    });

    let ground = RigidBodyBuilder::fixed();

    let collider = ColliderBuilder::new(SharedShape::new(LevelCollider::new(None, true)))
        .collision_groups(LEVEL_GROUP)
        .build();

    let sable_data = Arc::new(RwLock::new(SableSceneData {
        main_level_chunks: HashMap::<i64, ChunkSection>::new(),
        octree_chunks: HashMap::<i64, OctreeChunkSection>::new(),
        joint_set: SableJointSet::new(),
        rope_map: RopeMap::default(),
        level_colliders: HashMap::<LevelColliderID, ActiveLevelColliderInfo>::new(),
        rigid_bodies: HashMap::<LevelColliderID, RigidBodyHandle>::new(),
    }));
    let manifold_info_map = Arc::new(SableManifoldInfoMap::default());
    let reported_collisions = Arc::new(ReportedCollisionBuffer::new());
    let dispatcher = SableDispatcher {
        sable_data: Arc::clone(&sable_data),
        manifold_info_map: Arc::clone(&manifold_info_map),
    };

    let mut scene = PhysicsScene {
        sim_data: RwLock::new(SimulationSceneData {
            pipeline: PhysicsPipeline::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::with_query_dispatcher(
                dispatcher.chain(DefaultQueryDispatcher),
            ),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            physics_hooks: SablePhysicsHooks {
                sable_data: Arc::clone(&sable_data),
                manifold_info_map: Arc::clone(&manifold_info_map),
            },
            event_handler: SableEventHandler {
                reported_collisions: Arc::clone(&reported_collisions),
            },
        }),
        sable_data,
        ground_handle: None,
        reported_collisions,
        gravity: Vec3::new(x as Real, y as Real, z as Real),
        universal_drag: universal_drag as Real,
        manifold_info_map,
    };

    {
        let mut sim_data = scene.sim_data.write().unwrap();
        sim_data.collider_set.insert(collider);

        scene.ground_handle = Some(sim_data.rigid_body_set.insert(ground));
    }

    scene
}
#[allow(non_snake_case)]
pub(crate) fn getPose(scene: &PhysicsScene, id: i32, store: &mut [f64; 7]) {
    with_handle(scene, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let sim_data = scene.sim_data.read().unwrap();

        let rb: &RigidBody =
            &sim_data.rigid_body_set[sable_data.rigid_bodies[&(id as LevelColliderID)]];

        let arr: [f64; 7] = [
            rb.translation().x as f64,
            rb.translation().y as f64,
            rb.translation().z as f64,
            rb.rotation().x as f64,
            rb.rotation().y as f64,
            rb.rotation().z as f64,
            rb.rotation().w as f64,
        ];

        store.copy_from_slice(&arr);
    })
}
#[allow(non_snake_case)]
pub(crate) fn setCenterOfMass(scene: &PhysicsScene, id: i32, x: f64, y: f64, z: f64) {
    with_handle(scene, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let info = sable_data
            .level_colliders
            .get_mut(&(id as LevelColliderID))
            .unwrap();
        info.center_of_mass = Some(DVec3::new(x, y, z));
        let mut sim_data = scene.sim_data.write().unwrap();
        update_collider_aabb(&mut sim_data, info);
    })
}
#[allow(non_snake_case)]
pub(crate) fn setLocalBounds(
    scene: &PhysicsScene,
    id: i32,
    min_x: i32,
    min_y: i32,
    min_z: i32,
    max_x: i32,
    max_y: i32,
    max_z: i32,
) {
    with_handle(scene, |scene| {
        let physics_state = get_physics_state();
        let collider_map = &physics_state.voxel_collider_map;
        let mut sable_data = scene.sable_data.write().unwrap();
        let SableSceneData {
            level_colliders,
            main_level_chunks,
            ..
        } = &mut *sable_data;

        let info = level_colliders.get_mut(&(id as LevelColliderID)).unwrap();
        info.set_local_bounds(
            IVec3::new(min_x, min_y, min_z),
            IVec3::new(max_x, max_y, max_z),
            main_level_chunks,
            collider_map,
        );
        let mut sim_data = scene.sim_data.write().unwrap();
        update_collider_aabb(&mut sim_data, info);
    })
}
#[allow(non_snake_case)]
pub(crate) fn createSubLevel(scene: &PhysicsScene, id: i32, pose: [f64; 7]) {
    let pose_arr = pose;

    let quat = Quat::from_xyzw(
        pose_arr[3] as Real,
        pose_arr[4] as Real,
        pose_arr[5] as Real,
        pose_arr[6] as Real,
    );

    let mut rigid_body = RigidBodyBuilder::dynamic()
        .ccd_enabled(true)
        .translation(Vec3::new(
            pose_arr[0] as Real,
            pose_arr[1] as Real,
            pose_arr[2] as Real,
        ))
        .build();
    rigid_body.set_rotation(quat, false);
    let activation_params = rigid_body.activation_mut();
    activation_params.angular_threshold = 0.15;
    activation_params.normalized_linear_threshold = 0.15;

    with_handle(scene, |scene| {
        rigid_body.set_linear_damping(scene.universal_drag);
        rigid_body.set_angular_damping(scene.universal_drag);
        rigid_body.enable_gyroscopic_forces(true);

        let mut sim_data = scene.sim_data.write().unwrap();
        let sim_data = &mut *sim_data;
        let mut sable_data = scene.sable_data.write().unwrap();

        let handle = sim_data.rigid_body_set.insert(rigid_body);

        // make a level collider
        let collider = ColliderBuilder::new(SharedShape::new(LevelCollider::new(
            Some(id as LevelColliderID),
            false,
        )))
        .friction(0.525)
        .active_events(ActiveEvents::CONTACT_FORCE_EVENTS)
        .active_hooks(ActiveHooks::MODIFY_SOLVER_CONTACTS)
        .density(0.0)
        .collision_groups(LEVEL_GROUP)
        .build();

        let collider_handle = sim_data.collider_set.insert_with_parent(
            collider,
            handle,
            &mut sim_data.rigid_body_set,
        );

        sable_data.level_colliders.insert(
            id as LevelColliderID,
            ActiveLevelColliderInfo::new(collider_handle),
        );

        sable_data
            .rigid_bodies
            .insert(id as LevelColliderID, handle);
    })
}
#[allow(non_snake_case)]
pub(crate) fn removeSubLevel(scene: &PhysicsScene, id: i32) {
    with_handle(scene, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();

        sable_data.level_colliders.remove(&(id as LevelColliderID));
        let handle = sable_data
            .rigid_bodies
            .remove(&(id as LevelColliderID))
            .expect("No rigid body for id");

        let mut sim_data = scene.sim_data.write().unwrap();

        let sim_data = &mut *sim_data;
        let rigid_body_set = &mut sim_data.rigid_body_set;
        let island_manager = &mut sim_data.island_manager;
        let collider_set = &mut sim_data.collider_set;
        let impulse_joint_set = &mut sim_data.impulse_joint_set;
        let multibody_joint_set = &mut sim_data.multibody_joint_set;

        rigid_body_set.remove(
            handle,
            island_manager,
            collider_set,
            impulse_joint_set,
            multibody_joint_set,
            true,
        );
    })
}
#[allow(non_snake_case)]
pub(crate) fn addChunk(
    scene: &PhysicsScene,
    x: i32,
    y: i32,
    z: i32,
    data: [i32; 4096],
    global: u8,
    object_id: i32,
) {
    let ints = data;

    let mut blocks = Vec::with_capacity(ints.len());

    for block in ints {
        // split it in half
        let block_collider_id = (block >> 16) as u16;
        let voxel_state_id = (block & 0xFFFF) as u16;

        blocks.push((
            block_collider_id as u32,
            ALL_VOXEL_PHYSICS_STATES[voxel_state_id as usize],
        ));
    }

    let chunk = ChunkSection::new(blocks);

    with_handle(scene, |scene| {
        let physics_state = get_physics_state();
        let collider_map = &physics_state.voxel_collider_map;
        let mut sable_data = scene.sable_data.write().unwrap();
        let SableSceneData {
            main_level_chunks,
            level_colliders,
            octree_chunks,
            ..
        } = &mut *sable_data;

        main_level_chunks.insert(pack_section_pos(x, y, z), chunk);

        let chunk = main_level_chunks.get(&pack_section_pos(x, y, z)).unwrap();
        if global == 0 {
            if object_id != -1 {
                let body = level_colliders
                    .get_mut(&(object_id as LevelColliderID))
                    .unwrap();

                body.insert_chunk(chunk, x, y, z, collider_map);
            }
        } else {
            for bx in 0..16 {
                for by in 0..16 {
                    for bz in 0..16 {
                        let block = chunk.get_block(bx, by, bz);
                        let x = bx + (x << CHUNK_SHIFT);
                        let y = by + (y << CHUNK_SHIFT);
                        let z = bz + (z << CHUNK_SHIFT);

                        // insert into level octree
                        let ox = x >> OCTREE_CHUNK_SHIFT;
                        let oy = y >> OCTREE_CHUNK_SHIFT;
                        let oz = z >> OCTREE_CHUNK_SHIFT;

                        let mut octree_chunk = octree_chunks.get_mut(&pack_section_pos(ox, oy, oz));

                        if octree_chunk.is_none() {
                            octree_chunks
                                .insert(pack_section_pos(ox, oy, oz), OctreeChunkSection::new());
                            octree_chunk = octree_chunks.get_mut(&pack_section_pos(ox, oy, oz));
                        }

                        let Some(octree_chunk) = octree_chunk else {
                            panic!("No octree chunk!")
                        };

                        if block.0 == 0 {
                            insert_block_octree(
                                collider_map,
                                &mut octree_chunk.liquid_octree,
                                &block,
                                false,
                                x & (OCTREE_CHUNK_SIZE - 1),
                                y & (OCTREE_CHUNK_SIZE - 1),
                                z & (OCTREE_CHUNK_SIZE - 1),
                            );
                            insert_block_octree(
                                collider_map,
                                &mut octree_chunk.octree,
                                &block,
                                false,
                                x & (OCTREE_CHUNK_SIZE - 1),
                                y & (OCTREE_CHUNK_SIZE - 1),
                                z & (OCTREE_CHUNK_SIZE - 1),
                            );
                        } else {
                            if collider_map.voxel_colliders[(block.0 - 1) as usize]
                                .as_ref()
                                .unwrap()
                                .is_fluid
                            {
                                insert_block_octree(
                                    collider_map,
                                    &mut octree_chunk.liquid_octree,
                                    &block,
                                    false,
                                    x & (OCTREE_CHUNK_SIZE - 1),
                                    y & (OCTREE_CHUNK_SIZE - 1),
                                    z & (OCTREE_CHUNK_SIZE - 1),
                                );
                            } else {
                                insert_block_octree(
                                    collider_map,
                                    &mut octree_chunk.octree,
                                    &block,
                                    false,
                                    x & (OCTREE_CHUNK_SIZE - 1),
                                    y & (OCTREE_CHUNK_SIZE - 1),
                                    z & (OCTREE_CHUNK_SIZE - 1),
                                );
                            }
                        }
                    }
                }
            }
        }
    })
}
#[allow(non_snake_case)]
pub(crate) fn removeChunk(scene: &PhysicsScene, x: i32, y: i32, z: i32, global: u8) {
    with_handle(scene, |scene| {
        let physics_state = get_physics_state();
        let collider_map = &physics_state.voxel_collider_map;
        let mut sable_data = scene.sable_data.write().unwrap();

        sable_data
            .main_level_chunks
            .remove(&pack_section_pos(x, y, z));

        if global > 0 {
            let octree_chunk = sable_data.octree_chunks.get_mut(&pack_section_pos(
                (x << CHUNK_SHIFT) >> OCTREE_CHUNK_SHIFT,
                (y << CHUNK_SHIFT) >> OCTREE_CHUNK_SHIFT,
                (z << CHUNK_SHIFT) >> OCTREE_CHUNK_SHIFT,
            ));

            if let Some(octree_chunk) = octree_chunk {
                for bx in 0..16 {
                    for by in 0..16 {
                        for bz in 0..16 {
                            let x = bx + (x << CHUNK_SHIFT);
                            let y = by + (y << CHUNK_SHIFT);
                            let z = bz + (z << CHUNK_SHIFT);

                            insert_block_octree(
                                collider_map,
                                &mut octree_chunk.octree,
                                &(0, VoxelPhysicsState::Empty),
                                true,
                                x & (OCTREE_CHUNK_SIZE - 1),
                                y & (OCTREE_CHUNK_SIZE - 1),
                                z & (OCTREE_CHUNK_SIZE - 1),
                            );
                            insert_block_octree(
                                collider_map,
                                &mut octree_chunk.liquid_octree,
                                &(0, VoxelPhysicsState::Empty),
                                true,
                                x & (OCTREE_CHUNK_SIZE - 1),
                                y & (OCTREE_CHUNK_SIZE - 1),
                                z & (OCTREE_CHUNK_SIZE - 1),
                            );
                        }
                    }
                }

                if octree_chunk.octree.buffer[0] == 0 && octree_chunk.liquid_octree.buffer[0] == 0 {
                    sable_data.octree_chunks.remove(&pack_section_pos(
                        (x << CHUNK_SHIFT) >> OCTREE_CHUNK_SHIFT,
                        (y << CHUNK_SHIFT) >> OCTREE_CHUNK_SHIFT,
                        (z << CHUNK_SHIFT) >> OCTREE_CHUNK_SHIFT,
                    ));
                }
            }
        }
    })
}
#[allow(non_snake_case)]
pub(crate) fn change_block_data(data: &RwLock<SableSceneData>, x: i32, y: i32, z: i32, block: i32) {
    let block_collider_id = (block >> 16) as u16;
    let voxel_state_id = (block & 0xFFFF) as u16;

    {
        let physics_state = get_physics_state();
        let collider_map = &physics_state.voxel_collider_map;
        let mut sable_data = data.write().unwrap();
        let SableSceneData {
            main_level_chunks,
            level_colliders,
            octree_chunks,
            ..
        } = &mut *sable_data;

        let chunk = main_level_chunks.get_mut(&pack_section_pos(x >> 4, y >> 4, z >> 4));

        if let Some(chunk) = chunk {
            let block_state = (
                block_collider_id as u32,
                ALL_VOXEL_PHYSICS_STATES[voxel_state_id as usize],
            );

            chunk.set_block(x & 15, y & 15, z & 15, block_state);

            let mut any = false;
            for (_, sable_body) in level_colliders.iter_mut() {
                if sable_body.contains(x, y, z) {
                    sable_body.insert_block(x, y, z, &block_state, true, collider_map);
                    any = true;
                    break;
                }
            }

            if !any {
                // insert into level octree
                let ox = x >> OCTREE_CHUNK_SHIFT;
                let oy = y >> OCTREE_CHUNK_SHIFT;
                let oz = z >> OCTREE_CHUNK_SHIFT;

                let mut octree_chunk = octree_chunks.get_mut(&pack_section_pos(ox, oy, oz));

                if octree_chunk.is_none() {
                    octree_chunks.insert(pack_section_pos(ox, oy, oz), OctreeChunkSection::new());
                    octree_chunk = octree_chunks.get_mut(&pack_section_pos(ox, oy, oz));
                }

                let Some(octree_chunk) = octree_chunk else {
                    panic!("No octree chunk!")
                };

                if block_collider_id == 0 {
                    insert_block_octree(
                        collider_map,
                        &mut octree_chunk.octree,
                        &block_state,
                        true,
                        x & (OCTREE_CHUNK_SIZE - 1),
                        y & (OCTREE_CHUNK_SIZE - 1),
                        z & (OCTREE_CHUNK_SIZE - 1),
                    );
                    insert_block_octree(
                        collider_map,
                        &mut octree_chunk.liquid_octree,
                        &block_state,
                        true,
                        x & (OCTREE_CHUNK_SIZE - 1),
                        y & (OCTREE_CHUNK_SIZE - 1),
                        z & (OCTREE_CHUNK_SIZE - 1),
                    );
                } else {
                    if collider_map
                        .voxel_colliders
                        .get(block_collider_id as usize - 1)
                        .unwrap()
                        .as_ref()
                        .unwrap()
                        .is_fluid
                    {
                        insert_block_octree(
                            collider_map,
                            &mut octree_chunk.liquid_octree,
                            &block_state,
                            false,
                            x & (OCTREE_CHUNK_SIZE - 1),
                            y & (OCTREE_CHUNK_SIZE - 1),
                            z & (OCTREE_CHUNK_SIZE - 1),
                        );
                    } else {
                        insert_block_octree(
                            collider_map,
                            &mut octree_chunk.octree,
                            &block_state,
                            false,
                            x & (OCTREE_CHUNK_SIZE - 1),
                            y & (OCTREE_CHUNK_SIZE - 1),
                            z & (OCTREE_CHUNK_SIZE - 1),
                        );
                    }
                }
            }
        }
    }
}
#[allow(non_snake_case)]
pub(crate) fn setMassProperties(
    scene: &PhysicsScene,
    id: i32,
    mass: f64,
    center_of_mass: [f64; 3],
    inertia: [f64; 9],
) {
    let _com = center_of_mass;

    let inertia_arr = inertia;

    let inertia_tensor = Mat3::from_cols(
        Vec3::new(
            inertia_arr[0] as Real,
            inertia_arr[1] as Real,
            inertia_arr[2] as Real,
        ),
        Vec3::new(
            inertia_arr[3] as Real,
            inertia_arr[4] as Real,
            inertia_arr[5] as Real,
        ),
        Vec3::new(
            inertia_arr[6] as Real,
            inertia_arr[7] as Real,
            inertia_arr[8] as Real,
        ),
    );

    with_handle(scene, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let rb = &mut sim_data.rigid_body_set[sable_data.rigid_bodies[&(id as LevelColliderID)]];

        rb.set_additional_mass_properties(
            rapier3d::prelude::MassProperties::with_inertia_matrix(
                Vec3::ZERO,
                mass as Real,
                inertia_tensor.into(),
            ),
            true,
        );
    })
}
#[allow(non_snake_case)]
pub(crate) fn teleportObject(
    scene: &PhysicsScene,
    id: i32,
    x: f64,
    y: f64,
    z: f64,
    i: f64,
    j: f64,
    k: f64,
    r: f64,
) {
    with_handle(scene, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let rb = &mut sim_data.rigid_body_set[sable_data.rigid_bodies[&(id as LevelColliderID)]];

        let mut pose = *rb.position();
        pose.translation = Vec3::new(x as Real, y as Real, z as Real);
        pose.rotation = Quat::from_xyzw(i as Real, j as Real, k as Real, r as Real);
        rb.set_position(pose, true);
    })
}
#[allow(non_snake_case)]
pub(crate) fn wakeUpObject(scene: &PhysicsScene, id: i32) {
    with_handle(scene, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();
        let rb = &mut sim_data.rigid_body_set[sable_data.rigid_bodies[&(id as LevelColliderID)]];
        rb.wake_up(true);
    })
}
#[allow(non_snake_case)]
pub(crate) fn addLinearAngularVelocities(
    scene: &PhysicsScene,
    id: i32,
    linear_x: f64,
    linear_y: f64,
    linear_z: f64,
    angular_x: f64,
    angular_y: f64,
    angular_z: f64,
    wake_up: u8,
) {
    with_handle(scene, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();
        let rb = get_rigid_body_mut(&mut sim_data, &sable_data, id as LevelColliderID);

        if wake_up == 0 && rb.is_sleeping() {
            return;
        }

        rb.set_linvel(
            rb.linvel() + Vec3::new(linear_x as Real, linear_y as Real, linear_z as Real),
            wake_up > 0,
        );
        rb.set_angvel(
            rb.angvel() + Vec3::new(angular_x as Real, angular_y as Real, angular_z as Real),
            wake_up > 0,
        );
    })
}
#[allow(non_snake_case)]
pub(crate) fn clearCollisions(scene: &PhysicsScene) -> Vec<f64> {
    let arr: Vec<f64> = with_handle(scene, |scene| {
        let mut reported = scene.reported_collisions.borrow_mut();

        let max_collisions = 100;

        reported.truncate(max_collisions);
        let mut arr: Vec<f64> = Vec::with_capacity(reported.len() * 15);

        for collision in reported.iter() {
            let body_a = if let Some(id) = collision.body_a {
                id as f64
            } else {
                -1.0
            };

            let body_b = if let Some(id) = collision.body_b {
                id as f64
            } else {
                -1.0
            };

            arr.push(body_a);
            arr.push(body_b);
            arr.push(collision.force_amount as f64);
            arr.push(collision.local_normal_a.x as f64);
            arr.push(collision.local_normal_a.y as f64);
            arr.push(collision.local_normal_a.z as f64);
            arr.push(collision.local_normal_b.x as f64);
            arr.push(collision.local_normal_b.y as f64);
            arr.push(collision.local_normal_b.z as f64);
            arr.push(collision.local_point_a.x as f64);
            arr.push(collision.local_point_a.y as f64);
            arr.push(collision.local_point_a.z as f64);
            arr.push(collision.local_point_b.x as f64);
            arr.push(collision.local_point_b.y as f64);
            arr.push(collision.local_point_b.z as f64);
        }

        reported.clear();

        arr
    });

    arr
}
#[allow(non_snake_case)]
pub(crate) fn applyForce(
    scene: &PhysicsScene,
    id: i32,
    x: f64,
    y: f64,
    z: f64,
    fx: f64,
    fy: f64,
    fz: f64,
    wake_up: u8,
) {
    with_handle(scene, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let body = sable_data
            .rigid_bodies
            .get(&(id as LevelColliderID))
            .unwrap();
        let rb = &mut sim_data.rigid_body_set[*body];

        if wake_up == 0 && rb.is_sleeping() {
            return;
        }

        let force: Vec3 = rb
            .rotation()
            .mul_vec3(Vec3::new(fx as Real, fy as Real, fz as Real));
        let force_pos = rb
            .position()
            .transform_point(Vec3::new(x as Real, y as Real, z as Real));

        rb.apply_impulse(force, wake_up > 0);

        let torque_impulse = (force_pos - rb.position().translation).cross(force);
        rb.apply_torque_impulse(torque_impulse, wake_up > 0);
    })
}
#[allow(non_snake_case)]
pub(crate) fn applyForceAndTorque(
    scene: &PhysicsScene,
    id: i32,
    fx: f64,
    fy: f64,
    fz: f64,
    tx: f64,
    ty: f64,
    tz: f64,
    wake_up: u8,
) {
    with_handle(scene, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let body = sable_data
            .rigid_bodies
            .get(&(id as LevelColliderID))
            .unwrap();
        let rb = &mut sim_data.rigid_body_set[*body];

        if wake_up == 0 && rb.is_sleeping() {
            return;
        }

        let force: Vec3 = rb
            .rotation()
            .mul_vec3(Vec3::new(fx as Real, fy as Real, fz as Real));
        rb.apply_impulse(force, wake_up > 0);

        let torque: Vec3 = rb
            .rotation()
            .mul_vec3(Vec3::new(tx as Real, ty as Real, tz as Real));
        rb.apply_torque_impulse(torque, wake_up > 0);
    })
}
#[allow(non_snake_case)]
pub(crate) fn getLinearVelocity(scene: &PhysicsScene, id: i32, store: &mut [f64; 3]) {
    with_handle(scene, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let sim_data = scene.sim_data.read().unwrap();

        let body = sable_data
            .rigid_bodies
            .get(&(id as LevelColliderID))
            .unwrap();
        let rb = &sim_data.rigid_body_set[*body];

        let vel = rb.linvel();

        store.copy_from_slice(&[vel.x as f64, vel.y as f64, vel.z as f64]);
    })
}
#[allow(non_snake_case)]
pub(crate) fn getAngularVelocity(scene: &PhysicsScene, id: i32, store: &mut [f64; 3]) {
    with_handle(scene, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let sim_data = scene.sim_data.read().unwrap();

        let body = sable_data
            .rigid_bodies
            .get(&(id as LevelColliderID))
            .unwrap();
        let rb = &sim_data.rigid_body_set[*body];

        let vel = rb.angvel();

        store.copy_from_slice(&[vel.x as f64, vel.y as f64, vel.z as f64]);
    })
}
fn insert_block_octree(
    collider_map: &VoxelColliderMap,
    octree: &mut SubLevelOctree,
    state: &BlockState,
    remove: bool,
    x: i32,
    y: i32,
    z: i32,
) {
    let block_collider_id = state.0;
    let block_collider = if block_collider_id > 0 {
        Some(
            collider_map
                .voxel_colliders
                .get(block_collider_id as usize - 1)
                .unwrap(),
        )
    } else {
        None
    };
    let voxel_state = state.1;

    let solid = voxel_state != Interior
        && voxel_state != VoxelPhysicsState::Empty
        && (block_collider_id > 0
            && !block_collider
                .unwrap()
                .as_ref()
                .unwrap()
                .collision_boxes
                .is_empty());

    if remove && !solid {
        octree.insert(x, y, z, -1);
    }

    if solid {
        octree.insert(x, y, z, block_collider_id as i32);
    }
}

fn step(scene: &PhysicsScene, dt: f64) {
    get_physics_state_mut().integration_parameters.dt = dt as Real;
    rope::tick(scene);
    joints::tick(scene);
    scene.manifold_info_map.clear();
    let mut sim = scene.sim_data.write().unwrap();
    let sim = &mut *sim;
    let state = get_physics_state();
    sim.pipeline.step(
        scene.gravity,
        &state.integration_parameters,
        &mut sim.island_manager,
        &mut sim.broad_phase,
        &mut sim.narrow_phase,
        &mut sim.rigid_body_set,
        &mut sim.collider_set,
        &mut sim.impulse_joint_set,
        &mut sim.multibody_joint_set,
        &mut sim.ccd_solver,
        &sim.physics_hooks,
        &sim.event_handler,
    );
}
fn tick(scene: &PhysicsScene) {
    rope::tick(scene);
    joints::tick(scene);
    compute_buoyancy(scene);
}
mod boundary;
pub use boundary::*;
