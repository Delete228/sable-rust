// Derived from Sable 9b1a3d1555d35db9fb7b1103701e7490f0a968b9.
// PolyForm Shield 1.0.0; see ../../LICENSE-SABLE.md and provenance/origin.json.
//! A sparse voxel world.

use crate::octree::SubLevelOctree;

/// log_2 of the size of a chunk
pub const CHUNK_SHIFT: u8 = 4;

/// The size of a chunk in blocks
pub const CHUNK_SIZE: u8 = 1 << CHUNK_SHIFT;

/// Bitwise mask to apply to a block coordinate to get the chunk-relative coordinate of the block
pub const CHUNK_MASK: i32 = (CHUNK_SIZE - 1) as i32;

/// (Block Physics ID, Voxel Physics State)
pub type BlockState = (u32, VoxelPhysicsState);

/// A 16x16x16 voxel chunk.
/// Blocks are stored in xzy order. (x fastest changing)
#[derive(Debug)]
pub struct ChunkSection {
    blocks: Vec<BlockState>,
}

impl ChunkSection {
    /// Creates a new chunk section with the given blocks.
    pub fn new(blocks: Vec<BlockState>) -> Self {
        // Panic if the block count is invalid
        if blocks.len() != (CHUNK_SIZE as usize).pow(3) {
            panic!("Invalid block count: {}", blocks.len());
        }

        Self { blocks }
    }

    /// Computes the index of a coordinate inside the chunk.
    ///
    /// # Safety
    /// This method assumes that the coordinate is > than 0 and < than `CHUNK_SIZE` on all axes.
    #[inline(always)]
    fn get_index(&self, x: i32, y: i32, z: i32) -> usize {
        assert!(
            (0..16).contains(&x) && (0..16).contains(&y) && (0..16).contains(&z),
            "chunk coordinate outside 0..16"
        );
        (x + (z << 4) + (y << 8)) as usize
    }

    /// Sets the block at the given coordinate.
    ///
    /// # Safety
    /// This method assumes that the coordinate is > than 0 and < than `CHUNK_SIZE` on all axes.
    /// Invalid coordinates panic; checked indexing prevents undefined behavior.
    pub fn set_block(&mut self, x: i32, y: i32, z: i32, state: BlockState) {
        let index = self.get_index(x, y, z);
        self.blocks[index] = state;
    }

    /// Gets the block at the given coordinate.
    ///
    /// # Safety
    /// This method assumes that the coordinate is >= than 0 and < than `CHUNK_SIZE` on all axes.
    /// Invalid coordinates panic; checked indexing prevents undefined behavior.
    pub fn get_block(&self, x: i32, y: i32, z: i32) -> BlockState {
        let index = self.get_index(x, y, z);
        self.blocks[index]
    }
}

/// The physics data of a blockstate
#[derive(Debug)]
pub struct VoxelColliderData {
    /// Collision boxes within the 0-1 voxel space.
    /// Formatted [min_x, min_y, min_z, max_x, max_y, max_z]
    pub collision_boxes: Vec<(f32, f32, f32, f32, f32, f32)>,

    /// If this should be treated as a fluid for buoyancy
    pub is_fluid: bool,

    /// The friction multiplier
    pub friction: f32,
    pub volume: f32,
    pub restitution: f32,

    /// If this block has special contact behavior
    pub contact_events: Option<std::sync::Arc<dyn ContactPolicy>>,

    pub dynamic: bool,
}

pub const NEEDS_HOOKS_USER_DATA: u32 = 1;
pub const NO_HOOKS_USER_DATA: u32 = 0;

impl VoxelColliderData {
    pub fn get_user_data(&self) -> u32 {
        if self.contact_events.is_some() || self.friction != 1.0 || self.restitution != 0.0 {
            NEEDS_HOOKS_USER_DATA
        } else {
            NO_HOOKS_USER_DATA
        }
    }

    pub fn needs_hooks(data: u32) -> bool {
        data & NEEDS_HOOKS_USER_DATA > 0
    }
}

pub const OCTREE_CHUNK_SHIFT: i32 = 6;
pub const OCTREE_CHUNK_SIZE: i32 = 1 << OCTREE_CHUNK_SHIFT;

pub struct OctreeChunkSection {
    pub octree: SubLevelOctree,
    pub liquid_octree: SubLevelOctree,
}

impl Default for OctreeChunkSection {
    fn default() -> Self {
        Self::new()
    }
}

impl OctreeChunkSection {
    pub fn new() -> Self {
        Self {
            octree: SubLevelOctree::new(OCTREE_CHUNK_SHIFT),
            liquid_octree: SubLevelOctree::new(OCTREE_CHUNK_SHIFT),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoxelPhysicsState {
    Empty,
    Face,
    Edge,
    Corner,
    Interior,
}

pub const ALL_VOXEL_PHYSICS_STATES: [VoxelPhysicsState; 5] = [
    VoxelPhysicsState::Empty,
    VoxelPhysicsState::Face,
    VoxelPhysicsState::Edge,
    VoxelPhysicsState::Corner,
    VoxelPhysicsState::Interior,
];

/// Synchronous host decision at the original contact-hook call site.
#[derive(Debug, Clone, Copy)]
pub struct ContactRequest {
    pub block: [i32; 3],
    pub other: [i32; 3],
    pub point: [f64; 3],
    pub speed: f64,
    pub has_other: bool,
}
#[derive(Debug, Clone, Copy, Default)]
pub struct ContactResponse {
    pub tangent: [f64; 3],
    pub reject: bool,
}
pub trait ContactPolicy: std::fmt::Debug + Send + Sync {
    fn on_contact(&self, input: ContactRequest, world: &dyn ContactWorld) -> ContactResponse;
}

thread_local! { static IN_CONTACT:std::cell::Cell<bool> = const {std::cell::Cell::new(false)}; }
pub fn in_contact_callback() -> bool {
    IN_CONTACT.with(|v| v.get())
}
pub fn call_contact(
    policy: &dyn ContactPolicy,
    input: ContactRequest,
    world: &dyn ContactWorld,
) -> ContactResponse {
    struct Reset;
    impl Drop for Reset {
        fn drop(&mut self) {
            IN_CONTACT.with(|v| v.set(false));
        }
    }
    IN_CONTACT.with(|v| assert!(!v.replace(true), "reentrant contact service"));
    let _reset = Reset;
    policy.on_contact(input, world)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactWorldError {
    Untracked,
    InvalidCoordinate,
}
/// A call-local service. Edits execute before the contact decision returns.
pub trait ContactWorld {
    fn block(&self, position: [i32; 3]) -> Option<(u32, u8)>;
    fn remove_block(&self, position: [i32; 3]) -> Result<(), ContactWorldError>;
}
