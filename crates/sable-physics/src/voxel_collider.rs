// Derived from Sable 9b1a3d1555d35db9fb7b1103701e7490f0a968b9.
// PolyForm Shield 1.0.0; see ../LICENSE-SABLE.md and provenance/origin.json.
use std::collections::HashMap;

use marten::level::VoxelColliderData;
use rapier3d::glamx::IVec3;

/// The physics data of a blockstate
#[derive(Debug)]
pub struct VoxelColliderMap {
    pub(crate) voxel_colliders: Vec<Option<VoxelColliderData>>,
    dynamic_colliders: HashMap<IVec3, Option<VoxelColliderData>>,
}

impl VoxelColliderMap {
    pub fn new() -> Self {
        Self {
            voxel_colliders: Vec::new(),
            dynamic_colliders: HashMap::new(),
        }
    }

    pub fn get(&self, index: usize, block_pos: IVec3) -> Option<&VoxelColliderData> {
        let collider = &self.voxel_colliders[index];

        if collider.is_some() && collider.as_ref().unwrap().dynamic {
            let dynamic_collider = self.dynamic_colliders.get(&block_pos);

            if let Some(data) = dynamic_collider {
                return data.as_ref();
            }
        }

        collider.as_ref()
    }
}
