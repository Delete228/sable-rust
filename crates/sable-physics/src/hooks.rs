// Derived from Sable 9b1a3d1555d35db9fb7b1103701e7490f0a968b9.
// PolyForm Shield 1.0.0; see ../LICENSE-SABLE.md and provenance/origin.json.
use marten::Real;
use marten::level::VoxelColliderData;
use rapier3d::geometry::{Collider, SolverContact};
use rapier3d::glamx::DVec3;
use rapier3d::math::{Pose, Vec3};
use rapier3d::pipeline::{ContactModificationContext, PhysicsHooks};
use std::sync::{Arc, RwLock};

use crate::collider::LevelCollider;
use crate::scene::{LevelColliderID, SableManifoldInfoMap, SableSceneData};

#[derive(Clone)]
pub struct SablePhysicsHooks {
    pub sable_data: Arc<RwLock<SableSceneData>>,
    pub manifold_info_map: Arc<SableManifoldInfoMap>,
}

impl PhysicsHooks for SablePhysicsHooks {
    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        if !VoxelColliderData::needs_hooks(*context.user_data) {
            return;
        }

        let mut remove = false;
        for contact in context.solver_contacts.iter_mut() {
            let Some(collider_a) = context.colliders.get(context.collider1) else {
                panic!("No collider A!");
            };

            let Some(collider_b) = context.colliders.get(context.collider2) else {
                panic!("No collider B!");
            };

            let level_collider_a = collider_a.shape().as_shape::<LevelCollider>();
            let level_collider_b = collider_b.shape().as_shape::<LevelCollider>();

            if level_collider_a.is_none() && level_collider_b.is_none() {
                continue;
            }

            let mut tangent_velo = Vec3::ZERO;

            let mut velocity = 0.0;
            let mut friction_multiplier = 1.0;

            if let Some(handle) = context.rigid_body1 {
                let mut velo_1 = context
                    .bodies
                    .get(handle)
                    .unwrap()
                    .velocity_at_point(contact.point);
                velo_1 += self.get_fake_velocity(contact, collider_a, level_collider_a);
                velocity += velo_1.dot(*context.normal);
            }
            if let Some(handle) = context.rigid_body2 {
                let mut velo_2 = context
                    .bodies
                    .get(handle)
                    .unwrap()
                    .velocity_at_point(contact.point);
                velo_2 += self.get_fake_velocity(contact, collider_a, level_collider_a);
                velocity -= velo_2.dot(*context.normal);
            }

            velocity = velocity.abs();

            let mut restitution: Real = 0.0;

            let manifold_index = (*context.user_data >> 1) as usize;

            if let Some(level_collider_a) = level_collider_a {
                let (add_velo, remove_a, friction_mult, block_restitution) = self
                    .handle_block_params(
                        collider_a.position(),
                        collider_a,
                        Some(level_collider_a),
                        &contact.point,
                        velocity,
                        manifold_index,
                        true,
                    );
                tangent_velo += add_velo;
                remove |= remove_a;
                friction_multiplier *= friction_mult;
                restitution = restitution.max(block_restitution);
            }

            if let Some(level_collider_b) = level_collider_b {
                let (add_velo, remove_b, friction_mult, block_restitution) = self
                    .handle_block_params(
                        collider_b.position(),
                        collider_b,
                        Some(level_collider_b),
                        &contact.point,
                        velocity,
                        manifold_index,
                        false,
                    );
                tangent_velo -= add_velo;
                remove |= remove_b;
                friction_multiplier *= friction_mult;
                restitution = restitution.max(block_restitution);
            }

            tangent_velo -= *context.normal * tangent_velo.dot(*context.normal);

            contact.tangent_velocity = tangent_velo;
            contact.friction *= friction_multiplier;
            contact.restitution = contact.restitution.max(restitution);
        }

        if remove {
            context.solver_contacts.clear()
        }
    }
}

impl SablePhysicsHooks {
    fn get_fake_velocity(
        &self,
        contact: &SolverContact,
        collider_a: &Collider,
        level_collider_a: Option<&LevelCollider>,
    ) -> Vec3 {
        if let Some(level_collider_a) = level_collider_a
            && level_collider_a.id.is_some()
        {
            let sable_data = self.sable_data.read().unwrap();

            let collider_info =
                &sable_data.level_colliders[&(level_collider_a.id.unwrap() as LevelColliderID)];

            if let Some(fake_velo) = collider_info.fake_velocities {
                let transform = collider_a.position();
                return transform.transform_vector(fake_velo.velocity_at_point(
                    transform.inverse_transform_point(contact.point),
                    Vec3::ZERO,
                ));
            };
        }
        Vec3::ZERO
    }

    fn handle_block_params(
        &self,
        isometry: &Pose,
        _collider: &Collider,
        level_collider: Option<&LevelCollider>,
        global_point: &Vec3,
        velocity: Real,
        manifold_index: usize,
        body_a: bool,
    ) -> (Vec3, bool, Real, Real) {
        let state = crate::get_physics_state();

        let (tangent_velo, center_of_mass, skip_contact_events) = {
            let sable_data = self.sable_data.read().unwrap();
            let collider_info =
                level_collider.and_then(|lc| lc.id.map(|id| &sable_data.level_colliders[&(id)]));

            let mut tangent_velo = Vec3::ZERO;
            if let Some(fake_velo) = collider_info.and_then(|info| info.fake_velocities) {
                tangent_velo += isometry.transform_vector(fake_velo.velocity_at_point(
                    isometry.inverse_transform_point(*global_point),
                    Vec3::ZERO,
                ));
            }

            let center_of_mass = collider_info.map_or(DVec3::ZERO, |b| b.center_of_mass.unwrap());
            let skip_contact_events = collider_info.is_some_and(|info| info.has_own_chunks());

            (tangent_velo, center_of_mass, skip_contact_events)
        };

        let Some(manifold_info) = self.manifold_info_map.list.get(&manifold_index) else {
            return (tangent_velo, false, 1.0, 0.0);
        };

        let (block_coord, block_id, other_block_coord, has_other_block) = if body_a {
            (
                manifold_info.pos_a,
                manifold_info.col_a as u32,
                manifold_info.pos_b,
                manifold_info.col_b != 0,
            )
        } else {
            (
                manifold_info.pos_b,
                manifold_info.col_b as u32,
                manifold_info.pos_a,
                manifold_info.col_a != 0,
            )
        };

        let local = isometry.inverse_transform_point(*global_point);
        let block_coord_d: DVec3 = local.as_dvec3() + center_of_mass;

        if block_id == 0 {
            return (tangent_velo, false, 1.0, 0.0);
        }

        let voxel_collider_data = &state
            .voxel_collider_map
            .voxel_colliders
            .get((block_id - 1) as usize);
        let mut friction_multiplier = 1.0;
        let mut restitution = 0.0;

        if voxel_collider_data.is_none() {
            return (tangent_velo, false, friction_multiplier, restitution);
        }

        let voxel_collider_data = voxel_collider_data.unwrap().as_ref().unwrap();
        friction_multiplier *= voxel_collider_data.friction;
        restitution = voxel_collider_data.restitution;

        let Some(contact_events) = voxel_collider_data.contact_events.as_ref() else {
            return (tangent_velo, false, friction_multiplier, restitution);
        };

        if skip_contact_events {
            return (tangent_velo, false, friction_multiplier, restitution);
        }

        let response = marten::level::call_contact(
            contact_events.as_ref(),
            marten::level::ContactRequest {
                block: [block_coord.x, block_coord.y, block_coord.z],
                other: [
                    other_block_coord.x,
                    other_block_coord.y,
                    other_block_coord.z,
                ],
                point: [block_coord_d.x, block_coord_d.y, block_coord_d.z],
                speed: velocity as f64,
                has_other: has_other_block,
            },
            &ContactWorldView(&self.sable_data),
        );
        let velo_arr = [
            response.tangent[0],
            response.tangent[1],
            response.tangent[2],
            if response.reject { 1.0 } else { 0.0 },
        ];

        let velo = Vec3::new(
            velo_arr[0] as Real,
            velo_arr[1] as Real,
            velo_arr[2] as Real,
        );

        (
            tangent_velo + isometry.transform_vector(velo),
            velo_arr[3] > 0.0,
            friction_multiplier,
            restitution,
        )
    }
}

struct ContactWorldView<'a>(&'a RwLock<SableSceneData>);
impl marten::level::ContactWorld for ContactWorldView<'_> {
    fn block(&self, p: [i32; 3]) -> Option<(u32, u8)> {
        use crate::scene::ChunkAccess;
        if !crate::boundary::block_in_domain(p) {
            return None;
        }
        let data = self.0.read().unwrap();
        let c = data.get_chunk(p[0] >> 4, p[1] >> 4, p[2] >> 4)?;
        let b = c.get_block(p[0] & 15, p[1] & 15, p[2] & 15);
        Some((b.0, b.1 as u8))
    }
    fn remove_block(&self, p: [i32; 3]) -> Result<(), marten::level::ContactWorldError> {
        if !crate::boundary::block_in_domain(p) {
            return Err(marten::level::ContactWorldError::InvalidCoordinate);
        }
        if self.block(p).is_none() {
            return Err(marten::level::ContactWorldError::Untracked);
        }
        crate::change_block_data(self.0, p[0], p[1], p[2], 0);
        Ok(())
    }
}
