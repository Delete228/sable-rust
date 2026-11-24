// Derived from Sable 9b1a3d1555d35db9fb7b1103701e7490f0a968b9.
// PolyForm Shield 1.0.0; see ../LICENSE-SABLE.md and provenance/origin.json.
use marten::Real;
use rapier3d::dynamics::RigidBodyBuilder;
use rapier3d::geometry::{ColliderBuilder, SharedShape};
use rapier3d::glamx::Quat;
use rapier3d::math::Vec3;

use crate::scene::{LevelColliderID, PhysicsScene};
use crate::with_handle;

#[allow(non_snake_case)]
pub(crate) fn createBox(
    scene: &PhysicsScene,
    id: i32,
    mass: f64,
    half_extent_x: f64,
    half_extent_y: f64,
    half_extent_z: f64,
    pose: [f64; 7],
) {
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

    with_handle(scene, |scene| {
        let mut sim_data = scene.sim_data.write().unwrap();
        let sim_data = &mut *sim_data;
        let mut sable_data = scene.sable_data.write().unwrap();

        let handle = sim_data.rigid_body_set.insert(rigid_body);

        // make a level collider
        let collider = ColliderBuilder::new(SharedShape::cuboid(
            half_extent_x as Real,
            half_extent_y as Real,
            half_extent_z as Real,
        ))
        .mass(mass as Real)
        .friction(0.45)
        .build();

        sim_data
            .collider_set
            .insert_with_parent(collider, handle, &mut sim_data.rigid_body_set);

        sable_data
            .rigid_bodies
            .insert(id as LevelColliderID, handle);
    })
}
#[allow(non_snake_case)]
pub(crate) fn removeBox(scene: &PhysicsScene, id: i32) {
    with_handle(scene, |scene| {
        let mut sim_data = scene.sim_data.write().unwrap();
        let sim_data = &mut *sim_data;
        let mut sable_data = scene.sable_data.write().unwrap();

        let handle = sable_data.rigid_bodies[&(id as LevelColliderID)];
        sim_data.rigid_body_set.remove(
            handle,
            &mut sim_data.island_manager,
            &mut sim_data.collider_set,
            &mut sim_data.impulse_joint_set,
            &mut sim_data.multibody_joint_set,
            true,
        );

        sable_data.rigid_bodies.remove(&(id as LevelColliderID));
    })
}
