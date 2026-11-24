// Derived from Sable 9b1a3d1555d35db9fb7b1103701e7490f0a968b9.
// PolyForm Shield 1.0.0; see ../LICENSE-SABLE.md and provenance/origin.json.
use std::collections::HashMap;

use marten::Real;
use rapier3d::dynamics::{GenericJointBuilder, JointAxis, RigidBodyBuilder, SpringCoefficients};
use rapier3d::geometry::{ColliderBuilder, SharedShape};
use rapier3d::glamx::DVec3;
use rapier3d::math::Vec3;
use rapier3d::prelude::{
    ImpulseJointHandle, ImpulseJointSet, JointAxesMask, RigidBodyHandle, RopeJointBuilder,
};

use crate::config::{JOINT_SPRING_DAMPING_RATIO, JOINT_SPRING_FREQUENCY};
use crate::groups::ROPE_GROUP;
use crate::scene::{LevelColliderID, PhysicsScene, SableSceneData, SimulationSceneData};
use crate::with_handle;

const MIN_BOUND_STIFFNESS: Real = 150.0;
const MIN_BOUND_DAMPING: Real = 10.0;

struct RopeAttachment {
    joint: ImpulseJointHandle,
    sub_level_id: Option<LevelColliderID>,
    location: DVec3,
}

struct RopeStrand {
    points: Vec<RigidBodyHandle>,
    joints: Vec<(ImpulseJointHandle, ImpulseJointHandle)>,

    point_radius: Real,
    first_joint_length: Real,

    start_attachment: Option<RopeAttachment>,
    end_attachment: Option<RopeAttachment>,
}
#[derive(Default)]
pub struct RopeMap {
    counting_id: usize,
    ropes: HashMap<usize, RopeStrand>,
}

pub fn tick(scene: &PhysicsScene) {
    let mut sable_data = scene.sable_data.write().unwrap();
    let mut sim = scene.sim_data.write().unwrap();

    let mut dead_start_attachments = Vec::new();
    let mut dead_end_attachments = Vec::new();

    for (id, rope) in sable_data.rope_map.ropes.iter() {
        if let Some(attachment) = &rope.start_attachment {
            if !sim.impulse_joint_set.contains(attachment.joint) {
                dead_start_attachments.push(id.clone());
            } else {
                let local_anchor = attachment.location
                    - if let Some(id_b) = attachment.sub_level_id {
                        let rb_b = &sable_data.level_colliders[&id_b];
                        rb_b.center_of_mass.unwrap()
                    } else {
                        DVec3::ZERO
                    };

                let impulse_joint = sim
                    .impulse_joint_set
                    .get_mut(attachment.joint, false)
                    .unwrap();
                impulse_joint.data.set_local_anchor1(local_anchor.as_vec3());
            }
        }

        if let Some(attachment) = &rope.end_attachment {
            if !sim.impulse_joint_set.contains(attachment.joint) {
                dead_end_attachments.push(id.clone());
            } else {
                let local_anchor = attachment.location
                    - if let Some(id_b) = attachment.sub_level_id {
                        let rb_b = &sable_data.level_colliders[&id_b];
                        rb_b.center_of_mass.unwrap()
                    } else {
                        DVec3::ZERO
                    };

                let impulse_joint = sim
                    .impulse_joint_set
                    .get_mut(attachment.joint, false)
                    .unwrap();
                impulse_joint.data.set_local_anchor1(local_anchor.as_vec3());
            }
        }
    }

    for id in dead_start_attachments {
        if let Some(rope) = sable_data.rope_map.ropes.get_mut(&id) {
            rope.start_attachment = None;
        }
    }

    for id in dead_end_attachments {
        if let Some(rope) = sable_data.rope_map.ropes.get_mut(&id) {
            rope.end_attachment = None;
        }
    }
}
