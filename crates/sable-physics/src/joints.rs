// Derived from Sable 9b1a3d1555d35db9fb7b1103701e7490f0a968b9.
// PolyForm Shield 1.0.0; see ../LICENSE-SABLE.md and provenance/origin.json.
use crate::config::{JOINT_SPRING_DAMPING_RATIO, JOINT_SPRING_FREQUENCY};
use crate::scene::{LevelColliderID, PhysicsScene};
use crate::with_handle;
use marten::Real;
use rapier3d::dynamics::{
    GenericJointBuilder, JointAxesMask, JointAxis, RevoluteJointBuilder, SpringCoefficients,
};
use rapier3d::glamx::{DVec3, Quat};
use rapier3d::math::Vec3;
use rapier3d::prelude::{FixedJointBuilder, ImpulseJointHandle};
use std::collections::HashMap;

type SableJointHandle = i64;
type RapierJointHandle = ImpulseJointHandle;

struct SubLevelJoint {
    id_a: Option<LevelColliderID>,
    id_b: Option<LevelColliderID>,

    pos_a: DVec3,
    pos_b: DVec3,
    normal_a: DVec3,
    normal_b: DVec3,

    rotation_a: Option<Quat>,
    rotation_b: Option<Quat>,

    handle: RapierJointHandle,

    fixed: bool,
    contacts_enabled: bool,
}

pub struct SableJointSet {
    joints: HashMap<SableJointHandle, SubLevelJoint>,
}

impl SableJointSet {
    #[must_use]
    pub fn new() -> Self {
        Self {
            joints: HashMap::new(),
        }
    }
}

pub fn tick(scene: &PhysicsScene) {
    let mut sable_data = scene.sable_data.write().unwrap();
    let mut sim = scene.sim_data.write().unwrap();

    // filter the joints
    sable_data
        .joint_set
        .joints
        .retain(|_handle, joint| sim.impulse_joint_set.contains(joint.handle));

    // update every joint
    for (_handle, joint) in sable_data.joint_set.joints.iter() {
        let impulse_joint = sim.impulse_joint_set.get_mut(joint.handle, false).unwrap();
        impulse_joint.data.contacts_enabled = joint.contacts_enabled;
        if !joint.fixed && joint.rotation_a.is_none() {
            impulse_joint.data.set_local_axis1(joint.normal_a.as_vec3());
        }

        let center_of_mass_1 = if let Some(id_a) = joint.id_a
            && let Some(rb_a) = sable_data.level_colliders.get(&id_a)
        {
            rb_a.center_of_mass.unwrap()
        } else {
            DVec3::ZERO
        };

        let local_anchor_1 = joint.pos_a - center_of_mass_1;
        impulse_joint
            .data
            .set_local_anchor1(local_anchor_1.as_vec3());
        if !joint.fixed && joint.rotation_b.is_none() {
            impulse_joint.data.set_local_axis2(joint.normal_b.as_vec3());
        }

        let center_of_mass_2 = if let Some(id_b) = joint.id_b
            && let Some(rb_b) = sable_data.level_colliders.get(&id_b)
        {
            rb_b.center_of_mass.unwrap()
        } else {
            DVec3::ZERO
        };

        let local_anchor_2 = joint.pos_b - center_of_mass_2;
        impulse_joint
            .data
            .set_local_anchor2(local_anchor_2.as_vec3());

        if let Some(rotation_a) = joint.rotation_a {
            impulse_joint.data.local_frame1.rotation = rotation_a;
        }
        if let Some(rotation_b) = joint.rotation_b {
            impulse_joint.data.local_frame2.rotation = rotation_b;
        }
    }
}

const AXES: [JointAxis; 6] = [
    JointAxis::LinX,
    JointAxis::LinY,
    JointAxis::LinZ,
    JointAxis::AngX,
    JointAxis::AngY,
    JointAxis::AngZ,
];

#[allow(non_snake_case)]
pub(crate) fn setConstraintMotor(
    scene: &PhysicsScene,
    joint_id: i64,
    axis: i32,
    target_pos: f64,
    stiffness: f64,
    damping: f64,
    has_max_force: u8,
    max_force: f64,
) {
    with_handle(scene, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let Some(joint) = sable_data.joint_set.joints.get(&joint_id) else {
            return;
        };

        let data = &mut sim_data
            .impulse_joint_set
            .get_mut(joint.handle, false)
            .unwrap()
            .data;
        data.set_motor_position(
            AXES[axis as usize],
            target_pos as Real,
            stiffness as Real,
            damping as Real,
        );

        if has_max_force > 0 {
            data.motors[axis as usize].max_force = max_force as Real
        }
    })
}
#[allow(non_snake_case)]
pub(crate) fn setConstraintLimit(
    scene: &PhysicsScene,
    joint_id: i64,
    axis: i32,
    min: f64,
    max: f64,
) {
    with_handle(scene, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let Some(joint) = sable_data.joint_set.joints.get(&joint_id) else {
            return;
        };

        let data = &mut sim_data
            .impulse_joint_set
            .get_mut(joint.handle, false)
            .unwrap()
            .data;

        data.set_limits(AXES[axis as usize], [min as Real, max as Real]);
    })
}
#[allow(non_snake_case)]
pub(crate) fn lockConstraintAxes(scene: &PhysicsScene, joint_id: i64, mask: i8) {
    with_handle(scene, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let Some(joint) = sable_data.joint_set.joints.get(&joint_id) else {
            return;
        };

        let data = &mut sim_data
            .impulse_joint_set
            .get_mut(joint.handle, false)
            .unwrap()
            .data;

        data.lock_axes(JointAxesMask::from_bits(mask as u8).expect("Invalid mask!"));
    })
}
#[allow(non_snake_case)]
pub(crate) fn isConstraintValid(scene: &PhysicsScene, joint_id: i64) -> u8 {
    with_handle(scene, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        if sable_data.joint_set.joints.contains_key(&joint_id) {
            1
        } else {
            0
        }
    })
}
#[allow(non_snake_case)]
pub(crate) fn getConstraintImpulses(scene: &PhysicsScene, joint_id: i64, store: &mut [f64; 6]) {
    with_handle(scene, |scene| {
        let sable_data = scene.sable_data.read().unwrap();
        let sim_data = scene.sim_data.read().unwrap();

        let joint = sable_data.joint_set.joints.get(&joint_id).unwrap();
        let impulse_joint = sim_data.impulse_joint_set.get(joint.handle).unwrap();
        let impulses = impulse_joint.impulses;

        let arr: [f64; 6] = [
            impulses[0] as f64,
            impulses[1] as f64,
            impulses[2] as f64,
            impulses[3] as f64,
            impulses[4] as f64,
            impulses[5] as f64,
        ];

        store.copy_from_slice(&arr);
    })
}
#[allow(non_snake_case)]
pub(crate) fn setConstraintContactsEnabled(scene: &PhysicsScene, joint_id: i64, enabled: u8) {
    with_handle(scene, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let Some(joint) = sable_data.joint_set.joints.get_mut(&joint_id) else {
            return;
        };

        joint.contacts_enabled = enabled > 0;
    })
}
#[allow(non_snake_case)]
pub(crate) fn removeConstraint(scene: &PhysicsScene, joint_id: i64) {
    with_handle(scene, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();
        if let Some(joint) = sable_data.joint_set.joints.remove(&joint_id) {
            sim_data.impulse_joint_set.remove(joint.handle, true);
        }
    })
}
#[allow(non_snake_case)]
pub(crate) fn addRotaryConstraint(
    scene: &PhysicsScene,
    id_a: i32,
    id_b: i32,
    local_x_a: f64,
    local_y_a: f64,
    local_z_a: f64,
    local_x_b: f64,
    local_y_b: f64,
    local_z_b: f64,
    axis_x_a: f64,
    axis_y_a: f64,
    axis_z_a: f64,
    axis_x_b: f64,
    axis_y_b: f64,
    axis_z_b: f64,
) -> SableJointHandle {
    with_handle(scene, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let rb_a = if id_a == -1 {
            scene.ground_handle.unwrap()
        } else {
            sable_data.rigid_bodies[&(id_a as LevelColliderID)]
        };

        let rb_b = if id_b == -1 {
            scene.ground_handle.unwrap()
        } else {
            sable_data.rigid_bodies[&(id_b as LevelColliderID)]
        };

        let revolute = RevoluteJointBuilder::new(
            Vec3::new(axis_x_a as Real, axis_y_a as Real, axis_z_a as Real).normalize(),
        )
        .local_anchor1(Vec3::ZERO)
        .local_anchor2(Vec3::ZERO)
        .softness(SpringCoefficients::new(
            JOINT_SPRING_FREQUENCY,
            JOINT_SPRING_DAMPING_RATIO,
        ));

        let handle = sim_data
            .impulse_joint_set
            .insert(rb_a, rb_b, revolute.build(), true);

        let (index, generation) = handle.0.into_raw_parts();
        let handle_long: SableJointHandle = index as i64 | (generation as i64) << 32;

        sable_data.joint_set.joints.insert(
            handle_long,
            SubLevelJoint {
                id_a: if id_a == -1 {
                    None
                } else {
                    Some(id_a as LevelColliderID)
                },
                id_b: if id_b == -1 {
                    None
                } else {
                    Some(id_b as LevelColliderID)
                },

                pos_a: DVec3::new(local_x_a, local_y_a, local_z_a),
                pos_b: DVec3::new(local_x_b, local_y_b, local_z_b),

                normal_a: DVec3::new(axis_x_a, axis_y_a, axis_z_a),
                normal_b: DVec3::new(axis_x_b, axis_y_b, axis_z_b),

                rotation_a: None,
                rotation_b: None,

                handle,

                fixed: false,
                contacts_enabled: true,
            },
        );

        handle_long
    })
}
#[allow(non_snake_case)]
pub(crate) fn addFixedConstraint(
    scene: &PhysicsScene,
    id_a: i32,
    id_b: i32,
    local_x_a: f64,
    local_y_a: f64,
    local_z_a: f64,
    local_x_b: f64,
    local_y_b: f64,
    local_z_b: f64,
    local_q_x: f64,
    local_q_y: f64,
    local_q_z: f64,
    local_q_w: f64,
) -> SableJointHandle {
    with_handle(scene, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let rb_a = if id_a == -1 {
            scene.ground_handle.unwrap()
        } else {
            sable_data.rigid_bodies[&(id_a as LevelColliderID)]
        };

        let rb_b = if id_b == -1 {
            scene.ground_handle.unwrap()
        } else {
            sable_data.rigid_bodies[&(id_b as LevelColliderID)]
        };

        let quat = Quat::from_xyzw(
            local_q_x as Real,
            local_q_y as Real,
            local_q_z as Real,
            local_q_w as Real,
        );
        let mut revolute = FixedJointBuilder::new()
            .local_anchor1(Vec3::ZERO)
            .local_anchor2(Vec3::ZERO)
            .softness(SpringCoefficients::new(
                JOINT_SPRING_FREQUENCY,
                JOINT_SPRING_DAMPING_RATIO,
            ));
        revolute.0.data.local_frame1.rotation = quat;

        let handle = sim_data
            .impulse_joint_set
            .insert(rb_a, rb_b, revolute.build(), true);

        let (index, generation) = handle.0.into_raw_parts();
        let handle_long: SableJointHandle = index as i64 | (generation as i64) << 32;

        sable_data.joint_set.joints.insert(
            handle_long,
            SubLevelJoint {
                id_a: if id_a == -1 {
                    None
                } else {
                    Some(id_a as LevelColliderID)
                },
                id_b: if id_b == -1 {
                    None
                } else {
                    Some(id_b as LevelColliderID)
                },

                pos_a: DVec3::new(local_x_a, local_y_a, local_z_a),
                pos_b: DVec3::new(local_x_b, local_y_b, local_z_b),

                normal_a: DVec3::new(0.0, 0.0, 0.0),
                normal_b: DVec3::new(0.0, 0.0, 0.0),

                rotation_a: None,
                rotation_b: None,

                handle,

                fixed: true,
                contacts_enabled: false,
            },
        );

        handle_long
    })
}
#[allow(non_snake_case)]
pub(crate) fn addFreeConstraint(
    scene: &PhysicsScene,
    id_a: i32,
    id_b: i32,
    local_x_a: f64,
    local_y_a: f64,
    local_z_a: f64,
    local_x_b: f64,
    local_y_b: f64,
    local_z_b: f64,
    local_q_x: f64,
    local_q_y: f64,
    local_q_z: f64,
    local_q_w: f64,
) -> SableJointHandle {
    with_handle(scene, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let rb_a = if id_a == -1 {
            scene.ground_handle.unwrap()
        } else {
            sable_data.rigid_bodies[&(id_a as LevelColliderID)]
        };

        let rb_b = if id_b == -1 {
            scene.ground_handle.unwrap()
        } else {
            sable_data.rigid_bodies[&(id_b as LevelColliderID)]
        };

        let mut joint = GenericJointBuilder::new(JointAxesMask::empty()).softness(
            SpringCoefficients::new(JOINT_SPRING_FREQUENCY, JOINT_SPRING_DAMPING_RATIO),
        );

        let quat = Quat::from_xyzw(
            local_q_x as Real,
            local_q_y as Real,
            local_q_z as Real,
            local_q_w as Real,
        );
        joint.0.local_frame1.rotation = quat;

        let handle = sim_data
            .impulse_joint_set
            .insert(rb_a, rb_b, joint.build(), true);

        let (index, generation) = handle.0.into_raw_parts();
        let handle_long: SableJointHandle = index as i64 | (generation as i64) << 32;

        sable_data.joint_set.joints.insert(
            handle_long,
            SubLevelJoint {
                id_a: if id_a == -1 {
                    None
                } else {
                    Some(id_a as LevelColliderID)
                },
                id_b: if id_b == -1 {
                    None
                } else {
                    Some(id_b as LevelColliderID)
                },

                pos_a: DVec3::new(local_x_a, local_y_a, local_z_a),
                pos_b: DVec3::new(local_x_b, local_y_b, local_z_b),

                normal_a: DVec3::new(0.0, 0.0, 0.0),
                normal_b: DVec3::new(0.0, 0.0, 0.0),

                rotation_a: None,
                rotation_b: None,

                handle,

                fixed: true,
                contacts_enabled: true,
            },
        );

        handle_long
    })
}
#[allow(non_snake_case)]
pub(crate) fn addGenericConstraint(
    scene: &PhysicsScene,
    id_a: i32,
    id_b: i32,
    local_x_a: f64,
    local_y_a: f64,
    local_z_a: f64,
    local_q_x_a: f64,
    local_q_y_a: f64,
    local_q_z_a: f64,
    local_q_w_a: f64,
    local_x_b: f64,
    local_y_b: f64,
    local_z_b: f64,
    local_q_x_b: f64,
    local_q_y_b: f64,
    local_q_z_b: f64,
    local_q_w_b: f64,
    locked_axes_mask: i32,
) -> SableJointHandle {
    with_handle(scene, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let mut sim_data = scene.sim_data.write().unwrap();

        let rb_a = if id_a == -1 {
            scene.ground_handle.unwrap()
        } else {
            sable_data.rigid_bodies[&(id_a as LevelColliderID)]
        };

        let rb_b = if id_b == -1 {
            scene.ground_handle.unwrap()
        } else {
            sable_data.rigid_bodies[&(id_b as LevelColliderID)]
        };

        let locked_axes = JointAxesMask::from_bits_truncate(locked_axes_mask as u8);

        let rotation_a = Quat::from_xyzw(
            local_q_x_a as Real,
            local_q_y_a as Real,
            local_q_z_a as Real,
            local_q_w_a as Real,
        );
        let rotation_b = Quat::from_xyzw(
            local_q_x_b as Real,
            local_q_y_b as Real,
            local_q_z_b as Real,
            local_q_w_b as Real,
        );

        let mut joint = GenericJointBuilder::new(locked_axes).softness(SpringCoefficients::new(
            JOINT_SPRING_FREQUENCY,
            JOINT_SPRING_DAMPING_RATIO,
        ));
        joint.0.local_frame1.rotation = rotation_a;
        joint.0.local_frame2.rotation = rotation_b;

        let handle = sim_data
            .impulse_joint_set
            .insert(rb_a, rb_b, joint.build(), true);

        let (index, generation) = handle.0.into_raw_parts();
        let handle_long: SableJointHandle = index as i64 | (generation as i64) << 32;

        sable_data.joint_set.joints.insert(
            handle_long,
            SubLevelJoint {
                id_a: if id_a == -1 {
                    None
                } else {
                    Some(id_a as LevelColliderID)
                },
                id_b: if id_b == -1 {
                    None
                } else {
                    Some(id_b as LevelColliderID)
                },

                pos_a: DVec3::new(local_x_a as f64, local_y_a as f64, local_z_a as f64),
                pos_b: DVec3::new(local_x_b as f64, local_y_b as f64, local_z_b as f64),

                normal_a: DVec3::ZERO,
                normal_b: DVec3::ZERO,

                rotation_a: Some(rotation_a),
                rotation_b: Some(rotation_b),

                handle,

                fixed: true,
                contacts_enabled: true,
            },
        );

        handle_long
    })
}
#[allow(non_snake_case)]
pub(crate) fn setConstraintFrame(
    scene: &PhysicsScene,
    joint_id: i64,
    side: i32,
    local_x: f64,
    local_y: f64,
    local_z: f64,
    local_q_x: f64,
    local_q_y: f64,
    local_q_z: f64,
    local_q_w: f64,
) {
    with_handle(scene, |scene| {
        let mut sable_data = scene.sable_data.write().unwrap();
        let Some(joint) = sable_data.joint_set.joints.get_mut(&joint_id) else {
            return;
        };

        let position = DVec3::new(local_x as f64, local_y as f64, local_z as f64);
        let rotation = Quat::from_xyzw(
            local_q_x as Real,
            local_q_y as Real,
            local_q_z as Real,
            local_q_w as Real,
        );

        match side {
            0 => {
                joint.pos_a = position;
                joint.rotation_a = Some(rotation);
            }
            1 => {
                joint.pos_b = position;
                joint.rotation_b = Some(rotation);
            }
            _ => panic!("Invalid constraint frame side: {}", side),
        }
    })
}

pub(crate) fn backend_valid(scene: &PhysicsScene, id: i64) -> bool {
    let data = scene.sable_data.read().unwrap();
    let sim = scene.sim_data.read().unwrap();
    data.joint_set
        .joints
        .get(&id)
        .is_some_and(|j| sim.impulse_joint_set.contains(j.handle))
}
