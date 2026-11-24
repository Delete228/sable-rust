#![cfg(feature = "native-physics")]

//! Independently authored adverse regressions for the physics realization owner.
use sable_core::{
    mass::{MassRegistry, Properties},
    physics::PhysicsOwner,
    *,
};

fn properties(mass: f64, max_x: f64) -> Properties {
    Properties {
        solid: true,
        full: true,
        mass,
        center: [0.5; 3],
        inertia: None,
        boxes: vec![[0., 0., 0., max_x, 1., 1.]],
    }
}
fn block(state: &str) -> Block {
    Block {
        state: state.into(),
        block_entity: None,
        extension: Payload::default(),
    }
}

#[test]
fn invalid_material_does_not_leak_body_policy_changes_invalidate_and_capture_is_atomic() {
    let mut registry = Registry::new(Config::default()).unwrap();
    let mut level = Level::new(Id([1; 16]));
    level
        .contents
        .blocks
        .insert([1, 0, 1], block("minecraft:blackstone"));
    let first = registry.insert(level).unwrap();
    let mut owner = PhysicsOwner::new([0., -9.81, 0.]).unwrap();
    let mut masses = MassRegistry::pinned_full_blocks();
    owner.sync(&registry, &masses).unwrap();
    let old = owner.body(first.id()).unwrap();

    let mut bad = masses.clone();
    bad.register("review:new".into(), properties(2., 1e100))
        .unwrap();
    registry
        .edit(first, [1, 0, 1], Some(block("review:new")))
        .unwrap();
    assert!(owner.sync(&registry, &bad).is_err());
    assert_eq!(owner.counts(), (1, 1));
    assert!(owner.engine.pose(old).is_ok());
    masses
        .register("review:new".into(), properties(2., 1.))
        .unwrap();
    owner.sync(&registry, &masses).unwrap();
    assert_eq!(owner.counts(), (1, 1));
    assert!(owner.engine.pose(old).is_err());

    let before_policy = owner.body(first.id()).unwrap();
    masses
        .register("review:new".into(), properties(7., 0.5))
        .unwrap();
    let before_policy_capture = sable_core::persistence::encode(&registry).unwrap();
    assert!(owner.capture(&mut registry, &masses).is_err());
    assert_eq!(
        sable_core::persistence::encode(&registry).unwrap(),
        before_policy_capture
    );
    owner.sync(&registry, &masses).unwrap();
    assert!(owner.engine.pose(before_policy).is_err());
    assert_eq!(owner.counts(), (1, 1));

    let mut second_level = Level::new(Id([2; 16]));
    second_level
        .contents
        .blocks
        .insert([1, 0, 1], block("minecraft:blackstone"));
    second_level.motion.pose.position = [10., 0., 0.];
    let second = registry.insert(second_level).unwrap();
    owner.sync(&registry, &masses).unwrap();
    owner.engine.tick(owner.scene).unwrap();
    owner.engine.step(owner.scene, 0.05).unwrap();
    registry
        .edit(second, [1, 0, 1], Some(block("minecraft:blackstone")))
        .unwrap();
    let before_capture = sable_core::persistence::encode(&registry).unwrap();
    assert!(owner.capture(&mut registry, &masses).is_err());
    assert_eq!(
        sable_core::persistence::encode(&registry).unwrap(),
        before_capture
    );
}
