#![cfg(feature = "native-physics")]

//! Real accepted-Engine checks; payloads here are test data, not live containers.
use sable_core::{mass::MassRegistry, physics::PhysicsOwner, *};

fn block(state: &str, value: u8) -> Block {
    Block {
        state: state.into(),
        block_entity: Some(Payload {
            format: "test-only".into(),
            bytes: vec![value],
        }),
        extension: Payload::default(),
    }
}

#[test]
fn payload_revisions_preserve_native_body_motion_and_allow_capture_but_geometry_does_not() {
    let mut registry = Registry::new(Config::default()).unwrap();
    let mut level = Level::new(Id([1; 16]));
    for p in [[0, 0, 0], [16, 0, 0], [32, 0, 0]] {
        level
            .contents
            .blocks
            .insert(p, block("minecraft:blackstone", 0));
    }
    let h = registry.insert(level).unwrap();
    let masses = MassRegistry::pinned_full_blocks();
    let mut owner = PhysicsOwner::new([0.; 3]).unwrap();
    owner.sync(&registry, &masses).unwrap();
    let original = owner.body(h.id()).unwrap();
    owner
        .engine
        .add_velocity(original, [1., 2., 3.], [0., 0., 0.5], true)
        .unwrap();
    owner.engine.tick(owner.scene).unwrap();
    owner.engine.step(owner.scene, 0.025).unwrap();
    let pose = owner.engine.pose(original).unwrap();
    let velocity = owner.engine.velocity(original).unwrap();
    for value in 1..=20 {
        registry
            .edit(h, [16, 0, 0], Some(block("minecraft:blackstone", value)))
            .unwrap();
        // The prior capture boundary must reject the newly edited revision.
        assert!(owner.capture(&mut registry, &masses).is_err());
        owner.sync(&registry, &masses).unwrap();
        assert_eq!(owner.body(h.id()), Some(original));
        assert_eq!(owner.counts(), (1, 3));
        assert_eq!(owner.engine.pose(original).unwrap(), pose);
        assert_eq!(owner.engine.velocity(original).unwrap(), velocity);
        owner.capture(&mut registry, &masses).unwrap();
        let saved = persistence::encode(&registry).unwrap();
        let restored = persistence::decode(&saved, Default::default()).unwrap();
        let contents = &restored
            .get(restored.handle(h.id()).unwrap())
            .unwrap()
            .contents;
        assert_eq!(
            contents.blocks[&[16, 0, 0]]
                .block_entity
                .as_ref()
                .unwrap()
                .bytes,
            vec![value]
        );
    }
    registry
        .edit(h, [16, 0, 0], Some(block("minecraft:bedrock", 20)))
        .unwrap();
    owner.sync(&registry, &masses).unwrap();
    assert_ne!(owner.body(h.id()), Some(original));
    assert!(owner.engine.pose(original).is_err());
    let changed = owner.body(h.id()).unwrap();
    registry.edit(h, [16, 0, 0], None).unwrap();
    owner.sync(&registry, &masses).unwrap();
    assert_ne!(owner.body(h.id()), Some(changed));
    assert_eq!(owner.counts(), (1, 2));
    owner.capture(&mut registry, &masses).unwrap();
}

#[test]
fn same_geometry_reload_with_changed_motion_still_updates_native_velocity() {
    let mut registry = Registry::new(Config::default()).unwrap();
    let mut level = Level::new(Id([2; 16]));
    level
        .contents
        .blocks
        .insert([0; 3], block("minecraft:blackstone", 1));
    level.motion.linear = [4., 0., 0.];
    let h = registry.insert(level).unwrap();
    let masses = MassRegistry::pinned_full_blocks();
    let mut owner = PhysicsOwner::new([0.; 3]).unwrap();
    owner.sync(&registry, &masses).unwrap();
    let old = owner.body(h.id()).unwrap();
    registry.unload(h).unwrap();
    struct Ready;
    impl persistence::ResidencyHost for Ready {
        fn chunk_ready(&self, _: i32, _: i32) -> bool {
            true
        }
    }
    assert!(
        persistence::reload_with(
            &mut registry,
            h,
            &Ready,
            100,
            persistence::VelocityPolicy::JavaFields {
                retained_on_load: 0.5
            },
        )
        .unwrap()
    );
    // No empty sync between unload and reload: geometry alone is insufficient.
    owner.sync(&registry, &masses).unwrap();
    assert_ne!(owner.body(h.id()), Some(old));
    assert_eq!(
        owner.engine.velocity(owner.body(h.id()).unwrap()).unwrap()[0],
        2.
    );
    owner.capture(&mut registry, &masses).unwrap();
}
