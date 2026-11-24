#![cfg(feature = "native-physics")]

//! Real-Engine regressions for R1: a valid batch may permute native plot IDs.
use sable_core::{mass::MassRegistry, persistence, physics::PhysicsOwner, *};
use std::collections::BTreeMap;

fn id(n: u8) -> Id {
    Id([n; 16])
}

fn level(n: u8) -> Level {
    let mut level = Level::new(id(n));
    for position in [[1, 0, 1], [16 + i32::from(n % 3) * 16, 0, 1]] {
        level.contents.blocks.insert(
            position,
            Block {
                state: "minecraft:blackstone".into(),
                block_entity: Some(Payload {
                    format: "r1-inventory".into(),
                    bytes: vec![n, 64, 2, 1],
                }),
                extension: Payload::default(),
            },
        );
    }
    level.motion.pose.position = [f64::from(n) * 100., 20., 0.];
    level.motion.linear = [f64::from(n) / 10., 0., 0.];
    level
}

fn assert_realized(owner: &mut PhysicsOwner, registry: &Registry, masses: &MassRegistry) {
    registry.validate().unwrap();
    let live: Vec<_> = registry
        .levels()
        .filter(|(_, state)| *state == Residency::Live)
        .collect();
    assert_eq!(
        owner.counts(),
        (
            live.len(),
            live.iter()
                .map(|(level, _)| level.contents.sections().len())
                .sum()
        )
    );
    for (level, _) in live {
        let body = owner.body(level.id).unwrap();
        let expected = level
            .motion
            .pose
            .transform(masses.compute(&level.contents).unwrap().center);
        let actual = owner.engine.pose(body).unwrap();
        let velocity = owner.engine.velocity(body).unwrap();
        for axis in 0..3 {
            assert!((actual[axis] - expected[axis]).abs() < 1e-4);
            assert!((velocity[axis] - level.motion.linear[axis]).abs() < 1e-6);
        }
    }
    // Idempotent repeated synchronization keeps all actual native handles.
    let bodies: Vec<_> = registry
        .levels()
        .filter_map(|(level, _)| owner.body(level.id).map(|body| (level.id, body)))
        .collect();
    for _ in 0..3 {
        owner.sync(registry, masses).unwrap();
        for &(id, body) in &bodies {
            assert_eq!(owner.body(id), Some(body));
            assert!(owner.engine.pose(body).is_ok());
        }
    }
}

fn reinsert_batch(
    owner: &mut PhysicsOwner,
    registry: &mut Registry,
    masses: &MassRegistry,
    insertion_order: &[u8],
) {
    let contents: BTreeMap<_, _> = registry
        .levels()
        .map(|(level, _)| (level.id, level.contents.clone()))
        .collect();
    let all_old: BTreeMap<_, _> = registry
        .levels()
        .map(|(level, _)| (level.id, owner.body(level.id).unwrap()))
        .collect();
    let mut saved = BTreeMap::new();
    for &n in insertion_order {
        let handle = registry.handle(id(n)).unwrap();
        let contents = registry.get(handle).unwrap().contents.clone();
        let level = registry.delete(handle).unwrap();
        assert_eq!(level.contents, contents);
        saved.insert(n, level);
    }
    // No physics call occurs between delete and reinsert.
    for &n in insertion_order {
        registry.insert(saved.remove(&n).unwrap()).unwrap();
    }
    owner.sync(registry, masses).unwrap();
    for (id, old) in all_old {
        if insertion_order.iter().any(|&n| id == self::id(n)) {
            assert!(owner.engine.pose(old).is_err());
            assert_ne!(owner.body(id), Some(old));
        } else {
            assert_eq!(owner.body(id), Some(old));
        }
    }
    for (id, contents) in contents {
        assert_eq!(
            registry.get(registry.handle(id).unwrap()).unwrap().contents,
            contents
        );
    }
    assert_realized(owner, registry, masses);
}

#[test]
fn batched_slot_cycles_restore_and_refusal_recover_without_intermediate_empty_sync() {
    let masses = MassRegistry::pinned_full_blocks();
    let mut registry = Registry::new(Config::default()).unwrap();
    for n in 1..=7 {
        registry.insert(level(n)).unwrap();
    }
    let mut owner = PhysicsOwner::new([0., -9.81, 0.]).unwrap();
    owner.sync(&registry, &masses).unwrap();
    assert_realized(&mut owner, &registry, &masses);

    reinsert_batch(&mut owner, &mut registry, &masses, &[2, 1]);
    assert_eq!(
        registry.get(registry.handle(id(1)).unwrap()).unwrap().plot,
        128
    );
    assert_eq!(
        registry.get(registry.handle(id(2)).unwrap()).unwrap().plot,
        0
    );
    reinsert_batch(&mut owner, &mut registry, &masses, &[3, 2, 1]);
    reinsert_batch(&mut owner, &mut registry, &masses, &[5, 4, 7, 6]);
    reinsert_batch(&mut owner, &mut registry, &masses, &[7, 6, 5, 4, 3, 2, 1]);

    let saved = persistence::encode(&registry).unwrap();
    let decoded = persistence::decode(&saved, Default::default()).unwrap();
    let old: Vec<_> = registry
        .levels()
        .map(|(level, _)| owner.body(level.id).unwrap())
        .collect();
    let mut restored: Vec<_> = decoded
        .levels()
        .map(|(level, state)| (level.clone(), state))
        .collect();
    let plots: Vec<_> = restored.iter().map(|(level, _)| level.plot).collect();
    for (i, (level, _)) in restored.iter_mut().enumerate() {
        level.plot = plots[(i + 1) % plots.len()];
    }
    registry = Registry::restore(registry.config().clone(), restored).unwrap();

    // Refusal is preflight-only even when the valid portion permutes slots.
    let first = registry.handle(id(1)).unwrap();
    let ordinary = registry.get(first).unwrap().contents.blocks[&[1, 0, 1]].clone();
    let mut unknown = ordinary.clone();
    unknown.state = "r1:unresolved-state".into();
    registry.edit(first, [1, 0, 1], Some(unknown)).unwrap();
    assert!(owner.sync(&registry, &masses).is_err());
    assert_eq!(owner.counts(), (7, 14));
    for body in &old {
        assert!(owner.engine.pose(*body).is_ok());
    }
    registry.edit(first, [1, 0, 1], Some(ordinary)).unwrap();
    owner.sync(&registry, &masses).unwrap();
    for body in old {
        assert!(owner.engine.pose(body).is_err());
    }
    assert_realized(&mut owner, &registry, &masses);

    owner.engine.tick(owner.scene).unwrap();
    owner.engine.step(owner.scene, 0.025).unwrap();
    owner.capture(&mut registry, &masses).unwrap();
    assert_realized(&mut owner, &registry, &masses);

    let old: Vec<_> = registry
        .levels()
        .map(|(level, _)| owner.body(level.id).unwrap())
        .collect();
    for n in 1..=7 {
        registry.delete(registry.handle(id(n)).unwrap()).unwrap();
    }
    owner.sync(&registry, &masses).unwrap();
    assert_eq!(owner.counts(), (0, 0));
    for body in old {
        assert!(owner.engine.pose(body).is_err());
    }
    // Reusing every freed native ID succeeds, proving teardown is retryable.
    for n in 1..=7 {
        registry.insert(level(n)).unwrap();
    }
    owner.sync(&registry, &masses).unwrap();
    assert_realized(&mut owner, &registry, &masses);
}
