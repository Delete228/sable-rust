#![cfg(feature = "native-physics")]

use sable_core::{
    mass::MassRegistry, physics::PhysicsOwner, split::SplitPlan, transfer::PreservePayloads, *,
};
use std::collections::BTreeSet;
fn id(n: u8) -> Id {
    Id([n; 16])
}
fn b() -> Block {
    Block {
        state: "minecraft:blackstone".into(),
        block_entity: None,
        extension: Payload::default(),
    }
}
#[test]
fn real_engine_assembly_edit_split_unload_reload_delete_and_motion() {
    let masses = MassRegistry::pinned_full_blocks();
    let mut r = Registry::new(Config::default()).unwrap();
    let mut l = Level::new(id(1));
    for x in [15, 16, 31, 32] {
        l.contents.blocks.insert([x, 10, 16], b());
    }
    l.motion.pose.position = [100., 40., -20.];
    l.motion.pose.pivot = [23.5, 10.5, 16.5];
    l.motion.linear = [0.5, 0., 0.];
    l.motion.angular = [0., 0., 0.2];
    let parent = r.insert(l).unwrap();
    let mut p = PhysicsOwner::new([0., -9.81, 0.]).unwrap();
    p.sync(&r, &masses).unwrap();
    assert_eq!(p.counts(), (1, 3));
    let old = p.body(id(1)).unwrap();
    let before = p.engine.pose(old).unwrap();
    p.engine.tick(p.scene).unwrap();
    for _ in 0..3 {
        p.engine.step(p.scene, 0.01).unwrap();
    }
    let after = p.engine.pose(old).unwrap();
    assert!(after[1] < before[1]);
    p.capture(&mut r, &masses).unwrap();
    let rev = r.get(parent).unwrap().revision;
    let original = r.get(parent).unwrap().clone();
    let split = SplitPlan {
        source_epoch: 0,
        children: vec![BTreeSet::from([[31, 10, 16], [32, 10, 16]])],
        retained: None,
    };
    let child = r
        .apply_split(
            parent,
            rev,
            &split,
            &[id(2)],
            &mut PreservePayloads,
            &masses,
        )
        .unwrap()
        .children[0];
    p.sync(&r, &masses).unwrap();
    assert_eq!(p.counts(), (2, 4));
    assert!(p.engine.pose(old).is_err());
    for h in [parent, child] {
        let l = r.get(h).unwrap();
        let body = p.body(l.id).unwrap();
        let m = masses.compute(&l.contents).unwrap();
        let w = l.motion.pose.transform(m.center);
        let pose = p.engine.pose(body).unwrap();
        for i in 0..3 {
            assert!((pose[i] - w[i]).abs() < 1e-4)
        }
        let v = p.engine.velocity(body).unwrap();
        for i in 0..3 {
            assert!((v[i] - l.motion.linear[i]).abs() < 1e-5);
            assert!((v[i + 3] - original.motion.angular[i]).abs() < 1e-5)
        }
    }
    let child_body = p.body(id(2)).unwrap();
    r.unload(child).unwrap();
    p.sync(&r, &masses).unwrap();
    assert_eq!(p.counts(), (1, 2));
    assert!(p.engine.pose(child_body).is_err());
    // Restore through the real versioned saved boundary; runtime IDs are fresh.
    let bytes = sable_core::persistence::encode(&r).unwrap();
    let mut restored = sable_core::persistence::decode(&bytes, Default::default()).unwrap();
    let restored_child = restored.handle(id(2)).unwrap();
    assert!(restored.get(child).is_err());
    struct Ready;
    impl sable_core::persistence::ResidencyHost for Ready {
        fn chunk_ready(&self, _: i32, _: i32) -> bool {
            true
        }
    }
    assert!(
        sable_core::persistence::reload_with(
            &mut restored,
            restored_child,
            &Ready,
            10000,
            sable_core::persistence::VelocityPolicy::Exact
        )
        .unwrap()
    );
    p.sync(&restored, &masses).unwrap();
    assert_eq!(p.counts(), (2, 4));
    for uuid in [id(1), id(2)] {
        let h = restored.handle(uuid).unwrap();
        restored.delete(h).unwrap();
    }
    p.sync(&restored, &masses).unwrap();
    assert_eq!(p.counts(), (0, 0));
    let empty = sable_core::persistence::decode(
        &sable_core::persistence::encode(&restored).unwrap(),
        Default::default(),
    )
    .unwrap();
    assert_eq!(empty.levels().count(), 0);
    // Collision execution proves uploaded core cells reach the real solver.
    let terrain = p
        .engine
        .register_material(sable_physics::Material {
            friction: 1.,
            volume: 1.,
            restitution: 0.,
            fluid: false,
            dynamic: false,
            boxes: vec![[0., 0., 0., 1., 1., 1.]],
            contact: None,
        })
        .unwrap();
    p.engine
        .upload_chunk(
            p.scene,
            [0, 0, 0],
            true,
            None,
            vec![([0, 0, 0], terrain, 3)],
        )
        .unwrap();
    let mut falling = Level::new(id(3));
    falling.contents.blocks.insert([8, 0, 8], b());
    falling.motion.pose.pivot = [8.5, 0.5, 8.5];
    falling.motion.pose.position = [0.5, 4.5, 0.5];
    let f = restored.insert(falling).unwrap();
    p.sync(&restored, &masses).unwrap();
    let body = p.body(id(3)).unwrap();
    p.engine.tick(p.scene).unwrap();
    for _ in 0..180 {
        p.engine.step(p.scene, 0.025).unwrap();
    }
    let supported = p.engine.pose(body).unwrap();
    assert!(
        supported[1] > 1.4 && supported[1] < 1.6,
        "supported pose {supported:?}"
    );
    p.capture(&mut restored, &masses).unwrap();
    restored.delete(f).unwrap();
    p.sync(&restored, &masses).unwrap();
    assert_eq!(p.counts(), (0, 0));
    assert!(p.engine.pose(body).is_err());
    p.engine.remove_chunk(p.scene, [0, 0, 0], true).unwrap();
}
