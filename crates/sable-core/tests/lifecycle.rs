use sable_core::{
    gather::{self, GatherState},
    mass::MassRegistry,
    split::{SplitPlan, SplitTracker},
    transfer::PreservePayloads,
    *,
};
use std::collections::BTreeSet;
fn id(n: u8) -> Id {
    Id([n; 16])
}
fn block() -> Block {
    Block {
        state: "minecraft:blackstone".into(),
        block_entity: None,
        extension: Payload::default(),
    }
}
fn level(n: u8, positions: impl IntoIterator<Item = Pos>) -> Level {
    let mut l = Level::new(id(n));
    l.contents.blocks = positions.into_iter().map(|p| (p, block())).collect();
    l
}
#[test]
fn coordinate_spaces_limits_bounds_and_plot_reuse() {
    let mut r = Registry::new(Config::default()).unwrap();
    let h = r
        .insert(level(
            1,
            [
                [0, -64, 0],
                [15, 0, 15],
                [16, 0, 16],
                [31, 0, 31],
                [32, 319, 32],
                [2047, 0, 2047],
            ],
        ))
        .unwrap();
    assert_eq!(r.get(h).unwrap().contents.sections().len(), 5);
    let before = r.get(h).unwrap().clone();
    for p in [
        [-1, 0, 0],
        [2048, 0, 0],
        [0, -65, 0],
        [0, 320, 0],
        [0, 0, 2048],
        [i32::MAX, 0, 0],
    ] {
        assert_eq!(r.edit(h, p, Some(block())), Err(Error::InvalidCoordinate));
        assert_eq!(*r.get(h).unwrap(), before);
    }
    let h2 = r.insert(level(2, [[0, 0, 0]])).unwrap();
    assert_eq!(r.get(h2).unwrap().plot, 128); // x outer, z inner
    let reserved = r.config().reserved(0, [0, 0, 0]).unwrap();
    assert_eq!(reserved, [20_480_000, 0, 20_480_000]);
    let removed = r.delete(h).unwrap();
    assert_eq!(removed.id, id(1));
    let replacement = r.insert(level(3, [[0, 0, 0]])).unwrap();
    assert_eq!(r.get(replacement).unwrap().plot, 0);
    assert_eq!(r.get(h), Err(Error::StaleHandle));
    let mut other = Registry::new(Config::default()).unwrap();
    other.insert(level(3, [[0, 0, 0]])).unwrap();
    assert_eq!(other.get(replacement), Err(Error::StaleHandle));
    let bad = Config {
        origin: [16384, 0],
        ..Config::default()
    };
    assert!(Registry::new(bad).is_err());
}
#[test]
fn transforms_negative_floor_far_origin_and_nonzero_pivot() {
    let q = std::f64::consts::FRAC_PI_4;
    let pose = Pose {
        position: [30_000_000.125, 18., -30_000_000.25],
        rotation: [0., 0., q.sin(), q.cos()],
        pivot: [15.25, -2.5, 32.],
        scale: [1.; 3],
    };
    for p in [[-17., -16., -1.], [0., 15., 16.], [31., 32., 0.]] {
        let w = pose.transform(p);
        let back = pose.inverse(w);
        for i in 0..3 {
            assert!((back[i] - p[i]).abs() < 1e-8)
        }
    }
    let b = Pose::default().world_bounds([0, 0, 0, 15, 0, 31]);
    assert_eq!(b, [0., 0., 0., 16., 1., 32.]);
    assert_eq!(
        [-17i32, -16, -1, 0, 15, 16, 31, 32].map(|v| v.div_euclid(16)),
        [-2, -1, -1, 0, 0, 1, 1, 2]
    );
}
#[test]
fn gather_predicates_capacity_and_large_assembly_are_not_create_limit() {
    let set = BTreeSet::from([[0, 0, 0], [1, 1, 0], [2, 2, 1]]);
    let result = gather::connected([0, 0, 0], 2, |p| set.contains(&p), |_, _, _| true).unwrap();
    assert_eq!(result.count, 2);
    assert_eq!(result.blocks.unwrap().len(), 2);
    let r = gather::connected([0, 0, 0], 1, |p| set.contains(&p), |_, _, _| true).unwrap();
    assert_eq!((r.state, r.count), (GatherState::TooManyBlocks, 2));
    assert!(r.blocks.is_none());
    let r = gather::connected([0, 0, 0], 5, |p| set.contains(&p), |_, _, _| false).unwrap();
    assert_eq!(r.count, 1);
    assert_eq!(
        gather::connected([9, 9, 9], 1, |p| set.contains(&p), |_, _, _| true)
            .unwrap()
            .state,
        GatherState::NoBlocks
    );
    let mut world = Contents::default();
    for x in -1025..1025 {
        world.blocks.insert([x, 0, -1], block());
    }
    let gathered = gather::connected(
        [-1, 0, -1],
        2050,
        |p| world.blocks.contains_key(&p),
        |_, _, _| true,
    )
    .unwrap();
    assert_eq!(gathered.count, 2050);
    // Fit a multi-chunk >2048-block fixture into the 2048-side plot.
    world = Contents::default();
    for x in -32..32 {
        for z in -16..17 {
            world.blocks.insert([x, 0, z], block());
        }
    }
    let positions = world.blocks.keys().copied().collect::<Vec<_>>();
    let count = positions.len();
    assert!(count > 2048);
    let mut r = Registry::new(Config::default()).unwrap();
    let (h, _) = r
        .assemble(
            &mut world,
            id(1),
            [0, 0, 0],
            &positions,
            &mut PreservePayloads,
            &MassRegistry::pinned_full_blocks(),
        )
        .unwrap();
    assert!(world.blocks.is_empty());
    assert_eq!(r.get(h).unwrap().contents.blocks.len(), count);
    assert!(r.get(h).unwrap().contents.sections().len() > 1);
}
#[test]
fn atomic_assembly_refusal_and_split_generation_preserve_contents() {
    let masses = MassRegistry::pinned_full_blocks();
    let mut r = Registry::new(Config::default()).unwrap();
    let mut world = level(9, [[0, 0, 0], [3000, 0, 0]]).contents;
    let old = world.clone();
    assert!(
        r.assemble(
            &mut world,
            id(1),
            [0, 0, 0],
            &[[0, 0, 0], [3000, 0, 0]],
            &mut PreservePayloads,
            &masses
        )
        .is_err()
    );
    assert_eq!(world, old);
    assert_eq!(r.levels().count(), 0);
    let h = r
        .insert(level(
            1,
            [
                [10, 0, 10],
                [11, 0, 10],
                [12, 0, 10],
                [13, 0, 10],
                [14, 0, 10],
            ],
        ))
        .unwrap();
    let mut heat = SplitTracker::new();
    for p in r.get(h).unwrap().contents.blocks.keys() {
        heat.on_added(*p)
    }
    heat.tick(
        &r.get(h).unwrap().contents.blocks.keys().copied().collect(),
        200,
        true,
    );
    r.edit(h, [12, 0, 10], None).unwrap();
    heat.on_removed([12, 0, 10]);
    let solids = r.get(h).unwrap().contents.blocks.keys().copied().collect();
    let plans = heat.tick(&solids, 200, true);
    assert_eq!(plans.len(), 1);
    let revision = r.get(h).unwrap().revision;
    r.edit(h, [12, 0, 10], Some(block())).unwrap();
    assert_eq!(
        r.apply_split(
            h,
            revision,
            &plans[0],
            &[id(2)],
            &mut PreservePayloads,
            &masses
        ),
        Err(Error::StaleHandle)
    );
    assert_eq!(r.get(h).unwrap().contents.blocks.len(), 5);
    let fake = SplitPlan {
        source_epoch: 0,
        children: vec![BTreeSet::from([[14, 0, 10]])],
        retained: None,
    };
    let oldrev = r.get(h).unwrap().revision;
    r.delete(h).unwrap();
    let _new = r.insert(level(3, [[10, 0, 10]])).unwrap();
    assert_eq!(
        r.apply_split(h, oldrev, &fake, &[id(2)], &mut PreservePayloads, &masses),
        Err(Error::StaleHandle)
    );
}
#[test]
fn state_registry_mass_and_child_split_motion_match_rigid_velocity_field() {
    let masses = MassRegistry::pinned_full_blocks();
    let mut l = level(1, [[10, 0, 10], [11, 0, 10], [14, 0, 10]]);
    l.contents.blocks.get_mut(&[14, 0, 10]).unwrap().state = "minecraft:bedrock".into();
    let m = masses.compute(&l.contents).unwrap();
    assert_eq!(m.mass, 1004.);
    assert!((m.center[0] - (10.5 * 2. + 11.5 * 2. + 14.5 * 1000.) / 1004.).abs() < 1e-12);
    l.motion.pose = Pose {
        position: [100., 50., -10.],
        pivot: [12., 0.5, 10.5],
        rotation: [0., 0., (0.3f64).sin(), (0.3f64).cos()],
        scale: [1.; 3],
    };
    l.motion.linear = [1., 2., 3.];
    l.motion.angular = [0., 0., 2.];
    let old = l.clone();
    let old_com = old.motion.pose.transform(m.center);
    let mut r = Registry::new(Config::default()).unwrap();
    let h = r.insert(l).unwrap();
    let plan = SplitPlan {
        source_epoch: 0,
        children: vec![BTreeSet::from([[14, 0, 10]])],
        retained: None,
    };
    let rev = r.get(h).unwrap().revision;
    let child = r
        .apply_split(h, rev, &plan, &[id(2)], &mut PreservePayloads, &masses)
        .unwrap()
        .children[0];
    let c = r.get(child).unwrap();
    let child_com = c
        .motion
        .pose
        .transform(masses.compute(&c.contents).unwrap().center);
    assert_eq!(c.parent, Some(id(1)));
    assert_eq!(c.motion.angular, old.motion.angular);
    assert_eq!(
        c.motion.linear,
        add(
            old.motion.linear,
            cross(old.motion.angular, sub(child_com, old_com))
        )
    );
    assert_eq!(
        c.motion.pose.transform([14.5, 0.5, 10.5]),
        old.motion.pose.transform([14.5, 0.5, 10.5])
    );
    assert_eq!(
        r.levels()
            .map(|(l, _)| l.contents.blocks.len())
            .sum::<usize>(),
        3
    );
    assert_eq!(r.get(h).unwrap().motion.linear, old.motion.linear);
    let unknown = level(8, [[0, 0, 0]]);
    let mut bad = unknown.contents;
    bad.blocks.get_mut(&[0, 0, 0]).unwrap().state = "unknown".into();
    assert_eq!(masses.compute(&bad), Err(Error::InvalidInput));
}
#[test]
fn overflow_unload_does_not_partially_change_residency() {
    let mut l = level(1, [[0, 0, 0]]);
    l.revision = u64::MAX - 1;
    let mut r = Registry::restore(Config::default(), vec![(l, Residency::Live)]).unwrap();
    let h = r.handle(id(1)).unwrap();
    r.edit(h, [0, 0, 0], Some(block())).unwrap();
    assert_eq!(r.unload(h), Err(Error::Capacity));
    assert_eq!(r.residency(h).unwrap(), Residency::Live);
}

#[test]
fn split_unload_save_reload_delete_preserve_every_owned_payload() {
    let masses = MassRegistry::pinned_full_blocks();
    let mut l = level(1, [[10, 0, 10], [11, 0, 10], [13, 0, 10], [14, 0, 10]]);
    let inventory = Payload {
        format: "host-inventory-v1".into(),
        bytes: vec![0, 255, 2, 64, 7, 1],
    };
    l.contents
        .blocks
        .get_mut(&[14, 0, 10])
        .unwrap()
        .block_entity = Some(inventory.clone());
    let entity = Entity {
        id: id(3),
        anchor: [14, 0, 10],
        position: [14.25, 0.75, 10.5],
        payload: Payload {
            format: "retained".into(),
            bytes: vec![8, 9],
        },
    };
    l.contents.entities.insert(entity.id, entity.clone());
    let tick = ScheduledTick {
        id: id(4),
        position: [14, 0, 10],
        kind: TickKind::Fluid,
        due: 951,
        priority: -3,
        payload: Payload {
            format: "fluid-tick".into(),
            bytes: vec![17, 16],
        },
    };
    l.contents.ticks.push(tick.clone());
    let mut r = Registry::new(Config::default()).unwrap();
    let h = r.insert(l).unwrap();
    let p = SplitPlan {
        source_epoch: 0,
        children: vec![BTreeSet::from([[13, 0, 10], [14, 0, 10]])],
        retained: None,
    };
    let c = r
        .apply_split(
            h,
            r.get(h).unwrap().revision,
            &p,
            &[id(2)],
            &mut PreservePayloads,
            &masses,
        )
        .unwrap()
        .children[0];
    assert!(r.get(h).unwrap().contents.entities.is_empty());
    assert!(r.get(h).unwrap().contents.ticks.is_empty());
    r.unload(c).unwrap();
    let bytes = sable_core::persistence::encode(&r).unwrap();
    let mut restarted = sable_core::persistence::decode(&bytes, Default::default()).unwrap();
    let c = restarted.handle(id(2)).unwrap();
    struct Ready;
    impl sable_core::persistence::ResidencyHost for Ready {
        fn chunk_ready(&self, _: i32, _: i32) -> bool {
            true
        }
    }
    assert!(
        sable_core::persistence::reload_with(
            &mut restarted,
            c,
            &Ready,
            10000,
            sable_core::persistence::VelocityPolicy::Exact
        )
        .unwrap()
    );
    let child = restarted.get(c).unwrap();
    assert_eq!(
        child.contents.blocks[&[14, 0, 10]].block_entity,
        Some(inventory)
    );
    assert_eq!(child.contents.entities[&id(3)], entity);
    assert_eq!(child.contents.ticks, vec![tick]);
    assert_eq!(
        restarted
            .levels()
            .map(|(l, _)| l.contents.blocks.len())
            .sum::<usize>(),
        4
    );
    let removed = restarted.delete(c).unwrap();
    assert_eq!(removed.contents.entities.len(), 1);
    assert_eq!(removed.contents.ticks.len(), 1);
    let after = sable_core::persistence::decode(
        &sable_core::persistence::encode(&restarted).unwrap(),
        Default::default(),
    )
    .unwrap();
    assert!(after.handle(id(2)).is_err());
    assert_eq!(
        after
            .levels()
            .map(|(l, _)| l.contents.record_count())
            .sum::<usize>(),
        2
    );
}

#[test]
fn dependency_group_unloads_atomically_and_historical_ancestry_is_validated() {
    let a = level(1, [[0, 0, 0]]);
    let mut b = level(2, [[0, 0, 0]]);
    b.plot = 1;
    b.dependencies.insert(a.id);
    let mut r = Registry::restore(
        Config::default(),
        vec![(a.clone(), Residency::Live), (b.clone(), Residency::Live)],
    )
    .unwrap();
    let h = r.handle(a.id).unwrap();
    r.unload(h).unwrap();
    assert!(r.levels().all(|(_, s)| s == Residency::Holding));
    assert_eq!(r.delete(h), Err(Error::MissingDependency));
    b.dependencies.clear();
    b.parent = Some(b.id);
    assert!(matches!(
        Registry::restore(Config::default(), vec![(b.clone(), Residency::Live)]),
        Err(Error::CyclicDependency)
    ));
    b.parent = Some(id(99));
    assert!(Registry::restore(Config::default(), vec![(b, Residency::Live)]).is_ok());
}
