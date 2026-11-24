//! These checks require the actual separately compiled native library. They are
//! deliberately not replaced by a fake library when the artifact is absent.
use sable_core::{Config, Id, Motion, mass::MassRegistry};
use sable_sunrise::simulation::{GeometrySnapshot, PhysicsReplica, TerrainSection};
use std::collections::BTreeMap;

fn observation(id: u8, plot: u32) -> GeometrySnapshot {
    GeometrySnapshot {
        id: Id([id; 16]),
        generation: 1,
        revision: 1,
        plot,
        cells: BTreeMap::from([
            ([0, 0, 0], "minecraft:blackstone".into()),
            ([16, 0, 0], "minecraft:blackstone".into()),
        ]),
        motion: Motion {
            linear: [1., 0., 0.],
            ..Motion::default()
        },
    }
}

#[test]
#[ignore = "requires accepted-nightly cdylib; run with SABLE_PHYSICS_LIBRARY and --ignored"]
fn communicating_replica_keeps_payload_revisions_incremental_and_preflights_all_geometry() {
    let library =
        std::env::var("SABLE_PHYSICS_LIBRARY").expect("explicit actual native artifact required");
    let mut owner = PhysicsReplica::open(library, Config::default(), [0.; 3]).unwrap();
    let masses = MassRegistry::pinned_full_blocks();
    let mut snapshots = vec![observation(1, 0), observation(2, 1)];
    snapshots[1].motion.pose.position[0] = 100.;
    owner.sync(&snapshots, &masses).unwrap();
    let first = owner.body(snapshots[0].id).unwrap();
    let second = owner.body(snapshots[1].id).unwrap();
    assert_eq!(owner.counts(), (2, 4));
    assert!(owner.tick(2).is_err()); // Unobserved terrain is not an empty world.
    owner.sync_terrain(&[], &masses).unwrap();
    owner.tick(2).unwrap();
    let motions = owner.capture(&masses).unwrap();
    // Three fresh original JVMs yielded this exact world-COM channel. Preserve
    // the solver's f32 result rather than an idealized v*dt epsilon. Capture:
    // tools/phase3_bridge_motion/{cases,original,index}.v1.json.
    assert_eq!(
        motions[&snapshots[0].id].pose.position[0].to_bits(),
        (f64::from_bits(0x4021199800000000) - 8.5).to_bits()
    );
    let (native_pose, native_velocity) = owner.native_sample(snapshots[0].id).unwrap();
    assert_eq!(native_pose[0].to_bits(), 0x4021199800000000);
    assert_eq!(native_velocity, [1., 0., 0., 0., 0., 0.]);
    for snapshot in &mut snapshots {
        snapshot.motion = motions[&snapshot.id].clone();
    }
    for revision in 2..=20 {
        snapshots[0].revision = revision; // Host payloads stay exclusively in the host.
        owner.sync(&snapshots, &masses).unwrap();
        assert_eq!(owner.body(snapshots[0].id), Some(first));
        assert_eq!(owner.body(snapshots[1].id), Some(second));
        assert_eq!(owner.capture(&masses).unwrap(), motions);
    }
    // A newly resolved furnace-like state can have an identical physical
    // signature. Catalog growth and this state edit preserve both bodies.
    let mut masses = masses;
    masses
        .register(
            "fixture:same-physics-lit".into(),
            masses.get("minecraft:blackstone").unwrap().clone(),
        )
        .unwrap();
    owner.sync(&snapshots, &masses).unwrap();
    assert_eq!(owner.body(snapshots[0].id), Some(first));
    assert_eq!(owner.body(snapshots[1].id), Some(second));
    snapshots[0].revision += 1;
    snapshots[0]
        .cells
        .insert([0, 0, 0], "fixture:same-physics-lit".into());
    owner.sync(&snapshots, &masses).unwrap();
    assert_eq!(owner.body(snapshots[0].id), Some(first));
    assert_eq!(owner.body(snapshots[1].id), Some(second));
    assert_eq!(owner.capture(&masses).unwrap(), motions);
    let registrations = owner.material_count();
    for revision in 22..42 {
        snapshots[0].revision = revision;
        snapshots[0].cells.insert(
            [0, 0, 0],
            if revision % 2 == 0 {
                "minecraft:blackstone"
            } else {
                "fixture:same-physics-lit"
            }
            .into(),
        );
        owner.sync(&snapshots, &masses).unwrap();
        assert_eq!(owner.material_count(), registrations);
        assert_eq!(owner.body(snapshots[0].id), Some(first));
    }
    assert!(owner.tick(2).is_err()); // New catalog still needs observed terrain.
    owner.sync_terrain(&[], &masses).unwrap();
    let mut unsupported = snapshots.clone();
    unsupported[1].revision += 1;
    unsupported[1]
        .cells
        .insert([32, 0, 0], "unknown:unresolved".into());
    assert!(owner.sync(&unsupported, &masses).is_err());
    assert_eq!(owner.body(snapshots[0].id), Some(first));
    assert_eq!(owner.body(snapshots[1].id), Some(second));
    assert_eq!(owner.capture(&masses).unwrap(), motions);
    let mut huge = masses.clone();
    let mut properties = masses.get("minecraft:blackstone").unwrap().clone();
    properties.boxes = vec![[0., 0., 0., 1., 1., 1.]; sable_bridge::abi::MAX_BOXES + 1];
    huge.register("unsupported:box-budget".into(), properties)
        .unwrap();
    unsupported = snapshots.clone();
    unsupported[0].revision += 1;
    unsupported[0]
        .cells
        .insert([0, 0, 0], "unsupported:box-budget".into());
    assert!(owner.sync(&unsupported, &huge).is_err());
    assert_eq!(owner.body(snapshots[0].id), Some(first));
    assert_eq!(owner.capture(&masses).unwrap(), motions);
    snapshots[0].revision += 1;
    snapshots[0]
        .cells
        .insert([16, 0, 0], "minecraft:bedrock".into());
    owner.sync(&snapshots, &masses).unwrap();
    assert_ne!(owner.body(snapshots[0].id), Some(first));
    assert_eq!(owner.body(snapshots[1].id), Some(second));
    let affected = owner.body(snapshots[0].id).unwrap();
    let mut changed = masses.get("fixture:same-physics-lit").unwrap().clone();
    changed.mass = 3.;
    masses
        .register("fixture:same-physics-lit".into(), changed)
        .unwrap();
    owner.sync(&snapshots, &masses).unwrap();
    assert_ne!(owner.body(snapshots[0].id), Some(affected));
    assert_eq!(owner.body(snapshots[1].id), Some(second));
}

#[test]
#[ignore = "requires accepted-nightly cdylib; run with SABLE_PHYSICS_LIBRARY and --ignored"]
fn communicating_replica_handles_plot_swap_generation_change_and_changed_restored_motion() {
    let library =
        std::env::var("SABLE_PHYSICS_LIBRARY").expect("explicit actual native artifact required");
    let mut owner = PhysicsReplica::open(library, Config::default(), [0.; 3]).unwrap();
    let masses = MassRegistry::pinned_full_blocks();
    let mut snapshots = vec![observation(3, 0), observation(4, 1)];
    owner.sync(&snapshots, &masses).unwrap();
    let old: Vec<_> = snapshots
        .iter()
        .map(|s| owner.body(s.id).unwrap())
        .collect();
    snapshots[0].plot = 1;
    snapshots[1].plot = 0;
    for snapshot in &mut snapshots {
        snapshot.generation += 1;
    }
    owner.sync(&snapshots, &masses).unwrap();
    for (snapshot, old) in snapshots.iter().zip(old) {
        assert_ne!(owner.body(snapshot.id), Some(old));
    }
    let body = owner.body(snapshots[0].id).unwrap();
    let mut stale = snapshots.clone();
    stale[0].generation -= 1;
    assert!(owner.sync(&stale, &masses).is_err());
    assert_eq!(owner.body(snapshots[0].id), Some(body));
    snapshots[0].motion.linear = [0.5, 0., 0.];
    snapshots[0].revision += 1;
    owner.sync(&snapshots, &masses).unwrap();
    assert_ne!(owner.body(snapshots[0].id), Some(body));
    assert_eq!(
        owner.capture(&masses).unwrap()[&snapshots[0].id].linear,
        [0.5, 0., 0.]
    );
    owner.sync(&[], &masses).unwrap();
    assert_eq!(owner.counts(), (0, 0));
    assert!(owner.sync(&snapshots, &masses).is_err());
    for snapshot in &mut snapshots {
        snapshot.generation += 1;
    }
    owner.sync(&snapshots, &masses).unwrap();
    assert_eq!(owner.counts(), (2, 4));
}

#[test]
#[ignore = "requires accepted-nightly cdylib; run with SABLE_PHYSICS_LIBRARY and --ignored"]
fn communicating_replica_uses_sampled_ordinary_voxels_for_native_floor_contacts() {
    let library =
        std::env::var("SABLE_PHYSICS_LIBRARY").expect("explicit actual native artifact required");
    let mut owner = PhysicsReplica::open(library, Config::default(), [0., -9.81, 0.]).unwrap();
    let masses = MassRegistry::pinned_full_blocks();
    let mut snapshot = observation(8, 0);
    snapshot.motion.pose.position[1] = 3.;
    snapshot.motion.linear = [0.; 3];
    owner.sync(&[snapshot.clone()], &masses).unwrap();
    let mut floor = vec![
        TerrainSection {
            section: [0, 0, 0],
            cells: BTreeMap::new(),
        },
        TerrainSection {
            section: [1, 0, 0],
            cells: BTreeMap::new(),
        },
    ];
    for x in 0..=16 {
        floor[(x / 16) as usize]
            .cells
            .insert([x % 16, 0, 0], "minecraft:blackstone".into());
    }
    owner.sync_terrain(&floor, &masses).unwrap();
    for _ in 0..100 {
        owner.tick(2).unwrap();
    }
    let resting = owner.capture(&masses).unwrap()[&snapshot.id].clone();
    // Independent contact invariant, not a newly invented differential epsilon.
    assert!(
        (0.98..=1.02).contains(&resting.pose.position[1]),
        "{resting:?}"
    );
    let mut bad = floor.clone();
    bad[0].section = [1 << 21, 0, 0];
    assert!(owner.sync_terrain(&bad, &masses).is_err());
    assert_eq!(owner.capture(&masses).unwrap()[&snapshot.id], resting);
    snapshot.motion = resting;
    owner.sync(&[snapshot], &masses).unwrap();
    owner.sync_terrain(&[], &masses).unwrap();
    for _ in 0..20 {
        owner.tick(2).unwrap();
    }
    assert!(
        owner
            .capture(&masses)
            .unwrap()
            .values()
            .next()
            .unwrap()
            .pose
            .position[1]
            < 0.
    );
}

#[test]
#[ignore = "requires accepted-nightly cdylib; run with SABLE_PHYSICS_LIBRARY and --ignored"]
fn communicating_replica_shares_one_owner_across_controlled_and_dynamic_scenes() {
    let library =
        std::env::var("SABLE_PHYSICS_LIBRARY").expect("explicit actual native artifact required");
    let mut controlled = PhysicsReplica::open(library, Config::default(), [0.; 3]).unwrap();
    let mut dynamic = controlled
        .new_scene(Config::default(), [0., -9.81, 0.])
        .unwrap();
    let masses = MassRegistry::pinned_full_blocks();
    let control = observation(10, 0);
    controlled
        .sync(std::slice::from_ref(&control), &masses)
        .unwrap();
    controlled.sync_terrain(&[], &masses).unwrap();
    let mut released = observation(11, 0);
    released.cells = [[0, 0, 0], [1, 0, 0], [0, 1, 0]]
        .into_iter()
        .map(|pos| (pos, "minecraft:blackstone".into()))
        .collect();
    released.motion.pose.position = [2., 8., 2.];
    released.motion.linear = [0.; 3];
    dynamic.sync(&[released.clone()], &masses).unwrap();
    let mut floor: BTreeMap<_, BTreeMap<_, String>> = BTreeMap::new();
    for x in -4i32..=8 {
        for z in -4i32..=8 {
            floor
                .entry([x.div_euclid(16), 0, z.div_euclid(16)])
                .or_default()
                .insert(
                    [x.rem_euclid(16) as u8, 0, z.rem_euclid(16) as u8],
                    "minecraft:blackstone".into(),
                );
        }
    }
    dynamic
        .sync_terrain(
            &floor
                .into_iter()
                .map(|(section, cells)| TerrainSection { section, cells })
                .collect::<Vec<_>>(),
            &masses,
        )
        .unwrap();
    // The original solver realizes pending additional mass properties during
    // integration. Release one real step before applying the explicit impulse.
    dynamic.tick(2).unwrap();
    dynamic
        .impulse(released.id, [0.; 3], [0., 0., 1.], true)
        .unwrap();
    let kicked = dynamic.capture(&masses).unwrap()[&released.id].clone();
    assert!(kicked.angular[2] > 0.);
    for _ in 0..10 {
        dynamic.tick(2).unwrap();
        controlled.tick(2).unwrap();
    }
    let falling = dynamic.capture(&masses).unwrap()[&released.id].clone();
    assert!(falling.pose.position[1] < released.motion.pose.position[1]);
    assert!(falling.linear[1] < 0.);
    assert_ne!(falling.pose.rotation, released.motion.pose.rotation);
    let still_controlled = controlled.capture(&masses).unwrap()[&control.id].clone();
    assert_eq!(still_controlled.pose.position[1], 0.);
    assert!(still_controlled.pose.position[0] > 0.);
    let mut contacts = dynamic.collisions().unwrap();
    for _ in 0..90 {
        dynamic.tick(2).unwrap();
        contacts.extend(dynamic.collisions().unwrap());
    }
    assert!(!contacts.is_empty());
    assert_eq!(contacts.len() % 15, 0);
    assert!(contacts.iter().all(|value| value.is_finite()));
    let landed = dynamic.capture(&masses).unwrap()[&released.id].clone();
    assert!(
        landed.pose.position[1] > 0. && landed.pose.position[1] < 4.,
        "{landed:?}"
    );
    // Disposing one scenario leaves the single shared owner's other scene live.
    drop(dynamic);
    controlled.tick(2).unwrap();
    assert!(controlled.capture(&masses).is_ok());
}

#[test]
#[ignore = "requires accepted-nightly cdylib; run with SABLE_PHYSICS_LIBRARY and --ignored"]
fn communicating_replica_reconstructs_rotating_native_state_before_the_next_step() {
    let library =
        std::env::var("SABLE_PHYSICS_LIBRARY").expect("explicit actual native artifact required");
    let mut owner = PhysicsReplica::open(library, Config::default(), [0., -9.81, 0.]).unwrap();
    let masses = MassRegistry::pinned_full_blocks();
    let mut snapshot = observation(12, 0);
    snapshot.cells = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 2]]
        .into_iter()
        .map(|pos| (pos, "minecraft:blackstone".into()))
        .collect();
    snapshot.motion.pose.position = [-32_768.25, 256.5, 65_536.75];
    snapshot.motion.pose.pivot = [0.125, 0.75, 0.25];
    snapshot.motion.linear = [0.75, 0.125, -0.5];
    snapshot.motion.angular = [0.3125, -0.625, 0.9375];
    owner.sync(&[snapshot.clone()], &masses).unwrap();
    owner.sync_terrain(&[], &masses).unwrap();
    for tick in 1..=61 {
        owner.tick(2).unwrap();
        if ![1, 7, 20, 61].contains(&tick) {
            continue;
        }
        let before = owner.native_sample(snapshot.id).unwrap();
        assert_ne!(&before.0[3..], &[0., 0., 0., 1.]);
        assert!(before.1[1] < snapshot.motion.linear[1]);
        snapshot.motion = owner.capture(&masses).unwrap()[&snapshot.id].clone();
        // Recreate a real body in another scene under the same native owner.
        // This witnesses reconstruction only; it is not a game/process restart
        // or a claim to serialize the solver's contact warm-start state.
        let mut restored = owner.new_scene(Config::default(), [0., -9.81, 0.]).unwrap();
        restored
            .sync(std::slice::from_ref(&snapshot), &masses)
            .unwrap();
        let after = restored.native_sample(snapshot.id).unwrap();
        assert_eq!(before.0.map(f64::to_bits), after.0.map(f64::to_bits));
        assert_eq!(before.1.map(f64::to_bits), after.1.map(f64::to_bits));
    }
}
