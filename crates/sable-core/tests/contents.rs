use sable_core::persistence::{self, DecodeLimits, ResidencyHost, VelocityPolicy, WriteFault};
use sable_core::transfer::{self, PreservePayloads, TransferEvent, TransferHost};
use sable_core::*;
use std::collections::BTreeSet;

fn id(n: u8) -> Id {
    Id([n; 16])
}
fn payload(bytes: &[u8]) -> Payload {
    Payload {
        format: "test/opaque-coordinate-free".into(),
        bytes: bytes.to_vec(),
    }
}
fn contents() -> Contents {
    let mut c = Contents {
        extension: payload(b"chunk-light-heightmap-attachments"),
        ..Contents::default()
    };
    for p in [[15, 0, 31], [16, 0, 32], [32, 0, 32]] {
        c.blocks.insert(
            p,
            Block {
                state: "minecraft:chest[facing=north]".into(),
                block_entity: Some(payload(b"inventory\0\xff\x80items=64")),
                extension: payload(b"block-extension"),
            },
        );
    }
    c.entities.insert(
        id(2),
        Entity {
            id: id(2),
            anchor: [15, 0, 31],
            position: [15.25, 0.5, 31.75],
            payload: payload(b"entity-passengers-and-components"),
        },
    );
    c.ticks = vec![
        ScheduledTick {
            id: id(3),
            position: [15, 0, 31],
            kind: TickKind::Block,
            due: 12345,
            priority: -1,
            payload: payload(b"redstone"),
        },
        ScheduledTick {
            id: id(4),
            position: [16, 0, 32],
            kind: TickKind::Fluid,
            due: 12346,
            priority: 1,
            payload: payload(b"water"),
        },
    ];
    c
}
fn registry() -> Registry {
    let mut r = Registry::new(Config::default()).unwrap();
    let mut l = Level::new(id(1));
    l.contents = contents();
    l.name = Some("Moving inventory".into());
    l.user_data = payload(b"arbitrary-user-data");
    l.motion.linear = [1.25, -2.0, 3.0];
    l.motion.angular = [0.0, 0.5, 0.25];
    l.motion.pose.position = [30_000_000.0, 123.5, -30_000_000.0];
    l.motion.pose.pivot = [15.5, 0.5, 31.5];
    l.motion.pose.rotation = [0.0, 0.0, 0.5_f64.sqrt(), 0.5_f64.sqrt()];
    r.insert(l).unwrap();
    r
}

#[test]
fn transfer_retains_inventory_entity_and_tick_identity_and_order() {
    let mut source = contents();
    let original = source.clone();
    let mut destination = Contents::default();
    let p = transfer::plan(
        &source,
        &destination,
        &[[15, 0, 31], [16, 0, 32]],
        [-31, 4, -47],
        100,
    )
    .unwrap();
    let events =
        transfer::move_contents(&mut source, &mut destination, &p, &mut PreservePayloads).unwrap();
    assert_eq!(
        source.blocks.len() + destination.blocks.len(),
        original.blocks.len()
    );
    assert_eq!(
        destination.blocks[&[-16, 4, -16]].block_entity,
        original.blocks[&[15, 0, 31]].block_entity
    );
    assert_eq!(destination.entities[&id(2)].position, [-15.75, 4.5, -15.25]);
    assert_eq!(destination.entities[&id(2)].anchor, [-16, 4, -16]);
    assert_eq!(
        destination.entities[&id(2)].payload,
        original.entities[&id(2)].payload
    );
    assert_eq!(
        destination.ticks.iter().map(|t| t.id).collect::<Vec<_>>(),
        vec![id(3), id(4)]
    );
    assert_eq!(destination.ticks[0].due, 12345);
    assert_eq!(destination.ticks[1].kind, TickKind::Fluid);
    assert_eq!(source.extension, original.extension); // Owner-level extension belongs to its owner.
    assert_eq!(
        events,
        vec![
            TransferEvent::BeforeMove {
                from: [15, 0, 31],
                to: [-16, 4, -16]
            },
            TransferEvent::AfterMove {
                from: [15, 0, 31],
                to: [-16, 4, -16]
            },
            TransferEvent::BeforeMove {
                from: [16, 0, 32],
                to: [-15, 4, -15]
            },
            TransferEvent::AfterMove {
                from: [16, 0, 32],
                to: [-15, 4, -15]
            },
            TransferEvent::Notify {
                position: [-16, 4, -16]
            },
            TransferEvent::Notify {
                position: [-15, 4, -15]
            },
            TransferEvent::SourceRemoved {
                position: [15, 0, 31]
            },
            TransferEvent::SourceRemoved {
                position: [16, 0, 32]
            },
            TransferEvent::SendUpdated {
                position: [15, 0, 31]
            },
            TransferEvent::SendUpdated {
                position: [16, 0, 32]
            },
        ]
    );
}

struct FailAt {
    remaining: usize,
}
impl TransferHost for FailAt {
    fn rewrite_payload(&mut self, p: &Payload, _from: Pos, _to: Pos) -> Result<Payload> {
        if self.remaining == 0 {
            return Err(Error::HostRefused);
        }
        self.remaining -= 1;
        Ok(p.clone())
    }
    fn prepare_event(&mut self, _event: &TransferEvent) -> Result<()> {
        if self.remaining == 0 {
            return Err(Error::HostRefused);
        }
        self.remaining -= 1;
        Ok(())
    }
}

#[test]
fn transfer_failure_at_every_preparation_boundary_preserves_both_owners() {
    // 7 payload preparations and 10 event preparations; each failure is atomic.
    for remaining in 0..17 {
        let mut source = contents();
        let before = source.clone();
        let mut destination = Contents::default();
        let p = transfer::plan(
            &source,
            &destination,
            &[[15, 0, 31], [16, 0, 32]],
            [1, 0, 1],
            100,
        )
        .unwrap();
        let result =
            transfer::move_contents(&mut source, &mut destination, &p, &mut FailAt { remaining });
        assert_eq!(result, Err(Error::HostRefused), "boundary {remaining}");
        assert_eq!(source, before);
        assert_eq!(destination, Contents::default());
    }
}

#[test]
fn transfer_refusals_and_stale_plan_never_mutate_contents() {
    let mut s = contents();
    let mut d = Contents::default();
    assert!(matches!(
        transfer::plan(&s, &d, &[[15, 0, 31]; 2], [0; 3], 100),
        Err(Error::InvalidInput)
    ));
    assert!(matches!(
        transfer::plan(&s, &d, &[[15, 0, 31]], [i32::MAX, 0, 0], 100),
        Err(Error::InvalidCoordinate)
    ));
    assert!(matches!(
        transfer::plan(&s, &d, &[[15, 0, 31]], [0; 3], 1),
        Err(Error::Capacity)
    ));
    d.blocks.insert([15, 0, 31], s.blocks[&[15, 0, 31]].clone());
    assert!(matches!(
        transfer::plan(&s, &d, &[[15, 0, 31]], [0; 3], 100),
        Err(Error::Occupied)
    ));
    d.blocks.clear();
    let p = transfer::plan(&s, &d, &[[15, 0, 31]], [0; 3], 100).unwrap();
    s.ticks[0].due += 1;
    let before = s.clone();
    assert_eq!(
        transfer::move_contents(&mut s, &mut d, &p, &mut PreservePayloads),
        Err(Error::StaleHandle)
    );
    assert_eq!(s, before);
    assert_eq!(d, Contents::default());
}

#[test]
fn format_aware_host_rewrites_embedded_coordinates_without_touching_inventory() {
    struct Rewrite;
    impl TransferHost for Rewrite {
        fn rewrite_payload(&mut self, p: &Payload, _from: Pos, to: Pos) -> Result<Payload> {
            let mut p = p.clone();
            if p.format == "test/xyz-plus-inventory" {
                p.bytes
                    .splice(..12, to.into_iter().flat_map(i32::to_le_bytes));
            }
            Ok(p)
        }
    }
    let mut source = contents();
    let mut bytes = Vec::new();
    for n in [15_i32, 0, 31] {
        bytes.extend(n.to_le_bytes())
    }
    bytes.extend([0xff, 0x80, 64, 0]);
    source.blocks.get_mut(&[15, 0, 31]).unwrap().block_entity = Some(Payload {
        format: "test/xyz-plus-inventory".into(),
        bytes,
    });
    let mut target = Contents::default();
    let p = transfer::plan(&source, &target, &[[15, 0, 31]], [1, 0, 1], 100).unwrap();
    transfer::move_contents(&mut source, &mut target, &p, &mut Rewrite).unwrap();
    let result = &target.blocks[&[16, 0, 32]]
        .block_entity
        .as_ref()
        .unwrap()
        .bytes;
    assert_eq!(&result[..4], &16_i32.to_le_bytes());
    assert_eq!(&result[8..12], &32_i32.to_le_bytes());
    assert_eq!(&result[12..], &[0xff, 0x80, 64, 0]);
}

#[test]
fn binary_roundtrip_keeps_all_persistent_fields_and_rejects_runtime_handles() {
    let mut r = registry();
    let old = r.handle(id(1)).unwrap();
    r.unload(old).unwrap();
    let bytes = persistence::encode(&r).unwrap();
    let restored = persistence::decode(&bytes, DecodeLimits::default()).unwrap();
    assert_eq!(
        restored.levels().collect::<Vec<_>>(),
        r.levels().collect::<Vec<_>>()
    );
    assert_eq!(restored.get(old), Err(Error::StaleHandle));
    assert_eq!(persistence::encode(&restored).unwrap(), bytes);
    assert_eq!(
        restored.residency(restored.handle(id(1)).unwrap()).unwrap(),
        Residency::Holding
    );
}

fn checksum(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |h, b| {
        (h ^ u64::from(*b)).wrapping_mul(0x100_0000_01b3)
    })
}
fn recheck(bytes: &mut [u8]) {
    let n = bytes.len() - 8;
    let hash = checksum(&bytes[..n]);
    bytes[n..].copy_from_slice(&hash.to_le_bytes())
}

#[test]
fn corrupt_truncated_version_and_allocation_bomb_saves_fail_closed() {
    let bytes = persistence::encode(&registry()).unwrap();
    for n in 0..bytes.len() {
        assert!(
            persistence::decode(&bytes[..n], DecodeLimits::default()).is_err(),
            "prefix {n}"
        )
    }
    for i in 0..bytes.len() {
        let mut corrupt = bytes.clone();
        corrupt[i] ^= 0x80;
        assert!(
            persistence::decode(&corrupt, DecodeLimits::default()).is_err(),
            "corrupt byte {i}"
        )
    }
    let mut unknown = bytes.clone();
    unknown[8..12].copy_from_slice(&2_u32.to_le_bytes());
    recheck(&mut unknown);
    assert!(matches!(
        persistence::decode(&unknown, DecodeLimits::default()),
        Err(Error::UnsupportedVersion)
    ));
    let mut bomb = bytes.clone();
    bomb[38..42].copy_from_slice(&u32::MAX.to_le_bytes());
    recheck(&mut bomb);
    assert!(matches!(
        persistence::decode(&bomb, DecodeLimits::default()),
        Err(Error::Capacity)
    ));
    assert!(matches!(
        persistence::decode(
            &bytes,
            DecodeLimits {
                max_bytes: 10,
                ..DecodeLimits::default()
            }
        ),
        Err(Error::Capacity)
    ));
}

#[test]
fn snapshots_reject_duplicate_ownership_missing_and_cyclic_dependencies() {
    let mut a = Level::new(id(1));
    a.contents = contents();
    let mut b = Level::new(id(9));
    b.plot = 1;
    assert!(matches!(
        Registry::restore(
            Config::default(),
            vec![
                (a.clone(), Residency::Live),
                (a.clone(), Residency::Holding)
            ]
        ),
        Err(Error::DuplicateId)
    ));
    b.contents = contents();
    assert!(matches!(
        Registry::restore(
            Config::default(),
            vec![
                (a.clone(), Residency::Live),
                (b.clone(), Residency::Holding)
            ]
        ),
        Err(Error::DuplicateId)
    ));
    b.contents = Contents::default();
    a.dependencies.insert(id(9));
    assert!(matches!(
        Registry::restore(Config::default(), vec![(a.clone(), Residency::Holding)]),
        Err(Error::MissingDependency)
    ));
    b.dependencies.insert(id(1));
    assert!(matches!(
        Registry::restore(
            Config::default(),
            vec![(a, Residency::Holding), (b, Residency::Holding)]
        ),
        Err(Error::CyclicDependency)
    ));
}

#[test]
fn atomic_writes_keep_last_usable_save_at_every_precommit_failure() {
    let dir = std::env::temp_dir().join(format!("sable-phase2-contents-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("save.bin");
    let mut r = registry();
    let old = persistence::encode(&r).unwrap();
    persistence::write_atomic(&path, &old, WriteFault::None).unwrap();
    let h = r.handle(id(1)).unwrap();
    r.delete(h).unwrap();
    let empty = persistence::encode(&r).unwrap();
    for fault in [
        WriteFault::BeforeWrite,
        WriteFault::AfterWrite,
        WriteFault::AfterFileSync,
        WriteFault::BeforeRename,
    ] {
        assert_eq!(
            persistence::write_atomic(&path, &empty, fault),
            Err(Error::Interrupted)
        );
        assert_eq!(std::fs::read(&path).unwrap(), old);
        assert_eq!(
            persistence::read(&path, DecodeLimits::default())
                .unwrap()
                .levels()
                .count(),
            1
        );
    }
    std::fs::write(
        dir.join(".save.bin.pending-interrupted"),
        b"partial unfinished file",
    )
    .unwrap();
    assert_eq!(
        persistence::read(&path, DecodeLimits::default())
            .unwrap()
            .levels()
            .count(),
        1
    );
    persistence::write_atomic(&path, &empty, WriteFault::None).unwrap();
    assert_eq!(
        persistence::read(&path, DecodeLimits::default())
            .unwrap()
            .levels()
            .count(),
        0
    );
    std::fs::remove_dir_all(dir).unwrap();
}

struct Terrain {
    unavailable: BTreeSet<[i32; 2]>,
}
impl ResidencyHost for Terrain {
    fn chunk_ready(&self, x: i32, z: i32) -> bool {
        !self.unavailable.contains(&[x, z])
    }
}

#[test]
fn holding_checks_negative_margin_bounds_and_unavailable_terrain_once() {
    let host = Terrain {
        unavailable: BTreeSet::from([[-2, -2]]),
    };
    assert!(
        !persistence::ready_chunks(([-16.0, 0.0, -16.0], [-15.0, 1.0, -15.0]), &host, 100).unwrap()
    );
    assert!(persistence::ready_chunks(([0.0, 0.0, 0.0], [15.0, 1.0, 15.0]), &host, 100).unwrap());
    assert_eq!(
        persistence::ready_chunks(([0.0, 0.0, 0.0], [15.0, 1.0, 15.0]), &host, 1),
        Err(Error::Capacity)
    );
    assert_eq!(
        persistence::ready_chunks(([f64::NAN, 0.0, 0.0], [1.0; 3]), &host, 100),
        Err(Error::InvalidInput)
    );
}

#[test]
fn dependency_reload_is_atomic_retains_uuid_and_applies_velocity_policy_once() {
    let mut a = Level::new(id(1));
    a.contents.blocks.insert(
        [0; 3],
        Block {
            state: "minecraft:stone".into(),
            block_entity: None,
            extension: Payload::default(),
        },
    );
    a.dependencies.insert(id(9));
    a.motion.linear = [1.0, 0.0, 0.0];
    a.motion.angular = [0.1, 0.0, 0.0];
    let mut b = a.clone();
    b.id = id(9);
    b.plot = 1;
    b.dependencies.clear();
    b.motion.pose.position = [32.0, 0.0, 32.0];
    let mut r = Registry::restore(
        Config::default(),
        vec![(a, Residency::Holding), (b, Residency::Holding)],
    )
    .unwrap();
    let h = r.handle(id(1)).unwrap();
    let before = persistence::encode(&r).unwrap();
    let blocked = Terrain {
        unavailable: BTreeSet::from([[2, 2]]),
    };
    assert!(
        !persistence::reload_with(
            &mut r,
            h,
            &blocked,
            100,
            VelocityPolicy::JavaFields {
                retained_on_load: 0.9
            }
        )
        .unwrap()
    );
    assert_eq!(persistence::encode(&r).unwrap(), before);
    let ready = Terrain {
        unavailable: BTreeSet::new(),
    };
    assert!(
        persistence::reload_with(
            &mut r,
            h,
            &ready,
            100,
            VelocityPolicy::JavaFields {
                retained_on_load: 0.9
            }
        )
        .unwrap()
    );
    assert_eq!(r.get(h).unwrap().motion.linear, [0.9, 0.0, 0.0]);
    assert_eq!(r.get(h).unwrap().motion.angular, [0.0; 3]);
    assert_eq!(
        r.residency(r.handle(id(9)).unwrap()).unwrap(),
        Residency::Live
    );
    persistence::reload_with(
        &mut r,
        h,
        &ready,
        100,
        VelocityPolicy::JavaFields {
            retained_on_load: 0.9,
        },
    )
    .unwrap();
    assert_eq!(r.get(h).unwrap().motion.linear, [0.9, 0.0, 0.0]);
}

#[test]
fn saved_velocity_policy_keeps_questionable_original_angular_threshold_explicit() {
    let mut motion = Motion {
        linear: [0.1, 0.0, 0.0],
        angular: [0.13, 0.0, 0.0],
        ..Motion::default()
    };
    VelocityPolicy::JavaFields {
        retained_on_load: 0.9,
    }
    .apply(&mut motion)
    .unwrap();
    assert_eq!(motion.linear, [0.0; 3]);
    assert_eq!(motion.angular, [0.0; 3]);
    motion.linear = [0.1001, 0.0, 0.0];
    motion.angular = [0.14, 0.0, 0.0];
    VelocityPolicy::JavaFields {
        retained_on_load: 0.9,
    }
    .apply(&mut motion)
    .unwrap();
    assert_eq!(motion.linear[0], 0.1001 * 0.9);
    assert_eq!(motion.angular[0], 0.14 * 0.9);
    assert_eq!(
        VelocityPolicy::JavaFields {
            retained_on_load: f64::NAN
        }
        .apply(&mut motion),
        Err(Error::InvalidInput)
    );
}

#[test]
fn saved_velocity_fields_preserve_joml_right_associated_threshold_decision() {
    let linear = [0.1, 7e-10, 7e-10];
    // A left-associated sum rounds both tiny terms away separately. JOML adds
    // them first, crossing the strict source threshold by one representable bit.
    assert_eq!(
        linear[0] * linear[0] + linear[1] * linear[1] + linear[2] * linear[2],
        0.1 * 0.1
    );
    assert_eq!(
        persistence::java_saved_velocities(linear, [0.0; 3]),
        Ok((Some(linear), None))
    );
    assert_eq!(
        persistence::java_saved_velocities([0.1, 0.0, 0.0], [0.0; 3]),
        Ok((None, None))
    );
    assert_eq!(
        persistence::java_saved_velocities([f64::NAN, 0.0, 0.0], [0.0; 3]),
        Err(Error::InvalidInput)
    );
    assert_eq!(
        persistence::java_saved_velocities([0.0; 3], [f64::INFINITY, 0.0, 0.0]),
        Err(Error::InvalidInput)
    );
}

#[test]
fn transfer_roundtrip_metamorphism_preserves_all_record_multisets() {
    for offset in [[-32, 1, -16], [16, -4, 32], [1, 2, 3], [0; 3]] {
        let mut source = contents();
        let initial = source.clone();
        let mut target = Contents::default();
        let positions: Vec<_> = source.blocks.keys().copied().collect();
        let plan = transfer::plan(&source, &target, &positions, offset, 100).unwrap();
        transfer::move_contents(&mut source, &mut target, &plan, &mut PreservePayloads).unwrap();
        let positions: Vec<_> = target.blocks.keys().copied().collect();
        let reverse =
            transfer::plan(&target, &source, &positions, offset.map(|n| -n), 100).unwrap();
        transfer::move_contents(&mut target, &mut source, &reverse, &mut PreservePayloads).unwrap();
        assert_eq!(source, initial);
        assert_eq!(target, Contents::default());
    }
}
