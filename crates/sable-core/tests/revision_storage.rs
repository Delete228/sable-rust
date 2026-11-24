//! Public-API regressions for external Phase 2 restoration/storage findings.
use sable_core::{
    persistence::{self, DecodeLimits},
    *,
};

fn id(n: u8) -> Id {
    Id([n; 16])
}
fn block() -> Block {
    Block {
        state: "minecraft:blackstone".into(),
        block_entity: Some(Payload {
            format: "inventory-v1".into(),
            bytes: vec![0, 255, 64, 3],
        }),
        extension: Payload::default(),
    }
}
fn saved_level(revision: u64) -> Level {
    let mut level = Level::new(id(1));
    level.revision = revision;
    level.contents.blocks.insert([0; 3], block());
    level
}

#[test]
fn restored_changes_describe_saved_state_then_every_mutation_revision() {
    let mut registry =
        Registry::restore(Config::default(), vec![(saved_level(7), Residency::Live)]).unwrap();
    let handle = registry.handle(id(1)).unwrap();
    assert_eq!(registry.get(handle).unwrap().revision, 7);
    assert_eq!(
        registry.drain_changes(),
        vec![Change {
            id: id(1),
            revision: 7,
            kind: ChangeKind::Created,
        }]
    );
    registry.edit(handle, [1, 0, 0], Some(block())).unwrap();
    registry.unload(handle).unwrap();
    struct Ready;
    impl persistence::ResidencyHost for Ready {
        fn chunk_ready(&self, _: i32, _: i32) -> bool {
            true
        }
    }
    assert!(
        persistence::reload_with(
            &mut registry,
            handle,
            &Ready,
            100,
            persistence::VelocityPolicy::Exact,
        )
        .unwrap()
    );
    assert_eq!(registry.get(handle).unwrap().revision, 10);
    assert_eq!(
        registry.drain_changes(),
        vec![
            Change {
                id: id(1),
                revision: 8,
                kind: ChangeKind::Edited,
            },
            Change {
                id: id(1),
                revision: 9,
                kind: ChangeKind::Unloaded,
            },
            Change {
                id: id(1),
                revision: 10,
                kind: ChangeKind::Loaded,
            },
        ]
    );
}

#[test]
fn reachable_exhausted_revision_roundtrips_and_refused_mutation_is_atomic() {
    let mut registry = Registry::restore(
        Config::default(),
        vec![(saved_level(u64::MAX - 1), Residency::Live)],
    )
    .unwrap();
    let handle = registry.handle(id(1)).unwrap();
    registry.drain_changes();
    registry.edit(handle, [1, 0, 0], Some(block())).unwrap();
    assert_eq!(registry.get(handle).unwrap().revision, u64::MAX);
    assert_eq!(
        registry.drain_changes(),
        vec![Change {
            id: id(1),
            revision: u64::MAX,
            kind: ChangeKind::Edited,
        }]
    );
    let bytes = persistence::encode(&registry).unwrap();
    let mut restored = persistence::decode(&bytes, DecodeLimits::default()).unwrap();
    let restored_handle = restored.handle(id(1)).unwrap();
    assert_eq!(restored.get(handle), Err(Error::StaleHandle));
    assert_eq!(restored.get(restored_handle).unwrap().revision, u64::MAX);
    assert_eq!(
        restored.drain_changes(),
        vec![Change {
            id: id(1),
            revision: u64::MAX,
            kind: ChangeKind::Created,
        }]
    );
    assert_eq!(persistence::encode(&restored).unwrap(), bytes);
    assert_eq!(
        restored.edit(restored_handle, [0; 3], None),
        Err(Error::Capacity)
    );
    assert_eq!(restored.unload(restored_handle), Err(Error::Capacity));
    assert_eq!(
        restored.residency(restored_handle).unwrap(),
        Residency::Live
    );
    assert!(restored.drain_changes().is_empty());
    assert_eq!(persistence::encode(&restored).unwrap(), bytes);

    // New creation keeps the established increment rule, independently of
    // restoring an already existing saved revision.
    let mut empty = Registry::new(Config::default()).unwrap();
    assert_eq!(empty.insert(saved_level(u64::MAX)), Err(Error::Capacity));
    assert_eq!(empty.levels().count(), 0);
    assert!(empty.drain_changes().is_empty());
    let inserted = empty.insert(saved_level(7)).unwrap();
    assert_eq!(empty.get(inserted).unwrap().revision, 8);
    assert_eq!(empty.drain_changes()[0].revision, 8);
}

#[test]
fn restored_zero_and_holding_revisions_are_not_advanced() {
    let mut level = saved_level(0);
    level.plot = 128;
    let mut registry =
        Registry::restore(Config::default(), vec![(level, Residency::Holding)]).unwrap();
    let handle = registry.handle(id(1)).unwrap();
    assert_eq!(registry.get(handle).unwrap().revision, 0);
    assert_eq!(registry.residency(handle).unwrap(), Residency::Holding);
    assert_eq!(registry.drain_changes()[0].revision, 0);
}

#[cfg(unix)]
#[test]
fn unix_atomic_save_permissions_under_umask_022() {
    const CHILD_ENV: &str = "SABLE_PHASE2_PERMISSION_TEST_CHILD";
    if std::env::var_os(CHILD_ENV).is_none() {
        // umask is process-global. Isolate it in a child so other tests and
        // callers never observe a temporary change to their creation policy.
        let output = std::process::Command::new("/bin/sh")
            .args([
                "-c",
                "umask 022; exec \"$1\" --exact unix_atomic_save_permissions_under_umask_022 --nocapture",
                "sable-permission-test",
            ])
            .arg(std::env::current_exe().unwrap())
            .env(CHILD_ENV, "1")
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "child failed: stdout={} stderr={}",
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        );
        return;
    }
    use std::os::unix::fs::PermissionsExt;
    let directory = std::env::temp_dir().join(format!(
        "sable-phase2-permission-regression-{}",
        std::process::id()
    ));
    std::fs::create_dir(&directory).unwrap();
    let path = directory.join("save.bin");
    let registry =
        Registry::restore(Config::default(), vec![(saved_level(7), Residency::Live)]).unwrap();
    let bytes = persistence::encode(&registry).unwrap();
    // A fresh child has no previous writer sequence. A collision must refuse
    // without deleting a temporary that this call never owned.
    let collision = directory.join(format!(".save.bin.pending-{}-0", std::process::id()));
    std::fs::write(&collision, b"pre-existing-unowned-temporary").unwrap();
    assert_eq!(
        persistence::write_atomic(&path, &bytes, persistence::WriteFault::None),
        Err(Error::Io)
    );
    assert_eq!(
        std::fs::read(&collision).unwrap(),
        b"pre-existing-unowned-temporary"
    );
    assert!(!path.exists());
    std::fs::remove_file(collision).unwrap();
    persistence::write_atomic(&path, &bytes, persistence::WriteFault::None).unwrap();
    let mode = || std::fs::metadata(&path).unwrap().permissions().mode() & 0o777;
    assert_eq!(
        mode(),
        0o600,
        "a new save must not inherit umask-022 mode0644"
    );

    for (i, permissions) in [0o600, 0o640].into_iter().enumerate() {
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(permissions)).unwrap();
        let old = std::fs::read(&path).unwrap();
        for fault in [
            persistence::WriteFault::BeforeWrite,
            persistence::WriteFault::AfterWrite,
            persistence::WriteFault::AfterFileSync,
            persistence::WriteFault::BeforeRename,
        ] {
            assert_eq!(
                persistence::write_atomic(&path, b"interrupted-data", fault),
                Err(Error::Interrupted)
            );
            assert_eq!(mode(), permissions);
            assert_eq!(std::fs::read(&path).unwrap(), old);
            assert_eq!(std::fs::read_dir(&directory).unwrap().count(), 1);
        }
        let next_registry = Registry::restore(
            Config::default(),
            vec![(saved_level(8 + i as u64), Residency::Live)],
        )
        .unwrap();
        let next_bytes = persistence::encode(&next_registry).unwrap();
        assert_ne!(next_bytes, old);
        persistence::write_atomic(&path, &next_bytes, persistence::WriteFault::None).unwrap();
        assert_eq!(mode(), permissions);
        assert_eq!(std::fs::read(&path).unwrap(), next_bytes);
        let restored = persistence::read(&path, DecodeLimits::default()).unwrap();
        let handle = restored.handle(id(1)).unwrap();
        assert_eq!(restored.get(handle).unwrap().revision, 8 + i as u64);
    }
    std::fs::remove_dir_all(directory).unwrap();
}
