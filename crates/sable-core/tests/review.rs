//! Independent adverse inputs found while reviewing lifecycle and persistence.
//! These are conservation/safety checks, not Java differential expectations.

use std::collections::BTreeSet;

use sable_core::lifecycle::SplitService;
use sable_core::mass::{MassRegistry, Properties};
use sable_core::persistence;
#[cfg(feature = "native-physics")]
use sable_core::physics::PhysicsOwner;
use sable_core::split::SplitPlan;
use sable_core::transfer::{PreservePayloads, TransferEvent, TransferHost};
use sable_core::*;

fn id(n: u8) -> Id {
    Id([n; 16])
}

fn block() -> Block {
    Block {
        state: "minecraft:blackstone".into(),
        block_entity: Some(Payload {
            format: "test/inventory".into(),
            bytes: vec![0xff, 0x80, 64, 0],
        }),
        extension: Payload::default(),
    }
}

fn level(n: u8, positions: &[Pos]) -> Level {
    let mut level = Level::new(id(n));
    level.contents.blocks = positions.iter().map(|p| (*p, block())).collect();
    level
}

#[test]
fn cross_owner_entity_tick_identity_conflicts_are_atomic() {
    let mut registry = Registry::new(Config::default()).unwrap();
    let mut a = level(1, &[[0; 3]]);
    a.contents.entities.insert(
        id(20),
        Entity {
            id: id(20),
            anchor: [0; 3],
            position: [0.5; 3],
            payload: Payload::default(),
        },
    );
    let handle = registry.insert(a).unwrap();
    registry.drain_changes();
    let before = persistence::encode(&registry).unwrap();
    let mut b = level(2, &[[0; 3]]);
    b.contents.ticks.push(ScheduledTick {
        id: id(20),
        position: [0; 3],
        kind: TickKind::Fluid,
        due: 1,
        priority: 0,
        payload: Payload::default(),
    });
    assert_eq!(registry.insert(b), Err(Error::DuplicateId));
    assert_eq!(persistence::encode(&registry).unwrap(), before);
    assert!(registry.drain_changes().is_empty());
    assert!(registry.get(handle).is_ok());
}

#[test]
fn old_operations_cannot_reach_same_uuid_in_reused_plot() {
    let masses = MassRegistry::pinned_full_blocks();
    let mut registry = Registry::new(Config::default()).unwrap();
    let old = registry.insert(level(1, &[[0; 3], [1, 0, 0]])).unwrap();
    let revision = registry.get(old).unwrap().revision;
    let plan = SplitPlan {
        source_epoch: 0,
        children: vec![BTreeSet::from([[1, 0, 0]])],
        retained: None,
    };
    let old_plot = registry.get(old).unwrap().plot;
    let removed = registry.delete(old).unwrap();
    let new = registry.insert(level(1, &[[0; 3], [1, 0, 0]])).unwrap();
    assert_ne!(old, new);
    assert_eq!(registry.get(new).unwrap().plot, old_plot);
    assert_eq!(removed.contents.blocks.len(), 2);
    let before = persistence::encode(&registry).unwrap();
    assert_eq!(registry.edit(old, [0; 3], None), Err(Error::StaleHandle));
    assert_eq!(registry.unload(old), Err(Error::StaleHandle));
    assert_eq!(registry.delete(old), Err(Error::StaleHandle));
    assert_eq!(
        registry.apply_split(
            old,
            revision,
            &plan,
            &[id(2)],
            &mut PreservePayloads,
            &masses
        ),
        Err(Error::StaleHandle)
    );
    assert_eq!(persistence::encode(&registry).unwrap(), before);
}

#[test]
fn out_of_plot_transfer_and_overlapping_split_leave_every_payload_owned() {
    let masses = MassRegistry::pinned_full_blocks();
    let mut registry = Registry::new(Config::default()).unwrap();
    let a = registry.insert(level(1, &[[0; 3], [1, 0, 0]])).unwrap();
    let b = registry.insert(level(2, &[[10, 0, 0]])).unwrap();
    let before = persistence::encode(&registry).unwrap();
    assert_eq!(
        registry.transfer(a, b, &[[0; 3]], [-1, 0, 0], &mut PreservePayloads),
        Err(Error::InvalidCoordinate)
    );
    let overlap = SplitPlan {
        source_epoch: 0,
        children: vec![BTreeSet::from([[1, 0, 0]]), BTreeSet::from([[1, 0, 0]])],
        retained: None,
    };
    assert_eq!(
        registry.apply_split(
            a,
            registry.get(a).unwrap().revision,
            &overlap,
            &[id(3), id(4)],
            &mut PreservePayloads,
            &masses,
        ),
        Err(Error::InvalidInput)
    );
    assert_eq!(persistence::encode(&registry).unwrap(), before);
}

struct RefuseSecondPayload {
    seen: usize,
}

impl TransferHost for RefuseSecondPayload {
    fn rewrite_payload(&mut self, p: &Payload, _from: Pos, _to: Pos) -> Result<Payload> {
        self.seen += 1;
        if self.seen == 2 {
            Err(Error::HostRefused)
        } else {
            Ok(p.clone())
        }
    }

    fn prepare_event(&mut self, _event: &TransferEvent) -> Result<()> {
        Ok(())
    }
}

#[test]
fn failed_child_creation_rolls_back_uuid_plot_and_inventory() {
    let masses = MassRegistry::pinned_full_blocks();
    let mut registry = Registry::new(Config::default()).unwrap();
    let parent = registry.insert(level(1, &[[0; 3], [1, 0, 0]])).unwrap();
    let plan = SplitPlan {
        source_epoch: 0,
        children: vec![BTreeSet::from([[1, 0, 0]])],
        retained: None,
    };
    let before = persistence::encode(&registry).unwrap();
    assert_eq!(
        registry.apply_split(
            parent,
            registry.get(parent).unwrap().revision,
            &plan,
            &[id(2)],
            &mut RefuseSecondPayload { seen: 0 },
            &masses,
        ),
        Err(Error::HostRefused)
    );
    assert_eq!(persistence::encode(&registry).unwrap(), before);
    assert_eq!(registry.handle(id(2)), Err(Error::StaleHandle));
    let child = registry
        .apply_split(
            parent,
            registry.get(parent).unwrap().revision,
            &plan,
            &[id(2)],
            &mut PreservePayloads,
            &masses,
        )
        .unwrap()
        .children[0];
    assert_eq!(registry.get(child).unwrap().plot, 128);
    assert_eq!(
        registry.get(child).unwrap().contents.blocks[&[1, 0, 0]].block_entity,
        block().block_entity
    );
}

#[test]
fn zero_mass_child_is_refused_with_inventory_intact() {
    let mut masses = MassRegistry::pinned_full_blocks();
    masses
        .register(
            "test:massless".into(),
            Properties {
                solid: true,
                full: true,
                mass: 0.0,
                center: [0.5; 3],
                inertia: None,
                boxes: vec![[0.0, 0.0, 0.0, 1.0, 1.0, 1.0]],
            },
        )
        .unwrap();
    let mut l = level(1, &[[0; 3], [1, 0, 0]]);
    l.contents.blocks.get_mut(&[1, 0, 0]).unwrap().state = "test:massless".into();
    let mut registry = Registry::new(Config::default()).unwrap();
    let parent = registry.insert(l).unwrap();
    let before = persistence::encode(&registry).unwrap();
    let plan = SplitPlan {
        source_epoch: 0,
        children: vec![BTreeSet::from([[1, 0, 0]])],
        retained: None,
    };
    assert_eq!(
        registry.apply_split(
            parent,
            registry.get(parent).unwrap().revision,
            &plan,
            &[id(2)],
            &mut PreservePayloads,
            &masses
        ),
        Err(Error::InvalidInput)
    );
    assert_eq!(persistence::encode(&registry).unwrap(), before);
}

#[test]
#[cfg(feature = "native-physics")]
fn physics_preflight_refusal_preserves_existing_realization() {
    let mut masses = MassRegistry::pinned_full_blocks();
    let mut registry = Registry::new(Config::default()).unwrap();
    let handle = registry.insert(level(1, &[[0; 3]])).unwrap();
    let mut physics = PhysicsOwner::new([0.0; 3]).unwrap();
    physics.sync(&registry, &masses).unwrap();
    let original_body = physics.body(id(1)).unwrap();
    let original_pose = physics.engine.pose(original_body).unwrap();
    assert_eq!(physics.counts(), (1, 1));

    masses
        .register(
            "test:f64-only-box".into(),
            Properties {
                solid: true,
                full: true,
                mass: 2.0,
                center: [0.5; 3],
                inertia: None,
                boxes: vec![[0.0, 0.0, 0.0, 1.0e300, 1.0, 1.0]],
            },
        )
        .unwrap();
    let mut changed = block();
    changed.state = "test:f64-only-box".into();
    registry.edit(handle, [0; 3], Some(changed)).unwrap();
    assert!(physics.sync(&registry, &masses).is_err());
    assert_eq!(physics.counts(), (1, 1));
    assert_eq!(physics.body(id(1)), Some(original_body));
    assert_eq!(physics.engine.pose(original_body).unwrap(), original_pose);

    registry.edit(handle, [0; 3], Some(block())).unwrap();
    physics.sync(&registry, &masses).unwrap();
    let replacement = physics.body(id(1)).unwrap();
    assert_ne!(replacement, original_body);
    assert!(physics.engine.pose(original_body).is_err());
    assert_eq!(physics.counts(), (1, 1));
    physics.engine.step(physics.scene, 0.05).unwrap();
    registry.delete(handle).unwrap();
    physics.sync(&registry, &masses).unwrap();
    assert_eq!(physics.counts(), (0, 0));
    assert!(physics.engine.pose(replacement).is_err());
}

#[test]
fn stale_pending_split_after_topology_edit_cannot_stall_future_work() {
    let masses = MassRegistry::pinned_full_blocks();
    let mut registry = Registry::new(Config::default()).unwrap();
    let positions: Vec<_> = (10..=14).map(|x| [x, 0, 10]).collect();
    let parent = registry.insert(level(1, &positions)).unwrap();
    let mut service = SplitService::default();
    assert!(
        service
            .tick(&registry, parent, &masses, 200, true)
            .unwrap()
            .is_none()
    );
    registry.edit(parent, [12, 0, 10], None).unwrap();
    let pending = service
        .tick(&registry, parent, &masses, 200, true)
        .unwrap()
        .unwrap();
    registry.edit(parent, [12, 0, 10], Some(block())).unwrap();
    service.observe(&registry, parent, &masses).unwrap();
    assert_eq!(
        service.commit(
            &mut registry,
            pending,
            &[id(2)],
            &mut PreservePayloads,
            &masses
        ),
        Err(Error::StaleHandle)
    );
    assert!(
        service
            .tick(&registry, parent, &masses, 200, true)
            .unwrap()
            .is_none()
    );
    registry.edit(parent, [12, 0, 10], None).unwrap();
    let fresh = service
        .tick(&registry, parent, &masses, 200, true)
        .unwrap()
        .expect("stale plan cancellation must resume splitting");
    let children = service
        .commit(
            &mut registry,
            fresh,
            &[id(2)],
            &mut PreservePayloads,
            &masses,
        )
        .unwrap();
    assert_eq!(children.children.len(), 1);
    assert!(!children.events.is_empty());
    assert_eq!(children.events[0].0, children.children[0]);
    assert_eq!(
        registry
            .levels()
            .map(|(l, _)| l.contents.blocks.len())
            .sum::<usize>(),
        4
    );
}

#[test]
fn nonair_massless_bridge_still_participates_in_heat_connectivity() {
    let mut masses = MassRegistry::pinned_full_blocks();
    masses
        .register(
            "test:nonair-nonsolid".into(),
            Properties {
                solid: false,
                full: false,
                mass: 0.0,
                center: [0.5; 3],
                inertia: None,
                boxes: Vec::new(),
            },
        )
        .unwrap();
    let positions: Vec<_> = (10..=14).map(|x| [x, 0, 10]).collect();
    let mut l = level(1, &positions);
    l.contents.blocks.get_mut(&[12, 0, 10]).unwrap().state = "test:nonair-nonsolid".into();
    let mut registry = Registry::new(Config::default()).unwrap();
    let parent = registry.insert(l).unwrap();
    let mut service = SplitService::default();
    assert!(
        service
            .tick(&registry, parent, &masses, 200, true)
            .unwrap()
            .is_none()
    );
    registry.edit(parent, [11, 0, 10], None).unwrap();
    let pending = service
        .tick(&registry, parent, &masses, 200, true)
        .unwrap()
        .unwrap();
    assert_eq!(
        pending.plan.children,
        vec![BTreeSet::from([[12, 0, 10], [13, 0, 10], [14, 0, 10]])]
    );
    let child = service
        .commit(
            &mut registry,
            pending,
            &[id(2)],
            &mut PreservePayloads,
            &masses,
        )
        .unwrap()
        .children[0];
    assert!(
        registry
            .get(child)
            .unwrap()
            .contents
            .blocks
            .contains_key(&[12, 0, 10])
    );
}
