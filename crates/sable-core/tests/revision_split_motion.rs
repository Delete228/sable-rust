//! R5 contract comparison: unchanged original kick/mass/pose helpers with an
//! explicitly authored frozen-input driver, not real Java assembleBlocks/game.
//! Expected bits were captured before this independent Rust test was written.
use sable_core::{
    mass::MassRegistry,
    persistence,
    split::SplitPlan,
    transfer::{PreservePayloads, TransferEvent, TransferHost},
    *,
};
use std::collections::{BTreeMap, BTreeSet};

const INPUT: &str = include_str!("../../../tools/phase2/split_motion/cases.v1.tsv");
const EXPECTED: &str =
    include_str!("../../../tools/phase2/split_motion/observations/expected.v1.tsv");

struct Case {
    id: String,
    values: [f64; 11],
    order: [i32; 2],
}

fn cases() -> Vec<Case> {
    let mut lines = INPUT.lines();
    assert_eq!(lines.next(), Some("SABLE_SPLIT_MOTION\t1"));
    let mut ids = BTreeSet::new();
    let cases: Vec<_> = lines
        .map(|line| {
            let fields: Vec<_> = line.split('\t').collect();
            assert_eq!(fields.len(), 14);
            assert!(ids.insert(fields[0]));
            let values: [f64; 11] = std::array::from_fn(|i| fields[i + 1].parse().unwrap());
            assert!(values.iter().all(|v| v.is_finite()));
            assert!((values[0] * values[0] + values[1] * values[1] - 1.).abs() < 1e-12);
            let order = [fields[12].parse().unwrap(), fields[13].parse().unwrap()];
            assert_eq!(BTreeSet::from(order), BTreeSet::from([30, 50]));
            Case {
                id: fields[0].into(),
                values,
                order,
            }
        })
        .collect();
    assert_eq!(cases.len(), 6);
    cases
}

fn expected() -> BTreeMap<(String, String, i32), [u64; 9]> {
    let mut lines = EXPECTED.lines();
    assert_eq!(lines.next(), Some("SABLE_SPLIT_MOTION_BITS\t1"));
    let mut records = BTreeMap::new();
    for line in lines {
        let fields: Vec<_> = line.split('\t').collect();
        assert_eq!(fields.len(), 12);
        assert!(matches!(fields[1], "frozen" | "sequential"));
        let x: i32 = fields[2].parse().unwrap();
        assert!(matches!(x, 30 | 50));
        let bits = std::array::from_fn(|i| {
            assert_eq!(fields[i + 3].len(), 16);
            u64::from_str_radix(fields[i + 3], 16).unwrap()
        });
        assert!(
            records
                .insert((fields[0].into(), fields[1].into(), x), bits)
                .is_none()
        );
    }
    assert_eq!(records.len(), 24);
    records
}

fn id(n: u8) -> Id {
    Id([n; 16])
}

fn child_id(x: i32) -> Id {
    match x {
        30 => id(2),
        50 => id(3),
        _ => panic!("outside fixture partition"),
    }
}

fn registry(case: &Case, masses: &MassRegistry) -> (Registry, Handle) {
    let mut level = Level::new(id(1));
    for x in [10, 11, 30, 50] {
        level.contents.blocks.insert(
            [x, 0, 10],
            Block {
                state: "minecraft:blackstone".into(),
                block_entity: Some(Payload {
                    format: "r5-coordinate-free-inventory".into(),
                    bytes: vec![x as u8, 64, 9],
                }),
                extension: Payload::default(),
            },
        );
    }
    for x in [30, 50] {
        let entity = Entity {
            id: id(100 + x as u8),
            anchor: [x, 0, 10],
            position: [f64::from(x) + 0.25, 0.5, 10.5],
            payload: Payload {
                format: "r5-retained".into(),
                bytes: vec![x as u8, 7],
            },
        };
        level.contents.entities.insert(entity.id, entity);
        level.contents.ticks.push(ScheduledTick {
            id: id(200 + x as u8),
            position: [x, 0, 10],
            kind: if x == 30 {
                TickKind::Block
            } else {
                TickKind::Fluid
            },
            due: 1000 + i64::from(x),
            priority: 3,
            payload: Payload {
                format: "r5-scheduled".into(),
                bytes: vec![x as u8, 8],
            },
        });
    }
    let v = case.values;
    level.motion = Motion {
        pose: Pose {
            position: [v[2], v[3], v[4]],
            rotation: [0., 0., v[0], v[1]],
            pivot: masses.compute(&level.contents).unwrap().center,
            scale: [1.; 3],
        },
        linear: [v[5], v[6], v[7]],
        angular: [v[8], v[9], v[10]],
    };
    let mut registry = Registry::new(Config::default()).unwrap();
    let parent = registry.insert(level).unwrap();
    (registry, parent)
}

fn plan(case: &Case) -> SplitPlan {
    SplitPlan {
        source_epoch: 0,
        children: case.order.map(|x| BTreeSet::from([[x, 0, 10]])).to_vec(),
        retained: Some(BTreeSet::from([[10, 0, 10], [11, 0, 10]])),
    }
}

#[test]
fn frozen_original_helper_channels_match_exactly_and_child_order_does_not_change_motion() {
    let originals = expected();
    let masses = MassRegistry::pinned_full_blocks();
    let mut by_family = BTreeMap::new();
    for case in cases() {
        let (mut registry, parent) = registry(&case, &masses);
        let before = registry.get(parent).unwrap().clone();
        let ids = case.order.map(child_id);
        let outcome = registry
            .apply_split(
                parent,
                before.revision,
                &plan(&case),
                &ids,
                &mut PreservePayloads,
                &masses,
            )
            .unwrap();
        assert_eq!(outcome.children.len(), 2);
        assert_eq!(outcome.events.len(), 2);
        let mut motions = BTreeMap::new();
        for (index, handle) in outcome.children.iter().enumerate() {
            let x = case.order[index];
            let child = registry.get(*handle).unwrap();
            assert_eq!(child.id, child_id(x));
            assert_eq!(child.parent, Some(before.id));
            assert_eq!(outcome.events[index].0, *handle);
            assert!(!outcome.events[index].1.is_empty());
            let world = child
                .motion
                .pose
                .transform(masses.compute(&child.contents).unwrap().center);
            let actual: [u64; 9] = std::array::from_fn(|i| {
                if i < 3 {
                    world[i]
                } else if i < 6 {
                    child.motion.linear[i - 3]
                } else {
                    child.motion.angular[i - 6]
                }
                .to_bits()
            });
            assert_eq!(actual, originals[&(case.id.clone(), "frozen".into(), x)]);
            if index == 1 {
                assert_ne!(
                    actual,
                    originals[&(case.id.clone(), "sequential".into(), x)],
                    "the deliberate batch policy must not be relabeled sequential parity"
                );
            }
            assert_eq!(child.contents.blocks.len(), 1);
            assert_eq!(
                child.contents.blocks[&[x, 0, 10]],
                before.contents.blocks[&[x, 0, 10]]
            );
            assert_eq!(child.contents.entities.len(), 1);
            let entity_id = id(100 + x as u8);
            assert_eq!(
                child.contents.entities[&entity_id],
                before.contents.entities[&entity_id]
            );
            assert_eq!(child.contents.ticks.len(), 1);
            assert_eq!(
                child.contents.ticks[0],
                *before
                    .contents
                    .ticks
                    .iter()
                    .find(|tick| tick.position == [x, 0, 10])
                    .unwrap()
            );
            motions.insert(x, actual);
        }
        assert_eq!(registry.get(parent).unwrap().motion, before.motion);
        assert_eq!(registry.levels().count(), 3);
        assert_eq!(
            registry
                .levels()
                .map(|(level, _)| level.contents.record_count())
                .sum::<usize>(),
            before.contents.record_count()
        );
        assert_eq!(registry.get(parent).unwrap().contents.blocks.len(), 2);
        assert!(registry.get(parent).unwrap().contents.entities.is_empty());
        assert!(registry.get(parent).unwrap().contents.ticks.is_empty());
        registry.validate().unwrap();
        let family = case.id.rsplit_once('-').unwrap().0.to_owned();
        if let Some(previous) = by_family.insert(family, motions.clone()) {
            assert_eq!(previous, motions);
        }
    }
    let identity = &originals[&("identity-forward".into(), "frozen".into(), 50)];
    let sequential = &originals[&("identity-forward".into(), "sequential".into(), 50)];
    assert_eq!(f64::from_bits(identity[4]), 24.75);
    assert_eq!(f64::from_bits(sequential[4]), 26.333333333333336);
}

#[test]
fn second_child_preparation_failure_preserves_parent_records_revisions_and_identity() {
    struct RefuseSecondChild;
    impl TransferHost for RefuseSecondChild {
        fn rewrite_payload(&mut self, payload: &Payload, _: Pos, _: Pos) -> Result<Payload> {
            Ok(payload.clone())
        }
        fn prepare_event(&mut self, event: &TransferEvent) -> Result<()> {
            if matches!(
                event,
                TransferEvent::BeforeMove {
                    from: [50, 0, 10],
                    ..
                }
            ) {
                return Err(Error::HostRefused);
            }
            Ok(())
        }
    }
    let masses = MassRegistry::pinned_full_blocks();
    let case = cases().remove(0);
    let (mut registry, parent) = registry(&case, &masses);
    registry.drain_changes();
    let before = persistence::encode(&registry).unwrap();
    let revision = registry.get(parent).unwrap().revision;
    let ids = case.order.map(child_id);
    assert_eq!(
        registry.apply_split(
            parent,
            revision,
            &plan(&case),
            &ids,
            &mut RefuseSecondChild,
            &masses,
        ),
        Err(Error::HostRefused)
    );
    assert_eq!(persistence::encode(&registry).unwrap(), before);
    assert_eq!(registry.handle(id(1)).unwrap(), parent);
    assert_eq!(registry.handle(id(2)), Err(Error::StaleHandle));
    assert_eq!(registry.handle(id(3)), Err(Error::StaleHandle));
    assert!(registry.drain_changes().is_empty());
    let result = registry
        .apply_split(
            parent,
            revision,
            &plan(&case),
            &ids,
            &mut PreservePayloads,
            &masses,
        )
        .unwrap();
    assert_eq!(result.children.len(), 2);
    assert_eq!(registry.levels().count(), 3);
}
