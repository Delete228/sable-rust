//! Independent invariants and source-derived algorithm regressions. The Java
//! differential fixture is reported separately in docs/phase-2-reference.md.

use std::collections::BTreeSet;

use sable_core::Pos;
use sable_core::split::{DEFAULT_HEAT_STEPS, HeatState, SplitPlan, SplitTracker};

fn start(order: &[Pos]) -> (SplitTracker, BTreeSet<Pos>) {
    let mut tracker = SplitTracker::new();
    let mut solids = BTreeSet::new();
    for &pos in order {
        assert!(solids.insert(pos));
        tracker.on_added(pos);
    }
    (tracker, solids)
}

fn settle(tracker: &mut SplitTracker, solids: &BTreeSet<Pos>) {
    for _ in 0..100 {
        assert!(tracker.tick(solids, DEFAULT_HEAT_STEPS, true).is_empty());
        let snapshot = tracker.snapshot();
        if snapshot.state == HeatState::Clearing
            && snapshot.floodfill.is_empty()
            && snapshot.removed.is_empty()
            && snapshot.new_starts.is_empty()
            && snapshot.split_indices.is_empty()
        {
            return;
        }
    }
    panic!("heat did not settle in bounded work");
}

fn next_split(tracker: &mut SplitTracker, solids: &BTreeSet<Pos>) -> (usize, SplitPlan) {
    for tick in 1..=200 {
        let mut plans = tracker.tick(solids, 1, true);
        if let Some(plan) = plans.pop() {
            return (tick, plan);
        }
    }
    panic!("expected split did not complete in bounded work");
}

#[test]
fn bridge_removal_is_incremental_and_keeps_existing_root() {
    let order: Vec<_> = (-3..=3).map(|x| [x, 0, 0]).collect();
    let (mut tracker, mut solids) = start(&order);
    settle(&mut tracker, &solids);
    solids.remove(&[0, 0, 0]);
    tracker.on_removed([0, 0, 0]);
    let epoch = tracker.epoch();
    let (ticks, plan) = next_split(&mut tracker, &solids);
    assert_eq!(ticks, 5); // removal batch, 3 clearing pops, split decision
    assert_eq!(plan.source_epoch, epoch);
    assert_eq!(
        plan.children,
        vec![[[1, 0, 0], [2, 0, 0], [3, 0, 0]].into()]
    );
    assert_eq!(plan.retained, None);
    assert_eq!(solids.len(), 6); // planning never edits authoritative contents
    assert!(tracker.tick(&solids, 200, true).is_empty());
}

#[test]
fn all_components_select_unique_largest_and_documented_equal_tie() {
    for (order, expected) in [
        (
            vec![[0, 0, 0], [-1, 0, 0], [-2, 0, 0], [1, 0, 0]],
            vec![[-2, 0, 0], [-1, 0, 0]],
        ),
        (
            vec![[0, 0, 0], [-1, 0, 0], [-2, 0, 0], [1, 0, 0], [2, 0, 0]],
            vec![[-2, 0, 0], [-1, 0, 0]],
        ),
    ] {
        let (mut tracker, mut solids) = start(&order);
        settle(&mut tracker, &solids);
        solids.remove(&[0, 0, 0]);
        tracker.on_removed([0, 0, 0]);
        let (_, plan) = next_split(&mut tracker, &solids);
        let keep = plan.retained.expect("all components were candidates");
        assert_eq!(keep, expected.into_iter().collect());
        let mut union = keep;
        for child in plan.children {
            for pos in child {
                assert!(union.insert(pos), "child components overlap");
            }
        }
        assert_eq!(union, solids);
    }
}

#[test]
fn disabled_and_zero_budget_preserve_all_pending_work() {
    let (mut tracker, solids) = start(&[[0, 0, 0], [1, 0, 0]]);
    let before = tracker.snapshot();
    assert!(tracker.tick(&solids, 200, false).is_empty());
    assert_eq!(tracker.snapshot(), before);
    assert!(tracker.tick(&solids, 0, true).is_empty());
    assert_eq!(tracker.snapshot(), before);
    settle(&mut tracker, &solids);
}

#[test]
fn add_during_pending_removal_cancels_it_without_losing_heat() {
    let order: Vec<_> = (-2..=2).map(|x| [x, 0, 0]).collect();
    let (mut tracker, mut solids) = start(&order);
    settle(&mut tracker, &solids);
    let heat_before = tracker.snapshot().heat;
    solids.remove(&[0, 0, 0]);
    tracker.on_removed([0, 0, 0]);
    solids.insert([0, 0, 0]);
    tracker.on_added([0, 0, 0]);
    assert!(tracker.snapshot().removed.is_empty());
    assert_eq!(tracker.snapshot().heat, heat_before);
    settle(&mut tracker, &solids);
    assert_eq!(tracker.snapshot().solid_count, 5);
}

#[test]
fn face_and_edge_connect_but_three_axis_corner_does_not() {
    for neighbour in [[1, 0, 0], [1, 1, 0], [0, -1, -1]] {
        let (mut tracker, solids) = start(&[[0, 0, 0], neighbour]);
        settle(&mut tracker, &solids);
        assert_eq!(tracker.snapshot().heat.len(), 2);
    }
    let (mut tracker, solids) = start(&[[0, 0, 0], [1, 1, 1]]);
    let (_, plan) = next_split(&mut tracker, &solids);
    assert_eq!(plan.children, vec![[[1, 1, 1]].into()]);
}

#[test]
fn original_thin_air_after_completed_fill_rule_is_explicit() {
    let (mut tracker, mut solids) = start(&[[0, 0, 0]]);
    settle(&mut tracker, &solids);
    solids.insert([10, 0, 0]);
    tracker.on_added([10, 0, 0]);
    assert!(tracker.tick(&solids, 200, true).is_empty());
    assert_eq!(tracker.snapshot().solid_count, 2);
    assert!(!tracker.snapshot().heat.contains_key(&[10, 0, 0]));
}

#[test]
fn initial_thin_air_islands_share_reserved_split_index_zero() {
    let (mut tracker, solids) = start(&[[0, 0, 0], [10, 0, 0], [20, 0, 0]]);
    let (_, plan) = next_split(&mut tracker, &solids);
    // Java groups the two otherwise disconnected in-air additions into one
    // index-zero child. Generic connected-components splitting would differ.
    assert_eq!(plan.children, vec![[[10, 0, 0], [20, 0, 0]].into()]);
    assert_eq!(plan.retained, None);
}

#[test]
fn reconcile_discards_pending_work_and_never_reuses_plan_epoch() {
    let order: Vec<_> = (-3..=3).map(|x| [x, 0, 0]).collect();
    let (mut tracker, mut solids) = start(&order);
    settle(&mut tracker, &solids);
    solids.remove(&[0, 0, 0]);
    tracker.on_removed([0, 0, 0]);
    assert!(tracker.tick(&solids, 1, true).is_empty());
    solids.insert([0, 0, 0]);
    let epoch = tracker.epoch();
    tracker.reconcile(&solids);
    assert_ne!(tracker.epoch(), epoch);
    assert!(tracker.snapshot().removed.is_empty());
    settle(&mut tracker, &solids);
    assert_eq!(tracker.snapshot().solid_count, 7);
}

#[test]
fn empty_tracker_and_removal_finish_without_spurious_child() {
    let mut empty = SplitTracker::new();
    assert!(empty.tick(&BTreeSet::new(), 200, true).is_empty());
    assert!(empty.snapshot().split_complete);

    let (mut tracker, mut solids) = start(&[[15, -16, 31]]);
    settle(&mut tracker, &solids);
    solids.clear();
    tracker.on_removed([15, -16, 31]);
    assert!(tracker.tick(&solids, 200, true).is_empty());
    assert!(tracker.snapshot().heat.is_empty());
    assert_eq!(tracker.snapshot().solid_count, 0);
}

#[test]
fn budget_partition_changes_tick_latency_but_not_unique_split_partition() {
    let order: Vec<_> = (-31..=32).map(|x| [x, -17, 16]).collect();
    let (mut tracker, mut solids) = start(&order);
    settle(&mut tracker, &solids);
    solids.remove(&[0, -17, 16]);
    tracker.on_removed([0, -17, 16]);
    let mut single_step = tracker.clone();
    let (ticks, expected) = next_split(&mut single_step, &solids);
    let actual = tracker
        .tick(&solids, 200, true)
        .pop()
        .expect("default budget finishes");
    assert_eq!(ticks, 34);
    assert_eq!(actual, expected);
    assert_eq!(tracker.snapshot(), single_step.snapshot());
}
