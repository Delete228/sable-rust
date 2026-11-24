use sable_core::{
    Config, Error, Id, Motion, Pos,
    gather::{self, GatherState},
    host_lifecycle::{
        BatchReceipt, CommitReceipt, HostLifecycle, LifecyclePlan, Operation, PendingPhase,
        RecordedReceipt, RestoredState, RetirementReceipt, SplitDestination, SplitRequest,
        StateSnapshot,
    },
    mass::MassRegistry,
};
use std::collections::BTreeMap;

fn snapshot(revision: u64, positions: &[Pos]) -> StateSnapshot {
    StateSnapshot {
        revision,
        cells: positions
            .iter()
            .copied()
            .map(|position| (position, "minecraft:blackstone".into()))
            .collect(),
    }
}
fn initial() -> (HostLifecycle, StateSnapshot, StateSnapshot, MassRegistry) {
    (
        HostLifecycle::new(Config::default()).unwrap(),
        snapshot(7, &[[-17, 64, -1], [-16, 64, -1], [9, 64, 9]]),
        snapshot(11, &[]),
        MassRegistry::pinned_full_blocks(),
    )
}
fn readback(
    source: &StateSnapshot,
    destination: &StateSnapshot,
    plan: &LifecyclePlan,
) -> (StateSnapshot, StateSnapshot, CommitReceipt) {
    // This synthesizes observations for a policy invariant test only. It never
    // stands in for an actual host transfer or container-conservation proof.
    let mut from = source.clone();
    let mut to = destination.clone();
    for &(a, b) in plan.mapping() {
        let state = from.cells.remove(&a).unwrap();
        to.cells.insert(b, state);
    }
    from.revision += 1;
    to.revision += 1;
    let receipt = CommitReceipt {
        source_revision: from.revision,
        destination_revision: to.revision,
    };
    (from, to, receipt)
}
fn capture(
    policy: &mut HostLifecycle,
    source: &StateSnapshot,
    destination: &StateSnapshot,
    masses: &MassRegistry,
) -> LifecyclePlan {
    let gathered = gather::connected(
        [-17, 64, -1],
        512,
        |position| source.cells.contains_key(&position),
        |_, _, _| true,
    )
    .unwrap();
    assert_eq!(gathered.state, GatherState::Success);
    assert_eq!(gathered.count, 2);
    policy
        .prepare_capture(
            source.clone(),
            destination.clone(),
            Id([1; 16]),
            [-17, 64, -1],
            &gathered.blocks.unwrap().into_iter().collect::<Vec<_>>(),
            masses,
        )
        .unwrap()
}

#[test]
fn gathered_negative_chunk_edge_capture_reserves_without_publishing_and_adopts_readback() {
    let (mut policy, source, destination, masses) = initial();
    let plan = capture(&mut policy, &source, &destination, &masses);
    assert_eq!(plan.operation(), Operation::Capture);
    assert_eq!(plan.plot(), 0);
    assert_eq!(policy.handle(plan.id()), Err(Error::StaleHandle));
    assert_eq!(plan.mapping().len(), 2);
    for &(from, to) in plan.mapping() {
        assert_eq!(
            plan.motion().pose.transform(to.map(f64::from)),
            from.map(f64::from)
        );
    }
    assert!(
        policy
            .validate_pending(&plan, &source, &destination)
            .is_ok()
    );
    assert!(matches!(
        policy.prepare_capture(
            source.clone(),
            destination.clone(),
            Id([2; 16]),
            [-17, 64, -1],
            &[[-17, 64, -1]],
            &masses
        ),
        Err(Error::HostRefused)
    ));
    let (after_source, after_destination, receipt) = readback(&source, &destination, &plan);
    let handle = policy
        .adopt_committed(&plan, receipt, &after_source, &after_destination)
        .unwrap()
        .unwrap();
    let metadata = policy.metadata(handle).unwrap();
    assert_eq!(metadata.plot, 0);
    assert_eq!(metadata.host_revision, after_destination.revision);
    assert_eq!(metadata.motion, *plan.motion());
    assert_eq!(after_source.cells, snapshot(0, &[[9, 64, 9]]).cells);
    assert!(policy.pending_commit().is_none());
}

#[test]
fn callback_revision_and_geometry_changes_refuse_and_cancel_releases_reservation() {
    let (mut policy, source, destination, masses) = initial();
    let plan = capture(&mut policy, &source, &destination, &masses);
    let mut changed = source.clone();
    changed.revision += 1;
    assert_eq!(
        policy.validate_pending(&plan, &changed, &destination),
        Err(Error::StaleHandle)
    );
    changed.revision = source.revision;
    changed.cells.remove(&[-16, 64, -1]);
    assert_eq!(
        policy.validate_pending(&plan, &changed, &destination),
        Err(Error::StaleHandle)
    );
    let mut occupied = destination.clone();
    occupied
        .cells
        .insert(plan.mapping()[0].1, "minecraft:blackstone".into());
    assert_eq!(
        policy.validate_pending(&plan, &source, &occupied),
        Err(Error::StaleHandle)
    );
    policy.cancel(&plan).unwrap();
    assert_eq!(
        policy.validate_pending(&plan, &source, &destination),
        Err(Error::StaleHandle)
    );
    let next = capture(&mut policy, &source, &destination, &masses);
    assert_eq!(next.plot(), 0);
    policy.cancel(&next).unwrap();
    assert_eq!(policy.handle(Id([1; 16])), Err(Error::StaleHandle));
}

#[test]
fn successful_host_receipt_survives_failed_adoption_and_can_reconcile() {
    let (mut policy, source, destination, masses) = initial();
    let plan = capture(&mut policy, &source, &destination, &masses);
    let (mut after_source, mut after_destination, receipt) = readback(&source, &destination, &plan);
    // Matching predicted geometry does not establish that a host ever
    // committed. Only recording its actual successful receipt enables adoption.
    assert_eq!(
        policy.reconcile_committed(&plan, &after_source, &after_destination),
        Err(Error::HostRefused)
    );
    assert!(policy.pending_commit().is_none());
    let mut wrong = after_destination.clone();
    wrong.cells.remove(&plan.mapping()[0].1);
    assert_eq!(
        policy.adopt_committed(&plan, receipt, &after_source, &wrong),
        Err(Error::StaleHandle)
    );
    assert_eq!(
        policy.pending_commit().unwrap().receipt,
        RecordedReceipt::Transfer(receipt)
    );
    assert_eq!(policy.pending_commit().unwrap().id, Id([1; 16]));
    assert_eq!(policy.handle(Id([1; 16])), Err(Error::Interrupted));
    let mut older = after_destination.clone();
    older.revision = receipt.destination_revision - 1;
    assert_eq!(
        policy.reconcile_committed(&plan, &after_source, &older),
        Err(Error::StaleHandle)
    );
    assert_eq!(
        policy.pending_commit().unwrap().receipt,
        RecordedReceipt::Transfer(receipt)
    );
    assert_eq!(policy.handle(Id([1; 16])), Err(Error::Interrupted));
    assert_eq!(policy.cancel(&plan), Err(Error::HostRefused));
    assert!(matches!(
        policy.prepare_capture(
            source.clone(),
            destination.clone(),
            Id([2; 16]),
            [-17, 64, -1],
            &[[-17, 64, -1]],
            &masses
        ),
        Err(Error::HostRefused)
    ));
    let wrong_receipt = CommitReceipt {
        source_revision: receipt.source_revision + 1,
        ..receipt
    };
    assert_eq!(
        policy.adopt_committed(&plan, wrong_receipt, &after_source, &after_destination),
        Err(Error::StaleHandle)
    );
    assert_eq!(
        policy.pending_commit().unwrap().receipt,
        RecordedReceipt::Transfer(receipt)
    );
    // An unrelated clock change is permitted only with the same actual state
    // keys and a readback at or after the recorded successful transaction.
    after_source.revision += 1;
    after_destination.revision += 1;
    let handle = policy
        .reconcile_committed(&plan, &after_source, &after_destination)
        .unwrap()
        .unwrap();
    assert_eq!(
        policy.metadata(handle).unwrap().host_revision,
        after_destination.revision
    );
    assert!(policy.pending_commit().is_none());
}

#[test]
fn return_uses_current_actual_cells_then_retires_handle_and_reuses_plot_safely() {
    let (mut policy, source, destination, masses) = initial();
    let plan = capture(&mut policy, &source, &destination, &masses);
    let (after_source, mut current, receipt) = readback(&source, &destination, &plan);
    let old = policy
        .adopt_committed(&plan, receipt, &after_source, &current)
        .unwrap()
        .unwrap();
    // An actual supported placement after capture is included in return even
    // though the private geometry metadata has never cached that new cell.
    let added = [
        plan.mapping()[1].1[0] + 1,
        plan.mapping()[1].1[1],
        plan.mapping()[1].1[2],
    ];
    current.cells.insert(added, "minecraft:blackstone".into());
    current.revision += 1;
    let offset = std::array::from_fn(|axis| plan.mapping()[0].0[axis] - plan.mapping()[0].1[axis]);
    let mut blocked = after_source.clone();
    blocked
        .cells
        .insert(plan.mapping()[0].0, "minecraft:blackstone".into());
    assert!(matches!(
        policy.prepare_return(
            old,
            current.clone(),
            blocked,
            offset,
            Motion::default(),
            &masses
        ),
        Err(Error::Occupied)
    ));
    assert!(policy.metadata(old).is_ok());
    let returning = policy
        .prepare_return(
            old,
            current.clone(),
            after_source.clone(),
            offset,
            plan.motion().clone(),
            &masses,
        )
        .unwrap();
    assert_eq!(returning.operation(), Operation::Return);
    assert_eq!(returning.mapping().len(), 3);
    let (empty, ordinary, receipt) = readback(&current, &after_source, &returning);
    assert_eq!(
        policy
            .adopt_committed(&returning, receipt, &empty, &ordinary)
            .unwrap(),
        None
    );
    assert_eq!(policy.metadata(old), Err(Error::StaleHandle));
    let recapture = policy
        .prepare_capture(
            ordinary.clone(),
            empty.clone(),
            Id([1; 16]),
            [-17, 64, -1],
            &[[-17, 64, -1]],
            &masses,
        )
        .unwrap();
    assert_eq!(recapture.plot(), 0);
    let (ordinary, local, receipt) = readback(&ordinary, &empty, &recapture);
    let new = policy
        .adopt_committed(&recapture, receipt, &ordinary, &local)
        .unwrap()
        .unwrap();
    assert_ne!(old, new);
    assert_eq!(policy.metadata(old), Err(Error::StaleHandle));
}

#[test]
fn invalid_capture_and_unsupported_return_refuse_before_reserving() {
    let (mut policy, source, destination, masses) = initial();
    let attempts = [
        (Id([0; 16]), vec![[-17, 64, -1]]),
        (Id([1; 16]), vec![[-17, 64, -1], [-17, 64, -1]]),
        (Id([1; 16]), vec![[0, 0, 0]]),
        (Id([1; 16]), vec![]),
    ];
    for (id, positions) in attempts {
        assert!(
            policy
                .prepare_capture(
                    source.clone(),
                    destination.clone(),
                    id,
                    [-17, 64, -1],
                    &positions,
                    &masses
                )
                .is_err()
        );
    }
    let mut unsupported = source.clone();
    unsupported
        .cells
        .insert([-17, 64, -1], "unsupported:unknown".into());
    assert!(matches!(
        policy.prepare_capture(
            unsupported,
            destination.clone(),
            Id([1; 16]),
            [-17, 64, -1],
            &[[-17, 64, -1]],
            &masses
        ),
        Err(Error::InvalidInput)
    ));
    let plan = capture(&mut policy, &source, &destination, &masses);
    let (after_source, current, receipt) = readback(&source, &destination, &plan);
    let old = policy
        .adopt_committed(&plan, receipt, &after_source, &current)
        .unwrap()
        .unwrap();
    let mut rotated = plan.motion().clone();
    rotated.pose.rotation = [0., 1., 0., 0.];
    assert!(matches!(
        policy.prepare_return(
            old,
            current.clone(),
            after_source.clone(),
            [0; 3],
            rotated,
            &masses
        ),
        Err(Error::InvalidInput)
    ));
    let prepared = policy
        .prepare_return(
            old,
            current,
            after_source,
            [0; 3],
            plan.motion().clone(),
            &masses,
        )
        .unwrap();
    policy.cancel(&prepared).unwrap();
    assert!(policy.metadata(old).is_ok());
}

#[test]
fn far_origin_selection_keeps_checked_coordinates_and_rejects_plot_overflow() {
    let mut policy = HostLifecycle::new(Config::default()).unwrap();
    let masses = MassRegistry::pinned_full_blocks();
    let far = [(1 << 25) - 2, 64, -(1 << 25) + 1];
    let source = snapshot(1, &[far]);
    let destination = StateSnapshot {
        revision: 0,
        cells: BTreeMap::new(),
    };
    let plan = policy
        .prepare_capture(
            source.clone(),
            destination.clone(),
            Id([1; 16]),
            far,
            &[far],
            &masses,
        )
        .unwrap();
    assert_eq!(
        plan.motion()
            .pose
            .transform(plan.mapping()[0].1.map(f64::from)),
        far.map(f64::from)
    );
    policy.cancel(&plan).unwrap();
    let mut outside = far;
    outside[0] = (1 << 25) - 1;
    let source = snapshot(2, &[outside]);
    // A remote selection cannot be silently truncated into the local plot.
    assert!(matches!(
        policy.prepare_capture(
            source,
            destination,
            Id([1; 16]),
            [-(1 << 25), 64, 0],
            &[outside],
            &masses
        ),
        Err(Error::InvalidCoordinate)
    ));
}

#[test]
fn logical_restore_preserves_plots_generations_and_uses_fresh_actual_geometry() {
    let (mut policy, source, destination, masses) = initial();
    let plan = capture(&mut policy, &source, &destination, &masses);
    let planned = policy.planned_persistent_state(&plan).unwrap();
    assert_eq!(planned.owners[0].generation, plan.generation());
    assert!(policy.persistent_state().unwrap().owners.is_empty());
    let (ordinary, mut local, receipt) = readback(&source, &destination, &plan);
    let old = policy
        .adopt_committed(&plan, receipt, &ordinary, &local)
        .unwrap()
        .unwrap();
    let saved = policy.persistent_state().unwrap();
    let extra = [
        plan.mapping()[1].1[0] + 1,
        plan.mapping()[1].1[1],
        plan.mapping()[1].1[2],
    ];
    local.cells.insert(extra, "minecraft:bedrock".into());
    local.revision += 5;
    let actual = BTreeMap::from([(
        plan.id(),
        RestoredState {
            state: local.clone(),
            motion: plan.motion().clone(),
        },
    )]);
    let mut restored = HostLifecycle::restore(Config::default(), saved.clone(), actual).unwrap();
    let current = restored.handle(plan.id()).unwrap();
    assert_ne!(old, current);
    assert_eq!(restored.metadata(old), Err(Error::StaleHandle));
    assert_eq!(
        restored.metadata(current).unwrap().generation,
        plan.generation()
    );
    assert_eq!(restored.metadata(current).unwrap().plot, plan.plot());
    assert_eq!(
        restored.metadata(current).unwrap().host_revision,
        local.revision
    );
    assert_eq!(restored.persistent_state().unwrap(), saved);
    let offset = std::array::from_fn(|axis| plan.mapping()[0].0[axis] - plan.mapping()[0].1[axis]);
    let returning = restored
        .prepare_return(
            current,
            local.clone(),
            ordinary.clone(),
            offset,
            plan.motion().clone(),
            &masses,
        )
        .unwrap();
    assert_eq!(returning.mapping().len(), 3);
    let (empty, ordinary, receipt) = readback(&local, &ordinary, &returning);
    restored
        .adopt_committed(&returning, receipt, &empty, &ordinary)
        .unwrap();
    let saved = restored.persistent_state().unwrap();
    assert!(saved.owners.is_empty());
    assert_eq!(saved.plot_generations.get(&0), Some(&plan.generation()));
    let mut reopened = HostLifecycle::restore(Config::default(), saved, BTreeMap::new()).unwrap();
    let next = reopened
        .prepare_capture(
            ordinary,
            empty,
            Id([1; 16]),
            [-17, 64, -1],
            &[[-17, 64, -1]],
            &masses,
        )
        .unwrap();
    assert_eq!(next.plot(), 0);
    assert_eq!(next.generation(), plan.generation() + 1);
}

#[test]
fn malformed_logical_restore_refuses_duplicate_or_regressed_identity_and_missing_real_state() {
    let (mut policy, source, destination, masses) = initial();
    let plan = capture(&mut policy, &source, &destination, &masses);
    let (ordinary, local, receipt) = readback(&source, &destination, &plan);
    policy
        .adopt_committed(&plan, receipt, &ordinary, &local)
        .unwrap();
    let saved = policy.persistent_state().unwrap();
    let actual = BTreeMap::from([(
        plan.id(),
        RestoredState {
            state: local,
            motion: plan.motion().clone(),
        },
    )]);
    let changes: [fn(&mut sable_core::host_lifecycle::PersistentState); 6] = [
        |s| s.owners.push(s.owners[0]),
        |s| s.owners[0].id = Id([0; 16]),
        |s| s.owners[0].generation = 0,
        |s| s.owners[0].generation += 1,
        |s| {
            s.plot_generations.insert(0, 0);
        },
        |s| {
            s.plot_generations.insert(u32::MAX, 1);
        },
    ];
    for change in changes {
        let mut invalid = saved.clone();
        change(&mut invalid);
        assert!(HostLifecycle::restore(Config::default(), invalid, actual.clone()).is_err());
    }
    let mut duplicate_plot = saved.clone();
    let mut second = duplicate_plot.owners[0];
    second.id = Id([2; 16]);
    duplicate_plot.owners.push(second);
    let mut second_actual = actual.clone();
    second_actual.insert(second.id, actual[&plan.id()].clone());
    assert!(HostLifecycle::restore(Config::default(), duplicate_plot, second_actual).is_err());
    assert!(HostLifecycle::restore(Config::default(), saved.clone(), BTreeMap::new()).is_err());
    let mut extra = actual.clone();
    extra.insert(Id([2; 16]), actual[&plan.id()].clone());
    assert!(HostLifecycle::restore(Config::default(), saved.clone(), extra).is_err());
    let mut bad_pose = actual.clone();
    bad_pose.get_mut(&plan.id()).unwrap().motion.pose.rotation = [0.; 4];
    assert!(HostLifecycle::restore(Config::default(), saved.clone(), bad_pose).is_err());
    let mut invalid_cell = actual;
    invalid_cell
        .get_mut(&plan.id())
        .unwrap()
        .state
        .cells
        .insert([-1, 64, 0], "minecraft:blackstone".into());
    assert!(HostLifecycle::restore(Config::default(), saved, invalid_cell).is_err());
}

#[test]
fn uncertain_host_outcome_blocks_old_metadata_until_authoritative_resolution() {
    let (mut policy, source, destination, masses) = initial();
    let plan = capture(&mut policy, &source, &destination, &masses);
    let (ordinary, local, receipt) = readback(&source, &destination, &plan);
    assert_eq!(policy.pending_phase(), Some(PendingPhase::Prepared));
    policy.mark_unconfirmed(&plan).unwrap();
    assert_eq!(policy.pending_phase(), Some(PendingPhase::Unconfirmed));
    assert_eq!(policy.persistent_state(), Err(Error::Interrupted));
    assert_eq!(policy.handle(plan.id()), Err(Error::Interrupted));
    assert_eq!(policy.cancel(&plan), Err(Error::HostRefused));
    assert_eq!(
        policy.reconcile_committed(&plan, &ordinary, &local),
        Err(Error::HostRefused)
    );
    // Models an actual authoritative NotCommitted query, not a guessed failure.
    policy.confirm_not_committed(&plan).unwrap();
    assert_eq!(policy.pending_phase(), None);
    let plan = capture(&mut policy, &source, &destination, &masses);
    policy.mark_unconfirmed(&plan).unwrap();
    let handle = policy
        .adopt_committed(&plan, receipt, &ordinary, &local)
        .unwrap()
        .unwrap();
    assert!(policy.metadata(handle).is_ok());
    assert_eq!(policy.confirm_not_committed(&plan), Err(Error::StaleHandle));
}

#[test]
fn actual_empty_owner_requires_separate_retirement_barrier_and_preserves_reuse_generation() {
    let (mut policy, source, destination, masses) = initial();
    let plan = capture(&mut policy, &source, &destination, &masses);
    let (ordinary, local, receipt) = readback(&source, &destination, &plan);
    let old = policy
        .adopt_committed(&plan, receipt, &ordinary, &local)
        .unwrap()
        .unwrap();
    assert!(matches!(
        policy.prepare_empty_retirement(old, local.clone(), 7),
        Err(Error::InvalidInput)
    ));
    let empty = snapshot(local.revision + 1, &[]);
    let retire = policy
        .prepare_empty_retirement(old, empty.clone(), 7)
        .unwrap();
    assert_eq!(retire.operation(), Operation::RetireEmpty);
    assert!(retire.mapping().is_empty());
    assert!(policy.metadata(old).is_ok());
    assert_eq!(
        policy.validate_empty_retirement(&retire, &empty, 8),
        Err(Error::StaleHandle)
    );
    assert!(policy.validate_empty_retirement(&retire, &empty, 7).is_ok());
    assert_eq!(
        policy.reconcile_retired(&retire, &empty),
        Err(Error::HostRefused)
    );
    assert_eq!(
        policy.adopt_committed(&retire, receipt, &empty, &ordinary),
        Err(Error::InvalidInput)
    );
    let receipt = RetirementReceipt {
        barrier_epoch: 13,
        source_revision: empty.revision,
        retired_generation: 7,
    };
    let mut unexpected = empty.clone();
    unexpected.cells.insert(
        local.cells.keys().next().copied().unwrap(),
        "minecraft:blackstone".into(),
    );
    assert_eq!(
        policy.adopt_retired(&retire, receipt, &unexpected),
        Err(Error::StaleHandle)
    );
    assert_eq!(
        policy.pending_commit().unwrap().receipt,
        RecordedReceipt::Retirement(receipt)
    );
    assert_eq!(policy.metadata(old), Err(Error::Interrupted));
    assert_eq!(policy.cancel(&retire), Err(Error::HostRefused));
    policy.reconcile_retired(&retire, &empty).unwrap();
    assert_eq!(policy.metadata(old), Err(Error::StaleHandle));
    let saved = policy.persistent_state().unwrap();
    assert!(saved.owners.is_empty());
    assert_eq!(saved.plot_generations.get(&0), Some(&plan.generation()));
    let mut reopened = HostLifecycle::restore(Config::default(), saved, BTreeMap::new()).unwrap();
    let next = reopened
        .prepare_capture(
            ordinary,
            empty,
            Id([1; 16]),
            [9, 64, 9],
            &[[9, 64, 9]],
            &masses,
        )
        .unwrap();
    assert_eq!(next.plot(), 0);
    assert_eq!(next.generation(), plan.generation() + 1);
}

struct SplitFixture {
    policy: HostLifecycle,
    parent: sable_core::Handle,
    source: StateSnapshot,
    masses: MassRegistry,
    motion: Motion,
    split: sable_core::split::SplitPlan,
}
impl SplitFixture {
    fn new() -> Self {
        // Capacity exactly fits the old connected beam. Split planning must
        // replace the stale cached beam with the four current actual cells
        // before inserting children, never double-count transferred geometry.
        let config = Config {
            max_records: 21,
            ..Config::default()
        };
        let mut policy = HostLifecycle::new(config).unwrap();
        let positions = (0..=20).map(|x| [x, 64, 0]).collect::<Vec<_>>();
        let ordinary = snapshot(1, &positions);
        let local = snapshot(0, &[]);
        let masses = MassRegistry::pinned_full_blocks();
        let capture = policy
            .prepare_capture(
                ordinary.clone(),
                local.clone(),
                Id([1; 16]),
                [0, 64, 0],
                &positions,
                &masses,
            )
            .unwrap();
        let (ordinary, mut source, receipt) = readback(&ordinary, &local, &capture);
        let parent = policy
            .adopt_committed(&capture, receipt, &ordinary, &source)
            .unwrap()
            .unwrap();
        let kept = [0, 1, 10, 20].map(|index| capture.mapping()[index].1);
        let initial_solids = source.cells.keys().copied().collect();
        let mut tracker = sable_core::split::SplitTracker::new();
        for &position in source.cells.keys() {
            tracker.on_added(position);
        }
        assert!(tracker.tick(&initial_solids, 200, true).is_empty());
        // Exercise actual accepted incremental removal handling; rebuilding a
        // fresh tracker from disconnected cells is a different source case.
        for &position in source
            .cells
            .keys()
            .filter(|position| !kept.contains(position))
        {
            tracker.on_removed(position);
        }
        source.cells.retain(|position, _| kept.contains(position));
        source.revision += 1;
        let solids = source.cells.keys().copied().collect();
        let mut split = None;
        for _ in 0..32 {
            if let Some(plan) = tracker.tick(&solids, 200, true).into_iter().next() {
                split = Some(plan);
                break;
            }
        }
        let split = split.expect("actual accepted heat policy produced no split");
        assert_eq!(split.children.len(), 2);
        assert_eq!(split.source_epoch, tracker.epoch());
        let origin = capture.mapping()[0].1;
        let motion = Motion {
            pose: sable_core::Pose {
                position: [100., 30., 50.],
                pivot: [
                    f64::from(origin[0] + 7),
                    f64::from(origin[1]),
                    f64::from(origin[2]),
                ],
                ..sable_core::Pose::default()
            },
            linear: [1., 2., 3.],
            angular: [0., 2., 0.],
        };
        Self {
            policy,
            parent,
            source,
            masses,
            motion,
            split,
        }
    }
    fn destinations(&self) -> Vec<StateSnapshot> {
        vec![snapshot(3, &[]), snapshot(9, &[])]
    }
    fn request(&self) -> SplitRequest {
        SplitRequest {
            parent: self.parent,
            source: self.source.clone(),
            split: self.split.clone(),
            tracker_epoch: self.split.source_epoch,
            children: self
                .destinations()
                .into_iter()
                .enumerate()
                .map(|(index, state)| SplitDestination {
                    id: Id([index as u8 + 2; 16]),
                    state,
                })
                .collect(),
            motion: self.motion.clone(),
        }
    }
    fn readback(&self, plan: &LifecyclePlan) -> (StateSnapshot, Vec<StateSnapshot>, BatchReceipt) {
        let mut source = self.source.clone();
        let mut destinations = self.destinations();
        for (child, destination) in plan.split_children().iter().zip(&mut destinations) {
            for &(from, to) in &child.mapping {
                destination
                    .cells
                    .insert(to, source.cells.remove(&from).unwrap());
            }
            destination.revision += 1;
        }
        source.revision += 1;
        let receipt = BatchReceipt {
            barrier_epoch: 42,
            source_revision: source.revision,
            destination_revisions: destinations.iter().map(|state| state.revision).collect(),
        };
        (source, destinations, receipt)
    }
}

#[test]
fn actual_heat_split_stages_all_children_from_one_r5_com_and_requires_complete_receipt() {
    let mut fixture = SplitFixture::new();
    let plan = fixture
        .policy
        .prepare_split(fixture.request(), &fixture.masses)
        .unwrap();
    assert_eq!(plan.operation(), Operation::Split);
    assert_eq!(plan.motion(), &fixture.motion);
    assert!(plan.mapping().is_empty());
    assert_eq!(plan.split_children().len(), 2);
    assert_eq!(fixture.policy.persistent_state().unwrap().owners.len(), 1);
    assert_eq!(
        fixture
            .policy
            .planned_persistent_state(&plan)
            .unwrap()
            .owners
            .len(),
        3
    );
    // Four equal masses have original local x COM8.25 above the mapped origin;
    // singleton COMs10.5 and20.5 therefore see deltas2.25 and12.25. Removing
    // child one must not replace that frozen original COM for child two.
    assert_eq!(plan.split_children()[0].motion.linear, [1., 2., -1.5]);
    assert_eq!(plan.split_children()[1].motion.linear, [1., 2., -21.5]);
    for child in plan.split_children() {
        assert_eq!(child.motion.pose, fixture.motion.pose);
        assert_eq!(child.motion.angular, fixture.motion.angular);
    }
    let destinations = fixture.destinations();
    fixture
        .policy
        .validate_split(
            &plan,
            &fixture.source,
            &destinations,
            fixture.split.source_epoch,
        )
        .unwrap();
    let (source, destinations, receipt) = fixture.readback(&plan);
    assert_eq!(
        fixture
            .policy
            .reconcile_split(&plan, &source, &destinations),
        Err(Error::HostRefused)
    );
    assert_eq!(
        fixture
            .policy
            .adopt_split_committed(&plan, receipt.clone(), &source, &destinations[..1]),
        Err(Error::StaleHandle)
    );
    assert_eq!(
        fixture.policy.pending_commit().unwrap().receipt,
        RecordedReceipt::Batch(receipt)
    );
    assert_eq!(
        fixture.policy.metadata(fixture.parent),
        Err(Error::Interrupted)
    );
    assert_eq!(fixture.policy.cancel(&plan), Err(Error::HostRefused));
    let handles = fixture
        .policy
        .reconcile_split(&plan, &source, &destinations)
        .unwrap();
    assert_eq!(handles.len(), 2);
    assert_eq!(
        fixture.policy.metadata(fixture.parent).unwrap().motion,
        fixture.motion
    );
    for ((handle, child), state) in handles.iter().zip(plan.split_children()).zip(&destinations) {
        let metadata = fixture.policy.metadata(*handle).unwrap();
        assert_eq!(handle.id(), child.id);
        assert_eq!(metadata.plot, child.plot);
        assert_eq!(metadata.generation, child.generation);
        assert_eq!(metadata.motion, child.motion);
        assert_eq!(metadata.host_revision, state.revision);
    }
    assert_eq!(
        source.cells.len()
            + destinations
                .iter()
                .map(|state| state.cells.len())
                .sum::<usize>(),
        4
    );
    let saved = fixture.policy.persistent_state().unwrap();
    assert_eq!(saved.owners.len(), 3);
    assert!(
        saved
            .owners
            .iter()
            .filter(|owner| owner.id != fixture.parent.id())
            .all(|owner| owner.parent == Some(fixture.parent.id()))
    );
    let mut actual = BTreeMap::from([(
        fixture.parent.id(),
        RestoredState {
            state: source,
            motion: fixture.motion,
        },
    )]);
    for (child, state) in plan.split_children().iter().zip(destinations) {
        actual.insert(
            child.id,
            RestoredState {
                state,
                motion: child.motion.clone(),
            },
        );
    }
    let restored = HostLifecycle::restore(
        Config {
            max_records: 21,
            ..Config::default()
        },
        saved.clone(),
        actual,
    )
    .unwrap();
    assert_eq!(restored.persistent_state().unwrap(), saved);
}

#[test]
fn split_refuses_changed_tracker_or_destination_before_host_and_keeps_stale_readback_unadopted() {
    let mut fixture = SplitFixture::new();
    let mut stale = fixture.request();
    stale.tracker_epoch += 1;
    assert!(
        fixture
            .policy
            .prepare_split(stale, &fixture.masses)
            .is_err()
    );
    let mut overlap = fixture.request();
    let duplicate = *overlap.split.children[0].iter().next().unwrap();
    overlap.split.children[1].insert(duplicate);
    assert!(
        fixture
            .policy
            .prepare_split(overlap, &fixture.masses)
            .is_err()
    );
    let mut occupied = fixture.request();
    let position = *occupied.split.children[0].iter().next().unwrap();
    occupied.children[0]
        .state
        .cells
        .insert(position, "minecraft:blackstone".into());
    assert!(
        fixture
            .policy
            .prepare_split(occupied, &fixture.masses)
            .is_err()
    );
    assert_eq!(fixture.policy.pending_phase(), None);
    assert!(fixture.policy.metadata(fixture.parent).is_ok());
    let plan = fixture
        .policy
        .prepare_split(fixture.request(), &fixture.masses)
        .unwrap();
    let destinations = fixture.destinations();
    assert_eq!(
        fixture.policy.validate_split(
            &plan,
            &fixture.source,
            &destinations,
            fixture.split.source_epoch + 1
        ),
        Err(Error::StaleHandle)
    );
    let mut changed = fixture.source.clone();
    changed.revision += 1;
    assert_eq!(
        fixture
            .policy
            .validate_split(&plan, &changed, &destinations, fixture.split.source_epoch),
        Err(Error::StaleHandle)
    );
    let (source, mut destinations, receipt) = fixture.readback(&plan);
    destinations[1].revision -= 1;
    assert_eq!(
        fixture
            .policy
            .adopt_split_committed(&plan, receipt.clone(), &source, &destinations),
        Err(Error::StaleHandle)
    );
    assert_eq!(fixture.policy.handle(Id([1; 16])), Err(Error::Interrupted));
    let mut other_receipt = receipt.clone();
    other_receipt.barrier_epoch += 1;
    assert_eq!(
        fixture
            .policy
            .adopt_split_committed(&plan, other_receipt, &source, &destinations),
        Err(Error::StaleHandle)
    );
    assert_eq!(
        fixture.policy.pending_commit().unwrap().receipt,
        RecordedReceipt::Batch(receipt)
    );
    destinations[1].revision += 1;
    fixture
        .policy
        .reconcile_split(&plan, &source, &destinations)
        .unwrap();
    assert_eq!(fixture.policy.pending_phase(), None);
}

#[test]
fn published_observations_preserve_identity_and_refuse_regression_or_unversioned_geometry() {
    let (mut policy, source, destination, masses) = initial();
    let plan = capture(&mut policy, &source, &destination, &masses);
    let (ordinary, mut local, receipt) = readback(&source, &destination, &plan);
    let handle = policy
        .adopt_committed(&plan, receipt, &ordinary, &local)
        .unwrap()
        .unwrap();
    let original = policy.metadata(handle).unwrap();
    let mut motion = original.motion.clone();
    motion.pose.position[0] += 2.;
    motion.linear[0] = 0.5;
    policy
        .publish_observation(handle, local.clone(), motion.clone(), &masses)
        .unwrap();
    assert_eq!(policy.metadata(handle).unwrap().motion, motion);
    let mut regressed = local.clone();
    regressed.revision -= 1;
    assert_eq!(
        policy.publish_observation(handle, regressed, motion.clone(), &masses),
        Err(Error::StaleHandle)
    );
    let extra = [
        plan.mapping()[1].1[0] + 1,
        plan.mapping()[1].1[1],
        plan.mapping()[1].1[2],
    ];
    local.cells.insert(extra, "minecraft:blackstone".into());
    assert_eq!(
        policy.publish_observation(handle, local.clone(), motion.clone(), &masses),
        Err(Error::StaleHandle)
    );
    local.revision += 1;
    policy
        .publish_observation(handle, local.clone(), motion.clone(), &masses)
        .unwrap();
    let current = policy.metadata(handle).unwrap();
    assert_eq!(
        (current.handle, current.plot, current.generation),
        (original.handle, original.plot, original.generation)
    );
    assert_eq!(current.host_revision, local.revision);
    let mut unsupported = local.clone();
    unsupported.revision += 1;
    unsupported
        .cells
        .insert(extra, "unsupported:unknown".into());
    assert_eq!(
        policy.publish_observation(handle, unsupported, motion.clone(), &masses),
        Err(Error::InvalidInput)
    );
    let pending = policy
        .prepare_return(
            handle,
            local.clone(),
            ordinary,
            [0; 3],
            motion.clone(),
            &masses,
        )
        .unwrap();
    assert_eq!(
        policy.publish_observation(handle, local.clone(), motion.clone(), &masses),
        Err(Error::HostRefused)
    );
    policy.cancel(&pending).unwrap();
    let empty = snapshot(local.revision + 1, &[]);
    policy
        .publish_observation(handle, empty.clone(), motion, &masses)
        .unwrap();
    assert!(policy.prepare_empty_retirement(handle, empty, 7).is_ok());
}

#[test]
fn refreshed_other_owner_geometry_participates_in_global_capacity_preflight() {
    let mut policy = HostLifecycle::new(Config {
        max_records: 3,
        ..Config::default()
    })
    .unwrap();
    let masses = MassRegistry::pinned_full_blocks();
    let mut first = None;
    for index in 0..2 {
        let source = snapshot(1, &[[index, 64, 0]]);
        let local = snapshot(0, &[]);
        let plan = policy
            .prepare_capture(
                source.clone(),
                local.clone(),
                Id([index as u8 + 1; 16]),
                [index, 64, 0],
                &[[index, 64, 0]],
                &masses,
            )
            .unwrap();
        let (ordinary, local, receipt) = readback(&source, &local, &plan);
        let handle = policy
            .adopt_committed(&plan, receipt, &ordinary, &local)
            .unwrap()
            .unwrap();
        if index == 0 {
            first = Some((handle, local, plan.motion().clone()));
        }
    }
    let (handle, mut actual, motion) = first.unwrap();
    let original = *actual.cells.keys().next().unwrap();
    actual.cells.insert(
        [original[0] + 1, original[1], original[2]],
        "minecraft:blackstone".into(),
    );
    actual.revision += 1;
    policy
        .publish_observation(handle, actual, motion, &masses)
        .unwrap();
    let source = snapshot(1, &[[2, 64, 0]]);
    assert!(matches!(
        policy.prepare_capture(
            source,
            snapshot(0, &[]),
            Id([3; 16]),
            [2, 64, 0],
            &[[2, 64, 0]],
            &masses
        ),
        Err(Error::Capacity)
    ));
    assert_eq!(policy.pending_phase(), None);
}

#[test]
fn receipt_only_latches_survive_unavailable_post_state_without_fabricated_snapshots() {
    let (mut policy, source, destination, masses) = initial();
    let plan = capture(&mut policy, &source, &destination, &masses);
    let (ordinary, local, receipt) = readback(&source, &destination, &plan);
    policy.mark_unconfirmed(&plan).unwrap();
    // An actual successful receipt can arrive while post-state I/O is still
    // unavailable. No fake/empty StateSnapshot is passed to record it.
    policy.record_committed(&plan, receipt).unwrap();
    assert_eq!(policy.pending_phase(), Some(PendingPhase::Committed));
    assert_eq!(
        policy.pending_commit().unwrap().receipt,
        RecordedReceipt::Transfer(receipt)
    );
    assert_eq!(policy.persistent_state(), Err(Error::Interrupted));
    assert_eq!(policy.cancel(&plan), Err(Error::HostRefused));
    let handle = policy
        .reconcile_committed(&plan, &ordinary, &local)
        .unwrap()
        .unwrap();
    let empty = snapshot(local.revision + 1, &[]);
    let retire = policy
        .prepare_empty_retirement(handle, empty.clone(), 9)
        .unwrap();
    let receipt = RetirementReceipt {
        barrier_epoch: 55,
        source_revision: empty.revision,
        retired_generation: 9,
    };
    policy.record_retired(&retire, receipt).unwrap();
    assert_eq!(policy.metadata(handle), Err(Error::Interrupted));
    assert_eq!(
        policy.pending_commit().unwrap().receipt,
        RecordedReceipt::Retirement(receipt)
    );
    policy.reconcile_retired(&retire, &empty).unwrap();
    let mut fixture = SplitFixture::new();
    let split = fixture
        .policy
        .prepare_split(fixture.request(), &fixture.masses)
        .unwrap();
    let (source, destinations, receipt) = fixture.readback(&split);
    fixture
        .policy
        .record_split_committed(&split, receipt.clone())
        .unwrap();
    assert_eq!(
        fixture.policy.pending_commit().unwrap().receipt,
        RecordedReceipt::Batch(receipt)
    );
    assert_eq!(
        fixture.policy.metadata(fixture.parent),
        Err(Error::Interrupted)
    );
    assert_eq!(fixture.policy.cancel(&split), Err(Error::HostRefused));
    fixture
        .policy
        .reconcile_split(&split, &source, &destinations)
        .unwrap();
}
