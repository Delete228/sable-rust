//! Personal lifecycle policy connected to Sunrise's real prepared transfer and
//! common save journal. The controller is temporarily removed from STATE while
//! calling the host, leaving the actual admission policy available to callbacks.
//! All snapshots below contain state keys only; the host moves every live BE,
//! inventory, attachment and schedule itself.
use crate::{
    lifecycle_backend::{self, JournalWrites, PreparedTransfer, TransferOutcome},
    owner_state::{self, Episode, OwnerState, Phase, Profile, RawState},
    scene::{OriginalFixture, Slot},
};
use mod_api::*;
use sable_core::{
    Block, Config, Contents, Id, Payload, Pos, gather,
    host_lifecycle::{
        BatchReceipt, CommitReceipt, HostLifecycle, LifecyclePlan, LogicalIdentity,
        PersistentState, RestoredState, SplitDestination, SplitRequest, StateSnapshot,
    },
    split::{DEFAULT_HEAT_STEPS, SplitPlan, SplitTracker},
};
use sable_sunrise::registry::PropertyCatalog;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug)]
pub(crate) enum Request {
    Capture { slot: u8, seed: Pos },
    Rearm { slot: u8 },
    Return { slot: u8 },
    Split { slot: u8, child: u8 },
    Retire { slot: u8 },
    Dispose,
}
struct Heat {
    cells: BTreeSet<Pos>,
    tracker: SplitTracker,
    plan: Option<SplitPlan>,
}
/// One completed pre-plan readiness barrier, never transfer permission or a
/// second content owner. The next server frontier must observe all inputs anew.
struct CaptureFrontier {
    slot: u8,
    source: LevelRef,
    parent: LevelRef,
    id: Id,
    plot: u32,
    record: Option<OwnerState>,
    envelope: Vec<Pos>,
    barrier_epoch: u64,
}
fn same_optional_record(left: Option<&OwnerState>, right: Option<&OwnerState>) -> bool {
    match (left, right) {
        (Some(left), Some(right)) => owner_state::same_record(left, right),
        (None, None) => true,
        _ => false,
    }
}
/// Opaque caller attachments have no LevelRevisions readset. Re-read their
/// exact canonical values after callbacks and inside the effect transaction.
/// This contains policy metadata only, never live BE payloads or inventories.
struct CanonicalContext {
    owners: Vec<(String, LevelRef, Option<OwnerState>)>,
    registry_source: LevelRef,
    registry: PersistentState,
}
impl CanonicalContext {
    fn new(slots: &[Slot], registry: PersistentState) -> Result<Self, String> {
        let registry_source = slots
            .first()
            .ok_or_else(|| "lifecycle readiness has no registry Source".to_string())?
            .source;
        Ok(Self {
            owners: slots
                .iter()
                .map(|slot| (slot.source_id.clone(), slot.source, slot.record.clone()))
                .collect(),
            registry_source,
            registry,
        })
    }
    fn validate_records(
        &self,
        owners: &[(LevelRef, Option<OwnerState>)],
        registry: Option<&PersistentState>,
    ) -> Result<(), String> {
        if owners.len() != self.owners.len()
            || self.owners.iter().zip(owners).any(
                |((_, expected_source, expected), (source, actual))| {
                    expected_source != source
                        || !same_optional_record(expected.as_ref(), actual.as_ref())
                },
            )
        {
            return Err("lifecycle readiness canonical Source record changed".into());
        }
        if registry != Some(&self.registry) {
            return Err("lifecycle readiness logical registry changed or is missing".into());
        }
        Ok(())
    }
    fn verify(&self, host: &mut dyn ServerHost) -> Result<(), String> {
        let mut owners = Vec::with_capacity(self.owners.len());
        for (source_id, source, _) in &self.owners {
            if host.level(source_id) != Some(*source) {
                return Err("lifecycle readiness lost an actual current Source".into());
            }
            let actual = owner_state::OWNER
                .get_existing(&mut source.attachments(host), Holder::Level)
                .map_err(err)?;
            owners.push((*source, actual));
        }
        let registry = owner_state::REGISTRY
            .get_existing(&mut self.registry_source.attachments(host), Holder::Level)
            .map_err(err)?;
        self.validate_records(&owners, registry.as_ref())
    }
}
impl CaptureFrontier {
    fn completed(
        slot: &Slot,
        envelope: &[Pos],
        result: Result<LiveWorldSaveReceipt, LiveWorldError>,
    ) -> Result<Option<Self>, String> {
        let receipt = match result {
            Ok(receipt) => receipt,
            Err(LiveWorldError::Busy) => return Ok(None),
            Err(problem) => return Err(err(problem)),
        };
        if slot.source == slot.parent || slot.active || slot.world.is_some() {
            return Err("capture frontier requires an inactive distinct actual Source".into());
        }
        let mut source = None;
        let mut parent = None;
        for level in &receipt.levels {
            if level.level == slot.source && source.replace(level).is_some() {
                return Err("capture frontier repeated the actual Source".into());
            }
            if level.level == slot.parent && parent.replace(level).is_some() {
                return Err("capture frontier repeated the actual Parent".into());
            }
        }
        let source =
            source.ok_or_else(|| "capture frontier omitted the actual Source".to_string())?;
        if parent.is_none() || receipt.barrier_epoch == 0 || source.binding.is_some() {
            return Err("capture frontier has no complete unbound Source/Parent coverage".into());
        }
        if let Some(record) = &slot.record {
            let (minimum, generation) = match record.phase {
                Phase::Returned { barrier_epoch } if barrier_epoch > 0 => (barrier_epoch, None),
                Phase::Retired {
                    barrier_epoch,
                    generation,
                } if barrier_epoch > 0 && generation > 0 => (barrier_epoch, Some(generation)),
                Phase::Retiring { generation, .. } if generation > 0 => (1, Some(generation)),
                _ => return Err("capture frontier has no canonical inactive phase".into()),
            };
            if receipt.barrier_epoch < minimum
                || source.retired_generation.is_none_or(|actual| actual == 0)
                || generation.is_some_and(|expected| source.retired_generation != Some(expected))
                || source.retired_members != Some(0)
            {
                return Err(
                    "capture frontier disagrees with the canonical empty retirement".into(),
                );
            }
        }
        Ok(Some(Self {
            slot: slot.number,
            source: slot.source,
            parent: slot.parent,
            id: slot.id,
            plot: slot.plot,
            record: slot.record.clone(),
            envelope: envelope.to_vec(),
            barrier_epoch: receipt.barrier_epoch,
        }))
    }
    fn matches(&self, slot: &Slot, envelope: &[Pos]) -> bool {
        self.slot == slot.number
            && self.source == slot.source
            && self.parent == slot.parent
            && self.id == slot.id
            && self.plot == slot.plot
            && !slot.active
            && slot.world.is_none()
            && self.envelope == envelope
            && same_optional_record(self.record.as_ref(), slot.record.as_ref())
    }
}
/// A completed save frontier for a split's newly retained destination. This
/// stores only checked identity/geometry metadata; it is never a transfer token.
struct SplitFrontier {
    destination: CaptureFrontier,
    parent_slot: u8,
    parent_source: LevelRef,
    parent_level: LevelRef,
    parent_id: Id,
    parent_plot: u32,
    binding: LiveWorldRef,
    record: Option<OwnerState>,
    motion: sable_core::Motion,
    native_epoch: u64,
    geometry: StateSnapshot,
    tracker_epoch: u64,
}
fn same_motion_bits(left: &sable_core::Motion, right: &sable_core::Motion) -> bool {
    left.pose
        .position
        .iter()
        .chain(&left.pose.pivot)
        .chain(&left.pose.rotation)
        .chain(&left.pose.scale)
        .chain(&left.linear)
        .chain(&left.angular)
        .map(|value| value.to_bits())
        .eq(right
            .pose
            .position
            .iter()
            .chain(&right.pose.pivot)
            .chain(&right.pose.rotation)
            .chain(&right.pose.scale)
            .chain(&right.linear)
            .chain(&right.angular)
            .map(|value| value.to_bits()))
}
impl SplitFrontier {
    fn completed(
        parent: &Slot,
        destination: &Slot,
        geometry: &StateSnapshot,
        revisions: LevelRevisions,
        cells: &[Pos],
        tracker_epoch: u64,
        result: Result<LiveWorldSaveReceipt, LiveWorldError>,
    ) -> Result<Option<Self>, String> {
        let receipt = match result {
            Ok(receipt) => receipt,
            Err(LiveWorldError::Busy) => return Ok(None),
            Err(problem) => return Err(err(problem)),
        };
        let binding = parent
            .world
            .ok_or_else(|| "split frontier needs the actual parent binding".to_string())?;
        if !parent.active
            || !parent.initialized
            || parent.failed
            || !parent.paused
            || parent.source == destination.source
            || parent.parent != destination.parent
            || parent
                .record
                .as_ref()
                .is_none_or(|record| !matches!(record.phase, Phase::Projected { .. }))
        {
            return Err("split frontier parent identity/phase is not ready".into());
        }
        let mut rows = receipt
            .levels
            .iter()
            .filter(|row| row.level == parent.source);
        let row = rows
            .next()
            .ok_or_else(|| "split frontier omitted the actual parent".to_string())?;
        if rows.next().is_some()
            || row.binding != Some(binding)
            || row.block_revision != geometry.revision
            || revisions.block_revision != geometry.revision
            || row.content_revision != revisions.content_revision
        {
            return Err("split frontier omitted or changed its actual parent readset".into());
        }
        let destination = CaptureFrontier::completed(destination, cells, Ok(receipt))?
            .ok_or_else(|| "completed split frontier unexpectedly lacks a receipt".to_string())?;
        Ok(Some(Self {
            destination,
            parent_slot: parent.number,
            parent_source: parent.source,
            parent_level: parent.parent,
            parent_id: parent.id,
            parent_plot: parent.plot,
            binding,
            record: parent.record.clone(),
            motion: parent.motion.clone(),
            native_epoch: parent.epoch,
            geometry: geometry.clone(),
            tracker_epoch,
        }))
    }
    fn matches(
        &self,
        parent: &Slot,
        destination: &Slot,
        geometry: &StateSnapshot,
        cells: &[Pos],
        tracker_epoch: u64,
    ) -> bool {
        self.destination.matches(destination, cells)
            && self.parent_slot == parent.number
            && self.parent_source == parent.source
            && self.parent_level == parent.parent
            && self.parent_id == parent.id
            && self.parent_plot == parent.plot
            && parent.world == Some(self.binding)
            && parent.active
            && parent.initialized
            && !parent.failed
            && parent.paused
            && self.native_epoch == parent.epoch
            && same_motion_bits(&self.motion, &parent.motion)
            && same_optional_record(self.record.as_ref(), parent.record.as_ref())
            && self.geometry == *geometry
            && self.tracker_epoch == tracker_epoch
    }
}
struct Pending {
    plan: LifecyclePlan,
    transfer: PreparedTransfer,
    source: LevelRef,
    destination: LevelRef,
    source_positions: Vec<Pos>,
    destination_positions: Vec<Pos>,
    records: Vec<(u8, OwnerState)>,
    tracker_epoch: Option<u64>,
    receipt: Option<LiveWorldTransferReceipt>,
    core_adopted: bool,
    retained_samples: Vec<(u8, LiveWorldRef, LiveWorldSample)>,
}

fn projected_template(slot: &Slot, record: &OwnerState) -> Result<(), String> {
    let (
        Phase::Projected {
            pose_revision,
            pose,
        },
        Some(previous),
        Some((revision, current)),
    ) = (&record.phase, slot.record.as_ref(), slot.host_frame)
    else {
        return Err("retained split parent has no canonical projected frame".into());
    };
    let Phase::Projected {
        pose_revision: before_revision,
        pose: before_pose,
    } = &previous.phase
    else {
        return Err("retained split parent was not projected before the batch".into());
    };
    if !slot.active
        || !slot.initialized
        || slot.failed
        || !slot.paused
        || slot.world.is_none()
        || record.logical != previous.logical
        || record.logical.id != slot.id
        || record.logical.plot != slot.plot
        || !same_motion_bits(&record.current.motion, &slot.motion)
        || record.current.native_epoch != slot.epoch
        || !record.current.paused
        || *pose_revision != revision
        || *pose_revision != *before_revision
        || !crate::scene::same_host_bits(*pose, current)
        || !crate::scene::same_host_bits(*pose, *before_pose)
    {
        return Err("retained split parent identity/raw/current frame changed".into());
    }
    Ok(())
}

fn committed_projection_guard(
    slot: &Slot,
    record: &OwnerState,
    binding: Option<LiveWorldRef>,
    sample: Option<Result<LiveWorldSample, LiveWorldError>>,
    retained: Option<(LiveWorldRef, LiveWorldSample)>,
    retired: &[LiveWorldRef],
) -> Result<(), String> {
    match &record.phase {
        Phase::Projected { .. } => {
            projected_template(slot, record)?;
            let (expected, before) = retained
                .ok_or_else(|| "projected batch lost its exact retained sample".to_string())?;
            let actual = sample
                .ok_or_else(|| "retained sample missing".to_string())?
                .map_err(err)?;
            if slot.world != Some(expected)
                || binding != Some(expected)
                || retired.contains(&expected)
                || actual.revision != before.revision
                || !crate::scene::same_host_bits(actual.current, before.current)
                || !crate::scene::same_host_bits(actual.previous, before.previous)
            {
                return Err(
                    "committed split did not preserve the exact parent binding/sample".into(),
                );
            }
        }
        Phase::OwnedUnprojected { .. } | Phase::Returned { .. } => {
            if binding.is_some() {
                return Err(
                    "committed unprojected/returned phase still has an actual binding".into(),
                );
            }
            if let Some(old) = slot.world
                && (!retired.contains(&old) || !matches!(sample, Some(Err(LiveWorldError::Stale))))
            {
                return Err(
                    "actual transfer did not receipt and stale the previous binding".into(),
                );
            }
        }
        _ => return Err("unexpected committed lifecycle phase".into()),
    }
    Ok(())
}
struct PendingRetirement {
    plan: LifecyclePlan,
    slot: u8,
    generation: u64,
    core_adopted: bool,
    registry_level: LevelRef,
    registry: PersistentState,
    receipt: Option<LiveWorldSaveReceipt>,
}
#[derive(Default)]
pub(crate) struct Controller {
    core: Option<HostLifecycle>,
    request: Option<Request>,
    pending: Option<Pending>,
    heat: BTreeMap<Id, Heat>,
    tickets: Vec<(LevelRef, Id, [i32; 2])>,
    bootstrap: Option<TicketController>,
    pub(crate) updates: Vec<(u8, OwnerState)>,
    action: String,
    phase: String,
    problem: Option<String>,
    planned_epoch: Option<u64>,
    barrier_epoch: Option<u64>,
    moved_cells: u32,
    retired_generations: Vec<u64>,
    receipt_levels: Vec<(String, u64, u64)>,
    attempts: u32,
    disposed: bool,
    retirement: Option<PendingRetirement>,
    policy_observations: Vec<(Slot, StateSnapshot)>,
    split_trace: Option<String>,
    capture_frontier: Option<CaptureFrontier>,
    split_frontier: Option<SplitFrontier>,
}
fn err(value: impl std::fmt::Debug) -> String {
    format!("{value:?}")
}
fn quote(value: &str) -> String {
    let mut out = String::from("\"");
    for ch in value.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            ch if ch < ' ' => out.push_str(&format!("\\u{:04x}", u32::from(ch))),
            ch => out.push(ch),
        }
    }
    out.push('"');
    out
}
impl Controller {
    pub(crate) fn busy(&self) -> bool {
        self.request.is_some() || self.unresolved()
    }
    pub(crate) fn unresolved(&self) -> bool {
        self.pending.is_some() || self.retirement.is_some()
    }
    pub(crate) fn is_disposed(&self) -> bool {
        self.disposed
    }
    pub(crate) fn queue(&mut self, request: Request) -> Result<(), String> {
        if self.request.is_some() || self.pending.is_some() || self.retirement.is_some() {
            return Err(
                "a lifecycle operation is already pending; inspect its actual outcome".into(),
            );
        }
        if self.disposed {
            return Err(
                "this review runtime was disposed; restart restores its durable logical metadata"
                    .into(),
            );
        }
        self.action = format!("{request:?}");
        self.phase = "queued".into();
        self.problem = None;
        self.planned_epoch = None;
        self.barrier_epoch = None;
        self.moved_cells = 0;
        self.split_trace = None;
        self.retired_generations.clear();
        self.receipt_levels.clear();
        self.attempts = 0;
        self.capture_frontier = None;
        self.split_frontier = None;
        self.request = Some(request);
        Ok(())
    }
    pub(crate) fn status(&self) -> String {
        let levels = self
            .receipt_levels
            .iter()
            .map(|(level, block, content)| {
                format!(
                    r#"{{"level":{},"block_revision":{block},"content_revision":{content}}}"#,
                    quote(level)
                )
            })
            .collect::<Vec<_>>()
            .join(",");
        let mut output = format!(
            r#"{{"ready":{},"pending":{},"action":{},"phase":{},"planned_epoch":{},"barrier_epoch":{},"moved_cells":{},"retired_generations":{:?},"receipt_levels":[{}],"error":{},"attempts":{},"bootstrap_tickets":{},"disposed":{}}}"#,
            self.core.is_some(),
            self.request.is_some() || self.pending.is_some() || self.retirement.is_some(),
            quote(&self.action),
            quote(&self.phase),
            self.planned_epoch
                .map_or_else(|| "null".into(), |v| v.to_string()),
            self.barrier_epoch
                .map_or_else(|| "null".into(), |v| v.to_string()),
            self.moved_cells,
            self.retired_generations,
            levels,
            self.problem.as_deref().map_or_else(|| "null".into(), quote),
            self.attempts,
            self.tickets.len(),
            self.disposed
        );
        output.pop();
        format!(
            "{output},\"capture_frontier_barrier\":{},\"split_frontier_barrier\":{},\"split_trace\":{}}}",
            self.capture_frontier.as_ref().map_or_else(
                || "null".into(),
                |frontier| frontier.barrier_epoch.to_string()
            ),
            self.split_frontier.as_ref().map_or_else(
                || "null".into(),
                |frontier| frontier.destination.barrier_epoch.to_string()
            ),
            self.split_trace.as_deref().unwrap_or("null")
        )
    }
    /// Run only at the real ServerTickStart frontier. No STATE RefCell borrow
    /// may span this method: normal host callbacks must reach edit admission.
    pub(crate) fn turn(
        &mut self,
        host: &mut dyn ServerHost,
        slots: &[Slot],
        bootstrap: TicketController,
    ) {
        self.bootstrap = Some(bootstrap);
        if self.disposed {
            return;
        }
        let result = (|| -> Result<(), String> {
            if slots.iter().any(|slot| {
                slot.record
                    .as_ref()
                    .is_some_and(|r| matches!(r.phase, Phase::Retiring { .. }))
                    && slot.empty_witness.is_none()
            }) {
                self.phase = "waiting_actual_empty_retirement_witness".into();
                return Ok(());
            }
            if self.core.is_none() {
                if slots
                    .iter()
                    .any(|s| !s.initialized || (s.active && s.world.is_none()))
                {
                    return Ok(());
                }
                self.restore_or_adopt(host, slots)?;
            }
            self.refresh(host, slots)?;
            self.release_projected_tickets(host, slots)?;
            if self.pending.is_some() {
                return self.poll_pending(host, slots);
            }
            if self.retirement.is_some() {
                return self.poll_retirement(host, slots);
            }
            let Some(request) = self.request.clone() else {
                return Ok(());
            };
            self.attempts = self
                .attempts
                .checked_add(1)
                .ok_or_else(|| "lifecycle wait budget exhausted".to_string())?;
            if self.attempts > 200 {
                return Err(
                    "bounded lifecycle readiness wait exhausted; no source move was inferred"
                        .into(),
                );
            }
            if slots.iter().any(|s| {
                s.failed || !s.initialized || (s.active && (!s.paused || s.world.is_none()))
            }) {
                return Err(
                    "lifecycle requires every active source ready, healthy and explicitly paused"
                        .into(),
                );
            }
            match request {
                Request::Capture { slot, seed } => {
                    self.prepare_capture(host, slots, slot, seed, None)
                }
                Request::Rearm { slot } => {
                    let fixture = OriginalFixture::for_slot(slot)?;
                    self.prepare_capture(host, slots, slot, fixture.origin, Some(fixture))
                }
                Request::Return { slot } => self.prepare_return(host, slots, slot),
                Request::Split { slot, child } => self.prepare_split(host, slots, slot, child),
                Request::Retire { slot } => self.retire(host, slots, slot),
                Request::Dispose => {
                    if slots.iter().any(|s| s.active || s.native_bodies != 0) {
                        return Err(
                            "dispose requires actual return/empty retirement of every live owner"
                                .into(),
                        );
                    }
                    for slot in slots {
                        slot.source.checkpoint(host).map_err(err)?;
                    }
                    for slot in slots {
                        host.runtime_levels().unload(&slot.source_id).map_err(err)?;
                    }
                    self.disposed = true;
                    self.phase = "unload_requested".into();
                    self.request = None;
                    Ok(())
                }
            }
        })();
        if let Err(problem) = result {
            self.problem = Some(problem);
            self.phase = if self.pending.is_some() || self.retirement.is_some() {
                "unreconciled"
            } else {
                "refused"
            }
            .into();
            if self.pending.is_none() && self.retirement.is_none() {
                self.request = None;
                self.capture_frontier = None;
                self.split_frontier = None;
            }
        }
    }
    fn slot(slots: &[Slot], number: u8) -> Result<&Slot, String> {
        slots
            .iter()
            .find(|s| s.number == number)
            .ok_or_else(|| "unknown owned Source pool slot".into())
    }
    fn observation(
        host: &dyn ServerHost,
        slot: &Slot,
        catalog: &mut PropertyCatalog,
    ) -> Result<StateSnapshot, String> {
        if slot.world.is_none()
            && let Some(OwnerState {
                phase: Phase::Retiring { generation, .. },
                ..
            }) = &slot.record
        {
            let (actual_generation, block_revision, content_revision) =
                slot.empty_witness.ok_or_else(|| {
                    "complete empty membership awaits an actual host receipt".to_string()
                })?;
            let current = slot.source.revisions(host).map_err(err)?;
            if actual_generation != *generation
                || current.block_revision != block_revision
                || current.content_revision != content_revision
                || slot.source.live_world(host).map_err(err)?.is_some()
            {
                return Err("actual empty Source receipt is no longer current".into());
            }
            return Ok(StateSnapshot {
                revision: block_revision,
                cells: BTreeMap::new(),
            });
        }
        let positions = if let Some(world) = slot.world {
            let sample = world.sample(host).map_err(err)?;
            if slot.host_frame.is_none_or(|(revision, pose)| {
                revision != sample.revision || !crate::scene::same_host_bits(pose, sample.current)
            }) || !crate::frames::equivalent(&slot.motion, sample.current)
            {
                return Err(
                    "actual host frame no longer matches accepted raw physical state".into(),
                );
            }
            world
                .cells(host)
                .map_err(err)?
                .into_iter()
                .map(|cell| cell.pos)
                .collect::<Vec<_>>()
        } else {
            match slot.record.as_ref().map(|r| &r.phase) {
                Some(Phase::OwnedUnprojected { members, .. })
                | Some(Phase::Retiring { members, .. }) => members.clone(),
                _ => slot.members.clone(),
            }
        };
        Ok(lifecycle_backend::read_snapshot(host, slot.source, &positions, catalog)?.state)
    }
    fn restore_or_adopt(
        &mut self,
        host: &mut dyn ServerHost,
        slots: &[Slot],
    ) -> Result<(), String> {
        let registry_source = slots
            .first()
            .ok_or_else(|| "no owned source pool".to_string())?
            .source;
        let mut catalog = PropertyCatalog::new(1).map_err(err)?;
        let mut states = BTreeMap::new();
        let mut records = Vec::new();
        let saved = owner_state::REGISTRY
            .get_existing(&mut registry_source.attachments(host), Holder::Level)
            .map_err(err)?;
        match saved {
            None => {
                let authorized = [
                    "SABLE_PHASE3_RECOVER_INITIAL_SETUP",
                    "SABLE_PHASE3_INITIALIZE",
                ]
                .iter()
                .any(|key| std::env::var_os(key).as_deref() == Some(std::ffi::OsStr::new("1")));
                if !authorized
                    || slots.len() != 4
                    || slots
                        .iter()
                        .any(|s| s.record.is_some() || !s.active || s.epoch != 0 || !s.paused)
                {
                    return Err("canonical lifecycle metadata absent; only explicit checked initial-fixture adoption is authorized".into());
                }
                for slot in slots {
                    let observed = Self::observation(host, slot, &mut catalog)?;
                    let world = slot
                        .world
                        .ok_or_else(|| "initial adoption needs real binding".to_string())?;
                    let sample = world.sample(host).map_err(err)?;
                    let logical = LogicalIdentity {
                        id: slot.id,
                        plot: slot.plot,
                        generation: 1,
                        parent: None,
                    };
                    let profile = match slot.number {
                        0 => Profile::Translate,
                        1 => Profile::Yaw,
                        2 => Profile::Pitch,
                        3 => Profile::Dynamic,
                        _ => return Err("unknown initial fixture".into()),
                    };
                    let return_offset = if slot.number == 3 {
                        [12, 32, 24]
                    } else {
                        [
                            i32::from(slot.number) * 2,
                            if slot.number == 2 { 48 } else { 24 },
                            i32::from(slot.number) * 48,
                        ]
                    };
                    let record = OwnerState {
                        logical,
                        current: RawState {
                            motion: slot.motion.clone(),
                            native_epoch: 0,
                            paused: true,
                        },
                        episode: Episode {
                            profile,
                            initial: slot.motion.clone(),
                            return_offset,
                            cells: observed.cells.clone(),
                        },
                        phase: Phase::Projected {
                            pose_revision: sample.revision,
                            pose: sample.current,
                        },
                    };
                    states.insert(
                        slot.id,
                        RestoredState {
                            state: observed,
                            motion: slot.motion.clone(),
                        },
                    );
                    records.push((slot.number, record));
                }
                let persistent = PersistentState {
                    owners: records.iter().map(|(_, r)| r.logical).collect(),
                    plot_generations: records
                        .iter()
                        .map(|(_, r)| (r.logical.plot, r.logical.generation))
                        .collect(),
                };
                // This adoption records existing genuine sources. It executes no
                // synthetic capture/transfer and writes no block or inventory.
                let writes = JournalWrites {
                    owners: records
                        .iter()
                        .map(|(number, r)| (Self::slot(slots, *number).unwrap().source, r.clone()))
                        .collect(),
                    registry_level: registry_source,
                    registry: persistent.clone(),
                };
                host.transaction(|host, _| writes.write(host))?;
                writes.verify(host)?;
                let receipt = registry_source.checkpoint(host).map_err(err)?;
                if receipt.barrier_epoch == 0
                    || slots.iter().any(|slot| {
                        !receipt
                            .levels
                            .iter()
                            .any(|level| level.level == slot.source && level.binding == slot.world)
                    })
                {
                    return Err(
                        "initial metadata adoption lacks a complete actual host barrier receipt"
                            .into(),
                    );
                }
                self.core = Some(
                    HostLifecycle::restore(Config::default(), persistent, states).map_err(err)?,
                );
                self.updates.extend(records);
                self.phase = "initial_existing_sources_adopted".into();
                self.barrier_epoch = Some(receipt.barrier_epoch);
            }
            Some(saved) => {
                for logical in &saved.owners {
                    let slot =
                        slots
                            .iter()
                            .find(|slot| slot.id == logical.id)
                            .ok_or_else(|| {
                                "canonical logical owner has no actual source".to_string()
                            })?;
                    let record = owner_state::OWNER
                        .get_existing(&mut slot.source.attachments(host), Holder::Level)
                        .map_err(err)?
                        .ok_or_else(|| "canonical Source owner record missing".to_string())?;
                    if record.logical != *logical
                        || !matches!(
                            record.phase,
                            Phase::Projected { .. }
                                | Phase::OwnedUnprojected { .. }
                                | Phase::Retiring { .. }
                        )
                    {
                        return Err("canonical registry/source phase identity disagrees".into());
                    }
                    states.insert(
                        logical.id,
                        RestoredState {
                            state: Self::observation(host, slot, &mut catalog)?,
                            motion: record.current.motion.clone(),
                        },
                    );
                }
                self.core =
                    Some(HostLifecycle::restore(Config::default(), saved, states).map_err(err)?);
                self.phase = "restored".into();
            }
        }
        Ok(())
    }
    fn refresh(&mut self, host: &dyn ServerHost, slots: &[Slot]) -> Result<(), String> {
        if self.pending.is_some() || self.retirement.is_some() {
            return Ok(());
        }
        let core = self.core.as_mut().unwrap();
        let mut catalog = PropertyCatalog::new(1).map_err(err)?;
        let identities = core.persistent_state().map_err(err)?.owners;
        let mut observations = Vec::new();
        for logical in identities {
            let slot = slots
                .iter()
                .find(|s| s.id == logical.id)
                .ok_or_else(|| "logical owner missing its actual source".to_string())?;
            let state = Self::observation(host, slot, &mut catalog)?;
            observations.push((slot.clone(), state));
        }
        let masses = catalog
            .mass_registry(
                1,
                observations
                    .iter()
                    .flat_map(|(_, state)| state.cells.values())
                    .map(String::as_str),
            )
            .map_err(err)?;
        self.policy_observations.clear();
        for (slot, state) in observations {
            core.publish_observation(
                core.handle(slot.id).map_err(err)?,
                state.clone(),
                slot.motion.clone(),
                &masses,
            )
            .map_err(err)?;
            self.policy_observations.push((slot.clone(), state.clone()));
            let solids = state.cells.keys().copied().collect::<BTreeSet<_>>();
            let heat = self.heat.entry(slot.id).or_insert_with(|| Heat {
                cells: BTreeSet::new(),
                tracker: SplitTracker::new(),
                plan: None,
            });
            for removed in heat.cells.difference(&solids) {
                heat.tracker.on_removed(*removed);
                heat.plan = None;
            }
            for added in solids.difference(&heat.cells) {
                heat.tracker.on_added(*added);
                heat.plan = None;
            }
            if heat.plan.is_none() {
                heat.plan = heat
                    .tracker
                    .tick(&solids, DEFAULT_HEAT_STEPS, true)
                    .into_iter()
                    .next();
            }
            heat.cells = solids;
        }
        Ok(())
    }
    fn new_id(host: &mut dyn ServerHost, parent: LevelRef) -> Result<Id, String> {
        let mut rng = parent
            .random(host)
            .ok_or_else(|| "actual parent random unavailable".to_string())?;
        let mut id = [0u8; 16];
        id[..8].copy_from_slice(&rng.next_long().to_le_bytes());
        id[8..].copy_from_slice(&rng.next_long().to_le_bytes());
        if !parent.set_random(host, rng) {
            return Err("parent refused random stream advance".into());
        }
        id[6] = (id[6] & 15) | 64;
        id[8] = (id[8] & 63) | 128;
        Ok(Id(id))
    }
    fn hold(
        &mut self,
        host: &mut dyn ServerHost,
        source: LevelRef,
        id: Id,
        positions: &[Pos],
    ) -> Result<bool, String> {
        let controller = self.bootstrap.unwrap();
        let owner = TicketOwner::Entity(u128::from_le_bytes(id.0));
        let mut ready = true;
        for column in positions
            .iter()
            .map(|p| [p[0].div_euclid(16), p[2].div_euclid(16)])
            .collect::<BTreeSet<_>>()
        {
            let chunk = source
                .chunk_ref(host, column)
                .ok_or_else(|| "actual destination chunk handle unavailable".to_string())?;
            let _ = chunk.set_mod_forced(host, controller, owner, true, false);
            let actual = source
                .mod_forced_chunks(host, controller)
                .ok_or_else(|| "owned bootstrap ticket query unavailable".to_string())?;
            if !actual.contains(&(owner, column, false)) {
                return Err("scoped lifecycle ticket was refused".into());
            }
            if !self.tickets.contains(&(source, id, column)) {
                self.tickets.push((source, id, column));
            }
            ready &= chunk.loaded(host)
                && source.should_tick_blocks_at(host, [column[0] * 16, 0, column[1] * 16])
                    == Some(true);
        }
        Ok(ready)
    }
    fn release_projected_tickets(
        &mut self,
        host: &mut dyn ServerHost,
        slots: &[Slot],
    ) -> Result<(), String> {
        let controller = self.bootstrap.unwrap();
        let mut remaining = Vec::new();
        for (source, id, column) in self.tickets.drain(..) {
            let retained = slots
                .iter()
                .any(|s| s.source == source && s.world.is_some());
            let still_needed = self.request.is_some() || self.pending.is_some();
            if !retained && still_needed {
                remaining.push((source, id, column));
                continue;
            }
            let chunk = source
                .chunk_ref(host, column)
                .ok_or_else(|| "ticket release chunk unavailable".to_string())?;
            let owner = TicketOwner::Entity(u128::from_le_bytes(id.0));
            let _ = chunk.set_mod_forced(host, controller, owner, false, false);
            if source
                .mod_forced_chunks(host, controller)
                .ok_or_else(|| "ticket release query unavailable".to_string())?
                .contains(&(owner, column, false))
            {
                return Err("lifecycle bootstrap release refused".into());
            }
        }
        self.tickets = remaining;
        Ok(())
    }
    fn gather(host: &dyn ServerHost, parent: LevelRef, seed: Pos) -> Result<Vec<Pos>, String> {
        let mut problem = None;
        let result = gather::connected(
            seed,
            512,
            |position| match parent.block_state(host, position) {
                Some(state) => host.block_states().name(state.block()).map_or_else(
                    |e| {
                        problem = Some(err(e));
                        false
                    },
                    |name| {
                        !matches!(
                            name,
                            "minecraft:air" | "minecraft:cave_air" | "minecraft:void_air"
                        )
                    },
                ),
                None => {
                    problem = Some("connected selection touched an unavailable parent cell".into());
                    false
                }
            },
            |_, _, _| true,
        )
        .map_err(err)?;
        if let Some(problem) = problem {
            return Err(problem);
        }
        result
            .blocks
            .map(|positions| positions.into_iter().collect())
            .ok_or_else(|| format!("actual connected selection refused: {:?}", result.state))
    }
    fn empty_destination(
        host: &dyn ServerHost,
        slot: &Slot,
        positions: &[Pos],
        catalog: &mut PropertyCatalog,
    ) -> Result<StateSnapshot, String> {
        if slot.active
            || slot.world.is_some()
            || slot.source.live_world(host).map_err(err)?.is_some()
        {
            return Err("destination Source pool slot still has an active owner".into());
        }
        let state = lifecycle_backend::read_snapshot(host, slot.source, positions, catalog)?.state;
        if !state.cells.is_empty() {
            return Err("actual destination contains live blocks; no replacement permitted".into());
        }
        Ok(state)
    }
    fn validate_capture_context(
        &self,
        host: &mut dyn ServerHost,
        slots: &[Slot],
        destination: &Slot,
    ) -> Result<(), String> {
        crate::validate_policy_slots(slots)?;
        self.canonical_context(slots)?.verify(host)?;
        // This ordinary read also rejects a stale Parent handle. No world
        // revision is treated as frozen across the later server turn.
        destination.parent.revisions(host).map_err(err)?;
        if destination.active
            || destination.world.is_some()
            || destination.source.live_world(host).map_err(err)?.is_some()
        {
            return Err("capture readiness Source is no longer unbound and inactive".into());
        }
        Ok(())
    }
    fn canonical_context(&self, slots: &[Slot]) -> Result<CanonicalContext, String> {
        let registry = self
            .core
            .as_ref()
            .ok_or_else(|| "lifecycle readiness logical owner is unavailable".to_string())?
            .persistent_state()
            .map_err(err)?;
        CanonicalContext::new(slots, registry)
    }
    fn prepare_capture(
        &mut self,
        host: &mut dyn ServerHost,
        slots: &[Slot],
        number: u8,
        seed: Pos,
        rearm: Option<OriginalFixture>,
    ) -> Result<(), String> {
        let slot = Self::slot(slots, number)?;
        if rearm.is_some() {
            if slot.active
                || slot.world.is_some()
                || slot.source.live_world(host).map_err(err)?.is_some()
                || slot.record.as_ref().is_none_or(|record| {
                    !matches!(record.phase, Phase::Returned { .. } | Phase::Retired { .. })
                })
            {
                return Err(
                    "rearm requires an inactive, actually returned or retired Source pool slot"
                        .into(),
                );
            }
            if self
                .core
                .as_ref()
                .unwrap()
                .persistent_state()
                .map_err(err)?
                .owners
                .iter()
                .any(|logical| logical.id == slot.id)
            {
                return Err("rearm cannot overwrite a still-owned logical identity; complete retirement first".into());
            }
        }
        let positions = Self::gather(host, slot.parent, seed)?;
        let anchor = Config::default().anchor();
        let destination_positions = positions
            .iter()
            .map(|p| {
                let mut q = [0; 3];
                for axis in 0..3 {
                    q[axis] = p[axis]
                        .checked_sub(seed[axis])
                        .and_then(|v| v.checked_add(anchor[axis]))
                        .ok_or_else(|| "capture coordinate overflow".to_string())?;
                }
                Ok(q)
            })
            .collect::<Result<Vec<_>, String>>()?;
        let source_readset = rearm
            .as_ref()
            .map_or_else(|| positions.clone(), |fixture| fixture.envelope.clone());
        let destination_readset = if rearm.is_some() {
            source_readset
                .iter()
                .map(|p| {
                    let mut q = [0; 3];
                    for axis in 0..3 {
                        q[axis] = p[axis]
                            .checked_sub(seed[axis])
                            .and_then(|v| v.checked_add(anchor[axis]))
                            .ok_or_else(|| "rearm envelope coordinate overflow".to_string())?;
                    }
                    Ok(q)
                })
                .collect::<Result<Vec<_>, String>>()?
        } else {
            destination_positions.clone()
        };
        if self
            .capture_frontier
            .as_ref()
            .is_some_and(|frontier| !frontier.matches(slot, &destination_readset))
        {
            return Err("capture frontier identity, canonical phase or envelope changed".into());
        }
        // Allocate an id once readiness is achieved; the temporary reservation
        // uses the current inactive pool identity and is released explicitly.
        if !self.hold(host, slot.source, slot.id, &destination_readset)? {
            self.capture_frontier = None;
            self.phase = "waiting_source_readiness".into();
            return Ok(());
        }
        self.validate_capture_context(host, slots, slot)?;
        if self.capture_frontier.is_none() {
            // Selected chunks can already be loaded/ticking while ordinary
            // streaming still has generation work to settle. Do that at a
            // public common frontier BEFORE allocating any plan or transfer.
            let result = slot.source.checkpoint(host);
            let Some(frontier) = CaptureFrontier::completed(slot, &destination_readset, result)?
            else {
                self.phase = "waiting_capture_checkpoint".into();
                return Ok(());
            };
            self.validate_capture_context(host, slots, slot)?;
            self.capture_frontier = Some(frontier);
            self.phase = "waiting_capture_frontier_turn".into();
            return Ok(()); // Fresh selection and observations on a later turn.
        }
        let mut catalog = PropertyCatalog::new(1).map_err(err)?;
        let source =
            lifecycle_backend::read_snapshot(host, slot.parent, &source_readset, &mut catalog)?
                .state;
        let destination = Self::empty_destination(host, slot, &destination_readset, &mut catalog)?;
        if let Some(fixture) = &rearm {
            fixture.validate(&source, &positions, &catalog)?;
            fixture.validate_live_entities(host, slot.parent)?;
        }
        let masses = catalog
            .mass_registry(1, source.cells.values().map(String::as_str))
            .map_err(err)?;
        let id = Self::new_id(host, slot.parent)?;
        self.validate_capture_context(host, slots, slot)?;
        self.refresh(host, slots)?;
        self.capture_frontier = None;
        let core = self.core.as_mut().unwrap();
        let selected = positions
            .iter()
            .zip(&destination_positions)
            .map(|(from, to)| (*to, source.cells[from].clone()))
            .collect::<BTreeMap<_, _>>();
        let plan = core
            .prepare_capture(source, destination, id, seed, &positions, &masses)
            .map_err(err)?;
        let persistent = core.planned_persistent_state(&plan).map_err(err)?;
        let logical = *persistent
            .owners
            .iter()
            .find(|v| v.id == id)
            .ok_or_else(|| "planned capture identity missing".to_string())?;
        let offset = std::array::from_fn(|axis| seed[axis] - anchor[axis]);
        let motion = rearm.as_ref().map_or_else(
            || plan.motion().clone(),
            |fixture| fixture.motion(plan.motion()),
        );
        let profile = rearm
            .as_ref()
            .map_or(Profile::Split, |fixture| fixture.profile);
        let record = OwnerState {
            logical,
            current: RawState {
                motion: motion.clone(),
                native_epoch: slot.epoch,
                paused: true,
            },
            episode: Episode {
                profile,
                initial: motion,
                return_offset: offset,
                cells: selected,
            },
            phase: Phase::OwnedUnprojected {
                barrier_epoch: 1,
                members: destination_positions.clone(),
            },
        };
        let cells = plan
            .mapping()
            .iter()
            .map(|(from, to)| LiveWorldTransferCell::new(slot.parent, *from, slot.source, *to))
            .collect();
        self.begin(
            host,
            slots,
            plan,
            slot.parent,
            slot.source,
            source_readset,
            destination_readset,
            vec![(number, record)],
            None,
            persistent,
            cells,
            rearm,
        )
    }
    fn prepare_return(
        &mut self,
        host: &mut dyn ServerHost,
        slots: &[Slot],
        number: u8,
    ) -> Result<(), String> {
        let slot = Self::slot(slots, number)?;
        let record = slot
            .record
            .as_ref()
            .ok_or_else(|| "return requires canonical owner metadata".to_string())?;
        let mut catalog = PropertyCatalog::new(1).map_err(err)?;
        let source = Self::observation(host, slot, &mut catalog)?;
        let positions = source.cells.keys().copied().collect::<Vec<_>>();
        if !self.hold(host, slot.source, slot.id, &positions)? {
            self.phase = "waiting_source_readback_retention".into();
            return Ok(());
        }
        let destination_positions = positions
            .iter()
            .map(|p| {
                let mut q = [0; 3];
                for axis in 0..3 {
                    q[axis] = p[axis]
                        .checked_add(record.episode.return_offset[axis])
                        .ok_or_else(|| "return coordinate overflow".to_string())?;
                }
                Ok(q)
            })
            .collect::<Result<Vec<_>, String>>()?;
        let destination = lifecycle_backend::read_snapshot(
            host,
            slot.parent,
            &destination_positions,
            &mut catalog,
        )?
        .state;
        let masses = catalog
            .mass_registry(
                1,
                source
                    .cells
                    .values()
                    .chain(destination.cells.values())
                    .map(String::as_str),
            )
            .map_err(err)?;
        crate::validate_policy_slots(slots)?;
        self.refresh(host, slots)?;
        let core = self.core.as_mut().unwrap();
        let handle = core.handle(slot.id).map_err(err)?;
        let plan = core
            .prepare_return(
                handle,
                source,
                destination,
                record.episode.return_offset,
                slot.motion.clone(),
                &masses,
            )
            .map_err(err)?;
        let persistent = core.planned_persistent_state(&plan).map_err(err)?;
        let mut returned = record.clone();
        returned.current.motion = slot.motion.clone();
        returned.current.native_epoch = slot.epoch;
        returned.current.paused = true;
        returned.phase = Phase::Returned { barrier_epoch: 1 };
        let cells = plan
            .mapping()
            .iter()
            .map(|(from, to)| LiveWorldTransferCell::new(slot.source, *from, slot.parent, *to))
            .collect();
        self.begin(
            host,
            slots,
            plan,
            slot.source,
            slot.parent,
            positions,
            destination_positions,
            vec![(number, returned)],
            None,
            persistent,
            cells,
            None,
        )
    }
    fn prepare_split(
        &mut self,
        host: &mut dyn ServerHost,
        slots: &[Slot],
        number: u8,
        child_number: u8,
    ) -> Result<(), String> {
        if number == child_number {
            return Err("split needs a distinct free Source slot".into());
        }
        let slot = Self::slot(slots, number)?;
        let destination_slot = Self::slot(slots, child_number)?;
        if slot.number == 3 || child_number == 3 {
            return Err("this bounded split keeps the separate gravity scene untouched".into());
        }
        let record = slot
            .record
            .as_ref()
            .ok_or_else(|| "split requires canonical metadata".to_string())?;
        let heat = self
            .heat
            .get(&slot.id)
            .ok_or_else(|| "split heat has not observed the actual source".to_string())?;
        let split = heat
            .plan
            .clone()
            .ok_or_else(|| "actual incremental heat has no completed split".to_string())?;
        if split.children.len() != 1 {
            return Err(
                "this four-view runtime command needs exactly one child; core batch supports more"
                    .into(),
            );
        }
        let tracker_epoch = heat.tracker.epoch();
        let child_positions = split.children[0].iter().copied().collect::<Vec<_>>();
        if !self.hold(
            host,
            destination_slot.source,
            destination_slot.id,
            &child_positions,
        )? {
            self.phase = "waiting_source_readiness".into();
            return Ok(());
        }
        let mut catalog = PropertyCatalog::new(1).map_err(err)?;
        let source = Self::observation(host, slot, &mut catalog)?;
        let positions = source.cells.keys().copied().collect::<Vec<_>>();
        if !self.hold(host, slot.source, slot.id, &positions)? {
            self.phase = "waiting_source_readback_retention".into();
            return Ok(());
        }
        let destination =
            Self::empty_destination(host, destination_slot, &child_positions, &mut catalog)?;
        self.validate_capture_context(host, slots, destination_slot)?;
        if self.split_frontier.as_ref().is_some_and(|frontier| {
            !frontier.matches(
                slot,
                destination_slot,
                &source,
                &child_positions,
                tracker_epoch,
            )
        }) {
            return Err("split frontier identity, geometry, motion or tracker changed".into());
        }
        if self.split_frontier.is_none() {
            // Loaded/ticking columns may still have save-generation work. Quiesce
            // through the public common writer BEFORE reserving any core plan,
            // child id or host token, then observe everything again next turn.
            let revisions = slot.source.revisions(host).map_err(err)?;
            let result = destination_slot.source.checkpoint(host);
            let Some(frontier) = SplitFrontier::completed(
                slot,
                destination_slot,
                &source,
                revisions,
                &child_positions,
                tracker_epoch,
                result,
            )?
            else {
                self.phase = "waiting_split_checkpoint".into();
                return Ok(());
            };
            self.validate_capture_context(host, slots, destination_slot)?;
            let after_source = Self::observation(host, slot, &mut catalog)?;
            Self::empty_destination(host, destination_slot, &child_positions, &mut catalog)?;
            if !frontier.matches(
                slot,
                destination_slot,
                &after_source,
                &child_positions,
                tracker_epoch,
            ) || slot.source.revisions(host).map_err(err)? != revisions
            {
                return Err(
                    "split save callback changed the fresh source/destination readset".into(),
                );
            }
            self.split_frontier = Some(frontier);
            self.phase = "waiting_split_frontier_turn".into();
            return Ok(());
        }
        // This completed frontier belongs to one request only. All observations
        // above are fresh on this later turn; transfer still needs its own receipt.
        self.split_frontier = None;
        let masses = catalog
            .mass_registry(1, source.cells.values().map(String::as_str))
            .map_err(err)?;
        let source_mass = masses
            .compute(&state_contents(&source.cells))
            .map_err(err)?;
        let child_geometry = child_positions
            .iter()
            .map(|position| {
                source
                    .cells
                    .get(position)
                    .cloned()
                    .map(|key| (*position, key))
                    .ok_or_else(|| "completed split references a changed actual cell".to_string())
            })
            .collect::<Result<BTreeMap<_, _>, String>>()?;
        let child_mass = masses
            .compute(&state_contents(&child_geometry))
            .map_err(err)?;
        let native_before = slot
            .native_sample
            .ok_or_else(|| "split requires a real current native readback".to_string())?;
        let child_id = Self::new_id(host, slot.parent)?;
        self.validate_capture_context(host, slots, destination_slot)?;
        self.refresh(host, slots)?;
        self.validate_capture_context(host, slots, destination_slot)?;
        if self
            .heat
            .get(&slot.id)
            .is_none_or(|heat| heat.tracker.epoch() != tracker_epoch)
        {
            return Err(
                "actual split tracker changed during destination readiness callbacks".into(),
            );
        }
        let core = self.core.as_mut().unwrap();
        let plan = core
            .prepare_split(
                SplitRequest {
                    parent: core.handle(slot.id).map_err(err)?,
                    source: source.clone(),
                    split,
                    tracker_epoch,
                    children: vec![SplitDestination {
                        id: child_id,
                        state: destination,
                    }],
                    motion: slot.motion.clone(),
                },
                &masses,
            )
            .map_err(err)?;
        let persistent = core.planned_persistent_state(&plan).map_err(err)?;
        let child = &plan.split_children()[0];
        let original_com = slot.motion.pose.transform(source_mass.center);
        let child_com = child.motion.pose.transform(child_mass.center);
        self.split_trace = Some(format!(
            r#"{{"source_revision":{},"tracker_epoch":{},"original_cells":{},"child_cells":{},"frozen_parent_linear":{:?},"frozen_parent_angular":{:?},"frozen_parent_world_com":{:?},"child_world_com":{:?},"planned_child_linear":{:?},"planned_child_angular":{:?},"actual_native_before_pose":{:?},"actual_native_before_velocity":{:?}}}"#,
            source.revision,
            tracker_epoch,
            source.cells.len(),
            child.mapping.len(),
            slot.motion.linear,
            slot.motion.angular,
            original_com,
            child_com,
            child.motion.linear,
            child.motion.angular,
            native_before.0,
            native_before.1
        ));
        let child_cells = child
            .mapping
            .iter()
            .map(|(from, to)| (*to, source.cells[from].clone()))
            .collect::<BTreeMap<_, _>>();
        let retained_cells = source
            .cells
            .iter()
            .filter(|(p, _)| !child_positions.contains(p))
            .map(|(p, key)| (*p, key.clone()))
            .collect::<BTreeMap<_, _>>();
        let child_record = OwnerState {
            logical: *persistent.owners.iter().find(|v| v.id == child_id).unwrap(),
            current: RawState {
                motion: child.motion.clone(),
                native_epoch: slot.epoch,
                paused: true,
            },
            episode: Episode {
                profile: Profile::Split,
                initial: child.motion.clone(),
                return_offset: record.episode.return_offset,
                cells: child_cells,
            },
            phase: Phase::OwnedUnprojected {
                barrier_epoch: 1,
                members: child_positions.clone(),
            },
        };
        let mut parent_record = record.clone();
        parent_record.current.motion = plan.motion().clone();
        parent_record.current.paused = true;
        parent_record.current.native_epoch = slot.epoch;
        parent_record.episode = Episode {
            profile: Profile::Split,
            initial: plan.motion().clone(),
            return_offset: record.episode.return_offset,
            cells: retained_cells.clone(),
        };
        // The host preserves a nonempty source binding during a subset move.
        // Keep its exact accepted projected frame; only the new child is unprojected.
        if let Err(problem) = projected_template(slot, &parent_record) {
            self.core.as_mut().unwrap().cancel(&plan).map_err(err)?;
            return Err(problem);
        }
        let cells = child
            .mapping
            .iter()
            .map(|(from, to)| {
                LiveWorldTransferCell::new(slot.source, *from, destination_slot.source, *to)
            })
            .collect();
        self.begin(
            host,
            slots,
            plan,
            slot.source,
            destination_slot.source,
            positions,
            child_positions,
            vec![(number, parent_record), (child_number, child_record)],
            Some(tracker_epoch),
            persistent,
            cells,
            None,
        )
    }
    #[allow(clippy::too_many_arguments)]
    fn begin(
        &mut self,
        host: &mut dyn ServerHost,
        slots: &[Slot],
        plan: LifecyclePlan,
        source: LevelRef,
        destination: LevelRef,
        source_positions: Vec<Pos>,
        destination_positions: Vec<Pos>,
        records: Vec<(u8, OwnerState)>,
        tracker_epoch: Option<u64>,
        persistent: PersistentState,
        cells: Vec<LiveWorldTransferCell>,
        rearm: Option<OriginalFixture>,
    ) -> Result<(), String> {
        let retained_samples: Result<Vec<_>, String> = records
            .iter()
            .filter(|(_, record)| matches!(record.phase, Phase::Projected { .. }))
            .map(|(number, record)| {
                let slot = Self::slot(slots, *number)?;
                if tracker_epoch.is_none() || slot.source != source {
                    return Err("only a split source may retain a projected binding".into());
                }
                projected_template(slot, record)?;
                let world = slot.world.unwrap();
                let sample = world.sample(host).map_err(err)?;
                let Phase::Projected {
                    pose_revision,
                    pose,
                } = &record.phase
                else {
                    unreachable!()
                };
                if sample.revision != *pose_revision
                    || !crate::scene::same_host_bits(sample.current, *pose)
                {
                    return Err("split source sample differs before host preparation".into());
                }
                Ok((*number, world, sample))
            })
            .collect();
        let retained_samples = match retained_samples {
            Ok(samples) => samples,
            Err(problem) => {
                self.core.as_mut().unwrap().cancel(&plan).map_err(err)?;
                return Err(problem);
            }
        };
        let others = self
            .policy_observations
            .iter()
            .filter(|(s, _)| s.source != source && s.source != destination)
            .cloned()
            .collect::<Vec<_>>();
        // A prepared core plan still exposes its original persistent registry.
        // Freeze that expected metadata before host prepare/stage callbacks;
        // a fresh read of changed attachments is never accepted as a baseline.
        let canonical = self.canonical_context(slots);
        let validate_others = |host: &mut dyn ServerHost| -> Result<(), String> {
            crate::validate_policy_slots(slots)?;
            canonical.as_ref().map_err(Clone::clone)?.verify(host)?;
            for (number, world, expected) in &retained_samples {
                let actual = world.sample(host).map_err(err)?;
                if Self::slot(slots, *number)?.world != Some(*world)
                    || actual.revision != expected.revision
                    || !crate::scene::same_host_bits(actual.current, expected.current)
                    || !crate::scene::same_host_bits(actual.previous, expected.previous)
                {
                    return Err("retained split sample changed during preparation callbacks".into());
                }
            }
            let mut catalog = PropertyCatalog::new(1).map_err(err)?;
            for (slot, before) in &others {
                if Self::observation(host, slot, &mut catalog)? != *before {
                    return Err(
                        "other live owner changed since core aggregate policy snapshot".into(),
                    );
                }
            }
            Ok(())
        };
        if let Err(problem) = validate_others(host) {
            self.core.as_mut().unwrap().cancel(&plan).map_err(err)?;
            return Err(problem);
        }
        let mut transfer = match PreparedTransfer::prepare(host, source, cells) {
            Ok(t) => t,
            Err(problem) => {
                self.core.as_mut().unwrap().cancel(&plan).map_err(err)?;
                return Err(problem);
            }
        };
        if let Err(problem) = validate_others(host) {
            transfer.cancel_unused(host)?;
            self.core.as_mut().unwrap().cancel(&plan).map_err(err)?;
            return Err(problem);
        }
        let core = self.core.as_mut().unwrap();
        let templates = records.clone();
        let registry_level = slots[0].source;
        let outcome = transfer.execute(
            host,
            |host| {
                let mut catalog = PropertyCatalog::new(1).map_err(err)?;
                validate_others(host)?;
                let current_source = lifecycle_backend::read_snapshot(
                    host,
                    source,
                    &source_positions,
                    &mut catalog,
                )?
                .state;
                let current_destination = lifecycle_backend::read_snapshot(
                    host,
                    destination,
                    &destination_positions,
                    &mut catalog,
                )?
                .state;
                if let Some(fixture) = &rearm {
                    let connected = Self::gather(host, source, fixture.origin)?;
                    fixture.validate(&current_source, &connected, &catalog)?;
                    fixture.validate_live_entities(host, source)?;
                }
                if let Some(epoch) = tracker_epoch {
                    core.validate_split(&plan, &current_source, &[current_destination], epoch)
                        .map_err(err)?;
                } else {
                    core.validate_pending(&plan, &current_source, &current_destination)
                        .map_err(err)?;
                }
                // Verify again after all actual geometry/fixture reads, at the
                // last guard before caller metadata is constructed and written.
                canonical.as_ref().map_err(Clone::clone)?.verify(host)
            },
            |epoch| {
                let owners = templates
                    .iter()
                    .map(|(number, record)| {
                        let mut record = record.clone();
                        if matches!(record.phase, Phase::Projected { .. }) {
                            if tracker_epoch.is_none()
                                || Self::slot(slots, *number)?.source != source
                            {
                                return Err(
                                    "only the retained split source may stage Projected".into()
                                );
                            }
                            projected_template(Self::slot(slots, *number)?, &record)?;
                        } else {
                            match &mut record.phase {
                                Phase::OwnedUnprojected { barrier_epoch, .. }
                                | Phase::Returned { barrier_epoch } => *barrier_epoch = epoch,
                                _ => return Err("unexpected staged lifecycle phase".into()),
                            }
                        }
                        Ok((Self::slot(slots, *number)?.source, record))
                    })
                    .collect::<Result<Vec<_>, String>>()?;
                Ok(JournalWrites {
                    owners,
                    registry_level,
                    registry: persistent,
                })
            },
        );
        self.planned_epoch = transfer.planned_epoch();
        self.pending = Some(Pending {
            plan,
            transfer,
            source,
            destination,
            source_positions,
            destination_positions,
            records,
            tracker_epoch,
            receipt: None,
            core_adopted: false,
            retained_samples,
        });
        self.request = None;
        self.resolve_outcome(host, slots, outcome)
    }
    fn poll_pending(&mut self, host: &mut dyn ServerHost, slots: &[Slot]) -> Result<(), String> {
        let pending = self.pending.as_ref().unwrap();
        let outcome = if let Some(receipt) = &pending.receipt {
            match pending.transfer.verify_committed(host, receipt) {
                Ok(()) => TransferOutcome::Committed(receipt.clone()),
                Err(problem) => TransferOutcome::CommittedUnreconciled {
                    receipt: receipt.clone(),
                    problem,
                },
            }
        } else {
            pending.transfer.poll(host)
        };
        self.resolve_outcome(host, slots, outcome)
    }
    fn resolve_outcome(
        &mut self,
        host: &mut dyn ServerHost,
        slots: &[Slot],
        outcome: TransferOutcome,
    ) -> Result<(), String> {
        let pending = self.pending.as_mut().unwrap();
        let core = self.core.as_mut().unwrap();
        let mut caller_problem = None;
        let receipt = match outcome {
            TransferOutcome::NotCommitted { problem } => {
                pending.transfer.cancel_not_committed(host)?;
                match core.pending_phase() {
                    Some(sable_core::host_lifecycle::PendingPhase::Prepared) => {
                        core.cancel(&pending.plan).map_err(err)?
                    }
                    Some(sable_core::host_lifecycle::PendingPhase::Unconfirmed) => {
                        core.confirm_not_committed(&pending.plan).map_err(err)?
                    }
                    _ => {
                        return Err(
                            "actual NotCommitted conflicts with a previously latched receipt"
                                .into(),
                        );
                    }
                }
                self.pending = None;
                self.phase = "not_committed".into();
                self.problem = problem;
                return Ok(());
            }
            TransferOutcome::Waiting => {
                core.mark_unconfirmed(&pending.plan).map_err(err)?;
                self.phase = "waiting_actual_receipt".into();
                return Ok(());
            }
            TransferOutcome::Unconfirmed { problem } => {
                core.mark_unconfirmed(&pending.plan).map_err(err)?;
                self.phase = "unconfirmed".into();
                return Err(problem);
            }
            TransferOutcome::Committed(receipt) => receipt,
            TransferOutcome::CommittedUnreconciled { receipt, problem } => {
                caller_problem = Some(problem);
                receipt
            }
        };
        pending.receipt = Some(receipt.clone());
        self.barrier_epoch = Some(receipt.barrier_epoch);
        self.moved_cells = receipt.moved_cells;
        self.retired_generations = receipt
            .retired_bindings
            .iter()
            .map(|b| b.generation())
            .collect();
        self.receipt_levels = receipt
            .levels
            .iter()
            .map(|l| {
                (
                    format!("{:?}", l.level),
                    l.block_revision,
                    l.content_revision,
                )
            })
            .collect();
        let row = |level| {
            receipt
                .levels
                .iter()
                .find(|r| r.level == level)
                .ok_or_else(|| {
                    "actual transfer receipt omitted policy source/destination".to_string()
                })
        };
        if !pending.core_adopted {
            // Latch the genuine receipt BEFORE any fallible content readback.
            if pending.tracker_epoch.is_some() {
                core.record_split_committed(
                    &pending.plan,
                    BatchReceipt {
                        barrier_epoch: receipt.barrier_epoch,
                        source_revision: row(pending.source)?.block_revision,
                        destination_revisions: vec![row(pending.destination)?.block_revision],
                    },
                )
                .map_err(err)?;
            } else {
                core.record_committed(
                    &pending.plan,
                    CommitReceipt {
                        source_revision: row(pending.source)?.block_revision,
                        destination_revision: row(pending.destination)?.block_revision,
                    },
                )
                .map_err(err)?;
            }
            if let Some(problem) = caller_problem.take() {
                self.phase = "committed_caller_metadata_unreconciled".into();
                return Err(problem);
            }
            for (number, record) in &pending.records {
                let slot = Self::slot(slots, *number)?;
                let retained = pending
                    .retained_samples
                    .iter()
                    .find(|(n, _, _)| n == number)
                    .map(|(_, world, sample)| (*world, *sample));
                committed_projection_guard(
                    slot,
                    record,
                    slot.source.live_world(host).map_err(err)?,
                    slot.world.map(|old| old.sample(host)),
                    retained,
                    &receipt.retired_bindings,
                )?;
            }
            let mut catalog = PropertyCatalog::new(1).map_err(err)?;
            let source = lifecycle_backend::read_snapshot(
                host,
                pending.source,
                &pending.source_positions,
                &mut catalog,
            )?
            .state;
            let destination = lifecycle_backend::read_snapshot(
                host,
                pending.destination,
                &pending.destination_positions,
                &mut catalog,
            )?
            .state;
            if pending.tracker_epoch.is_some() {
                core.reconcile_split(&pending.plan, &source, &[destination])
                    .map_err(err)?;
            } else {
                core.reconcile_committed(&pending.plan, &source, &destination)
                    .map_err(err)?;
            }
            pending.core_adopted = true;
        }
        if let Some(problem) = caller_problem {
            self.phase = "committed_caller_metadata_unreconciled".into();
            return Err(problem);
        }
        let mut updates = Vec::new();
        for (number, template) in &pending.records {
            let slot = Self::slot(slots, *number)?;
            let actual = owner_state::OWNER
                .get_existing(&mut slot.source.attachments(host), Holder::Level)
                .map_err(err)?
                .ok_or_else(|| "committed canonical ownership phase missing".to_string())?;
            if actual.logical != template.logical {
                return Err("committed logical identity mismatch".into());
            }
            let retained = pending
                .retained_samples
                .iter()
                .find(|(n, _, _)| n == number)
                .map(|(_, world, sample)| (*world, *sample));
            committed_projection_guard(
                slot,
                &actual,
                slot.source.live_world(host).map_err(err)?,
                slot.world.map(|old| old.sample(host)),
                retained,
                &receipt.retired_bindings,
            )?;
            updates.push((*number, actual));
        }
        let mut catalog = PropertyCatalog::new(1).map_err(err)?;
        let source = lifecycle_backend::read_snapshot(
            host,
            pending.source,
            &pending.source_positions,
            &mut catalog,
        )?
        .state;
        if let Some(heat) = self.heat.get_mut(&pending.plan.id()) {
            heat.tracker
                .reconcile(&source.cells.keys().copied().collect());
            heat.cells = source.cells.keys().copied().collect();
            heat.plan = None;
        }
        self.updates.extend(updates);
        self.pending = None;
        self.phase = "ownership_committed".into();
        Ok(())
    }
    fn retire(
        &mut self,
        host: &mut dyn ServerHost,
        slots: &[Slot],
        number: u8,
    ) -> Result<(), String> {
        let slot = Self::slot(slots, number)?;
        let record = slot
            .record
            .as_ref()
            .ok_or_else(|| "empty retirement requires canonical metadata".to_string())?;
        let Phase::Retiring { generation, .. } = &record.phase else {
            return Err("empty retirement requires the actual final-empty host retirement".into());
        };
        let generation = *generation;
        let mut catalog = PropertyCatalog::new(1).map_err(err)?;
        let state = Self::observation(host, slot, &mut catalog)?;
        if !state.cells.is_empty() || slot.source.live_world(host).map_err(err)?.is_some() {
            return Err("retirement refuses remaining live cells or binding".into());
        }
        let core = self.core.as_mut().unwrap();
        let handle = match core.handle(slot.id) {
            Ok(handle) => handle,
            Err(sable_core::Error::StaleHandle) => {
                // A previous process durably saved the planned registry
                // tombstone and Retiring intent but stopped before recording
                // its acknowledged phase. Re-prove the real empty level
                // barrier; never invent a zero-cell transfer or reseed.
                let persistent = core.persistent_state().map_err(err)?;
                if persistent
                    .owners
                    .iter()
                    .any(|logical| logical.id == slot.id)
                    || persistent
                        .plot_generations
                        .get(&record.logical.plot)
                        .is_none_or(|floor| *floor < record.logical.generation)
                {
                    return Err("retiring intent lacks canonical generation highwater".into());
                }
                return self.finish_retired(host, slots, number, generation);
            }
            Err(problem) => return Err(err(problem)),
        };
        let plan = core
            .prepare_empty_retirement(handle, state, generation)
            .map_err(err)?;
        let persistent = core.planned_persistent_state(&plan).map_err(err)?;
        self.retirement = Some(PendingRetirement {
            plan,
            slot: number,
            generation,
            core_adopted: false,
            registry_level: slots[0].source,
            registry: persistent.clone(),
            receipt: None,
        });
        self.request = None;
        let pending = self.retirement.as_ref().unwrap();
        let result = host.transaction(|host, _| -> Result<(), String> {
            let current = Self::observation(host, slot, &mut catalog)?;
            core.validate_empty_retirement(&pending.plan, &current, generation)
                .map_err(err)?;
            owner_state::REGISTRY
                .set(
                    &mut slots[0].source.attachments(host),
                    Holder::Level,
                    persistent.clone(),
                )
                .map_err(err)
        });
        if let Err(problem) = result {
            core.mark_unconfirmed(&pending.plan).map_err(err)?;
            return Err(problem);
        }
        // No cancellation is inferred from a successful root close. The real
        // level checkpoint below decides whether the tombstone is durable.
        self.poll_retirement(host, slots)
    }
    fn poll_retirement(&mut self, host: &mut dyn ServerHost, slots: &[Slot]) -> Result<(), String> {
        let pending = self.retirement.as_mut().unwrap();
        let slot = Self::slot(slots, pending.slot)?;
        let receipt = if let Some(receipt) = &pending.receipt {
            receipt.clone()
        } else {
            match slot.source.checkpoint(host) {
                Ok(receipt) => receipt,
                Err(LiveWorldError::Busy) => {
                    self.phase = "waiting_retirement_barrier".into();
                    return Ok(());
                }
                Err(problem) => {
                    if !pending.core_adopted
                        && self.core.as_ref().unwrap().pending_commit().is_none()
                    {
                        self.core
                            .as_mut()
                            .unwrap()
                            .mark_unconfirmed(&pending.plan)
                            .map_err(err)?;
                    }
                    return Err(err(problem));
                }
            }
        };
        let level = receipt
            .levels
            .iter()
            .find(|level| level.level == slot.source)
            .ok_or_else(|| "actual retirement receipt omitted source".to_string())?;
        if receipt.barrier_epoch == 0
            || level.binding.is_some()
            || level.retired_generation != Some(pending.generation)
            || level.retired_members != Some(0)
        {
            return Err(
                "actual level barrier did not confirm the requested generation retirement".into(),
            );
        }
        pending.receipt = Some(receipt.clone());
        if !pending.core_adopted {
            let core = self.core.as_mut().unwrap();
            core.record_retired(
                &pending.plan,
                sable_core::host_lifecycle::RetirementReceipt {
                    barrier_epoch: receipt.barrier_epoch,
                    source_revision: level.block_revision,
                    retired_generation: pending.generation,
                },
            )
            .map_err(err)?;
            if !receipt
                .levels
                .iter()
                .any(|level| level.level == pending.registry_level)
            {
                return Err(
                    "actual common retirement receipt omitted logical registry Source".into(),
                );
            }
            let actual = owner_state::REGISTRY
                .get_existing(&mut pending.registry_level.attachments(host), Holder::Level)
                .map_err(err)?
                .ok_or_else(|| "actual retirement registry afterimage missing".to_string())?;
            if actual != pending.registry {
                return Err(
                    "actual retirement registry differs from planned highwater/tombstone".into(),
                );
            }
            let mut catalog = PropertyCatalog::new(1).map_err(err)?;
            let current = Self::observation(host, slot, &mut catalog)?;
            core.reconcile_retired(&pending.plan, &current)
                .map_err(err)?;
            pending.core_adopted = true;
        }
        let number = pending.slot;
        let generation = pending.generation;
        self.finish_retired(host, slots, number, generation)?;
        self.retirement = None;
        Ok(())
    }
    fn finish_retired(
        &mut self,
        host: &mut dyn ServerHost,
        slots: &[Slot],
        number: u8,
        generation: u64,
    ) -> Result<(), String> {
        let slot = Self::slot(slots, number)?;
        let receipt = slot.source.checkpoint(host).map_err(err)?;
        if receipt.barrier_epoch == 0
            || !receipt.levels.iter().any(|level| {
                level.level == slot.source
                    && level.binding.is_none()
                    && level.retired_generation == Some(generation)
                    && level.retired_members == Some(0)
            })
        {
            return Err("retirement reconciliation lacks a real empty Source barrier".into());
        }
        let mut retired = slot
            .record
            .clone()
            .ok_or_else(|| "retirement canonical record absent".to_string())?;
        retired.phase = Phase::Retired {
            barrier_epoch: receipt.barrier_epoch,
            generation,
        };
        host.transaction(|host, _| {
            owner_state::OWNER
                .set(
                    &mut slot.source.attachments(host),
                    Holder::Level,
                    retired.clone(),
                )
                .map_err(err)
        })?;
        let final_receipt = slot.source.checkpoint(host).map_err(err)?;
        let actual = owner_state::OWNER
            .get_existing(&mut slot.source.attachments(host), Holder::Level)
            .map_err(err)?
            .ok_or_else(|| "retired canonical record missing after barrier".to_string())?;
        if final_receipt.barrier_epoch < receipt.barrier_epoch
            || !owner_state::same_record(&actual, &retired)
        {
            return Err("retirement phase is not acknowledged by its final actual barrier".into());
        }
        self.updates.push((number, retired));
        self.request = None;
        self.phase = "empty_retired".into();
        self.barrier_epoch = Some(final_receipt.barrier_epoch);
        self.retired_generations = vec![generation];
        Ok(())
    }
}

fn state_contents(cells: &BTreeMap<Pos, String>) -> Contents {
    Contents {
        blocks: cells
            .iter()
            .map(|(position, state)| {
                (
                    *position,
                    Block {
                        state: state.clone(),
                        block_entity: None,
                        extension: Payload::default(),
                    },
                )
            })
            .collect(),
        ..Contents::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Typed metadata fixtures only; no constructed handle/receipt is passed to
    // a host operation and these do not replace the actual capture regression.
    fn inactive_slot() -> Slot {
        let motion = sable_core::Motion::default();
        let logical = LogicalIdentity {
            id: Id([1; 16]),
            plot: 0,
            generation: 1,
            parent: None,
        };
        Slot {
            number: 0,
            source: LevelRef::with_generation(WorldEpoch::new(9), DimensionKey::NETHER, 3),
            parent: LevelRef::new(WorldEpoch::new(9), DimensionKey::OVERWORLD),
            source_id: "sable_port:fixture".into(),
            world: None,
            active: false,
            initialized: true,
            failed: false,
            id: logical.id,
            plot: logical.plot,
            motion: motion.clone(),
            epoch: 0,
            paused: true,
            members: vec![],
            record: Some(OwnerState {
                logical,
                current: RawState {
                    motion: motion.clone(),
                    native_epoch: 0,
                    paused: true,
                },
                episode: Episode {
                    profile: Profile::Translate,
                    initial: motion,
                    return_offset: [0; 3],
                    cells: BTreeMap::from([([0; 3], "minecraft:blackstone".into())]),
                },
                phase: Phase::Returned { barrier_epoch: 171 },
            }),
            native_bodies: 0,
            host_frame: None,
            native_sample: None,
            empty_witness: Some((71, 20, 30)),
        }
    }
    fn readiness_receipt(slot: &Slot) -> LiveWorldSaveReceipt {
        LiveWorldSaveReceipt::new(
            174,
            vec![
                LiveWorldLevelReceipt::new(slot.parent, 10, 15, None, None, None),
                LiveWorldLevelReceipt::new(slot.source, 20, 30, None, Some(71), Some(0)),
            ],
        )
    }

    #[test]
    fn capture_frontier_requires_both_exact_covered_levels_and_empty_source() {
        let slot = inactive_slot();
        let envelope = [[1032, 128, 1032], [1040, 128, 1032]];
        let valid = readiness_receipt(&slot);
        let frontier = CaptureFrontier::completed(&slot, &envelope, Ok(valid.clone()))
            .unwrap()
            .unwrap();
        assert!(frontier.matches(&slot, &envelope));
        assert_eq!(frontier.barrier_epoch, 174);
        let mut invalid = vec![
            LiveWorldSaveReceipt::new(0, valid.levels.clone()),
            LiveWorldSaveReceipt::new(170, valid.levels.clone()),
            LiveWorldSaveReceipt::new(174, vec![valid.levels[0].clone()]),
            LiveWorldSaveReceipt::new(174, vec![valid.levels[1].clone()]),
        ];
        for index in 0..2 {
            let mut duplicate = valid.clone();
            duplicate.levels.push(duplicate.levels[index].clone());
            invalid.push(duplicate);
            let mut wrong_open = valid.clone();
            let old = wrong_open.levels[index].level;
            wrong_open.levels[index].level =
                LevelRef::with_generation(old.epoch(), old.dimension(), old.generation() + 1);
            invalid.push(wrong_open);
        }
        let mut bound = valid.clone();
        bound.levels[1].binding = Some(LiveWorldRef::new(slot.source, 72));
        invalid.push(bound);
        for generation in [None, Some(0)] {
            let mut missing_retirement = valid.clone();
            missing_retirement.levels[1].retired_generation = generation;
            invalid.push(missing_retirement);
        }
        for members in [None, Some(1)] {
            let mut nonempty = valid.clone();
            nonempty.levels[1].retired_members = members;
            invalid.push(nonempty);
        }
        for receipt in invalid {
            assert!(CaptureFrontier::completed(&slot, &envelope, Ok(receipt)).is_err());
        }
        for phase in [
            Phase::Retired {
                barrier_epoch: 171,
                generation: 72,
            },
            Phase::Returned { barrier_epoch: 0 },
            Phase::OwnedUnprojected {
                barrier_epoch: 171,
                members: vec![[0; 3]],
            },
        ] {
            let mut inconsistent = inactive_slot();
            inconsistent.record.as_mut().unwrap().phase = phase;
            assert!(
                CaptureFrontier::completed(&inconsistent, &envelope, Ok(valid.clone())).is_err()
            );
        }
    }

    #[test]
    fn capture_frontier_defers_only_busy_and_never_swallows_revision_or_storage() {
        let slot = inactive_slot();
        let envelope = [[1032, 128, 1032]];
        assert!(
            CaptureFrontier::completed(&slot, &envelope, Err(LiveWorldError::Busy))
                .unwrap()
                .is_none()
        );
        for problem in [
            LiveWorldError::Revision,
            LiveWorldError::Storage,
            LiveWorldError::Stale,
            LiveWorldError::NotSaved,
            LiveWorldError::NotResident,
            LiveWorldError::Denied,
        ] {
            assert!(CaptureFrontier::completed(&slot, &envelope, Err(problem)).is_err());
        }
    }

    #[test]
    fn capture_frontier_rejects_reused_identities_changed_raw_metadata_and_envelope() {
        let slot = inactive_slot();
        let envelope = [[1032, 128, 1032], [1040, 128, 1032]];
        let frontier = CaptureFrontier::completed(&slot, &envelope, Ok(readiness_receipt(&slot)))
            .unwrap()
            .unwrap();
        let mut replacements = Vec::new();
        let mut changed = slot.clone();
        changed.source = LevelRef::with_generation(slot.source.epoch(), slot.source.dimension(), 4);
        replacements.push(changed);
        let mut changed = slot.clone();
        changed.parent = LevelRef::new(WorldEpoch::new(10), slot.parent.dimension());
        replacements.push(changed);
        let mut changed = slot.clone();
        changed.id = Id([2; 16]);
        replacements.push(changed);
        let mut changed = slot.clone();
        changed.plot = 1;
        replacements.push(changed);
        let mut changed = slot.clone();
        changed.record.as_mut().unwrap().logical.generation += 1;
        replacements.push(changed);
        let mut changed = slot.clone();
        changed.record.as_mut().unwrap().current.motion.linear[0] = f64::from_bits(1);
        replacements.push(changed);
        let mut changed = slot.clone();
        changed.record.as_mut().unwrap().phase = Phase::Retired {
            barrier_epoch: 174,
            generation: 71,
        };
        replacements.push(changed);
        let mut changed = slot.clone();
        changed.record = None;
        replacements.push(changed);
        let mut changed = slot.clone();
        changed.active = true;
        replacements.push(changed);
        let mut changed = slot.clone();
        changed.world = Some(LiveWorldRef::new(slot.source, 72));
        replacements.push(changed);
        for changed in replacements {
            assert!(!frontier.matches(&changed, &envelope));
        }
        assert!(!frontier.matches(&slot, &[[1032, 128, 1032], [1056, 128, 1032]]));
    }

    #[test]
    fn capture_frontier_is_per_request_and_busy_queue_cannot_replace_it() {
        let slot = inactive_slot();
        let envelope = [[1032, 128, 1032]];
        let mut controller = Controller {
            capture_frontier: CaptureFrontier::completed(
                &slot,
                &envelope,
                Ok(readiness_receipt(&slot)),
            )
            .unwrap(),
            ..Controller::default()
        };
        controller.queue(Request::Rearm { slot: 0 }).unwrap();
        assert!(controller.capture_frontier.is_none());
        controller.capture_frontier =
            CaptureFrontier::completed(&slot, &envelope, Ok(readiness_receipt(&slot))).unwrap();
        assert!(controller.queue(Request::Return { slot: 0 }).is_err());
        assert!(
            controller
                .capture_frontier
                .as_ref()
                .unwrap()
                .matches(&slot, &envelope)
        );
        assert!(matches!(
            controller.request,
            Some(Request::Rearm { slot: 0 })
        ));
    }

    struct SplitFixture {
        parent: Slot,
        child: Slot,
        state: StateSnapshot,
        revisions: LevelRevisions,
        cells: Vec<Pos>,
        receipt: LiveWorldSaveReceipt,
    }
    fn split_frontier_fixture() -> SplitFixture {
        let mut parent = inactive_slot();
        parent.active = true;
        parent.world = Some(LiveWorldRef::new(parent.source, 72));
        let pose = crate::frames::requested(&parent.motion);
        parent.record.as_mut().unwrap().phase = Phase::Projected {
            pose_revision: 0,
            pose,
        };
        let mut child = inactive_slot();
        child.number = 1;
        child.source = LevelRef::with_generation(parent.source.epoch(), DimensionKey::END, 4);
        child.id = Id([2; 16]);
        child.plot = 1;
        let record = child.record.as_mut().unwrap();
        record.logical.id = child.id;
        record.logical.plot = child.plot;
        let cells = vec![[1039, 128, 1032], [1040, 128, 1032]];
        let state = StateSnapshot {
            revision: 20,
            cells: cells
                .iter()
                .map(|p| (*p, "minecraft:blackstone".into()))
                .collect(),
        };
        let revisions = LevelRevisions::new(20, 30);
        let mut receipt = readiness_receipt(&child);
        receipt.levels.push(LiveWorldLevelReceipt::new(
            parent.source,
            20,
            30,
            parent.world,
            None,
            None,
        ));
        SplitFixture {
            parent,
            child,
            state,
            revisions,
            cells,
            receipt,
        }
    }

    #[test]
    fn split_frontier_requires_parent_binding_and_complete_destination_receipt() {
        let SplitFixture {
            parent,
            child,
            state,
            revisions,
            cells,
            receipt,
        } = split_frontier_fixture();
        let frontier = SplitFrontier::completed(
            &parent,
            &child,
            &state,
            revisions,
            &cells,
            142,
            Ok(receipt.clone()),
        )
        .unwrap()
        .unwrap();
        assert!(frontier.matches(&parent, &child, &state, &cells, 142));
        assert_eq!(frontier.destination.barrier_epoch, 174);
        for index in 0..receipt.levels.len() {
            let mut missing = receipt.clone();
            missing.levels.remove(index);
            assert!(
                SplitFrontier::completed(
                    &parent,
                    &child,
                    &state,
                    revisions,
                    &cells,
                    142,
                    Ok(missing)
                )
                .is_err()
            );
        }
        for changed in [
            LiveWorldLevelReceipt::new(parent.source, 21, 30, parent.world, None, None),
            LiveWorldLevelReceipt::new(parent.source, 20, 31, parent.world, None, None),
            LiveWorldLevelReceipt::new(parent.source, 20, 30, None, None, None),
            LiveWorldLevelReceipt::new(
                parent.source,
                20,
                30,
                Some(LiveWorldRef::new(parent.source, 73)),
                None,
                None,
            ),
        ] {
            let mut invalid = receipt.clone();
            *invalid.levels.last_mut().unwrap() = changed;
            assert!(
                SplitFrontier::completed(
                    &parent,
                    &child,
                    &state,
                    revisions,
                    &cells,
                    142,
                    Ok(invalid)
                )
                .is_err()
            );
        }
        let mut duplicate = receipt.clone();
        duplicate
            .levels
            .push(receipt.levels.last().unwrap().clone());
        assert!(
            SplitFrontier::completed(
                &parent,
                &child,
                &state,
                revisions,
                &cells,
                142,
                Ok(duplicate)
            )
            .is_err()
        );
    }

    #[test]
    fn split_frontier_waits_only_for_busy_without_hiding_revision_or_storage() {
        let SplitFixture {
            parent,
            child,
            state,
            revisions,
            cells,
            ..
        } = split_frontier_fixture();
        assert!(
            SplitFrontier::completed(
                &parent,
                &child,
                &state,
                revisions,
                &cells,
                142,
                Err(LiveWorldError::Busy)
            )
            .unwrap()
            .is_none()
        );
        for error in [
            LiveWorldError::Revision,
            LiveWorldError::Storage,
            LiveWorldError::NotResident,
            LiveWorldError::Stale,
            LiveWorldError::NotCommitted,
        ] {
            assert!(
                SplitFrontier::completed(
                    &parent,
                    &child,
                    &state,
                    revisions,
                    &cells,
                    142,
                    Err(error)
                )
                .is_err()
            );
        }
    }

    #[test]
    fn split_frontier_refuses_callback_motion_geometry_tracker_and_child_reuse() {
        let SplitFixture {
            parent,
            child,
            state,
            revisions,
            cells,
            receipt,
        } = split_frontier_fixture();
        let frontier =
            SplitFrontier::completed(&parent, &child, &state, revisions, &cells, 142, Ok(receipt))
                .unwrap()
                .unwrap();
        let mut moved = parent.clone();
        moved.motion.linear[0] = f64::from_bits(1);
        assert!(!frontier.matches(&moved, &child, &state, &cells, 142));
        let mut changed = parent.clone();
        changed.record.as_mut().unwrap().current.motion.angular[0] = f64::from_bits(1);
        assert!(!frontier.matches(&changed, &child, &state, &cells, 142));
        let mut rebound = parent.clone();
        rebound.world = Some(LiveWorldRef::new(parent.source, 73));
        assert!(!frontier.matches(&rebound, &child, &state, &cells, 142));
        let mut geometry = state.clone();
        geometry.cells.remove(&cells[0]);
        assert!(!frontier.matches(&parent, &child, &geometry, &cells, 142));
        let mut revision = state.clone();
        revision.revision += 1;
        assert!(!frontier.matches(&parent, &child, &revision, &cells, 142));
        assert!(!frontier.matches(&parent, &child, &state, &cells, 143));
        let mut reused = child.clone();
        reused.record.as_mut().unwrap().logical.generation += 1;
        assert!(!frontier.matches(&parent, &reused, &state, &cells, 142));
        assert!(!frontier.matches(&parent, &child, &state, &[cells[0]], 142));
    }

    #[test]
    fn split_frontier_is_reset_for_new_request_and_cannot_replace_busy_state() {
        let SplitFixture {
            parent,
            child,
            state,
            revisions,
            cells,
            receipt,
        } = split_frontier_fixture();
        let mut controller = Controller {
            split_frontier: SplitFrontier::completed(
                &parent,
                &child,
                &state,
                revisions,
                &cells,
                142,
                Ok(receipt.clone()),
            )
            .unwrap(),
            ..Controller::default()
        };
        controller
            .queue(Request::Split { slot: 0, child: 1 })
            .unwrap();
        assert!(controller.split_frontier.is_none());
        controller.split_frontier =
            SplitFrontier::completed(&parent, &child, &state, revisions, &cells, 142, Ok(receipt))
                .unwrap();
        assert!(controller.queue(Request::Return { slot: 0 }).is_err());
        assert!(
            controller
                .split_frontier
                .as_ref()
                .unwrap()
                .matches(&parent, &child, &state, &cells, 142)
        );
    }

    fn canonical_fixture() -> CanonicalContext {
        let fixture = split_frontier_fixture();
        let registry = PersistentState {
            owners: vec![fixture.parent.record.as_ref().unwrap().logical],
            plot_generations: BTreeMap::from([(0, 1), (1, 1)]),
        };
        CanonicalContext::new(&[fixture.parent, fixture.child], registry).unwrap()
    }
    fn canonical_records(context: &CanonicalContext) -> Vec<(LevelRef, Option<OwnerState>)> {
        context
            .owners
            .iter()
            .map(|(_, source, record)| (*source, record.clone()))
            .collect()
    }

    #[test]
    fn canonical_guard_refuses_episode_phase_and_raw_changes_with_stable_cached_slots() {
        let context = canonical_fixture();
        let baseline = canonical_records(&context);
        context
            .validate_records(&baseline, Some(&context.registry))
            .unwrap();
        let mut changed_episode = baseline.clone();
        changed_episode[0].1.as_mut().unwrap().episode.return_offset[0] = 1;
        assert!(
            context
                .validate_records(&changed_episode, Some(&context.registry))
                .is_err()
        );
        let mut changed_phase = baseline.clone();
        changed_phase[1].1.as_mut().unwrap().phase = Phase::Retired {
            barrier_epoch: 171,
            generation: 71,
        };
        assert!(
            context
                .validate_records(&changed_phase, Some(&context.registry))
                .is_err()
        );
        let mut changed_raw = baseline.clone();
        changed_raw[0].1.as_mut().unwrap().episode.initial.angular[0] = f64::from_bits(1);
        assert!(
            context
                .validate_records(&changed_raw, Some(&context.registry))
                .is_err()
        );
        // Failed callback verification never rebases the expected records to
        // the changed values. They remain the same cached policy input.
        context
            .validate_records(&baseline, Some(&context.registry))
            .unwrap();
    }

    #[test]
    fn canonical_guard_refuses_changed_or_missing_registry_without_geometry_changes() {
        let context = canonical_fixture();
        let owners = canonical_records(&context);
        let mut changed_highwater = context.registry.clone();
        changed_highwater.plot_generations.insert(1, 2);
        let mut changed_owner = context.registry.clone();
        changed_owner.owners[0].generation += 1;
        let mut removed_owner = context.registry.clone();
        removed_owner.owners.clear();
        for registry in [&changed_highwater, &changed_owner, &removed_owner] {
            assert!(context.validate_records(&owners, Some(registry)).is_err());
        }
        assert!(context.validate_records(&owners, None).is_err());
        context
            .validate_records(&owners, Some(&context.registry))
            .unwrap();
    }

    #[test]
    fn canonical_effect_guard_retains_expected_source_records_across_callbacks() {
        let context = canonical_fixture();
        let owners = canonical_records(&context);
        let mut missing = owners.clone();
        missing[0].1 = None;
        let mut rebound = owners.clone();
        let source = rebound[1].0;
        rebound[1].0 =
            LevelRef::with_generation(source.epoch(), source.dimension(), source.generation() + 1);
        let mut removed = owners.clone();
        removed.pop();
        let mut duplicated = owners.clone();
        duplicated[1] = owners[0].clone();
        for records in [missing, rebound, removed, duplicated] {
            assert!(
                context
                    .validate_records(&records, Some(&context.registry))
                    .is_err()
            );
        }
        let mut callback_record = owners.clone();
        callback_record[1].1.as_mut().unwrap().episode.cells.clear();
        assert!(
            context
                .validate_records(&callback_record, Some(&context.registry))
                .is_err()
        );
        // The same frozen guard is used before/after prepare and after stage,
        // immediately before caller afterimages can overwrite attachments.
        context
            .validate_records(&owners, Some(&context.registry))
            .unwrap();
    }
    #[test]
    fn committed_retained_projected_parent_keeps_exact_binding_and_sample_at_both_gates() {
        let mut parent = split_frontier_fixture().parent;
        let world = parent.world.unwrap();
        let record = parent.record.clone().unwrap();
        let Phase::Projected {
            pose,
            pose_revision,
        } = &record.phase
        else {
            panic!()
        };
        let (pose, pose_revision) = (*pose, *pose_revision);
        parent.host_frame = Some((pose_revision, pose));
        let sample = LiveWorldSample::new(pose, pose, pose_revision);
        let retained = Some((world, sample));
        assert!(
            committed_projection_guard(
                &parent,
                &record,
                Some(world),
                Some(Ok(sample)),
                retained,
                &[]
            )
            .is_ok()
        );
        let mut changed = sample;
        changed.previous.translation[0] = f64::from_bits(1);
        assert!(
            committed_projection_guard(
                &parent,
                &record,
                Some(world),
                Some(Ok(changed)),
                retained,
                &[]
            )
            .is_err()
        );
        let mut changed = sample;
        changed.revision += 1;
        assert!(
            committed_projection_guard(
                &parent,
                &record,
                Some(world),
                Some(Ok(changed)),
                retained,
                &[]
            )
            .is_err()
        );
        assert!(
            committed_projection_guard(
                &parent,
                &record,
                None,
                Some(Err(LiveWorldError::Stale)),
                retained,
                &[]
            )
            .is_err()
        );
        assert!(
            committed_projection_guard(
                &parent,
                &record,
                Some(LiveWorldRef::new(parent.source, 73)),
                Some(Ok(sample)),
                retained,
                &[]
            )
            .is_err()
        );
        assert!(
            committed_projection_guard(
                &parent,
                &record,
                Some(world),
                Some(Ok(sample)),
                retained,
                &[world]
            )
            .is_err()
        );
        let mut wrong = record.clone();
        wrong.logical.generation += 1;
        assert!(projected_template(&parent, &wrong).is_err());
    }

    #[test]
    fn committed_unprojected_child_and_returned_owner_still_require_unbound_and_true_retirement() {
        let mut parent = split_frontier_fixture().parent;
        let world = parent.world.unwrap();
        let mut record = parent.record.clone().unwrap();
        record.phase = Phase::Returned { barrier_epoch: 785 };
        assert!(
            committed_projection_guard(
                &parent,
                &record,
                None,
                Some(Err(LiveWorldError::Stale)),
                None,
                &[world]
            )
            .is_ok()
        );
        assert!(
            committed_projection_guard(
                &parent,
                &record,
                None,
                Some(Err(LiveWorldError::Stale)),
                None,
                &[]
            )
            .is_err()
        );
        assert!(
            committed_projection_guard(
                &parent,
                &record,
                Some(world),
                Some(Err(LiveWorldError::Stale)),
                None,
                &[world]
            )
            .is_err()
        );
        parent.world = None;
        record.phase = Phase::OwnedUnprojected {
            barrier_epoch: 785,
            members: vec![[1035, 128, 1032]],
        };
        assert!(committed_projection_guard(&parent, &record, None, None, None, &[]).is_ok());
        assert!(
            committed_projection_guard(&parent, &record, Some(world), None, None, &[]).is_err()
        );
    }
}
