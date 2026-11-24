//! Actual Sunrise services and the derived native scene controller.
use crate::frames::requested as host_pose;
use crate::{
    frames,
    owner_state::{self, OwnerState, Phase, Profile},
    persist,
};
use mod_api::*;
use sable_core::{Block, Config, Contents, Id, Motion, Payload, Pose, mass::MassRegistry};
use sable_sunrise::{
    registry::{HostGeometry, PhysicsPolicy, PropertyCatalog, StateDefinition},
    simulation::{GeometrySnapshot, PhysicsReplica, TerrainSection},
};
use std::collections::{BTreeMap, BTreeSet};
use std::time::Instant;

/// Opt-in investigation only. This observes native steps and accepted host
/// publications without supplying any simulation input or changing its clock.
fn render_step_trace(event: &str, epoch: u64, motions: Option<&BTreeMap<Id, Motion>>) {
    use std::io::Write;
    use std::sync::{Mutex, OnceLock};
    use std::time::{SystemTime, UNIX_EPOCH};
    struct Trace {
        output: std::io::BufWriter<std::fs::File>,
        start: Instant,
        rows: u64,
        bytes: usize,
        stopped: bool,
    }
    static TRACE: OnceLock<Option<Mutex<Trace>>> = OnceLock::new();
    let Some(trace) = TRACE.get_or_init(|| {
        let path = std::env::var_os("SABLE_PHASE3_RENDER_TRACE")?;
        match std::fs::OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
        {
            Ok(file) => Some(Mutex::new(Trace {
                output: std::io::BufWriter::new(file),
                start: Instant::now(),
                rows: 0,
                bytes: 0,
                stopped: false,
            })),
            Err(problem) => {
                eprintln!("[sable_render_trace] disabled: {problem}");
                None
            }
        }
    }) else {
        return;
    };
    let Ok(mut trace) = trace.lock() else { return };
    if trace.stopped {
        return;
    }
    if trace.rows >= 20_000
        || trace.bytes >= 16 * 1024 * 1024
        || trace.start.elapsed().as_secs() >= 180
    {
        let _ = writeln!(trace.output, "{{\"event\":\"trace_limit\"}}");
        let _ = trace.output.flush();
        trace.stopped = true;
        return;
    }
    let bodies = motions.map_or_else(String::new, |motions| {
        motions
            .iter()
            .map(|(id, motion)| {
                format!(
                    concat!(
                        "{{\"id\":{:?},\"position_bits\":{:?},\"rotation_bits\":{:?},",
                        "\"linear_bits\":{:?},\"angular_bits\":{:?}}}"
                    ),
                    id.0,
                    motion.pose.position.map(f64::to_bits),
                    motion.pose.rotation.map(f64::to_bits),
                    motion.linear.map(f64::to_bits),
                    motion.angular.map(f64::to_bits)
                )
            })
            .collect::<Vec<_>>()
            .join(",")
    });
    let unix_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |v| v.as_nanos());
    let line = format!(
        "{{\"schema\":\"sable.native-render-observation.v1\",\"pid\":{},\"seq\":{},\"elapsed_ns\":{},\"unix_ns\":{unix_ns},\"event\":{event:?},\"native_epoch\":{epoch},\"bodies\":[{bodies}]}}\n",
        std::process::id(),
        trace.rows,
        trace.start.elapsed().as_nanos()
    );
    trace.bytes += line.len();
    trace.rows += 1;
    if trace.output.write_all(line.as_bytes()).is_err()
        || (matches!(event, "publication_accepted" | "status_end") && trace.output.flush().is_err())
    {
        trace.stopped = true;
    }
}

const OPEN_SOURCE: &str = "sable_port:open_demo/body";
const OPEN_ORIGIN: [i32; 3] = [1032, 128, 1032];

/// Conservative clearance for this full-cube demo, including a player standing
/// on a projection. This is an admission check, not the physics collision solver.
fn open_reset_clearance(
    observer: [[f64; 3]; 2],
    placement: LiveWorldPose,
    cells: &[[i32; 3]],
) -> Result<bool, &'static str> {
    if (0..3).any(|axis| {
        !observer[0][axis].is_finite()
            || !observer[1][axis].is_finite()
            || observer[0][axis] >= observer[1][axis]
    }) {
        return Err("reset observer bounds unavailable");
    }
    let placement = frames::canonical_at(placement, placement.pivot)
        .ok_or("reset projection pose unavailable")?;
    let pose = Pose {
        position: std::array::from_fn(|axis| placement.pivot[axis] + placement.translation[axis]),
        pivot: placement.pivot,
        rotation: placement.rotation,
        ..Pose::default()
    };
    for cell in cells {
        if cell.contains(&i32::MAX) {
            return Err("reset cell bounds unavailable");
        }
        let [x, y, z] = *cell;
        // Core bounds are inclusive cells; world_bounds adds the final unit.
        let bounds = pose.world_bounds([x, y, z, x, y, z]);
        if bounds.iter().any(|value| !value.is_finite()) {
            return Err("reset projection bounds unavailable");
        }
        if (0..3).all(|axis| {
            observer[0][axis] <= bounds[axis + 3] + 0.15 && observer[1][axis] >= bounds[axis] - 0.15
        }) {
            return Ok(false);
        }
    }
    Ok(true)
}

#[derive(Default)]
struct OpenStage {
    count: u64,
    total_ns: u128,
    max_ns: u128,
}
impl OpenStage {
    fn record(&mut self, started: Instant) {
        let elapsed = started.elapsed().as_nanos();
        self.count = self.count.saturating_add(1);
        self.total_ns = self.total_ns.saturating_add(elapsed);
        self.max_ns = self.max_ns.max(elapsed);
    }
    fn json(&self) -> String {
        format!(
            "{{\"count\":{},\"total_ns\":{},\"max_ns\":{}}}",
            self.count, self.total_ns, self.max_ns
        )
    }
}
#[derive(Default)]
struct OpenProfile {
    sampling: OpenStage,
    native: OpenStage,
    publication: OpenStage,
    released_at: Option<Instant>,
    finished_elapsed: Option<f64>,
    release_epoch: u64,
    impulse_epoch: Option<u64>,
    ledge_sections: Vec<TerrainSection>,
    ledge_contacts: [u64; 3],
    first_ledge_epoch: [Option<u64>; 3],
    peak_ledge_force: [f64; 3],
}

fn open_cells() -> Vec<[i32; 3]> {
    [
        [0, 0, 0],
        [1, 0, 0],
        [2, 0, 0],
        [3, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
        [1, 0, 1],
    ]
    .into_iter()
    .map(|p| std::array::from_fn(|axis| OPEN_ORIGIN[axis] + p[axis]))
    .collect()
}
fn open_ledges() -> Vec<[i32; 3]> {
    [(4, 106), (1, 99), (2, 96)]
        .into_iter()
        .flat_map(|(x, y)| (-2..=3).map(move |z| [x, y, z]))
        .collect()
}
fn open_ledge_contact(row: &[f64]) -> Option<usize> {
    if row.len() != 15 {
        return None;
    }
    let point = if row[0] == -1. {
        &row[9..12]
    } else if row[1] == -1. {
        &row[12..15]
    } else {
        return None;
    };
    [(4., 106.), (1., 99.), (2., 96.)]
        .into_iter()
        .position(|(x, y)| {
            point[0] >= x - 0.02
                && point[0] <= x + 1.02
                && point[1] >= y - 0.02
                && point[1] <= y + 1.02
                && point[2] >= -2.02
                && point[2] <= 4.02
        })
}
fn open_swept_bounds(snapshot: &GeometrySnapshot) -> Result<[f64; 6], String> {
    let center = snapshot.motion.pose.position;
    let mut radius = 0f64;
    for position in snapshot.cells.keys() {
        for corner in 0..8 {
            let p = snapshot.motion.pose.transform(std::array::from_fn(|axis| {
                f64::from(position[axis]) + if corner & (1 << axis) == 0 { 0. } else { 1. }
            }));
            radius = radius.max(
                (0..3)
                    .map(|axis| (p[axis] - center[axis]).powi(2))
                    .sum::<f64>()
                    .sqrt(),
            );
        }
    }
    // A bounding sphere includes every intermediate orientation. The sweep
    // includes this native epoch's velocity and gravity, plus a cell margin.
    radius += 1.;
    let delta = std::array::from_fn::<_, 3, _>(|axis| {
        snapshot.motion.linear[axis] * 0.05
            + if axis == 1 {
                -0.5 * 9.81 * 0.05 * 0.05
            } else {
                0.
            }
    });
    let bounds = std::array::from_fn(|index| {
        let axis = index % 3;
        center[axis]
            + if index < 3 {
                delta[axis].min(0.) - radius
            } else {
                delta[axis].max(0.) + radius
            }
    });
    if bounds.iter().any(|v| !v.is_finite())
        || (0..3)
            .map(|axis| (bounds[axis + 3] - bounds[axis]).ceil() + 2.)
            .product::<f64>()
            > 8192.
    {
        return Err("open demo swept terrain exceeds its explicit observation budget".into());
    }
    Ok(bounds)
}

#[derive(Clone, Copy)]
enum StartupWait {
    Source,
    SourceTicking,
    Parent,
    Restore,
    MigrationFrontier,
    RetirementReceipt,
    OwnedReceipt,
}
impl StartupWait {
    fn name(self) -> &'static str {
        match self {
            Self::Source => "source_residency",
            Self::SourceTicking => "source_ticking",
            Self::Parent => "parent_residency",
            Self::Restore => "restore_residency",
            Self::MigrationFrontier => "migration_global_frontier",
            Self::RetirementReceipt => "awaiting_empty_retirement_receipt",
            Self::OwnedReceipt => "awaiting_nonempty_owned_receipt",
        }
    }
}

#[derive(Default)]
struct InitializationDiagnostics {
    level_hooks: u64,
    attempts: u64,
    force_calls: u64,
    force_changes: u64,
    in_place: Option<bool>,
    ticking: Option<bool>,
}

struct Owner {
    epoch_policy: persist::EpochPolicy,
    source: LevelRef,
    source_id: String,
    parent: LevelRef,
    world: Option<LiveWorldRef>,
    id: Id,
    plot: u32,
    initialized: bool,
    waiting: Option<StartupWait>,
    diagnostics: InitializationDiagnostics,
    active: bool,
    paused: bool,
    members: Vec<[i32; 3]>,
    motion: Motion,
    mode: u8,
    persisted_pose: Option<(u64, LiveWorldPose)>,
    source_hooks: u64,
    persisted_epoch: Option<u64>,
    persisted_paused: Option<bool>,
    persisted_motion: Option<Motion>,
    persisted_deleted: Option<bool>,
    host_frame: Option<(u64, LiveWorldPose)>,
    recovered_initial: bool,
    footprint: Option<LiveWorldFootprint>,
    bootstrap_forced: BTreeSet<[i32; 2]>,
    bootstrap_released: u64,
    restored_from_save: bool,
    initial_native_readback: Option<NativeReadback>,
    bootstrap_columns: BTreeSet<[i32; 2]>,
    saved_descriptor: Option<LiveWorldSavedProjection>,
    migrated_saved_record: bool,
    episode_members: Vec<[i32; 3]>,
    episode_motion: Motion,
    record: Option<OwnerState>,
    last_integrated: Option<(u64, u64)>,
    last_native_body: Option<u64>,
    native_realizations: u64,
    empty_witness: Option<(u64, u64, u64)>,
}

#[derive(Clone)]
enum InactiveImage {
    Canonical(Box<OwnerState>),
    Legacy(Box<persist::Saved>),
}
struct NativeReadback {
    restored_from_save: bool,
    native_epoch: u64,
    process_publications_before: u64,
    native_pose: [f64; 7],
    native_velocity: [f64; 6],
    logical: Motion,
    host_revision: u64,
    host_pose: LiveWorldPose,
    host_previous: LiveWorldPose,
}
fn bits(values: &[f64]) -> String {
    format!(
        "[{}]",
        values
            .iter()
            .map(|v| format!("\"{:016x}\"", v.to_bits()))
            .collect::<Vec<_>>()
            .join(",")
    )
}
impl NativeReadback {
    fn json(&self) -> String {
        format!(
            r#"{{"restored_from_save":{},"native_epoch":{},"process_publications_before":{},"native_pose_bits":{},"native_velocity_bits":{},"raw_position_bits":{},"raw_pivot_bits":{},"raw_rotation_bits":{},"raw_linear_bits":{},"raw_angular_bits":{},"host_revision":{},"host_translation_bits":{},"host_pivot_bits":{},"host_rotation_bits":{},"host_previous_translation_bits":{},"host_previous_pivot_bits":{},"host_previous_rotation_bits":{},"raw_scale_bits":{}}}"#,
            self.restored_from_save,
            self.native_epoch,
            self.process_publications_before,
            bits(&self.native_pose),
            bits(&self.native_velocity),
            bits(&self.logical.pose.position),
            bits(&self.logical.pose.pivot),
            bits(&self.logical.pose.rotation),
            bits(&self.logical.linear),
            bits(&self.logical.angular),
            self.host_revision,
            bits(&self.host_pose.translation),
            bits(&self.host_pose.pivot),
            bits(&self.host_pose.rotation),
            bits(&self.host_previous.translation),
            bits(&self.host_previous.pivot),
            bits(&self.host_previous.rotation),
            bits(&self.logical.pose.scale)
        )
    }
}
struct CheckpointRecord {
    owner: u8,
    barrier_epoch: u64,
    generation: u64,
    retired_generation: Option<u64>,
    pose_revision: u64,
    source_revision: u64,
    content_revision: u64,
    source: LevelRef,
    retired_members: Option<u32>,
}
/// Owned description for a global lifecycle turn. It contains no live payload.
#[derive(Clone)]
pub(crate) struct Slot {
    pub number: u8,
    pub source: LevelRef,
    pub parent: LevelRef,
    pub source_id: String,
    pub world: Option<LiveWorldRef>,
    pub active: bool,
    pub initialized: bool,
    pub failed: bool,
    pub id: Id,
    pub plot: u32,
    pub motion: Motion,
    pub epoch: u64,
    pub paused: bool,
    pub members: Vec<[i32; 3]>,
    pub record: Option<OwnerState>,
    pub native_bodies: usize,
    pub host_frame: Option<(u64, LiveWorldPose)>,
    pub native_sample: Option<([f64; 7], [f64; 6])>,
    pub empty_witness: Option<(u64, u64, u64)>,
}
pub(crate) struct Scene {
    replica: PhysicsReplica,
    bootstrap_controller: TicketController,
    owners: Vec<Owner>,
    ticks: u64,
    process_native_publications: u64,
    hook_levels: Vec<(LevelRef, u64)>,
    pub(crate) failed: bool,
    initialize_allowed: bool,
    recover_initial_allowed: bool,
    migrate_saved_allowed: bool,
    paused: bool,
    step_once: bool,
    pub(crate) checkpoint_requested: bool,
    pub(crate) world_running: bool,
    epoch_restored: bool,
    checkpoint_requests: u64,
    checkpoint_attempts: u64,
    checkpoint_completed: u64,
    checkpoint_failed: u64,
    checkpoint_epoch: u64,
    checkpoint_paused: bool,
    checkpoint_receipts: Vec<CheckpointRecord>,
    dynamic: bool,
    pending_impulse: bool,
    collision_count: u64,
    last_collision: Vec<f64>,
    reset_requested: bool,
    stale_handle_refused: bool,
    reset_receipts: Vec<(u8, u64, u64)>,
    reset_error: Option<String>,
    final_save_witness_emitted: bool,
    open_profile: Option<OpenProfile>,
}

fn error(value: impl std::fmt::Debug) -> String {
    format!("{value:?}")
}
fn accept_epoch_policy(owner: &mut Owner, policy: persist::EpochPolicy) -> Result<(), String> {
    if !epoch_policy_source_allowed(&owner.source_id, policy) {
        return Err(
            "OpenDemo epoch metadata cannot be adopted by a historical review Source".into(),
        );
    }
    owner.epoch_policy = policy;
    Ok(())
}
fn epoch_policy_source_allowed(source_id: &str, policy: persist::EpochPolicy) -> bool {
    policy == persist::EpochPolicy::Review || source_id == OPEN_SOURCE
}
fn migration_completion(
    result: Result<LiveWorldSavedProjection, LiveWorldError>,
) -> Result<Option<LiveWorldSavedProjection>, String> {
    match result {
        Ok(saved) => Ok(Some(saved)),
        Err(LiveWorldError::Busy | LiveWorldError::NotResident) => Ok(None),
        Err(problem) => Err(error(problem)),
    }
}
/// A missing descriptor or Stale is not retirement evidence: a descriptor has
/// no active binding to describe for a valid saved retirement. Require the
/// actual current Source and its completed common zero-member receipt instead.
fn inactive_retirement_completion(
    source: LevelRef,
    phase: &Phase,
    current_source: Option<LevelRef>,
    binding: Result<Option<LiveWorldRef>, LiveWorldError>,
    descriptor: Result<LiveWorldSavedProjection, LiveWorldError>,
    checkpoint: impl FnOnce() -> Result<LiveWorldSaveReceipt, LiveWorldError>,
) -> Result<Option<LiveWorldLevelReceipt>, String> {
    let (minimum_barrier, expected_generation) = match phase {
        Phase::Returned { barrier_epoch } if *barrier_epoch > 0 => (*barrier_epoch, None),
        Phase::Retired {
            barrier_epoch,
            generation,
        } if *barrier_epoch > 0 && *generation > 0 => (*barrier_epoch, Some(*generation)),
        Phase::Retiring { generation, .. } if *generation > 0 => (1, Some(*generation)),
        _ => return Err("inactive startup has no valid canonical retirement phase".into()),
    };
    if current_source != Some(source) || binding.map_err(error)?.is_some() {
        return Err("inactive startup has no matching current unbound Source".into());
    }
    match descriptor {
        Err(LiveWorldError::NotSaved | LiveWorldError::Stale) => {}
        Ok(_) => return Err("inactive startup still has a saved active binding".into()),
        Err(problem) => return Err(error(problem)),
    }
    let receipt = match checkpoint() {
        Ok(receipt) => receipt,
        Err(LiveWorldError::Busy) => return Ok(None),
        Err(problem) => return Err(error(problem)),
    };
    let mut matching = receipt.levels.iter().filter(|level| level.level == source);
    let level = matching
        .next()
        .ok_or_else(|| "inactive startup receipt does not cover the actual Source".to_string())?;
    let generation = level
        .retired_generation
        .filter(|generation| *generation > 0)
        .ok_or_else(|| "inactive startup receipt has no retired generation".to_string())?;
    if receipt.barrier_epoch < minimum_barrier
        || matching.next().is_some()
        || level.binding.is_some()
        || level.retired_members != Some(0)
        || expected_generation.is_some_and(|expected| expected != generation)
    {
        return Err(
            "inactive startup lacks a matching complete zero-member retirement receipt".into(),
        );
    }
    Ok(Some(level.clone()))
}

fn owned_unprojected_completion(
    source: LevelRef,
    phase: &Phase,
    current: Option<LevelRef>,
    binding: Result<Option<LiveWorldRef>, LiveWorldError>,
    descriptor: Result<LiveWorldSavedProjection, LiveWorldError>,
    checkpoint: impl FnOnce() -> Result<LiveWorldSaveReceipt, LiveWorldError>,
) -> Result<Option<LiveWorldLevelReceipt>, String> {
    let Phase::OwnedUnprojected {
        barrier_epoch,
        members,
    } = phase
    else {
        return Err("nonempty receipt requires canonical OwnedUnprojected".into());
    };
    if *barrier_epoch == 0
        || members.is_empty()
        || members.len() > 512
        || members.iter().copied().collect::<BTreeSet<_>>().len() != members.len()
        || current != Some(source)
        || binding.map_err(error)?.is_some()
    {
        return Err("nonempty owned receipt has changed identity/members/binding".into());
    }
    // Stale alone proves no ownership: the actual nonempty canonical image,
    // current Source/binding and successful common receipt are also required.
    // This path never infers empty retirement from a missing saved descriptor.
    if !matches!(descriptor, Err(LiveWorldError::Stale)) {
        return Err("nonempty owned checkpoint is not the exact stale-descriptor domain".into());
    }
    let receipt = match checkpoint() {
        Ok(receipt) => receipt,
        Err(LiveWorldError::Busy) => return Ok(None),
        Err(problem) => return Err(error(problem)),
    };
    let mut rows = receipt.levels.iter().filter(|row| row.level == source);
    let row = rows
        .next()
        .ok_or_else(|| "owned receipt omitted Source".to_string())?;
    if rows.next().is_some()
        || receipt.barrier_epoch < *barrier_epoch
        || row.binding.is_some()
        || row.retired_generation == Some(0)
        || row.retired_members.is_some()
    {
        return Err("nonempty owned receipt lacks exact current unbound coverage".into());
    }
    // On fresh reopen, saved generationSome/membersNone does not hydrate the
    // host retirement table. Its current receipt is then None/None. Neither
    // that row nor a retained generationSome/membersNone proves an empty Source.
    Ok(Some(row.clone()))
}

fn member_key_equal(left: &str, right: &str) -> bool {
    let key = |value: &str| -> Option<String> {
        if let Some(properties) = value
            .strip_prefix("minecraft:furnace[")
            .and_then(|v| v.strip_suffix(']'))
        {
            let lit = properties
                .split(',')
                .filter(|p| p.starts_with("lit="))
                .collect::<Vec<_>>();
            if lit.len() != 1 || !matches!(lit[0], "lit=true" | "lit=false") {
                return None;
            }
            Some(format!(
                "minecraft:furnace[{}]",
                properties
                    .split(',')
                    .filter(|p| !p.starts_with("lit="))
                    .collect::<Vec<_>>()
                    .join(",")
            ))
        } else {
            None
        }
    };
    match (key(left), key(right)) {
        (Some(a), Some(b)) => a == b,
        _ => left == right,
    }
}

// Explicit, one-case caller-metadata repair for the already acknowledged785
// split. It is never used by normal activation and stores no live payload.
pub(crate) struct SplitParentRepair {
    sources: [LevelRef; 4],
    before: Vec<OwnerState>,
    registry: sable_core::host_lifecycle::PersistentState,
    parent_binding: LiveWorldRef,
    sample: LiveWorldSample,
    after: OwnerState,
    minimum_barrier: u64,
    pub(crate) staged: bool,
}
const REPAIR_PARENT: Id = Id([
    0x33, 0x52, 0xef, 0x83, 0xa5, 0x8e, 0x49, 0x27, 0xbe, 0x38, 0x90, 0xfb, 0x89, 0x67, 0x72, 0x76,
]);
const REPAIR_CHILD: Id = Id([
    0xcd, 0x90, 0x6b, 0x25, 0x4f, 0x86, 0x4d, 0xe2, 0xa3, 0xd2, 0xd2, 0x86, 0x68, 0x80, 0x28, 0x23,
]);
const REPAIR_PARENT_MOTION: [u64; 16] = [
    0x4030e87f2b5b5400,
    0x403882159c536650,
    0x3fffde2dfdde7c00,
    0x40906397b89a11da,
    0x40601042b36f5324,
    0x409027f78b821d20,
    0,
    0,
    0,
    0x3ff0000000000000,
    0x3fc99999a0000000,
    0,
    0,
    0xbed6211e20000000,
    0x3f14db9d00000000,
    0xbe3b796000000000,
];
const REPAIR_CHILD_MOTION: [u64; 16] = [
    0x4030e87f2b5b5400,
    0x403882159c536650,
    0x3fffde2dfdde7c00,
    0x40906397b89a11da,
    0x40601042b36f5324,
    0x409027f78b821d20,
    0,
    0,
    0,
    0x3ff0000000000000,
    0x3fc99999426beb9f,
    0xbe44a7069d061246,
    0xbf15efaae1daba60,
    0xbed6211e20000000,
    0x3f14db9d00000000,
    0xbe3b796000000000,
];

fn repair_motion_bits(motion: &Motion) -> Vec<u64> {
    motion
        .pose
        .position
        .into_iter()
        .chain(motion.pose.pivot)
        .chain(motion.pose.rotation)
        .chain(motion.linear)
        .chain(motion.angular)
        .map(f64::to_bits)
        .collect()
}
fn repair785_records(
    records: &[OwnerState],
    registry: &sable_core::host_lifecycle::PersistentState,
) -> Result<(), String> {
    if records.len() != 4 || registry.owners.len() != 4 {
        return Err("repair785 requires the exact four-owner registry".into());
    }
    for record in records {
        if registry
            .owners
            .iter()
            .filter(|identity| **identity == record.logical)
            .count()
            != 1
            || registry.plot_generations.get(&record.logical.plot)
                != Some(&record.logical.generation)
        {
            return Err("repair785 canonical registry/identity differs".into());
        }
    }
    for (index, count, pinned, id, plot, generation) in [
        (0, 8, REPAIR_PARENT_MOTION, REPAIR_PARENT, 0, 4),
        (1, 126, REPAIR_CHILD_MOTION, REPAIR_CHILD, 128, 2),
    ] {
        let record = &records[index];
        let Phase::OwnedUnprojected {
            barrier_epoch,
            members,
        } = &record.phase
        else {
            return Err("repair785 requires the original committed unprojected afterimage".into());
        };
        if *barrier_epoch != 785
            || members.len() != count
            || record.episode.cells.len() != count
            || members.iter().copied().collect::<BTreeSet<_>>()
                != record.episode.cells.keys().copied().collect()
            || record.logical.id != id
            || record.logical.plot != plot
            || record.logical.generation != generation
            || record.episode.profile != Profile::Split
            || record.episode.return_offset != [-1032, -104, -1032]
            || !record.current.paused
            || record.current.native_epoch != 1
            || record.current.motion.pose.scale != [1.; 3]
            || repair_motion_bits(&record.current.motion).as_slice() != pinned.as_slice()
            || !same_motion_bits(&record.current.motion, &record.episode.initial)
        {
            return Err(
                "repair785 committed identity/member/initial/current motion differs".into(),
            );
        }
    }
    if records[0].logical.parent.is_some()
        || records[1].logical.parent != Some(records[0].logical.id)
        || records[0]
            .episode
            .cells
            .keys()
            .any(|position| records[1].episode.cells.contains_key(position))
        || records[2].episode.profile != Profile::Pitch
        || records[3].episode.profile != Profile::Dynamic
        || !matches!(records[2].phase, Phase::Projected { .. })
        || !matches!(records[3].phase, Phase::Projected { .. })
        || records[2].logical.id != Id([3; 16])
        || records[3].logical.id != Id([4; 16])
        || records[2].current.native_epoch != 1
        || records[3].current.native_epoch != 0
        || !records[2].current.paused
        || !records[3].current.paused
    {
        return Err("repair785 lineage/other-owner afterimage differs".into());
    }
    let parent = (1032..=1033)
        .flat_map(|x| (1032..=1035).map(move |z| [x, 128, z]))
        .collect::<BTreeSet<_>>();
    let mut child = (1035..=1065)
        .flat_map(|x| (1032..=1035).map(move |z| [x, 128, z]))
        .filter(|p| *p != [1049, 128, 1034])
        .collect::<BTreeSet<_>>();
    child.extend([[1035, 129, 1033], [1037, 129, 1033], [1040, 129, 1033]]);
    for (index, positions) in [(0, parent), (1, child)] {
        if records[index]
            .episode
            .cells
            .keys()
            .copied()
            .collect::<BTreeSet<_>>()
            != positions
        {
            return Err("repair785 exact committed cell positions differ".into());
        }
        for (position, key) in &records[index].episode.cells {
            let expected = match *position {
                [1035, 129, 1033] => "minecraft:chest[facing=north,type=single,waterlogged=false]",
                [1037, 129, 1033] => "minecraft:furnace[facing=north,lit=true]",
                [1040, 129, 1033] => "minecraft:stone_slab[type=bottom,waterlogged=false]",
                _ => "minecraft:blackstone",
            };
            if !member_key_equal(key, expected) {
                return Err("repair785 exact committed palette differs".into());
            }
        }
    }
    Ok(())
}

impl SplitParentRepair {
    fn records(
        host: &mut dyn ServerHost,
        sources: &[LevelRef; 4],
    ) -> Result<Vec<OwnerState>, String> {
        sources
            .iter()
            .map(|source| {
                owner_state::OWNER
                    .get_existing(&mut source.attachments(host), Holder::Level)
                    .map_err(error)?
                    .ok_or_else(|| "repair785 canonical owner missing".to_string())
            })
            .collect()
    }
    fn registry(
        host: &mut dyn ServerHost,
        source: LevelRef,
    ) -> Result<sable_core::host_lifecycle::PersistentState, String> {
        owner_state::REGISTRY
            .get_existing(&mut source.attachments(host), Holder::Level)
            .map_err(error)?
            .ok_or_else(|| "repair785 registry missing".into())
    }
    fn geometry(
        host: &dyn ServerHost,
        source: LevelRef,
        record: &OwnerState,
    ) -> Result<(), String> {
        let positions = record.episode.cells.keys().copied().collect::<Vec<_>>();
        let low: [i32; 3] =
            std::array::from_fn(|axis| positions.iter().map(|p| p[axis]).min().unwrap());
        let high: [i32; 3] =
            std::array::from_fn(|axis| positions.iter().map(|p| p[axis]).max().unwrap());
        let volume = (0..3).try_fold(1i64, |n, axis| {
            n.checked_mul(i64::from(high[axis]) - i64::from(low[axis]) + 1)
        });
        if volume.is_none_or(|n| !(1..=512).contains(&n)) {
            return Err("repair785 envelope exceeds bounded readset".into());
        }
        let mut envelope = Vec::new();
        for x in low[0]..=high[0] {
            for y in low[1]..=high[1] {
                for z in low[2]..=high[2] {
                    envelope.push([x, y, z]);
                }
            }
        }
        let mut catalog = PropertyCatalog::new(1).map_err(error)?;
        let observed =
            crate::lifecycle_backend::read_snapshot(host, source, &envelope, &mut catalog)?.state;
        if observed.cells.len() != record.episode.cells.len()
            || observed.cells.iter().any(|(position, key)| {
                record
                    .episode
                    .cells
                    .get(position)
                    .is_none_or(|expected| !member_key_equal(key, expected))
            })
        {
            return Err("repair785 actual Source cells/states/shapes differ".into());
        }
        for (&position, key) in &record.episode.cells {
            let expected =
                key.starts_with("minecraft:chest[") || key.starts_with("minecraft:furnace[");
            let entity = host.block_entity(source, position);
            if entity.is_some() != expected
                || entity.is_some_and(|entity| entity.data(host).is_none())
            {
                return Err("repair785 actual live BE binding/readability differs".into());
            }
        }
        Ok(())
    }
    pub(crate) fn prepare(host: &mut dyn ServerHost) -> Result<Option<Self>, String> {
        let ids = [
            "sable_port:review/translate",
            "sable_port:review/yaw",
            "sable_port:review/pitch",
            "sable_port:review/dynamic",
        ];
        let mut levels = Vec::new();
        for id in ids {
            levels.push(
                host.level(id)
                    .ok_or_else(|| "repair785 actual Source absent".to_string())?,
            );
        }
        let sources: [LevelRef; 4] = levels
            .try_into()
            .map_err(|_| "repair785 Source count".to_string())?;
        let before = Self::records(host, &sources)?;
        let registry = Self::registry(host, sources[0])?;
        repair785_records(&before, &registry)?;
        for i in 0..2 {
            let columns = before[i]
                .episode
                .cells
                .keys()
                .map(|p| [p[0].div_euclid(16), p[2].div_euclid(16)])
                .collect::<BTreeSet<_>>();
            if !columns.iter().all(|position| {
                sources[i]
                    .chunk_ref(host, *position)
                    .is_some_and(|chunk| chunk.loaded(host))
                    && sources[i]
                        .should_tick_blocks_at(host, [position[0] * 16, 0, position[1] * 16])
                        == Some(true)
            }) {
                return Ok(None);
            }
            Self::geometry(host, sources[i], &before[i])?;
        }
        let checkpoint = match sources[1].checkpoint(host) {
            Ok(receipt) => receipt,
            Err(LiveWorldError::Busy) => return Ok(None),
            Err(problem) => return Err(error(problem)),
        };
        let Some(_) = owned_unprojected_completion(
            sources[1],
            &before[1].phase,
            host.level(ids[1]),
            sources[1].live_world(host),
            sources[1].saved_projection(host),
            || Ok(checkpoint.clone()),
        )?
        else {
            return Ok(None);
        };
        // The public fresh common receipt need not retain historical185.
        // Its exact nonempty785 canonical image and current unbound coverage,
        // checked above, are the authority; no private saved JSON is read here.
        let descriptor = sources[0].saved_projection(host).map_err(error)?;
        if descriptor.destination != "minecraft:overworld"
            || descriptor.member_cells != 8
            || descriptor.barrier_epoch < 785
            || descriptor.recovery != LiveWorldRecovery::Ready
        {
            return Err("repair785 parent saved descriptor differs".into());
        }
        let parent_binding = match sources[0].live_world(host).map_err(error)? {
            Some(binding) => binding,
            None => match sources[0].restore_projection(host) {
                Ok(binding) => binding,
                Err(LiveWorldError::NotResident | LiveWorldError::Busy) => return Ok(None),
                Err(problem) => return Err(error(problem)),
            },
        };
        let sample = parent_binding.sample(host).map_err(error)?;
        let expected = LiveWorldPose {
            translation: [0xc0901ff5bbeca48a, 0xc059ffffffc9ccb4, 0xc09020000002a581]
                .map(f64::from_bits),
            pivot: [0x40906397b89a11da, 0x40601042b36f5324, 0x409027f78b821d20].map(f64::from_bits),
            rotation: [0., 0., 0., 1.],
        };
        if !restored_sample_matches(&sample, expected, 1)
            || !frames::equivalent(&before[0].current.motion, sample.current)
            || parent_binding
                .cells(host)
                .map_err(error)?
                .iter()
                .map(|cell| cell.pos)
                .collect::<BTreeSet<_>>()
                != before[0].episode.cells.keys().copied().collect()
        {
            return Err("repair785 parent actual accepted frame/members differ".into());
        }
        let mut after = before[0].clone();
        after.phase = Phase::Projected {
            pose_revision: sample.revision,
            pose: sample.current,
        };
        let repair = Self {
            sources,
            before,
            registry,
            parent_binding,
            sample,
            after,
            minimum_barrier: checkpoint.barrier_epoch.max(descriptor.barrier_epoch),
            staged: false,
        };
        repair.verify(host, false)?;
        Ok(Some(repair))
    }
    fn verify(&self, host: &mut dyn ServerHost, after: bool) -> Result<(), String> {
        for (source, id) in self.sources.iter().zip([
            "sable_port:review/translate",
            "sable_port:review/yaw",
            "sable_port:review/pitch",
            "sable_port:review/dynamic",
        ]) {
            if host.level(id) != Some(*source) {
                return Err("repair785 actual Source identity changed".into());
            }
        }
        let records = Self::records(host, &self.sources)?;
        for (index, record) in records.iter().enumerate() {
            let expected = if after && index == 0 {
                &self.after
            } else {
                &self.before[index]
            };
            if !owner_state::same_record(record, expected) {
                return Err("repair785 canonical afterimage changed during callbacks".into());
            }
        }
        if Self::registry(host, self.sources[0])? != self.registry
            || self.sources[0].live_world(host).map_err(error)? != Some(self.parent_binding)
            || self.sources[1].live_world(host).map_err(error)?.is_some()
        {
            return Err("repair785 registry/binding changed".into());
        }
        let actual = self.parent_binding.sample(host).map_err(error)?;
        if actual.revision != self.sample.revision
            || !same_host_bits(actual.current, self.sample.current)
            || !same_host_bits(actual.previous, self.sample.previous)
            || self
                .parent_binding
                .cells(host)
                .map_err(error)?
                .iter()
                .map(|cell| cell.pos)
                .collect::<BTreeSet<_>>()
                != self.before[0].episode.cells.keys().copied().collect()
        {
            return Err("repair785 actual accepted frame changed".into());
        }
        for i in 0..2 {
            Self::geometry(host, self.sources[i], &self.before[i])?;
        }
        Ok(())
    }
    pub(crate) fn advance(&mut self, host: &mut dyn ServerHost) -> Result<Option<u64>, String> {
        if !self.staged {
            self.verify(host, false)?;
            // Attempted is latched BEFORE entering a fallible host close. An
            // error never authorizes rewriting or claims that no write occurred.
            self.staged = true;
            host.transaction(|host, _| -> Result<(), String> {
                self.verify(host, false)?;
                owner_state::OWNER
                    .set(
                        &mut self.sources[0].attachments(host),
                        Holder::Level,
                        self.after.clone(),
                    )
                    .map_err(error)
            })?;
        }
        self.verify(host, true)?;
        let receipt = match self.sources[0].checkpoint(host) {
            Ok(receipt) => receipt,
            Err(LiveWorldError::Busy) => return Ok(None),
            Err(problem) => return Err(error(problem)),
        };
        let mut rows = receipt
            .levels
            .iter()
            .filter(|row| row.level == self.sources[0]);
        let row = rows
            .next()
            .ok_or_else(|| "repair785 common ACK omitted parent Source".to_string())?;
        if rows.next().is_some()
            || receipt.barrier_epoch <= self.minimum_barrier
            || row.binding != Some(self.parent_binding)
        {
            return Err("repair785 has no fresh exact common parent ACK".into());
        }
        self.verify(host, true)?;
        Ok(Some(receipt.barrier_epoch))
    }
}
fn state(host: &dyn ServerHost, name: &str) -> Result<BlockState, String> {
    let block = host
        .block_key(name)
        .ok_or_else(|| format!("unknown actual block {name}"))?;
    BlockState::of(&host.block_states(), block, &[]).map_err(error)
}
fn item(host: &dyn ServerHost, name: &str, count: u32) -> Result<ItemStack, String> {
    let id = Identifier::parse(name).map_err(error)?;
    ItemStack::new(
        host.item_key(&id)
            .ok_or_else(|| format!("unknown actual item {name}"))?,
        count,
    )
    .ok_or_else(|| "invalid item count".into())
}
fn cells() -> Vec<[i32; 3]> {
    let mut cells = Vec::new();
    for x in 0..34 {
        for z in 0..4 {
            if !(x == 17 && z == 2) {
                cells.push([x, 0, z]);
            }
        }
    }
    cells.extend([[3, 1, 1], [5, 1, 1], [8, 1, 1]]);
    cells
}
fn dynamic_cells() -> Vec<[i32; 3]> {
    vec![
        [0, 0, 0],
        [1, 0, 0],
        [2, 0, 0],
        [0, 1, 0],
        [3, 0, 0],
        [0, 0, 1],
        [0, 0, 2],
    ]
}

fn initial_motion(mode: u8) -> Motion {
    if mode == 3 {
        return Motion {
            pose: Pose {
                position: [12., 32., 24.],
                ..Pose::default()
            },
            ..Motion::default()
        };
    }
    let mut motion = Motion {
        pose: Pose {
            position: [
                17. + f64::from(mode) * 2.,
                if mode == 2 { 48. } else { 24. },
                2. + f64::from(mode) * 48.,
            ],
            pivot: [17., 0., 2.],
            ..Pose::default()
        },
        ..Motion::default()
    };
    match mode {
        0 => motion.linear = [0.2, 0., 0.],
        1 => motion.angular = [0., 0.05, 0.],
        _ => motion.angular = [0., 0., 0.2],
    }
    motion
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum NativeImpulse {
    Spin,
    Drift,
    FastDrift,
}
impl NativeImpulse {
    pub(crate) fn parse(command: &str) -> Option<Self> {
        match command {
            "spin" => Some(Self::Spin),
            "drift" => Some(Self::Drift),
            "fast-drift" => Some(Self::FastDrift),
            _ => None,
        }
    }
    fn name(self) -> &'static str {
        match self {
            Self::Spin => "spin",
            Self::Drift => "drift",
            Self::FastDrift => "fast-drift",
        }
    }
    fn linear_speed(self) -> Option<f64> {
        match self {
            Self::Spin => None,
            Self::Drift => Some(0.2),
            Self::FastDrift => Some(2.0),
        }
    }
}

/// The original demo contract, independent of a pool slot's former arbitrary
/// capture/split episode. It describes required real blocks, never a source
/// from which blocks, inventories or BE payloads can be created.
#[derive(Clone, Copy, Debug)]
pub(crate) enum FixtureBlock {
    Blackstone,
    Chest,
    Furnace,
    BottomSlab,
}
impl FixtureBlock {
    fn name(self) -> &'static str {
        match self {
            Self::Blackstone => "minecraft:blackstone",
            Self::Chest => "minecraft:chest",
            Self::Furnace => "minecraft:furnace",
            Self::BottomSlab => "minecraft:stone_slab",
        }
    }
    pub(crate) fn has_entity(self) -> bool {
        matches!(self, Self::Chest | Self::Furnace)
    }
    fn state_allowed(self, key: &str) -> bool {
        if matches!(self, Self::Blackstone) {
            return key == self.name();
        }
        let Some(properties) = key
            .strip_prefix(self.name())
            .and_then(|s| s.strip_prefix('['))
            .and_then(|s| s.strip_suffix(']'))
        else {
            return false;
        };
        let mut values = BTreeMap::new();
        for pair in properties.split(',') {
            let Some((name, value)) = pair.split_once('=') else {
                return false;
            };
            if values.insert(name, value).is_some() {
                return false;
            }
        }
        let facing = |value: Option<&&str>| {
            matches!(value.copied(), Some("north" | "south" | "east" | "west"))
        };
        match self {
            Self::Blackstone => false,
            Self::Chest => {
                values.len() == 3
                    && facing(values.get("facing"))
                    && values.get("type") == Some(&"single")
                    && values.get("waterlogged") == Some(&"false")
            }
            Self::Furnace => {
                values.len() == 2
                    && facing(values.get("facing"))
                    && matches!(values.get("lit").copied(), Some("true" | "false"))
            }
            Self::BottomSlab => {
                values.len() == 2
                    && values.get("type") == Some(&"bottom")
                    && values.get("waterlogged") == Some(&"false")
            }
        }
    }
    fn properties(self) -> sable_core::mass::Properties {
        let (mass, center, full, boxes) = match self {
            Self::Blackstone => (2., [0.5; 3], true, vec![[0., 0., 0., 1., 1., 1.]]),
            Self::Furnace => (1., [0.5; 3], true, vec![[0., 0., 0., 1., 1., 1.]]),
            Self::Chest => (
                0.5,
                [0.5, 0.4375, 0.5],
                false,
                vec![[0.0625, 0., 0.0625, 0.9375, 0.875, 0.9375]],
            ),
            Self::BottomSlab => (1., [0.5, 0.25, 0.5], false, vec![[0., 0., 0., 1., 0.5, 1.]]),
        };
        sable_core::mass::Properties {
            solid: true,
            full,
            mass,
            center,
            inertia: None,
            boxes,
        }
    }
}
pub(crate) struct OriginalFixture {
    pub(crate) origin: [i32; 3],
    pub(crate) profile: Profile,
    pub(crate) members: BTreeMap<[i32; 3], FixtureBlock>,
    pub(crate) envelope: Vec<[i32; 3]>,
    linear: [f64; 3],
    angular: [f64; 3],
}
impl OriginalFixture {
    pub(crate) fn for_slot(number: u8) -> Result<Self, String> {
        let profile = match number {
            0 => Profile::Translate,
            1 => Profile::Yaw,
            2 => Profile::Pitch,
            3 => Profile::Dynamic,
            _ => return Err("unknown original demo slot".into()),
        };
        let origin = if number == 3 {
            [12, 32, 24]
        } else {
            [
                i32::from(number) * 2,
                if number == 2 { 48 } else { 24 },
                i32::from(number) * 48,
            ]
        };
        let members = (if number == 3 {
            dynamic_cells()
        } else {
            cells()
        })
        .into_iter()
        .map(|local| {
            let role = if number == 3 {
                FixtureBlock::Blackstone
            } else {
                match local {
                    [3, 1, 1] => FixtureBlock::Chest,
                    [5, 1, 1] => FixtureBlock::Furnace,
                    [8, 1, 1] => FixtureBlock::BottomSlab,
                    _ => FixtureBlock::Blackstone,
                }
            };
            (std::array::from_fn(|axis| local[axis] + origin[axis]), role)
        })
        .collect::<BTreeMap<_, _>>();
        let min: [i32; 3] =
            std::array::from_fn(|axis| members.keys().map(|p| p[axis]).min().unwrap());
        let max: [i32; 3] =
            std::array::from_fn(|axis| members.keys().map(|p| p[axis]).max().unwrap());
        let mut envelope = Vec::new();
        for x in min[0]..=max[0] {
            for y in min[1]..=max[1] {
                for z in min[2]..=max[2] {
                    envelope.push([x, y, z]);
                }
            }
        }
        let inputs = initial_motion(number);
        Ok(Self {
            origin,
            profile,
            members,
            envelope,
            linear: inputs.linear,
            angular: inputs.angular,
        })
    }
    /// The actual resolver has already validated complete host states and
    /// collision geometry. This additional fixture check admits normal furnace
    /// lit/facing and dry single-chest facing, while requiring original physical
    /// roles, bottom-slab shape and mass at every original coordinate.
    pub(crate) fn validate(
        &self,
        actual: &sable_core::host_lifecycle::StateSnapshot,
        connected: &[[i32; 3]],
        catalog: &PropertyCatalog,
    ) -> Result<(), String> {
        let expected = self.members.keys().copied().collect::<BTreeSet<_>>();
        let connected_set = connected.iter().copied().collect::<BTreeSet<_>>();
        if connected_set.len() != connected.len()
            || connected_set != expected
            || actual.cells.keys().copied().collect::<BTreeSet<_>>() != expected
        {
            return Err(
                "rearm refuses missing/extra original cells or a changed connected component"
                    .into(),
            );
        }
        for (position, role) in &self.members {
            let key = &actual.cells[position];
            if !role.state_allowed(key) {
                return Err("rearm refuses replaced original palette roles".into());
            }
            let properties = catalog
                .get(catalog.epoch(), key)
                .map_err(error)?
                .properties();
            if *properties != role.properties() {
                return Err("rearm refuses changed original collision/mass properties".into());
            }
        }
        let masses = catalog
            .mass_registry(catalog.epoch(), actual.cells.values().map(String::as_str))
            .map_err(error)?;
        let geometry = Contents {
            blocks: actual
                .cells
                .iter()
                .map(|(position, key)| {
                    (
                        *position,
                        Block {
                            state: key.clone(),
                            block_entity: None,
                            extension: Payload::default(),
                        },
                    )
                })
                .collect(),
            ..Contents::default()
        };
        let actual_mass = masses.compute(&geometry).map_err(error)?;
        let expected_mass = self
            .members
            .values()
            .map(|role| role.properties().mass)
            .sum::<f64>();
        if actual_mass.mass != expected_mass {
            return Err("rearm original aggregate mass mismatch".into());
        }
        Ok(())
    }
    pub(crate) fn validate_live_entities(
        &self,
        host: &dyn ServerHost,
        parent: LevelRef,
    ) -> Result<(), String> {
        for (position, role) in &self.members {
            let entity = host.block_entity(parent, *position);
            if entity.is_some() != role.has_entity() {
                return Err("rearm original block-entity binding is missing or mismatched".into());
            }
            if entity.is_some_and(|entity| entity.data(host).is_none()) {
                return Err("rearm real block entity is not currently readable".into());
            }
        }
        // Contents are intentionally neither counted against seeded values nor
        // copied into caller metadata. The host token freezes/moves them.
        Ok(())
    }
    pub(crate) fn motion(&self, planned: &Motion) -> Motion {
        // Keep the accepted core capture's actual coordinate mapping and COM.
        // Only explicit native input velocities change; no literal old pose is
        // transplanted into relocated local coordinates.
        let mut motion = planned.clone();
        motion.linear = self.linear;
        motion.angular = self.angular;
        motion
    }
}

fn review_limits(owner_mode: u8) -> LiveWorldLimits {
    // Explicit bounded review reservations, not Sable's general capture limits.
    // The gravity fixture's native contact response can exceed the controlled
    // input's five-degree step. These admit publication; they never alter the
    // solver output. Existing saved bindings retain their stored limits until
    // an explicit episode reset retires and reprojects them.
    let (translation, rotation_degrees) = if owner_mode == 3 { (4., 90.) } else { (1., 5.) };
    LiveWorldLimits {
        min: [0, -64, 0],
        max: [2047, 319, 2047],
        cells: 512,
        chunks: 256,
        boxes: 1024,
        contacts: 65536,
        candidates: 65536,
        translation,
        rotation_degrees,
        motion_steps: 32,
        publication: LiveWorldPublication::ExternallyResolved,
    }
}

#[derive(PartialEq)]
struct FixtureObservation {
    position: [i32; 3],
    state: BlockState,
    collision: Vec<[f64; 6]>,
    entity: Option<(BlockEntityRef, Compound)>,
}

pub(crate) fn same_host_bits(left: LiveWorldPose, right: LiveWorldPose) -> bool {
    left.translation
        .into_iter()
        .chain(left.pivot)
        .chain(left.rotation)
        .map(f64::to_bits)
        .eq(right
            .translation
            .into_iter()
            .chain(right.pivot)
            .chain(right.rotation)
            .map(f64::to_bits))
}
// Restore preserves accepted CURRENT component bits/revision. Its fresh
// presentation/contact frame is settled, so a historical publish delta must
// not become a new carry/interpolation step in the new process.
fn restored_sample_matches(
    sample: &LiveWorldSample,
    current: LiveWorldPose,
    revision: u64,
) -> bool {
    sample.revision == revision
        && same_host_bits(sample.current, current)
        && same_host_bits(sample.previous, sample.current)
}

fn same_motion_bits(left: &Motion, right: &Motion) -> bool {
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

fn parent_resident(
    host: &dyn ServerHost,
    parent: LevelRef,
    bounds: [f64; 6],
) -> Result<bool, String> {
    // Match the ordinary collision cursor's padding. Absence is never air.
    let mut columns = [[0i32; 2]; 2];
    for (index, axis) in [0usize, 2].into_iter().enumerate() {
        let lower = (bounds[axis] - 1e-7).floor();
        let upper = (bounds[axis + 3] + 1e-7).floor();
        if !lower.is_finite()
            || !upper.is_finite()
            || lower < f64::from(i32::MIN) + 1.
            || upper > f64::from(i32::MAX) - 1.
        {
            return Err("terrain scan coordinates exceed the checked parent cursor".into());
        }
        columns[index] = [
            ((lower as i32) - 1).div_euclid(16),
            ((upper as i32) + 1).div_euclid(16),
        ];
    }
    let count =
        i64::from(columns[0][1] - columns[0][0] + 1) * i64::from(columns[1][1] - columns[1][0] + 1);
    if !(1..=256).contains(&count) {
        return Err("bounded parent terrain column budget".into());
    }
    for x in columns[0][0]..=columns[0][1] {
        for z in columns[1][0]..=columns[1][1] {
            if parent
                .chunk_ref(host, [x, z])
                .is_none_or(|chunk| !chunk.loaded(host))
            {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

fn startup_parent_resident(
    host: &dyn ServerHost,
    owner: &Owner,
    pose: &Pose,
) -> Result<bool, String> {
    let mut bounds = [f64::INFINITY; 6];
    bounds[3..].fill(f64::NEG_INFINITY);
    for position in &owner.members {
        for corner in 0..8 {
            let local = std::array::from_fn(|axis| {
                f64::from(position[axis]) + if corner & (1 << axis) == 0 { 0. } else { 1. }
            });
            let point = pose.transform(local);
            for axis in 0..3 {
                bounds[axis] = bounds[axis].min(point[axis] - 1.);
                bounds[axis + 3] = bounds[axis + 3].max(point[axis] + 1.);
            }
        }
    }
    parent_resident(host, owner.parent, bounds)
}

pub(crate) fn resolve(
    host: &dyn ServerHost,
    actual: BlockState,
    boxes: Vec<[f64; 6]>,
    catalog: &mut PropertyCatalog,
) -> Result<String, String> {
    let metadata = host.block_states();
    let name = metadata.name(actual.block()).map_err(error)?.to_string();
    let definition = metadata
        .definition(actual.block())
        .ok_or_else(|| "missing actual state definition".to_string())?;
    let mut values = Vec::new();
    let mut definitions = Vec::new();
    for property in definition {
        let value = actual
            .property(&metadata, &property.name)
            .map_err(error)?
            .ok_or_else(|| "incomplete actual property".to_string())?
            .to_string();
        values.push((property.name.clone(), value));
        definitions.push((property.name, property.values));
    }
    let canonical = StateDefinition::new(name.clone(), definitions)
        .map_err(error)?
        .resolve(values.clone())
        .map_err(error)?;
    let key = canonical.key().to_string();
    let property = |name: &str| {
        values
            .iter()
            .find(|(key, _)| key == name)
            .map(|(_, value)| value.as_str())
    };
    if property("waterlogged") == Some("true") {
        return Err("waterlogged states are outside the dry review palette".into());
    }
    let expected = match name.as_str() {
        "minecraft:blackstone" | "minecraft:bedrock" | "minecraft:furnace" => {
            vec![[0., 0., 0., 1., 1., 1.]]
        }
        "minecraft:chest" if property("type") == Some("single") => {
            vec![[0.0625, 0., 0.0625, 0.9375, 0.875, 0.9375]]
        }
        "minecraft:stone_slab" => match property("type") {
            Some("bottom") => vec![[0., 0., 0., 1., 0.5, 1.]],
            Some("top") => vec![[0., 0.5, 0., 1., 1., 1.]],
            Some("double") => vec![[0., 0., 0., 1., 1., 1.]],
            _ => return Err("unsupported actual slab type".into()),
        },
        _ => return Err(format!("unsupported actual palette/state {name}")),
    };
    if boxes != expected {
        return Err(format!("unsupported actual palette collision shape {name}"));
    }
    let mass = match name.as_str() {
        "minecraft:blackstone" => 2.,
        "minecraft:bedrock" => 1000.,
        "minecraft:chest" => 0.5,
        "minecraft:furnace" => 1.,
        "minecraft:stone_slab" => {
            if values.iter().any(|(k, v)| k == "type" && v == "double") {
                2.
            } else {
                1.
            }
        }
        _ => return Err(format!("unsupported actual palette {name}")),
    };
    let full = boxes == vec![[0., 0., 0., 1., 1., 1.]];
    let solid = !boxes.is_empty();
    let mut center = [0.5; 3];
    if solid && !full {
        for axis in 0..3 {
            let min = boxes
                .iter()
                .map(|b| b[axis])
                .fold(f64::INFINITY, f64::min)
                .max(0.);
            let max = boxes
                .iter()
                .map(|b| b[axis + 3])
                .fold(f64::NEG_INFINITY, f64::max)
                .min(1.);
            center[axis] = (min + max) * 0.5;
        }
    }
    if let Ok(existing) = catalog.get(catalog.epoch(), &key) {
        if existing.properties().boxes != boxes {
            return Err("state-invariant profile encountered contextual geometry".into());
        }
        return Ok(key);
    }
    catalog.register(canonical,HostGeometry {state_invariant:true,solid,full,boxes},PhysicsPolicy {rule:"explicit pinned blackstone/bedrock/light chest/default furnace/normal slab profile".into(),mass,center,inertia:None}).map_err(error)?;
    Ok(key)
}

fn open_ground_key(
    host: &dyn ServerHost,
    actual: BlockState,
    boxes: Vec<[f64; 6]>,
    catalog: &mut PropertyCatalog,
) -> Result<String, String> {
    let metadata = host.block_states();
    let name = metadata.name(actual.block()).map_err(error)?.to_string();
    if matches!(name.as_str(), "minecraft:blackstone" | "minecraft:bedrock") {
        return resolve(host, actual, boxes, catalog);
    }
    if !matches!(
        name.as_str(),
        "minecraft:grass_block"
            | "minecraft:dirt"
            | "minecraft:coarse_dirt"
            | "minecraft:rooted_dirt"
            | "minecraft:podzol"
            | "minecraft:mycelium"
            | "minecraft:stone"
            | "minecraft:granite"
            | "minecraft:diorite"
            | "minecraft:andesite"
            | "minecraft:deepslate"
            | "minecraft:tuff"
            | "minecraft:calcite"
            | "minecraft:sand"
            | "minecraft:red_sand"
            | "minecraft:sandstone"
            | "minecraft:red_sandstone"
            | "minecraft:gravel"
            | "minecraft:clay"
            | "minecraft:snow_block"
            | "minecraft:ice"
            | "minecraft:packed_ice"
            | "minecraft:blue_ice"
            | "minecraft:terracotta"
            | "minecraft:coal_ore"
            | "minecraft:iron_ore"
            | "minecraft:copper_ore"
            | "minecraft:gold_ore"
            | "minecraft:redstone_ore"
            | "minecraft:lapis_ore"
            | "minecraft:diamond_ore"
            | "minecraft:emerald_ore"
    ) || boxes != vec![[0., 0., 0., 1., 1., 1.]]
    {
        return Err(format!(
            "open demo refuses unsupported actual ground state/shape {name}"
        ));
    }
    let definitions = metadata
        .definition(actual.block())
        .ok_or_else(|| "missing ordinary ground state definition".to_string())?;
    let mut values = Vec::new();
    let mut properties = Vec::new();
    for property in definitions {
        let value = actual
            .property(&metadata, &property.name)
            .map_err(error)?
            .ok_or_else(|| "incomplete ordinary ground state".to_string())?
            .to_string();
        values.push((property.name.clone(), value));
        properties.push((property.name, property.values));
    }
    let canonical = StateDefinition::new(name, properties)
        .map_err(error)?
        .resolve(values)
        .map_err(error)?;
    let key = canonical.key().to_string();
    if let Ok(existing) = catalog.get(catalog.epoch(), &key) {
        if existing.properties().boxes != boxes {
            return Err("ordinary ground state has contextual collision geometry".into());
        }
        return Ok(key);
    }
    catalog
        .register(
            canonical,
            HostGeometry {
                state_invariant: true,
                solid: true,
                full: true,
                boxes,
            },
            PhysicsPolicy {
                rule: "open demo actual ordinary full-cube static terrain; unit reference density"
                    .into(),
                mass: 1.,
                center: [0.5; 3],
                inertia: None,
            },
        )
        .map_err(error)?;
    Ok(key)
}

// The ordinary furnace ticker changes lit while retaining the same admitted
// shape/mass. Reset preserves that live state; all other episode state changes
// still refuse before retiring any binding.
fn reset_state_matches(expected: &str, actual: &str) -> bool {
    if expected == actual {
        return true;
    }
    fn furnace(value: &str) -> Option<BTreeMap<&str, &str>> {
        let (name, properties) = value.split_once('[')?;
        if name != "minecraft:furnace" {
            return None;
        }
        let properties = properties.strip_suffix(']')?;
        let mut values = BTreeMap::new();
        for pair in properties.split(',') {
            let (key, value) = pair.split_once('=')?;
            if values.insert(key, value).is_some() {
                return None;
            }
        }
        if !matches!(values.remove("lit"), Some("true" | "false")) {
            return None;
        }
        Some(values)
    }
    matches!((furnace(expected),furnace(actual)),(Some(a),Some(b)) if a==b)
}

impl Scene {
    fn confirm_owned_unprojected(
        host: &mut dyn ServerHost,
        owner: &Owner,
        record: &OwnerState,
    ) -> Result<Option<LiveWorldLevelReceipt>, String> {
        let Phase::OwnedUnprojected { members, .. } = &record.phase else {
            return Err("owned startup record changed phase".into());
        };
        let selected = members.iter().copied().collect::<BTreeSet<_>>();
        if selected.is_empty()
            || selected.len() != members.len()
            || selected != record.episode.cells.keys().copied().collect()
            || !record.current.paused
        {
            return Err("owned startup canonical member/current image differs".into());
        }
        let low: [i32; 3] =
            std::array::from_fn(|axis| selected.iter().map(|p| p[axis]).min().unwrap());
        let high: [i32; 3] =
            std::array::from_fn(|axis| selected.iter().map(|p| p[axis]).max().unwrap());
        let volume = (0..3).try_fold(1i64, |n, axis| {
            n.checked_mul(i64::from(high[axis]) - i64::from(low[axis]) + 1)
        });
        if volume.is_none_or(|size| !(1..=512).contains(&size)) {
            return Err("owned startup readback exceeds its bounded review envelope".into());
        }
        let mut positions = Vec::new();
        for x in low[0]..=high[0] {
            for y in low[1]..=high[1] {
                for z in low[2]..=high[2] {
                    positions.push([x, y, z]);
                }
            }
        }
        let read = |host: &dyn ServerHost| -> Result<_, String> {
            let mut catalog = PropertyCatalog::new(1).map_err(error)?;
            let observed = crate::lifecycle_backend::read_snapshot(
                host,
                owner.source,
                &positions,
                &mut catalog,
            )?;
            if observed.state.cells.len() != record.episode.cells.len()
                || observed.state.cells.iter().any(|(p, key)| {
                    record
                        .episode
                        .cells
                        .get(p)
                        .is_none_or(|expected| !member_key_equal(key, expected))
                })
            {
                return Err("owned startup actual cell/state/shape image differs".into());
            }
            Ok(observed)
        };
        read(host)?;
        let Some(receipt) = owned_unprojected_completion(
            owner.source,
            &record.phase,
            host.level(&owner.source_id),
            owner.source.live_world(host),
            owner.source.saved_projection(host),
            || owner.source.checkpoint(host),
        )?
        else {
            return Ok(None);
        };
        let observed = read(host)?;
        let actual = owner_state::OWNER
            .get_existing(&mut owner.source.attachments(host), Holder::Level)
            .map_err(error)?
            .ok_or_else(|| "owned startup canonical record vanished".to_string())?;
        if host.level(&owner.source_id) != Some(owner.source)
            || owner.source.live_world(host).map_err(error)?.is_some()
            || !owner_state::same_record(record, &actual)
            || observed.revisions.block_revision != receipt.block_revision
            || observed.revisions.content_revision != receipt.content_revision
        {
            return Err("owned startup Source/record/readback changed during checkpoint".into());
        }
        Ok(Some(receipt))
    }
    pub(crate) fn start(
        host: &mut dyn ServerHost,
        bootstrap_controller: TicketController,
    ) -> Result<Self, String> {
        let path = std::env::var_os("SABLE_PHYSICS_LIBRARY")
            .ok_or_else(|| "explicit SABLE_PHYSICS_LIBRARY is required".to_string())?;
        // This proof runs from an actual SDK hook when the real mod is loaded.
        // A successful Cargo compile alone is never reported as this observation.
        {
            let mut bridge = sable_bridge::Bridge::open(&path).map_err(error)?;
            let scene = bridge.scene([0.; 3], 0.).map_err(error)?;
            let body = bridge
                .create_box(scene, 1, 2., [0.5; 3], [0., 0., 0., 0., 0., 0., 1.])
                .map_err(error)?;
            bridge
                .impulse(body, [2., 0., 0.], [0.; 3], true)
                .map_err(error)?;
            let velocity = bridge.velocity(body).map_err(error)?;
            if velocity[0] != 1. {
                return Err("C ABI impulse/mass invariant failed".into());
            }
            bridge.close().map_err(error)?;
            eprintln!(
                "[sable_phase3] {{\"event\":\"sdk_hook_bridge_loaded\",\"impulse_mass_velocity\":1}}"
            );
        }
        let replica = PhysicsReplica::open(path, Config::default(), [0.; 3]).map_err(error)?;
        let parent = host
            .level("minecraft:overworld")
            .ok_or_else(|| "ordinary parent world unavailable".to_string())?;
        let initialize_allowed = std::env::var_os("SABLE_PHASE3_INITIALIZE").as_deref()
            == Some(std::ffi::OsStr::new("1"));
        let recover_initial_allowed = std::env::var_os("SABLE_PHASE3_RECOVER_INITIAL_SETUP")
            .as_deref()
            == Some(std::ffi::OsStr::new("1"));
        let migrate_saved_allowed = std::env::var_os("SABLE_PHASE3_MIGRATE_EXPERIMENTAL_SAVE")
            .as_deref()
            == Some(std::ffi::OsStr::new("1"));
        if migrate_saved_allowed && (initialize_allowed || recover_initial_allowed) {
            return Err(
                "choose initial setup/recovery or explicit saved-record migration separately"
                    .into(),
            );
        }
        if initialize_allowed && recover_initial_allowed {
            return Err(
                "choose explicit first setup or interrupted-setup recovery, not both".into(),
            );
        }
        let mut owners = Vec::new();
        for (mode, name) in ["translate", "yaw", "pitch"].into_iter().enumerate() {
            let source_id = format!("sable_port:review/{name}");
            let source = host
                .runtime_levels()
                .open(
                    &source_id,
                    RuntimeLevel::new("minecraft:overworld", RuntimeGenerator::Void, 4242),
                )
                .map_err(error)?;
            owners.push(Owner {
                epoch_policy: persist::EpochPolicy::Review,
                source,
                source_id,
                parent,
                world: None,
                id: Id([mode as u8 + 1; 16]),
                plot: mode as u32,
                initialized: false,
                waiting: Some(StartupWait::Source),
                diagnostics: InitializationDiagnostics::default(),
                active: true,
                paused: initialize_allowed || recover_initial_allowed,
                members: cells(),
                motion: initial_motion(mode as u8),
                mode: mode as u8,
                persisted_pose: None,
                source_hooks: 0,
                persisted_epoch: None,
                persisted_paused: None,
                persisted_motion: None,
                persisted_deleted: None,
                host_frame: None,
                recovered_initial: false,
                footprint: None,
                bootstrap_forced: BTreeSet::new(),
                bootstrap_released: 0,
                restored_from_save: false,
                initial_native_readback: None,
                bootstrap_columns: BTreeSet::new(),
                saved_descriptor: None,
                migrated_saved_record: false,
                episode_members: cells(),
                episode_motion: initial_motion(mode as u8),
                record: None,
                last_integrated: None,
                last_native_body: None,
                native_realizations: 0,
                empty_witness: None,
            });
        }
        eprintln!(
            "[sable_phase3] {{\"event\":\"experimental_profile\",\"gravity\":0,\"substeps\":2,\"terrain_reaction_parity\":false}}"
        );
        Ok(Self::new(
            replica,
            bootstrap_controller,
            owners,
            initialize_allowed,
            recover_initial_allowed,
            migrate_saved_allowed,
            false,
        ))
    }
    fn new(
        replica: PhysicsReplica,
        bootstrap_controller: TicketController,
        owners: Vec<Owner>,
        initialize_allowed: bool,
        recover_initial_allowed: bool,
        migrate_saved_allowed: bool,
        dynamic: bool,
    ) -> Self {
        Self {
            replica,
            bootstrap_controller,
            owners,
            ticks: 0,
            process_native_publications: 0,
            hook_levels: Vec::new(),
            failed: false,
            paused: initialize_allowed || recover_initial_allowed,
            initialize_allowed,
            recover_initial_allowed,
            migrate_saved_allowed,
            step_once: false,
            checkpoint_requested: false,
            world_running: false,
            epoch_restored: false,
            checkpoint_requests: 0,
            checkpoint_attempts: 0,
            checkpoint_completed: 0,
            checkpoint_failed: 0,
            checkpoint_epoch: 0,
            checkpoint_paused: false,
            checkpoint_receipts: Vec::new(),
            dynamic,
            pending_impulse: false,
            collision_count: 0,
            last_collision: Vec::new(),
            reset_requested: false,
            stale_handle_refused: false,
            reset_receipts: Vec::new(),
            reset_error: None,
            final_save_witness_emitted: false,
            open_profile: None,
        }
    }
    pub(crate) fn open_demo(
        host: &mut dyn ServerHost,
        bootstrap_controller: TicketController,
    ) -> Result<Self, String> {
        let path = std::env::var_os("SABLE_PHYSICS_LIBRARY")
            .ok_or_else(|| "explicit SABLE_PHYSICS_LIBRARY is required".to_string())?;
        let replica =
            PhysicsReplica::open(path, Config::default(), [0., -9.81, 0.]).map_err(error)?;
        let parent = host
            .level("minecraft:overworld")
            .ok_or_else(|| "ordinary parent world unavailable".to_string())?;
        let initialize_allowed = std::env::var_os("SABLE_OPEN_DEMO_INITIALIZE").as_deref()
            == Some(std::ffi::OsStr::new("1"));
        let source = host
            .runtime_levels()
            .open(
                OPEN_SOURCE,
                RuntimeLevel::new("minecraft:overworld", RuntimeGenerator::Void, 4242),
            )
            .map_err(error)?;
        let motion = Motion {
            pose: Pose {
                position: [0., 114., 0.],
                pivot: OPEN_ORIGIN.map(f64::from),
                ..Pose::default()
            },
            ..Motion::default()
        };
        let owner = Owner {
            epoch_policy: persist::EpochPolicy::OpenDemo,
            source,
            source_id: OPEN_SOURCE.into(),
            parent,
            world: None,
            id: Id([5; 16]),
            plot: 4,
            initialized: false,
            waiting: Some(StartupWait::Source),
            diagnostics: InitializationDiagnostics::default(),
            active: true,
            paused: true,
            members: open_cells(),
            motion: motion.clone(),
            mode: 3,
            persisted_pose: None,
            source_hooks: 0,
            persisted_epoch: None,
            persisted_paused: None,
            persisted_motion: None,
            persisted_deleted: None,
            host_frame: None,
            recovered_initial: false,
            footprint: None,
            bootstrap_forced: BTreeSet::new(),
            bootstrap_released: 0,
            restored_from_save: false,
            initial_native_readback: None,
            bootstrap_columns: BTreeSet::new(),
            saved_descriptor: None,
            migrated_saved_record: false,
            episode_members: open_cells(),
            episode_motion: motion,
            record: None,
            last_integrated: None,
            last_native_body: None,
            native_realizations: 0,
            empty_witness: None,
        };
        let mut scene = Self::new(
            replica,
            bootstrap_controller,
            vec![owner],
            initialize_allowed,
            false,
            false,
            true,
        );
        scene.paused = initialize_allowed;
        scene.open_profile = Some(OpenProfile::default());
        Ok(scene)
    }
    pub(crate) fn dynamic(host: &mut dyn ServerHost, zero: &Self) -> Result<Self, String> {
        let replica = zero
            .replica
            .new_scene(Config::default(), [0., -9.81, 0.])
            .map_err(error)?;
        let parent = zero.owners[0].parent;
        let source = host
            .runtime_levels()
            .open(
                "sable_port:review/dynamic",
                RuntimeLevel::new("minecraft:overworld", RuntimeGenerator::Void, 4242),
            )
            .map_err(error)?;
        let owner = Owner {
            epoch_policy: persist::EpochPolicy::Review,
            source,
            source_id: "sable_port:review/dynamic".into(),
            parent,
            world: None,
            id: Id([4; 16]),
            plot: 3,
            initialized: false,
            waiting: Some(StartupWait::Source),
            diagnostics: InitializationDiagnostics::default(),
            active: true,
            paused: zero.initialize_allowed || zero.recover_initial_allowed,
            members: dynamic_cells(),
            motion: initial_motion(3),
            mode: 3,
            persisted_pose: None,
            source_hooks: 0,
            persisted_epoch: None,
            persisted_paused: None,
            persisted_motion: None,
            persisted_deleted: None,
            host_frame: None,
            recovered_initial: false,
            footprint: None,
            bootstrap_forced: BTreeSet::new(),
            bootstrap_released: 0,
            restored_from_save: false,
            initial_native_readback: None,
            bootstrap_columns: BTreeSet::new(),
            saved_descriptor: None,
            migrated_saved_record: false,
            episode_members: dynamic_cells(),
            episode_motion: initial_motion(3),
            record: None,
            last_integrated: None,
            last_native_body: None,
            native_realizations: 0,
            empty_witness: None,
        };
        eprintln!(
            "[sable_phase3] {{\"event\":\"dynamic_profile\",\"gravity_m_s2\":[0,-9.81,0],\"dt_s\":0.05,\"substeps\":2,\"cells\":7,\"mass\":14,\"logical_origin\":[12,32,24],\"impulse\":[0,0,2],\"torque_impulse\":[2,-2,0]}}"
        );
        Ok(Self::new(
            replica,
            zero.bootstrap_controller,
            vec![owner],
            zero.initialize_allowed,
            zero.recover_initial_allowed,
            zero.migrate_saved_allowed,
            true,
        ))
    }
    fn confirm_inactive_retirement(
        host: &mut dyn ServerHost,
        owner: &Owner,
        record: &OwnerState,
    ) -> Result<Option<LiveWorldLevelReceipt>, String> {
        let source = owner.source;
        let current = host.level(&owner.source_id);
        let binding = source.live_world(host);
        let descriptor = source.saved_projection(host);
        let Some(level) = inactive_retirement_completion(
            source,
            &record.phase,
            current,
            binding,
            descriptor,
            || source.checkpoint(host),
        )?
        else {
            return Ok(None);
        };
        // Checkpoint callbacks must not replace the caller phase, raw physical
        // bits or actual Source while the retirement witness is being obtained.
        let revisions = source.revisions(host).map_err(error)?;
        let actual = owner_state::OWNER
            .get_existing(&mut source.attachments(host), Holder::Level)
            .map_err(error)?
            .ok_or_else(|| {
                "inactive canonical Source record disappeared during checkpoint".to_string()
            })?;
        if host.level(&owner.source_id) != Some(source)
            || source.live_world(host).map_err(error)?.is_some()
            || revisions.block_revision != level.block_revision
            || revisions.content_revision != level.content_revision
            || !owner_state::same_record(record, &actual)
        {
            return Err("inactive Source or canonical metadata changed during checkpoint".into());
        }
        Ok(Some(level))
    }
    pub(crate) fn activate_sources(&mut self, host: &mut dyn ServerHost) -> Result<(), String> {
        if self.failed {
            return Ok(());
        }
        for owner in &mut self.owners {
            if !owner.active || owner.initialized {
                continue;
            }
            // Empty runtime levels are not selected for a source tick until
            // they have tickets. Request only these owned columns globally;
            // seeding and attachment writes still wait for the source turn.
            let Some(current) = host.level(&owner.source_id) else {
                owner.waiting = Some(StartupWait::Source);
                continue; // The queued runtime open has not reached its boundary.
            };
            if current != owner.source {
                return Err("owned source handle changed before activation".into());
            }
            let record = owner_state::OWNER
                .get_existing(&mut owner.source.attachments(host), Holder::Level)
                .map_err(error)?;
            if let Some(record) = record {
                accept_epoch_policy(owner, record.episode.profile.epoch_policy())?;
                owner.id = record.logical.id;
                owner.plot = record.logical.plot;
                owner.motion = record.current.motion.clone();
                owner.paused = record.current.paused;
                owner.episode_motion = record.episode.initial.clone();
                owner.episode_members = record.episode.cells.keys().copied().collect();
                match &record.phase {
                    Phase::OwnedUnprojected { members, .. } => owner.members = members.clone(),
                    Phase::Returned { .. } | Phase::Retired { .. } | Phase::Retiring { .. } => {
                        if let Phase::Retiring { members, .. } = &record.phase {
                            owner.members = members.clone();
                        }
                        let Some(retired) =
                            Self::confirm_inactive_retirement(host, owner, &record)?
                        else {
                            owner.waiting = Some(StartupWait::RetirementReceipt);
                            continue;
                        };
                        owner.empty_witness = Some((
                            retired.retired_generation.unwrap(),
                            retired.block_revision,
                            retired.content_revision,
                        ));
                        owner.active = false;
                        owner.initialized = true;
                        owner.waiting = None;
                        owner.persisted_epoch = Some(record.current.native_epoch);
                        owner.persisted_paused = Some(record.current.paused);
                        owner.persisted_deleted = Some(true);
                        owner.persisted_motion = Some(record.current.motion.clone());
                        owner.record = Some(record);
                        Self::release_owned_tickets(host, owner, self.bootstrap_controller)?;
                        continue;
                    }
                    Phase::Projected { .. } => {}
                }
                owner.record = Some(record);
            }
            match owner.source.saved_projection(host) {
                Ok(saved) => {
                    if host.level(&saved.destination) != Some(owner.parent)
                        || saved.member_cells == 0
                        || saved.member_cells > review_limits(owner.mode).cells
                        || saved.retained_columns == 0
                        || saved.retained_columns > review_limits(owner.mode).chunks
                        || saved.source_chunks.is_empty()
                    {
                        return Err(
                            "saved projection is outside the bounded review parent/footprint"
                                .into(),
                        );
                    }
                    if saved.recovery == LiveWorldRecovery::ExperimentalMigrationRequired
                        && !self.migrate_saved_allowed
                    {
                        return Err("experimental saved projection needs explicit byte-preserving migration; no restore/reseed fallback".into());
                    }
                    let columns = saved.source_chunks.iter().copied().collect::<BTreeSet<_>>();
                    if columns.len() != saved.source_chunks.len() {
                        return Err("saved descriptor repeated a source column".into());
                    }
                    owner.bootstrap_columns = columns;
                    owner.saved_descriptor = Some(saved);
                }
                Err(LiveWorldError::NotSaved) => {
                    owner.saved_descriptor = None;
                    if owner.record.is_none()
                        && !(self.initialize_allowed || self.recover_initial_allowed)
                    {
                        let saved = persist::MOTION
                            .get_existing(&mut owner.source.attachments(host), Holder::Level)
                            .map_err(error)?;
                        if saved.as_ref().is_some_and(|saved| !saved.deleted) {
                            return Err("physical metadata has no saved binding; normal resume refuses adoption/reseed".into());
                        }
                    }
                    // This is only the explicitly known first-setup/legacy
                    // tombstone domain. Genuine saved bindings always use the
                    // host's authoritative validated descriptor above.
                    owner.bootstrap_columns = owner
                        .members
                        .iter()
                        .map(|p| [p[0].div_euclid(16), p[2].div_euclid(16)])
                        .collect();
                    let footprint =
                        Self::checked_footprint(host, owner.source, &owner.members, owner.mode)?;
                    if owner.footprint != Some(footprint) {
                        eprintln!(
                            "[sable_phase3] initial fixture footprint owner={} actual={footprint:?} limits={:?}",
                            owner.mode,
                            review_limits(owner.mode)
                        );
                        owner.footprint = Some(footprint);
                    }
                }
                Err(LiveWorldError::Stale)
                    if owner.record.as_ref().is_some_and(|record| {
                        matches!(record.phase, Phase::OwnedUnprojected { .. })
                    }) =>
                {
                    // This is nonempty ownership after a real transfer into a
                    // previously retired slot, not inactive/empty retirement.
                    owner.bootstrap_columns = owner
                        .members
                        .iter()
                        .map(|p| [p[0].div_euclid(16), p[2].div_euclid(16)])
                        .collect();
                    Self::hold_bootstrap(host, owner, self.bootstrap_controller)?;
                    let resident = owner.bootstrap_columns.iter().all(|position| {
                        owner
                            .source
                            .chunk_ref(host, *position)
                            .is_some_and(|chunk| chunk.loaded(host))
                            && owner.source.should_tick_blocks_at(
                                host,
                                [position[0] * 16, 0, position[1] * 16],
                            ) == Some(true)
                    });
                    if !resident {
                        owner.waiting = Some(StartupWait::Source);
                        continue;
                    }
                    let record = owner.record.clone().unwrap();
                    if Self::confirm_owned_unprojected(host, owner, &record)?.is_none() {
                        owner.waiting = Some(StartupWait::OwnedReceipt);
                        continue;
                    }
                    owner.saved_descriptor = None;
                }
                Err(problem) => return Err(error(problem)),
            }
            Self::hold_bootstrap(host, owner, self.bootstrap_controller)?;
            if owner.saved_descriptor.as_ref().is_some_and(|saved| {
                saved.recovery == LiveWorldRecovery::ExperimentalMigrationRequired
            }) {
                // activate_sources runs at ServerTickStart, the actual common
                // frontier. Source LevelTickPre must never perform this save.
                let mut ready = true;
                for &[x, z] in &owner.bootstrap_columns {
                    let chunk = owner
                        .source
                        .chunk_ref(host, [x, z])
                        .ok_or_else(|| "migration source chunk handle unavailable".to_string())?;
                    let position = [
                        x.checked_mul(16)
                            .ok_or_else(|| "migration source coordinate overflow".to_string())?,
                        0,
                        z.checked_mul(16)
                            .ok_or_else(|| "migration source coordinate overflow".to_string())?,
                    ];
                    ready &= chunk.loaded(host)
                        && owner.source.should_tick_blocks_at(host, position) == Some(true);
                }
                if !ready {
                    owner.waiting = Some(StartupWait::MigrationFrontier);
                    continue;
                }
                let Some(migrated) = migration_completion(owner.source.migrate_projection(host))?
                else {
                    owner.waiting = Some(StartupWait::MigrationFrontier);
                    continue;
                };
                if migrated.recovery != LiveWorldRecovery::Ready
                    || owner.saved_descriptor.as_ref().is_none_or(|old| {
                        old.source_chunks != migrated.source_chunks
                            || old.destination != migrated.destination
                    })
                {
                    return Err(
                        "host migration changed authoritative source requirements unexpectedly"
                            .into(),
                    );
                }
                owner.saved_descriptor = Some(migrated);
                owner.migrated_saved_record = true;
                owner.waiting = None;
            }
        }
        Ok(())
    }
    fn hold_bootstrap(
        host: &mut dyn ServerHost,
        owner: &mut Owner,
        controller: TicketController,
    ) -> Result<(), String> {
        for position in owner.bootstrap_columns.iter().copied() {
            let chunk = owner
                .source
                .chunk_ref(host, position)
                .ok_or_else(|| "current owned source chunk handle unavailable".to_string())?;
            owner.diagnostics.force_calls = owner.diagnostics.force_calls.saturating_add(1);
            let ticket_owner = TicketOwner::Entity(u128::from_le_bytes(owner.id.0));
            let changed = chunk.set_mod_forced(host, controller, ticket_owner, true, false);
            owner.diagnostics.force_changes = owner
                .diagnostics
                .force_changes
                .saturating_add(u64::from(changed));
            let held = owner
                .source
                .mod_forced_chunks(host, controller)
                .ok_or_else(|| "owned source bootstrap ticket query unavailable".to_string())?;
            if !held.contains(&(ticket_owner, position, false)) {
                return Err("owned source bootstrap ticket was not established".into());
            }
            // The registered controller + UUID is our scoped ownership,
            // including a ticket persisted by an interrupted earlier setup.
            owner.bootstrap_forced.insert(position);
        }
        Ok(())
    }
    fn checked_footprint(
        host: &dyn ServerHost,
        source: LevelRef,
        members: &[[i32; 3]],
        owner_mode: u8,
    ) -> Result<LiveWorldFootprint, String> {
        let limits = review_limits(owner_mode);
        if members.is_empty()
            || members.len() > limits.cells as usize
            || members.iter().any(|position| {
                (0..3).any(|axis| {
                    position[axis] < limits.min[axis] || position[axis] > limits.max[axis]
                })
            })
        {
            return Err("fixture membership exceeds explicit review reservations".into());
        }
        // Use the public host calculation, including its actual ordinary
        // streaming ring, rather than reproducing a private halo constant.
        let footprint = source.projection_footprint(host, members).map_err(error)?;
        if footprint.cells as usize != members.len()
            || footprint.retained_columns == 0
            || footprint.retained_columns > limits.chunks
            || footprint.source_chunks > footprint.retained_columns
        {
            return Err("actual source footprint exceeds explicit review reservations".into());
        }
        Ok(footprint)
    }
    fn release_bootstrap(
        host: &mut dyn ServerHost,
        owner: &mut Owner,
        controller: TicketController,
    ) -> Result<(), String> {
        let world = owner
            .world
            .ok_or_else(|| "bootstrap release requires an active projection".to_string())?;
        world.sample(host).map_err(error)?;
        Self::release_owned_tickets(host, owner, controller)
    }
    fn release_owned_tickets(
        host: &mut dyn ServerHost,
        owner: &mut Owner,
        controller: TicketController,
    ) -> Result<(), String> {
        // Successful project/restore now establishes its own ephemeral source
        // retention. An inactive source or shutdown also needs no bootstrap.
        // Release only our controller/UUID; never change the world's persistent
        // /forceload or another owner.
        let ticket_owner = TicketOwner::Entity(u128::from_le_bytes(owner.id.0));
        let positions = owner.bootstrap_forced.iter().copied().collect::<Vec<_>>();
        for position in positions {
            let chunk = owner
                .source
                .chunk_ref(host, position)
                .ok_or_else(|| "bootstrap release source chunk unavailable".to_string())?;
            let _ = chunk.set_mod_forced(host, controller, ticket_owner, false, false);
            let held = owner
                .source
                .mod_forced_chunks(host, controller)
                .ok_or_else(|| "owned bootstrap release query unavailable".to_string())?;
            if held.contains(&(ticket_owner, position, false)) {
                return Err("owned bootstrap ticket release was refused".into());
            }
            owner.bootstrap_released = owner.bootstrap_released.saturating_add(1);
            owner.bootstrap_forced.remove(&position);
        }
        Ok(())
    }
    /// A stale binding only requests an actual current-Source barrier query.
    /// Neither Stale nor a cached old cell selection proves empty membership.
    pub(crate) fn settle_auto_retirements(
        &mut self,
        host: &mut dyn ServerHost,
    ) -> Result<(), String> {
        if self.failed {
            return Ok(());
        }
        for owner in self
            .owners
            .iter_mut()
            .filter(|owner| !owner.active && owner.initialized)
        {
            let Some(OwnerState {
                phase: Phase::Retiring { generation, .. },
                ..
            }) = owner.record.as_ref()
            else {
                continue;
            };
            let generation = *generation;
            let current = owner.source.revisions(host).map_err(error)?;
            if owner.empty_witness
                == Some((generation, current.block_revision, current.content_revision))
            {
                continue;
            }
            let receipt = match owner.source.checkpoint(host) {
                Ok(receipt) => receipt,
                Err(LiveWorldError::Busy) => {
                    owner.waiting = Some(StartupWait::RetirementReceipt);
                    continue;
                }
                Err(problem) => return Err(error(problem)),
            };
            let matching = receipt
                .levels
                .iter()
                .filter(|level| level.level == owner.source)
                .collect::<Vec<_>>();
            if receipt.barrier_epoch == 0
                || matching.len() != 1
                || matching[0].binding.is_some()
                || matching[0].retired_generation != Some(generation)
                || matching[0].retired_members != Some(0)
            {
                return Err("retiring record lacks an actual complete zero-member receipt".into());
            }
            let level = matching[0];
            owner.empty_witness = Some((generation, level.block_revision, level.content_revision));
            owner.waiting = None;
        }
        for owner in self
            .owners
            .iter_mut()
            .filter(|owner| owner.active && owner.initialized)
        {
            let Some(old) = owner.world else { continue };
            match old.sample(host) {
                Ok(_) => continue,
                Err(LiveWorldError::Stale) => {}
                Err(problem) => return Err(error(problem)),
            }
            if host.level(&owner.source_id) != Some(owner.source)
                || owner.source.live_world(host).map_err(error)?.is_some()
            {
                return Err("stale binding has no matching current unbound owned Source".into());
            }
            let receipt = match owner.source.checkpoint(host) {
                Ok(receipt) => receipt,
                Err(LiveWorldError::Busy) => {
                    owner.waiting = Some(StartupWait::RetirementReceipt);
                    continue;
                }
                Err(problem) => return Err(error(problem)),
            };
            let matching = receipt
                .levels
                .iter()
                .filter(|level| level.level == owner.source)
                .collect::<Vec<_>>();
            if receipt.barrier_epoch == 0
                || matching.len() != 1
                || matching[0].binding.is_some()
                || matching[0].retired_generation != Some(old.generation())
                || matching[0].retired_members != Some(0)
            {
                return Err(
                    "stale binding lacks an acknowledged actual final-empty retirement witness"
                        .into(),
                );
            }
            let level = matching[0];
            let record = host.transaction(|host, _| -> Result<Option<OwnerState>, String> {
                let current = owner.source.revisions(host).map_err(error)?;
                if current.block_revision != level.block_revision
                    || current.content_revision != level.content_revision
                    || owner.source.live_world(host).map_err(error)?.is_some()
                {
                    return Err(
                        "empty retirement witness changed before caller phase publication".into(),
                    );
                }
                if let Some(mut record) = owner.record.clone() {
                    record.phase = Phase::Retiring {
                        generation: old.generation(),
                        members: owner.members.clone(),
                    };
                    owner_state::OWNER
                        .set(
                            &mut owner.source.attachments(host),
                            Holder::Level,
                            record.clone(),
                        )
                        .map_err(error)?;
                    Ok(Some(record))
                } else {
                    let mut saved = Self::read_physical(host, owner.source)?
                        .ok_or_else(|| "retired legacy physical metadata missing".to_string())?;
                    saved.deleted = true;
                    persist::MOTION
                        .set(&mut owner.source.attachments(host), Holder::Level, saved)
                        .map_err(error)?;
                    Ok(None)
                }
            })?;
            let confirmed = owner.source.checkpoint(host).map_err(error)?;
            if confirmed.barrier_epoch < receipt.barrier_epoch
                || !confirmed.levels.iter().any(|level| {
                    level.level == owner.source
                        && level.binding.is_none()
                        && level.retired_generation == Some(old.generation())
                        && level.retired_members == Some(0)
                })
            {
                return Err(
                    "caller empty-retirement phase lacks an actual matching barrier".into(),
                );
            }
            if let Some(expected) = &record {
                let actual = owner_state::OWNER
                    .get_existing(&mut owner.source.attachments(host), Holder::Level)
                    .map_err(error)?
                    .ok_or_else(|| "retirement caller phase missing".to_string())?;
                if !owner_state::same_record(expected, &actual) {
                    return Err("retirement caller afterimage is unconfirmed".into());
                }
            }
            let level = confirmed
                .levels
                .iter()
                .find(|level| level.level == owner.source)
                .ok_or_else(|| "acknowledged empty Source row missing".to_string())?;
            owner.empty_witness = Some((
                old.generation(),
                level.block_revision,
                level.content_revision,
            ));
            owner.record = record;
            owner.world = None;
            owner.active = false;
            owner.waiting = None;
            owner.persisted_deleted = Some(true);
        }
        Ok(())
    }
    pub(crate) fn release_bootstrap_on_stop(
        &mut self,
        host: &mut dyn ServerHost,
    ) -> Result<(), String> {
        for owner in &mut self.owners {
            Self::release_owned_tickets(host, owner, self.bootstrap_controller)?;
        }
        Ok(())
    }
    fn existing_fixture(
        host: &dyn ServerHost,
        owner: &Owner,
    ) -> Result<Vec<FixtureObservation>, String> {
        if host.level(&owner.source_id) != Some(owner.source) {
            return Err("operation requires the current owned open source".into());
        }
        if let Some(record) = &owner.record {
            let positions = record.episode.cells.keys().copied().collect::<Vec<_>>();
            let mut catalog = PropertyCatalog::new(1).map_err(error)?;
            let observed = crate::lifecycle_backend::read_snapshot(
                host,
                owner.source,
                &positions,
                &mut catalog,
            )?;
            if observed.state.cells.len() != record.episode.cells.len()
                || observed.state.cells.iter().any(|(position, actual)| {
                    record
                        .episode
                        .cells
                        .get(position)
                        .is_none_or(|expected| !reset_state_matches(expected, actual))
                })
            {
                return Err("reset refuses changed episode states before mutation".into());
            }
            return positions
                .into_iter()
                .map(|position| {
                    let actual = owner
                        .source
                        .block_state(host, position)
                        .ok_or_else(|| "episode cell unavailable".to_string())?;
                    let collision = owner
                        .source
                        .block_shape(host, actual, position, BlockShapeKind::Collision)
                        .ok_or_else(|| "episode shape unavailable".to_string())?;
                    let entity = host
                        .block_entity(owner.source, position)
                        .map(|entity| {
                            entity
                                .data(host)
                                .map(|data| (entity, data))
                                .ok_or_else(|| "episode BE unreadable".to_string())
                        })
                        .transpose()?;
                    Ok(FixtureObservation {
                        position,
                        state: actual,
                        collision,
                        entity,
                    })
                })
                .collect();
        }
        let selected = owner.members.iter().copied().collect::<BTreeSet<_>>();
        if selected.len() != owner.members.len() || selected.is_empty() || selected.len() > 512 {
            return Err("recovery fixture membership is invalid".into());
        }
        let min: [i32; 3] =
            std::array::from_fn(|axis| selected.iter().map(|p| p[axis]).min().unwrap());
        let max: [i32; 3] =
            std::array::from_fn(|axis| selected.iter().map(|p| p[axis]).max().unwrap());
        let volume = (0..3).try_fold(1i64, |n, axis| {
            n.checked_mul(i64::from(max[axis]) - i64::from(min[axis]) + 1)
        });
        if owner.mode > 3 || volume.is_none_or(|n| !(1..=1024).contains(&n)) {
            return Err("recovery is limited to the known bounded initial fixture envelope".into());
        }
        let mut catalog = PropertyCatalog::new(1).map_err(error)?;
        let mut observed = Vec::new();
        for x in min[0]..=max[0] {
            for y in min[1]..=max[1] {
                for z in min[2]..=max[2] {
                    let position = [x, y, z];
                    let actual = owner
                        .source
                        .block_state(host, position)
                        .ok_or_else(|| "recovery source envelope is not resident".to_string())?;
                    let metadata = host.block_states();
                    let name = metadata.name(actual.block()).map_err(error)?;
                    if !selected.contains(&position) {
                        if name != "minecraft:air" {
                            return Err(
                                "recovery refuses extra real cells inside the fixture envelope"
                                    .into(),
                            );
                        }
                        continue;
                    }
                    let expected = match position {
                        [3, 1, 1] => "minecraft:chest",
                        [5, 1, 1] => "minecraft:furnace",
                        [8, 1, 1] => "minecraft:stone_slab",
                        _ => "minecraft:blackstone",
                    };
                    if name != expected {
                        return Err("recovery refuses missing/replaced fixture cells".into());
                    }
                    let collision = owner
                        .source
                        .block_shape(host, actual, position, BlockShapeKind::Collision)
                        .ok_or_else(|| "recovery source shape unavailable".to_string())?;
                    resolve(host, actual, collision.clone(), &mut catalog)?;
                    let entity = host.block_entity(owner.source, position);
                    if entity.is_some() != matches!(name, "minecraft:chest" | "minecraft:furnace") {
                        return Err("recovery refuses an invalid live block-entity binding".into());
                    }
                    let entity = entity
                        .map(|entity| {
                            entity
                                .data(host)
                                .map(|data| (entity, data))
                                .ok_or_else(|| "recovery block entity is not readable".to_string())
                        })
                        .transpose()?;
                    observed.push(FixtureObservation {
                        position,
                        state: actual,
                        collision,
                        entity,
                    });
                }
            }
        }
        Ok(observed)
    }
    fn validate_saved_initial(saved: &persist::Saved, expected: &Motion) -> Result<(), String> {
        let motion = Motion {
            pose: saved.raw_pose,
            linear: saved.linear,
            angular: saved.angular,
        };
        if saved.deleted
            || !saved.paused
            || saved.native_epoch != 0
            || saved.pose_revision != 0
            || !same_motion_bits(&motion, expected)
            || !frames::equivalent(expected, saved.pose)
        {
            return Err("explicit setup recovery requires the exact paused initial epoch0 profile; moving/deleted/stale metadata is refused".into());
        }
        Ok(())
    }
    fn recover_initial(
        host: &mut dyn ServerHost,
        owner: &mut Owner,
        saved: Option<&persist::Saved>,
        controller: TicketController,
    ) -> Result<bool, String> {
        // The caller has received exactly NotSaved for the host checkpoint.
        // This is explicit operator adoption of the known interrupted own
        // fixture, never a normal restore fallback or an inventory source.
        let before = Self::existing_fixture(host, owner)?;
        let motion = Self::initial_live_motion(host, owner, initial_motion(owner.mode))?;
        if let Some(saved) = saved {
            Self::validate_saved_initial(saved, &motion)?;
        }
        if !startup_parent_resident(host, owner, &motion.pose)? {
            owner.waiting = Some(StartupWait::Parent);
            return Ok(false);
        }
        owner.motion = motion;
        owner.paused = true;
        Self::project_existing(host, owner, controller)?;
        let after = Self::existing_fixture(host, owner)?;
        if before != after {
            return Err("recovery observed a source cell/BE/data change during projection; session remains failed, no rollback claimed".into());
        }
        owner.recovered_initial = true;
        eprintln!(
            "[sable_phase3] explicit interrupted setup recovered owner={} cells_preserved={} live_entities_preserved={} blocks_written=0 items_written=0 native_epoch=0",
            owner.mode,
            before.len(),
            before.iter().filter(|cell| cell.entity.is_some()).count()
        );
        Ok(true)
    }
    fn initialize(
        host: &mut dyn ServerHost,
        owner: &mut Owner,
        allow_create: bool,
        recover_initial: bool,
        migrate_saved: bool,
        controller: TicketController,
    ) -> Result<bool, String> {
        if !owner.active {
            return Ok(false);
        }
        if owner.initialized {
            return Ok(owner.world.is_some());
        }
        owner.diagnostics.attempts = owner.diagnostics.attempts.saturating_add(1);
        owner.diagnostics.in_place = Some(owner.source.in_place(host));
        let mut resident = true;
        for [x, z] in owner.bootstrap_columns.iter().copied() {
            let chunk = owner
                .source
                .chunk_ref(host, [x, z])
                .ok_or_else(|| "source chunk unavailable".to_string())?;
            resident &= chunk.loaded(host);
        }
        if !resident {
            owner.waiting = Some(StartupWait::Source);
            return Ok(false);
        }
        owner.diagnostics.ticking = owner
            .bootstrap_columns
            .iter()
            .map(|&[x, z]| {
                let position = [x.checked_mul(16)?, 0, z.checked_mul(16)?];
                owner.source.should_tick_blocks_at(host, position)
            })
            .try_fold(true, |all, value| value.map(|value| all && value));
        if owner.diagnostics.ticking != Some(true) {
            owner.waiting = Some(StartupWait::SourceTicking);
            return Ok(false);
        }
        if owner
            .saved_descriptor
            .as_ref()
            .is_some_and(|saved| saved.recovery == LiveWorldRecovery::ExperimentalMigrationRequired)
        {
            if !migrate_saved {
                return Err("saved projection migration not authorized for this launch".into());
            }
            owner.waiting = Some(StartupWait::MigrationFrontier);
            return Ok(false);
        }
        if let Some(record) = owner_state::OWNER
            .get_existing(&mut owner.source.attachments(host), Holder::Level)
            .map_err(error)?
        {
            accept_epoch_policy(owner, record.episode.profile.epoch_policy())?;
            owner.id = record.logical.id;
            owner.plot = record.logical.plot;
            owner.motion = record.current.motion.clone();
            owner.paused = record.current.paused;
            owner.episode_motion = record.episode.initial.clone();
            owner.episode_members = record.episode.cells.keys().copied().collect();
            owner.record = Some(record.clone());
            match &record.phase {
                Phase::OwnedUnprojected { members, .. } => {
                    if owner.saved_descriptor.is_some() {
                        return Err(
                            "owned-unprojected phase disagrees with a saved active binding".into(),
                        );
                    }
                    owner.members = members.clone();
                    if !startup_parent_resident(host, owner, &owner.motion.pose)? {
                        owner.waiting = Some(StartupWait::Parent);
                        return Ok(false);
                    }
                    return Self::project_existing(host, owner, controller);
                }
                Phase::Returned { .. } | Phase::Retired { .. } | Phase::Retiring { .. } => {
                    if owner.saved_descriptor.is_some() {
                        return Err(
                            "inactive caller phase disagrees with actual host binding".into()
                        );
                    }
                    let Some(retired) = Self::confirm_inactive_retirement(host, owner, &record)?
                    else {
                        owner.waiting = Some(StartupWait::RetirementReceipt);
                        return Ok(false);
                    };
                    owner.empty_witness = Some((
                        retired.retired_generation.unwrap(),
                        retired.block_revision,
                        retired.content_revision,
                    ));
                    owner.active = false;
                    owner.initialized = true;
                    owner.waiting = None;
                    owner.persisted_epoch = Some(record.current.native_epoch);
                    owner.persisted_paused = Some(record.current.paused);
                    owner.persisted_deleted = Some(true);
                    owner.persisted_motion = Some(owner.motion.clone());
                    Self::release_owned_tickets(host, owner, controller)?;
                    return Ok(false);
                }
                Phase::Projected { .. } => {}
            }
        }
        let saved = Self::read_physical(host, owner.source)?;
        if let Some(mut saved) = saved {
            accept_epoch_policy(owner, saved.epoch_policy)?;
            if recover_initial && owner.record.is_none() {
                Self::existing_fixture(host, owner)?;
                let expected = Self::initial_live_motion(host, owner, initial_motion(owner.mode))?;
                Self::validate_saved_initial(&saved, &expected)?;
            }
            if saved.deleted {
                Self::release_owned_tickets(host, owner, controller)?;
                owner.initialized = true;
                owner.waiting = None;
                owner.paused = saved.paused;
                owner.persisted_epoch = Some(saved.native_epoch);
                owner.persisted_paused = Some(saved.paused);
                owner.persisted_deleted = Some(true);
                owner.motion = Motion {
                    pose: saved.raw_pose,
                    linear: saved.linear,
                    angular: saved.angular,
                };
                owner.persisted_motion = Some(owner.motion.clone());
                owner.persisted_pose = Some((saved.pose_revision, saved.pose));
                owner.active = false;
                return Ok(false);
            }
            if owner.saved_descriptor.is_none()
                && !startup_parent_resident(host, owner, &saved.raw_pose)?
            {
                owner.waiting = Some(StartupWait::Parent);
                return Ok(false);
            }
            // Restore uses the actual saved binding limits, including an older
            // gravity profile. Only the explicit reset/reproject path below
            // installs the currently requested per-owner limits.
            let world = match owner.source.restore_projection(host) {
                Ok(world) => world,
                Err(LiveWorldError::NotResident) => {
                    owner.waiting = Some(StartupWait::Restore);
                    return Ok(false);
                }
                Err(LiveWorldError::NotSaved) if recover_initial => {
                    return Self::recover_initial(host, owner, Some(&saved), controller);
                }
                Err(LiveWorldError::MigrationRequired) if migrate_saved => {
                    owner.waiting = Some(StartupWait::MigrationFrontier);
                    return Ok(false);
                }
                Err(problem) => return Err(error(problem)),
            };
            owner.world = Some(world);
            owner.members = world
                .cells(host)
                .map_err(error)?
                .into_iter()
                .map(|cell| cell.pos)
                .collect();
            let sample = world.sample(host).map_err(error)?;
            if !same_host_bits(sample.current, saved.pose) || sample.revision != saved.pose_revision
            {
                let raw = Motion {
                    pose: saved.raw_pose,
                    linear: saved.linear,
                    angular: saved.angular,
                };
                if !migrate_saved || !frames::equivalent(&raw, sample.current) {
                    return Err("saved host pose and motion metadata disagree; no raw-state reset or implicit migration".into());
                }
                // Old JSON cannot retroactively prove IEEE identity. This
                // explicit operator migration changes ONLY the host receipt.
                let canonical = host.transaction(|host, _| -> Result<_, String> {
                    let canonical = world.sample(host).map_err(error)?;
                    if !frames::equivalent(&raw, canonical.current) {
                        return Err(
                            "migration canonical receipt is not equivalent to saved raw physics"
                                .into(),
                        );
                    }
                    Self::write_physical(
                        host,
                        owner,
                        &raw,
                        &canonical,
                        saved.native_epoch,
                        saved.paused,
                    )?;
                    Ok(canonical)
                })?;
                Self::confirm_publication(
                    host,
                    (owner.source, owner.epoch_policy),
                    world,
                    &canonical,
                    &raw,
                    saved.native_epoch,
                    saved.paused,
                )?;
                saved.pose = canonical.current;
                saved.pose_revision = canonical.revision;
                owner.migrated_saved_record = true;
            }
            if !restored_sample_matches(&sample, saved.pose, saved.pose_revision) {
                return Err("fresh restore must retain current pose bits/revision and settle previous to current; historical carry is refused".into());
            }
            owner.motion.pose = saved.raw_pose;
            owner.motion.linear = saved.linear;
            owner.motion.angular = saved.angular;
            owner.persisted_motion = Some(owner.motion.clone());
            owner.persisted_pose = Some((saved.pose_revision, saved.pose));
            owner.host_frame = Some((sample.revision, sample.current));
            owner.paused = saved.paused;
            owner.persisted_epoch = Some(saved.native_epoch);
            owner.persisted_paused = Some(saved.paused);
            owner.persisted_deleted = Some(false);
            if let Some(record) = owner.record.as_mut() {
                record.accept(&owner.motion, &sample, saved.native_epoch, saved.paused);
            }
            owner.restored_from_save = true;
            owner.footprint = Some(Self::checked_footprint(
                host,
                owner.source,
                &owner.members,
                owner.mode,
            )?);
            Self::release_bootstrap(host, owner, controller)?;
            owner.initialized = true;
            owner.waiting = None;
            return Ok(true);
        }
        if !allow_create && !recover_initial {
            Self::release_owned_tickets(host, owner, controller)?;
            owner.active = false;
            owner.initialized = true;
            owner.waiting = None;
            eprintln!(
                "[sable_phase3] no saved owner {}; explicit first-run flag absent",
                owner.mode
            );
            return Ok(false);
        }
        // No corruption/storage/stale error is treated as absence.
        match owner.source.restore_projection(host) {
            Err(LiveWorldError::NotSaved) => {}
            Ok(world) => {
                owner.world = Some(world);
                return Err("existing checkpoint has no matching motion metadata".into());
            }
            Err(problem) => return Err(error(problem)),
        }
        if recover_initial {
            return Self::recover_initial(host, owner, None, controller);
        }
        if !startup_parent_resident(host, owner, &owner.motion.pose)? {
            owner.waiting = Some(StartupWait::Parent);
            return Ok(false);
        }
        let seed_position = if owner.source_id == OPEN_SOURCE {
            // The open fixture lives in a distant sparse Source section. Its
            // own first member, never unrelated Source air, gates seeding.
            owner.members[0]
        } else {
            [0, 0, 0]
        };
        let seed = owner
            .source
            .block(host, seed_position)
            .and_then(|block| host.block_states().name(block).ok().map(str::to_owned));
        if seed.as_deref() == Some("minecraft:air") {
            if owner.source_id == OPEN_SOURCE {
                for &position in &owner.members {
                    let actual = owner.source.block_state(host, position).ok_or_else(|| {
                        "open fixture initialization cell unavailable".to_string()
                    })?;
                    if host.block_states().name(actual.block()).map_err(error)? != "minecraft:air" {
                        return Err(
                            "open fixture initialization refuses partial existing Source cells"
                                .into(),
                        );
                    }
                }
            }
            let blackstone = state(host, "minecraft:blackstone")?;
            let chest = state(host, "minecraft:chest")?;
            let furnace = state(host, "minecraft:furnace")?;
            let slab = state(host, "minecraft:stone_slab")?;
            for position in &owner.members {
                let block = match *position {
                    [3, 1, 1] => chest,
                    [5, 1, 1] => furnace,
                    [8, 1, 1] => slab,
                    _ => blackstone,
                };
                if !owner
                    .source
                    .set_block(host, *position, block, BlockUpdateFlags::CLIENTS)
                    .map_err(error)?
                {
                    return Err("initial source write refused".into());
                }
            }
            if owner.mode < 3 {
                let chest = owner
                    .source
                    .capability(host, &ITEMS, [3, 1, 1], None)
                    .ok_or_else(|| "genuine chest ITEMS unavailable".to_string())?;
                let furnace = owner
                    .source
                    .capability(host, &ITEMS, [5, 1, 1], None)
                    .ok_or_else(|| "genuine furnace ITEMS unavailable".to_string())?;
                let contents = ItemResource::of(&item(host, "minecraft:diamond", 7)?);
                let ore = ItemResource::of(&item(host, "minecraft:iron_ore", 3)?);
                let fuel = ItemResource::of(&item(host, "minecraft:coal", 1)?);
                host.transaction(|host, tx| -> Result<(), String> {
                    if chest.insert(host, tx, 0, &contents, 7) != 7
                        || furnace.insert(host, tx, 0, &ore, 3) != 3
                        || furnace.insert(host, tx, 1, &fuel, 1) != 1
                    {
                        return Err("initial genuine inventory transaction refused".into());
                    }
                    Ok(())
                })?;
            }
        } else {
            return Err(
                "existing source has no physical initialization metadata; refusing reseed/adoption"
                    .into(),
            );
        }
        Self::project_existing(host, owner, controller)
    }
    fn initial_live_motion(
        host: &dyn ServerHost,
        owner: &Owner,
        motion: Motion,
    ) -> Result<Motion, String> {
        let limits = review_limits(owner.mode);
        let footprint = Self::checked_footprint(host, owner.source, &owner.members, owner.mode)?;
        let mut box_count = 0usize;
        let mut catalog = PropertyCatalog::new(1).map_err(error)?;
        let mut contents = Contents::default();
        for position in &owner.members {
            let actual = owner
                .source
                .block_state(host, *position)
                .ok_or_else(|| "actual source unavailable".to_string())?;
            let boxes = owner
                .source
                .block_shape(host, actual, *position, BlockShapeKind::Collision)
                .ok_or_else(|| "actual source shape unavailable".to_string())?;
            box_count = box_count
                .checked_add(boxes.len())
                .ok_or_else(|| "box count exhausted".to_string())?;
            if box_count > limits.boxes as usize {
                return Err("real fixture shape boxes exceed explicit review reservations".into());
            }
            let key = resolve(host, actual, boxes, &mut catalog)?;
            contents.blocks.insert(
                *position,
                Block {
                    state: key,
                    block_entity: None,
                    extension: Payload::default(),
                },
            );
        }
        let registry = catalog
            .mass_registry(1, contents.blocks.values().map(|b| b.state.as_str()))
            .map_err(error)?;
        let mass = registry.compute(&contents).map_err(error)?;
        let center = mass.center;
        eprintln!(
            "[sable_phase3] actual owner={} mass_properties={mass:?} live_geometry={:?}",
            owner.mode,
            contents
                .blocks
                .iter()
                .map(|(position, block)| (*position, block.state.as_str()))
                .collect::<Vec<_>>()
        );
        let mut motion = motion.at_self_com(center);
        // Initial physical COM uses the accepted f32 native boundary before the
        // host sees the first pose. This is not a per-tick position correction.
        motion.pose.position = motion.pose.position.map(|v| f64::from(v as f32));
        eprintln!(
            "[sable_phase3] review reservations owner={} actual_cells={} actual_boxes={} actual_footprint={footprint:?} limits={limits:?}",
            owner.mode,
            owner.members.len(),
            box_count
        );
        Ok(motion)
    }
    fn project_existing(
        host: &mut dyn ServerHost,
        owner: &mut Owner,
        controller: TicketController,
    ) -> Result<bool, String> {
        if owner.record.is_none() {
            owner.motion = Self::initial_live_motion(host, owner, owner.motion.clone())?;
        }
        let (world, sample) = host.transaction(|host, _| -> Result<_, String> {
            let world = owner
                .source
                .project(
                    host,
                    owner.parent,
                    &owner.members,
                    host_pose(&owner.motion),
                    review_limits(owner.mode),
                )
                .map_err(error)?;
            let sample = world.sample(host).map_err(error)?;
            if !frames::equivalent(&owner.motion, sample.current) {
                return Err("initial host pose is outside the explicit physical mapping".into());
            }
            let epoch = owner
                .record
                .as_ref()
                .map_or(0, |record| record.current.native_epoch);
            Self::write_physical(host, owner, &owner.motion, &sample, epoch, owner.paused)?;
            Ok((world, sample))
        })?;
        let epoch = owner
            .record
            .as_ref()
            .map_or(0, |record| record.current.native_epoch);
        Self::confirm_publication(
            host,
            (owner.source, owner.epoch_policy),
            world,
            &sample,
            &owner.motion,
            epoch,
            owner.paused,
        )?;
        owner.world = Some(world);
        owner.empty_witness = None;
        owner.footprint = Some(Self::checked_footprint(
            host,
            owner.source,
            &owner.members,
            owner.mode,
        )?);
        Self::release_bootstrap(host, owner, controller)?;
        owner.persisted_pose = Some((sample.revision, sample.current));
        owner.persisted_epoch = Some(epoch);
        owner.persisted_paused = Some(owner.paused);
        owner.persisted_motion = Some(owner.motion.clone());
        owner.persisted_deleted = Some(false);
        owner.host_frame = Some((sample.revision, sample.current));
        if let Some(record) = owner.record.as_mut() {
            record.accept(&owner.motion, &sample, epoch, owner.paused);
        }
        owner.initialized = true;
        owner.waiting = None;
        if owner.record.is_none() {
            owner.episode_motion = owner.motion.clone();
            owner.episode_members = owner.members.clone();
        }
        eprintln!(
            "[sable_phase3] real source projection ready owner {} cells {}",
            owner.mode,
            owner.members.len()
        );
        Ok(true)
    }
    fn read_physical(
        host: &mut dyn ServerHost,
        source: LevelRef,
    ) -> Result<Option<persist::Saved>, String> {
        if let Some(record) = owner_state::OWNER
            .get_existing(&mut source.attachments(host), Holder::Level)
            .map_err(error)?
        {
            return Ok(record.projected());
        }
        persist::MOTION
            .get_existing(&mut source.attachments(host), Holder::Level)
            .map_err(error)
    }
    fn write_physical(
        host: &mut dyn ServerHost,
        owner: &Owner,
        motion: &Motion,
        sample: &LiveWorldSample,
        epoch: u64,
        paused: bool,
    ) -> Result<(), String> {
        Self::write_physical_policy(
            host,
            owner,
            motion,
            sample,
            epoch,
            paused,
            owner.epoch_policy,
        )
    }
    fn write_physical_policy(
        host: &mut dyn ServerHost,
        owner: &Owner,
        motion: &Motion,
        sample: &LiveWorldSample,
        epoch: u64,
        paused: bool,
        epoch_policy: persist::EpochPolicy,
    ) -> Result<(), String> {
        if !epoch_policy_source_allowed(&owner.source_id, epoch_policy) {
            return Err("OpenDemo physical metadata requires its explicitly owned Source".into());
        }
        if let Some(mut record) = owner.record.clone() {
            if record.episode.profile.epoch_policy() != epoch_policy {
                if owner.source_id != OPEN_SOURCE || epoch_policy != persist::EpochPolicy::OpenDemo
                {
                    return Err(
                        "physical publication cannot change a historical epoch policy".into(),
                    );
                }
                record.episode.profile = Profile::OpenDemo;
            }
            record.accept(motion, sample, epoch, paused);
            owner_state::OWNER
                .set(&mut owner.source.attachments(host), Holder::Level, record)
                .map_err(error)
        } else {
            let mut saved = persist::Saved::new(motion, sample, false, paused, epoch);
            saved.epoch_policy = epoch_policy;
            persist::MOTION
                .set(&mut owner.source.attachments(host), Holder::Level, saved)
                .map_err(error)
        }
    }
    fn write_inactive(
        host: &mut dyn ServerHost,
        owner: &Owner,
        epoch: u64,
        paused: bool,
    ) -> Result<InactiveImage, String> {
        if owner.source.live_world(host).map_err(error)?.is_some() {
            return Err("inactive source still has a host binding".into());
        }
        if let Some(mut record) = owner.record.clone() {
            if matches!(
                record.phase,
                Phase::Projected { .. } | Phase::OwnedUnprojected { .. }
            ) {
                return Err("inactive owner has an active canonical phase".into());
            }
            record.current.native_epoch = epoch;
            record.current.paused = paused;
            owner_state::OWNER
                .set(
                    &mut owner.source.attachments(host),
                    Holder::Level,
                    record.clone(),
                )
                .map_err(error)?;
            Ok(InactiveImage::Canonical(Box::new(record)))
        } else {
            let mut saved = persist::MOTION
                .get_existing(&mut owner.source.attachments(host), Holder::Level)
                .map_err(error)?
                .ok_or_else(|| "inactive source tombstone missing".to_string())?;
            saved.deleted = true;
            saved.native_epoch = epoch;
            saved.paused = paused;
            persist::MOTION
                .set(
                    &mut owner.source.attachments(host),
                    Holder::Level,
                    saved.clone(),
                )
                .map_err(error)?;
            Ok(InactiveImage::Legacy(Box::new(saved)))
        }
    }
    fn verify_inactive(
        host: &mut dyn ServerHost,
        owner: &Owner,
        expected: &InactiveImage,
    ) -> Result<(), String> {
        if owner.source.live_world(host).map_err(error)?.is_some() {
            return Err("post-close inactive Source has an actual binding".into());
        }
        let matches = match expected {
            InactiveImage::Canonical(expected) => owner_state::OWNER
                .get_existing(&mut owner.source.attachments(host), Holder::Level)
                .map_err(error)?
                .is_some_and(|actual| owner_state::same_record(&actual, expected)),
            InactiveImage::Legacy(expected) => persist::MOTION
                .get_existing(&mut owner.source.attachments(host), Holder::Level)
                .map_err(error)?
                .is_some_and(|actual| actual.same_bits(expected)),
        };
        if !matches {
            return Err("post-close inactive metadata is unconfirmed".into());
        }
        Ok(())
    }
    fn remember_inactive(owner: &mut Owner, image: InactiveImage) {
        let (motion, epoch, paused) = match image {
            InactiveImage::Canonical(record) => {
                let values = (
                    record.current.motion.clone(),
                    record.current.native_epoch,
                    record.current.paused,
                );
                owner.record = Some(*record);
                values
            }
            InactiveImage::Legacy(saved) => (
                Motion {
                    pose: saved.raw_pose,
                    linear: saved.linear,
                    angular: saved.angular,
                },
                saved.native_epoch,
                saved.paused,
            ),
        };
        owner.motion = motion.clone();
        owner.persisted_epoch = Some(epoch);
        owner.persisted_paused = Some(paused);
        owner.persisted_deleted = Some(true);
        owner.persisted_motion = Some(motion);
        owner.paused = paused;
    }
    fn confirm_publication(
        host: &mut dyn ServerHost,
        source: (LevelRef, persist::EpochPolicy),
        world: LiveWorldRef,
        accepted: &LiveWorldSample,
        motion: &Motion,
        epoch: u64,
        paused: bool,
    ) -> Result<(), String> {
        let (source, epoch_policy) = source;
        // A void root close or fenced backend cannot turn closure Ok into
        // accepted feedback. Re-read the actual owner/revision/IEEE sample and
        // the actual target attachment after the root has closed. Previous
        // equality here checks the immediate in-process publication receipt;
        // it is not a promise to replay that historical frame on restore.
        let actual = world.sample(host).map_err(error)?;
        if actual.revision != accepted.revision
            || !same_host_bits(actual.current, accepted.current)
            || !same_host_bits(actual.previous, accepted.previous)
        {
            return Err(
                "post-close host publication sample differs from returned canonical receipt".into(),
            );
        }
        let saved = Self::read_physical(host, source)?
            .ok_or_else(|| "post-close source metadata is unavailable".to_string())?;
        if saved.epoch_policy != epoch_policy
            || !saved.matches(motion, &actual, false, paused, epoch)
        {
            return Err(
                "post-close source metadata differs from accepted physical publication".into(),
            );
        }
        Ok(())
    }
    fn remember_motion(
        owner: &mut Owner,
        motion: &Motion,
        sample: &LiveWorldSample,
        epoch: u64,
        paused: bool,
    ) {
        owner.motion = motion.clone();
        owner.host_frame = Some((sample.revision, sample.current));
        owner.persisted_pose = Some((sample.revision, sample.current));
        owner.persisted_epoch = Some(epoch);
        owner.persisted_paused = Some(paused);
        owner.persisted_motion = Some(motion.clone());
        owner.persisted_deleted = Some(false);
        owner.paused = paused;
        if let Some(record) = owner.record.as_mut() {
            record.accept(motion, sample, epoch, paused);
        }
    }
    fn accept_batch(
        &mut self,
        host: &mut dyn ServerHost,
        motions: &BTreeMap<Id, Motion>,
        epoch: u64,
        paused: bool,
        publish_pose: bool,
    ) -> Result<(), String> {
        // One root for the complete scene. A later refusal rolls back every
        // earlier pose/attachment/once-per-tick marker in this host batch.
        // Native integration has already happened and is never rolled back.
        let (active, deleted) = host.transaction(|host, _| -> Result<_, String> {
            let mut active = Vec::new();
            let mut deleted = Vec::new();
            for (index, owner) in self.owners.iter().enumerate() {
                if !owner.active {
                    deleted.push((index, Self::write_inactive(host, owner, epoch, paused)?));
                    continue;
                }
                let world = owner.world.ok_or_else(|| {
                    "physical batch requires every active host binding".to_string()
                })?;
                let motion = motions
                    .get(&owner.id)
                    .ok_or_else(|| "physical batch omitted an active native motion".to_string())?;
                let before = world.sample(host).map_err(error)?;
                if owner.host_frame != Some((before.revision, before.current)) {
                    return Err("host frame changed before physical batch".into());
                }
                let accepted = if publish_pose {
                    world
                        .publish_pose(host, before.revision, host_pose(motion))
                        .map_err(error)?
                } else {
                    before
                };
                if !frames::equivalent(motion, accepted.current) {
                    return Err("host receipt differs from the explicit native mapping".into());
                }
                Self::write_physical(host, owner, motion, &accepted, epoch, paused)?;
                active.push((index, accepted, motion.clone()));
            }
            Ok((active, deleted))
        })?;
        // Verify ALL actual post-close results before remembering ANY result
        // or advancing the global scene epoch. Closure Ok is not a save receipt.
        for (index, sample, motion) in &active {
            let owner = &self.owners[*index];
            Self::confirm_publication(
                host,
                (owner.source, owner.epoch_policy),
                owner
                    .world
                    .ok_or_else(|| "post-close binding missing".to_string())?,
                sample,
                motion,
                epoch,
                paused,
            )?;
        }
        for (index, expected) in &deleted {
            Self::verify_inactive(host, &self.owners[*index], expected)?;
        }
        for (index, sample, motion) in active {
            Self::remember_motion(&mut self.owners[index], &motion, &sample, epoch, paused);
        }
        for (index, image) in deleted {
            Self::remember_inactive(&mut self.owners[index], image);
        }
        Ok(())
    }
    fn retire_empty(host: &mut dyn ServerHost, owner: &mut Owner) -> Result<(), String> {
        let old = owner
            .world
            .ok_or_else(|| "empty retirement has no binding".to_string())?;
        let mut record = owner.record.clone();
        if let Some(record) = record.as_mut() {
            record.phase = Phase::Retiring {
                generation: old.generation(),
                members: owner.members.clone(),
            };
        }
        let legacy = if record.is_none() {
            Some(
                Self::read_physical(host, owner.source)?
                    .ok_or_else(|| "empty owner physical metadata missing".to_string())?,
            )
        } else {
            None
        };
        host.transaction(|host, _| -> Result<(), String> {
            if !old.cells(host).map_err(error)?.iter().all(|cell| {
                host.block_states().name(cell.state.block()).ok() == Some("minecraft:air")
            }) {
                return Err("empty retirement readset changed".into());
            }
            old.retire(host).map_err(error)?;
            if let Some(record) = &record {
                owner_state::OWNER
                    .set(
                        &mut owner.source.attachments(host),
                        Holder::Level,
                        record.clone(),
                    )
                    .map_err(error)?;
            } else if let Some(saved) = &legacy {
                let mut saved = saved.clone();
                saved.deleted = true;
                persist::MOTION
                    .set(&mut owner.source.attachments(host), Holder::Level, saved)
                    .map_err(error)?;
            }
            Ok(())
        })?;
        if !matches!(old.sample(host), Err(LiveWorldError::Stale))
            || owner.source.live_world(host).map_err(error)?.is_some()
        {
            return Err("empty retirement post-close binding is unconfirmed".into());
        }
        owner.record = record;
        owner.world = None;
        owner.active = false;
        owner.persisted_deleted = Some(true);
        Ok(())
    }
    fn observe(
        host: &dyn ServerHost,
        owner: &Owner,
        catalog: &mut PropertyCatalog,
    ) -> Result<Option<GeometrySnapshot>, String> {
        let world = owner
            .world
            .ok_or_else(|| "projection unavailable".to_string())?;
        let mut cells = BTreeMap::new();
        let mut revision = 0;
        for cell in world.cells(host).map_err(error)? {
            revision = revision.max(cell.revision);
            if host
                .block_states()
                .name(cell.state.block())
                .map_err(error)?
                == "minecraft:air"
            {
                continue;
            }
            let boxes = cell
                .collision
                .iter()
                .map(|b| [b.min[0], b.min[1], b.min[2], b.max[0], b.max[1], b.max[2]])
                .collect();
            cells.insert(cell.pos, resolve(host, cell.state, boxes, catalog)?);
        }
        if cells.is_empty() {
            return Ok(None);
        }
        Ok(Some(GeometrySnapshot {
            id: owner.id,
            generation: world.generation(),
            revision,
            plot: owner.plot,
            cells,
            motion: owner.motion.clone(),
        }))
    }
    fn terrain(
        host: &dyn ServerHost,
        parent: LevelRef,
        snapshots: &[GeometrySnapshot],
        catalog: &mut PropertyCatalog,
    ) -> Result<Vec<TerrainSection>, String> {
        let mut positions = BTreeSet::new();
        let mut sections: BTreeMap<[i32; 3], TerrainSection> = BTreeMap::new();
        for snapshot in snapshots {
            let mut bounds = [
                f64::INFINITY,
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ];
            for position in snapshot.cells.keys() {
                for corner in 0..8 {
                    let local = std::array::from_fn(|axis| {
                        f64::from(position[axis]) + if corner & (1 << axis) == 0 { 0. } else { 1. }
                    });
                    let point = snapshot.motion.pose.transform(local);
                    for axis in 0..3 {
                        bounds[axis] = bounds[axis].min(point[axis] - 1.);
                        bounds[axis + 3] = bounds[axis + 3].max(point[axis] + 1.);
                    }
                }
            }
            if !parent_resident(host, parent, bounds)? {
                return Err("actual parent terrain columns are not resident".into());
            }
            let boxes = parent
                .collision_boxes(host, bounds)
                .ok_or_else(|| "actual terrain not resident or shape query refused".to_string())?;
            for bounds in boxes {
                if (0..3).any(|axis| {
                    bounds[axis].fract() != 0.
                        || bounds[axis + 3] - bounds[axis] != 1.
                        || bounds[axis] < f64::from(i32::MIN)
                        || bounds[axis] > f64::from(i32::MAX)
                }) {
                    return Err("nonfull/contextual static terrain outside bounded profile".into());
                }
                positions.insert([bounds[0] as i32, bounds[1] as i32, bounds[2] as i32]);
                if positions.len() > 8192 {
                    return Err("bounded terrain observation budget".into());
                }
            }
        }
        for position in positions {
            let actual = parent
                .block_state(host, position)
                .ok_or_else(|| "actual terrain cell unavailable".to_string())?;
            let name = host
                .block_states()
                .name(actual.block())
                .map_err(error)?
                .to_string();
            if !matches!(name.as_str(), "minecraft:blackstone" | "minecraft:bedrock") {
                return Err(format!("unsupported actual terrain state {name}"));
            }
            let boxes = parent
                .block_shape(host, actual, position, BlockShapeKind::Collision)
                .ok_or_else(|| "actual terrain shape unavailable".to_string())?;
            if boxes != vec![[0., 0., 0., 1., 1., 1.]] {
                return Err("actual static shape is not a full cube".into());
            }
            let key = resolve(host, actual, boxes, catalog)?;
            let section = position.map(|v| v.div_euclid(16));
            sections
                .entry(section)
                .or_insert_with(|| TerrainSection {
                    section,
                    cells: BTreeMap::new(),
                })
                .cells
                .insert(position.map(|v| v.rem_euclid(16) as u8), key);
        }
        Ok(sections.into_values().collect())
    }
    fn open_ledge_terrain(
        host: &dyn ServerHost,
        parent: LevelRef,
        catalog: &mut PropertyCatalog,
    ) -> Result<Vec<TerrainSection>, String> {
        let mut sections = BTreeMap::<[i32; 3], TerrainSection>::new();
        for position in open_ledges() {
            let actual = parent
                .block_state(host, position)
                .ok_or_else(|| "open demo actual ledge cell unavailable".to_string())?;
            if host.block_states().name(actual.block()).map_err(error)? != "minecraft:blackstone" {
                return Err(format!(
                    "open demo requires actual blackstone ledge at {position:?}"
                ));
            }
            let boxes = parent
                .block_shape(host, actual, position, BlockShapeKind::Collision)
                .ok_or_else(|| "open demo actual ledge collision shape unavailable".to_string())?;
            if boxes != vec![[0., 0., 0., 1., 1., 1.]] {
                return Err("open demo ledge is not an actual full cube".into());
            }
            let key = resolve(host, actual, boxes, catalog)?;
            let section = position.map(|v| v.div_euclid(16));
            sections
                .entry(section)
                .or_insert_with(|| TerrainSection {
                    section,
                    cells: BTreeMap::new(),
                })
                .cells
                .insert(position.map(|v| v.rem_euclid(16) as u8), key);
        }
        Ok(sections.into_values().collect())
    }
    fn open_terrain(
        host: &dyn ServerHost,
        parent: LevelRef,
        snapshots: &[GeometrySnapshot],
        catalog: &mut PropertyCatalog,
        ledges: &[TerrainSection],
    ) -> Result<Vec<TerrainSection>, String> {
        let mut sections = ledges
            .iter()
            .cloned()
            .map(|s| (s.section, s))
            .collect::<BTreeMap<_, _>>();
        let mut positions = BTreeSet::new();
        for snapshot in snapshots {
            let bounds = open_swept_bounds(snapshot)?;
            if !parent_resident(host, parent, bounds)? {
                return Err("open demo actual swept terrain columns are not resident".into());
            }
            let boxes = parent
                .collision_boxes(host, bounds)
                .ok_or_else(|| "open demo actual swept collision query refused".to_string())?;
            for bounds in boxes {
                if (0..3).any(|axis| {
                    bounds[axis].fract() != 0.
                        || bounds[axis + 3] - bounds[axis] != 1.
                        || bounds[axis] < f64::from(i32::MIN)
                        || bounds[axis] > f64::from(i32::MAX)
                }) {
                    return Err(
                        "open demo actual nonfull/contextual ground collision is unsupported"
                            .into(),
                    );
                }
                positions.insert([bounds[0] as i32, bounds[1] as i32, bounds[2] as i32]);
                if positions.len() > 8192 {
                    return Err("open demo actual terrain cell budget exceeded".into());
                }
            }
        }
        for position in positions {
            let actual = parent
                .block_state(host, position)
                .ok_or_else(|| "open demo actual ground cell unavailable".to_string())?;
            let boxes = parent
                .block_shape(host, actual, position, BlockShapeKind::Collision)
                .ok_or_else(|| "open demo actual ground collision shape unavailable".to_string())?;
            let key = open_ground_key(host, actual, boxes, catalog)?;
            let section = position.map(|v| v.div_euclid(16));
            sections
                .entry(section)
                .or_insert_with(|| TerrainSection {
                    section,
                    cells: BTreeMap::new(),
                })
                .cells
                .insert(position.map(|v| v.rem_euclid(16) as u8), key);
        }
        Ok(sections.into_values().collect())
    }
    pub(crate) fn admit(
        &self,
        host: &dyn ServerHost,
        world: LiveWorldRef,
        changes: &[LiveWorldPlacementCell],
    ) -> Option<Result<(), String>> {
        let owner = self
            .owners
            .iter()
            .find(|owner| owner.world == Some(world))?;
        Some((|| {
            if self.failed || !owner.active || changes.is_empty() {
                return Err("owned projection is not admitting physical edits".into());
            }
            let mut catalog = PropertyCatalog::new(1).map_err(error)?;
            let mut snapshots = Vec::new();
            let mut candidate = None;
            for current in &self.owners {
                if current.active
                    && let Some(snapshot) = Self::observe(host, current, &mut catalog)?
                {
                    if current.id == owner.id {
                        candidate = Some(snapshot);
                    } else {
                        snapshots.push(snapshot);
                    }
                }
            }
            let mut candidate = candidate.unwrap_or_else(|| GeometrySnapshot {
                id: owner.id,
                generation: world.generation(),
                revision: 0,
                plot: owner.plot,
                cells: BTreeMap::new(),
                motion: owner.motion.clone(),
            });
            let mut positions = BTreeSet::new();
            for change in changes {
                if !positions.insert(change.pos) {
                    return Err("duplicate prospective placement cell".into());
                }
                if host
                    .block_states()
                    .name(change.state.block())
                    .map_err(error)?
                    == "minecraft:air"
                {
                    candidate.cells.remove(&change.pos);
                } else {
                    let boxes = change
                        .collision
                        .iter()
                        .map(|b| [b.min[0], b.min[1], b.min[2], b.max[0], b.max[1], b.max[2]])
                        .collect();
                    candidate.cells.insert(
                        change.pos,
                        resolve(host, change.state, boxes, &mut catalog)?,
                    );
                }
            }
            // This is a derived validation epoch only, not host commit authority.
            // The generic action path revalidates its real source/pose epochs.
            candidate.revision = candidate
                .revision
                .checked_add(1)
                .ok_or_else(|| "source revision exhausted".to_string())?;
            if !candidate.cells.is_empty() {
                snapshots.push(candidate);
            }
            let masses = catalog
                .mass_registry(
                    1,
                    snapshots
                        .iter()
                        .flat_map(|snapshot| snapshot.cells.values())
                        .map(String::as_str),
                )
                .map_err(error)?;
            self.replica
                .validate_snapshots(&snapshots, &masses)
                .map_err(error)?;
            Ok(())
        })())
    }
    pub(crate) fn matches_policy_slot(&self, expected: &Slot) -> Option<bool> {
        let owner = self
            .owners
            .iter()
            .find(|owner| owner.mode == expected.number)?;
        Some(
            !self.failed
                && owner.initialized == expected.initialized
                && owner.active == expected.active
                && owner.source == expected.source
                && owner.parent == expected.parent
                && owner.id == expected.id
                && owner.plot == expected.plot
                && owner.world == expected.world
                && owner.paused == expected.paused
                && self.ticks == expected.epoch
                && same_motion_bits(&owner.motion, &expected.motion)
                && owner.record.as_ref().map(|r| r.logical)
                    == expected.record.as_ref().map(|r| r.logical),
        )
    }
    pub(crate) fn lifecycle_slots(&self) -> Vec<Slot> {
        self.owners
            .iter()
            .map(|owner| Slot {
                number: owner.mode,
                source: owner.source,
                parent: owner.parent,
                source_id: owner.source_id.clone(),
                world: owner.world,
                active: owner.active,
                initialized: owner.initialized,
                failed: self.failed,
                id: owner.id,
                plot: owner.plot,
                motion: owner.motion.clone(),
                epoch: self.ticks,
                paused: owner.paused,
                members: owner.members.clone(),
                record: owner.record.clone(),
                native_bodies: self.replica.counts().0,
                host_frame: owner.host_frame,
                native_sample: self.replica.native_sample(owner.id).ok(),
                empty_witness: owner.empty_witness,
            })
            .collect()
    }
    pub(crate) fn install_records(
        &mut self,
        host: &mut dyn ServerHost,
        records: &[(u8, OwnerState)],
    ) -> Result<(), String> {
        for (number, expected) in records {
            let Some(owner) = self.owners.iter_mut().find(|owner| owner.mode == *number) else {
                continue;
            };
            let actual = owner_state::OWNER
                .get_existing(&mut owner.source.attachments(host), Holder::Level)
                .map_err(error)?
                .ok_or_else(|| "committed canonical Source record absent".to_string())?;
            if !owner_state::same_record(expected, &actual) {
                return Err("committed Source record changed before cache installation".into());
            }
            let same_owner = owner.id == actual.logical.id;
            accept_epoch_policy(owner, actual.episode.profile.epoch_policy())?;
            owner.id = actual.logical.id;
            owner.plot = actual.logical.plot;
            owner.motion = actual.current.motion.clone();
            owner.paused = actual.current.paused;
            owner.episode_motion = actual.episode.initial.clone();
            owner.episode_members = actual.episode.cells.keys().copied().collect();
            match &actual.phase {
                Phase::Projected {
                    pose_revision,
                    pose,
                } if same_owner => {
                    let world = owner
                        .world
                        .ok_or_else(|| "projected install lost actual binding".to_string())?;
                    let sample = world.sample(host).map_err(error)?;
                    if sample.revision != *pose_revision || !same_host_bits(sample.current, *pose) {
                        return Err("projected install has a stale host frame".into());
                    }
                    Self::remember_motion(
                        owner,
                        &actual.current.motion,
                        &sample,
                        actual.current.native_epoch,
                        actual.current.paused,
                    );
                }
                Phase::OwnedUnprojected { members, .. } => {
                    if owner.source.live_world(host).map_err(error)?.is_some() {
                        return Err("owned-unprojected install still has a binding".into());
                    }
                    owner.world = None;
                    owner.active = true;
                    owner.initialized = false;
                    owner.members = members.clone();
                    owner.bootstrap_columns = members
                        .iter()
                        .map(|p| [p[0].div_euclid(16), p[2].div_euclid(16)])
                        .collect();
                    owner.saved_descriptor = None;
                    owner.waiting = Some(StartupWait::Source);
                }
                Phase::Returned { .. } | Phase::Retired { .. } | Phase::Retiring { .. } => {
                    if owner.source.live_world(host).map_err(error)?.is_some() {
                        return Err("retired install still has a host binding".into());
                    }
                    owner.world = None;
                    owner.active = false;
                    owner.initialized = true;
                    owner.waiting = None;
                    owner.persisted_deleted = Some(true);
                }
                Phase::Projected { .. } => {
                    return Err("a new logical owner cannot adopt an old projection handle".into());
                }
            }
            owner.persisted_epoch = Some(actual.current.native_epoch);
            owner.persisted_paused = Some(actual.current.paused);
            owner.persisted_motion = Some(actual.current.motion.clone());
            owner.record = Some(actual);
            if !same_owner {
                owner.native_realizations = 0;
            }
            if !same_owner || owner.world.is_none() {
                owner.last_native_body = None;
                owner.empty_witness = None;
                owner.initial_native_readback = None;
                owner.last_integrated = None;
                owner.host_frame = None;
            }
        }
        // Realization is refreshed from the complete actual set on the next
        // parent tick. No inventory or geometry is reconstructed from records.
        Ok(())
    }
    pub(crate) fn reset_pending(&self) -> bool {
        self.reset_requested
    }
    pub(crate) fn cancel_open_demo_reset(&mut self, problem: &str) {
        if self.open_profile.is_some() {
            self.reset_requested = false;
            self.reset_error = Some(problem.into());
        }
    }
    fn open_demo_reset_observers_clear(&self, host: &dyn ServerHost) -> Result<(), String> {
        let players = host.players().server_players();
        if players.is_empty() {
            return Err("reset requires an online observer".into());
        }
        for owner in self.owners.iter().filter(|owner| owner.active) {
            let world = owner
                .world
                .ok_or("reset requires current active bindings")?;
            let sample = world.sample(host).map_err(error)?;
            for player in &players {
                let parent = player
                    .level(host)
                    .ok_or("reset observer level unavailable")?;
                if parent != owner.parent {
                    continue;
                }
                let bounds = player
                    .entity()
                    .bounding_box(host)
                    .ok_or("reset observer bounds unavailable")?;
                for (pose, members) in [
                    (sample.previous, &owner.members),
                    (sample.current, &owner.members),
                    (host_pose(&owner.episode_motion), &owner.episode_members),
                ] {
                    if !open_reset_clearance(bounds, pose, members)? {
                        return Err("reset observer overlaps current or initial projection".into());
                    }
                }
            }
        }
        Ok(())
    }
    pub(crate) fn reset_episode(&mut self, host: &mut dyn ServerHost) -> Result<(), String> {
        self.reset_requested = false;
        self.reset_error = None;
        self.reset_receipts.clear();
        let mut root_attempted = false;
        let result = (|| -> Result<(), String> {
            if self.open_profile.is_some() {
                self.open_demo_reset_observers_clear(host)?;
                // Recheck at the mutation boundary; the command's earlier
                // observation cannot authorize changed/unloaded real ledges.
                let mut catalog = PropertyCatalog::new(1).map_err(error)?;
                Self::open_ledge_terrain(host, self.owners[0].parent, &mut catalog)?;
            }
            let mut prepared = Vec::new();
            // Complete every readiness/geometry/mass/parent check before the
            // first retire. Observations are short-lived diagnostics only.
            for (index, owner) in self
                .owners
                .iter_mut()
                .enumerate()
                .filter(|(_, owner)| owner.active)
            {
                let world = owner
                    .world
                    .ok_or_else(|| "reset requires current active bindings".to_string())?;
                let cells = world.cells(host).map_err(error)?;
                let expected = owner
                    .episode_members
                    .iter()
                    .copied()
                    .collect::<BTreeSet<_>>();
                if cells.len() != expected.len()
                    || cells.iter().map(|cell| cell.pos).collect::<BTreeSet<_>>() != expected
                    || owner.members.iter().copied().collect::<BTreeSet<_>>() != expected
                {
                    return Err(
                        "reset refuses edited/expanded episode membership before mutation".into(),
                    );
                }
                let before = Self::existing_fixture(host, owner)?;
                let motion = if owner.record.is_some() {
                    owner.episode_motion.clone()
                } else {
                    Self::initial_live_motion(host, owner, owner.episode_motion.clone())?
                };
                if !startup_parent_resident(host, owner, &motion.pose)? {
                    return Err(
                        "reset initial parent region is not resident; old bindings retained".into(),
                    );
                }
                for &position in &owner.members {
                    if owner.source.should_tick_blocks_at(host, position) != Some(true) {
                        return Err("reset Source is not actually tick-ready".into());
                    }
                }
                Self::checked_footprint(host, owner.source, &owner.members, owner.mode)?;
                prepared.push((index, world, motion, before));
            }
            if prepared.is_empty() {
                return Err(
                    "reset has no active episode; it never recreates deleted structures".into(),
                );
            }
            for (index, _, _, _) in &prepared {
                let owner = &mut self.owners[*index];
                owner.bootstrap_columns = owner
                    .members
                    .iter()
                    .map(|p| [p[0].div_euclid(16), p[2].div_euclid(16)])
                    .collect();
                Self::hold_bootstrap(host, owner, self.bootstrap_controller)?;
            }
            root_attempted = true;
            let accepted = host.transaction(|host, _| -> Result<_, String> {
                if self.open_profile.is_some() {
                    // Check again immediately before retire/project. No player
                    // is relocated and no occupied surface is removed or added.
                    self.open_demo_reset_observers_clear(host)?;
                }
                let mut accepted = Vec::new();
                for (index, old, motion, _) in &prepared {
                    let owner = &self.owners[*index];
                    old.retire(host).map_err(error)?;
                    let world = owner
                        .source
                        .project(
                            host,
                            owner.parent,
                            &owner.members,
                            host_pose(motion),
                            review_limits(owner.mode),
                        )
                        .map_err(error)?;
                    let sample = world.sample(host).map_err(error)?;
                    if !frames::equivalent(motion, sample.current) {
                        return Err("reset host canonical mapping changed".into());
                    }
                    let policy = if self.open_profile.is_some() {
                        persist::EpochPolicy::OpenDemo
                    } else {
                        owner.epoch_policy
                    };
                    Self::write_physical_policy(host, owner, motion, &sample, 0, true, policy)?;
                    accepted.push((*index, world, sample, motion.clone()));
                }
                let mut inactive = Vec::new();
                for (index, owner) in self
                    .owners
                    .iter()
                    .enumerate()
                    .filter(|(_, owner)| !owner.active)
                {
                    inactive.push((index, Self::write_inactive(host, owner, 0, true)?));
                }
                Ok((accepted, inactive))
            })?;
            let (accepted, inactive) = accepted;
            for (index, image) in &inactive {
                Self::verify_inactive(host, &self.owners[*index], image)?;
            }
            // Verify all actual results and same live BE handles/NBT BEFORE
            // disposing native bodies or remembering any new episode epoch.
            for (index, world, sample, motion) in &accepted {
                let owner = &self.owners[*index];
                let policy = if self.open_profile.is_some() {
                    persist::EpochPolicy::OpenDemo
                } else {
                    owner.epoch_policy
                };
                Self::confirm_publication(
                    host,
                    (owner.source, policy),
                    *world,
                    sample,
                    motion,
                    0,
                    true,
                )?;
                let before = &prepared
                    .iter()
                    .find(|(before_index, _, _, _)| before_index == index)
                    .unwrap()
                    .3;
                if Self::existing_fixture(host, owner)? != *before {
                    return Err(
                        "reset source cells/BE handles/data changed; no rollback claimed".into(),
                    );
                }
            }
            for (_, old, _, _) in &prepared {
                if !matches!(old.sample(host), Err(LiveWorldError::Stale)) {
                    return Err("reset old handle did not become stale".into());
                }
            }
            for (index, world, sample, motion) in accepted {
                let old = prepared
                    .iter()
                    .find(|(before_index, _, _, _)| *before_index == index)
                    .unwrap()
                    .1;
                let owner = &mut self.owners[index];
                if self.open_profile.is_some() {
                    owner.epoch_policy = persist::EpochPolicy::OpenDemo;
                    if let Some(record) = &mut owner.record {
                        record.episode.profile = Profile::OpenDemo;
                    }
                }
                owner.world = Some(world);
                owner.empty_witness = None;
                Self::remember_motion(owner, &motion, &sample, 0, true);
                owner.restored_from_save = false;
                owner.initial_native_readback = None;
                owner.last_integrated = None;
                Self::release_bootstrap(host, owner, self.bootstrap_controller)?;
                self.reset_receipts
                    .push((owner.mode, old.generation(), world.generation()));
            }
            for (index, image) in inactive {
                Self::remember_inactive(&mut self.owners[index], image);
            }
            self.ticks = 0;
            self.paused = true;
            self.epoch_restored = true;
            if let Err(problem) = self
                .replica
                .sync(&[], &MassRegistry::default())
                .map_err(error)
            {
                self.failed = true;
                return Err(format!(
                    "host reset committed; native realization disposal failed: {problem}"
                ));
            }
            self.ticks = 0;
            self.paused = true;
            self.failed = false;
            self.epoch_restored = true;
            self.pending_impulse = false;
            self.step_once = false;
            self.collision_count = 0;
            self.last_collision.clear();
            if let Some(profile) = &mut self.open_profile {
                *profile = OpenProfile::default();
            }
            self.stale_handle_refused = true;
            eprintln!(
                "[sable_phase3] explicit episode reset dynamic={} preserved_actual_sources={} native_epoch=0",
                self.dynamic,
                prepared.len()
            );
            Ok(())
        })();
        if let Err(problem) = &result {
            self.reset_error = Some(problem.clone());
            if root_attempted {
                self.failed = true;
                self.paused = true;
            }
            // A preflight refusal does not alter the old binding. Scoped
            // tickets acquired by this attempt are released only when the old
            // binding is still the observed current binding.
            for owner in &mut self.owners {
                if owner.world.is_some()
                    && owner.source.live_world(host).ok().flatten() == owner.world
                    && let Err(cleanup) =
                        Self::release_owned_tickets(host, owner, self.bootstrap_controller)
                {
                    self.failed = true;
                    self.reset_error = Some(format!("{problem}; bootstrap cleanup: {cleanup}"));
                }
            }
        }
        result
    }
    pub(crate) fn tick(
        &mut self,
        host: &mut dyn ServerHost,
        level: LevelRef,
    ) -> Result<(), String> {
        if let Some((_, count)) = self.hook_levels.iter_mut().find(|(seen, _)| *seen == level) {
            *count = count.saturating_add(1);
        } else if self.hook_levels.len() < 16 {
            self.hook_levels.push((level, 1));
        }
        for owner in &mut self.owners {
            if owner.source == level {
                owner.diagnostics.level_hooks = owner.diagnostics.level_hooks.saturating_add(1);
                owner.diagnostics.in_place = Some(owner.source.in_place(host));
            }
        }
        if self.failed && !self.reset_requested {
            return Ok(());
        }
        for owner in self
            .owners
            .iter_mut()
            .filter(|owner| owner.active && owner.initialized)
        {
            if let Some(world) = owner.world
                && matches!(world.sample(host), Err(LiveWorldError::Stale))
                && host.level(&owner.source_id) == Some(owner.source)
                && owner.source.live_world(host).map_err(error)?.is_none()
            {
                owner.waiting = Some(StartupWait::RetirementReceipt);
                return Ok(()); // No native advance before a real zero-member receipt.
            }
        }
        for owner in &mut self.owners {
            if owner.source == level {
                let _ = Self::initialize(
                    host,
                    owner,
                    self.initialize_allowed,
                    self.recover_initial_allowed,
                    self.migrate_saved_allowed,
                    self.bootstrap_controller,
                )?;
                if let Some(world) = owner.world
                    && self.epoch_restored
                {
                    let sample = world.sample(host).map_err(error)?;
                    if owner.host_frame != Some((sample.revision, sample.current))
                        || !frames::equivalent(&owner.motion, sample.current)
                    {
                        return Err("actual host frame no longer matches the accepted native-to-host mapping".into());
                    }
                    let saved = Self::read_physical(host, owner.source)?.ok_or_else(|| {
                        "accepted physical publication has no source metadata".to_string()
                    })?;
                    if saved.epoch_policy != owner.epoch_policy
                        || !saved.matches(&owner.motion, &sample, false, owner.paused, self.ticks)
                    {
                        return Err(
                            "source metadata is not the same accepted pose/motion epoch".into()
                        );
                    }
                    owner.source_hooks += 1;
                    if owner.source_hooks == 1 || owner.source_hooks.is_multiple_of(40) {
                        for cell in world.cells(host).map_err(error)? {
                            let metadata = host.block_states();
                            let name = metadata.name(cell.state.block()).map_err(error)?;
                            if !matches!(name, "minecraft:chest" | "minecraft:furnace") {
                                continue;
                            }
                            let data = host
                                .block_entity(owner.source, cell.pos)
                                .and_then(|entity| entity.data(host));
                            if let Some(data) = data {
                                eprintln!(
                                    "[sable_phase3] live owner={} source_hook={} kind={} cell={:?} items={:?} cooking={:?} lit={:?}",
                                    owner.mode,
                                    owner.source_hooks,
                                    name,
                                    cell.pos,
                                    data.get(&"Items".into()),
                                    data.get(&"cooking_time_spent".into()),
                                    data.get(&"lit_time_remaining".into())
                                );
                            } else {
                                return Err("actual projected BE became unreadable".into());
                            }
                        }
                    }
                } else if !owner.active
                    && let Some(expected) = &owner.record
                {
                    let actual = owner_state::OWNER
                        .get_existing(&mut owner.source.attachments(host), Holder::Level)
                        .map_err(error)?
                        .ok_or_else(|| "inactive canonical owner missing".to_string())?;
                    if !owner_state::same_record(expected, &actual) {
                        return Err(
                            "inactive owner metadata changed outside its accepted transaction"
                                .into(),
                        );
                    }
                }
            }
        }
        if self
            .owners
            .first()
            .is_none_or(|owner| owner.parent != level)
            || self
                .owners
                .iter()
                .any(|owner| !owner.initialized || (owner.active && owner.world.is_none()))
        {
            return Ok(());
        }
        if !self.epoch_restored {
            let known = self
                .owners
                .iter()
                .filter_map(|owner| owner.persisted_epoch)
                .collect::<BTreeSet<_>>();
            if known.len() > 1
                || (!known.is_empty()
                    && self
                        .owners
                        .iter()
                        .any(|owner| owner.persisted_epoch.is_none()))
            {
                return Err("saved owners disagree about global native epoch; refusing partial lifetime budget restore".into());
            }
            self.ticks = known.first().copied().unwrap_or(0);
            self.epoch_restored = true;
        }
        let sampling_started = self.open_profile.as_ref().map(|_| Instant::now());
        let mut catalog = PropertyCatalog::new(1).map_err(error)?;
        let mut snapshots = Vec::new();
        for owner in &mut self.owners {
            if owner.active {
                let observed_members = owner
                    .world
                    .ok_or_else(|| "active owner binding missing".to_string())?
                    .cells(host)
                    .map_err(error)?
                    .into_iter()
                    .map(|cell| cell.pos)
                    .collect::<Vec<_>>();
                if !observed_members.is_empty() {
                    owner.members = observed_members;
                }
                match Self::observe(host, owner, &mut catalog)? {
                    Some(snapshot) => snapshots.push(snapshot),
                    None => {
                        Self::retire_empty(host, owner)?;
                    }
                }
            }
        }
        let terrain = if let Some(profile) = &mut self.open_profile {
            let ledges = Self::open_ledge_terrain(host, self.owners[0].parent, &mut catalog)?;
            if !profile.ledge_sections.is_empty() && profile.ledge_sections != ledges {
                return Err("open demo verified real ledge geometry changed".into());
            }
            profile.ledge_sections = ledges;
            Self::open_terrain(
                host,
                self.owners[0].parent,
                &snapshots,
                &mut catalog,
                &profile.ledge_sections,
            )?
        } else {
            Self::terrain(host, self.owners[0].parent, &snapshots, &mut catalog)?
        };
        let keys = snapshots
            .iter()
            .flat_map(|s| s.cells.values())
            .chain(terrain.iter().flat_map(|s| s.cells.values()));
        let masses: MassRegistry = catalog
            .mass_registry(1, keys.map(String::as_str))
            .map_err(error)?;
        self.replica.sync(&snapshots, &masses).map_err(error)?;
        self.replica
            .sync_terrain(&terrain, &masses)
            .map_err(error)?;
        if let (Some(profile), Some(started)) = (&mut self.open_profile, sampling_started) {
            profile.sampling.record(started);
        }
        for owner in &mut self.owners {
            if owner.active {
                let body = self
                    .replica
                    .body(owner.id)
                    .ok_or_else(|| "realized native body unavailable".to_string())?
                    .0;
                if owner.last_native_body != Some(body) {
                    owner.native_realizations =
                        owner.native_realizations.checked_add(1).ok_or_else(|| {
                            "native realization diagnostic counter exhausted".to_string()
                        })?;
                    owner.last_native_body = Some(body);
                }
            }
            if owner.active && owner.initial_native_readback.is_none() {
                let (native_pose, native_velocity) =
                    self.replica.native_sample(owner.id).map_err(error)?;
                let sample = owner
                    .world
                    .ok_or_else(|| "native readback has no host binding".to_string())?
                    .sample(host)
                    .map_err(error)?;
                if owner.host_frame != Some((sample.revision, sample.current))
                    || !frames::equivalent(&owner.motion, sample.current)
                {
                    return Err("pre-step native readback host receipt changed".into());
                }
                if owner.restored_from_save
                    && owner.host_frame.is_none_or(|(revision, current)| {
                        !restored_sample_matches(&sample, current, revision)
                    })
                {
                    return Err("fresh pre-step readback has an unpreserved current frame or historical carry".into());
                }
                let observed = NativeReadback {
                    restored_from_save: owner.restored_from_save,
                    native_epoch: self.ticks,
                    process_publications_before: self.process_native_publications,
                    native_pose,
                    native_velocity,
                    logical: owner.motion.clone(),
                    host_revision: sample.revision,
                    host_pose: sample.current,
                    host_previous: sample.previous,
                };
                eprintln!(
                    "[sable_phase3] {{\"event\":\"native_pre_step_readback\",\"owner\":{},\"readback\":{}}}",
                    owner.mode,
                    observed.json()
                );
                owner.initial_native_readback = Some(observed);
            }
        }
        if snapshots.is_empty() || self.migrate_saved_allowed {
            return Ok(());
        }
        if self.open_profile.is_some()
            && self
                .owners
                .iter()
                .any(|owner| owner.epoch_policy != persist::EpochPolicy::OpenDemo)
        {
            // Existing v3 metadata retains its exact review policy. Only the
            // explicit reset transaction may replace it with OpenDemo v4.
            self.paused = true;
            return Ok(());
        }
        if self.pending_impulse
            && self.world_running
            && (self.open_profile.is_none() || self.ticks == 1)
        {
            self.pending_impulse = false;
            let id = self.owners[0].id;
            let (impulse, torque) = if self.open_profile.is_some() {
                ([28., 0., 0.], [0.; 3])
            } else {
                ([0., 0., 2.], [2., -2., 0.])
            };
            self.replica
                .impulse(id, impulse, torque, true)
                .map_err(error)?;
            let motions = self.replica.capture(&masses).map_err(error)?;
            for owner in self.owners.iter().filter(|owner| owner.active) {
                let motion = motions
                    .get(&owner.id)
                    .ok_or_else(|| "impulse motion missing".to_string())?;
                if host_pose(motion) != host_pose(&owner.motion) {
                    return Err("impulse unexpectedly changed accepted pose".into());
                }
            }
            self.accept_batch(host, &motions, self.ticks, self.paused, false)?;
            if let Some(profile) = &mut self.open_profile {
                profile.impulse_epoch = Some(self.ticks);
            }
            eprintln!(
                "[sable_phase3] actual impulse native_epoch={} J={impulse:?} torque={torque:?}",
                self.ticks
            );
        }
        let epoch_limit = self.owners[0].epoch_policy.limit();
        self.paused |= self.owners.iter().any(|owner| owner.paused) || self.ticks >= epoch_limit;
        if self.paused {
            for owner in &mut self.owners {
                owner.paused = true;
            }
            return Ok(());
        }
        if !self.world_running {
            return Ok(());
        }
        let native_started = self.open_profile.as_ref().map(|_| Instant::now());
        render_step_trace("native_step_begin", self.ticks.saturating_add(1), None);
        self.replica.tick(2).map_err(error)?;
        let collisions = self.replica.collisions().map_err(error)?;
        if !collisions.len().is_multiple_of(15) {
            return Err("invalid native collision report stride".into());
        }
        if !collisions.is_empty() {
            self.collision_count += (collisions.len() / 15) as u64;
            self.last_collision = collisions[collisions.len() - 15..].to_vec();
            if let Some(profile) = &mut self.open_profile {
                for row in collisions.as_chunks::<15>().0 {
                    if let Some(index) = open_ledge_contact(row) {
                        profile.ledge_contacts[index] =
                            profile.ledge_contacts[index].saturating_add(1);
                        profile.first_ledge_epoch[index].get_or_insert(self.ticks + 1);
                        profile.peak_ledge_force[index] =
                            profile.peak_ledge_force[index].max(row[2]);
                    }
                }
            } else {
                eprintln!(
                    "[sable_phase3] actual native contacts dynamic={} epoch={} reports={:?}",
                    self.dynamic,
                    self.ticks + 1,
                    collisions
                );
            }
        }
        let motions = self.replica.capture(&masses).map_err(error)?;
        render_step_trace(
            "native_step_end",
            self.ticks.saturating_add(1),
            Some(&motions),
        );
        if let (Some(profile), Some(started)) = (&mut self.open_profile, native_started) {
            profile.native.record(started);
        }
        if self.dynamic && self.open_profile.is_none() {
            let motion = motions
                .get(&self.owners[0].id)
                .ok_or_else(|| "dynamic native sample missing".to_string())?;
            eprintln!(
                "[sable_phase3] native dynamic epoch={} host_feedback_confirmed=false pose={:?} linear={:?} angular={:?}",
                self.ticks + 1,
                motion.pose,
                motion.linear,
                motion.angular
            );
        }
        let next_epoch = self
            .ticks
            .checked_add(1)
            .ok_or_else(|| "native epoch exhausted".to_string())?;
        let will_pause = self.step_once || next_epoch == epoch_limit;
        let publication_started = self.open_profile.as_ref().map(|_| Instant::now());
        render_step_trace("publication_begin", next_epoch, Some(&motions));
        self.accept_batch(host, &motions, next_epoch, will_pause, true)?;
        render_step_trace("publication_accepted", next_epoch, Some(&motions));
        if let (Some(profile), Some(started)) = (&mut self.open_profile, publication_started) {
            profile.publication.record(started);
        }
        self.ticks = next_epoch;
        for snapshot in &snapshots {
            if let Some(owner) = self.owners.iter_mut().find(|owner| owner.id == snapshot.id) {
                owner.last_integrated = self
                    .replica
                    .body(owner.id)
                    .map(|body| (body.0, snapshot.revision));
            }
        }
        self.process_native_publications = self.process_native_publications.saturating_add(1);
        if self.dynamic && self.open_profile.is_none() {
            eprintln!(
                "[sable_phase3] dynamic host feedback confirmed epoch={}",
                self.ticks
            );
        }
        if self.step_once || self.ticks == epoch_limit {
            if let Some(profile) = &mut self.open_profile {
                profile.finished_elapsed = profile
                    .released_at
                    .map(|start| start.elapsed().as_secs_f64());
            }
            self.paused = true;
            self.step_once = false;
            for owner in &mut self.owners {
                owner.paused = true;
            }
        }
        if self.ticks.is_multiple_of(40) {
            eprintln!(
                "[sable_phase3] native publication epoch {} bodies {} sections {}",
                self.ticks,
                self.replica.counts().0,
                self.replica.counts().1
            );
        }
        if self.ticks == epoch_limit {
            eprintln!(
                "[sable_phase3] bounded native review pause at epoch {epoch_limit}; real local block ticking continues"
            );
        }
        Ok(())
    }
    pub(crate) fn checkpoint(&mut self, host: &mut dyn ServerHost) -> Result<(), String> {
        if self.checkpoint_requests == self.checkpoint_attempts {
            self.checkpoint_requests = self
                .checkpoint_requests
                .checked_add(1)
                .ok_or_else(|| "checkpoint serial exhausted".to_string())?;
        }
        self.checkpoint_attempts = self.checkpoint_requests;
        self.checkpoint_receipts.clear();
        self.checkpoint_epoch = self.ticks;
        self.checkpoint_paused = self.paused;
        let result = (|| {
            // Each accepted publication already wrote physical metadata in
            // the same host transaction. Require the actual durable barrier;
            // a queued request or successful closure is not that receipt.
            if self.failed {
                return Err(
                    "checkpoint refuses a failed session with unconfirmed native/host feedback"
                        .into(),
                );
            }
            if !self.epoch_restored
                || self
                    .owners
                    .iter()
                    .any(|owner| !owner.initialized || (owner.active && owner.world.is_none()))
            {
                return Err(
                    "checkpoint requires a coherently initialized scene and every active binding"
                        .into(),
                );
            }
            for owner in &self.owners {
                if owner.persisted_epoch != Some(self.ticks)
                    || owner.persisted_deleted != Some(!owner.active)
                {
                    return Err("checkpoint source metadata epoch/tombstone is not current".into());
                }
                if let Some(world) = owner.world {
                    let sample = world.sample(host).map_err(error)?;
                    if owner.persisted_pose != Some((sample.revision, sample.current))
                        || owner.persisted_epoch != Some(self.ticks)
                        || owner.persisted_paused != Some(owner.paused)
                        || owner.persisted_motion.as_ref() != Some(&owner.motion)
                        || !self.epoch_restored
                    {
                        return Err(format!(
                            "source motion attachment not at accepted pose epoch for {}",
                            owner.mode
                        ));
                    }
                }
            }
            for owner in &self.owners {
                let receipt = owner.source.checkpoint(host).map_err(error)?;
                if receipt.barrier_epoch == 0 {
                    return Err("host returned an uncompleted save barrier".into());
                }
                let matching = receipt
                    .levels
                    .iter()
                    .filter(|level| level.level == owner.source)
                    .collect::<Vec<_>>();
                if matching.len() != 1 {
                    return Err("host save receipt has no unique actual Source level".into());
                }
                let level = matching[0];
                if level.binding != owner.world
                    || (owner.world.is_some() && level.retired_generation.is_some())
                    || (owner.world.is_none() && level.retired_generation.is_none())
                {
                    return Err(
                        "host save receipt disagrees with active/retired binding state".into(),
                    );
                }
                self.checkpoint_receipts.push(CheckpointRecord {
                    owner: owner.mode,
                    barrier_epoch: receipt.barrier_epoch,
                    generation: level.binding.map_or(0, |binding| binding.generation()),
                    retired_generation: level.retired_generation,
                    pose_revision: owner.persisted_pose.map_or(0, |(revision, _)| revision),
                    source_revision: level.block_revision,
                    content_revision: level.content_revision,
                    source: level.level,
                    retired_members: level.retired_members,
                });
                eprintln!(
                    "[sable_phase3] host checkpoint owner={} barrier_epoch={}",
                    owner.mode, receipt.barrier_epoch
                );
            }
            Ok(())
        })();
        if result.is_ok() {
            self.checkpoint_completed = self.checkpoint_attempts;
        } else {
            self.checkpoint_failed = self.checkpoint_attempts;
        }
        eprintln!(
            "[sable_phase3] checkpoint attempt={} completed={} failed={} native_epoch={} paused={} host_receipts={:?}",
            self.checkpoint_attempts,
            self.checkpoint_completed,
            self.checkpoint_failed,
            self.checkpoint_epoch,
            self.checkpoint_paused,
            self.checkpoint_receipts.len()
        );
        result
    }
    /// Emit only after the completed stop checkpoint. The same owner thread
    /// cannot integrate between acknowledgment and this actual readback. The
    /// flag is the real saved pause state; this method never pauses physics.
    pub(crate) fn final_save_witness(
        &mut self,
        host: &mut dyn ServerHost,
    ) -> Result<String, String> {
        if self.final_save_witness_emitted {
            return Err("final-save witness already emitted for this scene".into());
        }
        if self.failed
            || self.checkpoint_completed != self.checkpoint_attempts
            || self.checkpoint_attempts == 0
            || self.checkpoint_receipts.len() != self.owners.len()
        {
            return Err(
                "final-save witness requires this scene's successful complete checkpoint".into(),
            );
        }
        fn text(value: &str) -> String {
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
        fn pose(value: LiveWorldPose) -> String {
            format!(
                r#"{{"translation_bits":{},"pivot_bits":{},"rotation_bits":{}}}"#,
                bits(&value.translation),
                bits(&value.pivot),
                bits(&value.rotation)
            )
        }
        let mut owners = Vec::new();
        for owner in &self.owners {
            let receipts = self
                .checkpoint_receipts
                .iter()
                .filter(|receipt| receipt.owner == owner.mode)
                .collect::<Vec<_>>();
            if receipts.len() != 1
                || receipts[0].source != owner.source
                || host.level(&owner.source_id) != Some(owner.source)
            {
                return Err("final checkpoint Source identity changed before witness".into());
            }
            let receipt = receipts[0];
            let current = owner.source.revisions(host).map_err(error)?;
            if current.block_revision != receipt.source_revision
                || current.content_revision != receipt.content_revision
            {
                return Err("final Source revision changed after acknowledged save".into());
            }
            let host_sample = if let Some(world) = owner.world {
                let sample = world.sample(host).map_err(error)?;
                if sample.revision != receipt.pose_revision {
                    return Err("final pose revision differs from acknowledged save".into());
                }
                Self::confirm_publication(
                    host,
                    (owner.source, owner.epoch_policy),
                    world,
                    &sample,
                    &owner.motion,
                    self.ticks,
                    owner.paused,
                )?;
                format!(
                    r#"{{"revision":{},"previous":{},"current":{}}}"#,
                    sample.revision,
                    pose(sample.previous),
                    pose(sample.current)
                )
            } else {
                if owner.source.live_world(host).map_err(error)?.is_some() {
                    return Err("final inactive witness found an unexpected active binding".into());
                }
                "null".into()
            };
            let native = self.replica.native_sample(owner.id).ok();
            let native = native
                .map(|(pose, velocity)| {
                    format!(
                        r#"{{"pose_bits":{},"velocity_bits":{}}}"#,
                        bits(&pose),
                        bits(&velocity)
                    )
                })
                .unwrap_or_else(|| "null".into());
            let id = owner
                .id
                .0
                .iter()
                .map(|byte| format!("{byte:02x}"))
                .collect::<String>();
            let logical_generation = owner
                .record
                .as_ref()
                .map(|r| r.logical.generation)
                .map(|v| v.to_string())
                .unwrap_or_else(|| "null".into());
            let retired_generation = receipt
                .retired_generation
                .map(|v| v.to_string())
                .unwrap_or_else(|| "null".into());
            let retired_members = receipt
                .retired_members
                .map(|v| v.to_string())
                .unwrap_or_else(|| "null".into());
            let integrated = owner.last_integrated.is_some_and(|(integrated, _)| {
                self.replica
                    .body(owner.id)
                    .is_some_and(|body| body.0 == integrated)
            });
            owners.push(format!(r#"{{"owner":{},"active":{},"id":"{}","plot":{},"logical_generation":{},"source_id":{},"source_handle":{},"native_epoch":{},"paused":{},"body_integrated_in_process":{},"raw_motion":{{"position_bits":{},"pivot_bits":{},"rotation_bits":{},"scale_bits":{},"linear_bits":{},"angular_bits":{}}},"host_sample":{},"native":{},"receipt":{{"barrier_epoch":{},"binding_generation":{},"retired_generation":{},"retired_members":{},"block_revision":{},"content_revision":{}}}}}"#,
                owner.mode,owner.active,id,owner.plot,logical_generation,text(&owner.source_id),text(&format!("{:?}",receipt.source)),self.ticks,owner.paused,integrated,bits(&owner.motion.pose.position),bits(&owner.motion.pose.pivot),bits(&owner.motion.pose.rotation),bits(&owner.motion.pose.scale),bits(&owner.motion.linear),bits(&owner.motion.angular),host_sample,native,receipt.barrier_epoch,receipt.generation,retired_generation,retired_members,receipt.source_revision,receipt.content_revision));
        }
        let witness = format!(
            r#"{{"schema":"sable.phase3.final_save.v1","scene":"{}","native_epoch":{},"paused":{},"process_native_publications":{},"checkpoint_request":{},"owners":[{}]}}"#,
            if self.dynamic {
                "gravity"
            } else {
                "zero_gravity"
            },
            self.ticks,
            self.paused,
            self.process_native_publications,
            self.checkpoint_completed,
            owners.join(",")
        );
        self.final_save_witness_emitted = true;
        Ok(witness)
    }
    pub(crate) fn status(&self, host: &dyn ServerHost) -> String {
        fn integer(data: &Compound, key: &str) -> Option<i64> {
            match data.get(&key.into()) {
                Some(Tag::Int(v)) => Some(i64::from(*v)),
                _ => None,
            }
        }
        fn items_in(data: &Compound, slot: Option<i8>) -> Option<i64> {
            let Tag::List(items) = data.get(&"Items".into())? else {
                return None;
            };
            items.iter().try_fold(0i64, |total, item| {
                let Tag::Compound(item) = item else {
                    return None;
                };
                if let Some(slot) = slot {
                    let Some(Tag::Byte(actual)) = item.get(&"Slot".into()) else {
                        return None;
                    };
                    if *actual != slot {
                        return Some(total);
                    }
                }
                total.checked_add(integer(item, "count")?)
            })
        }
        fn items(data: &Compound) -> Option<i64> {
            items_in(data, None)
        }
        fn json(value: Option<i64>) -> String {
            value
                .map(|v| v.to_string())
                .unwrap_or_else(|| "null".into())
        }
        fn boolean(value: Option<bool>) -> String {
            value.map_or_else(|| "null".into(), |value| value.to_string())
        }
        fn text(value: &str) -> String {
            let mut output = String::from("\"");
            for ch in value.chars() {
                match ch {
                    '"' => output.push_str("\\\""),
                    '\\' => output.push_str("\\\\"),
                    ch if ch < ' ' => output.push_str(&format!("\\u{:04x}", u32::from(ch))),
                    ch => output.push(ch),
                }
            }
            output.push('"');
            output
        }
        let hook_levels = self
            .hook_levels
            .iter()
            .map(|(level, count)| {
                format!(
                    r#"{{"handle":{},"calls":{count}}}"#,
                    text(&format!("{level:?}"))
                )
            })
            .collect::<Vec<_>>()
            .join(",");
        let owners = self.owners.iter().map(|owner| {
            let sample = owner.world.and_then(|world| world.sample(host).ok());
            let pose = sample.as_ref().map_or_else(|| host_pose(&owner.motion), |sample| sample.current);
            let data = |position| host.block_entity(owner.source, position).and_then(|entity| entity.data(host));
            let find_entity = |name:&str| owner.world.and_then(|world|world.cells(host).ok()).and_then(|cells| cells.into_iter().find(|cell|host.block_states().name(cell.state.block()).ok() == Some(name))).and_then(|cell| data(cell.pos));
            let chest = find_entity("minecraft:chest");
            let furnace = find_entity("minecraft:furnace");
            let mut base = format!(r#"{{"owner":{},"active":{},"initialization_state":"{}","source_hooks":{},"source_game_time":{},"parent_game_time":{},"generation":{},"native_body":{},"pose_revision":{},"translation":{:?},"pivot":{:?},"rotation":{:?},"linear":{:?},"angular":{:?},"chest_items":{},"furnace_items":{},"furnace_cooking":{},"furnace_cooking_total":{},"furnace_lit":{},"furnace_input_items":{},"furnace_fuel_items":{},"furnace_output_items":{}}}"#,
                owner.mode, owner.active, if owner.waiting.is_some_and(|wait|matches!(wait,StartupWait::RetirementReceipt)) { "awaiting_empty_retirement_receipt" } else if owner.initialized { if owner.active { "ready" } else { "inactive" } } else if self.failed { "failed" } else { owner.waiting.map_or("preparing", StartupWait::name) }, owner.source_hooks, json(owner.source.game_time(host)), json(owner.parent.game_time(host)), owner.world.map_or(0, |w| w.generation()), self.replica.body(owner.id).map_or(0, |body| body.0), sample.map_or(0, |s| s.revision), pose.translation, pose.pivot, pose.rotation, owner.motion.linear, owner.motion.angular,
                json(chest.as_ref().and_then(items)), json(furnace.as_ref().and_then(items)), json(furnace.as_ref().and_then(|d| integer(d, "cooking_time_spent"))), json(furnace.as_ref().and_then(|d| integer(d, "cooking_total_time"))), json(furnace.as_ref().and_then(|d| integer(d, "lit_time_remaining"))),
                json(furnace.as_ref().and_then(|d| items_in(d, Some(0)))), json(furnace.as_ref().and_then(|d| items_in(d, Some(1)))), json(furnace.as_ref().and_then(|d| items_in(d, Some(2)))));
            base.pop();
            let identity = owner.record.as_ref().map(|record| format!(r#"{{"id":"{}","plot":{},"generation":{},"phase":"{}","profile":"{:?}"}}"#,record.logical.id.0.iter().map(|v|format!("{v:02x}")).collect::<String>(),record.logical.plot,record.logical.generation,match record.phase {Phase::OwnedUnprojected{..}=>"owned_unprojected",Phase::Projected{..}=>"projected",Phase::Returned{..}=>"returned",Phase::Retiring{..}=>"retiring",Phase::Retired{..}=>"retired"},record.episode.profile)).unwrap_or_else(||"null".into());
            let member_count=owner.world.map(|world|world.cells(host).ok().map(|cells|cells.len())).unwrap_or(Some(0)).map(|count|count.to_string()).unwrap_or_else(||"null".into());
            let revisions = owner.source.revisions(host).ok().map(|r|format!(r#"{{"block":{},"content":{}}}"#,r.block_revision,r.content_revision)).unwrap_or_else(||"null".into());
            base.push_str(&format!(r#", "logical":{},"revisions":{},"member_count":{},"native_realizations":{},"last_integrated_body":{}"#,identity,revisions,member_count,owner.native_realizations,owner.last_integrated.map(|(body,revision)|format!(r#"{{"body":{body},"block_revision":{revision}}}"#)).unwrap_or_else(||"null".into())));
            let columns = owner.members.iter().map(|p| [p[0].div_euclid(16),p[2].div_euclid(16)]).collect::<BTreeSet<_>>().into_iter().map(|position| {
                let chunk = owner.source.chunk_ref(host,position);
                format!(r#"{{"position":{position:?},"handle_present":{},"loaded":{},"forced":{}}}"#, chunk.is_some(), boolean(chunk.map(|chunk|chunk.loaded(host))), boolean(chunk.and_then(|chunk|chunk.forced(host))))
            }).collect::<Vec<_>>().join(",");
            base.push_str(&format!(r#","source_handle":{},"initial_setup_recovered":{},"matched_source_level_hooks":{},"initialization_attempts":{},"source_force_calls":{},"source_force_changed":{},"source_in_place_at_last_hook":{},"source_ticking_at_last_attempt":{},"source_columns":[{}]"#,
                text(&format!("{:?}",owner.source)),owner.recovered_initial,owner.diagnostics.level_hooks,owner.diagnostics.attempts,owner.diagnostics.force_calls,owner.diagnostics.force_changes,boolean(owner.diagnostics.in_place),boolean(owner.diagnostics.ticking),columns));
            let footprint = owner.footprint.map(|f| format!(r#"{{"cells":{},"source_chunks":{},"retained_columns":{},"view_areas":{}}}"#, f.cells,f.source_chunks,f.retained_columns,f.view_areas)).unwrap_or_else(|| "null".into());
            base.push_str(&format!(r#","source_footprint":{},"bootstrap_tickets_owned":{},"bootstrap_tickets_released":{}"#, footprint,owner.bootstrap_forced.len(),owner.bootstrap_released));
            let readback = owner.initial_native_readback.as_ref().map(NativeReadback::json).unwrap_or_else(|| "null".into());
            base.push_str(&format!(r#","initial_native_readback":{}"#,readback));
            let saved_descriptor=owner.saved_descriptor.as_ref().map(|saved|format!(r#"{{"destination":{},"member_cells":{},"retained_columns":{},"source_chunks":{:?},"barrier_epoch":{}}}"#,text(&saved.destination),saved.member_cells,saved.retained_columns,saved.source_chunks,saved.barrier_epoch)).unwrap_or_else(||"null".into());
            base.push_str(&format!(r#","saved_projection":{},"saved_record_migrated":{},"bootstrap_from_saved_record":{}"#,saved_descriptor,owner.migrated_saved_record,owner.saved_descriptor.is_some()));
            let native = self.replica.native_sample(owner.id).ok();
            let native_pose = native.as_ref().map(|(pose,_)| format!("{pose:?}")).unwrap_or_else(|| "null".into());
            let native_velocity = native.as_ref().map(|(_,velocity)| format!("{velocity:?}")).unwrap_or_else(|| "null".into());
            let native_pose_bits = native.as_ref().map(|(pose,_)| bits(pose)).unwrap_or_else(|| "null".into());
            let native_velocity_bits = native.as_ref().map(|(_,velocity)| bits(velocity)).unwrap_or_else(|| "null".into());
            format!(r#"{base},"host_bound":{},"logical_host_mapping_confirmed":{},"native_feedback_confirmed":{},"host_bits":{{"translation":{},"pivot":{},"rotation":{}}},"raw_motion":{{"position":{:?},"pivot":{:?},"rotation":{:?},"position_bits":{},"pivot_bits":{},"rotation_bits":{},"linear_bits":{},"angular_bits":{}}},"native":{{"pose":{},"velocity":{},"pose_bits":{},"velocity_bits":{}}}}}"#,
                sample.is_some(), sample.is_some() && frames::equivalent(&owner.motion,pose), owner.last_integrated.is_some_and(|(integrated,_)|self.replica.body(owner.id).is_some_and(|body|body.0==integrated)) && !self.failed && native.is_some() && sample.is_some(), bits(&pose.translation), bits(&pose.pivot), bits(&pose.rotation),
                owner.motion.pose.position, owner.motion.pose.pivot, owner.motion.pose.rotation,
                bits(&owner.motion.pose.position), bits(&owner.motion.pose.pivot), bits(&owner.motion.pose.rotation), bits(&owner.motion.linear), bits(&owner.motion.angular),
                native_pose,native_velocity,native_pose_bits,native_velocity_bits)
        }).collect::<Vec<_>>().join(",");
        let receipts = self
            .checkpoint_receipts
            .iter()
            .map(|receipt| {
                let retired=receipt.retired_generation.map_or_else(|| "null".into(),|generation|generation.to_string());
                format!(r#"{{"owner":{},"barrier_epoch":{},"generation":{},"retired_generation":{},"pose_revision":{},"source_revision":{},"content_revision":{}}}"#,
                    receipt.owner,receipt.barrier_epoch,receipt.generation,retired,receipt.pose_revision,receipt.source_revision,receipt.content_revision)
            })
            .collect::<Vec<_>>()
            .join(",");
        let reset_receipts=self.reset_receipts.iter().map(|(owner,before,after)|format!(r#"{{"owner":{},"before_generation":{},"after_generation":{},"source_unchanged":true}}"#,owner,before,after)).collect::<Vec<_>>().join(",");
        let reset_error = self
            .reset_error
            .as_ref()
            .map_or_else(|| "null".into(), |problem| text(problem));
        let limits = review_limits(if self.dynamic { 3 } else { 0 });
        let mut output = format!(
            r#"{{"schema":"sable.phase3.status","native_epoch":{},"process_native_publications":{},"observed_level_hooks":[{}],"paused":{},"failed":{},"last_host_world_running":{},"bodies":{},"materials":{},"review_reservations":{{"cells":{},"boxes":{},"chunks":{}}},"requested_publication_limits":{{"translation":{},"rotation_degrees":{},"motion_steps":{}}},"free_dynamics":{},"explicit_initial_recovery":{},"reset_pending":{},"stale_handle_refused":{},"impulse_pending":{},"gravity":{:?},"dt":0.05,"substeps":2,"collision_count":{},"last_collision":{:?},"checkpoint":{{"pending":{},"requested":{},"attempted":{},"completed":{},"failed":{},"native_epoch":{},"paused":{},"host_receipts":[{}]}},"owners":[{}]}}"#,
            self.ticks,
            self.process_native_publications,
            hook_levels,
            self.paused,
            self.failed,
            self.world_running,
            self.replica.counts().0,
            self.replica.material_count(),
            limits.cells,
            limits.boxes,
            limits.chunks,
            limits.translation,
            limits.rotation_degrees,
            limits.motion_steps,
            self.dynamic,
            self.recover_initial_allowed,
            self.reset_requested,
            self.stale_handle_refused,
            self.pending_impulse,
            if self.dynamic {
                [0., -9.81, 0.]
            } else {
                [0.; 3]
            },
            self.collision_count,
            self.last_collision,
            self.checkpoint_requested,
            self.checkpoint_requests,
            self.checkpoint_attempts,
            self.checkpoint_completed,
            self.checkpoint_failed,
            self.checkpoint_epoch,
            self.checkpoint_paused,
            receipts,
            owners
        );
        output.pop();
        format!(
            r#"{output},"reset_receipts":[{reset_receipts}],"reset_error":{reset_error},"migration_only":{}}}"#,
            self.migrate_saved_allowed
        )
    }
    /// Explicit native torque impulse for the paused R5 review. The actual
    /// body's mass must have completed an integration; this preserves the
    /// original solver's lazy mass-property behavior.
    pub(crate) fn native_impulse(
        &mut self,
        host: &mut dyn ServerHost,
        number: u8,
        input: NativeImpulse,
    ) -> Result<String, String> {
        if self.dynamic
            || self.failed
            || !self.paused
            || !self.epoch_restored
            || self.ticks == 0
            || self.ticks >= 200
            || self.migrate_saved_allowed
        {
            return Err(
                "native impulse requires a healthy paused episode after an actual native step"
                    .into(),
            );
        }
        let owner = self
            .owners
            .iter()
            .find(|owner| owner.mode == number && owner.active)
            .ok_or_else(|| "spin owner unavailable".to_string())?;
        let world = owner
            .world
            .ok_or_else(|| "spin binding unavailable".to_string())?;
        let revision = world
            .cells(host)
            .map_err(error)?
            .iter()
            .map(|cell| cell.revision)
            .max()
            .ok_or_else(|| "spin cannot target empty geometry".to_string())?;
        let body = self
            .replica
            .body(owner.id)
            .ok_or_else(|| "spin native body unavailable".to_string())?;
        if owner.last_integrated != Some((body.0, revision)) {
            return Err(
                "spin waits for an actual integration of this exact body/geometry revision".into(),
            );
        }
        let id = owner.id;
        let mut catalog = PropertyCatalog::new(1).map_err(error)?;
        let snapshots = self
            .owners
            .iter()
            .filter(|owner| owner.active)
            .map(|owner| {
                Self::observe(host, owner, &mut catalog)?
                    .ok_or_else(|| "spin scene has empty pending geometry".to_string())
            })
            .collect::<Result<Vec<_>, String>>()?;
        let masses = catalog
            .mass_registry(
                1,
                snapshots
                    .iter()
                    .flat_map(|s| s.cells.values())
                    .map(String::as_str),
            )
            .map_err(error)?;
        let snapshot = snapshots
            .iter()
            .find(|snapshot| snapshot.id == id)
            .ok_or_else(|| "native impulse source snapshot missing".to_string())?;
        let contents = Contents {
            blocks: snapshot
                .cells
                .iter()
                .map(|(p, key)| {
                    (
                        *p,
                        Block {
                            state: key.clone(),
                            block_entity: None,
                            extension: Payload::default(),
                        },
                    )
                })
                .collect(),
            ..Contents::default()
        };
        let mass = masses.compute(&contents).map_err(error)?.mass;
        let (_, before_velocity) = self.replica.native_sample(id).map_err(error)?;
        let speed = input.linear_speed();
        if speed.is_some() && before_velocity.iter().any(|value| *value != 0.) {
            return Err("drift requires an actually stationary native body; it never resets existing velocity".into());
        }
        let linear_impulse = speed.map_or([0.; 3], |speed| [mass * speed, 0., 0.]);
        let angular_impulse = if speed.is_some() {
            [0.; 3]
        } else {
            [0., 2., 0.]
        };
        let result = (|| -> Result<[f64; 6], String> {
            self.replica
                .impulse(id, linear_impulse, angular_impulse, true)
                .map_err(error)?;
            let motions = self.replica.capture(&masses).map_err(error)?;
            // Impulse never authorizes a host pose move. Persist only the
            // genuine velocities against the accepted canonical sample.
            self.accept_batch(host, &motions, self.ticks, true, false)?;
            self.replica
                .native_sample(id)
                .map(|(_, velocity)| velocity)
                .map_err(error)
        })();
        let after_velocity = match result {
            Ok(velocity) => velocity,
            Err(problem) => {
                self.failed = true;
                self.paused = true;
                return Err(problem);
            }
        };
        let kind = input.name();
        let requested_speed = speed
            .map(|value| value.to_string())
            .unwrap_or_else(|| "null".into());
        let receipt = format!(
            r#"{{"kind":"{kind}","requested_linear_speed":{requested_speed},"owner":{number},"native_epoch":{},"body":{},"geometry_revision":{revision},"mass":{mass},"linear_impulse":{linear_impulse:?},"angular_impulse":{angular_impulse:?},"before_velocity":{before_velocity:?},"after_velocity":{after_velocity:?},"before_velocity_bits":{},"after_velocity_bits":{},"host_metadata_confirmed":true}}"#,
            self.ticks,
            body.0,
            bits(&before_velocity),
            bits(&after_velocity)
        );
        eprintln!("[sable_phase3] explicit native impulse {receipt}");
        Ok(receipt)
    }
    pub(crate) fn open_demo_status(&self, host: &dyn ServerHost) -> String {
        fn pose(value: LiveWorldPose) -> String {
            format!(
                "{{\"translation\":{:?},\"pivot\":{:?},\"rotation\":{:?}}}",
                value.translation, value.pivot, value.rotation
            )
        }
        let Some(profile) = &self.open_profile else {
            return "{\"open_demo\":false}".into();
        };
        render_step_trace("status_begin", self.ticks, None);
        let owner = &self.owners[0];
        let sample = owner.world.and_then(|world| world.sample(host).ok());
        render_step_trace("status_host_sample_end", self.ticks, None);
        let native = self.replica.native_sample(owner.id).ok();
        render_step_trace("status_native_sample_end", self.ticks, None);
        let elapsed = profile.finished_elapsed.or_else(|| {
            profile
                .released_at
                .map(|start| start.elapsed().as_secs_f64())
        });
        let elapsed_json = elapsed.map_or_else(|| "null".into(), |value| value.to_string());
        let epoch_rate = elapsed
            .filter(|value| *value > 0.)
            .map(|value| (self.ticks.saturating_sub(profile.release_epoch)) as f64 / value);
        let number = |value: Option<u64>| value.map_or_else(|| "null".into(), |v| v.to_string());
        let first = format!(
            "[{},{},{}]",
            number(profile.first_ledge_epoch[0]),
            number(profile.first_ledge_epoch[1]),
            number(profile.first_ledge_epoch[2])
        );
        let reply = format!(
            concat!(
                "{{\"schema\":\"sable.open-demo.status.v1\",\"open_demo\":true,\"native_epoch\":{},\"paused\":{},\"failed\":{},",
                "\"initialized\":{},\"active\":{},\"world_running\":{},\"reset_pending\":{},",
                "\"dt\":0.05,\"substeps\":2,\"gravity\":[0,-9.81,0],\"epoch_policy\":\"{}\",\"epoch_limit\":{},\"requested_epoch_limit\":400,\"policy_reset_required\":{},",
                "\"pose\":{},\"velocity\":{},\"world_current\":{},\"world_previous\":{},\"world_revision\":{},",
                "\"collision_count\":{},\"last_collision\":{:?},",
                "\"telemetry\":{{\"sampling\":{},\"native\":{},\"publish\":{},\"release_elapsed_s\":{},\"release_epoch_rate\":{},",
                "\"impulse_epoch\":{},\"ledge_cells\":{},\"ledge_contacts\":{:?},\"first_ledge_epoch\":{},\"peak_ledge_force\":{:?}}}}}"
            ),
            self.ticks,
            self.paused,
            self.failed,
            owner.initialized,
            owner.active,
            self.world_running,
            self.reset_requested,
            owner.epoch_policy.name(),
            owner.epoch_policy.limit(),
            owner.epoch_policy != persist::EpochPolicy::OpenDemo,
            native.map_or_else(|| "null".into(), |(pose, _)| format!("{pose:?}")),
            native.map_or_else(|| "null".into(), |(_, velocity)| format!("{velocity:?}")),
            sample
                .as_ref()
                .map_or_else(|| "null".into(), |sample| pose(sample.current)),
            sample
                .as_ref()
                .map_or_else(|| "null".into(), |sample| pose(sample.previous)),
            number(sample.as_ref().map(|sample| sample.revision)),
            self.collision_count,
            self.last_collision,
            profile.sampling.json(),
            profile.native.json(),
            profile.publication.json(),
            elapsed_json,
            epoch_rate.map_or_else(|| "null".into(), |value| value.to_string()),
            number(profile.impulse_epoch),
            profile
                .ledge_sections
                .iter()
                .map(|section| section.cells.len())
                .sum::<usize>(),
            profile.ledge_contacts,
            first,
            profile.peak_ledge_force
        );
        render_step_trace("status_end", self.ticks, None);
        reply
    }
    pub(crate) fn open_demo_control(
        &mut self,
        host: &mut dyn ServerHost,
        action: &str,
    ) -> Result<(), String> {
        if self.open_profile.is_none() {
            return Err("not an open demo scene".into());
        }
        if matches!(action, "status" | "checkpoint") {
            return self.control(host, action);
        }
        if !matches!(action, "start" | "reset") {
            return Err("open demo supports status, start, reset and checkpoint".into());
        }
        if !self.epoch_restored
            || self
                .owners
                .iter()
                .any(|owner| !owner.initialized || !owner.active || owner.world.is_none())
        {
            return Err("open demo waits for its actual restored Source and live binding".into());
        }
        let mut catalog = PropertyCatalog::new(1).map_err(error)?;
        let ledges = Self::open_ledge_terrain(host, self.owners[0].parent, &mut catalog)?;
        if action == "reset" {
            if self.reset_requested {
                return Err("reset already pending".into());
            }
            self.open_demo_reset_observers_clear(host)?;
            return self.control(host, "reset");
        }
        if self.owners[0].epoch_policy != persist::EpochPolicy::OpenDemo {
            return Err("existing review-v3 Open Source requires explicit reset to OpenDemo-v4 before start".into());
        }
        if self.failed || !self.paused || self.ticks != 0 || self.pending_impulse {
            return Err(
                "open demo start requires healthy paused native epoch zero; reset first".into(),
            );
        }
        let motions = self
            .owners
            .iter()
            .map(|owner| (owner.id, owner.motion.clone()))
            .collect();
        self.accept_batch(host, &motions, 0, false, false)?;
        self.paused = false;
        self.step_once = false;
        self.pending_impulse = true;
        let profile = self.open_profile.as_mut().unwrap();
        *profile = OpenProfile {
            ledge_sections: ledges,
            released_at: Some(Instant::now()),
            ..OpenProfile::default()
        };
        Ok(())
    }
    pub(crate) fn control(
        &mut self,
        host: &mut dyn ServerHost,
        action: &str,
    ) -> Result<(), String> {
        if matches!(action, "resume" | "release" | "step" | "impulse")
            && self.owners.iter().all(|owner| !owner.active)
        {
            return Err(
                "no active native owner; control never recreates deleted structures".into(),
            );
        }
        if self.migrate_saved_allowed && !matches!(action, "status" | "checkpoint") {
            return Err("migration-only launch does not advance/reset native physics".into());
        }
        if !matches!(action, "status" | "checkpoint")
            && (!self.epoch_restored
                || self
                    .owners
                    .iter()
                    .any(|owner| !owner.initialized || (owner.active && owner.world.is_none())))
        {
            return Err(
                "physical control waits for coherent restoration and actual host bindings".into(),
            );
        }
        match action {
            "status" => return Ok(()),
            "checkpoint" => {
                if self.checkpoint_requested {
                    return Err("checkpoint already pending".into());
                }
                self.checkpoint_requests = self
                    .checkpoint_requests
                    .checked_add(1)
                    .ok_or_else(|| "checkpoint serial exhausted".to_string())?;
                self.checkpoint_requested = true;
                return Ok(());
            }
            "pause" => self.paused = true,
            "reset" => {
                self.reset_receipts.clear();
                self.reset_error = None;
                self.reset_requested = true;
                self.paused = true;
            }
            "impulse"
                if self.dynamic
                    && !self.failed
                    && self.ticks > 0
                    && self.ticks < 200
                    && !self.pending_impulse =>
            {
                self.pending_impulse = true;
                return Ok(());
            }
            "resume" | "release" | "step"
                if !self.failed && self.ticks < 200 && (action != "release" || self.dynamic) =>
            {
                self.paused = false;
                self.step_once = action == "step";
            }
            _ => {
                return Err(
                    "unknown command, failed session, or bounded native budget exhausted".into(),
                );
            }
        }
        if self.failed {
            for owner in &mut self.owners {
                owner.paused = self.paused;
            }
        } else {
            let motions = self
                .owners
                .iter()
                .filter(|owner| owner.active)
                .map(|owner| (owner.id, owner.motion.clone()))
                .collect();
            if let Err(problem) = self.accept_batch(host, &motions, self.ticks, self.paused, false)
            {
                self.failed = true;
                self.paused = true;
                return Err(problem);
            }
        }
        eprintln!(
            "[sable_phase3] native control {action} at epoch {}",
            self.ticks
        );
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn open_epoch_policy_never_expands_historical_review_sources() {
        assert!(epoch_policy_source_allowed(
            OPEN_SOURCE,
            persist::EpochPolicy::OpenDemo
        ));
        assert!(epoch_policy_source_allowed(
            OPEN_SOURCE,
            persist::EpochPolicy::Review
        ));
        for source in [
            "sable_port:review/translate",
            "sable_port:review/yaw",
            "sable_port:review/pitch",
            "sable_port:review/dynamic",
        ] {
            assert!(epoch_policy_source_allowed(
                source,
                persist::EpochPolicy::Review
            ));
            assert!(!epoch_policy_source_allowed(
                source,
                persist::EpochPolicy::OpenDemo
            ));
        }
    }

    #[test]
    fn open_demo_fixture_retains_the_native_probe_mass_center_and_real_ledges() {
        let cells = open_cells();
        let center: [f64; 3] = std::array::from_fn(|axis| {
            cells
                .iter()
                .map(|p| f64::from(p[axis] - OPEN_ORIGIN[axis]) + 0.5)
                .sum::<f64>()
                / cells.len() as f64
        });
        assert_eq!(cells.len(), 7);
        assert_eq!(center, [1.5, 0.6428571428571429, 0.7857142857142857]);
        assert_eq!(open_ledges().len(), 18);
        assert_eq!(open_ledges().into_iter().collect::<BTreeSet<_>>().len(), 18);
        assert_eq!(open_ledges()[0], [4, 106, -2]);
        assert_eq!(open_ledges()[6], [1, 99, -2]);
        assert_eq!(open_ledges()[12], [2, 96, -2]);
        // The accepted old seven-cell fixture remains independent.
        assert!(dynamic_cells().contains(&[0, 0, 2]));
        assert!(!cells.contains(&[OPEN_ORIGIN[0], OPEN_ORIGIN[1], OPEN_ORIGIN[2] + 2]));
    }

    #[test]
    fn open_demo_sweep_contains_intermediate_rotations_and_velocity_endpoint() {
        let center = [1033.5, 128.64285714285714, 1032.7857142857142];
        let mut snapshot = GeometrySnapshot {
            id: Id([5; 16]),
            generation: 1,
            revision: 0,
            plot: 4,
            cells: open_cells()
                .into_iter()
                .map(|p| (p, "minecraft:blackstone".into()))
                .collect(),
            motion: Motion {
                pose: Pose {
                    position: [1.5, 114.64285714285714, 0.7857142857142857],
                    pivot: center,
                    ..Pose::default()
                },
                linear: [2., -25., 0.],
                ..Motion::default()
            },
        };
        let bounds = open_swept_bounds(&snapshot).unwrap();
        for degrees in [0f64, 17., 46., 90.] {
            let angle = degrees.to_radians() * 0.5;
            for end in [false, true] {
                let mut pose = snapshot.motion.pose;
                pose.rotation = [0., 0., angle.sin(), angle.cos()];
                if end {
                    for axis in 0..3 {
                        pose.position[axis] += snapshot.motion.linear[axis] * 0.05;
                    }
                }
                for p in snapshot.cells.keys() {
                    for corner in 0..8 {
                        let world = pose.transform(std::array::from_fn(|axis| {
                            f64::from(p[axis]) + if corner & (1 << axis) == 0 { 0. } else { 1. }
                        }));
                        assert!(
                            (0..3).all(|axis| world[axis] >= bounds[axis]
                                && world[axis] <= bounds[axis + 3])
                        );
                    }
                }
            }
        }
        snapshot.motion.linear = [1e6; 3];
        assert!(open_swept_bounds(&snapshot).is_err());
    }

    #[test]
    fn open_demo_contacts_use_the_actual_static_body_point_on_either_side() {
        let mut row = [0.; 15];
        row[0] = -1.;
        row[1] = 4.;
        row[9..12].copy_from_slice(&[4.5, 107., 0.5]);
        assert_eq!(open_ledge_contact(&row), Some(0));
        row[0] = 4.;
        row[1] = -1.;
        row[12..15].copy_from_slice(&[1.5, 100., 0.5]);
        assert_eq!(open_ledge_contact(&row), Some(1));
        row[12..15].copy_from_slice(&[8., 65., 1.]);
        assert_eq!(open_ledge_contact(&row), None);
        row[1] = 5.;
        assert_eq!(open_ledge_contact(&row), None);
    }

    fn repair785_fixture() -> (Vec<OwnerState>, sable_core::host_lifecycle::PersistentState) {
        use owner_state::{Episode, RawState};
        use sable_core::host_lifecycle::{LogicalIdentity, PersistentState};
        let motion = |bits: [u64; 16]| Motion {
            pose: Pose {
                position: std::array::from_fn(|i| f64::from_bits(bits[i])),
                pivot: std::array::from_fn(|i| f64::from_bits(bits[i + 3])),
                rotation: std::array::from_fn(|i| f64::from_bits(bits[i + 6])),
                scale: [1.; 3],
            },
            linear: std::array::from_fn(|i| f64::from_bits(bits[i + 10])),
            angular: std::array::from_fn(|i| f64::from_bits(bits[i + 13])),
        };
        let mut records = Vec::new();
        for (index, id, plot, generation, profile, bits, epoch) in [
            (
                0,
                REPAIR_PARENT,
                0,
                4,
                Profile::Split,
                REPAIR_PARENT_MOTION,
                1,
            ),
            (
                1,
                REPAIR_CHILD,
                128,
                2,
                Profile::Split,
                REPAIR_CHILD_MOTION,
                1,
            ),
            (
                2,
                Id([3; 16]),
                256,
                1,
                Profile::Pitch,
                REPAIR_PARENT_MOTION,
                1,
            ),
            (
                3,
                Id([4; 16]),
                384,
                1,
                Profile::Dynamic,
                REPAIR_PARENT_MOTION,
                0,
            ),
        ] {
            let mut cells = BTreeMap::new();
            if index < 2 {
                for x in if index == 0 { 1032..=1033 } else { 1035..=1065 } {
                    for z in 1032..=1035 {
                        if [x, 128, z] != [1049, 128, 1034] {
                            cells.insert([x, 128, z], "minecraft:blackstone".to_string());
                        }
                    }
                }
                if index == 1 {
                    for (position, key) in [
                        (
                            [1035, 129, 1033],
                            "minecraft:chest[facing=north,type=single,waterlogged=false]",
                        ),
                        (
                            [1037, 129, 1033],
                            "minecraft:furnace[facing=north,lit=false]",
                        ),
                        (
                            [1040, 129, 1033],
                            "minecraft:stone_slab[type=bottom,waterlogged=false]",
                        ),
                    ] {
                        cells.insert(position, key.into());
                    }
                }
            }
            let phase = if index < 2 {
                Phase::OwnedUnprojected {
                    barrier_epoch: 785,
                    members: cells.keys().copied().collect(),
                }
            } else {
                Phase::Projected {
                    pose_revision: 1,
                    pose: LiveWorldPose {
                        translation: [0.; 3],
                        pivot: [0.; 3],
                        rotation: [0., 0., 0., 1.],
                    },
                }
            };
            let current = motion(bits);
            records.push(OwnerState {
                logical: LogicalIdentity {
                    id,
                    plot,
                    generation,
                    parent: (index == 1).then_some(REPAIR_PARENT),
                },
                current: RawState {
                    motion: current.clone(),
                    native_epoch: epoch,
                    paused: true,
                },
                episode: Episode {
                    profile,
                    initial: current,
                    return_offset: [-1032, -104, -1032],
                    cells,
                },
                phase,
            });
        }
        let registry = PersistentState {
            owners: records.iter().map(|record| record.logical).collect(),
            plot_generations: records
                .iter()
                .map(|record| (record.logical.plot, record.logical.generation))
                .collect(),
        };
        (records, registry)
    }

    #[test]
    fn explicit_repair785_admits_only_pinned_committed_afterimage() {
        let (records, registry) = repair785_fixture();
        repair785_records(&records, &registry).unwrap();
        let mut old_barrier = records.clone();
        if let Phase::OwnedUnprojected { barrier_epoch, .. } = &mut old_barrier[0].phase {
            *barrier_epoch = 784;
        }
        let mut wrong_parent = records.clone();
        wrong_parent[1].logical.parent = None;
        let mut raw_bit = records.clone();
        raw_bit[0].current.motion.linear[0] = f64::from_bits(REPAIR_PARENT_MOTION[10] ^ 1);
        let mut old_epoch = records.clone();
        old_epoch[0].current.native_epoch = 0;
        let mut old_initial = records.clone();
        old_initial[1].episode.initial = records[0].current.motion.clone();
        let mut wrong_palette = records.clone();
        wrong_palette[1].episode.cells.insert(
            [1037, 129, 1033],
            "minecraft:furnace[facing=south,lit=false]".into(),
        );
        let mut lost_member = records.clone();
        lost_member[1].episode.cells.remove(&[1035, 129, 1033]);
        for changed in [
            old_barrier,
            wrong_parent,
            raw_bit,
            old_epoch,
            old_initial,
            wrong_palette,
            lost_member,
        ] {
            assert!(repair785_records(&changed, &registry).is_err());
        }
        let mut wrong_registry = registry.clone();
        wrong_registry.plot_generations.insert(0, 5);
        assert!(repair785_records(&records, &wrong_registry).is_err());
    }

    #[test]
    fn explicit_repair785_promotion_changes_only_parent_phase() {
        let (records, registry) = repair785_fixture();
        let mut promoted = records[0].clone();
        promoted.phase = Phase::Projected {
            pose_revision: 1,
            pose: LiveWorldPose {
                translation: [0.; 3],
                pivot: [0.; 3],
                rotation: [0., 0., 0., 1.],
            },
        };
        assert_eq!(
            repair_motion_bits(&promoted.current.motion),
            REPAIR_PARENT_MOTION
        );
        assert_eq!(promoted.current.native_epoch, 1);
        promoted.phase = records[0].phase.clone();
        assert!(owner_state::same_record(&promoted, &records[0]));
        assert!(matches!(
            records[1].phase,
            Phase::OwnedUnprojected {
                barrier_epoch: 785,
                ..
            }
        ));
        repair785_records(&records, &registry).unwrap();
    }

    // Typed receipt fixtures only. These synthetic handles never reach a host
    // operation and do not stand in for the actual normal-reopen regression.
    fn retirement_source() -> LevelRef {
        LevelRef::with_generation(WorldEpoch::new(9), DimensionKey::OVERWORLD, 3)
    }
    fn retirement_receipt() -> LiveWorldSaveReceipt {
        LiveWorldSaveReceipt::new(
            171,
            vec![LiveWorldLevelReceipt::new(
                retirement_source(),
                20,
                30,
                None,
                Some(71),
                Some(0),
            )],
        )
    }
    fn check_retirement_fixture(
        phase: &Phase,
        descriptor: Result<LiveWorldSavedProjection, LiveWorldError>,
        receipt: Result<LiveWorldSaveReceipt, LiveWorldError>,
    ) -> Result<Option<LiveWorldLevelReceipt>, String> {
        let source = retirement_source();
        inactive_retirement_completion(source, phase, Some(source), Ok(None), descriptor, || {
            receipt
        })
    }

    #[test]
    fn inactive_startup_accepts_only_completed_current_empty_retirement_receipts() {
        for phase in [
            Phase::Returned { barrier_epoch: 171 },
            Phase::Retired {
                barrier_epoch: 170,
                generation: 71,
            },
            Phase::Retiring {
                generation: 71,
                members: vec![[15, 0, 0], [16, 0, 0]],
            },
        ] {
            // Stale is the actual saved descriptor response for a validated
            // tombstone whose binding is None. It is insufficient on its own.
            let confirmed = check_retirement_fixture(
                &phase,
                Err(LiveWorldError::Stale),
                Ok(retirement_receipt()),
            )
            .unwrap()
            .unwrap();
            assert_eq!(confirmed.level, retirement_source());
            assert_eq!(confirmed.retired_generation, Some(71));
            assert_eq!(confirmed.retired_members, Some(0));
            assert!(confirmed.binding.is_none());
            assert_eq!(confirmed.block_revision, 20);
            assert_eq!(confirmed.content_revision, 30);
        }
        // The previous NotSaved domain also requires an actual retired receipt;
        // it never becomes permission to seed, recover or assume an empty world.
        assert!(
            check_retirement_fixture(
                &Phase::Returned { barrier_epoch: 171 },
                Err(LiveWorldError::NotSaved),
                Ok(retirement_receipt()),
            )
            .unwrap()
            .is_some()
        );
    }

    #[test]
    fn inactive_startup_refuses_missing_old_or_nonempty_retirement_coverage() {
        let source = retirement_source();
        let phase = Phase::Returned { barrier_epoch: 171 };
        let valid = retirement_receipt();
        let mut variants = vec![
            LiveWorldSaveReceipt::new(0, valid.levels.clone()),
            LiveWorldSaveReceipt::new(170, valid.levels.clone()),
            LiveWorldSaveReceipt::new(171, vec![]),
            LiveWorldSaveReceipt::new(171, vec![valid.levels[0].clone(), valid.levels[0].clone()]),
        ];
        let mut foreign = valid.clone();
        foreign.levels[0].level = LevelRef::with_generation(source.epoch(), source.dimension(), 4);
        variants.push(foreign);
        let mut active = valid.clone();
        active.levels[0].binding = Some(LiveWorldRef::new(source, 72));
        variants.push(active);
        for generation in [None, Some(0)] {
            let mut missing_generation = valid.clone();
            missing_generation.levels[0].retired_generation = generation;
            variants.push(missing_generation);
        }
        for members in [None, Some(1), Some(138)] {
            let mut nonempty = valid.clone();
            nonempty.levels[0].retired_members = members;
            variants.push(nonempty);
        }
        for receipt in variants {
            assert!(
                check_retirement_fixture(&phase, Err(LiveWorldError::Stale), Ok(receipt)).is_err()
            );
        }
        for phase in [
            Phase::Retired {
                barrier_epoch: 170,
                generation: 72,
            },
            Phase::Retiring {
                generation: 72,
                members: vec![],
            },
        ] {
            assert!(
                check_retirement_fixture(&phase, Err(LiveWorldError::Stale), Ok(valid.clone()),)
                    .is_err()
            );
        }
    }

    #[test]
    fn inactive_startup_defers_only_busy_checkpoint_and_keeps_host_errors() {
        let phase = Phase::Returned { barrier_epoch: 171 };
        assert_eq!(
            check_retirement_fixture(
                &phase,
                Err(LiveWorldError::Stale),
                Err(LiveWorldError::Busy),
            ),
            Ok(None)
        );
        for problem in [
            LiveWorldError::Stale,
            LiveWorldError::Storage,
            LiveWorldError::NotSaved,
            LiveWorldError::NotResident,
            LiveWorldError::Denied,
        ] {
            assert!(
                check_retirement_fixture(&phase, Err(LiveWorldError::Stale), Err(problem)).is_err()
            );
        }
        let source = retirement_source();
        for descriptor_error in [
            LiveWorldError::Storage,
            LiveWorldError::Denied,
            LiveWorldError::Busy,
            LiveWorldError::MigrationRequired,
        ] {
            assert!(
                inactive_retirement_completion(
                    source,
                    &phase,
                    Some(source),
                    Ok(None),
                    Err(descriptor_error),
                    || panic!("invalid descriptor must not start a checkpoint"),
                )
                .is_err()
            );
        }
        for binding in [
            Ok(Some(LiveWorldRef::new(source, 71))),
            Err(LiveWorldError::Stale),
            Err(LiveWorldError::Storage),
        ] {
            assert!(
                inactive_retirement_completion(
                    source,
                    &phase,
                    Some(source),
                    binding,
                    Err(LiveWorldError::Stale),
                    || panic!("unconfirmed Source binding must not start a checkpoint"),
                )
                .is_err()
            );
        }
        for current in [
            None,
            Some(LevelRef::with_generation(
                source.epoch(),
                source.dimension(),
                4,
            )),
        ] {
            assert!(
                inactive_retirement_completion(
                    source,
                    &phase,
                    current,
                    Ok(None),
                    Err(LiveWorldError::Stale),
                    || panic!("stale current Source must not start a checkpoint"),
                )
                .is_err()
            );
        }
    }

    #[test]
    fn inactive_startup_rejects_an_active_saved_descriptor_or_invalid_phase() {
        let source = retirement_source();
        let saved = LiveWorldSavedProjection::new(
            "minecraft:overworld".into(),
            vec![[0, 0]],
            1,
            81,
            LiveWorldLimits::default(),
            171,
            LiveWorldRecovery::Ready,
        );
        assert!(
            inactive_retirement_completion(
                source,
                &Phase::Returned { barrier_epoch: 171 },
                Some(source),
                Ok(None),
                Ok(saved),
                || panic!("active saved projection must not start a retirement checkpoint"),
            )
            .is_err()
        );
        for phase in [
            Phase::Returned { barrier_epoch: 0 },
            Phase::Retired {
                barrier_epoch: 0,
                generation: 71,
            },
            Phase::Retired {
                barrier_epoch: 171,
                generation: 0,
            },
            Phase::Retiring {
                generation: 0,
                members: vec![],
            },
            Phase::OwnedUnprojected {
                barrier_epoch: 171,
                members: vec![[0, 0, 0]],
            },
            Phase::Projected {
                pose_revision: 1,
                pose: LiveWorldPose::at([0.; 3]),
            },
        ] {
            assert!(
                inactive_retirement_completion(
                    source,
                    &phase,
                    Some(source),
                    Ok(None),
                    Err(LiveWorldError::Stale),
                    || panic!("invalid inactive phase must not start a checkpoint"),
                )
                .is_err()
            );
        }
    }

    #[test]
    fn gravity_publication_profile_covers_observed_contact_without_widening_controlled_steps() {
        // Real native inputs from the first 0013 floor impact (epochs 29/30).
        // This is a publication-profile regression, not a solver oracle.
        let pivot = [19. / 14., 9. / 14., 13. / 14.];
        let pose = |position: [f64; 3], rotation: [f64; 4]| LiveWorldPose {
            translation: std::array::from_fn(|axis| position[axis] - pivot[axis]),
            pivot,
            rotation,
        };
        let before = frames::canonical_at(
            pose(
                [13.357142448425293, 22.320171356201172, 25.128522872924805],
                [
                    0.1434757262468338,
                    -0.06239017844200134,
                    -0.027486996725201607,
                    0.9873027801513672,
                ],
            ),
            pivot,
        )
        .unwrap();
        let after = frames::canonical_at(
            pose(
                [13.379454612731934, 21.736141204833984, 25.065711975097656],
                [
                    0.05892923101782799,
                    -0.06981168687343597,
                    -0.04287087917327881,
                    0.9948948621749878,
                ],
            ),
            pivot,
        )
        .unwrap();
        let translation = before
            .translation
            .iter()
            .zip(after.translation)
            .map(|(a, b)| (b - a).abs())
            .fold(0., f64::max);
        let dot: f64 = before
            .rotation
            .iter()
            .zip(after.rotation)
            .map(|(a, b)| a * b)
            .sum();
        let degrees = (2. * dot.abs().min(1.).acos()).to_degrees();
        let dynamic = review_limits(3);
        assert!(translation < dynamic.translation);
        assert!(degrees < dynamic.rotation_degrees);
        assert_eq!(dynamic.translation, 4.);
        assert_eq!(dynamic.rotation_degrees, 90.);
        for mode in 0..3 {
            let controlled = review_limits(mode);
            assert_eq!(controlled.translation, 1.);
            assert_eq!(controlled.rotation_degrees, 5.);
            assert!(degrees > controlled.rotation_degrees);
            assert_eq!(dynamic.min, controlled.min);
            assert_eq!(dynamic.max, controlled.max);
            assert_eq!(dynamic.cells, controlled.cells);
            assert_eq!(dynamic.chunks, controlled.chunks);
            assert_eq!(dynamic.boxes, controlled.boxes);
            assert_eq!(dynamic.contacts, controlled.contacts);
            assert_eq!(dynamic.candidates, controlled.candidates);
            assert_eq!(dynamic.motion_steps, controlled.motion_steps);
            assert_eq!(dynamic.publication, controlled.publication);
        }
    }

    fn rearm_test_snapshot(
        number: u8,
        mutate: Option<fn(FixtureBlock, &mut sable_core::mass::Properties)>,
    ) -> (
        OriginalFixture,
        sable_core::host_lifecycle::StateSnapshot,
        PropertyCatalog,
    ) {
        let fixture = OriginalFixture::for_slot(number).unwrap();
        let mut catalog = PropertyCatalog::new(1).unwrap();
        let mut actual = sable_core::host_lifecycle::StateSnapshot {
            revision: 17,
            cells: BTreeMap::new(),
        };
        for (position, role) in &fixture.members {
            let definitions: Vec<(&str, Vec<&str>, &str)> = match role {
                FixtureBlock::Blackstone => Vec::new(),
                FixtureBlock::Chest => vec![
                    ("facing", vec!["north", "south", "east", "west"], "south"),
                    ("type", vec!["single", "left", "right"], "single"),
                    ("waterlogged", vec!["false", "true"], "false"),
                ],
                // A completed/cold furnace is valid. Real item/NBT contents
                // are deliberately not inputs to this state-only preflight.
                FixtureBlock::Furnace => vec![
                    ("facing", vec!["north", "south", "east", "west"], "west"),
                    ("lit", vec!["false", "true"], "false"),
                ],
                FixtureBlock::BottomSlab => vec![
                    ("type", vec!["bottom", "top", "double"], "bottom"),
                    ("waterlogged", vec!["false", "true"], "false"),
                ],
            };
            let state = StateDefinition::new(
                role.name().into(),
                definitions
                    .iter()
                    .map(|(name, allowed, _)| {
                        (
                            (*name).into(),
                            allowed.iter().map(|value| (*value).into()).collect(),
                        )
                    })
                    .collect(),
            )
            .unwrap()
            .resolve(
                definitions
                    .iter()
                    .map(|(name, _, value)| ((*name).into(), (*value).into()))
                    .collect(),
            )
            .unwrap();
            let key = state.key().to_string();
            if catalog.get(1, &key).is_err() {
                let mut properties = role.properties();
                if let Some(mutate) = mutate {
                    mutate(*role, &mut properties);
                }
                catalog
                    .register(
                        state,
                        HostGeometry {
                            state_invariant: true,
                            solid: properties.solid,
                            full: properties.full,
                            boxes: properties.boxes,
                        },
                        PhysicsPolicy {
                            rule: "explicit rearm preflight test facts".into(),
                            mass: properties.mass,
                            center: properties.center,
                            inertia: properties.inertia,
                        },
                    )
                    .unwrap();
            }
            actual.cells.insert(*position, key);
        }
        (fixture, actual, catalog)
    }

    #[test]
    fn queued_migration_retries_only_actual_frontier_or_residency_refusals() {
        assert!(matches!(
            migration_completion(Err(LiveWorldError::Busy)),
            Ok(None)
        ));
        assert!(matches!(
            migration_completion(Err(LiveWorldError::NotResident)),
            Ok(None)
        ));
        for error in [
            LiveWorldError::NotSaved,
            LiveWorldError::Storage,
            LiveWorldError::Stale,
            LiveWorldError::Denied,
            LiveWorldError::MigrationRequired,
        ] {
            assert!(migration_completion(Err(error)).is_err());
        }
    }

    #[test]
    fn native_drift_routing_keeps_default_and_explicit_far_input_distinct() {
        assert_eq!(NativeImpulse::parse("drift"), Some(NativeImpulse::Drift));
        assert_eq!(
            NativeImpulse::parse("fast-drift"),
            Some(NativeImpulse::FastDrift)
        );
        assert_eq!(NativeImpulse::parse("spin"), Some(NativeImpulse::Spin));
        assert_eq!(NativeImpulse::Drift.linear_speed(), Some(0.2));
        assert_eq!(NativeImpulse::FastDrift.linear_speed(), Some(2.0));
        assert_eq!(NativeImpulse::Spin.linear_speed(), None);
        assert_eq!(NativeImpulse::FastDrift.name(), "fast-drift");
        assert!(NativeImpulse::parse("drift=2.0").is_none());
    }

    #[test]
    fn rearm_accepts_exact_original_fixtures_and_rejects_missing_or_extra_cells() {
        for number in 0..4 {
            let (fixture, actual, catalog) = rearm_test_snapshot(number, None);
            let connected = actual.cells.keys().copied().collect::<Vec<_>>();
            assert_eq!(fixture.members.len(), if number == 3 { 7 } else { 138 });
            assert_eq!(fixture.envelope.len(), if number == 3 { 24 } else { 272 });
            assert!(fixture.validate(&actual, &connected, &catalog).is_ok());
            let mut missing = actual.clone();
            missing.cells.remove(&connected[0]);
            assert!(fixture.validate(&missing, &connected, &catalog).is_err());
            let mut extra = actual.clone();
            let unexpected = if number == 3 {
                [
                    fixture.origin[0] + 3,
                    fixture.origin[1] + 1,
                    fixture.origin[2] + 2,
                ]
            } else {
                [
                    fixture.origin[0] + 17,
                    fixture.origin[1],
                    fixture.origin[2] + 2,
                ]
            };
            assert!(!extra.cells.contains_key(&unexpected));
            extra
                .cells
                .insert(unexpected, "minecraft:blackstone".into());
            // This also catches the dynamic fixture's disconnected extra cell
            // inside its envelope, which gather alone cannot discover.
            assert!(fixture.validate(&extra, &connected, &catalog).is_err());
            let mut extended_component = connected.clone();
            extended_component.push([fixture.origin[0] + 34, fixture.origin[1], fixture.origin[2]]);
            assert!(
                fixture
                    .validate(&actual, &extended_component, &catalog)
                    .is_err()
            );
        }
        assert!(OriginalFixture::for_slot(4).is_err());
    }

    #[test]
    fn rearm_refuses_changed_mass_shape_and_unsupported_container_states() {
        fn wrong_mass(role: FixtureBlock, properties: &mut sable_core::mass::Properties) {
            if matches!(role, FixtureBlock::Blackstone) {
                properties.mass = 3.;
            }
        }
        fn wrong_shape(role: FixtureBlock, properties: &mut sable_core::mass::Properties) {
            if matches!(role, FixtureBlock::BottomSlab) {
                properties.boxes = vec![[0., 0.5, 0., 1., 1., 1.]];
                properties.center = [0.5, 0.75, 0.5];
            }
        }
        for mutate in [
            wrong_mass as fn(FixtureBlock, &mut sable_core::mass::Properties),
            wrong_shape,
        ] {
            let (fixture, actual, catalog) = rearm_test_snapshot(0, Some(mutate));
            let connected = actual.cells.keys().copied().collect::<Vec<_>>();
            assert!(fixture.validate(&actual, &connected, &catalog).is_err());
        }
        assert!(
            !FixtureBlock::Chest
                .state_allowed("minecraft:chest[facing=north,type=left,waterlogged=false]")
        );
        assert!(
            !FixtureBlock::Chest
                .state_allowed("minecraft:chest[facing=north,type=single,waterlogged=true]")
        );
        assert!(
            !FixtureBlock::BottomSlab
                .state_allowed("minecraft:stone_slab[type=top,waterlogged=false]")
        );
        assert!(!FixtureBlock::Blackstone.state_allowed("minecraft:stone"));
        assert!(
            FixtureBlock::Chest
                .state_allowed("minecraft:chest[facing=east,type=single,waterlogged=false]")
        );
        assert!(FixtureBlock::Furnace.state_allowed("minecraft:furnace[facing=south,lit=true]"));
    }

    #[test]
    fn rearm_selects_original_inputs_without_transplanting_a_default_pose() {
        let planned = Motion {
            pose: Pose {
                position: [-17.25, 48.5, -0.],
                pivot: [1034.5, 128.25, 1033.5],
                ..Pose::default()
            },
            linear: [3., 4., 5.],
            angular: [6., 7., 8.],
        };
        for (number, linear, angular) in [
            (0, [0.2, 0., 0.], [0.; 3]),
            (1, [0.; 3], [0., 0.05, 0.]),
            (2, [0.; 3], [0., 0., 0.2]),
            (3, [0.; 3], [0.; 3]),
        ] {
            let fixture = OriginalFixture::for_slot(number).unwrap();
            let motion = fixture.motion(&planned);
            assert_eq!(
                motion.pose.position.map(f64::to_bits),
                planned.pose.position.map(f64::to_bits)
            );
            assert_eq!(
                motion.pose.pivot.map(f64::to_bits),
                planned.pose.pivot.map(f64::to_bits)
            );
            assert_eq!(
                motion.pose.rotation.map(f64::to_bits),
                planned.pose.rotation.map(f64::to_bits)
            );
            assert_eq!(
                motion.pose.scale.map(f64::to_bits),
                planned.pose.scale.map(f64::to_bits)
            );
            assert_eq!(motion.linear, linear);
            assert_eq!(motion.angular, angular);
            assert_eq!(
                fixture.profile,
                match number {
                    0 => Profile::Translate,
                    1 => Profile::Yaw,
                    2 => Profile::Pitch,
                    _ => Profile::Dynamic,
                }
            );
        }
    }

    #[test]
    fn fresh_restore_preserves_current_bits_and_settles_historical_previous() {
        let historical = LiveWorldPose::at([-17., 48., -0.]);
        let current = LiveWorldPose {
            translation: [-16.75, 48.25, -0.],
            pivot: [1.25, 0.5, 3.],
            rotation: [0., 0., 0.35_f64.sin(), 0.35_f64.cos()],
        };
        let publish_receipt = LiveWorldSample::new(historical, current, 91);
        let fresh = LiveWorldSample::new(current, current, 91);
        assert!(restored_sample_matches(
            &fresh,
            publish_receipt.current,
            publish_receipt.revision
        ));
        assert!(!restored_sample_matches(&publish_receipt, current, 91));
        assert!(!restored_sample_matches(&fresh, current, 90));
        let mut changed_current = fresh;
        changed_current.current.translation[2] = 0.;
        changed_current.previous = changed_current.current;
        assert!(!restored_sample_matches(&changed_current, current, 91));
        let mut equivalent_rotation = fresh;
        equivalent_rotation.current.rotation = current.rotation.map(|value| -value);
        equivalent_rotation.previous = equivalent_rotation.current;
        assert!(!restored_sample_matches(&equivalent_rotation, current, 91));
        let mut changed_previous = fresh;
        changed_previous.previous.rotation[0] = -0.;
        assert!(!restored_sample_matches(&changed_previous, current, 91));
    }

    #[test]
    fn episode_reset_preserves_normal_furnace_lit_transitions_but_refuses_replacements() {
        assert!(reset_state_matches(
            "minecraft:furnace[facing=north,lit=true]",
            "minecraft:furnace[facing=north,lit=false]"
        ));
        assert!(!reset_state_matches(
            "minecraft:furnace[facing=north,lit=true]",
            "minecraft:furnace[facing=south,lit=false]"
        ));
        assert!(!reset_state_matches(
            "minecraft:furnace[facing=north,lit=true]",
            "minecraft:blackstone"
        ));
        assert!(!reset_state_matches(
            "minecraft:stone_slab[type=top,waterlogged=false]",
            "minecraft:stone_slab[type=bottom,waterlogged=false]"
        ));
        assert!(!reset_state_matches(
            "minecraft:furnace[facing=north,lit=invalid]",
            "minecraft:furnace[facing=north,lit=false]"
        ));
    }

    #[test]
    fn explicit_setup_recovery_accepts_only_the_exact_paused_initial_epoch() {
        for mode in 0..=3 {
            // Recovery computes this frame from actual admitted live geometry;
            // this unit test exercises the metadata gate independently.
            let mut expected = initial_motion(mode).at_self_com([1.25, 0.5, 0.75]);
            expected.pose.position = expected.pose.position.map(|v| f64::from(v as f32));
            let pose = host_pose(&expected);
            let sample = LiveWorldSample::new(pose, pose, 0);
            let saved = persist::Saved::new(&expected, &sample, false, true, 0);
            assert!(Scene::validate_saved_initial(&saved, &expected).is_ok());
            let changes: [fn(&mut persist::Saved); 9] = [
                |s| s.deleted = true,
                |s| s.paused = false,
                |s| s.native_epoch = 1,
                |s| s.pose_revision = 1,
                |s| s.linear[1] = 0.125,
                |s| s.angular[0] = -0.,
                |s| s.raw_pose.position[0] += 0.25,
                |s| s.raw_pose.pivot[1] += 0.25,
                |s| s.pose.translation[2] += 0.25,
            ];
            for change in changes {
                let mut invalid = saved.clone();
                change(&mut invalid);
                assert!(Scene::validate_saved_initial(&invalid, &expected).is_err());
            }
        }
    }
    #[test]
    fn owned_nonempty_tombstone_requires_positive_common_receipt_and_never_empty_retirement() {
        let source = LevelRef::new(WorldEpoch::new(9), DimensionKey::NETHER);
        let phase = Phase::OwnedUnprojected {
            barrier_epoch: 785,
            members: vec![[1035, 128, 1032]],
        };
        let row = LiveWorldLevelReceipt::new(source, 126, 600, None, Some(185), None);
        let valid = LiveWorldSaveReceipt::new(831, vec![row]);
        let check = |receipt| {
            owned_unprojected_completion(
                source,
                &phase,
                Some(source),
                Ok(None),
                Err(LiveWorldError::Stale),
                || Ok(receipt),
            )
        };
        assert!(check(valid.clone()).unwrap().is_some());
        let mut old = valid.clone();
        old.barrier_epoch = 784;
        assert!(check(old).is_err());
        let mut empty = valid.clone();
        empty.levels[0].retired_members = Some(0);
        assert!(check(empty).is_err());
        let mut missing = valid.clone();
        missing.levels.clear();
        assert!(check(missing).is_err());
        let mut duplicate = valid.clone();
        duplicate.levels.push(duplicate.levels[0].clone());
        assert!(check(duplicate).is_err());
        let mut active = valid.clone();
        active.levels[0].binding = Some(LiveWorldRef::new(source, 186));
        assert!(check(active).is_err());
        let mut zero_generation = valid.clone();
        zero_generation.levels[0].retired_generation = Some(0);
        assert!(check(zero_generation).is_err());
        assert!(
            owned_unprojected_completion(
                source,
                &phase,
                Some(source),
                Ok(None),
                Err(LiveWorldError::Stale),
                || Err(LiveWorldError::Busy)
            )
            .unwrap()
            .is_none()
        );
        assert!(
            owned_unprojected_completion(
                source,
                &phase,
                Some(source),
                Ok(None),
                Err(LiveWorldError::Storage),
                || Ok(valid.clone())
            )
            .is_err()
        );
        assert!(
            owned_unprojected_completion(
                source,
                &phase,
                Some(source),
                Ok(None),
                Err(LiveWorldError::Stale),
                || Err(LiveWorldError::Storage)
            )
            .is_err()
        );
    }

    #[test]
    fn owned_nonempty_fresh_receipt_needs_current_common_coverage_not_historical_generation() {
        let source = LevelRef::new(WorldEpoch::new(9), DimensionKey::NETHER);
        let foreign = LevelRef::new(WorldEpoch::new(10), DimensionKey::NETHER);
        let phase = Phase::OwnedUnprojected {
            barrier_epoch: 785,
            members: vec![[1035, 128, 1032]],
        };
        let valid = LiveWorldSaveReceipt::new(
            834,
            vec![LiveWorldLevelReceipt::new(
                source, 126, 600, None, None, None,
            )],
        );
        let check = |phase: &Phase, current, binding, descriptor, receipt| {
            owned_unprojected_completion(source, phase, current, binding, descriptor, || {
                Ok(receipt)
            })
        };
        let accepted = check(
            &phase,
            Some(source),
            Ok(None),
            Err(LiveWorldError::Stale),
            valid.clone(),
        )
        .unwrap()
        .unwrap();
        assert_eq!(accepted.retired_generation, None);
        assert_eq!(accepted.retired_members, None);
        assert_eq!(accepted.block_revision, 126);
        assert_eq!(accepted.content_revision, 600);
        let mut historical = valid.clone();
        historical.levels[0].retired_generation = Some(185);
        assert!(
            check(
                &phase,
                Some(source),
                Ok(None),
                Err(LiveWorldError::Stale),
                historical
            )
            .unwrap()
            .is_some()
        );
        let mut zero = valid.clone();
        zero.levels[0].retired_generation = Some(0);
        let mut empty = valid.clone();
        empty.levels[0].retired_members = Some(0);
        let mut nonempty = valid.clone();
        nonempty.levels[0].retired_members = Some(126);
        let mut old = valid.clone();
        old.barrier_epoch = 784;
        let mut wrong = valid.clone();
        wrong.levels[0].level = foreign;
        let mut duplicate = valid.clone();
        duplicate.levels.push(duplicate.levels[0].clone());
        let mut active = valid.clone();
        active.levels[0].binding = Some(LiveWorldRef::new(source, 186));
        for receipt in [
            zero,
            empty,
            nonempty,
            old,
            wrong,
            duplicate,
            active,
            LiveWorldSaveReceipt::new(834, vec![]),
        ] {
            assert!(
                check(
                    &phase,
                    Some(source),
                    Ok(None),
                    Err(LiveWorldError::Stale),
                    receipt
                )
                .is_err()
            );
        }
        for current in [None, Some(foreign)] {
            assert!(
                check(
                    &phase,
                    current,
                    Ok(None),
                    Err(LiveWorldError::Stale),
                    valid.clone()
                )
                .is_err()
            );
        }
        for descriptor in [
            LiveWorldError::NotSaved,
            LiveWorldError::Storage,
            LiveWorldError::NotResident,
        ] {
            assert!(
                check(
                    &phase,
                    Some(source),
                    Ok(None),
                    Err(descriptor),
                    valid.clone()
                )
                .is_err()
            );
        }
        for phase in [
            Phase::Returned { barrier_epoch: 785 },
            Phase::OwnedUnprojected {
                barrier_epoch: 785,
                members: vec![],
            },
        ] {
            assert!(
                check(
                    &phase,
                    Some(source),
                    Ok(None),
                    Err(LiveWorldError::Stale),
                    valid.clone()
                )
                .is_err()
            );
        }
        assert!(
            check(
                &phase,
                Some(source),
                Ok(Some(LiveWorldRef::new(source, 186))),
                Err(LiveWorldError::Stale),
                valid.clone()
            )
            .is_err()
        );
        assert!(
            check(
                &phase,
                Some(source),
                Err(LiveWorldError::Storage),
                Err(LiveWorldError::Stale),
                valid
            )
            .is_err()
        );
    }

    #[test]
    fn owned_state_readback_only_ignores_valid_furnace_lit() {
        assert!(member_key_equal(
            "minecraft:furnace[facing=north,lit=true]",
            "minecraft:furnace[facing=north,lit=false]"
        ));
        assert!(!member_key_equal(
            "minecraft:furnace[facing=south,lit=true]",
            "minecraft:furnace[facing=north,lit=false]"
        ));
        assert!(!member_key_equal(
            "minecraft:furnace[facing=north,lit=bad]",
            "minecraft:furnace[facing=north,lit=false]"
        ));
        assert!(!member_key_equal(
            "minecraft:furnace[facing=north]",
            "minecraft:furnace[facing=north,lit=false]"
        ));
        assert!(!member_key_equal("minecraft:blackstone", "minecraft:stone"));
    }
}

#[cfg(test)]
mod open_reset_tests {
    use super::*;

    fn placement(position: [f64; 3]) -> LiveWorldPose {
        host_pose(&Motion {
            pose: Pose {
                position,
                pivot: OPEN_ORIGIN.map(f64::from),
                ..Pose::default()
            },
            ..Motion::default()
        })
    }

    #[test]
    fn safe_air_and_nearby_beach_do_not_require_ground_or_a_fixed_radius() {
        let pose = placement([0., 114., 0.]);
        for observer in [
            [[2.8, 111.7, 7.9], [3.4, 113.5, 8.5]],
            [[-3.3, 65., 0.2], [-2.7, 66.8, 0.8]],
            [[0.2, 116.16, 0.2], [0.8, 117.96, 0.8]],
        ] {
            assert_eq!(
                open_reset_clearance(observer, pose, &open_cells()),
                Ok(true)
            );
        }
    }

    #[test]
    fn touching_support_and_initial_spawn_volume_are_refused() {
        let pose = placement([0., 114., 0.]);
        for observer in [
            [[0.2, 114.2, 0.2], [0.8, 116., 0.8]],
            [[0.2, 116., 0.2], [0.8, 117.8, 0.8]],
            [[0.2, 116.1, 0.2], [0.8, 117.9, 0.8]],
        ] {
            assert_eq!(
                open_reset_clearance(observer, pose, &open_cells()),
                Ok(false)
            );
        }
        let observer = [[0.2, 114.2, 0.2], [0.8, 116., 0.8]];
        assert_eq!(
            open_reset_clearance(observer, placement([8., 58., 0.]), &open_cells()),
            Ok(true)
        );
        // Checking only the settled current body would allow a reset into this player.
        assert_eq!(
            open_reset_clearance(observer, pose, &open_cells()),
            Ok(false)
        );
    }

    #[test]
    fn rotated_negative_world_placement_uses_the_source_pivot() {
        let mut pose = placement([-100., 72., -40.]);
        pose.rotation = [
            0.,
            0.,
            std::f64::consts::FRAC_1_SQRT_2,
            std::f64::consts::FRAC_1_SQRT_2,
        ];
        assert_eq!(
            open_reset_clearance(
                [[-100.8, 72.2, -39.8], [-100.2, 74., -39.2]],
                pose,
                &open_cells()
            ),
            Ok(false)
        );
        assert_eq!(
            open_reset_clearance(
                [[-100.8, 72.2, -30.8], [-100.2, 74., -30.2]],
                pose,
                &open_cells()
            ),
            Ok(true)
        );
    }

    #[test]
    fn missing_or_corrupt_bounds_are_not_permission_to_reset() {
        let pose = placement([0., 114., 0.]);
        for bounds in [
            [[f64::NAN, 114., 0.], [0.8, 115.8, 0.8]],
            [[0.8, 114., 0.], [0.2, 115.8, 0.8]],
            [[0.2, 114., 0.], [0.2, 115.8, 0.8]],
        ] {
            assert!(open_reset_clearance(bounds, pose, &open_cells()).is_err());
        }
        let mut corrupt = pose;
        corrupt.rotation = [0.; 4];
        assert!(
            open_reset_clearance(
                [[20., 90., 20.], [20.6, 91.8, 20.6]],
                corrupt,
                &open_cells()
            )
            .is_err()
        );
    }
}
