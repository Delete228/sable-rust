//! Concrete Sunrise transfer execution. All live contents stay behind the public
//! host token; this layer carries only cell mappings, caller metadata and actual
//! receipts. It is called from the global scheduler frontier after dropping the
//! State RefCell borrow, so normal admission callbacks can read the personal policy.
use crate::owner_state::{self, OwnerState};
use mod_api::*;
use sable_core::host_lifecycle::PersistentState;

pub struct ObservedState {
    pub state: sable_core::host_lifecycle::StateSnapshot,
    pub revisions: LevelRevisions,
}
/// A bounded readset over actual requested cells. Missing chunks are errors,
/// never air. The native prepared token separately captures BE/attachment/
/// schedule readsets and revalidates them before any first-source removal.
pub fn read_snapshot(
    host: &dyn ServerHost,
    level: LevelRef,
    positions: &[[i32; 3]],
    catalog: &mut sable_sunrise::registry::PropertyCatalog,
) -> Result<ObservedState, String> {
    let unique = positions
        .iter()
        .copied()
        .collect::<std::collections::BTreeSet<_>>();
    if unique.len() != positions.len() || unique.len() > 512 {
        return Err("invalid bounded actual readset selection".into());
    }
    let before = level
        .revisions(host)
        .map_err(|error| format!("actual level revisions: {error:?}"))?;
    let mut cells = std::collections::BTreeMap::new();
    for &position in positions {
        let state = level
            .block_state(host, position)
            .ok_or_else(|| "actual transfer cell unavailable; no air fallback".to_string())?;
        let metadata = host.block_states();
        let name = metadata
            .name(state.block())
            .map_err(|error| format!("{error:?}"))?;
        if matches!(
            name,
            "minecraft:air" | "minecraft:cave_air" | "minecraft:void_air"
        ) {
            continue;
        }
        let boxes = level
            .block_shape(host, state, position, BlockShapeKind::Collision)
            .ok_or_else(|| "actual transfer shape unavailable".to_string())?;
        cells.insert(
            position,
            crate::scene::resolve(host, state, boxes, catalog)?,
        );
    }
    let after = level
        .revisions(host)
        .map_err(|error| format!("actual level revisions: {error:?}"))?;
    if before != after {
        return Err("actual readset changed during policy observation".into());
    }
    Ok(ObservedState {
        state: sable_core::host_lifecycle::StateSnapshot {
            revision: before.block_revision,
            cells,
        },
        revisions: before,
    })
}
pub struct JournalWrites {
    pub owners: Vec<(LevelRef, OwnerState)>,
    pub registry_level: LevelRef,
    pub registry: PersistentState,
}
impl JournalWrites {
    pub(crate) fn write(&self, host: &mut dyn ServerHost) -> Result<(), String> {
        for (source, record) in &self.owners {
            owner_state::OWNER
                .set(&mut source.attachments(host), Holder::Level, record.clone())
                .map_err(|error| format!("{error:?}"))?;
        }
        owner_state::REGISTRY
            .set(
                &mut self.registry_level.attachments(host),
                Holder::Level,
                self.registry.clone(),
            )
            .map_err(|error| format!("{error:?}"))
    }
    pub(crate) fn verify(&self, host: &mut dyn ServerHost) -> Result<(), String> {
        for (source, expected) in &self.owners {
            let actual = owner_state::OWNER
                .get_existing(&mut source.attachments(host), Holder::Level)
                .map_err(|error| format!("{error:?}"))?
                .ok_or_else(|| "committed Source owner metadata is missing".to_string())?;
            if !owner_state::same_record(expected, &actual) {
                return Err(
                    "committed Source owner metadata differs from the staged journal afterimage"
                        .into(),
                );
            }
        }
        let actual = owner_state::REGISTRY
            .get_existing(&mut self.registry_level.attachments(host), Holder::Level)
            .map_err(|error| format!("{error:?}"))?
            .ok_or_else(|| "committed logical registry metadata is missing".to_string())?;
        if actual != self.registry {
            return Err(
                "committed logical registry metadata differs from the staged afterimage".into(),
            );
        }
        Ok(())
    }
}

pub enum TransferOutcome {
    /// Actual ownership and caller metadata are committed; presentation may
    /// still be OwnedUnprojected and requires its separate checked frontier.
    Committed(LiveWorldTransferReceipt),
    /// Actual host token says no commit. Only this permits cancelling core work.
    NotCommitted { problem: Option<String> },
    /// Actual host has not settled the root yet. Keep core Unconfirmed.
    Waiting,
    /// Fenced I/O or an unexpected host failure. Never infer rollback/reseed.
    Unconfirmed { problem: String },
    /// A real successful receipt survives a failed caller metadata validation.
    CommittedUnreconciled {
        receipt: LiveWorldTransferReceipt,
        problem: String,
    },
}

pub struct PreparedTransfer {
    token: LiveWorldTransferRef,
    cells: Vec<LiveWorldTransferCell>,
    expected_levels: Vec<LevelRef>,
    planned_epoch: Option<u64>,
    writes: Option<JournalWrites>,
    closure_problem: Option<String>,
}
impl PreparedTransfer {
    pub fn prepare(
        host: &mut dyn ServerHost,
        receiver: LevelRef,
        cells: Vec<LiveWorldTransferCell>,
    ) -> Result<Self, String> {
        if cells.is_empty() || cells.len() > 512 {
            return Err("bounded lifecycle selection must contain 1..512 real cells".into());
        }
        let mut levels = Vec::new();
        for cell in &cells {
            for level in [cell.source, cell.destination] {
                if !levels.contains(&level) {
                    levels.push(level);
                }
            }
        }
        if levels.len() > 4 || !levels.contains(&receiver) {
            return Err("bounded lifecycle level budget or receiver identity".into());
        }
        let limits = LiveWorldTransferLimits {
            cells: 512,
            levels: 4,
            chunks: 256,
            boxes: 1024,
            scheduled_ticks: 4096,
            // Common-WAL coverage includes complete world/player/metadata afterimages.
            bytes: 64 * 1024 * 1024,
        };
        let token = receiver
            .prepare_transfer(host, &cells, limits)
            .map_err(|error| format!("{error:?}"))?;
        Ok(Self {
            token,
            cells,
            expected_levels: levels,
            planned_epoch: None,
            writes: None,
            closure_problem: None,
        })
    }
    pub fn planned_epoch(&self) -> Option<u64> {
        self.planned_epoch
    }
    pub fn token(&self) -> LiveWorldTransferRef {
        self.token
    }
    /// `validate` reads fresh actual state and checks the reserved core plan
    /// after host admission callbacks. Neither closure may retain borrowed Host.
    /// `metadata` constructs explicit OwnedUnprojected/Returned records using
    /// the PLANNED epoch; only a later actual receipt proves durable ownership.
    pub fn execute(
        &mut self,
        host: &mut dyn ServerHost,
        validate: impl FnOnce(&mut dyn ServerHost) -> Result<(), String>,
        metadata: impl FnOnce(u64) -> Result<JournalWrites, String>,
    ) -> TransferOutcome {
        if self.planned_epoch.is_some() {
            return TransferOutcome::Unconfirmed {
                problem: "transfer was already staged; poll its actual receipt".into(),
            };
        }
        let attempt = host.transaction(|host, tx| -> Result<(), String> {
            let pending = self
                .token
                .stage(host, tx)
                .map_err(|error| format!("{error:?}"))?;
            if pending.planned_epoch == 0 {
                return Err("host returned an invalid planned barrier".into());
            }
            self.planned_epoch = Some(pending.planned_epoch);
            validate(host)?;
            let writes = metadata(pending.planned_epoch)?;
            writes.write(host)?;
            self.writes = Some(writes);
            Ok(())
        });
        if let Err(problem) = attempt {
            self.closure_problem = Some(problem);
        }
        self.poll(host)
    }
    pub fn poll(&self, host: &mut dyn ServerHost) -> TransferOutcome {
        match self.token.receipt(host) {
            Ok(receipt) => {
                let validate = self.verify_committed(host, &receipt);
                match validate {
                    Ok(()) => TransferOutcome::Committed(receipt),
                    Err(problem) => TransferOutcome::CommittedUnreconciled { receipt, problem },
                }
            }
            Err(LiveWorldError::NotCommitted) => TransferOutcome::NotCommitted {
                problem: self.closure_problem.clone(),
            },
            Err(LiveWorldError::Busy) => TransferOutcome::Waiting,
            Err(problem) => TransferOutcome::Unconfirmed {
                problem: format!("{problem:?}"),
            },
        }
    }
    /// A retained successful payload receipt never bypasses exact caller
    /// afterimage and complete common-journal level coverage validation.
    pub(crate) fn verify_committed(
        &self,
        host: &mut dyn ServerHost,
        receipt: &LiveWorldTransferReceipt,
    ) -> Result<(), String> {
        if let Some(problem) = &self.closure_problem {
            return Err(format!("host committed after caller root error: {problem}"));
        }
        if self.planned_epoch != Some(receipt.barrier_epoch)
            || receipt.barrier_epoch == 0
            || receipt.moved_cells as usize != self.cells.len()
        {
            return Err("actual ownership receipt does not match the staged epoch/mapping".into());
        }
        let mut seen = Vec::new();
        for level in &receipt.levels {
            if seen.contains(&level.level) {
                return Err("actual ownership receipt repeated a level".into());
            }
            seen.push(level.level);
        }
        if self
            .expected_levels
            .iter()
            .any(|level| !seen.contains(level))
        {
            return Err("actual ownership receipt omitted a mapped level".into());
        }
        let writes = self
            .writes
            .as_ref()
            .ok_or_else(|| "actual commit has no staged caller metadata".to_string())?;
        for level in writes
            .owners
            .iter()
            .map(|(level, _)| *level)
            .chain(std::iter::once(writes.registry_level))
        {
            if !seen.contains(&level) {
                return Err("actual common receipt omitted a caller-metadata Source level".into());
            }
        }
        writes.verify(host)
    }
    pub fn cancel_not_committed(&self, host: &mut dyn ServerHost) -> Result<(), String> {
        match self.token.receipt(host) {
            Err(LiveWorldError::NotCommitted) => self
                .token
                .cancel(host)
                .map_err(|error| format!("{error:?}")),
            Ok(_) => Err("a committed transfer cannot be cancelled".into()),
            Err(problem) => Err(format!(
                "transfer outcome is not confirmed NotCommitted: {problem:?}"
            )),
        }
    }
    /// Only for a still-unused plan. Staged aborts use cancel_not_committed;
    /// A successful/ambiguous root is never cancelled as a rollback substitute.
    pub fn cancel_unused(&self, host: &mut dyn ServerHost) -> Result<(), String> {
        if self.planned_epoch.is_some() {
            return Err("a staged transfer must be resolved through its actual receipt".into());
        }
        self.token
            .cancel(host)
            .map_err(|error| format!("{error:?}"))
    }
}
