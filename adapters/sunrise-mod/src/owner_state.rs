//! Caller policy metadata in the actual host attachment journal. These records
//! contain identities, physical inputs and reset geometry only, never live BE,
//! inventory, entity or schedule payloads. A phase distinguishes actual durable
//! ownership from a projected presentation and from an explicit retirement.
use crate::{frames, persist};
use mod_api::*;
use sable_core::{
    Config, Id, Motion, Pos, Pose,
    host_lifecycle::{LogicalIdentity, PersistentState},
};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Profile {
    Translate,
    Yaw,
    Pitch,
    Dynamic,
    Split,
    OpenDemo,
}
impl Profile {
    fn number(self) -> i32 {
        match self {
            Self::Translate => 0,
            Self::Yaw => 1,
            Self::Pitch => 2,
            Self::Dynamic => 3,
            Self::Split => 4,
            Self::OpenDemo => 5,
        }
    }
    fn decode(value: i32) -> Option<Self> {
        match value {
            0 => Some(Self::Translate),
            1 => Some(Self::Yaw),
            2 => Some(Self::Pitch),
            3 => Some(Self::Dynamic),
            4 => Some(Self::Split),
            5 => Some(Self::OpenDemo),
            _ => None,
        }
    }
    pub fn epoch_policy(self) -> persist::EpochPolicy {
        match self {
            Self::OpenDemo => persist::EpochPolicy::OpenDemo,
            _ => persist::EpochPolicy::Review,
        }
    }
}
#[derive(Clone)]
pub struct RawState {
    pub motion: Motion,
    pub native_epoch: u64,
    pub paused: bool,
}
#[derive(Clone)]
pub struct Episode {
    pub profile: Profile,
    pub initial: Motion,
    /// Explicit integer return mapping; never rounded from a simulated pose.
    pub return_offset: Pos,
    /// Derived canonical state keys for the explicitly recorded reset episode.
    /// This is never a block/BE source; reset validates current real members.
    pub cells: BTreeMap<Pos, String>,
}
#[derive(Clone)]
pub enum Phase {
    OwnedUnprojected {
        barrier_epoch: u64,
        members: Vec<Pos>,
    },
    Projected {
        pose_revision: u64,
        pose: LiveWorldPose,
    },
    Returned {
        barrier_epoch: u64,
    },
    Retired {
        barrier_epoch: u64,
        generation: u64,
    },
    /// Actual in-memory retirement, still awaiting its durable level receipt.
    Retiring {
        generation: u64,
        members: Vec<Pos>,
    },
}
#[derive(Clone)]
pub struct OwnerState {
    pub logical: LogicalIdentity,
    pub current: RawState,
    pub episode: Episode,
    pub phase: Phase,
}
impl OwnerState {
    pub fn projected(&self) -> Option<persist::Saved> {
        let Phase::Projected {
            pose_revision,
            pose,
        } = &self.phase
        else {
            return None;
        };
        Some(persist::Saved {
            epoch_policy: self.episode.profile.epoch_policy(),
            deleted: false,
            paused: self.current.paused,
            native_epoch: self.current.native_epoch,
            pose_revision: *pose_revision,
            pose: *pose,
            raw_pose: self.current.motion.pose,
            linear: self.current.motion.linear,
            angular: self.current.motion.angular,
        })
    }
    pub fn accept(&mut self, motion: &Motion, sample: &LiveWorldSample, epoch: u64, paused: bool) {
        self.current = RawState {
            motion: motion.clone(),
            native_epoch: epoch,
            paused,
        };
        self.phase = Phase::Projected {
            pose_revision: sample.revision,
            pose: sample.current,
        };
    }
}
fn raw_valid(raw: &RawState, profile: Profile) -> bool {
    profile.epoch_policy().valid(raw.native_epoch, raw.paused)
        && raw.motion.pose.valid()
        && raw.motion.pose.scale == [1.; 3]
        && raw
            .motion
            .pose
            .position
            .iter()
            .chain(&raw.motion.pose.pivot)
            .chain(&raw.motion.pose.rotation)
            .chain(&raw.motion.linear)
            .chain(&raw.motion.angular)
            .all(|v| v.is_finite() && (*v as f32).is_finite())
}
fn pos_valid(position: Pos) -> bool {
    Config::default().local_valid(position)
}
fn raw_tag(raw: &RawState, profile: Profile) -> Result<Tag, AttachmentError> {
    if !raw_valid(raw, profile) {
        return Err(AttachmentError::Encode);
    }
    let values = raw
        .motion
        .pose
        .position
        .iter()
        .chain(&raw.motion.pose.pivot)
        .chain(&raw.motion.pose.rotation)
        .chain(&raw.motion.linear)
        .chain(&raw.motion.angular)
        .map(|v| Tag::Double(v.to_bits()))
        .collect();
    Ok(Tag::Compound(
        [
            ("native_epoch".into(), ulong(raw.native_epoch)),
            ("paused".into(), Tag::Byte(i8::from(raw.paused))),
            ("values".into(), Tag::List(values)),
        ]
        .into(),
    ))
}
fn ulong(value: u64) -> Tag {
    Tag::Long(i64::from_le_bytes(value.to_le_bytes()))
}
fn read_ulong(value: &Tag) -> Result<u64, AttachmentError> {
    match value {
        Tag::Long(value) => Ok(u64::from_le_bytes(value.to_le_bytes())),
        _ => Err(AttachmentError::Decode),
    }
}
fn int(data: &Compound, name: &str) -> Result<i32, AttachmentError> {
    match data.get(&name.into()) {
        Some(Tag::Int(value)) => Ok(*value),
        _ => Err(AttachmentError::Decode),
    }
}
fn field<'a>(data: &'a Compound, name: &str) -> Result<&'a Tag, AttachmentError> {
    data.get(&name.into()).ok_or(AttachmentError::Decode)
}
fn read_raw(value: &Tag, profile: Profile) -> Result<RawState, AttachmentError> {
    let Tag::Compound(data) = value else {
        return Err(AttachmentError::Decode);
    };
    if data.len() != 3 {
        return Err(AttachmentError::Decode);
    }
    let epoch = read_ulong(field(data, "native_epoch")?)?;
    let paused = match field(data, "paused")? {
        Tag::Byte(0) => false,
        Tag::Byte(1) => true,
        _ => return Err(AttachmentError::Decode),
    };
    let Tag::List(values) = field(data, "values")? else {
        return Err(AttachmentError::Decode);
    };
    if values.len() != 16 {
        return Err(AttachmentError::Decode);
    }
    let values = values
        .iter()
        .map(|value| match value {
            Tag::Double(value) => Ok(f64::from_bits(*value)),
            _ => Err(AttachmentError::Decode),
        })
        .collect::<Result<Vec<_>, _>>()?;
    let raw = RawState {
        native_epoch: epoch,
        paused,
        motion: Motion {
            pose: Pose {
                position: values[..3].try_into().unwrap(),
                pivot: values[3..6].try_into().unwrap(),
                rotation: values[6..10].try_into().unwrap(),
                scale: [1.; 3],
            },
            linear: values[10..13].try_into().unwrap(),
            angular: values[13..16].try_into().unwrap(),
        },
    };
    if !raw_valid(&raw, profile) {
        return Err(AttachmentError::Decode);
    }
    Ok(raw)
}
fn position_tag(position: Pos) -> Tag {
    Tag::List(position.into_iter().map(Tag::Int).collect())
}
fn read_vector(value: &Tag) -> Result<Pos, AttachmentError> {
    let Tag::List(values) = value else {
        return Err(AttachmentError::Decode);
    };
    if values.len() != 3 {
        return Err(AttachmentError::Decode);
    }
    let mut position = [0; 3];
    for (index, value) in values.iter().enumerate() {
        let Tag::Int(value) = value else {
            return Err(AttachmentError::Decode);
        };
        position[index] = *value;
    }
    Ok(position)
}
fn read_position(value: &Tag) -> Result<Pos, AttachmentError> {
    let position = read_vector(value)?;
    if !pos_valid(position) {
        return Err(AttachmentError::Decode);
    }
    Ok(position)
}
fn identity_tag(identity: LogicalIdentity) -> Result<Tag, AttachmentError> {
    if identity.id.0 == [0; 16]
        || identity.plot >= Config::default().slots()
        || identity.generation == 0
        || identity.parent == Some(identity.id)
    {
        return Err(AttachmentError::Encode);
    }
    let id = |id: Id| Tag::List(id.0.into_iter().map(|byte| Tag::Byte(byte as i8)).collect());
    Ok(Tag::Compound(
        [
            ("id".into(), id(identity.id)),
            ("plot".into(), Tag::Int(identity.plot as i32)),
            ("generation".into(), ulong(identity.generation)),
            (
                "parent".into(),
                identity.parent.map_or_else(|| Tag::List(Vec::new()), id),
            ),
        ]
        .into(),
    ))
}
fn read_id(value: &Tag, optional: bool) -> Result<Option<Id>, AttachmentError> {
    let Tag::List(values) = value else {
        return Err(AttachmentError::Decode);
    };
    if optional && values.is_empty() {
        return Ok(None);
    }
    if values.len() != 16 {
        return Err(AttachmentError::Decode);
    }
    let mut id = [0; 16];
    for (index, value) in values.iter().enumerate() {
        let Tag::Byte(value) = value else {
            return Err(AttachmentError::Decode);
        };
        id[index] = *value as u8;
    }
    if id == [0; 16] {
        return Err(AttachmentError::Decode);
    }
    Ok(Some(Id(id)))
}
fn read_identity(value: &Tag) -> Result<LogicalIdentity, AttachmentError> {
    let Tag::Compound(data) = value else {
        return Err(AttachmentError::Decode);
    };
    if data.len() != 4 {
        return Err(AttachmentError::Decode);
    }
    let plot = int(data, "plot")?;
    let logical = LogicalIdentity {
        id: read_id(field(data, "id")?, false)?.ok_or(AttachmentError::Decode)?,
        plot: u32::try_from(plot).map_err(|_| AttachmentError::Decode)?,
        generation: read_ulong(field(data, "generation")?)?,
        parent: read_id(field(data, "parent")?, true)?,
    };
    identity_tag(logical).map_err(|_| AttachmentError::Decode)?;
    Ok(logical)
}
fn encode_owner(owner: &OwnerState) -> Result<Compound, AttachmentError> {
    if owner.episode.cells.is_empty() || owner.episode.cells.len() > 512 {
        return Err(AttachmentError::Encode);
    }
    let cells = owner
        .episode
        .cells
        .iter()
        .map(|(&position, state)| {
            if !pos_valid(position) || state.is_empty() || state.len() > 1024 {
                return Err(AttachmentError::Encode);
            }
            Ok(Tag::Compound(
                [
                    ("position".into(), position_tag(position)),
                    ("state".into(), Tag::String(state.as_str().into())),
                ]
                .into(),
            ))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let initial = RawState {
        motion: owner.episode.initial.clone(),
        native_epoch: 0,
        paused: true,
    };
    let episode = Tag::Compound(
        [
            ("profile".into(), Tag::Int(owner.episode.profile.number())),
            ("initial".into(), raw_tag(&initial, owner.episode.profile)?),
            (
                "return_offset".into(),
                position_tag(owner.episode.return_offset),
            ),
            ("cells".into(), Tag::List(cells)),
        ]
        .into(),
    );
    let phase = match &owner.phase {
        Phase::Retiring {
            generation,
            members,
        } => {
            let unique = members.iter().copied().collect::<BTreeSet<_>>();
            if *generation == 0
                || members.len() > 512
                || unique.len() != members.len()
                || unique.iter().any(|position| !pos_valid(*position))
            {
                return Err(AttachmentError::Encode);
            }
            Tag::Compound(
                [
                    ("kind".into(), Tag::Int(4)),
                    ("generation".into(), ulong(*generation)),
                    (
                        "members".into(),
                        Tag::List(members.iter().copied().map(position_tag).collect()),
                    ),
                ]
                .into(),
            )
        }
        Phase::Projected {
            pose_revision,
            pose,
        } => {
            if !frames::equivalent(&owner.current.motion, *pose) {
                return Err(AttachmentError::Encode);
            }
            Tag::Compound(
                [
                    ("kind".into(), Tag::Int(1)),
                    ("pose_revision".into(), ulong(*pose_revision)),
                    (
                        "pose".into(),
                        Tag::List(
                            pose.translation
                                .iter()
                                .chain(&pose.pivot)
                                .chain(&pose.rotation)
                                .map(|v| Tag::Double(v.to_bits()))
                                .collect(),
                        ),
                    ),
                ]
                .into(),
            )
        }
        Phase::OwnedUnprojected {
            barrier_epoch,
            members,
        } => {
            let unique = members.iter().copied().collect::<BTreeSet<_>>();
            if *barrier_epoch == 0
                || unique.len() != members.len()
                || unique.is_empty()
                || unique.len() > 512
                || unique.iter().any(|p| !pos_valid(*p))
            {
                return Err(AttachmentError::Encode);
            }
            Tag::Compound(
                [
                    ("kind".into(), Tag::Int(0)),
                    ("barrier_epoch".into(), ulong(*barrier_epoch)),
                    (
                        "members".into(),
                        Tag::List(members.iter().copied().map(position_tag).collect()),
                    ),
                ]
                .into(),
            )
        }
        Phase::Returned { barrier_epoch } => {
            if *barrier_epoch == 0 {
                return Err(AttachmentError::Encode);
            }
            Tag::Compound(
                [
                    ("kind".into(), Tag::Int(2)),
                    ("barrier_epoch".into(), ulong(*barrier_epoch)),
                ]
                .into(),
            )
        }
        Phase::Retired {
            barrier_epoch,
            generation,
        } => {
            if *barrier_epoch == 0 || *generation == 0 {
                return Err(AttachmentError::Encode);
            }
            Tag::Compound(
                [
                    ("kind".into(), Tag::Int(3)),
                    ("barrier_epoch".into(), ulong(*barrier_epoch)),
                    ("generation".into(), ulong(*generation)),
                ]
                .into(),
            )
        }
    };
    Ok([
        ("version".into(), Tag::Int(1)),
        ("logical".into(), identity_tag(owner.logical)?),
        (
            "current".into(),
            raw_tag(&owner.current, owner.episode.profile)?,
        ),
        ("episode".into(), episode),
        ("phase".into(), phase),
    ]
    .into())
}
pub fn same_record(left: &OwnerState, right: &OwnerState) -> bool {
    match (encode_owner(left), encode_owner(right)) {
        (Ok(left), Ok(right)) => left == right,
        _ => false,
    }
}
fn decode_owner(data: &Compound) -> Result<OwnerState, AttachmentError> {
    if data.len() != 5 || int(data, "version")? != 1 {
        return Err(AttachmentError::Decode);
    }
    let logical = read_identity(field(data, "logical")?)?;
    let Tag::Compound(episode) = field(data, "episode")? else {
        return Err(AttachmentError::Decode);
    };
    if episode.len() != 4 {
        return Err(AttachmentError::Decode);
    }
    let profile = Profile::decode(int(episode, "profile")?).ok_or(AttachmentError::Decode)?;
    let current = read_raw(field(data, "current")?, profile)?;
    let initial = read_raw(field(episode, "initial")?, profile)?;
    let return_offset = read_vector(field(episode, "return_offset")?)?;
    if initial.native_epoch != 0 || !initial.paused {
        return Err(AttachmentError::Decode);
    }
    let Tag::List(values) = field(episode, "cells")? else {
        return Err(AttachmentError::Decode);
    };
    if values.is_empty() || values.len() > 512 {
        return Err(AttachmentError::Decode);
    }
    let mut cells = BTreeMap::new();
    for value in values {
        let Tag::Compound(cell) = value else {
            return Err(AttachmentError::Decode);
        };
        if cell.len() != 2 {
            return Err(AttachmentError::Decode);
        }
        let position = read_position(field(cell, "position")?)?;
        let Tag::String(state) = field(cell, "state")? else {
            return Err(AttachmentError::Decode);
        };
        let state = state.to_string().map_err(|_| AttachmentError::Decode)?;
        if state.is_empty() || state.len() > 1024 || cells.insert(position, state).is_some() {
            return Err(AttachmentError::Decode);
        }
    }
    let Tag::Compound(phase) = field(data, "phase")? else {
        return Err(AttachmentError::Decode);
    };
    let phase = match int(phase, "kind")? {
        0 if phase.len() == 3 => {
            let barrier_epoch = read_ulong(field(phase, "barrier_epoch")?)?;
            let Tag::List(members) = field(phase, "members")? else {
                return Err(AttachmentError::Decode);
            };
            if members.len() > 512 {
                return Err(AttachmentError::Decode);
            }
            Phase::OwnedUnprojected {
                barrier_epoch,
                members: members
                    .iter()
                    .map(read_position)
                    .collect::<Result<_, _>>()?,
            }
        }
        1 if phase.len() == 3 => {
            let pose_revision = read_ulong(field(phase, "pose_revision")?)?;
            let Tag::List(values) = field(phase, "pose")? else {
                return Err(AttachmentError::Decode);
            };
            if values.len() != 10 {
                return Err(AttachmentError::Decode);
            }
            let values = values
                .iter()
                .map(|value| match value {
                    Tag::Double(value) => Ok(f64::from_bits(*value)),
                    _ => Err(AttachmentError::Decode),
                })
                .collect::<Result<Vec<_>, _>>()?;
            Phase::Projected {
                pose_revision,
                pose: LiveWorldPose {
                    translation: values[..3].try_into().unwrap(),
                    pivot: values[3..6].try_into().unwrap(),
                    rotation: values[6..10].try_into().unwrap(),
                },
            }
        }
        4 if phase.len() == 3 => {
            let generation = read_ulong(field(phase, "generation")?)?;
            let Tag::List(members) = field(phase, "members")? else {
                return Err(AttachmentError::Decode);
            };
            if members.len() > 512 {
                return Err(AttachmentError::Decode);
            }
            Phase::Retiring {
                generation,
                members: members
                    .iter()
                    .map(read_position)
                    .collect::<Result<_, _>>()?,
            }
        }
        2 if phase.len() == 2 => Phase::Returned {
            barrier_epoch: read_ulong(field(phase, "barrier_epoch")?)?,
        },
        3 if phase.len() == 3 => Phase::Retired {
            barrier_epoch: read_ulong(field(phase, "barrier_epoch")?)?,
            generation: read_ulong(field(phase, "generation")?)?,
        },
        _ => return Err(AttachmentError::Decode),
    };
    let owner = OwnerState {
        logical,
        current,
        episode: Episode {
            profile,
            initial: initial.motion,
            return_offset,
            cells,
        },
        phase,
    };
    encode_owner(&owner).map_err(|_| AttachmentError::Decode)?;
    Ok(owner)
}
fn encode_registry(saved: &PersistentState) -> Result<Compound, AttachmentError> {
    if saved.owners.len() > 4 || saved.plot_generations.len() > Config::default().slots() as usize {
        return Err(AttachmentError::Encode);
    }
    let mut ids = BTreeSet::new();
    let mut plots = BTreeSet::new();
    let owners = saved
        .owners
        .iter()
        .map(|&owner| {
            if !ids.insert(owner.id)
                || !plots.insert(owner.plot)
                || saved.plot_generations.get(&owner.plot) != Some(&owner.generation)
            {
                return Err(AttachmentError::Encode);
            }
            identity_tag(owner)
        })
        .collect::<Result<Vec<_>, _>>()?;
    let floors = saved
        .plot_generations
        .iter()
        .map(|(&plot, &generation)| {
            if plot >= Config::default().slots() || generation == 0 {
                return Err(AttachmentError::Encode);
            }
            Ok(Tag::Compound(
                [
                    ("plot".into(), Tag::Int(plot as i32)),
                    ("generation".into(), ulong(generation)),
                ]
                .into(),
            ))
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok([
        ("version".into(), Tag::Int(1)),
        ("owners".into(), Tag::List(owners)),
        ("plot_generations".into(), Tag::List(floors)),
    ]
    .into())
}
fn decode_registry(data: &Compound) -> Result<PersistentState, AttachmentError> {
    if data.len() != 3 || int(data, "version")? != 1 {
        return Err(AttachmentError::Decode);
    }
    let Tag::List(owners) = field(data, "owners")? else {
        return Err(AttachmentError::Decode);
    };
    let Tag::List(floors) = field(data, "plot_generations")? else {
        return Err(AttachmentError::Decode);
    };
    if owners.len() > 4 || floors.len() > Config::default().slots() as usize {
        return Err(AttachmentError::Decode);
    }
    let owners = owners
        .iter()
        .map(read_identity)
        .collect::<Result<Vec<_>, _>>()?;
    let mut plot_generations = BTreeMap::new();
    for value in floors {
        let Tag::Compound(value) = value else {
            return Err(AttachmentError::Decode);
        };
        if value.len() != 2 {
            return Err(AttachmentError::Decode);
        }
        let plot = u32::try_from(int(value, "plot")?).map_err(|_| AttachmentError::Decode)?;
        if plot_generations
            .insert(plot, read_ulong(field(value, "generation")?)?)
            .is_some()
        {
            return Err(AttachmentError::Decode);
        }
    }
    let saved = PersistentState {
        owners,
        plot_generations,
    };
    encode_registry(&saved).map_err(|_| AttachmentError::Decode)?;
    Ok(saved)
}
// get_existing is mandatory; defaults are never permission to seed/adopt.
pub static OWNER: AttachmentType<OwnerState> = AttachmentType {
    namespace: "sable_port",
    name: "owner",
    default: || OwnerState {
        logical: LogicalIdentity {
            id: Id([0; 16]),
            plot: 0,
            generation: 0,
            parent: None,
        },
        current: RawState {
            motion: Motion::default(),
            native_epoch: 0,
            paused: true,
        },
        episode: Episode {
            profile: Profile::Dynamic,
            initial: Motion::default(),
            return_offset: [0; 3],
            cells: BTreeMap::new(),
        },
        phase: Phase::Returned { barrier_epoch: 0 },
    },
    codec: Some(AttachmentCodec {
        encode: encode_owner,
        decode: decode_owner,
    }),
    copy_on_death: false,
    sync: false,
};
pub static REGISTRY: AttachmentType<PersistentState> = AttachmentType {
    namespace: "sable_port",
    name: "lifecycle",
    default: || PersistentState {
        owners: Vec::new(),
        plot_generations: BTreeMap::new(),
    },
    codec: Some(AttachmentCodec {
        encode: encode_registry,
        decode: decode_registry,
    }),
    copy_on_death: false,
    sync: false,
};

#[cfg(test)]
mod tests {
    use super::*;
    fn owner() -> OwnerState {
        let motion = Motion {
            pose: Pose {
                position: [12.5, 48.25, -7.75],
                pivot: [1.75, 2.125, 0.5],
                rotation: [0., 0., f64::from(0.35f32.sin()), f64::from(0.35f32.cos())],
                ..Pose::default()
            },
            linear: [0.2, -0., -9.81],
            angular: [0.1, 0.2, 0.3],
        };
        let pose = frames::canonical_at(frames::requested(&motion), [3., 4., 5.]).unwrap();
        OwnerState {
            logical: LogicalIdentity {
                id: Id([7; 16]),
                plot: 3,
                generation: 41,
                parent: Some(Id([2; 16])),
            },
            current: RawState {
                motion: motion.clone(),
                native_epoch: 17,
                paused: false,
            },
            episode: Episode {
                profile: Profile::Split,
                initial: motion,
                return_offset: [-1020, -96, -1008],
                cells: BTreeMap::from([([1032, 128, 1032], "minecraft:blackstone".into())]),
            },
            phase: Phase::Projected {
                pose_revision: u64::MAX,
                pose,
            },
        }
    }
    #[test]
    fn only_explicit_open_demo_profile_allows_unpaused_200_and_paused_400() {
        for profile in [
            Profile::Translate,
            Profile::Yaw,
            Profile::Pitch,
            Profile::Dynamic,
            Profile::Split,
            Profile::OpenDemo,
        ] {
            for (epoch, paused) in [
                (199, false),
                (200, false),
                (200, true),
                (399, false),
                (400, false),
                (400, true),
                (401, true),
            ] {
                let mut original = owner();
                original.episode.profile = profile;
                original.current.native_epoch = epoch;
                original.current.paused = paused;
                let expected = if profile == Profile::OpenDemo {
                    epoch < 400 || (epoch == 400 && paused)
                } else {
                    epoch < 200 || (epoch == 200 && paused)
                };
                let encoded = encode_owner(&original);
                assert_eq!(
                    encoded.is_ok(),
                    expected,
                    "{profile:?} {epoch} paused={paused}"
                );
                if let Ok(encoded) = encoded {
                    let restored = decode_owner(&encoded).unwrap();
                    assert_eq!(restored.episode.profile, profile);
                    assert_eq!(restored.current.native_epoch, epoch);
                    assert_eq!(
                        restored.projected().unwrap().epoch_policy,
                        profile.epoch_policy()
                    );
                    assert_eq!(encode_owner(&restored).unwrap(), encoded);
                    if profile == Profile::OpenDemo && epoch >= 200 && !paused {
                        let mut forged = encoded;
                        let Some(Tag::Compound(episode)) = forged.get_mut(&"episode".into()) else {
                            unreachable!()
                        };
                        episode.insert("profile".into(), Tag::Int(Profile::Dynamic.number()));
                        assert!(decode_owner(&forged).is_err());
                    }
                }
            }
        }
    }
    #[test]
    fn all_phases_preserve_raw_bits_and_do_not_invent_projection_receipts() {
        let mut owner = owner();
        let original = encode_owner(&owner).unwrap();
        let mut repeated = original.clone();
        for _ in 0..10 {
            repeated = encode_owner(&decode_owner(&repeated).unwrap()).unwrap();
            assert_eq!(repeated, original);
        }
        let saved = decode_owner(&original).unwrap().projected().unwrap();
        assert_eq!(
            saved.linear.map(f64::to_bits),
            owner.current.motion.linear.map(f64::to_bits)
        );
        for phase in [
            Phase::OwnedUnprojected {
                barrier_epoch: 25,
                members: vec![[1032, 128, 1032]],
            },
            Phase::Returned { barrier_epoch: 26 },
            Phase::Retired {
                barrier_epoch: 27,
                generation: 9,
            },
        ] {
            owner.phase = phase;
            let encoded = encode_owner(&owner).unwrap();
            let restored = decode_owner(&encoded).unwrap();
            assert!(restored.projected().is_none());
            assert_eq!(encode_owner(&restored).unwrap(), encoded);
        }
    }
    #[test]
    fn malformed_owner_rejects_missing_barrier_duplicate_members_and_epoch_reset() {
        let mut owner = owner();
        owner.phase = Phase::OwnedUnprojected {
            barrier_epoch: 0,
            members: vec![[1032, 128, 1032]],
        };
        assert!(encode_owner(&owner).is_err());
        owner.phase = Phase::OwnedUnprojected {
            barrier_epoch: 1,
            members: vec![[1032, 128, 1032]; 2],
        };
        assert!(encode_owner(&owner).is_err());
        let mut data = encode_owner(&self::owner()).unwrap();
        data.insert("unknown".into(), Tag::Int(0));
        assert!(decode_owner(&data).is_err());
        let mut invalid = self::owner();
        invalid.current.native_epoch = 200;
        assert!(encode_owner(&invalid).is_err());
        invalid.current.paused = true;
        assert!(encode_owner(&invalid).is_ok());
        invalid.logical.generation = 0;
        assert!(encode_owner(&invalid).is_err());
    }
    #[test]
    fn registry_round_trip_retains_retired_highwater_and_refuses_duplicates() {
        let owner = owner().logical;
        let registry = PersistentState {
            owners: vec![owner],
            plot_generations: BTreeMap::from([(3, 41), (8, u64::MAX)]),
        };
        let data = encode_registry(&registry).unwrap();
        assert_eq!(decode_registry(&data).unwrap(), registry);
        let mut duplicate = registry.clone();
        duplicate.owners.push(owner);
        assert!(encode_registry(&duplicate).is_err());
        let mut regressed = registry;
        regressed.plot_generations.insert(3, 40);
        assert!(encode_registry(&regressed).is_err());
    }
}
