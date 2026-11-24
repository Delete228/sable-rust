//! Physical metadata lives in the real source level's existing persistent
//! attachment store. It is written before the host checkpoint, never after save.
use crate::frames;
use mod_api::*;
use sable_core::{Motion, Pose};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EpochPolicy {
    Review,
    OpenDemo,
}
impl EpochPolicy {
    pub fn name(self) -> &'static str {
        match self {
            Self::Review => "review",
            Self::OpenDemo => "open_demo",
        }
    }
    pub fn limit(self) -> u64 {
        match self {
            Self::Review => 200,
            Self::OpenDemo => 400,
        }
    }
    pub fn valid(self, epoch: u64, paused: bool) -> bool {
        epoch < self.limit() || (epoch == self.limit() && paused)
    }
}

#[derive(Clone)]
pub struct Saved {
    pub epoch_policy: EpochPolicy,
    pub deleted: bool,
    pub paused: bool,
    pub pose_revision: u64,
    pub native_epoch: u64,
    pub pose: LiveWorldPose,
    pub raw_pose: Pose,
    pub linear: [f64; 3],
    pub angular: [f64; 3],
}
impl Saved {
    fn valid(&self) -> bool {
        self.epoch_policy.valid(self.native_epoch, self.paused)
            && Pose {
                position: std::array::from_fn(|axis| {
                    self.pose.translation[axis] + self.pose.pivot[axis]
                }),
                pivot: self.pose.pivot,
                rotation: self.pose.rotation,
                scale: [1.; 3],
            }
            .valid()
            && self
                .linear
                .iter()
                .chain(&self.angular)
                .all(|v| v.is_finite() && (*v as f32).is_finite())
            && self.raw_pose.valid()
            && self.raw_pose.scale == [1.; 3]
            && self
                .raw_pose
                .position
                .iter()
                .chain(&self.raw_pose.pivot)
                .chain(&self.raw_pose.rotation)
                .all(|v| (*v as f32).is_finite())
            && frames::equivalent(
                &Motion {
                    pose: self.raw_pose,
                    linear: self.linear,
                    angular: self.angular,
                },
                self.pose,
            )
    }
    pub fn same_bits(&self, other: &Self) -> bool {
        match (encode(self), encode(other)) {
            (Ok(left), Ok(right)) => left == right,
            _ => false,
        }
    }
    pub fn matches(
        &self,
        motion: &Motion,
        sample: &LiveWorldSample,
        deleted: bool,
        paused: bool,
        native_epoch: u64,
    ) -> bool {
        self.deleted == deleted
            && self.paused == paused
            && self.pose_revision == sample.revision
            && self.native_epoch == native_epoch
            && self.pose.translation.map(f64::to_bits)
                == sample.current.translation.map(f64::to_bits)
            && self.pose.pivot.map(f64::to_bits) == sample.current.pivot.map(f64::to_bits)
            && self.pose.rotation.map(f64::to_bits) == sample.current.rotation.map(f64::to_bits)
            && self.raw_pose.position.map(f64::to_bits) == motion.pose.position.map(f64::to_bits)
            && self.raw_pose.pivot.map(f64::to_bits) == motion.pose.pivot.map(f64::to_bits)
            && self.raw_pose.rotation.map(f64::to_bits) == motion.pose.rotation.map(f64::to_bits)
            && self.raw_pose.scale.map(f64::to_bits) == motion.pose.scale.map(f64::to_bits)
            && self.linear.map(f64::to_bits) == motion.linear.map(f64::to_bits)
            && self.angular.map(f64::to_bits) == motion.angular.map(f64::to_bits)
    }
    pub fn new(
        motion: &Motion,
        sample: &LiveWorldSample,
        deleted: bool,
        paused: bool,
        native_epoch: u64,
    ) -> Self {
        Self {
            epoch_policy: EpochPolicy::Review,
            deleted,
            paused,
            pose_revision: sample.revision,
            native_epoch,
            pose: sample.current,
            raw_pose: motion.pose,
            linear: motion.linear,
            angular: motion.angular,
        }
    }
}
fn encode(saved: &Saved) -> Result<Compound, AttachmentError> {
    if !saved.valid() {
        return Err(AttachmentError::Encode);
    }
    let values = saved
        .pose
        .translation
        .iter()
        .chain(&saved.pose.pivot)
        .chain(&saved.pose.rotation)
        .chain(&saved.raw_pose.position)
        .chain(&saved.raw_pose.pivot)
        .chain(&saved.raw_pose.rotation)
        .chain(&saved.linear)
        .chain(&saved.angular)
        .map(|value| Tag::Double(value.to_bits()))
        .collect();
    let mut data: Compound = [
        ("version".into(), Tag::Int(3)),
        ("deleted".into(), Tag::Byte(i8::from(saved.deleted))),
        ("paused".into(), Tag::Byte(i8::from(saved.paused))),
        (
            "pose_revision".into(),
            Tag::Long(i64::from_le_bytes(saved.pose_revision.to_le_bytes())),
        ),
        ("native_epoch".into(), Tag::Long(saved.native_epoch as i64)),
        ("values".into(), Tag::List(values)),
    ]
    .into();
    if saved.epoch_policy == EpochPolicy::OpenDemo {
        data.insert("version".into(), Tag::Int(4));
        data.insert("profile".into(), Tag::String("open_demo".into()));
        data.insert("epoch_limit".into(), Tag::Int(400));
    }
    Ok(data)
}
fn decode(data: &Compound) -> Result<Saved, AttachmentError> {
    let epoch_policy = match data.get(&"version".into()) {
        Some(Tag::Int(3)) if data.len() == 6 => EpochPolicy::Review,
        Some(Tag::Int(4))
            if data.len() == 8
                && data.get(&"profile".into()) == Some(&Tag::String("open_demo".into()))
                && data.get(&"epoch_limit".into()) == Some(&Tag::Int(400)) =>
        {
            EpochPolicy::OpenDemo
        }
        _ => return Err(AttachmentError::Decode),
    };
    let deleted = match data.get(&"deleted".into()) {
        Some(Tag::Byte(0)) => false,
        Some(Tag::Byte(1)) => true,
        _ => return Err(AttachmentError::Decode),
    };
    let paused = match data.get(&"paused".into()) {
        Some(Tag::Byte(0)) => false,
        Some(Tag::Byte(1)) => true,
        _ => return Err(AttachmentError::Decode),
    };
    let revision = match data.get(&"pose_revision".into()) {
        Some(Tag::Long(value)) => u64::from_le_bytes(value.to_le_bytes()),
        _ => return Err(AttachmentError::Decode),
    };
    let native_epoch = match data.get(&"native_epoch".into()) {
        Some(Tag::Long(value)) if (0..=epoch_policy.limit() as i64).contains(value) => {
            *value as u64
        }
        _ => return Err(AttachmentError::Decode),
    };
    let Some(Tag::List(values)) = data.get(&"values".into()) else {
        return Err(AttachmentError::Decode);
    };
    if values.len() != 26 {
        return Err(AttachmentError::Decode);
    }
    let values = values
        .iter()
        .map(|value| match value {
            Tag::Double(bits) => Ok(f64::from_bits(*bits)),
            _ => Err(AttachmentError::Decode),
        })
        .collect::<Result<Vec<_>, _>>()?;
    let saved = Saved {
        epoch_policy,
        deleted,
        paused,
        pose_revision: revision,
        native_epoch,
        pose: LiveWorldPose {
            translation: values[..3].try_into().unwrap(),
            pivot: values[3..6].try_into().unwrap(),
            rotation: values[6..10].try_into().unwrap(),
        },
        raw_pose: Pose {
            position: values[10..13].try_into().unwrap(),
            pivot: values[13..16].try_into().unwrap(),
            rotation: values[16..20].try_into().unwrap(),
            scale: [1.; 3],
        },
        linear: values[20..23].try_into().unwrap(),
        angular: values[23..26].try_into().unwrap(),
    };
    if !saved.valid() {
        return Err(AttachmentError::Decode);
    }
    Ok(saved)
}
pub static MOTION: AttachmentType<Saved> = AttachmentType {
    namespace: "sable_port",
    name: "motion",
    default: || Saved {
        epoch_policy: EpochPolicy::Review,
        deleted: false,
        paused: false,
        pose_revision: 0,
        native_epoch: 0,
        pose: LiveWorldPose::at([0.; 3]),
        raw_pose: Pose::default(),
        linear: [0.; 3],
        angular: [0.; 3],
    },
    codec: Some(AttachmentCodec { encode, decode }),
    copy_on_death: false,
    sync: false,
};

#[cfg(test)]
mod tests {
    use super::*;
    fn sample() -> Saved {
        Saved {
            epoch_policy: EpochPolicy::Review,
            deleted: false,
            paused: true,
            pose_revision: u64::MAX,
            native_epoch: 123,
            pose: LiveWorldPose {
                translation: [-17.25, 48., 2_000_000.],
                pivot: [16.5, 0.5, 2.],
                rotation: [0., 0., 0., 1.],
            },
            raw_pose: Pose {
                position: [-0.75, 48.5, 2_000_002.],
                pivot: [16.5, 0.5, 2.],
                ..Pose::default()
            },
            linear: [0.2, -0., 1e-6],
            angular: [0., 0.1, -0.2],
        }
    }
    #[test]
    fn epoch_policy_round_trip_preserves_review_v3_and_scopes_open_v4_to_400() {
        for (policy, epoch, paused, allowed) in [
            (EpochPolicy::Review, 199, false, true),
            (EpochPolicy::Review, 200, false, false),
            (EpochPolicy::Review, 200, true, true),
            (EpochPolicy::Review, 201, true, false),
            (EpochPolicy::OpenDemo, 199, false, true),
            (EpochPolicy::OpenDemo, 200, false, true),
            (EpochPolicy::OpenDemo, 399, false, true),
            (EpochPolicy::OpenDemo, 400, false, false),
            (EpochPolicy::OpenDemo, 400, true, true),
            (EpochPolicy::OpenDemo, 401, true, false),
        ] {
            let original = Saved {
                epoch_policy: policy,
                native_epoch: epoch,
                paused,
                ..sample()
            };
            let encoded = encode(&original);
            assert_eq!(
                encoded.is_ok(),
                allowed,
                "{policy:?} {epoch} paused={paused}"
            );
            if let Ok(encoded) = encoded {
                assert_eq!(
                    encoded.len(),
                    if policy == EpochPolicy::Review { 6 } else { 8 }
                );
                assert_eq!(
                    encoded.get(&"version".into()),
                    Some(&Tag::Int(if policy == EpochPolicy::Review { 3 } else { 4 }))
                );
                let restored = decode(&encoded).unwrap();
                assert_eq!(restored.epoch_policy, policy);
                assert_eq!(restored.native_epoch, epoch);
                assert_eq!(encode(&restored).unwrap(), encoded);
            }
        }
    }
    #[test]
    fn open_v4_refuses_missing_or_arbitrary_profile_and_budget_tags() {
        let open = Saved {
            epoch_policy: EpochPolicy::OpenDemo,
            native_epoch: 400,
            ..sample()
        };
        let encoded = encode(&open).unwrap();
        let mut wrong = encoded.clone();
        wrong.insert("native_epoch".into(), Tag::Long(401));
        assert!(decode(&wrong).is_err());
        wrong = encoded.clone();
        wrong.insert("paused".into(), Tag::Byte(0));
        assert!(decode(&wrong).is_err());
        wrong = encoded.clone();
        wrong.insert("epoch_limit".into(), Tag::Int(401));
        assert!(decode(&wrong).is_err());
        wrong.insert("epoch_limit".into(), Tag::Int(200));
        assert!(decode(&wrong).is_err());
        wrong = encoded.clone();
        wrong.insert("profile".into(), Tag::String("review".into()));
        assert!(decode(&wrong).is_err());
        wrong = encoded.clone();
        wrong.remove(&"profile".into());
        assert!(decode(&wrong).is_err());
        wrong = encoded;
        wrong.insert("version".into(), Tag::Int(3));
        assert!(decode(&wrong).is_err());
    }
    #[test]
    fn motion_attachment_preserves_revision_and_velocity_bits() {
        let original = sample();
        let restored = decode(&encode(&original).unwrap()).unwrap();
        assert_eq!(restored.pose_revision, u64::MAX);
        assert_eq!(restored.native_epoch, 123);
        assert_eq!(restored.pose, original.pose);
        assert_eq!(
            restored.linear.map(f64::to_bits),
            original.linear.map(f64::to_bits)
        );
        assert_eq!(
            restored.angular.map(f64::to_bits),
            original.angular.map(f64::to_bits)
        );
        assert!(restored.paused);
        assert!(!restored.deleted);
    }
    #[test]
    fn publication_metadata_matches_exact_raw_and_host_bits_at_one_epoch() {
        let saved = sample();
        let motion = Motion {
            pose: saved.raw_pose,
            linear: saved.linear,
            angular: saved.angular,
        };
        let sample = LiveWorldSample::new(saved.pose, saved.pose, saved.pose_revision);
        assert!(saved.matches(&motion, &sample, false, true, 123));
        assert!(!saved.matches(&motion, &sample, false, true, 122));
        assert!(!saved.matches(&motion, &sample, false, false, 123));
        let mut changed = motion.clone();
        changed.linear[1] = 0.;
        assert!(!saved.matches(&changed, &sample, false, true, 123));
        let mut changed = sample;
        changed.current.rotation[0] = -0.;
        assert!(!saved.matches(&motion, &changed, false, true, 123));
    }
    #[test]
    fn malformed_or_nonfinite_motion_attachment_refuses() {
        let valid = encode(&sample()).unwrap();
        let mut unsupported = valid.clone();
        unsupported.insert("version".into(), Tag::Int(1));
        assert!(decode(&unsupported).is_err());
        unsupported.insert("version".into(), Tag::Int(2));
        assert!(decode(&unsupported).is_err());
        let mut extra = valid.clone();
        extra.insert("unknown".into(), Tag::Int(0));
        assert!(decode(&extra).is_err());
        let mut flag = valid.clone();
        flag.insert("deleted".into(), Tag::Byte(2));
        assert!(decode(&flag).is_err());
        let mut budget = valid.clone();
        budget.insert("native_epoch".into(), Tag::Long(201));
        assert!(decode(&budget).is_err());
        budget.insert("native_epoch".into(), Tag::Long(200));
        budget.insert("paused".into(), Tag::Byte(0));
        assert!(decode(&budget).is_err());
        let mut nonfinite = valid;
        let Some(Tag::List(values)) = nonfinite.get_mut(&"values".into()) else {
            unreachable!()
        };
        values[20] = Tag::Double(f64::NAN.to_bits());
        assert!(decode(&nonfinite).is_err());
        let Some(Tag::List(values)) = nonfinite.get_mut(&"values".into()) else {
            unreachable!()
        };
        values.pop();
        assert!(decode(&nonfinite).is_err());
    }
    #[test]
    fn rotating_raw_and_canonical_host_frames_round_trip_without_renormalizing() {
        let raw = Motion {
            pose: Pose {
                position: [12.5, 48.25, -7.75],
                pivot: [1.75, 2.125, -0.5],
                rotation: [0., 0., f64::from(0.35f32.sin()), f64::from(0.35f32.cos())],
                ..Pose::default()
            },
            linear: [0.2, -0., -9.81],
            angular: [0.1, 0.2, 0.3],
        };
        let saved = Saved {
            pose: frames::canonical_at(frames::requested(&raw), [3., 4., 5.]).unwrap(),
            raw_pose: raw.pose,
            linear: raw.linear,
            angular: raw.angular,
            ..sample()
        };
        let original = encode(&saved).unwrap();
        let mut repeated = original.clone();
        for _ in 0..10 {
            repeated = encode(&decode(&repeated).unwrap()).unwrap();
            assert_eq!(repeated, original);
        }
        let mut mismatch = original;
        let Some(Tag::List(values)) = mismatch.get_mut(&"values".into()) else {
            unreachable!()
        };
        let Tag::Double(value) = values[0] else {
            unreachable!()
        };
        values[0] = Tag::Double((f64::from_bits(value) + 0.25).to_bits());
        assert!(decode(&mismatch).is_err());
    }
}
