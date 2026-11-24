// Derived from Sable 9b1a3d1555d35db9fb7b1103701e7490f0a968b9.
// PolyForm Shield 1.0.0; see ../LICENSE-SABLE.md and provenance/origin.json.
use rapier3d::prelude::{Group, InteractionGroups, InteractionTestMode};

pub const LEVEL_GROUP: InteractionGroups = InteractionGroups::new(
    Group::GROUP_1,
    Group::GROUP_1.union(Group::GROUP_2),
    InteractionTestMode::Or,
);
pub const ROPE_GROUP: InteractionGroups =
    InteractionGroups::new(Group::GROUP_2, Group::GROUP_1, InteractionTestMode::Or);
