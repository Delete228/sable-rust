//! Concrete mapping to this candidate Sunrise pose convention. Host
//! normalization never feeds back into the accepted native simulation state.
//! Derived pose-normalization/reanchoring mapping from Sunrise; see
//! ../LICENSE-SUNRISE and ../provenance/origin.json for source identities.
use mod_api::LiveWorldPose;
use sable_core::Motion;

pub fn requested(motion: &Motion) -> LiveWorldPose {
    LiveWorldPose {
        translation: std::array::from_fn(|axis| {
            motion.pose.position[axis] - motion.pose.pivot[axis]
        }),
        pivot: motion.pose.pivot,
        rotation: motion.pose.rotation,
    }
}

/// This mirrors the audited candidate's public-pose interpretation: normalize
/// xyzw, canonicalize quaternion sign, then preserve the binding's fixed pivot.
/// Arithmetic order follows its Rotation/RigidTransform mapping; no tolerance
/// or private engine dependency is used. Keep this replaceable SDK adapter
/// mapping separate from native pose arithmetic.
pub fn canonical_at(input: LiveWorldPose, pivot: [f64; 3]) -> Option<LiveWorldPose> {
    if input
        .translation
        .iter()
        .chain(&input.pivot)
        .chain(&input.rotation)
        .chain(&pivot)
        .any(|v| !v.is_finite())
    {
        return None;
    }
    let [x, y, z, w] = input.rotation;
    let scale = w.abs().max(x.abs().max(y.abs()).max(z.abs()));
    if scale == 0. {
        return None;
    }
    let (w, x, y, z) = (w / scale, x / scale, y / scale, z / scale);
    let norm = (w * w + x * x + y * y + z * z).sqrt();
    let (mut w, mut x, mut y, mut z) = (w / norm, x / norm, y / norm, z / norm);
    if w < 0. || (w == 0. && (x < 0. || (x == 0. && (y < 0. || (y == 0. && z < 0.))))) {
        w = -w;
        x = -x;
        y = -y;
        z = -z;
    }
    let q = [x, y, z];
    let local: [f64; 3] = std::array::from_fn(|axis| pivot[axis] - input.pivot[axis]);
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let t = cross(q, local).map(|v| v * 2.);
    let second = cross(q, t);
    let rotated: [f64; 3] = std::array::from_fn(|axis| (local[axis] + t[axis] * w) + second[axis]);
    Some(LiveWorldPose {
        translation: std::array::from_fn(|axis| {
            ((input.translation[axis] + input.pivot[axis]) + rotated[axis]) - pivot[axis]
        }),
        pivot,
        rotation: [x, y, z, w],
    })
}

/// `project` preserves supplied fields; `publish_pose` returns the canonical
/// reparameterization. These are the two explicit mappings, compared exactly
/// (IEEE positive/negative zero denote the same coordinate).
pub fn equivalent(motion: &Motion, accepted: LiveWorldPose) -> bool {
    let input = requested(motion);
    input == accepted || canonical_at(input, accepted.pivot) == Some(accepted)
}

#[cfg(test)]
mod tests {
    use super::*;
    use sable_core::Pose;
    #[test]
    fn normalized_host_frame_keeps_raw_native_frame_distinct() {
        let raw = Motion {
            pose: Pose {
                position: [12.5, 48.25, -7.75],
                pivot: [1.75, 2.125, -0.5],
                rotation: [0., 0., f64::from(0.35f32.sin()), f64::from(0.35f32.cos())],
                ..Pose::default()
            },
            ..Motion::default()
        };
        let before = raw.clone();
        let host = canonical_at(requested(&raw), [3., 4., 5.]).unwrap();
        assert!(equivalent(&raw, host));
        assert_eq!(raw, before);
        assert_ne!(
            host.rotation.map(f64::to_bits),
            raw.pose.rotation.map(f64::to_bits)
        );
        let mut invalid = host;
        invalid.translation[0] += 0.25;
        assert!(!equivalent(&raw, invalid));
    }
}
