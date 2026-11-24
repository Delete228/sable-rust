//! Owned publication data and preflight only. No host action or contact route.
use sable_core::{Handle, Motion, Pose, Registry, Residency, add, cross, sub};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    Stale,
    Invalid,
    OutOfReach,
}

/// A single local world's immutable pose endpoints. Handles include registry
/// ownership and generation; epochs are supplied by the serialized tick owner.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    handle: Handle,
    revision: u64,
    epoch: u64,
    previous: Pose,
    motion: Motion,
}

impl Frame {
    /// The caller captures actual physics into the registry before publishing.
    /// This constructor cannot establish a host save/transfer barrier.
    pub fn publish(
        registry: &Registry,
        handle: Handle,
        epoch: u64,
        previous: Option<&Self>,
    ) -> Result<Self, FrameError> {
        let level = registry.get(handle).map_err(|_| FrameError::Stale)?;
        if registry.residency(handle) != Ok(Residency::Live) {
            return Err(FrameError::Stale);
        }
        if !level.motion.pose.valid()
            || !level
                .motion
                .linear
                .iter()
                .chain(&level.motion.angular)
                .all(|v| v.is_finite())
        {
            return Err(FrameError::Invalid);
        }
        if previous.is_some_and(|p| p.handle != handle || epoch <= p.epoch) {
            return Err(FrameError::Stale);
        }
        Ok(Self {
            handle,
            revision: level.revision,
            epoch,
            previous: previous.map_or(level.motion.pose, |p| p.motion.pose),
            motion: level.motion.clone(),
        })
    }

    pub fn epoch(&self) -> u64 {
        self.epoch
    }
    pub fn current_pose(&self) -> Pose {
        self.motion.pose
    }

    /// Companion Pose3dc uses linear position/pivot/scale and quaternion nlerp.
    /// This is the same convention, not a bitwise JOML interpolation claim.
    pub fn render_pose(&self, alpha: f64) -> Result<Pose, FrameError> {
        if !alpha.is_finite() || !(0.0..=1.0).contains(&alpha) {
            return Err(FrameError::Invalid);
        }
        let a = self.previous;
        let b = self.motion.pose;
        let lerp =
            |x: [f64; 3], y: [f64; 3]| std::array::from_fn(|i| x[i] * (1. - alpha) + y[i] * alpha);
        let dot: f64 = a.rotation.iter().zip(b.rotation).map(|(x, y)| x * y).sum();
        let sign = if dot < 0. { -1. } else { 1. };
        let mut q: [f64; 4] =
            std::array::from_fn(|i| a.rotation[i] * (1. - alpha) + sign * b.rotation[i] * alpha);
        let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        for v in &mut q {
            *v /= norm;
        }
        let pose = Pose {
            position: lerp(a.position, b.position),
            rotation: q,
            pivot: lerp(a.pivot, b.pivot),
            scale: lerp(a.scale, b.scale),
        };
        if !pose.valid() {
            return Err(FrameError::Invalid);
        }
        Ok(pose)
    }

    /// World m/s at a support point: world COM is supplied explicitly, never
    /// inferred from the plot origin or logical pose position.
    pub fn point_velocity(
        &self,
        world_com: [f64; 3],
        point: [f64; 3],
    ) -> Result<[f64; 3], FrameError> {
        if !world_com.iter().chain(&point).all(|v| v.is_finite()) {
            return Err(FrameError::Invalid);
        }
        let velocity = add(
            self.motion.linear,
            cross(self.motion.angular, sub(point, world_com)),
        );
        if !velocity.iter().all(|v| v.is_finite()) {
            return Err(FrameError::Invalid);
        }
        Ok(velocity)
    }

    /// Early refusal guard. Successful preflight is NOT host authorization:
    /// actual local ray/shape/occlusion, permissions and action semantics must
    /// still run synchronously through the missing authoritative host service.
    pub fn check_target(
        &self,
        registry: &Registry,
        observed: &Self,
        local_hit: [f64; 3],
        eye: [f64; 3],
        reach: f64,
    ) -> Result<(), FrameError> {
        if self != observed {
            return Err(FrameError::Stale);
        }
        let level = registry.get(self.handle).map_err(|_| FrameError::Stale)?;
        if registry.residency(self.handle) != Ok(Residency::Live)
            || level.revision != self.revision
            || level.motion != self.motion
        {
            return Err(FrameError::Stale);
        }
        if !reach.is_finite() || reach < 0. || !local_hit.iter().chain(&eye).all(|v| v.is_finite())
        {
            return Err(FrameError::Invalid);
        }
        let hit = self.motion.pose.transform(local_hit);
        let distance = sub(hit, eye).into_iter().fold(0.0_f64, |d, v| d.hypot(v));
        if !distance.is_finite() || distance > reach {
            return Err(FrameError::OutOfReach);
        }
        Ok(())
    }
}

#[cfg(test)]
mod review_tests {
    use super::*;
    use sable_core::{Config, Id, Level};

    fn frame_with_endpoints(previous: Pose, current: Pose) -> Frame {
        let mut registry = Registry::new(Config::default()).unwrap();
        let mut level = Level::new(Id([11; 16]));
        level.motion.pose = current;
        let handle = registry.insert(level).unwrap();
        let mut frame = Frame::publish(&registry, handle, 1, None).unwrap();
        // Arithmetic fixture only. This deliberately does not claim that a host
        // or physics owner captured either endpoint in an actual game.
        frame.previous = previous;
        frame
    }

    fn near(actual: [f64; 3], expected: [f64; 3]) {
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert!((actual - expected).abs() <= 2e-14, "{actual} != {expected}");
        }
    }

    #[test]
    fn different_endpoints_interpolate_all_pose_channels_and_do_not_accumulate() {
        let previous = Pose {
            position: [-1., 2., 3.],
            pivot: [16., 5., 8.],
            ..Pose::default()
        };
        let current = Pose {
            position: [3., 10., -1.],
            rotation: [
                0.,
                0.,
                std::f64::consts::FRAC_1_SQRT_2,
                std::f64::consts::FRAC_1_SQRT_2,
            ],
            pivot: [18., 7., 10.],
            scale: [2.; 3],
        };
        let frame = frame_with_endpoints(previous, current);
        let halfway = frame.render_pose(0.5).unwrap();
        assert_eq!(halfway.position, [1., 6., 1.]);
        assert_eq!(halfway.pivot, [17., 6., 9.]);
        assert_eq!(halfway.scale, [1.5; 3]);
        near(
            halfway.transform([19., 6., 9.]),
            [
                1. + 3. * std::f64::consts::FRAC_1_SQRT_2,
                6. + 3. * std::f64::consts::FRAC_1_SQRT_2,
                1.,
            ],
        );
        near(
            frame.render_pose(0.).unwrap().transform([19., 6., 9.]),
            previous.transform([19., 6., 9.]),
        );
        near(
            frame.render_pose(1.).unwrap().transform([19., 6., 9.]),
            current.transform([19., 6., 9.]),
        );
        for _ in 0..100 {
            assert_eq!(frame.render_pose(0.5).unwrap(), halfway);
        }
        assert_eq!(frame.current_pose(), current);
    }

    #[test]
    fn antipodal_quaternions_take_the_short_path_with_moving_endpoints() {
        let previous = Pose {
            rotation: [0., 0.6, 0., 0.8],
            ..Pose::default()
        };
        let current = Pose {
            position: [4., -2., 8.],
            rotation: previous.rotation.map(|value| -value),
            ..Pose::default()
        };
        let frame = frame_with_endpoints(previous, current);
        for alpha in [0., 0.25, 0.5, 0.75, 1.] {
            let pose = frame.render_pose(alpha).unwrap();
            assert!(pose.valid());
            near(pose.position, [4. * alpha, -2. * alpha, 8. * alpha]);
            near(
                sub(pose.transform([1., 0., 0.]), pose.position),
                previous.transform([1., 0., 0.]),
            );
        }
    }
}
