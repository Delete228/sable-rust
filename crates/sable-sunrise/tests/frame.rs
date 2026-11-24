use sable_core::*;
use sable_sunrise::frame::{Frame, FrameError};

fn setup() -> (Registry, Handle) {
    let mut registry = Registry::new(Config::default()).unwrap();
    let mut level = Level::new(Id([1; 16]));
    level.motion.pose.position = [1e6, 10., -1e6];
    level.motion.pose.pivot = [16., 0., 16.];
    level.motion.pose.rotation = [0., 0., 1., 0.];
    level.motion.linear = [1., 2., 3.];
    level.motion.angular = [0., 0., 2.];
    level.contents.blocks.insert(
        [16, 0, 16],
        Block {
            state: "minecraft:blackstone".into(),
            block_entity: None,
            extension: Payload::default(),
        },
    );
    let h = registry.insert(level).unwrap();
    (registry, h)
}

#[test]
fn current_frame_refuses_stale_epochs_revisions_generations_and_unreachable_nonfinite_hits() {
    let (mut registry, h) = setup();
    let frame = Frame::publish(&registry, h, 7, None).unwrap();
    let next = Frame::publish(&registry, h, 8, Some(&frame)).unwrap();
    assert_eq!(
        Frame::publish(&registry, h, 7, Some(&frame)),
        Err(FrameError::Stale)
    );
    let local = [16., 0., 16.];
    let eye = frame.current_pose().transform(local);
    assert_eq!(
        frame.check_target(&registry, &frame, local, eye, 0.),
        Ok(())
    );
    assert_eq!(
        next.check_target(&registry, &frame, local, eye, 4.),
        Err(FrameError::Stale)
    );
    assert_eq!(
        frame.check_target(&registry, &frame, [30., 0., 16.], eye, 4.),
        Err(FrameError::OutOfReach)
    );
    assert_eq!(
        frame.check_target(&registry, &frame, local, eye, f64::INFINITY),
        Err(FrameError::Invalid)
    );
    assert_eq!(
        frame.check_target(&registry, &frame, [f64::NAN; 3], eye, 4.),
        Err(FrameError::Invalid)
    );
    let before = persistence::encode(&registry).unwrap();
    assert_eq!(
        frame.check_target(&registry, &frame, [f64::MAX; 3], eye, 4.),
        Err(FrameError::OutOfReach)
    );
    assert_eq!(before, persistence::encode(&registry).unwrap());
    let block = registry.get(h).unwrap().contents.blocks[&[16, 0, 16]].clone();
    registry.edit(h, [16, 0, 16], Some(block)).unwrap();
    assert_eq!(
        frame.check_target(&registry, &frame, local, eye, 4.),
        Err(FrameError::Stale)
    );
    let level = registry.delete(h).unwrap();
    let fresh = registry.insert(level).unwrap();
    assert_ne!(h, fresh);
    assert_eq!(
        Frame::publish(&registry, fresh, 9, Some(&frame)),
        Err(FrameError::Stale)
    );
}

#[test]
fn render_nlerp_is_stateless_and_support_velocity_uses_explicit_world_com() {
    let (registry, h) = setup();
    let first = Frame::publish(&registry, h, 0, None).unwrap();
    assert_eq!(
        first
            .point_velocity([100., 3., 7.], [102., 3., 7.])
            .unwrap(),
        [1., 6., 3.]
    );
    let mut moved = registry.get(h).unwrap().clone();
    moved.motion.pose.position[0] += 4.;
    moved.motion.pose.rotation = [0., 0., -1., 0.];
    // Restore gives another generation, so only start endpoints in that owner.
    let restored =
        Registry::restore(registry.config().clone(), vec![(moved, Residency::Live)]).unwrap();
    let h2 = restored.handle(h.id()).unwrap();
    let frame = Frame::publish(&restored, h2, 1, None).unwrap();
    for _ in 0..100 {
        let pose = frame.render_pose(0.5).unwrap();
        assert!(pose.valid());
        assert_eq!(pose.position, frame.current_pose().position);
        assert_eq!(
            frame.render_pose(0.).unwrap().transform([1., 0., 0.]),
            pose.transform([1., 0., 0.])
        );
    }
    assert_eq!(frame.render_pose(-0.01), Err(FrameError::Invalid));
    assert_eq!(frame.render_pose(f64::NAN), Err(FrameError::Invalid));
}
