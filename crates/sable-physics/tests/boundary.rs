use sable_physics::*;
use std::sync::{Arc, Mutex};
fn mass() -> BodyMass {
    BodyMass {
        mass: 1.,
        center: [64.5, 0.5, 64.5],
        inertia: [1. / 6., 0., 0., 0., 1. / 6., 0., 0., 0., 1. / 6.],
    }
}
fn material(contact: Option<Arc<dyn ContactPolicy>>) -> Material {
    Material {
        friction: 1.,
        volume: 1.,
        restitution: 0.,
        fluid: false,
        dynamic: false,
        boxes: vec![[0., 0., 0., 1., 1., 1.]],
        contact,
    }
}
#[derive(Debug, Default)]
struct Observation {
    calls: usize,
    before: Option<(u32, u8)>,
    after: Option<(u32, u8)>,
    reentrant: Option<Error>,
}
#[derive(Debug)]
struct RemoveImmediately(Arc<Mutex<Observation>>);
impl ContactPolicy for RemoveImmediately {
    fn on_contact(&self, _: ContactRequest, world: &dyn ContactWorld) -> ContactResponse {
        let mut o = self.0.lock().unwrap();
        o.calls += 1;
        if o.calls == 1 {
            o.before = world.block([0, 0, 0]);
            world.remove_block([0, 0, 0]).unwrap();
            o.after = world.block([0, 0, 0]);
            o.reentrant = Engine::try_new().err();
        }
        ContactResponse {
            tangent: [0.; 3],
            reject: true,
        }
    }
}
#[test]
fn ownership_checked_inputs_global_materials_and_synchronous_contact() {
    let mut e = Engine::try_new().unwrap();
    assert_eq!(Engine::try_new().err(), Some(Error::ProcessBusy));
    assert_eq!(e.create_scene([0.; 3], f64::MAX), Err(Error::InvalidInput));
    let s = e.create_scene([0.; 3], 0.).unwrap();
    assert_eq!(
        e.create_box(s, 1, f64::MAX, [0.5; 3], [0., 0., 0., 0., 0., 0., 1.]),
        Err(Error::InvalidInput)
    );
    let b = e
        .create_box(s, 1, 2., [0.5; 3], [0., 0., 0., 0., 0., 0., 1.])
        .unwrap();
    let material_handle = e.register_material(material(None)).unwrap();
    e.add_material_box(material_handle, [0., 0., 0., 1., 1.5, 1.])
        .unwrap(); // original ordered boxes may extend outside a cell
    assert_eq!(
        e.upload_chunk(s, [i32::MAX, 0, 0], true, None, vec![]),
        Err(Error::InvalidInput)
    );
    assert_eq!(
        e.upload_chunk(s, [0; 3], false, None, vec![]),
        Err(Error::InvalidInput)
    );
    assert_eq!(
        e.upload_chunk(s, [0; 3], false, Some(b), vec![]),
        Err(Error::InvalidInput)
    );
    assert_eq!(
        e.upload_chunk(s, [0; 3], true, Some(b), vec![]),
        Err(Error::InvalidInput)
    );
    assert_eq!(
        e.upload_chunk(
            s,
            [0; 3],
            true,
            None,
            vec![([16, 0, 0], material_handle, 3)]
        ),
        Err(Error::InvalidInput)
    );
    assert_eq!(
        e.upload_chunk(s, [0; 3], true, None, vec![([0; 3], material_handle, 5)]),
        Err(Error::InvalidInput)
    );
    e.step(s, 0.025).unwrap();
    e.impulse(b, [2., 0., 0.], [0.; 3], true).unwrap();
    assert_eq!(e.velocity(b).unwrap()[0], 1.); // independent J/m invariant
    let joint = e
        .fixed_joint(s, Some(b), None, [0.; 6], [0., 0., 0., 1.])
        .unwrap();
    e.remove_body(b).unwrap();
    assert_eq!(e.pose(b), Err(Error::StaleBody));
    assert!(!e.joint_sample(joint).unwrap().0);
    let replacement = e
        .create_box(s, 1, 1., [0.5; 3], [0., 0., 0., 0., 0., 0., 1.])
        .unwrap();
    assert_ne!(b, replacement);
    assert_eq!(e.velocity(b), Err(Error::StaleBody));
    e.dispose_scene(s).unwrap();
    assert_eq!(e.step(s, 0.025), Err(Error::StaleScene));
    drop(e);
    let mut e = Engine::try_new().unwrap();
    let scene = e.create_scene([0., -9.81, 0.], 0.).unwrap();
    e.clear_material_boxes(material_handle).unwrap();
    e.add_material_box(material_handle, [0., 0., 0., 1., 1., 1.])
        .unwrap(); // process-global material outlives all scenes/owners
    e.configure([18, 2, 2], [40., 5.], 128).unwrap();
    let observed = Arc::new(Mutex::new(Observation::default()));
    let floor = e
        .register_material(material(Some(Arc::new(RemoveImmediately(Arc::clone(
            &observed,
        ))))))
        .unwrap();
    let body_material = e.register_material(material(None)).unwrap();
    e.upload_chunk(scene, [0; 3], true, None, vec![([0; 3], floor, 3)])
        .unwrap();
    let body = e
        .create_sublevel(
            scene,
            1,
            [0.5, 1.45, 0.5, 0., 0., 0., 1.],
            [64, 0, 64, 64, 0, 64],
            mass(),
        )
        .unwrap();
    e.upload_chunk(
        scene,
        [4, 0, 4],
        false,
        Some(body),
        vec![([0; 3], body_material, 3)],
    )
    .unwrap();
    for _ in 0..24 {
        e.step(scene, 0.025).unwrap();
    }
    let o = observed.lock().unwrap();
    assert!(o.calls > 0);
    assert!(o.before.unwrap().0 > 0);
    assert_eq!(o.after, Some((0, 0)));
    assert_eq!(o.reentrant, Some(Error::ReentrantOperation));
    assert!(e.pose(body).unwrap()[1] < 0.);
    assert!(e.collisions(scene).unwrap().is_empty());
}
#[test]
fn malformed_legacy_geometry_panics_without_unchecked_access() {
    use marten::level::{ChunkSection, VoxelPhysicsState};
    use std::panic::catch_unwind;
    let c = ChunkSection::new(vec![(0, VoxelPhysicsState::Empty); 4096]);
    assert!(catch_unwind(|| c.get_block(16, 0, 0)).is_err());
    assert!(catch_unwind(|| c.get_block(-1, 0, 0)).is_err());
    let mut tree = marten::octree::SubLevelOctree::new(4);
    tree.buffer.clear();
    assert!(catch_unwind(|| tree.query(0, 0, 0, 0)).is_err());
    assert!(catch_unwind(|| marten::octree::SubLevelOctree::new(31)).is_err());
}
