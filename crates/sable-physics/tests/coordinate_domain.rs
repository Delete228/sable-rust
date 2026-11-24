//! Public API regressions for packed-key aliasing, including real contact hooks.
use sable_physics::*;
use std::sync::{Arc, Mutex};

// Engine intentionally permits only one process owner. Keep these independent
// integration cases serialized within this test process.
static SERIAL: Mutex<()> = Mutex::new(());
const SECTION_BITS: [u32; 3] = [22, 20, 22];
const BLOCK_BITS: [u32; 3] = [26, 24, 26];

fn on_axis(axis: usize, value: i32) -> [i32; 3] {
    let mut point = [0; 3];
    point[axis] = value;
    point
}
fn edges(bits: [u32; 3]) -> Vec<[i32; 3]> {
    let mut points = Vec::new();
    for (axis, width) in bits.into_iter().enumerate() {
        let half = 1 << (width - 1);
        points.push(on_axis(axis, -half));
        points.push(on_axis(axis, half - 1));
    }
    points
}
fn invalid_points(bits: [u32; 3]) -> Vec<[i32; 3]> {
    let mut points = Vec::new();
    for (axis, width) in bits.into_iter().enumerate() {
        let half = 1 << (width - 1);
        // The first invalid values wrap onto populated opposite-domain edges;
        // full packing periods wrap onto the real origin. Test both signs.
        for value in [
            -half - 1,
            half,
            -(1 << width),
            1 << width,
            i32::MIN,
            i32::MAX,
        ] {
            points.push(on_axis(axis, value));
        }
    }
    points
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
fn body(engine: &mut Engine, scene: SceneHandle, material: MaterialHandle) -> BodyHandle {
    let body = engine
        .create_sublevel(
            scene,
            1,
            [0.5, 1.45, 0.5, 0., 0., 0., 1.],
            [64, 0, 64, 64, 0, 64],
            BodyMass {
                mass: 1.,
                center: [64.5, 0.5, 64.5],
                inertia: [1. / 6., 0., 0., 0., 1. / 6., 0., 0., 0., 1. / 6.],
            },
        )
        .unwrap();
    engine
        .upload_chunk(
            scene,
            [4, 0, 4],
            false,
            Some(body),
            vec![([0; 3], material, 3)],
        )
        .unwrap();
    body
}
#[derive(Debug)]
struct InspectOrigin(Arc<Mutex<usize>>);
impl ContactPolicy for InspectOrigin {
    fn on_contact(&self, _: ContactRequest, world: &dyn ContactWorld) -> ContactResponse {
        let origin = world
            .block([0; 3])
            .expect("real floor section must survive refusal");
        assert!(origin.0 > 0);
        assert_eq!(origin.1, 3);
        assert_eq!(
            world.block([256, 0, 0]),
            None,
            "valid unrelated removal must execute"
        );
        *self.0.lock().unwrap() += 1;
        ContactResponse::default()
    }
}

#[test]
fn section_domain_checks_both_signs_all_axes_and_preserves_origin_contacts() {
    let _serial = SERIAL.lock().unwrap();
    let mut engine = Engine::try_new().unwrap();
    let scene = engine.create_scene([0., -9.81, 0.], 0.).unwrap();
    let control = engine.create_scene([0., -9.81, 0.], 0.).unwrap();
    engine.configure([18, 2, 2], [40., 5.], 128).unwrap();
    let callbacks = Arc::new(Mutex::new(0));
    let floor = engine
        .register_material(material(Some(Arc::new(InspectOrigin(Arc::clone(
            &callbacks,
        ))))))
        .unwrap();
    let moving = engine.register_material(material(None)).unwrap();
    for scene in [scene, control] {
        engine
            .upload_chunk(scene, [0; 3], true, None, vec![([0; 3], floor, 3)])
            .unwrap();
    }
    // All exact signed section edges remain accepted; none is a capture limit.
    for edge in edges(SECTION_BITS) {
        engine
            .upload_chunk(scene, edge, true, None, vec![([0; 3], floor, 3)])
            .unwrap();
    }
    for point in invalid_points(SECTION_BITS) {
        assert_eq!(
            engine.upload_chunk(scene, point, true, None, vec![([0; 3], floor, 3)]),
            Err(Error::InvalidInput),
            "invalid upload {point:?}"
        );
        for global in [false, true] {
            assert_eq!(
                engine.remove_chunk(scene, point, global),
                Err(Error::InvalidInput),
                "invalid removal {point:?}, global={global}"
            );
        }
    }
    // Contrast invalid aliased coordinates with an actual valid unrelated key.
    engine
        .upload_chunk(scene, [16, 0, 0], true, None, vec![([0; 3], floor, 3)])
        .unwrap();
    engine.remove_chunk(scene, [16, 0, 0], true).unwrap();
    for edge in edges(SECTION_BITS) {
        engine.remove_chunk(scene, edge, true).unwrap();
    }
    let protected_body = body(&mut engine, scene, moving);
    let control_body = body(&mut engine, control, moving);
    for _ in 0..24 {
        engine.step(scene, 0.025).unwrap();
        engine.step(control, 0.025).unwrap();
    }
    assert!(
        *callbacks.lock().unwrap() > 0,
        "must execute actual floor contact hooks"
    );
    assert_eq!(
        engine.pose(protected_body).unwrap(),
        engine.pose(control_body).unwrap()
    );
    assert_eq!(
        engine.velocity(protected_body).unwrap(),
        engine.velocity(control_body).unwrap()
    );
    assert!(
        engine.pose(protected_body).unwrap()[1] > 1.4,
        "floor must still support body"
    );
    assert!(!engine.collisions(scene).unwrap().is_empty());
    assert!(!engine.collisions(control).unwrap().is_empty());
}

#[derive(Debug, Default)]
struct WorldObservations {
    callbacks: usize,
    checked_invalid: usize,
    checked_edges: usize,
}
#[derive(Debug)]
struct CheckWorldDomain {
    observations: Arc<Mutex<WorldObservations>>,
    remove_origin: bool,
}
impl ContactPolicy for CheckWorldDomain {
    fn on_contact(&self, _: ContactRequest, world: &dyn ContactWorld) -> ContactResponse {
        let mut observations = self.observations.lock().unwrap();
        observations.callbacks += 1;
        if observations.callbacks == 1 {
            let origin = world.block([0; 3]).unwrap();
            assert!(origin.0 > 0);
            assert_eq!(origin.1, 3);
            let edge_points = edges(BLOCK_BITS);
            let original_edges: Vec<_> = edge_points
                .iter()
                .map(|&edge| {
                    let block = world
                        .block(edge)
                        .expect("exact valid block edge remains queryable");
                    assert!(block.0 > 0);
                    assert_eq!(block.1, 3);
                    block
                })
                .collect();
            for point in invalid_points(BLOCK_BITS) {
                assert_eq!(
                    world.block(point),
                    None,
                    "invalid query {point:?} must not alias"
                );
                assert_eq!(
                    world.remove_block(point),
                    Err(ContactWorldError::InvalidCoordinate),
                    "invalid edit {point:?} must not mutate"
                );
                assert_eq!(
                    world.block([0; 3]),
                    Some(origin),
                    "real origin survives {point:?}"
                );
                for (&edge, &expected) in edge_points.iter().zip(&original_edges) {
                    assert_eq!(
                        world.block(edge),
                        Some(expected),
                        "edge survives invalid edit {point:?}"
                    );
                }
                observations.checked_invalid += 1;
            }
            // Valid edge edits and a valid unrelated edit still execute in this
            // real callback, with immediate visibility and no origin mutation.
            for edge in edge_points {
                world.remove_block(edge).unwrap();
                assert_eq!(world.block(edge), Some((0, 0)));
                assert_eq!(world.block([0; 3]), Some(origin));
                observations.checked_edges += 1;
            }
            assert!(world.block([256, 0, 0]).unwrap().0 > 0);
            world.remove_block([256, 0, 0]).unwrap();
            assert_eq!(world.block([256, 0, 0]), Some((0, 0)));
            assert_eq!(world.block([0; 3]), Some(origin));
            if self.remove_origin {
                world.remove_block([0; 3]).unwrap();
                assert_eq!(world.block([0; 3]), Some((0, 0)));
            }
        }
        ContactResponse {
            tangent: [0.; 3],
            reject: self.remove_origin,
        }
    }
}
fn contact_world_run(
    engine: &mut Engine,
    remove_origin: bool,
) -> ([f64; 7], usize, Arc<Mutex<WorldObservations>>) {
    let scene = engine.create_scene([0., -9.81, 0.], 0.).unwrap();
    engine.configure([18, 2, 2], [40., 5.], 128).unwrap();
    let observations = Arc::new(Mutex::new(WorldObservations::default()));
    let floor = engine
        .register_material(material(Some(Arc::new(CheckWorldDomain {
            observations: Arc::clone(&observations),
            remove_origin,
        }))))
        .unwrap();
    let other = engine.register_material(material(None)).unwrap();
    engine
        .upload_chunk(scene, [0; 3], true, None, vec![([0; 3], floor, 3)])
        .unwrap();
    engine
        .upload_chunk(scene, [16, 0, 0], true, None, vec![([0; 3], other, 3)])
        .unwrap();
    for edge in edges(BLOCK_BITS) {
        let section = edge.map(|coordinate| coordinate >> 4);
        let local = edge.map(|coordinate| (coordinate & 15) as u8);
        engine
            .upload_chunk(scene, section, true, None, vec![(local, other, 3)])
            .unwrap();
    }
    let moving_body = body(engine, scene, other);
    for _ in 0..24 {
        engine.step(scene, 0.025).unwrap();
    }
    let pose = engine.pose(moving_body).unwrap();
    let collisions = engine.collisions(scene).unwrap().len();
    engine.dispose_scene(scene).unwrap();
    (pose, collisions, observations)
}
#[test]
fn real_contact_queries_and_edits_reject_aliases_but_accept_exact_domain_edges() {
    let _serial = SERIAL.lock().unwrap();
    let mut engine = Engine::try_new().unwrap();
    let (protected, contacts, observed) = contact_world_run(&mut engine, false);
    {
        let observed = observed.lock().unwrap();
        assert!(observed.callbacks > 0);
        assert_eq!(observed.checked_invalid, invalid_points(BLOCK_BITS).len());
        assert_eq!(observed.checked_edges, 6);
    }
    assert!(
        protected[1] > 1.4,
        "invalid edits must preserve physical floor support"
    );
    assert!(contacts > 0);
    // This control proves that real valid origin edits still remove collision;
    // refusing an invalid alias is distinct from making all edits no-ops.
    let (removed, contacts, observed) = contact_world_run(&mut engine, true);
    assert!(observed.lock().unwrap().callbacks > 0);
    assert!(
        removed[1] < 0.,
        "valid immediate origin removal must make body fall"
    );
    assert_eq!(contacts, 0);
}
