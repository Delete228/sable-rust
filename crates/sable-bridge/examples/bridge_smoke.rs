//! Executed with the SDK compiler; calls the separately built nightly dylib.
use sable_bridge::{
    abi::{self, Header, Request, Response, op, status},
    *,
};
use std::collections::BTreeSet;
use std::sync::{Arc, Mutex};

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
    count: usize,
    owner_thread: bool,
    before: Option<(u32, u8)>,
    after: Option<(u32, u8)>,
    reentrant: u32,
    world_service: Option<(u64, abi::WorldFn)>,
    callback_threads: BTreeSet<String>,
    foreign_world_refused: bool,
}
#[derive(Debug)]
struct Immediate {
    observed: Arc<Mutex<Observation>>,
    owner_thread: std::thread::ThreadId,
    entry: abi::ExecuteFn,
    owner: u64,
}
impl ContactPolicy for Immediate {
    fn on_contact(&self, input: abi::Contact, world: &ContactWorld<'_>) -> abi::ContactReply {
        let mut observed = self.observed.lock().unwrap();
        observed.count += 1;
        observed
            .callback_threads
            .insert(format!("{:?}", std::thread::current().id()));
        observed.owner_thread &= std::thread::current().id() == self.owner_thread;
        if observed.count == 1 {
            observed.before = world.block([0, 0, 0]).unwrap();
            let (nonce, service) = world.protocol_service();
            observed.world_service = Some((nonce, service));
            observed.foreign_world_refused = std::thread::spawn(move || {
                let query = abi::WorldRequest {
                    header: Header::of::<abi::WorldRequest>(),
                    world: nonce,
                    operation: 2,
                    position: [0; 3],
                };
                (unsafe { service(&query, &mut abi::WorldResponse::default()) }) == status::STALE
            })
            .join()
            .unwrap();
            let mut query = abi::WorldRequest {
                header: Header::of::<abi::WorldRequest>(),
                world: nonce,
                operation: 1,
                position: [0, 0, 0],
            };
            let mut reused = abi::WorldResponse::default();
            assert_eq!(unsafe { service(&query, &mut reused) }, status::OK);
            assert_eq!(reused.present, 1);
            query.position = [16, 0, 0];
            assert_eq!(unsafe { service(&query, &mut reused) }, status::OK);
            assert_eq!((reused.present, reused.material, reused.state), (0, 0, 0));
            assert!(world.remove_block([1 << 25, 0, 0]).is_err());
            assert_eq!(world.block([0, 0, 0]).unwrap(), observed.before);
            world.remove_block([0, 0, 0]).unwrap();
            observed.after = world.block([0, 0, 0]).unwrap();
            let mut request = Request::new(op::SCENE);
            request.owner = self.owner;
            observed.reentrant = unsafe { (self.entry)(&request, &mut Response::default()) };
        }
        assert!(input.speed.is_finite());
        abi::ContactReply {
            reject: 1,
            ..Default::default()
        }
    }
}
#[derive(Debug)]
struct InvalidReply;
impl ContactPolicy for InvalidReply {
    fn on_contact(&self, _: abi::Contact, _: &ContactWorld<'_>) -> abi::ContactReply {
        abi::ContactReply {
            reject: 2,
            ..Default::default()
        }
    }
}
fn main() {
    let library = std::env::args().nth(1).expect("bridge_smoke LIBRARY");
    let mut bridge = Bridge::open(&library).unwrap();
    assert!(matches!(Bridge::open(&library), Err(Error::ProcessBusy)));
    let scene = bridge.scene([0.; 3], 0.).unwrap();
    bridge.configure([18, 2, 2], [40., 5.], 128).unwrap();
    let body = bridge
        .create_box(scene, 1, 2., [0.5; 3], [0., 0., 0., 0., 0., 0., 1.])
        .unwrap();
    bridge.impulse(body, [2., 0., 0.], [0.; 3], true).unwrap();
    assert_eq!(bridge.velocity(body).unwrap()[0], 1.);
    let before = bridge.pose(body).unwrap();
    let velocity = bridge.velocity(body).unwrap();
    let mut invalid = Request::new(op::IMPULSE);
    invalid.owner = bridge.owner_token();
    invalid.object = body.0;
    invalid.header.version += 1;
    assert_eq!(
        unsafe { bridge.raw_call(&invalid, &mut Response::default()) },
        status::VERSION
    );
    invalid.header = Header::of::<Request>();
    invalid.header.size -= 1;
    assert_eq!(
        unsafe { bridge.raw_call(&invalid, &mut Response::default()) },
        status::VERSION
    );
    let entry = bridge.protocol_entry();
    let owner = bridge.owner_token();
    let wrong_thread = std::thread::spawn(move || {
        let mut q = Request::new(op::SCENE);
        q.owner = owner;
        unsafe { entry(&q, &mut Response::default()) }
    })
    .join()
    .unwrap();
    assert_eq!(wrong_thread, status::THREAD);
    let mut invalid = Request::new(op::UPLOAD);
    invalid.owner = owner;
    invalid.scene = scene.0;
    invalid.flags = 1;
    invalid.length = 4097;
    assert_eq!(
        unsafe { bridge.raw_call(&invalid, &mut Response::default()) },
        status::INVALID
    );
    let mut invalid_flags = Request::new(op::CLOSE);
    invalid_flags.owner = owner;
    invalid_flags.flags = 1;
    assert_eq!(
        unsafe { bridge.raw_call(&invalid_flags, &mut Response::default()) },
        status::INVALID
    );
    assert_eq!(
        unsafe { entry(std::ptr::null(), &mut Response::default()) },
        status::INVALID
    );
    assert_eq!(bridge.pose(body).unwrap(), before);
    assert_eq!(bridge.velocity(body).unwrap(), velocity);
    bridge.remove_body(body).unwrap();
    assert_eq!(bridge.pose(body), Err(Error::Native(status::STALE)));
    let next = bridge
        .create_box(scene, 1, 2., [0.5; 3], [0., 0., 0., 0., 0., 0., 1.])
        .unwrap();
    assert_ne!(body, next);
    bridge.dispose_scene(scene).unwrap();
    assert!(bridge.pose(next).is_err());

    let scene = bridge.scene([0., -9.81, 0.], 0.).unwrap();
    let observed = Arc::new(Mutex::new(Observation {
        owner_thread: true,
        ..Default::default()
    }));
    let policy = Arc::new(Immediate {
        observed: Arc::clone(&observed),
        owner_thread: std::thread::current().id(),
        entry: bridge.protocol_entry(),
        owner,
    });
    let floor = bridge.material(material(Some(policy))).unwrap();
    let cube = bridge.material(material(None)).unwrap();
    bridge
        .upload_chunk(
            scene,
            [0; 3],
            true,
            None,
            &[abi::Cell {
                position: [0; 3],
                state: 3,
                reserved: 0,
                material: floor.handle,
            }],
        )
        .unwrap();
    let body = bridge
        .create_sublevel(
            scene,
            1,
            [0.5, 1.45, 0.5, 0., 0., 0., 1.],
            [64, 0, 64, 64, 0, 64],
            Mass {
                mass: 1.,
                center: [64.5, 0.5, 64.5],
                inertia: [1. / 6., 0., 0., 0., 1. / 6., 0., 0., 0., 1. / 6.],
            },
        )
        .unwrap();
    bridge
        .upload_chunk(
            scene,
            [4, 0, 4],
            false,
            Some(body),
            &[abi::Cell {
                position: [0; 3],
                state: 3,
                reserved: 0,
                material: cube.handle,
            }],
        )
        .unwrap();
    for _ in 0..24 {
        bridge.step(scene, 0.025).unwrap();
    }
    let observed = observed.lock().unwrap();
    assert!(observed.count > 0);
    assert!(observed.owner_thread);
    assert!(observed.before.is_some_and(|(id, _)| id != 0));
    assert_eq!(observed.after, Some((0, 0)));
    assert_eq!(observed.reentrant, status::REENTRANT);
    assert!(observed.foreign_world_refused);
    println!(
        "{{\"observed_callback_threads\":{:?},\"foreign_thread_world_service_refused\":true}}",
        observed.callback_threads.iter().collect::<Vec<_>>()
    );
    println!(
        "{{\"two_compilers_communicated\":true,\"impulse_mass_invariant\":true,\"stale_length_version_thread_refusals\":true,\"callbacks\":{},\"callback_owner_thread\":{},\"immediate_remove_visible\":true,\"reentrant_status\":{}}}",
        observed.count, observed.owner_thread, observed.reentrant
    );
    let (nonce, service) = observed.world_service.unwrap();
    let query = abi::WorldRequest {
        header: Header::of::<abi::WorldRequest>(),
        world: nonce,
        operation: 1,
        position: [0, 0, 0],
    };
    assert_eq!(
        unsafe { service(&query, &mut abi::WorldResponse::default()) },
        status::STALE
    );
    drop(observed);
    // A distinct, ordinary contact produces native collision data. A caller's
    // undersized output must neither consume that snapshot nor mutate a body.
    let buffer_scene = bridge.scene([0., -9.81, 0.], 0.).unwrap();
    bridge
        .upload_chunk(
            buffer_scene,
            [0; 3],
            true,
            None,
            &[abi::Cell {
                position: [0; 3],
                state: 3,
                reserved: 0,
                material: cube.handle,
            }],
        )
        .unwrap();
    let buffer_body = bridge
        .create_sublevel(
            buffer_scene,
            7,
            [0.5, 1.45, 0.5, 0., 0., 0., 1.],
            [64, 0, 64, 64, 0, 64],
            Mass {
                mass: 1.,
                center: [64.5, 0.5, 64.5],
                inertia: [1. / 6., 0., 0., 0., 1. / 6., 0., 0., 0., 1. / 6.],
            },
        )
        .unwrap();
    bridge
        .upload_chunk(
            buffer_scene,
            [4, 0, 4],
            false,
            Some(buffer_body),
            &[abi::Cell {
                position: [0; 3],
                state: 3,
                reserved: 0,
                material: cube.handle,
            }],
        )
        .unwrap();
    for _ in 0..24 {
        bridge.step(buffer_scene, 0.025).unwrap();
    }
    let pose = bridge.pose(buffer_body).unwrap();
    let mut capture = Request::new(op::CAPTURE_COLLISIONS);
    capture.owner = owner;
    capture.scene = buffer_scene.0;
    let mut response = Response::default();
    assert_eq!(
        unsafe { bridge.raw_call(&capture, &mut response) },
        status::OK
    );
    assert!(response.count > 0);
    let count = response.count as usize;
    let mut copy = Request::new(op::COPY_COLLISIONS);
    copy.owner = owner;
    copy.scene = buffer_scene.0;
    let mut sentinel = [1234.5];
    let mut short = Response {
        data: sentinel.as_mut_ptr(),
        capacity: 1,
        ..Default::default()
    };
    assert_eq!(
        unsafe { bridge.raw_call(&copy, &mut short) },
        status::BUFFER
    );
    assert_eq!(sentinel, [1234.5]);
    assert_eq!(bridge.pose(buffer_body).unwrap(), pose);
    let mut values = vec![0.; count];
    let mut full = Response {
        data: values.as_mut_ptr(),
        capacity: count as u32,
        ..Default::default()
    };
    assert_eq!(unsafe { bridge.raw_call(&copy, &mut full) }, status::OK);
    assert_eq!(full.written as usize, count);
    assert_eq!(bridge.pose(buffer_body).unwrap(), pose);
    println!(
        "{{\"short_collision_buffer_refused\":true,\"snapshot_retry_preserved\":true,\"collision_values\":{count}}}"
    );
    let invalid_policy = bridge
        .material(material(Some(Arc::new(InvalidReply))))
        .unwrap();
    bridge
        .upload_chunk(
            buffer_scene,
            [0; 3],
            true,
            None,
            &[abi::Cell {
                position: [0; 3],
                state: 3,
                reserved: 0,
                material: invalid_policy.handle,
            }],
        )
        .unwrap();
    let mut refused = false;
    for _ in 0..24 {
        match bridge.step(buffer_scene, 0.025) {
            Err(Error::Native(status::CALLBACK)) => {
                refused = true;
                break;
            }
            result => result.unwrap(),
        }
    }
    assert!(refused);
    let after_failure = bridge.pose(buffer_body).unwrap();
    assert_eq!(
        bridge.impulse(buffer_body, [1., 0., 0.], [0.; 3], true),
        Err(Error::Native(status::CALLBACK))
    );
    assert_eq!(
        bridge.step(buffer_scene, 0.025),
        Err(Error::Native(status::CALLBACK))
    );
    assert_eq!(bridge.pose(buffer_body).unwrap(), after_failure);
    println!(
        "{{\"invalid_callback_refused\":true,\"session_mutation_frozen\":true,\"diagnostic_pose_available\":true}}"
    );
    bridge.close().unwrap();
    let mut fresh = Bridge::open(&library).unwrap();
    let _ = fresh.scene([0.; 3], 0.).unwrap();
    let retained = fresh.material(material(None)).unwrap();
    assert!(retained.index > cube.index);
    let mut old = Request::new(op::SCENE);
    old.owner = owner;
    assert_eq!(
        unsafe { fresh.raw_call(&old, &mut Response::default()) },
        status::STALE
    );
    fresh.close().unwrap();
}
