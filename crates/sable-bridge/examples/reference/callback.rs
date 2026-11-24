//! Personal bridge counterpart of the preserved callback fixture. Call-scoped
//! contact data crosses the C ABI; no borrowed Host or Rust layout does.
use crate::callback_contract::{Mode, ValidatedCase};
use sable_bridge::{
    Bridge as Engine, ContactPolicy, ContactWorld, Mass as BodyMass, Material, abi,
};
use serde_json::{Value, json};
use std::sync::{Arc, Mutex};
use std::thread::ThreadId;
fn scalar(v: f64) -> Value {
    json!({"decimal":v.to_string(),"bits":format!("{:016x}",v.to_bits()),"scalar":"f64","classification":if v.is_nan(){"nan"}else if v.is_infinite(){"infinite"}else{"finite"}})
}
fn values(v: &[f64]) -> Value {
    Value::Array(v.iter().map(|&x| scalar(x)).collect())
}
#[derive(Debug)]
struct State {
    occupied: bool,
    count: usize,
    events: Vec<Value>,
    reentrant: Option<String>,
    edit_visible: bool,
}
impl State {
    fn event(&mut self, phase: &str, before: bool, after: bool, owner: bool) {
        let sequence = self.events.len();
        self.events.push(json!({"sequence":sequence,"phase":phase,"world_before":before,"world_after":after,"owner_thread":owner}));
    }
}
#[derive(Debug)]
struct Policy {
    mode: Mode,
    state: Arc<Mutex<State>>,
    owner: ThreadId,
}
impl ContactPolicy for Policy {
    fn on_contact(&self, input: abi::Contact, world: &ContactWorld<'_>) -> abi::ContactReply {
        let mut s = self.state.lock().unwrap();
        s.count += 1;
        let before = s.occupied;
        if matches!(self.mode, Mode::ImmediateWorld | Mode::ReentrantChange) {
            s.occupied = false;
        }
        let after = s.occupied;
        let owner = std::thread::current().id() == self.owner;
        if s.events.len() < 2048 {
            s.event("callback_enter", before, after, owner);
        }
        if s.events.len() < 2048 {
            let sequence = s.events.len();
            s.events.push(json!({"sequence":sequence,"phase":"contact_arguments","block":input.block,"other_block":input.other,"point_and_speed":values(&[input.point[0],input.point[1],input.point[2],input.speed]),"has_other_block":input.has_other != 0}));
        }
        if s.count == 1 && self.mode.is_reentrant() {
            s.event("before_reentrant_native", after, after, owner);
            if self.mode == Mode::ReentrantChange {
                world.remove_block([0, 0, 0]).unwrap();
                s.edit_visible = world
                    .block([0, 0, 0])
                    .unwrap()
                    .is_some_and(|(id, _)| id == 0);
                assert!(
                    s.edit_visible,
                    "same-callback query must observe synchronous removal"
                );
            } else {
                let error = Engine::open(std::env::var_os("SABLE_PHYSICS_LIBRARY").unwrap())
                    .err()
                    .expect("reentrant owner must fail");
                s.reentrant = Some(format!("{error:?}"));
            }
            s.event("after_reentrant_native", after, after, owner);
        }
        abi::ContactReply {
            tangent: [0.; 3],
            reject: u32::from(self.mode == Mode::Reject || !s.occupied),
            ..Default::default()
        }
    }
}
pub fn run(input: ValidatedCase) -> Value {
    let mut engine = super::open_bridge();
    let scene = engine.scene([0., -9.81, 0.], 0.).unwrap();
    engine.configure([18, 2, 2], [40., 5.], 128).unwrap();
    let state = Arc::new(Mutex::new(State {
        occupied: true,
        count: 0,
        events: vec![],
        reentrant: None,
        edit_visible: false,
    }));
    let policy = Arc::new(Policy {
        mode: input.mode(),
        state: Arc::clone(&state),
        owner: std::thread::current().id(),
    });
    let material = |contact| Material {
        friction: 1.,
        volume: 1.,
        restitution: 0.,
        fluid: false,
        dynamic: false,
        boxes: vec![[0., 0., 0., 1., 1., 1.]],
        contact,
    };
    let floor = engine.material(material(Some(policy))).unwrap();
    let body_material = engine.material(material(None)).unwrap();
    engine
        .upload_chunk(
            scene,
            [0, 0, 0],
            true,
            None,
            &[abi::Cell {
                position: [0; 3],
                material: floor.handle,
                state: 3,
                reserved: 0,
            }],
        )
        .unwrap();
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
            &[abi::Cell {
                position: [0; 3],
                material: body_material.handle,
                state: 3,
                reserved: 0,
            }],
        )
        .unwrap();
    state
        .lock()
        .unwrap()
        .event("before_steps", true, true, true);
    let initial = engine.pose(body).unwrap();
    for _ in 0..24 {
        engine.step(scene, 0.025).unwrap();
    }
    let after = engine.pose(body).unwrap();
    let collisions = engine.collisions(scene).unwrap();
    let mut s = state.lock().unwrap();
    let occupied = s.occupied;
    s.event("after_steps", occupied, occupied, true);
    let mut result = json!({"schema":"sable.abi_adapted_observations","schema_version":1,"scenario_id":input.id(),"reference_kind":"unchanged native / ABI-adapted fixture","full_game_evidence":false,"callback_count":s.count,"world_after_steps":s.occupied,"initial_pose":values(&initial),"final_pose":values(&after),"collision_records":collisions.len()/15,"events":s.events});
    if let Some(error) = &s.reentrant {
        result["candidate_reentrant_error"] = json!(error);
    }
    drop(s);
    engine.dispose_scene(scene).unwrap();
    result
}
