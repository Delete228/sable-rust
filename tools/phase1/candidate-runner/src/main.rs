mod callback;
mod callback_contract;
// Candidate executor for the same owned-input scenario document as the native oracle.
use sable_physics::*;
use serde_json::{Value, json};
use std::collections::HashMap;
fn text<'a>(a: &'a Value, k: &str) -> &'a str {
    a[k].as_str().unwrap()
}
fn number(a: &Value, k: &str) -> f64 {
    a[k].as_f64().unwrap()
}
fn int(a: &Value, k: &str) -> i32 {
    a[k].as_i64().unwrap() as i32
}
fn array<const N: usize>(v: &Value) -> [f64; N] {
    v.as_array()
        .unwrap()
        .iter()
        .map(|n| n.as_f64().unwrap())
        .collect::<Vec<_>>()
        .try_into()
        .unwrap()
}
fn integers<const N: usize>(v: &Value) -> [i32; N] {
    v.as_array()
        .unwrap()
        .iter()
        .map(|n| n.as_i64().unwrap() as i32)
        .collect::<Vec<_>>()
        .try_into()
        .unwrap()
}
fn scalar(v: f64) -> Value {
    json!({"decimal":v.to_string(),"bits":format!("{:016x}",v.to_bits()),"scalar":"f64","source_scalar":"native_f32_widened_unless_documented","classification":if v.is_nan(){"nan"}else if v.is_infinite(){"infinite"}else{"finite"}})
}
fn values(v: &[f64]) -> Value {
    Value::Array(v.iter().map(|&v| scalar(v)).collect())
}
struct Runner {
    engine: Engine,
    scenes: HashMap<String, SceneHandle>,
    bodies: HashMap<String, Vec<(i32, BodyHandle)>>,
    materials: HashMap<String, MaterialHandle>,
    joints: HashMap<String, JointHandle>,
    samples: Vec<Value>,
    events: Vec<Value>,
    action: usize,
    step: usize,
}
impl Runner {
    fn body(&self, a: &Value) -> BodyHandle {
        self.bodies[text(a, "scene")]
            .iter()
            .find(|b| b.0 == int(a, "id"))
            .unwrap()
            .1
    }
    fn sample(&mut self, alias: &str, id: i32, h: BodyHandle, label: &str) {
        let pose = self.engine.pose(h).unwrap();
        let v = self.engine.velocity(h).unwrap();
        self.samples.push(json!({"schema":"sable.sample","schema_version":1,"action_index":self.action,"case_id":label,"scene_alias":alias,"body_id":id,"logical_tick":null,"phase":"after_native_action","substep":self.step,"pose":values(&pose),"linear_velocity":values(&v[..3]),"angular_velocity":values(&v[3..])}));
    }
    fn collisions(&mut self, alias: &str, label: &str) {
        let raw = self.engine.collisions(self.scenes[alias]).unwrap();
        self.samples.push(json!({"schema":"sable.contact_sample","schema_version":1,"action_index":self.action,"case_id":label,"scene_alias":alias,"substep":self.step,"record_width":15,"raw":values(&raw)}));
    }
    fn execute(&mut self, a: &Value) {
        let op = text(a, "op");
        let event = match op {
            "scene" => {
                let alias = text(a, "scene");
                let h = self
                    .engine
                    .create_scene(array(&a["gravity"]), number(a, "drag"))
                    .unwrap();
                self.scenes.insert(alias.into(), h);
                self.bodies.insert(alias.into(), vec![]);
                Some(json!({"scene":alias}))
            }
            "config" => {
                self.engine
                    .configure(
                        integers(&a["iterations"]),
                        array(&a["frequency"]),
                        int(a, "min_island"),
                    )
                    .unwrap();
                Some(json!({"process_global":true}))
            }
            "material" => {
                let m = self
                    .engine
                    .register_material(Material {
                        friction: number(a, "friction"),
                        volume: number(a, "volume"),
                        restitution: number(a, "restitution"),
                        fluid: a["fluid"].as_bool().unwrap(),
                        dynamic: a["dynamic"].as_bool().unwrap(),
                        boxes: a["boxes"].as_array().unwrap().iter().map(array).collect(),
                        contact: None,
                    })
                    .unwrap();
                self.materials.insert(text(a, "material").into(), m);
                Some(
                    json!({"material":text(a,"material"),"index":self.engine.material_index(m),"dynamic":a["dynamic"]}),
                )
            }
            "material_clear" => {
                self.engine
                    .clear_material_boxes(self.materials[text(a, "material")])
                    .unwrap();
                Some(json!({}))
            }
            "material_add_box" => {
                self.engine
                    .add_material_box(self.materials[text(a, "material")], array(&a["bounds"]))
                    .unwrap();
                Some(json!({}))
            }
            "box" => {
                let alias = text(a, "scene");
                let id = int(a, "id");
                let h = self
                    .engine
                    .create_box(
                        self.scenes[alias],
                        id,
                        number(a, "mass"),
                        array(&a["half_extents"]),
                        array(&a["pose"]),
                    )
                    .unwrap();
                self.bodies.get_mut(alias).unwrap().push((id, h));
                Some(json!({"body_id":id}))
            }
            "sublevel" => {
                let alias = text(a, "scene");
                let id = int(a, "id");
                let h = self
                    .engine
                    .create_sublevel(
                        self.scenes[alias],
                        id,
                        array(&a["pose"]),
                        integers(&a["bounds"]),
                        BodyMass {
                            mass: number(a, "mass"),
                            center: array(&a["center"]),
                            inertia: array(&a["inertia"]),
                        },
                    )
                    .unwrap();
                self.bodies.get_mut(alias).unwrap().push((id, h));
                Some(json!({"body_id":id}))
            }
            "chunk" => {
                let alias = text(a, "scene");
                let id = int(a, "id");
                let body = if id == -1 {
                    None
                } else {
                    Some(self.bodies[alias].iter().find(|b| b.0 == id).unwrap().1)
                };
                let cells = a["cells"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|c| {
                        let p = integers::<3>(&c["local"]);
                        (
                            [p[0] as u8, p[1] as u8, p[2] as u8],
                            self.materials[text(c, "material")],
                            int(c, "state") as u8,
                        )
                    })
                    .collect();
                self.engine
                    .upload_chunk(
                        self.scenes[alias],
                        integers(&a["chunk"]),
                        a["global"].as_bool().unwrap(),
                        body,
                        cells,
                    )
                    .unwrap();
                Some(json!({"cell_count":a["cells"].as_array().unwrap().len()}))
            }
            "chunk_remove" => {
                self.engine
                    .remove_chunk(
                        self.scenes[text(a, "scene")],
                        integers(&a["chunk"]),
                        a["global"].as_bool().unwrap(),
                    )
                    .unwrap();
                Some(json!({"global":a["global"]}))
            }
            "impulse" | "velocity" => {
                let h = self.body(a);
                let l = array(&a["linear"]);
                let r = array(&a["angular"]);
                let wake = a["wake"].as_bool().unwrap();
                if op == "impulse" {
                    self.engine.impulse(h, l, r, wake).unwrap()
                } else {
                    self.engine.add_velocity(h, l, r, wake).unwrap()
                }
                Some(json!({}))
            }
            "wake" => {
                self.engine.wake(self.body(a)).unwrap();
                Some(json!({}))
            }
            "step" => {
                let alias = text(a, "scene");
                let count = int(a, "count");
                let every = int(a, "sample_every");
                for i in 1..=count {
                    self.engine
                        .step(self.scenes[alias], number(a, "dt"))
                        .unwrap();
                    self.step += 1;
                    if i % every == 0 || i == count {
                        for (id, h) in self.bodies[alias].clone() {
                            self.sample(alias, id, h, text(a, "label"));
                        }
                        self.collisions(alias, text(a, "label"));
                    }
                }
                None
            }
            "sample" => {
                self.sample(
                    text(a, "scene"),
                    int(a, "id"),
                    self.body(a),
                    text(a, "label"),
                );
                None
            }
            "collisions" => {
                self.collisions(text(a, "scene"), text(a, "label"));
                None
            }
            "fixed_joint" => {
                let alias = text(a, "scene");
                let lookup = |id: i32| {
                    if id == -1 {
                        None
                    } else {
                        Some(self.bodies[alias].iter().find(|b| b.0 == id).unwrap().1)
                    }
                };
                let h = self
                    .engine
                    .fixed_joint(
                        self.scenes[alias],
                        lookup(int(a, "a")),
                        lookup(int(a, "b")),
                        array(&a["anchors"]),
                        array(&a["orientation"]),
                    )
                    .unwrap();
                self.joints.insert(text(a, "joint").into(), h);
                Some(
                    json!({"joint_alias":text(a,"joint"),"valid":self.engine.joint_sample(h).unwrap().0}),
                )
            }
            "joint_sample" => {
                let (valid, impulses) = self
                    .engine
                    .joint_sample(self.joints[text(a, "joint")])
                    .unwrap();
                self.samples.push(json!({"schema":"sable.joint_sample","schema_version":1,"action_index":self.action,"case_id":text(a,"label"),"joint_alias":text(a,"joint"),"valid":valid,"impulses":values(&impulses)}));
                None
            }
            "joint_remove" => {
                let h = self.joints[text(a, "joint")];
                self.engine.remove_joint(h).unwrap();
                Some(json!({"valid_after":self.engine.joint_sample(h).unwrap().0}))
            }
            "body_remove" => {
                let h = self.body(a);
                self.engine.remove_body(h).unwrap();
                self.bodies
                    .get_mut(text(a, "scene"))
                    .unwrap()
                    .retain(|b| b.0 != int(a, "id"));
                Some(json!({"body_id":int(a,"id")}))
            }
            "dispose" => {
                let alias = text(a, "scene");
                self.engine
                    .dispose_scene(self.scenes.remove(alias).unwrap())
                    .unwrap();
                self.bodies.remove(alias);
                Some(json!({"scene":alias}))
            }
            _ => panic!("Unsupported scenario operation {op}"),
        };
        if let Some(data) = event {
            self.events
                .push(json!({"action_index":self.action,"type":op,"data":data}));
        }
    }
}
fn run() -> std::result::Result<(), String> {
    let args = std::env::args().collect::<Vec<_>>();
    if args.len() != 3 {
        return Err("Callback contract v2: native-candidate SCENARIOS_JSON SCENARIO_ID".into());
    }
    let root: Value =
        callback_contract::parse_document(&std::fs::read(&args[1]).map_err(|e| e.to_string())?)?;
    if root.get("schema").and_then(Value::as_str) != Some("sable.native_scenarios")
        || root.get("schema_version").and_then(Value::as_u64) != Some(1)
    {
        return Err("Callback contract v2: unsupported schema/version".into());
    }
    let callback_document = callback_contract::callback_document(&root)?;
    let scenario = root
        .get("scenarios")
        .and_then(Value::as_array)
        .ok_or_else(|| "Callback contract v2: scenarios must be array".to_string())?
        .iter()
        .find(|s| s["id"] == args[2])
        .ok_or_else(|| "Callback contract v2: unknown requested scenario ID".to_string())?;
    if callback_document || callback_contract::is_callback(&args[2], scenario) {
        println!(
            "{}",
            callback_contract::dispatch(&root, &args[2], callback::run)?
        );
        return Ok(());
    }
    let actions = callback_contract::core_actions(scenario)?;
    let mut r = Runner {
        engine: Engine::try_new().unwrap(),
        scenes: HashMap::new(),
        bodies: HashMap::new(),
        materials: HashMap::new(),
        joints: HashMap::new(),
        samples: vec![],
        events: vec![],
        action: 0,
        step: 0,
    };
    for a in actions {
        r.execute(a);
        r.action += 1;
    }
    println!(
        "{}",
        json!({"schema":"sable.abi_adapted_observations","schema_version":1,"scenario_id":args[2],"runtime_processors":std::thread::available_parallelism().map_or(1,|n|n.get()),"samples":r.samples,"events":r.events})
    );
    Ok(())
}
fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(2);
    }
}
