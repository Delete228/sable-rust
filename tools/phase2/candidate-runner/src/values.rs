//! Real core mass/pose and compatibility-policy consumer. Named fixture states
//! are arithmetic diagnostics, never advertised as actual game registry states.
use super::{Result, array, integer, object, pos, text};
use sable_core::{
    Block, Contents, Motion, Payload, Pose,
    mass::{MassRegistry, Properties},
};
use serde_json::{Value, json};
use std::collections::BTreeSet;

fn vector<const N: usize>(v: &Value) -> Result<[f64; N]> {
    let a = array(v, N)?;
    if a.len() != N {
        return Err("vector shape".into());
    }
    let mut values = [0.; N];
    for i in 0..N {
        values[i] = a[i]
            .as_f64()
            .filter(|v| v.is_finite())
            .ok_or("finite number required")?;
    }
    Ok(values)
}
pub fn validate(doc: &Value) -> Result<()> {
    object(doc, &["schema", "version", "cases"])?;
    if text(&doc["schema"])? != "sable.phase2.values" || integer(&doc["version"], 1, 1)? != 1 {
        return Err("schema/version".into());
    }
    let cases = array(&doc["cases"], 100)?;
    if cases.is_empty() {
        return Err("empty cases".into());
    }
    let mut ids = BTreeSet::new();
    for c in cases {
        let id = text(&c["id"])?;
        if id.is_empty()
            || id.len() > 80
            || !id
                .bytes()
                .all(|x| x.is_ascii_lowercase() || x.is_ascii_digit() || x == b'-' || x == b'_')
            || !ids.insert(id)
        {
            return Err("case id".into());
        }
        match text(&c["kind"])? {
            "mass" => {
                object(c, &["id", "kind", "blocks"])?;
                let bs = array(&c["blocks"], 100)?;
                if bs.is_empty() {
                    return Err("empty blocks".into());
                }
                let mut occupied = BTreeSet::new();
                let mut min = [i32::MAX; 3];
                let mut max = [i32::MIN; 3];
                for b in bs {
                    let a = array(b, 4)?;
                    if a.len() != 4 {
                        return Err("block shape".into());
                    }
                    let p = pos(&json!(a[..3]))?;
                    if !occupied.insert(p) {
                        return Err("duplicate position".into());
                    }
                    if ![
                        "minecraft:blackstone",
                        "minecraft:bedrock",
                        "fixture:mass7",
                        "fixture:slab",
                        "minecraft:air",
                    ]
                    .contains(&text(&a[3])?)
                    {
                        return Err("fixture state".into());
                    }
                    for i in 0..3 {
                        min[i] = min[i].min(p[i]);
                        max[i] = max[i].max(p[i]);
                    }
                }
                if (0..3).any(|i| max[i] - min[i] > 64) {
                    return Err("build bound".into());
                }
            }
            "pose" | "serialize" => {
                let serialize = c["kind"] == "serialize";
                object(
                    c,
                    if serialize {
                        &[
                            "id",
                            "kind",
                            "position",
                            "rotation",
                            "pivot",
                            "scale",
                            "center",
                            "linear",
                            "angular",
                            "dependencies",
                            "name",
                            "user",
                        ]
                    } else {
                        &[
                            "id", "kind", "position", "rotation", "pivot", "scale", "point",
                        ]
                    },
                )?;
                for f in ["position", "pivot", "scale"] {
                    vector::<3>(&c[f])?;
                }
                let rotation = vector::<4>(&c["rotation"])?;
                if (rotation.iter().map(|x| x * x).sum::<f64>() - 1.).abs() > 1e-12
                    || vector::<3>(&c["scale"])?.iter().any(|x| *x <= 0.)
                {
                    return Err("pose quaternion/scale".into());
                }
                if serialize {
                    for f in ["center", "linear", "angular"] {
                        vector::<3>(&c[f])?;
                    }
                    integer(&c["dependencies"], 0, 1)?;
                    if c["name"] != Value::Null && c["name"] != "fixture" {
                        return Err("name".into());
                    }
                    c["user"].as_bool().ok_or("user bool")?;
                } else {
                    vector::<3>(&c["point"])?;
                }
            }
            _ => return Err("kind".into()),
        }
    }
    Ok(())
}
fn bits(v: &Value) -> Value {
    match v {
        Value::Array(a) => Value::Array(a.iter().map(bits).collect()),
        Value::Object(o) => Value::Object(o.iter().map(|(k, v)| (k.clone(), bits(v))).collect()),
        Value::Number(n) => json!(format!("{:016x}", n.as_f64().unwrap().to_bits())),
        _ => v.clone(),
    }
}
fn pose(c: &Value) -> Result<Pose> {
    Ok(Pose {
        position: vector(&c["position"])?,
        rotation: vector(&c["rotation"])?,
        pivot: vector(&c["pivot"])?,
        scale: vector(&c["scale"])?,
    })
}
pub fn run(doc: &Value) -> Result<Value> {
    validate(doc)?;
    let mut outputs = Vec::new();
    for c in doc["cases"].as_array().unwrap() {
        let values = match text(&c["kind"])? {
            "mass" => {
                let mut registry = MassRegistry::pinned_full_blocks();
                for (key, mass, center, inertia, solid, boxes) in [
                    (
                        "fixture:mass7",
                        7.,
                        [0.5; 3],
                        None,
                        true,
                        vec![[0., 0., 0., 1., 1., 1.]],
                    ),
                    (
                        "fixture:slab",
                        3.5,
                        [0.5, 0.25, 0.5],
                        Some([0.1, 0.2, 0.3]),
                        true,
                        vec![[0., 0., 0., 1., 0.5, 1.]],
                    ),
                    ("minecraft:air", 0., [0.5; 3], None, false, Vec::new()),
                ] {
                    registry
                        .register(
                            key.into(),
                            Properties {
                                solid,
                                full: key != "fixture:slab" && solid,
                                mass,
                                center,
                                inertia,
                                boxes,
                            },
                        )
                        .map_err(|e| format!("register: {e:?}"))?;
                }
                let mut contents = Contents::default();
                for b in c["blocks"].as_array().unwrap() {
                    let a = b.as_array().unwrap();
                    contents.blocks.insert(
                        pos(&json!(a[..3]))?,
                        Block {
                            state: text(&a[3])?.into(),
                            block_entity: None,
                            extension: Payload::default(),
                        },
                    );
                }
                let mass = registry
                    .compute(&contents)
                    .map_err(|e| format!("compute: {e:?}"))?;
                json!({"mass":mass.mass,"center":mass.center,"inertia":mass.inertia})
            }
            "pose" => {
                let p = pose(c)?;
                let world = p.transform(vector(&c["point"])?);
                json!({"world":world,"inverse":p.inverse(world)})
            }
            "serialize" => {
                let motion = Motion {
                    pose: pose(c)?,
                    linear: vector(&c["linear"])?,
                    angular: vector(&c["angular"])?,
                };
                let motion = motion.at_self_com(vector(&c["center"])?);
                let (linear, angular) =
                    sable_core::persistence::java_saved_velocities(motion.linear, motion.angular)
                        .map_err(|e| format!("Java policy: {e:?}"))?;
                // Semantic projection only. Mock plot/NBT placeholder values are
                // carried for identity with the reference fixture, not produced
                // by a claim of Java NBT compatibility.
                let mut tag = json!({"uuid":"12345678-1234-4567-89ab-123456789abc","plot":{"fixture":"opaque plot placeholder; no game storage executed"},"world_bounds":"mock world bounds","pose":{"position":motion.pose.position,"rotation":motion.pose.rotation,"pivot":motion.pose.pivot,"scale":motion.pose.scale}});
                if let Some(v) = linear {
                    tag["linear_velocity"] = json!(v)
                }
                if let Some(v) = angular {
                    tag["angular_velocity"] = json!(v)
                }
                if c["name"] != Value::Null {
                    tag["display_name"] = c["name"].clone()
                }
                if c["dependencies"] == 1 {
                    tag["loading_dependencies"] = json!(["aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee"])
                }
                if c["user"] == true {
                    tag["user_data"] = json!({"inventory":"opaque fixture payload"})
                }
                json!({"tag":tag})
            }
            _ => unreachable!(),
        };
        outputs.push(json!({"id":c["id"],"kind":c["kind"],"bits":bits(&values),"values":values}));
    }
    Ok(json!({"schema":"sable.phase2.candidate-values","version":1,"cases":outputs}))
}
