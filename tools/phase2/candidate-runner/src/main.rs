//! Reference-only standalone consumer. Validate the complete closed document
//! before running the real core algorithms; never synthesize original results.
mod strict;
mod values;
use sable_core::{
    Pos, gather,
    split::{HeatState, SplitTracker},
};
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};

type Result<T> = std::result::Result<T, String>;
fn object<'a>(v: &'a Value, fields: &[&str]) -> Result<&'a serde_json::Map<String, Value>> {
    let m = v.as_object().ok_or("expected object")?;
    if m.len() != fields.len() || fields.iter().any(|k| !m.contains_key(*k)) {
        return Err("unknown/missing field".into());
    }
    Ok(m)
}
fn array(v: &Value, max: usize) -> Result<&Vec<Value>> {
    let a = v.as_array().ok_or("expected array")?;
    if a.len() > max {
        return Err("array limit".into());
    }
    Ok(a)
}
fn integer(v: &Value, lo: i64, hi: i64) -> Result<i64> {
    let n = v.as_i64().ok_or("expected integer")?;
    if n < lo || n > hi {
        return Err("integer limit".into());
    }
    Ok(n)
}
fn pos(v: &Value) -> Result<Pos> {
    let a = array(v, 3)?;
    if a.len() != 3 {
        return Err("position length".into());
    }
    Ok([
        integer(&a[0], -(1 << 25), (1 << 25) - 1)? as i32,
        integer(&a[1], -2048, 2047)? as i32,
        integer(&a[2], -(1 << 25), (1 << 25) - 1)? as i32,
    ])
}
fn text(v: &Value) -> Result<&str> {
    v.as_str().ok_or("expected string".into())
}
fn validate(v: &Value) -> Result<()> {
    object(v, &["schema", "version", "cases"])?;
    if text(&v["schema"])? != "sable.phase2.algorithm" || integer(&v["version"], 1, 1)? != 1 {
        return Err("schema/version".into());
    }
    let cases = array(&v["cases"], 100)?;
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
        let mut occupied = BTreeSet::new();
        match text(&c["kind"])? {
            "gather" => {
                object(
                    c,
                    &["id", "kind", "blocks", "origin", "capacity", "predicate"],
                )?;
                pos(&c["origin"])?;
                integer(&c["capacity"], 0, 25_600_000)?;
                if !["all", "faces", "edges", "same_state", "reject"]
                    .contains(&text(&c["predicate"])?)
                {
                    return Err("predicate".into());
                }
                for b in array(&c["blocks"], 10000)? {
                    let a = array(b, 4)?;
                    if a.len() != 4 {
                        return Err("block shape".into());
                    }
                    let p = pos(&Value::Array(a[..3].to_vec()))?;
                    integer(&a[3], 1, 32)?;
                    if !occupied.insert(p) {
                        return Err("duplicate coordinate".into());
                    }
                }
            }
            "heat" => {
                object(c, &["id", "kind", "steps", "initial", "actions"])?;
                let steps = integer(&c["steps"], 1, 1000)?;
                for p in array(&c["initial"], 10000)? {
                    if !occupied.insert(pos(p)?) {
                        return Err("duplicate coordinate".into());
                    }
                }
                let mut ticks = 0;
                for a in array(&c["actions"], 1000)? {
                    match text(&a["op"])? {
                        "add" | "remove" => {
                            object(a, &["op", "pos"])?;
                            pos(&a["pos"])?;
                        }
                        "tick" => {
                            object(a, &["op", "count"])?;
                            ticks += integer(&a["count"], 1, 10000)?;
                        }
                        "enabled" => {
                            object(a, &["op", "value"])?;
                            a["value"].as_bool().ok_or("boolean required")?;
                        }
                        _ => return Err("action".into()),
                    }
                }
                if ticks > 10000 || ticks * steps > 200000 {
                    return Err("work budget".into());
                }
            }
            _ => return Err("kind".into()),
        }
    }
    Ok(())
}
fn gather_case(c: &Value) -> Result<Value> {
    let mut blocks = BTreeMap::new();
    for b in c["blocks"].as_array().unwrap() {
        let a = b.as_array().unwrap();
        blocks.insert(pos(&json!(a[..3]))?, a[3].as_i64().unwrap());
    }
    let predicate = text(&c["predicate"])?;
    let r = gather::connected(
        pos(&c["origin"])?,
        c["capacity"].as_u64().unwrap() as usize,
        |p| blocks.contains_key(&p),
        |from, to, direction| match predicate {
            "all" => true,
            "faces" => direction.is_some(),
            "edges" => direction.is_none(),
            "same_state" => blocks.get(&from) == blocks.get(&to),
            "reject" => false,
            _ => unreachable!(),
        },
    )
    .map_err(|e| format!("core gather: {e:?}"))?;
    Ok(
        json!({"id":c["id"],"kind":"gather","state":match r.state{gather::GatherState::Success=>"SUCCESS",gather::GatherState::NoBlocks=>"NO_BLOCKS",gather::GatherState::TooManyBlocks=>"TOO_MANY_BLOCKS"},"checked":r.count,"bounds":r.bounds,"blocks":r.blocks}),
    )
}
fn snapshot(
    t: &SplitTracker,
    solids: &BTreeSet<Pos>,
    assembled: &Vec<Vec<Pos>>,
    tick: usize,
    action: &str,
) -> Value {
    let s = t.snapshot();
    let heat: Vec<_> = s
        .heat
        .into_iter()
        .map(|(p, h)| json!([p[0], p[1], p[2], h]))
        .collect();
    let split_map: Vec<_> = s
        .split_indices
        .into_iter()
        .map(|(p, h)| json!([p[0], p[1], p[2], h]))
        .collect();
    json!({"tick":tick,"action":action,"state":match s.state {HeatState::Filling=>"FILLING",HeatState::Clearing=>"CLEARING"},"initialized":s.initialized,"splitComplete":s.split_complete,"solidCount":s.solid_count,"heat":heat,"floodfill":s.floodfill,"removed":s.removed,"newStarts":s.new_starts,"split_map":split_map,"split_index":s.index_map,"solids":solids,"mock_assembled":assembled})
}
fn heat_case(c: &Value) -> Result<Value> {
    let mut tracker = SplitTracker::new();
    let mut solids = BTreeSet::new();
    let mut assembled = Vec::new();
    let mut states = Vec::new();
    let mut tick = 0;
    let mut enabled = true;
    for p in c["initial"].as_array().unwrap() {
        let p = pos(p)?;
        solids.insert(p);
        tracker.on_added(p);
    }
    states.push(snapshot(&tracker, &solids, &assembled, tick, "initial"));
    for a in c["actions"].as_array().unwrap() {
        let op = a["op"].as_str().unwrap();
        match op {
            "tick" => {
                for _ in 0..a["count"].as_u64().unwrap() {
                    tick += 1;
                    let plans =
                        tracker.tick(&solids, c["steps"].as_u64().unwrap() as usize, enabled);
                    for plan in plans {
                        for child in plan.children {
                            for p in &child {
                                if !solids.remove(p) {
                                    return Err("split duplicated content".into());
                                }
                            }
                            assembled.push(child.into_iter().collect());
                        }
                        tracker.reconcile(&solids);
                    }
                    states.push(snapshot(
                        &tracker,
                        &solids,
                        &assembled,
                        tick,
                        if enabled { "tick" } else { "disabled_tick" },
                    ));
                }
            }
            "enabled" => {
                enabled = a["value"].as_bool().unwrap();
                states.push(snapshot(
                    &tracker,
                    &solids,
                    &assembled,
                    tick,
                    if enabled { "enabled" } else { "disabled" },
                ));
            }
            "add" | "remove" => {
                let p = pos(&a["pos"])?;
                if op == "add" {
                    if !solids.insert(p) {
                        return Err("occupied addition".into());
                    }
                    tracker.on_added(p)
                } else {
                    if !solids.remove(&p) {
                        return Err("absent removal".into());
                    }
                    tracker.on_removed(p)
                }
                states.push(snapshot(&tracker, &solids, &assembled, tick, op));
            }
            _ => unreachable!(),
        }
    }
    Ok(json!({"id":c["id"],"kind":"heat","snapshots":states}))
}
fn run() -> Result<()> {
    let args: Vec<_> = std::env::args().collect();
    if args.len() != 2 {
        return Err("one input path required".into());
    }
    let path = &args[1];
    if std::fs::metadata(path).map_err(|e| e.to_string())?.len() > 4_000_000 {
        return Err("input byte limit".into());
    }
    let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
    let doc = strict::parse_document(&bytes)?;
    if doc["schema"] == "sable.phase2.values" {
        println!("{}", values::run(&doc)?);
        return Ok(());
    }
    validate(&doc)?;
    let mut cases = Vec::new();
    for c in doc["cases"].as_array().unwrap() {
        cases.push(if c["kind"] == "gather" {
            gather_case(c)?
        } else {
            heat_case(c)?
        });
    }
    println!(
        "{}",
        json!({"schema":"sable.phase2.candidate-observation","version":1,"cases":cases})
    );
    Ok(())
}
fn main() {
    if let Err(e) = run() {
        eprintln!("phase2 contract/execution refused: {e}");
        std::process::exit(2)
    }
}
