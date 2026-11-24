//! Closed callback fixture input contract v2. Validation does not create an Engine.
use serde::{
    Deserialize, Deserializer,
    de::{self, MapAccess, SeqAccess, Visitor},
};
use serde_json::{Map, Value};
use std::collections::HashSet;

pub const VERSION: u32 = 2;
const REFERENCE: &str = "unchanged native / ABI-adapted fixture";
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Accept,
    Reject,
    ImmediateWorld,
    ReentrantRead,
    ReentrantChange,
    ReentrantMaterial,
}
impl Mode {
    pub fn is_reentrant(self) -> bool {
        matches!(
            self,
            Self::ReentrantRead | Self::ReentrantChange | Self::ReentrantMaterial
        )
    }
}
#[derive(Debug)]
pub struct ValidatedCase {
    id: String,
    mode: Mode,
}
impl ValidatedCase {
    pub fn id(&self) -> &str {
        &self.id
    }
    pub fn mode(&self) -> Mode {
        self.mode
    }
}
fn error(message: &str) -> String {
    format!("Callback contract v{VERSION}: {message}")
}
fn spec(id: &str) -> Option<(&'static str, Mode, &'static [&'static str], &'static str)> {
    Some(match id {
        "callback-accept" => (
            "accept",
            Mode::Accept,
            &["synchronous_callback_order", "contact_acceptance"],
            "success",
        ),
        "callback-reject" => (
            "reject",
            Mode::Reject,
            &["synchronous_callback_order", "contact_rejection"],
            "success",
        ),
        "callback-immediate-world" => (
            "immediate_java_world",
            Mode::ImmediateWorld,
            &[
                "immediate_fixture_world_mutation",
                "subsequent_callback_visibility",
                "contact_rejection",
            ],
            "success",
        ),
        "callback-reentrant-read" => ("reentrant_pose_query", Mode::ReentrantRead, &[], "timeout"),
        "callback-reentrant-change" => (
            "reentrant_change_block",
            Mode::ReentrantChange,
            &[
                "synchronous_native_world_mutation",
                "post_step_geometry_visibility",
            ],
            "success",
        ),
        "callback-reentrant-material" => (
            "reentrant_material_edit",
            Mode::ReentrantMaterial,
            &[],
            "timeout",
        ),
        _ => return None,
    })
}
fn object(value: &Value) -> Result<&Map<String, Value>, String> {
    value.as_object().ok_or_else(|| error("expected object"))
}
fn keys(value: &Map<String, Value>, required: &[&str], optional: &[&str]) -> Result<(), String> {
    if required.iter().any(|key| !value.contains_key(*key))
        || value
            .keys()
            .any(|key| !required.contains(&key.as_str()) && !optional.contains(&key.as_str()))
    {
        return Err(error("unknown or missing field"));
    }
    Ok(())
}
pub fn is_callback(id: &str, scenario: &Value) -> bool {
    id.starts_with("callback-")
        || scenario
            .get("acceptance_scope")
            .and_then(Value::as_array)
            .is_some_and(|scope| {
                scope.iter().any(|item| {
                    matches!(
                        item.as_str(),
                        Some(
                            "synchronous_callback_order"
                                | "contact_acceptance"
                                | "contact_rejection"
                                | "immediate_fixture_world_mutation"
                                | "subsequent_callback_visibility"
                                | "synchronous_native_world_mutation"
                                | "post_step_geometry_visibility"
                        )
                    )
                })
            })
        || scenario
            .get("actions")
            .and_then(Value::as_array)
            .is_some_and(|actions| {
                actions
                    .iter()
                    .any(|a| a.get("op").and_then(Value::as_str) == Some("callback_fixture"))
            })
}
/// Existing root metadata distinguishes the two reviewed fixture families.
/// A callback document remains a callback document even if its ID/actions lie.
pub fn callback_document(root: &Value) -> Result<bool, String> {
    let root = object(root)?;
    if root.contains_key("reference_kind") {
        return Ok(true);
    }
    keys(
        root,
        &[
            "schema",
            "schema_version",
            "configuration_name",
            "units",
            "frames",
            "scenarios",
        ],
        &["discrete_equality", "numeric_policy"],
    )?;
    if root["configuration_name"].as_str() != Some(REFERENCE)
        || ["units", "frames"].iter().any(|key| {
            !root[*key]
                .as_object()
                .is_some_and(|map| !map.is_empty() && map.values().all(Value::is_string))
        })
    {
        return Err(error("malformed core document family"));
    }
    Ok(false)
}
/// Refuse empty/malformed generic streams before allocating physics state.
pub fn core_actions(scenario: &Value) -> Result<&[Value], String> {
    let actions = scenario
        .get("actions")
        .and_then(Value::as_array)
        .filter(|actions| !actions.is_empty())
        .ok_or_else(|| error("core actions must be a nonempty array"))?;
    for action in actions {
        if !action.is_object()
            || !matches!(
                action.get("op").and_then(Value::as_str),
                Some(
                    "scene"
                        | "config"
                        | "material"
                        | "material_clear"
                        | "material_add_box"
                        | "box"
                        | "sublevel"
                        | "chunk"
                        | "chunk_remove"
                        | "impulse"
                        | "velocity"
                        | "wake"
                        | "step"
                        | "sample"
                        | "collisions"
                        | "fixed_joint"
                        | "joint_sample"
                        | "joint_remove"
                        | "body_remove"
                        | "dispose"
                )
            )
        {
            return Err(error("unknown or malformed core operation"));
        }
    }
    Ok(actions)
}
pub fn validate(root: &Value, requested: &str) -> Result<ValidatedCase, String> {
    let root = object(root)?;
    keys(
        root,
        &["schema", "schema_version", "reference_kind", "scenarios"],
        &[],
    )?;
    if root["schema"].as_str() != Some("sable.native_scenarios")
        || root["schema_version"].as_u64() != Some(1)
        || root["reference_kind"].as_str() != Some(REFERENCE)
    {
        return Err(error("schema/version/reference kind mismatch"));
    }
    let scenarios = root["scenarios"]
        .as_array()
        .ok_or_else(|| error("scenarios must be an array"))?;
    if scenarios.is_empty() || scenarios.len() > 6 {
        return Err(error("one to six unique cases required"));
    }
    let mut seen = HashSet::new();
    let mut selected = None;
    for scenario in scenarios {
        let s = object(scenario)?;
        keys(
            s,
            &[
                "id",
                "repeat_count",
                "timeout_seconds",
                "acceptance_scope",
                "actions",
            ],
            &["expected_outcome"],
        )?;
        let id = s["id"].as_str().ok_or_else(|| error("ID must be string"))?;
        let (mode, kind, scope, outcome) = spec(id).ok_or_else(|| error("unknown callback ID"))?;
        if !seen.insert(id) {
            return Err(error("duplicate scenario ID"));
        }
        let actions = s["actions"]
            .as_array()
            .ok_or_else(|| error("actions must be an array"))?;
        if actions.len() != 1 {
            return Err(error("exactly one callback action required"));
        }
        let action = object(&actions[0])?;
        keys(action, &["op", "mode"], &[])?;
        if action["op"].as_str() != Some("callback_fixture")
            || action["mode"].as_str() != Some(mode)
        {
            return Err(error("ID/mode/action binding mismatch"));
        }
        let actual_scope = s["acceptance_scope"]
            .as_array()
            .ok_or_else(|| error("scope must be array"))?;
        if actual_scope.len() != scope.len()
            || actual_scope
                .iter()
                .zip(scope)
                .any(|(a, b)| a.as_str() != Some(*b))
        {
            return Err(error("unsupported eligibility scope"));
        }
        let expected = s.get("expected_outcome").unwrap_or(&Value::Null);
        if (expected.is_null() && outcome != "success")
            || (!expected.is_null() && expected.as_str() != Some(outcome))
            || (s.contains_key("expected_outcome") && expected.is_null())
        {
            return Err(error("expected outcome mismatch"));
        }
        if !s["repeat_count"]
            .as_u64()
            .is_some_and(|v| (1..=20).contains(&v))
        {
            return Err(error("repeat_count must be integer in 1..20"));
        }
        if !s["timeout_seconds"]
            .as_f64()
            .is_some_and(|v| v.is_finite() && (0.1..=60.).contains(&v))
        {
            return Err(error("timeout_seconds must be finite in 0.1..60"));
        }
        if id == requested {
            selected = Some(ValidatedCase {
                id: id.into(),
                mode: kind,
            });
        }
    }
    selected.ok_or_else(|| error("requested ID must resolve to one validated case"))
}
pub fn dispatch<T>(
    root: &Value,
    id: &str,
    execute: impl FnOnce(ValidatedCase) -> T,
) -> Result<T, String> {
    let case = validate(root, id)?;
    Ok(execute(case))
}
/// Deserialize through serde's standard JSON parser while refusing duplicate keys.
struct StrictValue(Value);
impl<'de> Deserialize<'de> for StrictValue {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        deserializer.deserialize_any(StrictVisitor).map(Self)
    }
}
struct StrictVisitor;
impl<'de> Visitor<'de> for StrictVisitor {
    type Value = Value;
    fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
        formatter.write_str("JSON with unique object keys")
    }
    fn visit_bool<E: de::Error>(self, value: bool) -> Result<Value, E> {
        Ok(Value::Bool(value))
    }
    fn visit_i64<E: de::Error>(self, value: i64) -> Result<Value, E> {
        Ok(Value::Number(value.into()))
    }
    fn visit_u64<E: de::Error>(self, value: u64) -> Result<Value, E> {
        Ok(Value::Number(value.into()))
    }
    fn visit_f64<E: de::Error>(self, value: f64) -> Result<Value, E> {
        serde_json::Number::from_f64(value)
            .map(Value::Number)
            .ok_or_else(|| E::custom("nonfinite JSON number"))
    }
    fn visit_str<E: de::Error>(self, value: &str) -> Result<Value, E> {
        Ok(Value::String(value.into()))
    }
    fn visit_string<E: de::Error>(self, value: String) -> Result<Value, E> {
        Ok(Value::String(value))
    }
    fn visit_none<E: de::Error>(self) -> Result<Value, E> {
        Ok(Value::Null)
    }
    fn visit_unit<E: de::Error>(self) -> Result<Value, E> {
        Ok(Value::Null)
    }
    fn visit_seq<A: SeqAccess<'de>>(self, mut source: A) -> Result<Value, A::Error> {
        let mut values = Vec::new();
        while let Some(StrictValue(value)) = source.next_element()? {
            values.push(value);
        }
        Ok(Value::Array(values))
    }
    fn visit_map<A: MapAccess<'de>>(self, mut source: A) -> Result<Value, A::Error> {
        let mut values = Map::new();
        while let Some(key) = source.next_key::<String>()? {
            if values.contains_key(&key) {
                return Err(de::Error::custom("duplicate JSON key"));
            }
            values.insert(key, source.next_value::<StrictValue>()?.0);
        }
        Ok(Value::Object(values))
    }
}
pub fn parse_document(bytes: &[u8]) -> Result<Value, String> {
    let mut parser = serde_json::Deserializer::from_slice(bytes);
    let value =
        StrictValue::deserialize(&mut parser).map_err(|e| error(&format!("invalid JSON: {e}")))?;
    parser
        .end()
        .map_err(|e| error(&format!("invalid trailing JSON: {e}")))?;
    Ok(value.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    fn document() -> Value {
        serde_json::from_str(include_str!("../../abi_adapted/callbacks.v1.json")).unwrap()
    }
    #[test]
    fn all_six_historical_bindings_validate_without_execution() {
        let root = document();
        for s in root["scenarios"].as_array().unwrap() {
            validate(&root, s["id"].as_str().unwrap()).unwrap();
        }
    }
    #[test]
    fn adverse_inputs_never_reach_callback_executor() {
        let original = document();
        let mut mutations = Vec::new();
        for (field, value) in [
            ("actions", json!([])),
            ("actions", Value::Null),
            ("actions", json!([{"op":"scene"}])),
            (
                "actions",
                json!([{"op":"callback_fixture","mode":"reject"}]),
            ),
            (
                "actions",
                json!([{"op":"callback_fixture","mode":"unknown"}]),
            ),
            ("actions", json!([{"op":"callback_fixture","mode":false}])),
            (
                "actions",
                json!([{"op":"callback_fixture","mode":"accept","extra":1}]),
            ),
            (
                "actions",
                json!([{"op":"callback_fixture","mode":"accept"},{"op":"callback_fixture","mode":"accept"}]),
            ),
            ("repeat_count", json!(true)),
            ("repeat_count", json!(1.0)),
            ("timeout_seconds", json!(false)),
            ("acceptance_scope", json!(["contact_rejection"])),
        ] {
            let mut root = original.clone();
            root["scenarios"][0][field] = value;
            mutations.push(root);
        }
        let mut duplicate = original.clone();
        duplicate["scenarios"][1] = duplicate["scenarios"][0].clone();
        mutations.push(duplicate);
        let mut wrongid = original.clone();
        wrongid["scenarios"][0]["id"] = json!("unknown");
        mutations.push(wrongid);
        let mut unknownfield = original.clone();
        unknownfield["scenarios"][0]["unknown"] = json!(1);
        mutations.push(unknownfield);
        for root in mutations {
            let mut executed = false;
            assert!(
                dispatch(&root, "callback-accept", |_| {
                    executed = true;
                })
                .is_err()
            );
            assert!(!executed, "invalid contract reached Engine executor");
        }
    }
    #[test]
    fn duplicate_json_keys_rejected_before_dispatch() {
        assert!(parse_document(br#"{"schema":1,"schema":2}"#).is_err());
    }
    #[test]
    fn valid_unicode_escape_preserves_mode_and_signed_escapes_are_invalid() {
        let raw = serde_json::to_string(&document()).unwrap();
        let escaped = raw.replacen("\"mode\":\"accept\"", r#""mode":"\u0061ccept""#, 1);
        assert_eq!(
            validate(
                &parse_document(escaped.as_bytes()).unwrap(),
                "callback-accept"
            )
            .unwrap()
            .mode(),
            Mode::Accept
        );
        for escape in [r#""mode":"\u+061ccept""#, r#""mode":"\u-061ccept""#] {
            assert!(
                parse_document(raw.replacen("\"mode\":\"accept\"", escape, 1).as_bytes()).is_err()
            );
        }
    }
    #[test]
    fn known_callback_id_cannot_fall_through_malformed_action() {
        for action in [Value::Null, json!([]), json!([{"op":"scene"}])] {
            assert!(is_callback("callback-accept", &json!({"actions":action})));
        }
        assert!(is_callback(
            "unknown",
            &json!({"actions":[{"op":"callback_fixture"}]})
        ));
    }
    #[test]
    fn callback_document_cannot_masquerade_as_core() {
        for actions in [
            json!([]),
            json!([{"op":"scene","scene":"a","gravity":[0,-9.81,0],"drag":0}]),
        ] {
            let mut root = document();
            root["scenarios"][0]["id"] = json!("ordinary");
            root["scenarios"][0]["actions"] = actions;
            assert!(callback_document(&root).unwrap());
            let mut executed = false;
            assert!(
                dispatch(&root, "ordinary", |_| {
                    executed = true;
                })
                .is_err()
            );
            assert!(!executed);
        }
        let mut missing = document();
        missing.as_object_mut().unwrap().remove("reference_kind");
        assert!(callback_document(&missing).is_err());
        let mut mixed = document();
        mixed["configuration_name"] = json!(REFERENCE);
        assert!(validate(&mixed, "callback-accept").is_err());
    }
    #[test]
    fn tracked_core_family_and_nonempty_operations_remain_valid() {
        let root: Value =
            serde_json::from_str(include_str!("../../abi_adapted/scenarios.v1.json")).unwrap();
        assert!(!callback_document(&root).unwrap());
        for scenario in root["scenarios"].as_array().unwrap() {
            core_actions(scenario).unwrap();
        }
        for actions in [
            Value::Null,
            json!([]),
            json!([false]),
            json!([{"op":"unknown"}]),
        ] {
            assert!(core_actions(&json!({"actions":actions})).is_err());
        }
    }
}
