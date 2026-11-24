//! Bounded, transactional ownership transfer. Java's per-block catch/continue is
//! intentionally replaced by preflight and one publication boundary.

use crate::{Contents, Error, Payload, Pos, Result};
use std::collections::BTreeSet;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TransferEvent {
    BeforeMove { from: Pos, to: Pos },
    AfterMove { from: Pos, to: Pos },
    Notify { position: Pos },
    SourceRemoved { position: Pos },
    SendUpdated { position: Pos },
}

/// Preparation must have no externally published side effects. The caller
/// publishes returned events only after both owners have committed.
pub trait TransferHost {
    fn rewrite_payload(&mut self, payload: &Payload, from: Pos, to: Pos) -> Result<Payload>;
    fn prepare_event(&mut self, _event: &TransferEvent) -> Result<()> {
        Ok(())
    }
}

/// The wrapper's coordinates are authoritative. Use only for payload formats
/// that contain no embedded location fields, or whose location is this wrapper.
pub struct PreservePayloads;
impl TransferHost for PreservePayloads {
    fn rewrite_payload(&mut self, payload: &Payload, _from: Pos, _to: Pos) -> Result<Payload> {
        Ok(payload.clone())
    }
}

#[derive(Clone, Debug)]
pub struct TransferPlan {
    source: Contents,
    destination: Contents,
    mapping: Vec<(Pos, Pos)>,
    offset: Pos,
    limit: usize,
}

fn translated(position: Pos, offset: Pos) -> Result<Pos> {
    let mut result = [0; 3];
    for i in 0..3 {
        result[i] = position[i]
            .checked_add(offset[i])
            .ok_or(Error::InvalidCoordinate)?;
    }
    Ok(result)
}

/// Plans are immutable, temporary snapshots, never a second authoritative store.
/// Entity selection is explicit: retained entities anchored in selected blocks
/// follow those blocks; scheduled ticks at selected positions follow as well.
pub fn plan(
    source: &Contents,
    destination: &Contents,
    ordered_blocks: &[Pos],
    offset: Pos,
    limit: usize,
) -> Result<TransferPlan> {
    if ordered_blocks.len() > limit {
        return Err(Error::Capacity);
    }
    let mut ids = BTreeSet::new();
    for contents in [source, destination] {
        for (id, entity) in &contents.entities {
            if id != &entity.id || id.0 == [0; 16] || !ids.insert(*id) {
                return Err(Error::DuplicateId);
            }
            if !entity.position.iter().all(|n| n.is_finite()) {
                return Err(Error::InvalidInput);
            }
        }
        for tick in &contents.ticks {
            if tick.id.0 == [0; 16] || !ids.insert(tick.id) {
                return Err(Error::DuplicateId);
            }
        }
    }
    let mut selected = BTreeSet::new();
    let mut mapping = Vec::new();
    for &from in ordered_blocks {
        if !selected.insert(from) || !source.blocks.contains_key(&from) {
            return Err(Error::InvalidInput);
        }
        let to = translated(from, offset)?;
        if destination.blocks.contains_key(&to) {
            return Err(Error::Occupied);
        }
        mapping.push((from, to));
    }
    for entity in source
        .entities
        .values()
        .filter(|e| selected.contains(&e.anchor))
    {
        if destination.entities.contains_key(&entity.id) {
            return Err(Error::DuplicateId);
        }
        translated(entity.anchor, offset)?;
        if !entity.position.iter().all(|n| n.is_finite()) {
            return Err(Error::InvalidInput);
        }
    }
    let tick_ids: BTreeSet<_> = destination.ticks.iter().map(|t| t.id).collect();
    for tick in source
        .ticks
        .iter()
        .filter(|t| selected.contains(&t.position))
    {
        if tick_ids.contains(&tick.id) {
            return Err(Error::DuplicateId);
        }
    }
    let moved_entities = source
        .entities
        .values()
        .filter(|e| selected.contains(&e.anchor))
        .count();
    let moved_ticks = source
        .ticks
        .iter()
        .filter(|t| selected.contains(&t.position))
        .count();
    let records = destination
        .blocks
        .len()
        .checked_add(destination.entities.len())
        .and_then(|n| n.checked_add(destination.ticks.len()))
        .and_then(|n| n.checked_add(mapping.len()))
        .and_then(|n| n.checked_add(moved_entities))
        .and_then(|n| n.checked_add(moved_ticks))
        .ok_or(Error::Capacity)?;
    if records > limit {
        return Err(Error::Capacity);
    }
    Ok(TransferPlan {
        source: source.clone(),
        destination: destination.clone(),
        mapping,
        offset,
        limit,
    })
}

/// Both owners stay byte-for-byte unchanged if planning, payload rewriting or
/// preparation fails. No external host action is executed by this function.
pub fn move_contents(
    source: &mut Contents,
    destination: &mut Contents,
    plan: &TransferPlan,
    host: &mut impl TransferHost,
) -> Result<Vec<TransferEvent>> {
    if *source != plan.source || *destination != plan.destination {
        return Err(Error::StaleHandle);
    }
    let selected: BTreeSet<_> = plan.mapping.iter().map(|&(from, _)| from).collect();
    let mut next_source = source.clone();
    let mut next_destination = destination.clone();
    let mut events = Vec::new();
    for &(from, to) in &plan.mapping {
        events.push(TransferEvent::BeforeMove { from, to });
        let mut block = next_source.blocks.remove(&from).ok_or(Error::StaleHandle)?;
        if let Some(payload) = &block.block_entity {
            block.block_entity = Some(host.rewrite_payload(payload, from, to)?);
        }
        block.extension = host.rewrite_payload(&block.extension, from, to)?;
        next_destination.blocks.insert(to, block);
        events.push(TransferEvent::AfterMove { from, to });
    }
    let entities: Vec<_> = next_source
        .entities
        .values()
        .filter(|e| selected.contains(&e.anchor))
        .map(|e| e.id)
        .collect();
    for id in entities {
        let mut entity = next_source.entities.remove(&id).ok_or(Error::StaleHandle)?;
        let from = entity.anchor;
        let to = translated(from, plan.offset)?;
        entity.payload = host.rewrite_payload(&entity.payload, from, to)?;
        entity.anchor = to;
        for i in 0..3 {
            entity.position[i] += f64::from(plan.offset[i]);
        }
        next_destination.entities.insert(id, entity);
    }
    let mut retained_ticks = Vec::new();
    for mut tick in next_source.ticks.drain(..) {
        if selected.contains(&tick.position) {
            let from = tick.position;
            let to = translated(from, plan.offset)?;
            tick.payload = host.rewrite_payload(&tick.payload, from, to)?;
            tick.position = to;
            next_destination.ticks.push(tick);
        } else {
            retained_ticks.push(tick);
        }
    }
    next_source.ticks = retained_ticks;
    for &(_, to) in &plan.mapping {
        events.push(TransferEvent::Notify { position: to });
    }
    for &(from, _) in &plan.mapping {
        events.push(TransferEvent::SourceRemoved { position: from });
    }
    // The original final sendBlockUpdated uses the untransformed source position.
    for &(from, _) in &plan.mapping {
        events.push(TransferEvent::SendUpdated { position: from });
    }
    if next_destination.blocks.len()
        + next_destination.entities.len()
        + next_destination.ticks.len()
        > plan.limit
    {
        return Err(Error::Capacity);
    }
    for event in &events {
        host.prepare_event(event)?;
    }
    *source = next_source;
    *destination = next_destination;
    Ok(events)
}
