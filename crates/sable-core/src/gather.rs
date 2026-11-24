//! SubLevelAssemblyHelper.gatherConnectedBlocks: FIFO ordered frontier, 18
//! neighbors, seed bypasses predicate, overflow counted before refusal.
use crate::{Error, Pos, Result};
use std::collections::{BTreeSet, VecDeque};
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GatherState {
    Success,
    NoBlocks,
    TooManyBlocks,
}
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GatherResult {
    pub state: GatherState,
    pub count: usize,
    pub blocks: Option<BTreeSet<Pos>>,
    pub bounds: Option<[i32; 6]>,
}
pub fn in_domain(p: Pos) -> bool {
    p.into_iter()
        .zip([1 << 25, 1 << 23, 1 << 25])
        .all(|(v, h)| (-h..h).contains(&v))
}
pub fn connected(
    seed: Pos,
    capacity: usize,
    mut occupied: impl FnMut(Pos) -> bool,
    mut predicate: impl FnMut(Pos, Pos, Option<Pos>) -> bool,
) -> Result<GatherResult> {
    if !in_domain(seed) {
        return Err(Error::InvalidCoordinate);
    }
    if !occupied(seed) {
        return Ok(GatherResult {
            state: GatherState::NoBlocks,
            count: 0,
            blocks: None,
            bounds: None,
        });
    }
    let mut pending = BTreeSet::from([seed]);
    let mut queue = VecDeque::from([seed]);
    let mut visited = BTreeSet::new();
    let mut count = 0;
    let mut bounds = [seed[0], seed[1], seed[2], seed[0], seed[1], seed[2]];
    while let Some(p) = queue.pop_front() {
        pending.remove(&p);
        count += 1;
        if count > capacity {
            return Ok(GatherResult {
                state: GatherState::TooManyBlocks,
                count,
                blocks: None,
                bounds: None,
            });
        }
        visited.insert(p);
        for i in 0..3 {
            bounds[i] = bounds[i].min(p[i]);
            bounds[i + 3] = bounds[i + 3].max(p[i]);
        }
        for x in -1i32..=1 {
            for y in -1i32..=1 {
                for z in -1i32..=1 {
                    let distance = x.abs() + y.abs() + z.abs();
                    if distance == 0 || distance == 3 {
                        continue;
                    }
                    let c = [p[0] + x, p[1] + y, p[2] + z];
                    if !in_domain(c) {
                        continue;
                    }
                    if !occupied(c)
                        || !predicate(p, c, if distance == 1 { Some([x, y, z]) } else { None })
                    {
                        continue;
                    }
                    if !visited.contains(&c) && pending.insert(c) {
                        queue.push_back(c)
                    }
                }
            }
        }
    }
    Ok(GatherResult {
        state: GatherState::Success,
        count,
        blocks: Some(visited),
        bounds: Some(bounds),
    })
}
