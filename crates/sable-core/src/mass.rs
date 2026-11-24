//! MassTracker.build's two passes over resolved per-state properties. Resolution
//! is explicit: unknown states fail, rather than silently using unit test mass.
use crate::{Contents, Error, Result};
use std::collections::BTreeMap;
#[derive(Debug, Clone, PartialEq)]
pub struct Properties {
    pub solid: bool,
    pub full: bool,
    pub mass: f64,
    pub center: [f64; 3],
    pub inertia: Option<[f64; 3]>,
    pub boxes: Vec<[f64; 6]>,
}
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MassRegistry {
    states: BTreeMap<String, Properties>,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mass {
    pub mass: f64,
    pub center: [f64; 3],
    pub inertia: [f64; 9],
}
impl MassRegistry {
    pub fn register(&mut self, key: String, p: Properties) -> Result<()> {
        if key.is_empty()
            || !p.mass.is_finite()
            || p.mass < 0.
            || !p.center.iter().all(|x| x.is_finite())
            || p.inertia
                .is_some_and(|a| a.iter().any(|x| !x.is_finite() || *x < 0.))
            || p.boxes
                .iter()
                .any(|b| !b.iter().all(|x| x.is_finite()) || (0..3).any(|i| b[i] > b[i + 3]))
        {
            return Err(Error::InvalidInput);
        }
        self.states.insert(key, p);
        Ok(())
    }
    pub fn get(&self, key: &str) -> Result<&Properties> {
        self.states.get(key).ok_or(Error::InvalidInput)
    }
    /// Bounded pinned resource profile, not a Minecraft registry emulator.
    /// heavy.json + tags/block/heavy.json explicitly include blackstone;
    /// bedrock_absurdity.json explicitly sets bedrock mass to 1000.
    pub fn pinned_full_blocks() -> Self {
        let mut r = Self::default();
        for (key, mass) in [("minecraft:blackstone", 2.), ("minecraft:bedrock", 1000.)] {
            r.register(
                key.into(),
                Properties {
                    solid: true,
                    full: true,
                    mass,
                    center: [0.5; 3],
                    inertia: None,
                    boxes: vec![[0., 0., 0., 1., 1., 1.]],
                },
            )
            .unwrap();
        }
        r
    }
    pub fn compute(&self, c: &Contents) -> Result<Mass> {
        let mut mass = 0.;
        let mut center = [0.; 3];
        for (pos, b) in &c.blocks {
            let p = self.get(&b.state)?;
            if !p.solid {
                continue;
            }
            mass += p.mass;
            for i in 0..3 {
                center[i] += p.mass * (pos[i] as f64 + p.center[i]);
            }
        }
        if mass <= 0. || !mass.is_finite() {
            return Err(Error::InvalidInput);
        }
        let inverse_mass = 1. / mass;
        for v in &mut center {
            *v *= inverse_mass;
        }
        let mut inertia = [0.; 9];
        for (pos, b) in &c.blocks {
            let p = self.get(&b.state)?;
            if !p.solid {
                continue;
            }
            let r: [f64; 3] = std::array::from_fn(|i| pos[i] as f64 + p.center[i] - center[i]);
            for i in 0..3 {
                inertia[i * 3 + i] += p.inertia.map_or(p.mass / 6., |diag| diag[i] * p.mass);
            }
            // Column-major symmetric parallel-axis tensor; same add order as Java.
            inertia[0] += (r[1] * r[1] + r[2] * r[2]) * p.mass;
            inertia[4] += (r[2] * r[2] + r[0] * r[0]) * p.mass;
            inertia[8] += (r[0] * r[0] + r[1] * r[1]) * p.mass;
            for (a, b) in [(0, 1), (0, 2), (1, 2)] {
                inertia[a * 3 + b] -= r[b] * r[a] * p.mass;
                inertia[b * 3 + a] -= r[a] * r[b] * p.mass;
            }
        }
        if !inertia.iter().all(|v| v.is_finite()) {
            return Err(Error::InvalidInput);
        }
        Ok(Mass {
            mass,
            center,
            inertia,
        })
    }
}
