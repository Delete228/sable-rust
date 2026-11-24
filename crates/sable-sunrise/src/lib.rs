//! Sunrise integration data, contracts and actual native-physics realization.
//! Runtime host calls live in the separately SDK-built `adapters/sunrise-mod`.
//! This crate owns no live inventory or ticker: registry facts and geometry are
//! derived host inputs. Capability reports are not runtime execution evidence.
//! See docs/phase-3-adapter.md for the historical contract-only scope and
//! docs/phase-3-continuation-simulation.md for the communicating native slice.

pub mod capabilities;
pub mod changes;
pub mod frame;
pub mod registry;
pub mod simulation;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    InvalidState,
    DuplicateState,
    UnknownState,
    InvalidShape,
    ContextDependentShape,
    InvalidPolicy,
    StaleRegistry,
    InvalidRevision,
    StaleOwner,
    InvalidConfiguration,
    InvalidCoordinate,
}

pub type Result<T> = std::result::Result<T, Error>;
