//! Contract tests with explicit supplied facts; no Sunrise host runs here.
use sable_core::{Config, Id};
use sable_sunrise::{
    Error,
    capabilities::{Capability, CapabilityReport, REQUIRED},
    changes::{CellIdentity, CellStamp, Invalidation},
    registry::{
        CanonicalState, HostGeometry, PhysicsPolicy, PropertyCatalog, ResolvedState,
        StateDefinition,
    },
};

fn state(name: &str) -> CanonicalState {
    StateDefinition::new(name.into(), vec![])
        .unwrap()
        .resolve(vec![])
        .unwrap()
}

fn geometry() -> HostGeometry {
    HostGeometry {
        state_invariant: true,
        solid: true,
        full: true,
        boxes: vec![[0., 0., 0., 1., 1., 1.]],
    }
}

fn policy(mass: f64) -> PhysicsPolicy {
    PhysicsPolicy {
        rule: "explicit test policy; not a host or game registry".into(),
        mass,
        center: [0.5; 3],
        inertia: None,
    }
}

fn catalog(epoch: u64, name: &str, mass: f64) -> PropertyCatalog {
    let mut catalog = PropertyCatalog::new(epoch).unwrap();
    catalog
        .register(state(name), geometry(), policy(mass))
        .unwrap();
    catalog
}

fn identity() -> CellIdentity {
    CellIdentity {
        owner: Id([1; 16]),
        generation: 1,
        local: [15, -64, 16],
    }
}

fn stamp(
    identity: CellIdentity,
    epoch: u64,
    state: &ResolvedState,
    revision: u64,
) -> sable_sunrise::Result<CellStamp> {
    CellStamp::new(&Config::default(), identity, epoch, state, revision)
}

#[test]
fn every_missing_required_capability_refuses_playable_admission() {
    assert_eq!(CapabilityReport::default().missing(), REQUIRED);
    for missing in REQUIRED {
        let report = CapabilityReport::reported(REQUIRED.into_iter().filter(|c| *c != missing));
        assert_eq!(report.require_playable_contracts(), Err(vec![missing]));
    }
    assert_eq!(
        CapabilityReport::reported(REQUIRED).require_playable_contracts(),
        Ok(())
    );
    // Rendering and ordinary dimensions cannot substitute for a live holder,
    // atomic transfer/save and ordinary transformed authoritative action paths.
    let partial = CapabilityReport::reported([
        Capability::LiveLocalRendering,
        Capability::MovingContactAndSupport,
        Capability::CoherentTickPublication,
    ]);
    assert_eq!(
        partial.missing(),
        vec![
            Capability::LiveLocalWorld,
            Capability::AtomicOwnershipAndSave,
            Capability::TransformedAuthoritativeActions,
        ]
    );
}

#[test]
fn canonical_properties_are_complete_unique_and_order_independent() {
    let definition = StateDefinition::new(
        "minecraft:oak_slab".into(),
        vec![
            ("waterlogged".into(), vec!["false".into(), "true".into()]),
            (
                "type".into(),
                vec!["bottom".into(), "top".into(), "double".into()],
            ),
        ],
    )
    .unwrap();
    let a = vec![
        ("waterlogged".into(), "false".into()),
        ("type".into(), "top".into()),
    ];
    let mut b = a.clone();
    b.reverse();
    let canonical = definition.resolve(a).unwrap();
    assert_eq!(canonical, definition.resolve(b).unwrap());
    assert_eq!(
        canonical.key(),
        "minecraft:oak_slab[type=top,waterlogged=false]"
    );
    for invalid in [
        vec![("type".into(), "top".into())],
        vec![
            ("type".into(), "top".into()),
            ("type".into(), "bottom".into()),
        ],
        vec![
            ("type".into(), "sideways".into()),
            ("waterlogged".into(), "false".into()),
        ],
        vec![
            ("type".into(), "top".into()),
            ("unknown".into(), "false".into()),
        ],
    ] {
        assert_eq!(definition.resolve(invalid), Err(Error::InvalidState));
    }
}

#[test]
fn malformed_definitions_cannot_alias_saved_state_keys() {
    for name in ["stone", "Minecraft:stone", "minecraft:a[b=c]", "minecraft:"] {
        assert_eq!(
            StateDefinition::new(name.into(), vec![]),
            Err(Error::InvalidState)
        );
    }
    for definitions in [
        vec![("axis".into(), vec![])],
        vec![("axis".into(), vec!["x".into(), "x".into()])],
        vec![("axis".into(), vec!["x,y=z".into()])],
        vec![
            ("axis".into(), vec!["x".into()]),
            ("axis".into(), vec!["z".into()]),
        ],
    ] {
        assert_eq!(
            StateDefinition::new("minecraft:stone".into(), definitions),
            Err(Error::InvalidState)
        );
    }
}

#[test]
fn unknown_states_and_old_registry_epochs_have_no_mass_fallback() {
    let resolved = catalog(12, "minecraft:blackstone", 2.);
    assert_eq!(
        resolved.get(12, "minecraft:unknown"),
        Err(Error::UnknownState)
    );
    assert_eq!(
        resolved.get(11, "minecraft:blackstone"),
        Err(Error::StaleRegistry)
    );
    assert!(
        resolved
            .mass_registry(12, ["minecraft:blackstone", "minecraft:unknown"])
            .is_err()
    );
    let mass = resolved
        .mass_registry(12, ["minecraft:blackstone"])
        .unwrap();
    assert_eq!(mass.get("minecraft:blackstone").unwrap().mass, 2.);
    assert!(mass.get("minecraft:bedrock").is_err());
}

#[test]
fn rejected_geometry_or_policy_does_not_change_catalog() {
    let mut resolved = catalog(1, "minecraft:blackstone", 2.);
    let original = resolved.get(1, "minecraft:blackstone").unwrap().clone();
    let mut context_dependent = geometry();
    context_dependent.state_invariant = false;
    assert_eq!(
        resolved.register(state("minecraft:oak_fence"), context_dependent, policy(1.)),
        Err(Error::ContextDependentShape)
    );
    for invalid_box in [
        [0., 0., 0., f64::NAN, 1., 1.],
        [0., 0., 0., 1e300, 1., 1.],
        [0., 0., 0., f64::MIN_POSITIVE, 1., 1.],
        [0., 0., 0., 0., 1., 1.],
        [1., 0., 0., 0., 1., 1.],
    ] {
        let mut invalid = geometry();
        invalid.boxes = vec![invalid_box];
        assert_eq!(
            resolved.register(state("minecraft:oak_fence"), invalid, policy(1.)),
            Err(Error::InvalidShape)
        );
    }
    for mass in [f64::NAN, f64::INFINITY, -1.] {
        assert_eq!(
            resolved.register(state("minecraft:oak_fence"), geometry(), policy(mass)),
            Err(Error::InvalidPolicy)
        );
    }
    assert_eq!(
        resolved.register(state("minecraft:blackstone"), geometry(), policy(500.)),
        Err(Error::DuplicateState)
    );
    assert_eq!(resolved.get(1, "minecraft:blackstone").unwrap(), &original);
    assert_eq!(
        resolved.get(1, "minecraft:oak_fence"),
        Err(Error::UnknownState)
    );
}

#[test]
fn inconsistent_solidity_and_shapes_refuse_before_catalog_installation() {
    let mut resolved = PropertyCatalog::new(1).unwrap();
    let mut noncolliding_solid = geometry();
    noncolliding_solid.full = false;
    noncolliding_solid.boxes.clear();
    let mut colliding_nonsolid = geometry();
    colliding_nonsolid.full = false;
    colliding_nonsolid.solid = false;
    for invalid in [noncolliding_solid, colliding_nonsolid] {
        assert_eq!(
            resolved.register(state("minecraft:stone"), invalid, policy(1.)),
            Err(Error::InvalidShape)
        );
        assert_eq!(resolved.get(1, "minecraft:stone"), Err(Error::UnknownState));
    }
    let empty = HostGeometry {
        state_invariant: true,
        solid: false,
        full: false,
        boxes: vec![],
    };
    resolved
        .register(state("minecraft:torch"), empty, policy(0.))
        .unwrap();
    assert!(
        !resolved
            .get(1, "minecraft:torch")
            .unwrap()
            .properties()
            .solid
    );
}

#[test]
fn cell_stamp_checks_core_plot_local_domain_and_config_before_construction() {
    let resolved = catalog(1, "minecraft:blackstone", 2.);
    let state = resolved.get(1, "minecraft:blackstone").unwrap();
    let config = Config::default();
    for local in [
        [0, config.min_y, 0],
        [15, 0, 16],
        [16, 0, 15],
        [config.side() - 1, config.max_y - 1, config.side() - 1],
    ] {
        let cell = CellIdentity {
            local,
            ..identity()
        };
        assert!(CellStamp::new(&config, cell, 1, state, 0).is_ok());
    }
    for local in [
        [-17, 0, 16],
        [-1, 0, 0],
        [0, 0, -1],
        [config.side(), 0, 0],
        [0, 0, config.side()],
        [0, config.min_y - 1, 0],
        [0, config.max_y, 0],
    ] {
        let cell = CellIdentity {
            local,
            ..identity()
        };
        assert_eq!(
            CellStamp::new(&config, cell, 1, state, 0),
            Err(Error::InvalidCoordinate)
        );
    }
    let invalid = Config {
        log_plot: 0,
        ..config
    };
    assert_eq!(
        CellStamp::new(&invalid, identity(), 1, state, 0),
        Err(Error::InvalidConfiguration)
    );
}

#[test]
fn content_updates_and_unchanged_ticks_do_not_invalidate_mass_or_geometry() {
    let resolved = catalog(1, "minecraft:blackstone", 2.);
    let state = resolved.get(1, "minecraft:blackstone").unwrap();
    let mut old = stamp(identity(), 1, state, 1).unwrap();
    for revision in 2..1000 {
        assert_eq!(old.classify(&old).unwrap(), Invalidation::default());
        let next = stamp(identity(), 1, state, revision).unwrap();
        assert_eq!(
            old.classify(&next).unwrap(),
            Invalidation {
                geometry: false,
                mass: false,
                render: true,
                content: true,
            }
        );
        old = next;
    }
}

#[test]
fn registry_policy_and_shape_changes_invalidate_separate_consumers() {
    let first = catalog(1, "minecraft:blackstone", 2.);
    let second = catalog(2, "minecraft:blackstone", 4.);
    let old = stamp(
        identity(),
        1,
        first.get(1, "minecraft:blackstone").unwrap(),
        3,
    )
    .unwrap();
    let next = stamp(
        identity(),
        2,
        second.get(2, "minecraft:blackstone").unwrap(),
        3,
    )
    .unwrap();
    assert_eq!(
        old.classify(&next).unwrap(),
        Invalidation {
            mass: true,
            ..Invalidation::default()
        }
    );
    let mut third = PropertyCatalog::new(3).unwrap();
    let mut slab = geometry();
    slab.full = false;
    slab.boxes = vec![[0., 0., 0., 1., 0.5, 1.]];
    third
        .register(state("minecraft:blackstone"), slab, policy(4.))
        .unwrap();
    let shaped = stamp(
        identity(),
        3,
        third.get(3, "minecraft:blackstone").unwrap(),
        3,
    )
    .unwrap();
    assert_eq!(
        next.classify(&shaped).unwrap(),
        Invalidation {
            geometry: true,
            render: true,
            ..Invalidation::default()
        }
    );
}

#[test]
fn recycled_owner_and_regressing_epochs_or_revisions_refuse() {
    let first = catalog(2, "minecraft:blackstone", 2.);
    let second = catalog(1, "minecraft:blackstone", 2.);
    let state = first.get(2, "minecraft:blackstone").unwrap();
    assert_eq!(stamp(identity(), 1, state, 2), Err(Error::StaleRegistry));
    let old = stamp(identity(), 2, state, 2).unwrap();
    let mut recycled = identity();
    recycled.generation += 1;
    assert_eq!(
        old.classify(&stamp(recycled, 2, state, 3).unwrap()),
        Err(Error::StaleOwner)
    );
    assert_eq!(
        old.classify(&stamp(identity(), 2, state, 1).unwrap()),
        Err(Error::InvalidRevision)
    );
    let stale = stamp(
        identity(),
        1,
        second.get(1, "minecraft:blackstone").unwrap(),
        3,
    )
    .unwrap();
    assert_eq!(old.classify(&stale), Err(Error::StaleRegistry));
}

#[test]
fn equal_physics_different_state_keeps_geometry_and_mass_cached() {
    let mut resolved = catalog(1, "minecraft:blackstone", 2.);
    resolved
        .register(
            state("minecraft:polished_blackstone"),
            geometry(),
            policy(2.),
        )
        .unwrap();
    let old = stamp(
        identity(),
        1,
        resolved.get(1, "minecraft:blackstone").unwrap(),
        1,
    )
    .unwrap();
    let next = stamp(
        identity(),
        1,
        resolved.get(1, "minecraft:polished_blackstone").unwrap(),
        2,
    )
    .unwrap();
    assert_eq!(
        old.classify(&next).unwrap(),
        Invalidation {
            render: true,
            content: true,
            ..Invalidation::default()
        }
    );
    let unversioned = stamp(
        identity(),
        1,
        resolved.get(1, "minecraft:polished_blackstone").unwrap(),
        1,
    )
    .unwrap();
    assert_eq!(old.classify(&unversioned), Err(Error::InvalidRevision));
}
