// Derived from Sable 9b1a3d1555d35db9fb7b1103701e7490f0a968b9.
// PolyForm Shield 1.0.0; see ../LICENSE-SABLE.md and provenance/origin.json.
use marten::Real;

use crate::get_physics_state_mut;

/// Global spring frequency for joints (Hz)
pub const JOINT_SPRING_FREQUENCY: Real = 550.0;

/// Global damping ratio for joints
pub const JOINT_SPRING_DAMPING_RATIO: Real = 4.0;

#[allow(non_snake_case)]
pub(crate) fn configFrequencyAndDamping(
    collision_natural_frequency: f64,
    collision_damping_ratio: f64,
) {
    let mut state = get_physics_state_mut();
    state
        .integration_parameters
        .contact_softness
        .natural_frequency = collision_natural_frequency as Real;
    state.integration_parameters.contact_softness.damping_ratio = collision_damping_ratio as Real;
}
#[allow(non_snake_case)]
pub(crate) fn configSolverIterations(
    num_solver_iterations: i32,
    num_internal_pgs_iterations: i32,
    num_internal_stabilization_iterations: i32,
) {
    let mut state = get_physics_state_mut();
    state.integration_parameters.num_solver_iterations = num_solver_iterations as usize;
    state.integration_parameters.num_internal_pgs_iterations = num_internal_pgs_iterations as usize;
    state
        .integration_parameters
        .num_internal_stabilization_iterations = num_internal_stabilization_iterations as usize;
}
#[allow(non_snake_case)]
pub(crate) fn configMinIslandSize(island_size: i32) {
    get_physics_state_mut()
        .integration_parameters
        .min_island_size = island_size as usize;
}
