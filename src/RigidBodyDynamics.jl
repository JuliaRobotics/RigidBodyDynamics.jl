__precompile__()

module RigidBodyDynamics

using Base.Random
using StaticArrays
using Rotations
using TypeSortedCollections
using LightXML
using DocStringExtensions
using Compat
using Reexport
import Base.Iterators: filter, flatten

# mechanism-related types
export
    RigidBody,
    Joint,
    GenericJoint,
    JointType,
    Mechanism,
    MechanismState,
    DynamicsResult

# specific joint types
export
    QuaternionFloating,
    Revolute,
    Prismatic,
    Fixed,
    Planar,
    QuaternionSpherical

# basic functionality related to mechanism structure
export
    has_defined_inertia,
    mass,
    default_frame,
    frame_before,
    frame_after,
    joint_type,
    add_frame!,
    root_frame,
    root_body,
    non_root_bodies,
    isroot,
    bodies,
    path,
    joints,
    tree_joints,
    successor,
    predecessor,
    in_joints,
    out_joints,
    joints_to_children,
    joint_to_parent,
    num_bodies,
    num_positions,
    num_velocities,
    num_additional_states,
    num_constraints,
    isfloating,
    configuration_range,
    velocity_range,
    has_fixed_subspaces,
    spatial_inertia!, # TODO: rename to set_spatial_inertia!
    add_body_fixed_frame!,
    fixed_transform,
    findbody,
    findjoint,
    body_fixed_frame_to_body,
    position_bounds,
    velocity_bounds,
    effort_bounds

# mechanism creation and modification
export
    rand_mechanism,
    rand_chain_mechanism,
    rand_tree_mechanism,
    rand_floating_tree_mechanism,
    attach!,
    remove_joint!,
    replace_joint!,
    maximal_coordinates,
    submechanism,
    remove_fixed_tree_joints!,
    parse_urdf

# contact-related functionality
export # note: contact-related functionality may be changed significantly in the future
    contact_points,
    add_contact_point!,
    add_environment_primitive!

# state-dependent functionality
export
    local_coordinates!,
    global_coordinates!,
    configuration_derivative,
    velocity_to_configuration_derivative!,
    configuration_derivative_to_velocity!,
    configuration_derivative_to_velocity_adjoint!,
    configuration,
    velocity,
    additional_state,
    joint_transform,
    motion_subspace,
    motion_subspace_in_world, # TODO: remove, just call it motion_subspace
    constraint_wrench_subspace,
    bias_acceleration,
    spatial_inertia,
    crb_inertia,
    twist_wrt_world,
    relative_twist,
    transform_to_root,
    relative_transform,
    setdirty!,
    state_vector, # TODO: Base.convert method to Vector?
    rand_configuration!,
    zero_configuration!,
    rand_velocity!,
    zero_velocity!,
    set_configuration!,
    set_velocity!,
    set_additional_state!,
    set!, # TODO: remove
    zero!, # TODO: remove
    center_of_mass,
    geometric_jacobian,
    geometric_jacobian!,
    relative_acceleration,
    kinetic_energy,
    gravitational_potential_energy,
    spatial_accelerations!,
    mass_matrix!,
    mass_matrix,
    momentum,
    momentum_matrix!,
    momentum_matrix,
    momentum_rate_bias,
    inverse_dynamics!,
    inverse_dynamics,
    dynamics_bias!,
    dynamics_bias,
    dynamics!,
    simulate

function name end # TODO

include("custom_collections.jl")
include("graphs.jl")
include("spatial/Spatial.jl")
include("contact.jl")
include("cache_element.jl")

using .Spatial # contains additional functions that are reexported
using .CustomCollections
using .Contact
using .Graphs

import .Spatial: rotation, translation, transform, center_of_mass, newton_euler, kinetic_energy

include("util.jl")
include("joint_types.jl")
include("joint.jl")
include("rigid_body.jl")
include("mechanism.jl")
include("mechanism_modification.jl")
include("mechanism_state.jl")
include("dynamics_result.jl")
include("mechanism_algorithms.jl")
include("parse_urdf.jl")
include("ode_integrators.jl")
include("simulate.jl")

end # module
