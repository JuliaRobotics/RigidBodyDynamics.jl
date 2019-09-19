module RigidBodyDynamics

using Random
using LinearAlgebra
using StaticArrays
using Rotations
using TypeSortedCollections
using DocStringExtensions
using Reexport

using Base.Iterators: filter, flatten
using Base: @propagate_inbounds, promote_eltype

# mechanism-related types
export
    RigidBody,
    Joint,
    JointType,
    Mechanism,
    MechanismState,
    DynamicsResult,
    StateCache,
    DynamicsResultCache,
    SegmentedVectorCache

# specific joint types
export
    QuaternionFloating,
    SPQuatFloating,
    Revolute,
    Prismatic,
    Fixed,
    Planar,
    QuaternionSpherical,
    SinCosRevolute

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
    non_tree_joints,
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
    rand_chain_mechanism,
    rand_tree_mechanism,
    rand_floating_tree_mechanism,
    attach!,
    remove_joint!,
    replace_joint!,
    maximal_coordinates,
    submechanism,
    remove_fixed_tree_joints!

# contact-related functionality
export # note: contact-related functionality may be changed significantly in the future
    contact_points,
    add_contact_point!,
    add_environment_primitive!

# state-dependent functionality
export
    local_coordinates!,
    global_coordinates!,
    configuration_derivative!,
    configuration_derivative,
    velocity_to_configuration_derivative!, # TODO: consider merging with configuration_derivative!
    configuration_derivative_to_velocity!,
    configuration_derivative_to_velocity_adjoint!,
    configuration,
    velocity,
    additional_state,
    joint_transform,
    motion_subspace,
    constraint_wrench_subspace,
    bias_acceleration,
    spatial_inertia,
    crb_inertia,
    twist_wrt_world,
    relative_twist,
    transform_to_root,
    relative_transform,
    setdirty!,
    rand_configuration!,
    zero_configuration!,
    rand_velocity!,
    zero_velocity!,
    set_configuration!,
    set_velocity!,
    set_additional_state!,
    zero!, # TODO: remove
    normalize_configuration!,
    principal_value!,
    center_of_mass,
    geometric_jacobian,
    geometric_jacobian!,
    point_velocity,
    point_acceleration,
    point_jacobian,
    point_jacobian!,
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

# Utility
export
    SegmentedVector,
    JointDict,
    BodyDict,
    JointID,
    BodyID,
    segments

include(joinpath("custom_collections", "custom_collections.jl"))
include(joinpath("graphs", "Graphs.jl"))
include(joinpath("spatial", "Spatial.jl"))
include("contact.jl")
include("pdcontrol.jl")

@reexport using .Spatial
using .CustomCollections
using .Contact
using .Graphs
using .PDControl

import .Spatial: rotation, translation, transform, center_of_mass, newton_euler, kinetic_energy

include("util.jl")
include("joint.jl")
include(joinpath("joint_types", "joint_types.jl"))
include("rigid_body.jl")
include("mechanism.jl")
include("mechanism_modification.jl")
include("mechanism_state.jl")
include("dynamics_result.jl")
include("caches.jl")
include("mechanism_algorithms.jl")
include("ode_integrators.jl")
include("simulate.jl")

include(joinpath("urdf", "URDF.jl"))
@reexport using .URDF

# import these for MechanismGeometries compatibility. TODO: stop importing these after updating MechanismGeometries.
import .URDF: parse_scalar, parse_vector, parse_pose

end # module
