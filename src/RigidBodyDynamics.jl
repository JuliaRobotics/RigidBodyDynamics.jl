# __precompile__() TODO: enable

module RigidBodyDynamics

import Base: convert, zero, one, *, +, /, -, call, inv, get, findfirst, Random.rand, Random.rand!, hcat, show, showcompact, isapprox, dot, cross
using FixedSizeArrays
using Quaternions
using DataStructures

include("third_party_addendum.jl")
include("frames.jl")
include("spatial.jl")
include("rigid_body.jl")
include("joint.jl")
include("tree.jl")
include("mechanism.jl")
include("cache_element.jl")
include("mechanism_state_cache.jl")
include("mechanism_algorithms.jl")

export
    # types
    CartesianFrame3D,
    Transform3D,
    Point3D,
    FreeVector3D,
    SpatialInertia,
    RigidBody,
    Joint,
    JointType,
    QuaternionFloating,
    Revolute,
    Prismatic,
    Twist,
    GeometricJacobian,
    Wrench,
    Momentum,
    MomentumMatrix,
    SpatialAcceleration,
    Mechanism,
    MechanismState,
    # functions
    transform,
    to_array,
    newton_euler,
    joint_torque,
    root_body,
    root_frame,
    root_vertex,
    isroot,
    bodies,
    toposort,
    path,
    joints,
    velocity_to_configuration_derivative,
    configuration_derivative_to_velocity,
    num_positions,
    num_velocities,
    joint_transform,
    motion_subspace,
    has_fixed_motion_subspace,
    bias_acceleration,
    spatial_inertia,
    crb_inertia,
    setdirty!,
    attach!,
    rand_mechanism,
    rand_chain_mechanism,
    rand_tree_mechanism,
    configuration_vector,
    velocity_vector,
    rand_configuration,
    rand_configuration!,
    zero_configuration,
    zero_configuration!,
    rand_velocity!,
    zero_velocity!,
    set_configuration!,
    set_velocity!,
    zero!,
    add_frame!,
    twist_wrt_world,
    relative_twist,
    transform_to_parent,
    transform_to_root,
    relative_transform,
    mass,
    center_of_mass,
    geometric_jacobian,
    kinetic_energy,
    potential_energy,
    mass_matrix,
    momentum_matrix,
    inverse_dynamics
end
