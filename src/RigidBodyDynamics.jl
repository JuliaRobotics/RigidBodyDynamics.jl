# __precompile__() TODO: enable

module RigidBodyDynamics

import Base: convert, zero, one, *, +, /, -, call, inv, get, findfirst, Random.rand, Random.rand!, hcat, show, showcompact, isapprox, dot, cross
using FixedSizeArrays
using Quaternions
using DataStructures
using LightXML

include("third_party_addendum.jl")
include("util.jl")
include("frames.jl")
include("spatial.jl")
include("rigid_body.jl")
include("joint.jl")
include("tree.jl")
include("mechanism.jl")
include("cache_element.jl")
include("mechanism_state.jl")
include("mechanism_algorithms.jl")
include("parse_urdf.jl")

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
    Fixed,
    Twist,
    GeometricJacobian,
    Wrench,
    Momentum,
    MomentumMatrix,
    SpatialAcceleration,
    Mechanism,
    MechanismState,
    # functions
    name,
    transform,
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
    num_cols,
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
    velocity_dict_to_vector,
    torque_dict_to_vector,
    configuration_dict_to_vector,
    velocity_vector_to_dict,
    torque_vector_to_dict,
    configuration_vector_to_dict,
    configuration_vector,
    velocity_vector,
    state_vector,
    rand_configuration,
    rand_configuration!,
    zero_configuration,
    zero_configuration!,
    rand_velocity!,
    zero_velocity!,
    set_configuration!,
    set_velocity!,
    set!,
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
    relative_acceleration,
    kinetic_energy,
    potential_energy,
    mass_matrix,
    momentum_matrix,
    inverse_dynamics,
    dynamics,
    parse_urdf
end
