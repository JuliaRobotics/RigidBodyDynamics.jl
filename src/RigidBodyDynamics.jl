# __precompile__() TODO: enable

module RigidBodyDynamics

import Base: convert, zero, one, *, +, /, -, call, inv, get, findfirst, Random.rand, Random.rand!
using FixedSizeArrays
using Quaternions
using DataStructures

include("frames.jl")
include("spatial_inertia.jl")
include("rigid_body.jl")
include("spatial_motion_force.jl")
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
    QuaternionFloating,
    Revolute,
    Prismatic,
    Twist,
    GeometricJacobian,
    Mechanism,
    MechanismState,
    MechanismStateCache,
    # functions
    transform,
    root,
    bodies,
    path,
    joints,
    num_positions,
    num_velocities,
    joint_transform,
    motion_subspace,
    attach!,
    configuration_vector,
    velocity_vector,
    zero_configuration!,
    zero_velocity!,
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
    mass_matrix
end
