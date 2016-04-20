# __precompile__() TODO: enable

module RigidBodyDynamics

import Base: convert, one, *, +, /, -, call, inv, get, findfirst, Random.rand
using FixedSizeArrays
using Quaternions
using DataStructures

include("frames.jl")
include("spatial_inertia.jl")
include("rigid_body.jl")
include("spatial_motion_force.jl")
include("joint.jl")
include("tree.jl")
include("cache_element.jl")
include("frame_cache.jl")
include("mechanism.jl")

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
    Mechanism,
    MechanismState,
    FrameCache,
    # functions
    transform,
    root,
    bodies,
    joints,
    num_positions,
    num_velocities,
    joint_transform,
    attach!,
    configuration_vector,
    velocity_vector,
    zero_configuration!,
    add_frame!,
    transform_to_parent,
    transform_to_root,
    relative_transform,
    mass,
    center_of_mass

end
