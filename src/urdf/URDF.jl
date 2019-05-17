module URDF

using RigidBodyDynamics
using LightXML
using StaticArrays
using Rotations
using DocStringExtensions
using RigidBodyDynamics.Graphs

using RigidBodyDynamics: Bounds, upper, lower
using RigidBodyDynamics: has_loops, joint_to_predecessor
using RigidBodyDynamics: DEFAULT_GRAVITATIONAL_ACCELERATION
using LinearAlgebra: ×

export
    default_urdf_joint_types,
    parse_urdf,
    write_urdf

include("parse.jl")
include("write.jl")

end # module
