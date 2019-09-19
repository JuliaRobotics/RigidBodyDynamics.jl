module Spatial

# types
export
    CartesianFrame3D,
    Transform3D,
    FreeVector3D,
    Point3D,
    GeometricJacobian,
    PointJacobian,
    Twist,
    SpatialAcceleration,
    MomentumMatrix, # TODO: consider combining with WrenchMatrix
    WrenchMatrix, # TODO: consider combining with MomentumMatrix
    Momentum,
    Wrench,
    SpatialInertia

# functions
export
    transform,
    rotation,
    translation,
    angular,
    linear,
    point_velocity,
    point_acceleration,
    change_base,
    log_with_time_derivative,
    center_of_mass,
    newton_euler,
    torque,
    torque!,
    kinetic_energy,
    rotation_vector_rate,
    quaternion_derivative,
    spquat_derivative,
    angular_velocity_in_body,
    velocity_jacobian,
    linearized_rodrigues_vec

# macros
export
    @framecheck

using LinearAlgebra
using Random
using StaticArrays
using Rotations
using DocStringExtensions

using Base: promote_eltype

include("frame.jl")
include("util.jl")
include("transform3d.jl")
include("threevectors.jl")
include("spatialmotion.jl")
include("spatialforce.jl")
include("motion_force_interaction.jl")
include("common.jl")

end # module
