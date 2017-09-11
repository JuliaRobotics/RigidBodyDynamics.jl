@reexport module Spatial

using Base.Random
using StaticArrays
using Rotations
using DocStringExtensions

# types
export
    CartesianFrame3D,
    Transform3D,
    FreeVector3D,
    Point3D,
    GeometricJacobian,
    Twist,
    SpatialAcceleration,
    MomentumMatrix, # TODO: consider combining with WrenchMatrix
    WrenchMatrix, # TODO: consider combining with MomentumMatrix
    Momentum,
    Wrench,
    SpatialInertia

# aliases
export
    MotionSubspace, # TODO: remove
    WrenchSubspace # TODO: remove

# functions
export
    transform,
    rotation,
    translation,
    angular,
    linear,
    point_velocity,
    change_base,
    log_with_time_derivative,
    center_of_mass,
    newton_euler,
    torque,
    torque!,
    kinetic_energy,
    angle_difference,
    rotation_vector_rate,
    quaternion_derivative,
    angular_velocity_in_body,
    body_angular_velocity_to_quat_derivative_jacobian,
    quat_derivative_to_body_angular_velocity_jacobian

# macros
export
    @framecheck

include("frame.jl")
include("util.jl")
include("transform3d.jl")
include("threevectors.jl")
include("spatialmotion.jl")
include("spatialforce.jl")
include("motion_force_interaction.jl")

end # module
