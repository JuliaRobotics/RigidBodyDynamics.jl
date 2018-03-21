@reexport module Spatial

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
    rotation_vector_rate,
    quaternion_derivative,
    angular_velocity_in_body,
    velocity_jacobian,
    linearized_rodrigues_vec

# macros
export
    @framecheck

using Compat
using Compat.LinearAlgebra
using Compat.Random
using StaticArrays
using Rotations
using DocStringExtensions

include("frame.jl")
include("util.jl")
include("transform3d.jl")
include("threevectors.jl")
include("spatialmotion.jl")
include("spatialforce.jl")
include("motion_force_interaction.jl")

end # module
