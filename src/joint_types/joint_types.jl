# TODO: document which methods are needed for a new JointType.
# Default implementations
function flip_direction(jt::JointType{T}) where {T}
    error("Flipping direction is not supported for $(typeof(jt))")
end

@propagate_inbounds zero_configuration!(q::AbstractVector, ::JointType) = (q .= 0; nothing)

@propagate_inbounds function local_coordinates!(ϕ::AbstractVector, ϕ̇::AbstractVector,
        jt::JointType, q0::AbstractVector, q::AbstractVector, v::AbstractVector)
    ϕ .= q .- q0
    velocity_to_configuration_derivative!(ϕ̇, jt, q, v)
    nothing
end

@propagate_inbounds function global_coordinates!(q::AbstractVector, jt::JointType, q0::AbstractVector, ϕ::AbstractVector)
    q .= q0 .+ ϕ
end

@propagate_inbounds function configuration_derivative_to_velocity_adjoint!(out, jt::JointType, q::AbstractVector, f)
    out .= f
end

@propagate_inbounds function configuration_derivative_to_velocity!(v::AbstractVector, ::JointType, q::AbstractVector, q̇::AbstractVector)
    v .= q̇
    nothing
end

@propagate_inbounds function velocity_to_configuration_derivative!(q̇::AbstractVector, ::JointType, q::AbstractVector, v::AbstractVector)
    q̇ .= v
    nothing
end

normalize_configuration!(q::AbstractVector, ::JointType) = nothing
is_configuration_normalized(::JointType, q::AbstractVector, rtol, atol) = true
principal_value!(q::AbstractVector, ::JointType) = nothing

include("quaternion_floating.jl")
include("spquat_floating.jl")
include("prismatic.jl")
include("revolute.jl")
include("fixed.jl")
include("planar.jl")
include("quaternion_spherical.jl")
include("sin_cos_revolute.jl")
