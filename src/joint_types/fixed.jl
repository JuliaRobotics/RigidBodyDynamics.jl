"""
$(TYPEDEF)

The `Fixed` joint type is a degenerate joint type, in the sense that it allows
no motion between its predecessor and successor rigid bodies.
"""
struct Fixed{T} <: JointType{T} end

Base.show(io::IO, jt::Fixed) = print(io, "Fixed joint")
Random.rand(::Type{Fixed{T}}) where {T} = Fixed{T}()
RigidBodyDynamics.flip_direction(jt::Fixed) = deepcopy(jt)

num_positions(::Type{<:Fixed}) = 0
num_velocities(::Type{<:Fixed}) = 0
has_fixed_subspaces(jt::Fixed) = true
isfloating(::Type{<:Fixed}) = false

@inline function joint_transform(jt::Fixed{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    one(Transform3D{S}, frame_after, frame_before)
end

@inline function joint_twist(jt::Fixed{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    zero(Twist{S}, frame_after, frame_before, frame_after)
end

@inline function joint_spatial_acceleration(jt::Fixed{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}, vd::AbstractVector{XD}) where {T, X, XD}
    S = promote_type(T, X, XD)
    zero(SpatialAcceleration{S}, frame_after, frame_before, frame_after)
end

@inline function motion_subspace(jt::Fixed{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    GeometricJacobian(frame_after, frame_before, frame_after, zero(SMatrix{3, 0, S}), zero(SMatrix{3, 0, S}))
end

@inline function constraint_wrench_subspace(jt::Fixed{T}, joint_transform::Transform3D{X}) where {T, X}
    S = promote_type(T, X)
    angular = hcat(one(SMatrix{3, 3, S}), zero(SMatrix{3, 3, S}))
    linear = hcat(zero(SMatrix{3, 3, S}), one(SMatrix{3, 3, S}))
    WrenchMatrix(joint_transform.from, angular, linear)
end

@inline zero_configuration!(q::AbstractVector, ::Fixed) = nothing
@inline rand_configuration!(q::AbstractVector, ::Fixed) = nothing

@inline function bias_acceleration(jt::Fixed{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    zero(SpatialAcceleration{S}, frame_after, frame_before, frame_after)
end

@inline configuration_derivative_to_velocity!(v::AbstractVector, ::Fixed, q::AbstractVector, q̇::AbstractVector) = nothing
@inline velocity_to_configuration_derivative!(q̇::AbstractVector, ::Fixed, q::AbstractVector, v::AbstractVector) = nothing
@inline joint_torque!(τ::AbstractVector, jt::Fixed, q::AbstractVector, joint_wrench::Wrench) = nothing

@inline function velocity_to_configuration_derivative_jacobian(::Fixed{T}, ::AbstractVector) where T
    SMatrix{0, 0, T}()
end

@inline function configuration_derivative_to_velocity_jacobian(::Fixed{T}, ::AbstractVector) where T
    SMatrix{0, 0, T}()
end
