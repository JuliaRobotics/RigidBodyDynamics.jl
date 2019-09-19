"""
$(TYPEDEF)

A `SinCosRevolute` joint type allows rotation about a fixed axis.

In contrast to the [`Revolute`](@ref) joint type, the configuration vector for the `SinCosRevolute` joint type
consists of the sine and cosine of the angle of rotation about the specified axis (in that order).
The velocity vector for the `SinCosRevolute` joint type is the same as for
the `Revolute` joint type, i.e., the time derivative of the angle about the axis.
"""
struct SinCosRevolute{T} <: JointType{T}
    axis::SVector{3, T}
    rotation_from_z_aligned::RotMatrix3{T}

    function SinCosRevolute{T}(axis::AbstractVector) where {T}
        a = normalize(axis)
        new{T}(a, rotation_between(SVector(zero(T), zero(T), one(T)), SVector{3, T}(a)))
    end
end

"""
$(SIGNATURES)

Construct a new `SinCosRevolute` joint type, allowing rotation about `axis`
(expressed in the frame before the joint).
"""
SinCosRevolute(axis::AbstractVector{T}) where {T} = SinCosRevolute{T}(axis)

Base.show(io::IO, jt::SinCosRevolute) = print(io, "SinCosRevolute joint with axis $(jt.axis)")

function Random.rand(::Type{SinCosRevolute{T}}) where {T}
    axis = normalize(randn(SVector{3, T}))
    SinCosRevolute(axis)
end

RigidBodyDynamics.flip_direction(jt::SinCosRevolute) = SinCosRevolute(-jt.axis)

num_positions(::Type{<:SinCosRevolute}) = 2
num_velocities(::Type{<:SinCosRevolute}) = 1
has_fixed_subspaces(jt::SinCosRevolute) = true
isfloating(::Type{<:SinCosRevolute}) = false

@propagate_inbounds function set_configuration!(q::AbstractVector, joint::Joint{<:Any, <:SinCosRevolute}, θ::Number)
    @boundscheck check_num_positions(joint, q)
    s, c = sincos(θ)
    @inbounds q[1] = s
    @inbounds q[2] = c
    q
end

@propagate_inbounds function set_velocity!(v::AbstractVector, joint::Joint{<:Any, <:SinCosRevolute}, θ̇::Number)
    @boundscheck check_num_velocities(joint, v)
    @inbounds v[1] = θ̇
    v
end

@propagate_inbounds function rand_configuration!(q::AbstractVector, jt::SinCosRevolute)
    q .= normalize(@SVector(randn(2)))
    nothing
end

@propagate_inbounds function zero_configuration!(q::AbstractVector, jt::SinCosRevolute)
    T = eltype(q)
    q[1] = zero(T)
    q[2] = one(T)
    nothing
end

@propagate_inbounds function joint_transform(jt::SinCosRevolute, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D, q::AbstractVector)
    # from https://github.com/FugroRoames/Rotations.jl/blob/8d77152a76950665302b5a730b420973bb397f41/src/angleaxis_types.jl#L50
    T = eltype(q)

    axis = jt.axis
    s, c = q[1], q[2]
    c1 = one(T) - c

    c1x2 = c1 * axis[1]^2
    c1y2 = c1 * axis[2]^2
    c1z2 = c1 * axis[3]^2

    c1xy = c1 * axis[1] * axis[2]
    c1xz = c1 * axis[1] * axis[3]
    c1yz = c1 * axis[2] * axis[3]

    sx = s * axis[1]
    sy = s * axis[2]
    sz = s * axis[3]

    # Note that the RotMatrix constructor argument order makes this look transposed:
    rot = RotMatrix(
        one(T) - c1y2 - c1z2, c1xy + sz, c1xz - sy,
        c1xy - sz, one(T) - c1x2 - c1z2, c1yz + sx,
        c1xz + sy, c1yz - sx, one(T) - c1x2 - c1y2)

    Transform3D(frame_after, frame_before, rot)
end

@propagate_inbounds function joint_twist(jt::SinCosRevolute, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector)
    angular = jt.axis * v[1]
    Twist(frame_after, frame_before, frame_after, angular, zero(angular))
end

@inline function bias_acceleration(jt::SinCosRevolute, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector)
    S = promote_eltype(jt, q, v)
    zero(SpatialAcceleration{S}, frame_after, frame_before, frame_after)
end

@propagate_inbounds function joint_spatial_acceleration(jt::SinCosRevolute, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector, vd::AbstractVector)
    S = promote_eltype(jt, q, v, vd)
    angular = convert(SVector{3, S}, jt.axis * vd[1])
    SpatialAcceleration(frame_after, frame_before, frame_after, angular, zero(angular))
end

@inline function motion_subspace(jt::SinCosRevolute, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector)
    S = promote_eltype(jt, q)
    angular = SMatrix{3, 1, S}(jt.axis)
    linear = zero(SMatrix{3, 1, S})
    GeometricJacobian(frame_after, frame_before, frame_after, angular, linear)
end

@inline function constraint_wrench_subspace(jt::SinCosRevolute, joint_transform::Transform3D)
    S = promote_eltype(jt, joint_transform)
    R = convert(RotMatrix3{S}, jt.rotation_from_z_aligned)
    Rcols12 = R[:, SVector(1, 2)]
    angular = hcat(Rcols12, zero(SMatrix{3, 3, S}))
    linear = hcat(zero(SMatrix{3, 2, S}), R)
    WrenchMatrix(joint_transform.from, angular, linear)
end

@propagate_inbounds function joint_torque!(τ::AbstractVector, jt::SinCosRevolute, q::AbstractVector, joint_wrench::Wrench)
    τ[1] = dot(angular(joint_wrench), jt.axis)
    nothing
end

@propagate_inbounds function configuration_derivative_to_velocity!(v::AbstractVector, jt::SinCosRevolute, q::AbstractVector, q̇::AbstractVector)
    # q̇ = [c; -s] * v
    # [c -s] * [c; -s] = c^2 + s^2 = 1
    # v = [c -s] * q̇
    s, c = q[1], q[2]
    v[1] = c * q̇[1] - s * q̇[2]
    nothing
end

@propagate_inbounds function configuration_derivative_to_velocity_jacobian(jt::SinCosRevolute, q::AbstractVector)
    s, c = q[1], q[2]
    @SMatrix [c -s]
end

@propagate_inbounds function configuration_derivative_to_velocity_adjoint!(fq, jt::SinCosRevolute, q::AbstractVector, fv)
    qnorm = sqrt(q[1]^2 + q[2]^2)
    qnormalized = @SVector([q[1], q[2]]) / qnorm
    fq .= (configuration_derivative_to_velocity_jacobian(jt, qnormalized)' * fv) ./ qnorm
    nothing
end

@propagate_inbounds function velocity_to_configuration_derivative!(q̇::AbstractVector, jt::SinCosRevolute, q::AbstractVector, v::AbstractVector)
    s, c = q[1], q[2]
    q̇[1] = c * v[1]
    q̇[2] = -s * v[1]
    nothing
end

@propagate_inbounds function velocity_to_configuration_derivative_jacobian(jt::SinCosRevolute, q::AbstractVector)
    s, c = q[1], q[2]
    @SMatrix [c; -s]
end

# uses exponential coordinates centered around q0 (i.e, the relative joint angle)
@propagate_inbounds function local_coordinates!(ϕ::AbstractVector, ϕd::AbstractVector,
        jt::SinCosRevolute, q0::AbstractVector, q::AbstractVector, v::AbstractVector)
    # RΔ = R₀ᵀ * R
    s0, c0 = q0[1], q0[2]
    s, c = q[1], q[2]
    sΔ = c0 * s - s0 * c
    cΔ = c0 * c + s0 * s
    θ = atan(sΔ, cΔ)
    ϕ[1] = θ
    ϕd[1] = v[1]
    nothing
end

@propagate_inbounds function global_coordinates!(q::AbstractVector, jt::SinCosRevolute, q0::AbstractVector, ϕ::AbstractVector)
    # R = R₀ * RΔ:
    s0, c0 = q0[1], q0[2]
    θ = ϕ[1]
    sΔ, cΔ = sincos(θ)
    s = s0 * cΔ + c0 * sΔ
    c = c0 * cΔ - s0 * sΔ
    q[1] = s
    q[2] = c
    nothing
end

@propagate_inbounds function normalize_configuration!(q::AbstractVector, jt::SinCosRevolute)
    q ./= sqrt(q[1]^2 + q[2]^2)
    q
end

@propagate_inbounds function is_configuration_normalized(jt::SinCosRevolute, q::AbstractVector, rtol, atol)
    # TODO: base off of norm squared
    qnorm = sqrt(q[1]^2 + q[2]^2)
    isapprox(qnorm, one(eltype(q)); rtol = rtol, atol = atol)
end
