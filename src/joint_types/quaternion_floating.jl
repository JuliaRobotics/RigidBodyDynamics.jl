"""
$(TYPEDEF)

A floating joint type that uses a unit quaternion representation for orientation.

Floating joints are 6-degree-of-freedom joints that are in a sense degenerate,
as they impose no constraints on the relative motion between two bodies.

The full, 7-dimensional configuration vector of a `QuaternionFloating` joint
type consists of a unit quaternion representing the orientation that rotates
vectors from the frame 'directly after' the joint to the frame 'directly before'
it, and a 3D position vector representing the origin of the frame after the
joint in the frame before the joint.

The 6-dimensional velocity vector of a `QuaternionFloating` joint is the twist
of the frame after the joint with respect to the frame before it, expressed in
the frame after the joint.
"""
struct QuaternionFloating{T} <: JointType{T} end

Base.show(io::IO, jt::QuaternionFloating) = print(io, "Quaternion floating joint")
Random.rand(::Type{QuaternionFloating{T}}) where {T} = QuaternionFloating{T}()

num_positions(::Type{<:QuaternionFloating}) = 7
num_velocities(::Type{<:QuaternionFloating}) = 6
has_fixed_subspaces(jt::QuaternionFloating) = true
isfloating(::Type{<:QuaternionFloating}) = true

@propagate_inbounds function rotation(jt::QuaternionFloating, q::AbstractVector, normalize::Bool = true)
    quat = Quat(q[1], q[2], q[3], q[4], normalize)
    quat
end

@propagate_inbounds function set_rotation!(q::AbstractVector, jt::QuaternionFloating, rot::Rotation{3})
    T = eltype(rot)
    quat = convert(Quat{T}, rot)
    q[1] = quat.w
    q[2] = quat.x
    q[3] = quat.y
    q[4] = quat.z
    nothing
end

@propagate_inbounds function set_rotation!(q::AbstractVector, jt::QuaternionFloating, rot::AbstractVector)
    q[1] = rot[1]
    q[2] = rot[2]
    q[3] = rot[3]
    q[4] = rot[4]
    nothing
end

@propagate_inbounds translation(jt::QuaternionFloating, q::AbstractVector) = SVector(q[5], q[6], q[7])
@propagate_inbounds set_translation!(q::AbstractVector, jt::QuaternionFloating, trans::AbstractVector) = copyto!(q, 5, trans, 1, 3)

@propagate_inbounds angular_velocity(jt::QuaternionFloating, v::AbstractVector) = SVector(v[1], v[2], v[3])
@propagate_inbounds set_angular_velocity!(v::AbstractVector, jt::QuaternionFloating, ω::AbstractVector) = copyto!(v, 1, ω, 1, 3)

@propagate_inbounds linear_velocity(jt::QuaternionFloating, v::AbstractVector) = SVector(v[4], v[5], v[6])
@propagate_inbounds set_linear_velocity!(v::AbstractVector, jt::QuaternionFloating, ν::AbstractVector) = copyto!(v, 4, ν, 1, 3)

@propagate_inbounds function set_configuration!(q::AbstractVector, joint::Joint{<:Any, <:QuaternionFloating}, config::Transform3D)
    @boundscheck check_num_positions(joint, q)
    @framecheck config.from frame_after(joint)
    @framecheck config.to frame_before(joint)
    set_rotation!(q, joint_type(joint), rotation(config))
    set_translation!(q, joint_type(joint), translation(config))
    q
end

@propagate_inbounds function set_velocity!(v::AbstractVector, joint::Joint{<:Any, <:QuaternionFloating}, twist::Twist)
    @boundscheck check_num_velocities(joint, v)
    @framecheck twist.base frame_before(joint)
    @framecheck twist.body frame_after(joint)
    @framecheck twist.frame frame_after(joint)
    set_angular_velocity!(v, joint_type(joint), angular(twist))
    set_linear_velocity!(v, joint_type(joint), linear(twist))
    v
end

@propagate_inbounds function joint_transform(jt::QuaternionFloating, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D, q::AbstractVector)
    Transform3D(frame_after, frame_before, rotation(jt, q, false), translation(jt, q))
end

@inline function motion_subspace(jt::QuaternionFloating, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector)
    S = promote_eltype(jt, q)
    angular = hcat(one(SMatrix{3, 3, S}), zero(SMatrix{3, 3, S}))
    linear = hcat(zero(SMatrix{3, 3, S}), one(SMatrix{3, 3, S}))
    GeometricJacobian(frame_after, frame_before, frame_after, angular, linear)
end

@inline function constraint_wrench_subspace(jt::QuaternionFloating, joint_transform::Transform3D)
    S = promote_eltype(jt, joint_transform)
    WrenchMatrix(joint_transform.from, zero(SMatrix{3, 0, S}), zero(SMatrix{3, 0, S}))
end

@inline function bias_acceleration(jt::QuaternionFloating, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector)
    S = promote_eltype(q, v)
    zero(SpatialAcceleration{S}, frame_after, frame_before, frame_after)
end

@propagate_inbounds function configuration_derivative_to_velocity!(v::AbstractVector, jt::QuaternionFloating,
        q::AbstractVector, q̇::AbstractVector)
    quat = rotation(jt, q, false)
    quatdot = SVector(q̇[1], q̇[2], q̇[3], q̇[4])
    ω = angular_velocity_in_body(quat, quatdot)
    posdot = translation(jt, q̇)
    linear = inv(quat) * posdot
    set_angular_velocity!(v, jt, ω)
    set_linear_velocity!(v, jt, linear)
    nothing
end

@propagate_inbounds function configuration_derivative_to_velocity_adjoint!(fq, jt::QuaternionFloating, q::AbstractVector, fv)
    quatnorm = sqrt(q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2) # TODO: make this nicer
    quat = Quat(q[1] / quatnorm, q[2] / quatnorm, q[3] / quatnorm, q[4] / quatnorm, false)
    rot = (velocity_jacobian(angular_velocity_in_body, quat)' * angular_velocity(jt, fv)) ./ quatnorm
    trans = quat * linear_velocity(jt, fv)
    set_rotation!(fq, jt, rot)
    set_translation!(fq, jt, trans)
    nothing
end

@propagate_inbounds function velocity_to_configuration_derivative!(q̇::AbstractVector, jt::QuaternionFloating,
        q::AbstractVector, v::AbstractVector)
    quat = rotation(jt, q, false)
    ω = angular_velocity(jt, v)
    linear = linear_velocity(jt, v)
    quatdot = quaternion_derivative(quat, ω)
    transdot = quat * linear
    set_rotation!(q̇, jt, quatdot)
    set_translation!(q̇, jt, transdot)
    nothing
end

@propagate_inbounds function velocity_to_configuration_derivative_jacobian(jt::QuaternionFloating, q::AbstractVector)
    quat = rotation(jt, q, false)
    vj = velocity_jacobian(quaternion_derivative, quat)
    R = RotMatrix(quat)
    # TODO: use hvcat once it's as fast
    @SMatrix(
        [vj[1] vj[5] vj[9]  0    0    0;
         vj[2] vj[6] vj[10] 0    0    0;
         vj[3] vj[7] vj[11] 0    0    0;
         vj[4] vj[8] vj[12] 0    0    0;
         0     0     0      R[1] R[4] R[7];
         0     0     0      R[2] R[5] R[8];
         0     0     0      R[3] R[6] R[9]])
end

@propagate_inbounds function configuration_derivative_to_velocity_jacobian(jt::QuaternionFloating, q::AbstractVector)
    quat = rotation(jt, q, false)
    vj = velocity_jacobian(angular_velocity_in_body, quat)
    R_inv = RotMatrix(inv(quat))
    # TODO: use hvcat once it's as fast
    @SMatrix(
        [vj[1] vj[4] vj[7] vj[10] 0        0        0;
         vj[2] vj[5] vj[8] vj[11] 0        0        0;
         vj[3] vj[6] vj[9] vj[12] 0        0        0;
         0     0     0     0      R_inv[1] R_inv[4] R_inv[7];
         0     0     0     0      R_inv[2] R_inv[5] R_inv[8];
         0     0     0     0      R_inv[3] R_inv[6] R_inv[9]])
end


@propagate_inbounds function zero_configuration!(q::AbstractVector, jt::QuaternionFloating)
    T = eltype(q)
    set_rotation!(q, jt, one(Quat{T}))
    set_translation!(q, jt, zero(SVector{3, T}))
    nothing
end

@propagate_inbounds function rand_configuration!(q::AbstractVector, jt::QuaternionFloating)
    T = eltype(q)
    set_rotation!(q, jt, rand(Quat{T}))
    set_translation!(q, jt, rand(SVector{3, T}) - 0.5)
    nothing
end

@propagate_inbounds function joint_twist(jt::QuaternionFloating, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector)
    S = promote_eltype(jt, q, v)
    angular = convert(SVector{3, S}, angular_velocity(jt, v))
    linear = convert(SVector{3, S}, linear_velocity(jt, v))
    Twist(frame_after, frame_before, frame_after, angular, linear)
end

@propagate_inbounds function joint_spatial_acceleration(jt::QuaternionFloating, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector, vd::AbstractVector)
    S = promote_eltype(jt, q, v, vd)
    angular = convert(SVector{3, S}, angular_velocity(jt, vd))
    linear = convert(SVector{3, S}, linear_velocity(jt, vd))
    SpatialAcceleration(frame_after, frame_before, frame_after, angular, linear)
end

@propagate_inbounds function joint_torque!(τ::AbstractVector, jt::QuaternionFloating, q::AbstractVector, joint_wrench::Wrench)
    set_angular_velocity!(τ, jt, angular(joint_wrench))
    set_linear_velocity!(τ, jt, linear(joint_wrench))
    nothing
end

# uses exponential coordinates centered around q0
@propagate_inbounds function local_coordinates!(ϕ::AbstractVector, ϕ̇::AbstractVector,
        jt::QuaternionFloating, q0::AbstractVector, q::AbstractVector, v::AbstractVector)
    # anonymous helper frames # FIXME
    frame_before = CartesianFrame3D()
    frame0 = CartesianFrame3D()
    frame_after = CartesianFrame3D()

    quat0 = rotation(jt, q0, false)
    quat = rotation(jt, q, false)
    p0 = translation(jt, q0)
    p = translation(jt, q)

    quat0inv = inv(quat0)
    δquat = quat0inv * quat
    δp = quat0inv * (p - p0)
    relative_transform = Transform3D(frame_after, frame0, δquat, δp)

    twist = joint_twist(jt, frame_after, frame0, q, v) # (q_0 is assumed not to change)
    ξ, ξ̇ = log_with_time_derivative(relative_transform, twist)

    copyto!(ϕ, 1, angular(ξ), 1, 3)
    copyto!(ϕ, 4, linear(ξ), 1, 3)

    copyto!(ϕ̇, 1, angular(ξ̇), 1, 3)
    copyto!(ϕ̇, 4, linear(ξ̇), 1, 3)

    nothing
end

@propagate_inbounds function global_coordinates!(q::AbstractVector, jt::QuaternionFloating, q0::AbstractVector, ϕ::AbstractVector)
    # anonymous helper frames #FIXME
    frame_before = CartesianFrame3D()
    frame0 = CartesianFrame3D()
    frame_after = CartesianFrame3D()

    t0 = joint_transform(jt, frame0, frame_before, q0)
    ξrot = SVector(ϕ[1], ϕ[2], ϕ[3])
    ξtrans = SVector(ϕ[4], ϕ[5], ϕ[6])
    ξ = Twist(frame_after, frame0, frame0, ξrot, ξtrans)
    relative_transform = exp(ξ)
    t = t0 * relative_transform
    set_rotation!(q, jt, rotation(t))
    set_translation!(q, jt, translation(t))
    nothing
end

@propagate_inbounds normalize_configuration!(q::AbstractVector, jt::QuaternionFloating) = set_rotation!(q, jt, rotation(jt, q, true))

@propagate_inbounds function is_configuration_normalized(jt::QuaternionFloating, q::AbstractVector, rtol, atol)
    isapprox(quatnorm(rotation(jt, q, false)), one(eltype(q)); rtol = rtol, atol = atol)
end

@propagate_inbounds function principal_value!(q::AbstractVector, jt::QuaternionFloating)
    quat = rotation(jt, q, false)
    set_rotation!(q, jt, principal_value(quat))
    nothing
end
