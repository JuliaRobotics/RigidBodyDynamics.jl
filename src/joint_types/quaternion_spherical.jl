"""
$(TYPEDEF)

The `QuaternionSpherical` joint type allows rotation in any direction. It is an
implementation of a ball-and-socket joint.

The 4-dimensional configuration vector ``q`` associated with a `QuaternionSpherical` joint
is the unit quaternion that describes the orientation of the frame after the joint
with respect to the frame before the joint. In other words, it is the quaternion that
can be used to rotate vectors from the frame after the joint to the frame before the
joint.

The 3-dimensional velocity vector ``v`` associated with a `QuaternionSpherical` joint is
the angular velocity of the frame after the joint with respect to the frame before
the joint, expressed in the frame after the joint (body frame).
"""
struct QuaternionSpherical{T} <: JointType{T} end

Base.show(io::IO, jt::QuaternionSpherical) = print(io, "Quaternion spherical joint")
Random.rand(::Type{QuaternionSpherical{T}}) where {T} = QuaternionSpherical{T}()
num_positions(::Type{<:QuaternionSpherical}) = 4
num_velocities(::Type{<:QuaternionSpherical}) = 3
has_fixed_subspaces(jt::QuaternionSpherical) = true
isfloating(::Type{<:QuaternionSpherical}) = false

@propagate_inbounds function rotation(jt::QuaternionSpherical, q::AbstractVector, normalize::Bool = true)
    Quat(q[1], q[2], q[3], q[4], normalize)
end

@propagate_inbounds function set_rotation!(q::AbstractVector, jt::QuaternionSpherical, rot::Rotation{3, T}) where {T}
    quat = convert(Quat{T}, rot)
    q[1] = quat.w
    q[2] = quat.x
    q[3] = quat.y
    q[4] = quat.z
    nothing
end

@propagate_inbounds function set_configuration!(q::AbstractVector, joint::Joint{<:Any, <:QuaternionSpherical}, rot::Rotation{3})
    @boundscheck check_num_positions(joint, q)
    @inbounds set_rotation!(q, joint_type(joint), rot)
    q
end

@propagate_inbounds function joint_transform(jt::QuaternionSpherical, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D, q::AbstractVector)
    quat = rotation(jt, q, false)
    Transform3D(frame_after, frame_before, quat)
end

@inline function motion_subspace(jt::QuaternionSpherical{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    angular = one(SMatrix{3, 3, S})
    linear = zero(SMatrix{3, 3, S})
    GeometricJacobian(frame_after, frame_before, frame_after, angular, linear)
end

@inline function constraint_wrench_subspace(jt::QuaternionSpherical{T}, joint_transform::Transform3D{X}) where {T, X}
    S = promote_type(T, X)
    angular = zero(SMatrix{3, 3, S})
    linear = one(SMatrix{3, 3, S})
    WrenchMatrix(joint_transform.from, angular, linear)
end

@inline function bias_acceleration(jt::QuaternionSpherical{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    zero(SpatialAcceleration{S}, frame_after, frame_before, frame_after)
end

@propagate_inbounds function configuration_derivative_to_velocity!(v::AbstractVector, jt::QuaternionSpherical, q::AbstractVector, q̇::AbstractVector)
    quat = rotation(jt, q, false)
    quatdot = SVector(q̇[1], q̇[2], q̇[3], q̇[4])
    v .= angular_velocity_in_body(quat, quatdot)
    nothing
end

@propagate_inbounds function configuration_derivative_to_velocity_adjoint!(fq, jt::QuaternionSpherical, q::AbstractVector, fv)
    quatnorm = sqrt(q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2) # TODO: make this nicer
    quat = Quat(q[1] / quatnorm, q[2] / quatnorm, q[3] / quatnorm, q[4] / quatnorm, false)
    fq .= (velocity_jacobian(angular_velocity_in_body, quat)' * fv) ./ quatnorm
    nothing
end

@propagate_inbounds function velocity_to_configuration_derivative!(q̇::AbstractVector, jt::QuaternionSpherical, q::AbstractVector, v::AbstractVector)
    quat = rotation(jt, q, false)
    q̇ .= quaternion_derivative(quat, v)
    nothing
end

@propagate_inbounds function velocity_to_configuration_derivative_jacobian(jt::QuaternionSpherical, q::AbstractVector)
    quat = rotation(jt, q, false)
    velocity_jacobian(quaternion_derivative, quat)
end

@propagate_inbounds function configuration_derivative_to_velocity_jacobian(jt::QuaternionSpherical, q::AbstractVector)
    quat = rotation(jt, q, false)
    velocity_jacobian(angular_velocity_in_body, quat)
end

@propagate_inbounds function zero_configuration!(q::AbstractVector, jt::QuaternionSpherical)
    T = eltype(q)
    set_rotation!(q, jt, one(Quat{T}))
    nothing
end

@propagate_inbounds function rand_configuration!(q::AbstractVector, jt::QuaternionSpherical)
    T = eltype(q)
    set_rotation!(q, jt, rand(Quat{T}))
    nothing
end

@propagate_inbounds function joint_twist(jt::QuaternionSpherical{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}) where {T, X}
    S = promote_type(T, X)
    angular = SVector{3, S}(v)
    linear = zero(SVector{3, S})
    Twist(frame_after, frame_before, frame_after, angular, linear)
end

@propagate_inbounds function joint_spatial_acceleration(jt::QuaternionSpherical{T}, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector{X}, v::AbstractVector{X}, vd::AbstractVector{XD}) where {T, X, XD}
    S = promote_type(T, X, XD)
    angular = SVector{3, S}(vd)
    linear = zero(SVector{3, S})
    SpatialAcceleration(frame_after, frame_before, frame_after, angular, linear)
end

@propagate_inbounds function joint_torque!(τ::AbstractVector, jt::QuaternionSpherical, q::AbstractVector, joint_wrench::Wrench)
    τ .= angular(joint_wrench)
    nothing
end

# uses exponential coordinates centered around q0
@propagate_inbounds function local_coordinates!(ϕ::AbstractVector, ϕ̇::AbstractVector,
        jt::QuaternionSpherical, q0::AbstractVector, q::AbstractVector, v::AbstractVector)
    quat = inv(rotation(jt, q0, false)) * rotation(jt, q, false)
    rv = RodriguesVec(quat)
    ϕstatic = SVector(rv.sx, rv.sy, rv.sz)
    ϕ .= ϕstatic
    ϕ̇ .= rotation_vector_rate(ϕstatic, v)
    nothing
end

@propagate_inbounds function global_coordinates!(q::AbstractVector, jt::QuaternionSpherical, q0::AbstractVector, ϕ::AbstractVector)
    quat0 = rotation(jt, q0, false)
    quat = quat0 * Quat(RodriguesVec(ϕ[1], ϕ[2], ϕ[3]))
    set_rotation!(q, jt, quat)
    nothing
end

@propagate_inbounds normalize_configuration!(q::AbstractVector, jt::QuaternionSpherical) = set_rotation!(q, jt, rotation(jt, q, true))

@propagate_inbounds function is_configuration_normalized(jt::QuaternionSpherical, q::AbstractVector, rtol, atol)
    isapprox(quatnorm(rotation(jt, q, false)), one(eltype(q)); rtol = rtol, atol = atol)
end

@propagate_inbounds function principal_value!(q::AbstractVector, jt::QuaternionSpherical)
    quat = rotation(jt, q, false)
    set_rotation!(q, jt, principal_value(quat))
    nothing
end
