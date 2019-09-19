"""
$(TYPEDEF)

A floating joint type that uses a SPQuat representation for orientation.

Floating joints are 6-degree-of-freedom joints that are in a sense degenerate,
as they impose no constraints on the relative motion between two bodies.

The 6-dimensional configuration vector of a `SPQuatFloating` joint
type consists of a SPQuat representing the orientation that rotates
vectors from the frame 'directly after' the joint to the frame 'directly before'
it, and a 3D position vector representing the origin of the frame after the
joint in the frame before the joint.

The 6-dimensional velocity vector of a `SPQuatFloating` joint is the twist
of the frame after the joint with respect to the frame before it, expressed in
the frame after the joint.
"""
struct SPQuatFloating{T} <: JointType{T} end

Base.show(io::IO, jt::SPQuatFloating) = print(io, "SPQuat floating joint")
Random.rand(::Type{SPQuatFloating{T}}) where {T} = SPQuatFloating{T}()

num_positions(::Type{<:SPQuatFloating}) = 6
num_velocities(::Type{<:SPQuatFloating}) = 6
has_fixed_subspaces(jt::SPQuatFloating) = true
isfloating(::Type{<:SPQuatFloating}) = true

@propagate_inbounds function rotation(jt::SPQuatFloating, q::AbstractVector)
    SPQuat(q[1], q[2], q[3])
end

@propagate_inbounds function set_rotation!(q::AbstractVector, jt::SPQuatFloating, rot::Rotation{3})
    T = eltype(rot)
    spq = convert(SPQuat{T}, rot)
    q[1] = spq.x
    q[2] = spq.y
    q[3] = spq.z
    nothing
end

@propagate_inbounds function set_rotation!(q::AbstractVector, jt::SPQuatFloating, rot::AbstractVector)
    q[1] = rot[1]
    q[2] = rot[2]
    q[3] = rot[3]
    nothing
end

@propagate_inbounds translation(jt::SPQuatFloating, q::AbstractVector) = SVector(q[4], q[5], q[6])
@propagate_inbounds set_translation!(q::AbstractVector, jt::SPQuatFloating, trans::AbstractVector) = copyto!(q, 4, trans, 1, 3)

@propagate_inbounds angular_velocity(jt::SPQuatFloating, v::AbstractVector) = SVector(v[1], v[2], v[3])
@propagate_inbounds set_angular_velocity!(v::AbstractVector, jt::SPQuatFloating, ω::AbstractVector) = copyto!(v, 1, ω, 1, 3)

@propagate_inbounds linear_velocity(jt::SPQuatFloating, v::AbstractVector) = SVector(v[4], v[5], v[6])
@propagate_inbounds set_linear_velocity!(v::AbstractVector, jt::SPQuatFloating, ν::AbstractVector) = copyto!(v, 4, ν, 1, 3)

@inline function set_configuration!(q::AbstractVector, joint::Joint{<:Any, <:SPQuatFloating}, config::Transform3D)
    @boundscheck check_num_positions(joint, q)
    @framecheck config.from frame_after(joint)
    @framecheck config.to frame_before(joint)
    @inbounds set_rotation!(q, joint_type(joint), rotation(config))
    @inbounds set_translation!(q, joint_type(joint), translation(config))
    q
end

@propagate_inbounds function set_velocity!(v::AbstractVector, joint::Joint{<:Any, <:SPQuatFloating}, twist::Twist)
    @boundscheck check_num_velocities(joint, v)
    @framecheck twist.base frame_before(joint)
    @framecheck twist.body frame_after(joint)
    @framecheck twist.frame frame_after(joint)
    @inbounds set_angular_velocity!(v, joint_type(joint), angular(twist))
    @inbounds set_linear_velocity!(v, joint_type(joint), linear(twist))
    v
end

@propagate_inbounds function joint_transform(jt::SPQuatFloating, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector)
    Transform3D(frame_after, frame_before, rotation(jt, q), translation(jt, q))
end

@inline function motion_subspace(jt::SPQuatFloating, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector)
    S = promote_eltype(jt, q)
    angular = hcat(one(SMatrix{3, 3, S}), zero(SMatrix{3, 3, S}))
    linear = hcat(zero(SMatrix{3, 3, S}), one(SMatrix{3, 3, S}))
    GeometricJacobian(frame_after, frame_before, frame_after, angular, linear)
end

@inline function constraint_wrench_subspace(jt::SPQuatFloating, joint_transform::Transform3D)
    S = promote_eltype(jt, joint_transform)
    WrenchMatrix(joint_transform.from, zero(SMatrix{3, 0, S}), zero(SMatrix{3, 0, S}))
end

@propagate_inbounds function bias_acceleration(jt::SPQuatFloating, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector)
    S = promote_eltype(jt, q, v)
    zero(SpatialAcceleration{S}, frame_after, frame_before, frame_after)
end

@propagate_inbounds function configuration_derivative_to_velocity!(v::AbstractVector, jt::SPQuatFloating,
        q::AbstractVector, q̇::AbstractVector)
    spq = rotation(jt, q)
    spqdot = SVector(q̇[1], q̇[2], q̇[3])
    ω = angular_velocity_in_body(spq, spqdot)
    posdot = translation(jt, q̇)
    linear = inv(spq) * posdot
    set_angular_velocity!(v, jt, ω)
    set_linear_velocity!(v, jt, linear)
    nothing
end

@propagate_inbounds function configuration_derivative_to_velocity_adjoint!(fq, jt::SPQuatFloating,
        q::AbstractVector, fv)
    spq = SPQuat(q[1], q[2], q[3])
    rot = velocity_jacobian(angular_velocity_in_body, spq)' * angular_velocity(jt, fv)
    trans = spq * linear_velocity(jt, fv)
    set_rotation!(fq, jt, rot)
    set_translation!(fq, jt, trans)
    nothing
end

@propagate_inbounds function velocity_to_configuration_derivative!(q̇::AbstractVector, jt::SPQuatFloating,
        q::AbstractVector, v::AbstractVector)
    spq = rotation(jt, q)
    ω = angular_velocity(jt, v)
    linear = linear_velocity(jt, v)
    spqdot = spquat_derivative(spq, ω)
    transdot = spq * linear
    set_rotation!(q̇, jt, spqdot)
    set_translation!(q̇, jt, transdot)
    nothing
end

@propagate_inbounds function velocity_to_configuration_derivative_jacobian(jt::SPQuatFloating,
        q::AbstractVector)
    spq = rotation(jt, q)
    vj = velocity_jacobian(spquat_derivative, spq)
    R = RotMatrix(spq)
    # TODO: use hvcat once it's as fast
    @SMatrix(
        [vj[1] vj[4] vj[7]  0    0    0;
         vj[2] vj[5] vj[8]  0    0    0;
         vj[3] vj[6] vj[9]  0    0    0;
         0     0     0      R[1] R[4] R[7];
         0     0     0      R[2] R[5] R[8];
         0     0     0      R[3] R[6] R[9]])
end

@propagate_inbounds function configuration_derivative_to_velocity_jacobian(jt::SPQuatFloating,
        q::AbstractVector)
    spq = rotation(jt, q)
    vj = velocity_jacobian(angular_velocity_in_body, spq)
    R_inv = RotMatrix(inv(spq))
    # TODO: use hvcat once it's as fast
    @SMatrix(
        [vj[1] vj[4] vj[7] 0        0        0;
         vj[2] vj[5] vj[8] 0        0        0;
         vj[3] vj[6] vj[9] 0        0        0;
         0     0     0     R_inv[1] R_inv[4] R_inv[7];
         0     0     0     R_inv[2] R_inv[5] R_inv[8];
         0     0     0     R_inv[3] R_inv[6] R_inv[9]])
end

@propagate_inbounds function zero_configuration!(q::AbstractVector, jt::SPQuatFloating)
    T = eltype(q)
    set_rotation!(q, jt, one(SPQuat{T}))
    set_translation!(q, jt, zero(SVector{3, T}))
    nothing
end

@propagate_inbounds function rand_configuration!(q::AbstractVector, jt::SPQuatFloating)
    T = eltype(q)
    set_rotation!(q, jt, rand(SPQuat{T}))
    set_translation!(q, jt, rand(SVector{3, T}) - 0.5)
    nothing
end

@propagate_inbounds function joint_twist(jt::SPQuatFloating, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector)
    S = promote_eltype(jt, q, v)
    angular = convert(SVector{3, S}, angular_velocity(jt, v))
    linear = convert(SVector{3, S}, linear_velocity(jt, v))
    Twist(frame_after, frame_before, frame_after, angular, linear)
end

@propagate_inbounds function joint_spatial_acceleration(jt::SPQuatFloating, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector, vd::AbstractVector)
    S = promote_eltype(jt, q, v, vd)
    angular = convert(SVector{3, S}, angular_velocity(jt, vd))
    linear = convert(SVector{3, S}, linear_velocity(jt, vd))
    SpatialAcceleration(frame_after, frame_before, frame_after, angular, linear)
end

@propagate_inbounds function joint_torque!(τ::AbstractVector, jt::SPQuatFloating, q::AbstractVector, joint_wrench::Wrench)
    set_angular_velocity!(τ, jt, angular(joint_wrench))
    set_linear_velocity!(τ, jt, linear(joint_wrench))
    nothing
end

@propagate_inbounds function principal_value!(q::AbstractVector, jt::SPQuatFloating)
    spq = rotation(jt, q)
    set_rotation!(q, jt, principal_value(spq))
    nothing
end
