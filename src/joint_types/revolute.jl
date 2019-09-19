"""
$(TYPEDEF)

A `Revolute` joint type allows rotation about a fixed axis.

The configuration vector for the `Revolute` joint type simply consists of the angle
of rotation about the specified axis. The velocity vector consists of the angular
rate, and is thus the time derivative of the configuration vector.
"""
struct Revolute{T} <: JointType{T}
    axis::SVector{3, T}
    rotation_from_z_aligned::RotMatrix3{T}

    function Revolute{T}(axis::AbstractVector) where {T}
        a = normalize(axis)
        new{T}(a, rotation_between(SVector(zero(T), zero(T), one(T)), SVector{3, T}(a)))
    end
end

"""
$(SIGNATURES)

Construct a new `Revolute` joint type, allowing rotation about `axis`
(expressed in the frame before the joint).
"""
Revolute(axis::AbstractVector{T}) where {T} = Revolute{T}(axis)

Base.show(io::IO, jt::Revolute) = print(io, "Revolute joint with axis $(jt.axis)")

function Random.rand(::Type{Revolute{T}}) where {T}
    axis = normalize(randn(SVector{3, T}))
    Revolute(axis)
end

RigidBodyDynamics.flip_direction(jt::Revolute) = Revolute(-jt.axis)

num_positions(::Type{<:Revolute}) = 1
num_velocities(::Type{<:Revolute}) = 1
has_fixed_subspaces(jt::Revolute) = true
isfloating(::Type{<:Revolute}) = false

@propagate_inbounds function set_configuration!(q::AbstractVector, joint::Joint{<:Any, <:Revolute}, θ::Number)
    @boundscheck check_num_positions(joint, q)
    @inbounds q[1] = θ
    q
end

@propagate_inbounds function set_velocity!(v::AbstractVector, joint::Joint{<:Any, <:Revolute}, θ̇::Number)
    check_num_velocities(joint, v)
    @inbounds v[1] = θ̇
    v
end

@propagate_inbounds function rand_configuration!(q::AbstractVector, ::Revolute)
    q[1] = randn()
    nothing
 end

@propagate_inbounds function joint_transform(jt::Revolute, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D, q::AbstractVector)
    aa = AngleAxis(q[1], jt.axis[1], jt.axis[2], jt.axis[3], false)
    Transform3D(frame_after, frame_before, convert(RotMatrix3{eltype(aa)}, aa))
end

@propagate_inbounds function joint_twist(jt::Revolute, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector)
    angular = jt.axis * v[1]
    Twist(frame_after, frame_before, frame_after, angular, zero(angular))
end

@inline function bias_acceleration(jt::Revolute, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector)
    S = promote_eltype(jt, q, v)
    zero(SpatialAcceleration{S}, frame_after, frame_before, frame_after)
end

@propagate_inbounds function joint_spatial_acceleration(jt::Revolute, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector, vd::AbstractVector)
    S = promote_eltype(jt, q, v, vd)
    angular = convert(SVector{3, S}, jt.axis * vd[1])
    SpatialAcceleration(frame_after, frame_before, frame_after, angular, zero(angular))
end

@inline function motion_subspace(jt::Revolute, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector)
    S = promote_eltype(jt, q)
    angular = SMatrix{3, 1, S}(jt.axis)
    linear = zero(SMatrix{3, 1, S})
    GeometricJacobian(frame_after, frame_before, frame_after, angular, linear)
end

@inline function constraint_wrench_subspace(jt::Revolute, joint_transform::Transform3D)
    S = promote_eltype(jt, joint_transform)
    R = convert(RotMatrix3{S}, jt.rotation_from_z_aligned)
    Rcols12 = R[:, SVector(1, 2)]
    angular = hcat(Rcols12, zero(SMatrix{3, 3, S}))
    linear = hcat(zero(SMatrix{3, 2, S}), R)
    WrenchMatrix(joint_transform.from, angular, linear)
end

@propagate_inbounds function joint_torque!(τ::AbstractVector, jt::Revolute, q::AbstractVector, joint_wrench::Wrench)
    τ[1] = dot(angular(joint_wrench), jt.axis)
    nothing
end

@inline function velocity_to_configuration_derivative_jacobian(jt::Revolute, q::AbstractVector)
    T = promote_eltype(jt, q)
    @SMatrix([one(T)])
end

@inline function configuration_derivative_to_velocity_jacobian(jt::Revolute, q::AbstractVector)
    T = promote_eltype(jt, q)
    @SMatrix([one(T)])
end

