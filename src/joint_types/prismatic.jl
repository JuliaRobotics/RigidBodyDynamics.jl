"""
$(TYPEDEF)

A `Prismatic` joint type allows translation along a fixed axis.
"""
struct Prismatic{T} <: JointType{T}
    axis::SVector{3, T}
    rotation_from_z_aligned::RotMatrix3{T}

    @doc """
    $(SIGNATURES)

    Construct a new `Prismatic` joint type, allowing translation along `axis`
    (expressed in the frame before the joint).
    """ ->
    function Prismatic(axis::AbstractVector{T}) where {T}
        a = normalize(axis)
        new{T}(a, rotation_between(SVector(zero(T), zero(T), one(T)), SVector{3, T}(a)))
    end
end

Base.show(io::IO, jt::Prismatic) = print(io, "Prismatic joint with axis $(jt.axis)")

function Random.rand(::Type{Prismatic{T}}) where {T}
    axis = normalize(randn(SVector{3, T}))
    Prismatic(axis)
end

num_positions(::Type{<:Prismatic}) = 1
num_velocities(::Type{<:Prismatic}) = 1
has_fixed_subspaces(jt::Prismatic) = true
isfloating(::Type{<:Prismatic}) = false

RigidBodyDynamics.flip_direction(jt::Prismatic) = Prismatic(-jt.axis)

@propagate_inbounds function set_configuration!(q::AbstractVector, joint::Joint{<:Any, <:Prismatic}, pos::Number)
    @boundscheck check_num_positions(joint, q)
    @inbounds q[1] = pos
    q
end

@propagate_inbounds function set_velocity!(v::AbstractVector, joint::Joint{<:Any, <:Prismatic}, vel::Number)
    @boundscheck check_num_velocities(joint, v)
    @inbounds v[1] = vel
    v
end

@propagate_inbounds function rand_configuration!(q::AbstractVector, ::Prismatic)
    randn!(q)
    nothing
 end

@inline function bias_acceleration(jt::Prismatic, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector)
    S = promote_eltype(jt, q, v)
    zero(SpatialAcceleration{S}, frame_after, frame_before, frame_after)
end

@inline function velocity_to_configuration_derivative_jacobian(jt::Prismatic, q::AbstractVector)
    T = promote_eltype(jt, q)
    @SMatrix([one(T)])
end

@inline function configuration_derivative_to_velocity_jacobian(jt::Prismatic, q::AbstractVector)
    T = promote_eltype(jt, q)
    @SMatrix([one(T)])
end

@propagate_inbounds function joint_transform(jt::Prismatic, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector)
    translation = q[1] * jt.axis
    Transform3D(frame_after, frame_before, translation)
end

@propagate_inbounds function joint_twist(jt::Prismatic, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector)
    linear = jt.axis * v[1]
    Twist(frame_after, frame_before, frame_after, zero(linear), linear)
end

@propagate_inbounds function joint_spatial_acceleration(jt::Prismatic, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector, v::AbstractVector, vd::AbstractVector)
    S = promote_eltype(jt, q, v, vd)
    linear = convert(SVector{3, S}, jt.axis * vd[1])
    SpatialAcceleration(frame_after, frame_before, frame_after, zero(linear), linear)
end

@inline function motion_subspace(jt::Prismatic, frame_after::CartesianFrame3D, frame_before::CartesianFrame3D,
        q::AbstractVector)
    S = promote_eltype(jt, q)
    angular = zero(SMatrix{3, 1, S})
    linear = SMatrix{3, 1, S}(jt.axis)
    GeometricJacobian(frame_after, frame_before, frame_after, angular, linear)
end

@inline function constraint_wrench_subspace(jt::Prismatic, joint_transform::Transform3D)
    S = promote_eltype(jt, joint_transform)
    R = convert(RotMatrix3{S}, jt.rotation_from_z_aligned)
    Rcols12 = R[:, SVector(1, 2)]
    angular = hcat(R, zero(SMatrix{3, 2, S}))
    linear = hcat(zero(SMatrix{3, 3, S}), Rcols12)
    WrenchMatrix(joint_transform.from, angular, linear)
end

@propagate_inbounds function joint_torque!(τ::AbstractVector, jt::Prismatic, q::AbstractVector, joint_wrench::Wrench)
    τ[1] = dot(linear(joint_wrench), jt.axis)
    nothing
end
