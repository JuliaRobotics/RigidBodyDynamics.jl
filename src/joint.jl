"""
$(TYPEDEF)

A joint represents a kinematic restriction of the relative twist between two
rigid bodies to a linear subspace of dimension ``k``.
The state related to the joint is parameterized by two sets of variables, namely

* a vector ``q \\in  \\mathcal{Q}``, parameterizing the relative homogeneous transform.
* a vector ``v \\in \\mathbb{R}^k``, parameterizing the relative twist.

A joint has a direction. The rigid body before the joint is called the
joint's predecessor, and the rigid body after the joint is its successor.

The twist of the successor with respect to the predecessor is a linear function
of ``v``.

For some joint types (notably those using a redundant representation of relative
orientation, such as a unit quaternion), ``\\dot{q}``, the time derivative of
``q``, may not be the same as ``v``.
However, an invertible linear transformation exists between
``\\dot{q}`` and ``v``.

See also:
* Definition 2.9 in Duindam, "Port-Based Modeling and Control for Efficient Bipedal Walking Robots", 2006.
* Section 4.4 of Featherstone, "Rigid Body Dynamics Algorithms", 2008.
"""
type Joint{T<:Number}
    name::String
    frameBefore::CartesianFrame3D
    frameAfter::CartesianFrame3D
    jointType::JointType{T}
    id::Int64

    function (::Type{Joint{T}}){T}(name::String, frameBefore::CartesianFrame3D, frameAfter::CartesianFrame3D, jointType::JointType{T})
        new{T}(name, frameBefore, frameAfter, jointType, -1)
    end
end

Joint{T}(name::String, frameBefore::CartesianFrame3D, frameAfter::CartesianFrame3D, jointType::JointType{T}) = Joint{T}(name, frameBefore, frameAfter, jointType)
Joint(name::String, jointType::JointType) = Joint(name, CartesianFrame3D(string("before_", name)), CartesianFrame3D(string("after_", name)), jointType)

frame_before(joint::Joint) = joint.frameBefore
frame_after(joint::Joint) = joint.frameAfter

RigidBodyDynamics.Graphs.edge_index(joint::Joint) = joint.id
RigidBodyDynamics.Graphs.edge_index!(joint::Joint, id::Int64) = (joint.id = id)
function RigidBodyDynamics.Graphs.flip_direction!(joint::Joint)
    newbefore = frame_after(joint)
    newafter = frame_before(joint)
    joint.frameBefore = newbefore
    joint.frameAfter = newafter
    joint.jointType = flip_direction(joint.jointType)
    joint
end

Base.show(io::IO, joint::Joint) = print(io, "Joint \"$(joint.name)\": $(joint.jointType)")
Base.showcompact(io::IO, joint::Joint) = print(io, "$(joint.name)")

# TODO: deprecate in favor of sum(num_positions, ...)
num_positions(itr) = reduce((val, joint) -> val + num_positions(joint), 0, itr)
num_velocities(itr) = reduce((val, joint) -> val + num_velocities(joint), 0, itr)

@inline function check_num_positions(joint::Joint, vec::AbstractVector)
    length(vec) == num_positions(joint) || error("wrong size")
    nothing
end

@inline function check_num_velocities(joint::Joint, vec::AbstractVector)
    length(vec) == num_velocities(joint) || error("wrong size")
    nothing
end


# 'RTTI'-style dispatch inspired by https://groups.google.com/d/msg/julia-users/ude2-MUiFLM/z-MuQ9nhAAAJ, hopefully a short-term solution.
# See https://github.com/tkoolen/RigidBodyDynamics.jl/issues/93.

"""
$(SIGNATURES)

Return the length of the configuration vector of `joint`.
"""
num_positions{M}(joint::Joint{M})::Int64 = @rtti_dispatch (QuaternionFloating{M}, Revolute{M}, Prismatic{M}, Fixed{M}) num_positions(joint.jointType)

"""
$(SIGNATURES)

Return the length of the velocity vector of `joint`.
"""
num_velocities{M}(joint::Joint{M})::Int64 = @rtti_dispatch (QuaternionFloating{M}, Revolute{M}, Prismatic{M}, Fixed{M}) num_velocities(joint.jointType)

"""
$(SIGNATURES)

Return the number of constraints imposed on the relative twist between the joint's predecessor and successor
"""
num_constraints(joint::Joint) = 6 - num_velocities(joint)

"""
$(SIGNATURES)

Return a `Transform3D` representing the homogeneous transform from the frame
after the joint to the frame before the joint for joint configuration vector ``q``.
"""
function joint_transform{M, X}(joint::Joint{M}, q::AbstractVector{X})::Transform3D{promote_type(M, X)}
    @boundscheck check_num_positions(joint, q)
    @rtti_dispatch (QuaternionFloating{M}, Revolute{M}, Prismatic{M}, Fixed{M}) _joint_transform(joint.jointType, frame_after(joint), frame_before(joint), q)
end

"""
$(SIGNATURES)

Return a basis for the motion subspace of the joint in configuration ``q``.

The motion subspace basis is a ``6 \\times  k`` matrix, where ``k`` is the dimension of
the velocity vector ``v``, that maps ``v`` to the twist of the joint's successor
with respect to its predecessor. The returned motion subspace is expressed in
the frame after the joint, which is attached to the joint's successor.
"""
function motion_subspace{M, X}(joint::Joint{M}, q::AbstractVector{X})::MotionSubspace{promote_type(M, X)}
    @boundscheck check_num_positions(joint, q)
    @rtti_dispatch (QuaternionFloating{M}, Revolute{M}, Prismatic{M}, Fixed{M}) _motion_subspace(joint.jointType, frame_after(joint), frame_before(joint), q)
end

"""
$(SIGNATURES)

Return a basis for the constraint wrench subspace of the joint, where
`jointTransform` is the transform from the frame after the joint to the frame
before the joint.

The constraint wrench subspace is a ``6 \\times (6 - k)`` matrix, where ``k``
is the dimension of the velocity vector ``v``, that maps a vector of Lagrange
multipliers ``\\lambda`` to the constraint wrench exerted across the joint onto
its successor.

The constraint wrench subspace is orthogonal to the motion subspace.
"""
function constraint_wrench_subspace{M, X}(joint::Joint{M}, jointTransform::Transform3D{X})#::WrenchSubspace{promote_type(M, X)} # FIXME: type assertion causes segfault! see https://github.com/JuliaLang/julia/issues/20034. should be fixed in 0.6
    @framecheck jointTransform.from frame_after(joint)
    @framecheck jointTransform.to frame_before(joint)
    @rtti_dispatch (QuaternionFloating{M}, Revolute{M}, Prismatic{M}, Fixed{M}) _constraint_wrench_subspace(joint.jointType, jointTransform)
end

"""
$(SIGNATURES)

Return the acceleration of the joint's successor with respect to its predecessor
in configuration ``q`` and at velocity ``v``, when the joint acceleration
``\\dot{v}`` is zero.
"""
function bias_acceleration{M, X}(joint::Joint{M}, q::AbstractVector{X}, v::AbstractVector{X})::SpatialAcceleration{promote_type(M, X)}
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    @rtti_dispatch (QuaternionFloating{M}, Revolute{M}, Prismatic{M}, Fixed{M}) _bias_acceleration(joint.jointType, frame_after(joint), frame_before(joint), q, v)
end

"""
$(SIGNATURES)

Whether the joint's motion subspace and constraint wrench subspace depend on
``q``.
"""
function has_fixed_subspaces{M}(joint::Joint{M})
    @rtti_dispatch (QuaternionFloating{M}, Revolute{M}, Prismatic{M}, Fixed{M}) _has_fixed_subspaces(joint.jointType)
end

"""
$(SIGNATURES)

Compute joint velocity vector ``v`` given the joint configuration vector ``q``
and its time derivative ``\\dot{q}`` (in place).

Note that this mapping is linear.

See also [`velocity_to_configuration_derivative!`](@ref), the inverse mapping.
"""
function configuration_derivative_to_velocity!{M}(joint::Joint{M}, v::AbstractVector, q::AbstractVector, q̇::AbstractVector)::Void
    @boundscheck check_num_velocities(joint, v)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_positions(joint, q̇)
    @rtti_dispatch (QuaternionFloating{M}, Revolute{M}, Prismatic{M}, Fixed{M}) _configuration_derivative_to_velocity!(joint.jointType, v, q, q̇)
end

"""
$(SIGNATURES)

Compute the time derivative ``\\dot{q}`` of the joint configuration vector ``q``
given ``q`` and the joint velocity vector ``v`` (in place).

Note that this mapping is linear.

See also [`configuration_derivative_to_velocity!`](@ref), the inverse mapping.
"""
function velocity_to_configuration_derivative!{M}(joint::Joint{M}, q̇::AbstractVector, q::AbstractVector, v::AbstractVector)::Void
    @boundscheck check_num_positions(joint, q̇)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    @rtti_dispatch (QuaternionFloating{M}, Revolute{M}, Prismatic{M}, Fixed{M}) _velocity_to_configuration_derivative!(joint.jointType, q̇, q, v)
end

"""
$(SIGNATURES)

Set ``q`` to the 'zero' configuration, corresponding to an identity joint
transform.
"""
function zero_configuration!{M}(joint::Joint{M}, q::AbstractVector)::Void
    @boundscheck check_num_positions(joint, q)
    @rtti_dispatch (QuaternionFloating{M}, Revolute{M}, Prismatic{M}, Fixed{M}) _zero_configuration!(joint.jointType, q)
end

"""
$(SIGNATURES)

Set ``q`` to a random configuration. The distribution used depends on the
joint type.
"""
function rand_configuration!{M}(joint::Joint{M}, q::AbstractVector)::Void
    @boundscheck check_num_positions(joint, q)
    @rtti_dispatch (QuaternionFloating{M}, Revolute{M}, Prismatic{M}, Fixed{M}) _rand_configuration!(joint.jointType, q)
end

"""
$(SIGNATURES)

Return the twist of `joint`'s  successor with respect to its predecessor,
expressed in the frame after the joint.

Note that this is the same as `Twist(motion_subspace(joint, q), v)`.
"""
function joint_twist{M, X}(joint::Joint{M}, q::AbstractVector{X}, v::AbstractVector{X})::Twist{promote_type(M, X)}
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    @rtti_dispatch (QuaternionFloating{M}, Revolute{M}, Prismatic{M}, Fixed{M}) _joint_twist(joint.jointType, frame_after(joint), frame_before(joint), q, v)
end

"""
$(SIGNATURES)

Given the wrench exerted across the joint on the joint's successor, compute the
vector of joint torques ``\\tau`` (in place), in configuration `q`.
"""
function joint_torque!{M}(joint::Joint{M}, τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench)::Void
    @boundscheck check_num_velocities(joint, τ)
    @boundscheck check_num_positions(joint, q)
    @framecheck(joint_wrench.frame, frame_after(joint))
    @rtti_dispatch (QuaternionFloating{M}, Revolute{M}, Prismatic{M}, Fixed{M}) _joint_torque!(joint.jointType, τ, q, joint_wrench)
end

"""
$(SIGNATURES)

Compute a vector of local coordinates ``\\phi`` around configuration ``q_0``
corresponding to configuration ``q`` (in place). Also compute the time
derivative ``\\dot{\\phi}`` of ``\\phi`` given the joint velocity vector ``v``.

The local coordinate vector ``\\phi`` must be zero if and only if ``q = q_0``.

For revolute or prismatic joint types, the local coordinates can just be
``\\phi = q - q_0``, but for joint types with configuration vectors that are
restricted to a manifold (e.g. when unit quaternions are used to represent
orientation), elementwise subtraction may not make sense. For such joints,
exponential coordinates could be used as the local coordinate vector ``\\phi``.

See also [`global_coordinates!`](@ref).
"""
function local_coordinates!{M}(joint::Joint{M},
        ϕ::AbstractVector, ϕ̇::AbstractVector,
        q0::AbstractVector, q::AbstractVector, v::AbstractVector)
    @boundscheck check_num_velocities(joint, ϕ)
    @boundscheck check_num_velocities(joint, ϕ̇)
    @boundscheck check_num_positions(joint, q0)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    @rtti_dispatch (QuaternionFloating{M}, Revolute{M}, Prismatic{M}, Fixed{M}) _local_coordinates!(joint.jointType, ϕ, ϕ̇, q0, q, v)
end

"""
$(SIGNATURES)

Compute the global parameterization of the joint's configuration, ``q``, given
a 'base' orientation ``q_0`` and a vector of local coordinates ``ϕ`` centered
around ``q_0``.

See also [`local_coordinates!`](@ref).
"""
function global_coordinates!{M}(joint::Joint{M}, q::AbstractVector, q0::AbstractVector, ϕ::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_positions(joint, q0)
    @boundscheck check_num_velocities(joint, ϕ)
    @rtti_dispatch (QuaternionFloating{M}, Revolute{M}, Prismatic{M}, Fixed{M}) _global_coordinates!(joint.jointType, q, q0, ϕ)
end
