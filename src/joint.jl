# The constructor setup for Joint may look strange. The constructors are
# designed so that e.g. a call to Joint("bla", QuaternionFloating{Float64}())
# returns a Joint{T, JointType{T}}, not a JointType{T, QuaternionFloating{T}}.
#
# This was done because we want a collection of joints with different joint
# types to be storable in a type-homogeneous container. If that were not the
# case, then e.g. iterating over the collection and calling frame_after on each
# joint would not be type-stable.
#
# If JointType-dependent functions need to be called in a type-stable fashion,
# a joint with a concrete joint type may be constructed from a
# Joint{T, JointType{T}}.

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
mutable struct Joint{T, JT<:JointType{T}}
    name::String
    frameBefore::CartesianFrame3D
    frameAfter::CartesianFrame3D
    jointType::JT
    id::Int64

    function Joint{T, JT}(name::String, frameBefore::CartesianFrame3D, frameAfter::CartesianFrame3D, jointType::JointType{T}) where {T, JT<:JointType{T}}
        new{T, JointType{T}}(name, frameBefore, frameAfter, jointType, -1)
    end

    function Joint(other::Joint{T}) where T
        JT = typeof(other.jointType)
        new{T, JT}(other.name, other.frameBefore, other.frameAfter, other.jointType, other.id)
    end
end

const GenericJoint{T} = Joint{T, JointType{T}}

function Joint(name::String, frameBefore::CartesianFrame3D, frameAfter::CartesianFrame3D, jtype::JointType{T}) where T
    GenericJoint{T}(name, frameBefore, frameAfter, jtype)
end

function Joint(name::String, jtype::JointType)
    Joint(name, CartesianFrame3D(string("before_", name)), CartesianFrame3D(string("after_", name)), jtype)
end

typedjoint(joint::Joint) = Joint(joint)

frame_before(joint::Joint) = joint.frameBefore
frame_after(joint::Joint) = joint.frameAfter
joint_type(joint::Joint) = joint.jointType

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


"""
$(SIGNATURES)

Return the length of the configuration vector of `joint`.
"""
num_positions(joint::Joint) = num_positions(joint.jointType)

"""
$(SIGNATURES)

Return the length of the velocity vector of `joint`.
"""
num_velocities(joint::Joint) = num_velocities(joint.jointType)

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
function joint_transform(joint::Joint, q::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    joint_transform(joint.jointType, frame_after(joint), frame_before(joint), q)
end

"""
$(SIGNATURES)

Return a basis for the motion subspace of the joint in configuration ``q``.

The motion subspace basis is a ``6 \\times  k`` matrix, where ``k`` is the dimension of
the velocity vector ``v``, that maps ``v`` to the twist of the joint's successor
with respect to its predecessor. The returned motion subspace is expressed in
the frame after the joint, which is attached to the joint's successor.
"""
@inline function motion_subspace(joint::Joint, q::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    motion_subspace(joint.jointType, frame_after(joint), frame_before(joint), q)
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
function constraint_wrench_subspace(joint::Joint, jointTransform::Transform3D)
    @framecheck jointTransform.from frame_after(joint)
    @framecheck jointTransform.to frame_before(joint)
    constraint_wrench_subspace(joint.jointType, jointTransform)
end

"""
$(SIGNATURES)

Return the acceleration of the joint's successor with respect to its predecessor
in configuration ``q`` and at velocity ``v``, when the joint acceleration
``\\dot{v}`` is zero.
"""
function bias_acceleration(joint::Joint, q::AbstractVector, v::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    bias_acceleration(joint.jointType, frame_after(joint), frame_before(joint), q, v)
end

"""
$(SIGNATURES)

Whether the joint's motion subspace and constraint wrench subspace depend on
``q``.
"""
has_fixed_subspaces(joint::Joint) = has_fixed_subspaces(joint.jointType)

"""
$(SIGNATURES)

Compute joint velocity vector ``v`` given the joint configuration vector ``q``
and its time derivative ``\\dot{q}`` (in place).

Note that this mapping is linear.

See also [`velocity_to_configuration_derivative!`](@ref), the inverse mapping.
"""
function configuration_derivative_to_velocity!(v::AbstractVector, joint::Joint, q::AbstractVector, q̇::AbstractVector)
    @boundscheck check_num_velocities(joint, v)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_positions(joint, q̇)
    configuration_derivative_to_velocity!(v, joint.jointType, q, q̇)
end
Base.@deprecate configuration_derivative_to_velocity!(joint::Joint, v::AbstractVector, q::AbstractVector, q̇::AbstractVector) configuration_derivative_to_velocity!(v, joint, q, q̇)

"""
$(SIGNATURES)

Given  a linear function

```math
f(v) = \\langle f_v, v \\rangle
```
where ``v`` is the joint velocity vector, return a vector ``f_q`` such that

```math
\\langle f_v, v \\rangle = \\langle f_q, \\dot{q}(v) \\rangle.
```

Note: since ``v`` is a linear function of ``\\dot{q}`` (see [`configuration_derivative_to_velocity!`](@ref)),
we can write ``v = V_q \\dot{q}``, so
```math
\\langle f_v, v \\rangle = \\langle f_v, V_q \\dot{q} \\rangle = \\langle V_q^{*} f_v, \\dot{q} \\rangle
```
so ``f_q = V_q^{*} f_v``.
"""
function configuration_derivative_to_velocity_adjoint!(fq, joint::Joint, q::AbstractVector, fv)
    @boundscheck check_num_positions(joint, fq)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, fv)
    configuration_derivative_to_velocity_adjoint!(fq, joint_type(joint), q, fv)
end

"""
$(SIGNATURES)

Compute the time derivative ``\\dot{q}`` of the joint configuration vector ``q``
given ``q`` and the joint velocity vector ``v`` (in place).

Note that this mapping is linear.

See also [`configuration_derivative_to_velocity!`](@ref), the inverse mapping.
"""
function velocity_to_configuration_derivative!(q̇::AbstractVector, joint::Joint, q::AbstractVector, v::AbstractVector)
    @boundscheck check_num_positions(joint, q̇)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    velocity_to_configuration_derivative!(q̇, joint.jointType, q, v)
end
Base.@deprecate velocity_to_configuration_derivative!(joint::Joint, q̇::AbstractVector, q::AbstractVector, v::AbstractVector) velocity_to_configuration_derivative!(q̇, joint, q, v)

"""
$(SIGNATURES)

Set ``q`` to the 'zero' configuration, corresponding to an identity joint
transform.
"""
function zero_configuration!(q::AbstractVector, joint::Joint)
    @boundscheck check_num_positions(joint, q)
    zero_configuration!(q, joint.jointType)
end
Base.@deprecate zero_configuration!(joint::Joint, q::AbstractVector) zero_configuration!(q, joint)

"""
$(SIGNATURES)

Set ``q`` to a random configuration. The distribution used depends on the
joint type.
"""
function rand_configuration!(q::AbstractVector, joint::Joint)
    @boundscheck check_num_positions(joint, q)
    rand_configuration!(q, joint.jointType)
end
Base.@deprecate rand_configuration!(joint::Joint, q::AbstractVector) rand_configuration!(q, joint)

"""
$(SIGNATURES)

Return the twist of `joint`'s  successor with respect to its predecessor,
expressed in the frame after the joint.

Note that this is the same as `Twist(motion_subspace(joint, q), v)`.
"""
function joint_twist(joint::Joint, q::AbstractVector, v::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    joint_twist(joint.jointType, frame_after(joint), frame_before(joint), q, v)
end

"""
$(SIGNATURES)

Return the spatial acceleration of `joint`'s  successor with respect to its predecessor,
expressed in the frame after the joint.
"""
function joint_spatial_acceleration(joint::Joint, q::AbstractVector, v::AbstractVector, vd::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    @boundscheck check_num_velocities(joint, vd)
    joint_spatial_acceleration(joint.jointType, frame_after(joint), frame_before(joint), q, v, vd)
end

"""
$(SIGNATURES)

Given the wrench exerted across the joint on the joint's successor, compute the
vector of joint torques ``\\tau`` (in place), in configuration `q`.
"""
function joint_torque!(τ::AbstractVector, joint::Joint, q::AbstractVector, joint_wrench::Wrench)
    @boundscheck check_num_velocities(joint, τ)
    @boundscheck check_num_positions(joint, q)
    @framecheck(joint_wrench.frame, frame_after(joint))
    joint_torque!(τ, joint.jointType, q, joint_wrench)
end

Base.@deprecate joint_torque!(joint::Joint, τ::AbstractVector, q::AbstractVector, joint_wrench::Wrench) joint_torque!(τ, joint, q, joint_wrench)

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
function local_coordinates!(ϕ::AbstractVector, ϕ̇::AbstractVector,
        joint::Joint, q0::AbstractVector, q::AbstractVector, v::AbstractVector)
    @boundscheck check_num_velocities(joint, ϕ)
    @boundscheck check_num_velocities(joint, ϕ̇)
    @boundscheck check_num_positions(joint, q0)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    local_coordinates!(ϕ, ϕ̇, joint.jointType, q0, q, v)
end
Base.@deprecate local_coordinates!(joint::Joint, ϕ::AbstractVector, ϕ̇::AbstractVector, q0::AbstractVector, q::AbstractVector, v::AbstractVector) local_coordinates!(ϕ, ϕ̇, joint, q0, q, v)

"""
$(SIGNATURES)

Compute the global parameterization of the joint's configuration, ``q``, given
a 'base' orientation ``q_0`` and a vector of local coordinates ``ϕ`` centered
around ``q_0``.

See also [`local_coordinates!`](@ref).
"""
function global_coordinates!(q::AbstractVector, joint::Joint, q0::AbstractVector, ϕ::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_positions(joint, q0)
    @boundscheck check_num_velocities(joint, ϕ)
    global_coordinates!(q, joint.jointType, q0, ϕ)
end
Base.@deprecate global_coordinates!(joint::Joint, q::AbstractVector, q0::AbstractVector, ϕ::AbstractVector) global_coordinates!(q, joint, q0, ϕ)

"""
$(SIGNATURES)

Whether the joint is a floating joint, i.e., whether it imposes no constraints
on the relative motions of its successor and predecessor bodies.
"""
isfloating(joint::Joint) = isfloating(typeof(joint.jointType))
