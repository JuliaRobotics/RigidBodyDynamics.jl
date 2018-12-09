@indextype JointID

"""
$(TYPEDEF)

The abstract supertype of all concrete joint types.
"""
abstract type JointType{T} end

Base.eltype(::Type{<:JointType{T}}) where {T} = T
isfloating(::T) where {T<:JointType} = isfloating(T)
num_velocities(::T) where {T<:JointType} = num_velocities(T)
num_positions(::T) where {T<:JointType} = num_positions(T)
num_constraints(::Type{T}) where {T<:JointType} = 6 - num_velocities(T)

"""
$(TYPEDEF)

A joint represents a kinematic restriction of the relative twist between two
rigid bodies to a linear subspace of dimension ``k``.

A joint has a direction. The rigid body before the joint is called the
joint's predecessor, and the rigid body after the joint is its successor.

The state related to the joint is parameterized by two sets of variables, namely

* a vector ``q \\in \\mathcal{Q}``, parameterizing the relative homogeneous transform.
* a vector ``v \\in \\mathbb{R}^k``, parameterizing the relative twist.

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
struct Joint{T, JT<:JointType{T}}
    name::String
    frame_before::CartesianFrame3D
    frame_after::CartesianFrame3D
    joint_type::JT
    id::Base.RefValue{JointID}
    joint_to_predecessor::Base.RefValue{Transform3D{T}}
    joint_to_successor::Base.RefValue{Transform3D{T}}
    position_bounds::Vector{Bounds{T}}
    velocity_bounds::Vector{Bounds{T}}
    effort_bounds::Vector{Bounds{T}}

    function Joint(name::String, frame_before::CartesianFrame3D, frame_after::CartesianFrame3D, joint_type::JointType{T};
            position_bounds::Vector{Bounds{T}}=fill(Bounds{T}(), num_positions(joint_type)),
            velocity_bounds::Vector{Bounds{T}}=fill(Bounds{T}(), num_velocities(joint_type)),
            effort_bounds::Vector{Bounds{T}}=fill(Bounds{T}(), num_velocities(joint_type))) where {T}
        JT = typeof(joint_type)
        id = Ref(JointID(-1))
        joint_to_predecessor = Ref(one(Transform3D{T}, frame_before))
        joint_to_successor = Ref(one(Transform3D{T}, frame_after))
        new{T, JT}(name, frame_before, frame_after, joint_type, id,
            joint_to_predecessor, joint_to_successor,
            position_bounds, velocity_bounds, effort_bounds)
    end
end

function Joint(name::String, jtype::JointType; kw...)
    Joint(name, CartesianFrame3D(string("before_", name)), CartesianFrame3D(string("after_", name)), jtype; kw...)
end

Base.print(io::IO, joint::Joint) = print(io, joint.name)
frame_before(joint::Joint) = joint.frame_before
frame_after(joint::Joint) = joint.frame_after
joint_type(joint::Joint) = joint.joint_type
joint_to_predecessor(joint::Joint) = joint.joint_to_predecessor[]
joint_to_successor(joint::Joint) = joint.joint_to_successor[]

"""
$(SIGNATURES)

Return a `Vector{Bounds{T}}` giving the upper and lower bounds of the
configuration for `joint`
"""
position_bounds(joint::Joint) = joint.position_bounds

"""
$(SIGNATURES)

Return a `Vector{Bounds{T}}` giving the upper and lower bounds of the
velocity for `joint`
"""
velocity_bounds(joint::Joint) = joint.velocity_bounds

"""
$(SIGNATURES)

Return a `Vector{Bounds{T}}` giving the upper and lower bounds of the
effort for `joint`
"""
effort_bounds(joint::Joint) = joint.effort_bounds

JointID(joint::Joint) = joint.id[]
Base.convert(::Type{JointID}, joint::Joint) = JointID(joint)
@inline RigidBodyDynamics.Graphs.edge_id_type(::Type{<:Joint}) = JointID
@inline RigidBodyDynamics.Graphs.edge_id(joint::Joint) = convert(JointID, joint)
@inline RigidBodyDynamics.Graphs.set_edge_id!(joint::Joint, id::JointID) = (joint.id[] = id)
function RigidBodyDynamics.Graphs.flip_direction(joint::Joint)
    jtype = RigidBodyDynamics.flip_direction(joint_type(joint))
    Joint(string(joint), frame_after(joint), frame_before(joint), jtype;
        position_bounds = .-joint.position_bounds,
        velocity_bounds = .-joint.velocity_bounds,
        effort_bounds = .-joint.effort_bounds)
end

function set_joint_to_predecessor!(joint::Joint, tf::Transform3D)
    @framecheck tf.from frame_before(joint)
    joint.joint_to_predecessor[] = tf
    joint
end

function set_joint_to_successor!(joint::Joint, tf::Transform3D)
    @framecheck tf.from frame_after(joint)
    joint.joint_to_successor[] = tf
    joint
end

function Base.show(io::IO, joint::Joint)
    if get(io, :compact, false)
        print(io, joint)
    else
        print(io, "Joint \"$(string(joint))\": $(joint.joint_type)")
    end
end

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

@propagate_inbounds function set_configuration!(q::AbstractVector, joint::Joint, config::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    copyto!(q, config)
    q
end

@propagate_inbounds function set_velocity!(v::AbstractVector, joint::Joint, vel::AbstractVector)
    @boundscheck check_num_velocities(joint, v)
    copyto!(v, vel)
    v
end

"""
$(SIGNATURES)

Return the length of the configuration vector of `joint`.
"""
@inline num_positions(joint::Joint) = num_positions(joint.joint_type)

"""
$(SIGNATURES)

Return the length of the velocity vector of `joint`.
"""
@inline num_velocities(joint::Joint) = num_velocities(joint.joint_type)

"""
$(SIGNATURES)

Return the number of constraints imposed on the relative twist between the joint's predecessor and successor
"""
@inline num_constraints(joint::Joint) = 6 - num_velocities(joint)

"""
$(SIGNATURES)

Return a `Transform3D` representing the homogeneous transform from the frame
after the joint to the frame before the joint for joint configuration vector ``q``.
"""
@propagate_inbounds function joint_transform(joint::Joint, q::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    @inbounds return joint_transform(joint.joint_type, frame_after(joint), frame_before(joint), q)
end

"""
$(SIGNATURES)

Return a basis for the motion subspace of the joint in configuration ``q``.

The motion subspace basis is a ``6 \\times  k`` matrix, where ``k`` is the dimension of
the velocity vector ``v``, that maps ``v`` to the twist of the joint's successor
with respect to its predecessor. The returned motion subspace is expressed in
the frame after the joint, which is attached to the joint's successor.
"""
@propagate_inbounds function motion_subspace(joint::Joint, q::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    @inbounds return motion_subspace(joint.joint_type, frame_after(joint), joint_to_predecessor(joint).to, q)
end

"""
$(SIGNATURES)

Return a basis for the constraint wrench subspace of the joint, where
`joint_transform` is the transform from the frame after the joint to the frame
before the joint.

The constraint wrench subspace is a ``6 \\times (6 - k)`` matrix, where ``k``
is the dimension of the velocity vector ``v``, that maps a vector of Lagrange
multipliers ``\\lambda`` to the constraint wrench exerted across the joint onto
its successor.

The constraint wrench subspace is orthogonal to the motion subspace.
"""
@propagate_inbounds function constraint_wrench_subspace(joint::Joint, joint_transform::Transform3D)
    @framecheck joint_transform.from frame_after(joint)
    @framecheck joint_transform.to frame_before(joint)
    @inbounds return constraint_wrench_subspace(joint.joint_type, joint_transform)
end

"""
$(SIGNATURES)

Return the acceleration of the joint's successor with respect to its predecessor
in configuration ``q`` and at velocity ``v``, when the joint acceleration
``\\dot{v}`` is zero.
"""
@propagate_inbounds function bias_acceleration(joint::Joint, q::AbstractVector, v::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    @inbounds return bias_acceleration(joint.joint_type, frame_after(joint), joint_to_predecessor(joint).to, q, v)
end

"""
$(SIGNATURES)

Whether the joint's motion subspace and constraint wrench subspace depend on
``q``.
"""
has_fixed_subspaces(joint::Joint) = has_fixed_subspaces(joint.joint_type)

"""
$(SIGNATURES)

Compute joint velocity vector ``v`` given the joint configuration vector ``q``
and its time derivative ``\\dot{q}`` (in place).

Note that this mapping is linear.

See also [`velocity_to_configuration_derivative!`](@ref), the inverse mapping.
"""
@propagate_inbounds function configuration_derivative_to_velocity!(v::AbstractVector, joint::Joint, q::AbstractVector, q̇::AbstractVector)
    @boundscheck check_num_velocities(joint, v)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_positions(joint, q̇)
    @inbounds return configuration_derivative_to_velocity!(v, joint.joint_type, q, q̇)
end

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
we can write ``v = J_{\\dot{q} \\rightarrow v} \\dot{q}``, so
```math
\\langle f_v, v \\rangle = \\langle f_v, J_{\\dot{q} \\rightarrow v} \\dot{q} \\rangle = \\langle J_{\\dot{q} \\rightarrow v}^{*} f_v, \\dot{q} \\rangle
```
so ``f_q = J_{\\dot{q} \\rightarrow v}^{*} f_v``.

To compute ``J_{\\dot{q} \\rightarrow v}`` see [`configuration_derivative_to_velocity_jacobian`](@ref).
"""
@propagate_inbounds function configuration_derivative_to_velocity_adjoint!(fq, joint::Joint, q::AbstractVector, fv)
    @boundscheck check_num_positions(joint, fq)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, fv)
    @inbounds return configuration_derivative_to_velocity_adjoint!(fq, joint_type(joint), q, fv)
end

"""
$(SIGNATURES)

Compute the time derivative ``\\dot{q}`` of the joint configuration vector ``q``
given ``q`` and the joint velocity vector ``v`` (in place).

Note that this mapping is linear.

See also [`configuration_derivative_to_velocity!`](@ref), the inverse mapping.
"""
@propagate_inbounds function velocity_to_configuration_derivative!(q̇::AbstractVector, joint::Joint, q::AbstractVector, v::AbstractVector)
    @boundscheck check_num_positions(joint, q̇)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    @inbounds return velocity_to_configuration_derivative!(q̇, joint.joint_type, q, v)
end


"""
$(SIGNATURES)

Compute the jacobian ``J_{v \\rightarrow \\dot{q}}`` which maps joint velocity to configuration
derivative for the given joint:

```math
\\dot{q} = J_{v \\rightarrow \\dot{q}} v
```
"""
@propagate_inbounds function velocity_to_configuration_derivative_jacobian(joint::Joint, q::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    @inbounds return velocity_to_configuration_derivative_jacobian(joint.joint_type, q)
end

"""
$(SIGNATURES)

Compute the jacobian ``J_{\\dot{q} \\rightarrow v}`` which maps joint
configuration derivative to velocity for the given joint:

```math
v = J_{\\dot{q} \\rightarrow v} \\dot{q}
```
"""
@propagate_inbounds function configuration_derivative_to_velocity_jacobian(joint::Joint, q::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    @inbounds return configuration_derivative_to_velocity_jacobian(joint.joint_type, q)
end

"""
$(SIGNATURES)

Set ``q`` to the 'zero' configuration, corresponding to an identity joint
transform.
"""
@propagate_inbounds function zero_configuration!(q::AbstractVector, joint::Joint)
    @boundscheck check_num_positions(joint, q)
    @inbounds return zero_configuration!(q, joint.joint_type)
end

"""
$(SIGNATURES)

Set ``q`` to a random configuration. The distribution used depends on the
joint type.
"""
@propagate_inbounds function rand_configuration!(q::AbstractVector, joint::Joint)
    @boundscheck check_num_positions(joint, q)
    @inbounds return rand_configuration!(q, joint.joint_type)
end

"""
$(SIGNATURES)

Return the twist of `joint`'s  successor with respect to its predecessor,
expressed in the frame after the joint.

Note that this is the same as `Twist(motion_subspace(joint, q), v)`.
"""
@propagate_inbounds function joint_twist(joint::Joint, q::AbstractVector, v::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    @inbounds return joint_twist(joint.joint_type, frame_after(joint), joint_to_predecessor(joint).to, q, v)
end

"""
$(SIGNATURES)

Return the spatial acceleration of `joint`'s  successor with respect to its predecessor,
expressed in the frame after the joint.
"""
@propagate_inbounds function joint_spatial_acceleration(joint::Joint, q::AbstractVector, v::AbstractVector, vd::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    @boundscheck check_num_velocities(joint, vd)
    @inbounds return joint_spatial_acceleration(joint.joint_type, frame_after(joint), joint_to_predecessor(joint).to, q, v, vd)
end

"""
$(SIGNATURES)

Given the wrench exerted across the joint on the joint's successor, compute the
vector of joint torques ``\\tau`` (in place), in configuration `q`.
"""
@propagate_inbounds function joint_torque!(τ::AbstractVector, joint::Joint, q::AbstractVector, joint_wrench::Wrench)
    @boundscheck check_num_velocities(joint, τ)
    @boundscheck check_num_positions(joint, q)
    @framecheck(joint_wrench.frame, frame_after(joint))
    @inbounds return joint_torque!(τ, joint.joint_type, q, joint_wrench)
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
@propagate_inbounds function local_coordinates!(ϕ::AbstractVector, ϕ̇::AbstractVector,
        joint::Joint, q0::AbstractVector, q::AbstractVector, v::AbstractVector)
    @boundscheck check_num_velocities(joint, ϕ)
    @boundscheck check_num_velocities(joint, ϕ̇)
    @boundscheck check_num_positions(joint, q0)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_velocities(joint, v)
    @inbounds return local_coordinates!(ϕ, ϕ̇, joint.joint_type, q0, q, v)
end

"""
$(SIGNATURES)

Compute the global parameterization of the joint's configuration, ``q``, given
a 'base' orientation ``q_0`` and a vector of local coordinates ``ϕ`` centered
around ``q_0``.

See also [`local_coordinates!`](@ref).
"""
@propagate_inbounds function global_coordinates!(q::AbstractVector, joint::Joint, q0::AbstractVector, ϕ::AbstractVector)
    @boundscheck check_num_positions(joint, q)
    @boundscheck check_num_positions(joint, q0)
    @boundscheck check_num_velocities(joint, ϕ)
    @inbounds return global_coordinates!(q, joint.joint_type, q0, ϕ)
end

"""
$(SIGNATURES)

Whether the joint is a floating joint, i.e., whether it imposes no constraints
on the relative motions of its successor and predecessor bodies.
"""
isfloating(joint::Joint) = isfloating(joint.joint_type)

"""
$(SIGNATURES)

Renormalize the configuration vector ``q`` associated with `joint` so that it
lies on the joint's configuration manifold.
"""
@propagate_inbounds function normalize_configuration!(q::AbstractVector, joint::Joint)
    @boundscheck check_num_positions(joint, q)
    @inbounds return normalize_configuration!(q, joint.joint_type)
end

@propagate_inbounds function is_configuration_normalized(joint::Joint, q::AbstractVector{X}; rtol::Real = sqrt(eps(X)), atol::Real = zero(X)) where {X}
    @boundscheck check_num_positions(joint, q)
    @inbounds return is_configuration_normalized(joint.joint_type, q, rtol, atol)
end

"""
$(SIGNATURES)

Applies the principal_value functions from [Rotations.jl](https://github.com/FugroRoames/Rotations.jl/blob/d080990517f89b56c37962ad53a7fd24bd94b9f7/src/principal_value.jl)
to joint angles. This currently only applies to `SPQuatFloating` joints.
"""
@propagate_inbounds function principal_value!(q::AbstractVector, joint::Joint)
    @boundscheck check_num_positions(joint, q)
    @inbounds return principal_value!(q, joint.joint_type)
end
