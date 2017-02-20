# State information pertaining to a single joint
immutable JointState{X<:Number, M<:Number, C<:Number}
    joint::Joint{M}
    q::VectorSegment{X}
    v::VectorSegment{X}
    jointTransform::CacheElement{Transform3D{C}}
    twist::CacheElement{Twist{C}}
    biasAcceleration::CacheElement{SpatialAcceleration{C}}
    motionSubspace::CacheElement{MotionSubspace{C}}

    function JointState(joint::Joint{M}, q::VectorSegment{X}, v::VectorSegment{X})
        jointTransform = CacheElement{Transform3D{C}}()
        twist = CacheElement{Twist{C}}()
        biasAcceleration = CacheElement{SpatialAcceleration{C}}()
        motionSubspace = CacheElement{MotionSubspace{C}}()
        new(joint, q, v, jointTransform, twist, biasAcceleration, motionSubspace)
    end
end
JointState{X, M}(joint::Joint{M}, q::VectorSegment{X}, v::VectorSegment{X}) = JointState{X, M, promote_type(M, X)}(joint, q, v)

configuration(state::JointState) = state.q
velocity(state::JointState) = state.v
configuration_range(state::JointState) = first(parentindexes(state.q))
velocity_range(state::JointState) = first(parentindexes(state.v))
transform(state::JointState) = @cache_element_get!(state.jointTransform, joint_transform(state.joint, state.q))
twist(state::JointState) = @cache_element_get!(state.twist, joint_twist(state.joint, state.q, state.v))
bias_acceleration(state::JointState) = @cache_element_get!(state.biasAcceleration, bias_acceleration(state.joint, state.q, state.v))
motion_subspace(state::JointState) = @cache_element_get!(state.motionSubspace, motion_subspace(state.joint, state.q))
zero_configuration!(state::JointState) = (zero_configuration!(state.joint, state.q))
rand_configuration!(state::JointState) = (rand_configuration!(state.joint, state.q))

function setdirty!(state::JointState)
    setdirty!(state.jointTransform)
    setdirty!(state.twist)
    setdirty!(state.biasAcceleration)
    setdirty!(state.motionSubspace)
end

# State information pertaining to a single rigid body
immutable RigidBodyState{M<:Number, C<:Number}
    body::RigidBody{M}
    transformToWorld::CacheElement{Transform3D{C}}
    twist::CacheElement{Twist{C}}
    biasAcceleration::CacheElement{SpatialAcceleration{C}}
    motionSubspace::CacheElement{MotionSubspace{C}} # TODO: should this be here?
    inertia::CacheElement{SpatialInertia{C}}
    crbInertia::CacheElement{SpatialInertia{C}}

    function RigidBodyState(body::RigidBody{M}, isroot::Bool)
        transformToWorld = CacheElement{Transform3D{C}}()
        twist = CacheElement{Twist{C}}()
        biasAcceleration = CacheElement{SpatialAcceleration{C}}()
        motionSubspace = CacheElement{MotionSubspace{C}}()
        inertia = CacheElement{SpatialInertia{C}}()
        crbInertia = CacheElement{SpatialInertia{C}}()

        if isroot
            frame = default_frame(body)
            update!(transformToWorld, Transform3D(C, frame))
            update!(twist, zero(Twist{C}, frame, frame, frame))
            update!(biasAcceleration, zero(SpatialAcceleration{C}, frame, frame, frame))
        end

        new(body, transformToWorld, twist, biasAcceleration, motionSubspace, inertia, crbInertia)
    end
end

RigidBodyState{M, X}(body::RigidBody{M}, ::Type{X}, isroot::Bool) = RigidBodyState{M, promote_type(M, X)}(body, isroot)

function setdirty!(state::RigidBodyState)
    setdirty!(state.transformToWorld)
    setdirty!(state.twist)
    setdirty!(state.biasAcceleration)
    setdirty!(state.motionSubspace)
    setdirty!(state.inertia)
    setdirty!(state.crbInertia)
end

"""
$(TYPEDEF)

A `MechanismState` stores state information for an entire `Mechanism`. It
contains the joint configuration and velocity vectors ``q`` and ``v``, as well
as cache variables that depend on ``q`` and ``v`` and are aimed at preventing
double work.

Type parameters:
* `X`: the scalar type of the ``q`` and ``v`` vectors.
* `M`: the scalar type of the `Mechanism`
* `C`: the scalar type of the cache variables (`== promote_type(X, M)`)
"""
immutable MechanismState{X<:Number, M<:Number, C<:Number}
    mechanism::Mechanism{M}
    q::Vector{X}
    v::Vector{X}
    toposortedStateVertices::Vector{TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}}}
    nonRootTopoSortedStateVertices::VectorSegment{TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}}} # because of https://github.com/JuliaLang/julia/issues/14955
    constraintJacobianStructure::SparseMatrixCSC{Int64,Int64} # TODO: consider just using a Vector{Vector{Pair{Int64, Int64}}}

    function MechanismState(::Type{X}, mechanism::Mechanism{M})
        q = Vector{X}(num_positions(mechanism))
        v = zeros(X, num_velocities(mechanism))
        rootBodyState = RigidBodyState(root_body(mechanism), X, true)
        tree = Tree{RigidBodyState{M, C}, JointState{X, M, C}}(rootBodyState)
        jointStates = Dict{Joint{M}, JointState{X, M, C}}()
        bodyStates = Dict{RigidBody{M}, RigidBodyState{M, C}}()

        qStart = 1
        vStart = 1
        for vertex in filter(x -> !isroot(x), mechanism.toposortedTree)
            body = vertex_data(vertex)
            bodyState = RigidBodyState(body, X, false)
            joint = edge_to_parent_data(vertex)
            parentBody = vertex_data(parent(vertex))
            parentStateVertex = findfirst(v -> vertex_data(v).body == parentBody, tree)
            qEnd = qStart + num_positions(joint) - 1
            vEnd = vStart + num_velocities(joint) - 1
            qJoint = view(q, qStart : qEnd)
            vJoint = view(v, vStart : vEnd)
            jointState = JointState(joint, qJoint, vJoint)
            insert!(parentStateVertex, bodyState, jointState)
            zero_configuration!(joint, qJoint)
            qStart = qEnd + 1
            vStart = vEnd + 1
        end
        vertices = toposort(tree)
        constraintJacobianStructure = constraint_jacobian_structure(mechanism)
        new(mechanism, q, v, vertices, view(vertices, 2 : length(vertices)), constraintJacobianStructure)
    end
end
MechanismState{X, M}(t::Type{X}, mechanism::Mechanism{M}) = MechanismState{X, M, promote_type(X, M)}(t, mechanism)

Base.show{X, M, C}(io::IO, ::MechanismState{X, M, C}) = print(io, "MechanismState{$X, $M, $C}(…)")

"""
$(SIGNATURES)

Return the length of the joint configuration vector ``q``.
"""
num_positions(state::MechanismState) = length(state.q)

"""
$(SIGNATURES)

Return the length of the joint velocity vector ``v``.
"""
num_velocities(state::MechanismState) = length(state.v)
state_vector_eltype{X, M, C}(state::MechanismState{X, M, C}) = X
mechanism_eltype{X, M, C}(state::MechanismState{X, M, C}) = M
cache_eltype{X, M, C}(state::MechanismState{X, M, C}) = C
state_vertex(state::MechanismState, body::RigidBody) = state.toposortedStateVertices[findfirst(v -> vertex_data(v).body == body, state.toposortedStateVertices)] # FIXME: linear time
state_vertex(state::MechanismState, joint::Joint) = state.toposortedStateVertices[findfirst(v -> !isroot(v) && edge_to_parent_data(v).joint == joint, state.toposortedStateVertices)] # FIXME: linear time

"""
$(SIGNATURES)

Return the part of the configuration vector ``q`` associated with `joint`.
"""
configuration(state::MechanismState, joint::Joint) = edge_to_parent_data(state_vertex(state, joint)).q

"""
$(SIGNATURES)

Return the part of the velocity vector ``v`` associated with `joint`.
"""
velocity(state::MechanismState, joint::Joint) = edge_to_parent_data(state_vertex(state, joint)).v
non_root_vertices(state::MechanismState) = state.nonRootTopoSortedStateVertices

"""
$(SIGNATURES)

Invalidate all cache variables.
"""
function setdirty!(state::MechanismState)
    for vertex in non_root_vertices(state)
        setdirty!(vertex_data(vertex))
        setdirty!(edge_to_parent_data(vertex))
    end
end

"""
$(SIGNATURES)

'Zero' the configuration vector ``q``. Invalidates cache variables.

Note that when the `Mechanism` contains e.g. quaternion-parameterized joints,
``q`` may not actually be set to all zeros; the quaternion part of the
configuration vector would be set to identity. The contract is that each of the
joint transforms should be an identity transform.
"""
function zero_configuration!(state::MechanismState)
    for vertex in non_root_vertices(state)
        zero_configuration!(edge_to_parent_data(vertex))
    end
    setdirty!(state)
end

"""
$(SIGNATURES)

Zero the velocity vector ``v``. Invalidates cache variables.
"""
function zero_velocity!(state::MechanismState)
    X = eltype(state.v)
    fill!(state.v,  zero(X))
    setdirty!(state)
end

"""
$(SIGNATURES)

Zero both the configuration and velocity. Invalidates cache variables.

See [`zero_configuration!`](@ref), [`zero_velocity!`](@ref).
"""
zero!(state::MechanismState) = begin zero_configuration!(state); zero_velocity!(state) end

"""
$(SIGNATURES)

Randomize the configuration vector ``q``. The distribution depends on
the particular joint types present in the associated `Mechanism`. The resulting
``q`` is guaranteed to be on the `Mechanism`'s configuration manifold.
Invalidates cache variables.
"""
function rand_configuration!(state::MechanismState)
    for vertex in non_root_vertices(state)
        rand_configuration!(edge_to_parent_data(vertex))
    end
    setdirty!(state)
end

"""
$(SIGNATURES)

Randomize the velocity vector ``v``.
Invalidates cache variables.
"""
function rand_velocity!(state::MechanismState)
    rand!(state.v)
    setdirty!(state)
end

"""
$(SIGNATURES)

Randomize both the configuration and velocity.
Invalidates cache variables.
"""
Random.rand!(state::MechanismState) = begin rand_configuration!(state); rand_velocity!(state) end

"""
$(SIGNATURES)

Return the configuration vector ``q``.

Note that this returns a reference to the underlying data in `state`. The user
is responsible for calling [`setdirty!`](@ref) after modifying this vector to
ensure that dependent cache variables are invalidated.
"""
configuration(state::MechanismState) = state.q
Base.@deprecate configuration_vector(state::MechanismState) configuration(state)

"""
$(SIGNATURES)

Return the velocity vector ``v``.

Note that this function returns a read-write reference to a field in `state`.
The user is responsible for calling [`setdirty!`](@ref) after modifying this
vector to ensure that dependent cache variables are invalidated.
"""
velocity(state::MechanismState) = state.v
Base.@deprecate velocity_vector(state::MechanismState) velocity(state)

state_vector(state::MechanismState) = [configuration(state); velocity(state)]

"""
$(SIGNATURES)

Return the part of the `Mechanism`'s configuration vector ``q`` associated with
the joints on `path`.
"""
configuration_vector{T}(state::MechanismState, path::Path{RigidBody{T}, Joint{T}}) = vcat([configuration(state, joint) for joint in path.edgeData]...)
Base.@deprecate configuration_vector{T}(state::MechanismState, path::Path{RigidBody{T}, Joint{T}}) configuration(state, path)

"""
$(SIGNATURES)

Return the part of the `Mechanism`'s velocity vector ``v`` associated with
the joints on `path`.
"""
velocity{T}(state::MechanismState, path::Path{RigidBody{T}, Joint{T}}) = vcat([velocity(state, joint) for joint in path.edgeData]...)
Base.@deprecate velocity_vector{T}(state::MechanismState, path::Path{RigidBody{T}, Joint{T}}) velocity(state, path)

"""
$(SIGNATURES)

Set the part of the configuration vector associated with `joint`.
Invalidates cache variables.
"""
function set_configuration!(state::MechanismState, joint::Joint, q::AbstractVector)
    configuration(state, joint)[:] = q
    setdirty!(state)
end

"""
$(SIGNATURES)

Set the part of the velocity vector associated with `joint`.
Invalidates cache variables.
"""
function set_velocity!(state::MechanismState, joint::Joint, v::AbstractVector)
    velocity(state, joint)[:] = v
    setdirty!(state)
end

"""
$(SIGNATURES)

Set the configuration vector ``q``. Invalidates cache variables.
"""
function set_configuration!(state::MechanismState, q::AbstractVector)
    copy!(state.q, q)
    setdirty!(state)
end

"""
$(SIGNATURES)

Set the velocity vector ``v``. Invalidates cache variables.
"""
function set_velocity!(state::MechanismState, v::AbstractVector)
    copy!(state.v, v)
    setdirty!(state)
end

function set!(state::MechanismState, x::AbstractVector)
    nq = num_positions(state)
    nv = num_velocities(state)
    length(x) == nq + nv || error("wrong size")
    @inbounds copy!(state.q, 1, x, 1, nq)
    @inbounds copy!(state.v, 1, x, nq + 1, nv)
    setdirty!(state)
end

function transform_to_root{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}})
    @cache_element_get!(vertex_data(vertex).transformToWorld, begin
        parentVertex = parent(vertex)
        parentBody = vertex_data(parentVertex).body
        jointState = edge_to_parent_data(vertex)
        parentToWorld = transform_to_root(parentVertex)
        beforeJointToParent = frame_definition(parentBody, jointState.joint.frameBefore)
        afterJointToBeforeJoint = transform(jointState)
        parentToWorld * beforeJointToParent * afterJointToBeforeJoint
    end)
end

function twist_wrt_world{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}})
    @cache_element_get!(vertex_data(vertex).twist, begin
        parentVertex = parent(vertex)
        parentFrame = default_frame(vertex_data(parentVertex).body)
        parentTwist = twist_wrt_world(parentVertex)
        jointTwist = change_base(twist(edge_to_parent_data(vertex)), parentFrame) # to make frames line up
        parentTwist + transform(jointTwist, transform_to_root(vertex))
    end)
end

function bias_acceleration{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}})
    @cache_element_get!(vertex_data(vertex).biasAcceleration, begin
        parentVertex = parent(vertex)
        parentFrame = default_frame(vertex_data(parentVertex).body)
        parentBias = bias_acceleration(parentVertex)
        toRoot = transform_to_root(vertex)
        jointBias = change_base(bias_acceleration(edge_to_parent_data(vertex)), parentFrame) # to make frames line up
        twistWrtWorld = transform(twist_wrt_world(vertex), inv(toRoot)) # TODO: awkward way of doing this
        jointTwist = change_base(twist(edge_to_parent_data(vertex)), parentFrame) # to make frames line up
        jointBias = transform(jointBias, toRoot, twistWrtWorld, jointTwist)
        parentBias + jointBias
    end)
end

function motion_subspace{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}})
    @cache_element_get!(vertex_data(vertex).motionSubspace, begin
        parentVertex = parent(vertex)
        parentFrame = default_frame(vertex_data(parentVertex).body)
        motionSubspace = change_base(motion_subspace(edge_to_parent_data(vertex)), parentFrame)
        transform(motionSubspace, transform_to_root(vertex))
    end)
end

function spatial_inertia{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}})
    @cache_element_get!(vertex_data(vertex).inertia,
        transform(spatial_inertia(vertex_data(vertex).body), transform_to_root(vertex)))
end

function crb_inertia{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}})
    @cache_element_get!(vertex_data(vertex).crbInertia, begin
        ret = spatial_inertia(vertex)
        for child in children(vertex)
            ret += crb_inertia(child)
        end
        ret
    end)
end

function newton_euler{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}}, accel::SpatialAcceleration)
    inertia = spatial_inertia(vertex)
    twist = twist_wrt_world(vertex)
    newton_euler(inertia, accel, twist)
end

momentum{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}}) = spatial_inertia(vertex) * twist_wrt_world(vertex)
momentum_rate_bias{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}}) = newton_euler(vertex, bias_acceleration(vertex))
kinetic_energy{X, M, C}(vertex::TreeVertex{RigidBodyState{M, C}, JointState{X, M, C}}) = kinetic_energy(spatial_inertia(vertex), twist_wrt_world(vertex))

function configuration_derivative!{X}(out::AbstractVector{X}, state::MechanismState{X})
    for vertex in non_root_vertices(state)
        jointState = edge_to_parent_data(vertex)
        q = configuration(jointState)
        v = velocity(jointState)
        q̇ = UnsafeVectorView(out, configuration_range(jointState))
        velocity_to_configuration_derivative!(jointState.joint, q̇, q, v)
    end
end

function configuration_derivative{X}(state::MechanismState{X})
    ret = Vector{X}(num_positions(state.mechanism))
    configuration_derivative!(ret, state)
    ret
end

function transform_to_root(state::MechanismState, frame::CartesianFrame3D)
    body = body_fixed_frame_to_body(state.mechanism, frame) # TODO: expensive
    tf = transform_to_root(state_vertex(state, body))
    if tf.from != frame
        tf = tf * body_fixed_frame_definition(state.mechanism, frame) # TODO: consider caching
    end
    tf
end

for fun in (:configuration_range, :velocity_range)
    @eval $fun(state::MechanismState, joint::Joint) = $fun(edge_to_parent_data(state_vertex(state, joint)))
end

motion_subspace(state::MechanismState, joint::Joint) = motion_subspace(state_vertex(state, joint))

for fun in (:twist_wrt_world, :bias_acceleration, :spatial_inertia, :crb_inertia, :momentum, :momentum_rate_bias, :kinetic_energy)
    @eval $fun{X, M, C}(state::MechanismState{X, M, C}, body::RigidBody{M}) = $fun(state_vertex(state, body))
end

for fun in (:momentum, :momentum_rate_bias, :kinetic_energy)
    @eval $fun{X, M, C}(state::MechanismState{X, M, C}) = sum($fun, non_root_vertices(state))
    @eval $fun{X, M, C}(state::MechanismState{X, M, C}, body_itr) = sum($fun(state, body) for body in body_itr)
end

"""
$(SIGNATURES)

Return the homogeneous transform from `from` to `to`.
"""
function relative_transform(state::MechanismState, from::CartesianFrame3D, to::CartesianFrame3D)
    rootFrame = root_frame(state.mechanism)
    if to == rootFrame
        return transform_to_root(state, from)
    elseif from == rootFrame
        return inv(transform_to_root(state, to))
    else
        return inv(transform_to_root(state, to)) * transform_to_root(state, from)
    end
end

"""
$(SIGNATURES)

Return the twist of `body` with respect to `base`, expressed in the
`Mechanism`'s root frame.
"""
function relative_twist(state::MechanismState, body::RigidBody, base::RigidBody)
    rootBody = root_body(state.mechanism)
    if base == rootBody
        return twist_wrt_world(state, body)
    elseif body == rootBody
        return -twist_wrt_world(state, base)
    else
        return -twist_wrt_world(state, base) + twist_wrt_world(state, body)
    end
 end

 """
 $(SIGNATURES)

 Return the twist of `bodyFrame` with respect to `baseFrame`, expressed in the
 `Mechanism`'s root frame.
 """
function relative_twist(state::MechanismState, bodyFrame::CartesianFrame3D, baseFrame::CartesianFrame3D)
    twist = relative_twist(state, body_fixed_frame_to_body(state.mechanism, bodyFrame), body_fixed_frame_to_body(state.mechanism, baseFrame))
    Twist(bodyFrame, baseFrame, twist.frame, twist.angular, twist.linear)
end

for VectorType in (:Point3D, :FreeVector3D, :Twist, :Momentum, :Wrench)
    @eval begin
        function transform(state::MechanismState, v::$VectorType, to::CartesianFrame3D)::similar_type(typeof(v), promote_type(cache_eltype(state), eltype(v)))
            # TODO: consider transforming in steps, so that computing the relative transform is not necessary
            v.frame == to ? v : transform(v, relative_transform(state, v.frame, to))
        end
    end
end

function transform(state::MechanismState, accel::SpatialAcceleration, to::CartesianFrame3D)
    accel.frame == to && return accel # nothing to be done
    oldToRoot = transform_to_root(state, accel.frame)
    rootToOld = inv(oldToRoot)
    twistOfBodyWrtBase = transform(relative_twist(state, accel.body, accel.base), rootToOld)
    twistOfOldWrtNew = transform(relative_twist(state, accel.frame, to), rootToOld)
    oldToNew = inv(transform_to_root(state, to)) * oldToRoot
    transform(accel, oldToNew, twistOfOldWrtNew, twistOfBodyWrtBase)
end

"""
$(SIGNATURES)

Compute local coordinates ``\phi`` centered around (global) configuration vector
``q_0``, as well as their time derivatives ``\\dot{\\phi}``.
""" # TODO: refer to the method that takes a joint once it's moved to its own Joints module
function local_coordinates!(state::MechanismState, ϕ::StridedVector, ϕd::StridedVector, q0::StridedVector)
    mechanism = state.mechanism
    for vertex in non_root_vertices(state)
        jointState = edge_to_parent_data(vertex)
        qRange = configuration_range(jointState)
        vRange = velocity_range(jointState)
        ϕjoint = UnsafeVectorView(ϕ, vRange)
        ϕdjoint = UnsafeVectorView(ϕd, vRange)
        q0joint = UnsafeVectorView(q0, qRange)
        qjoint = configuration(jointState)
        vjoint = velocity(jointState)
        local_coordinates!(jointState.joint, ϕjoint, ϕdjoint, q0joint, qjoint, vjoint)
    end
end

"""
$(SIGNATURES)

Convert local coordinates ``\phi`` centered around ``q_0`` to (global)
configuration vector ``q``.
""" # TODO: refer to the method that takes a joint once it's moved to its own Joints module
function global_coordinates!(state::MechanismState, q0::StridedVector, ϕ::StridedVector)
    mechanism = state.mechanism
    for vertex in non_root_vertices(state)
        jointState = edge_to_parent_data(vertex)
        q0joint = UnsafeVectorView(q0, configuration_range(jointState))
        ϕjoint = UnsafeVectorView(ϕ, velocity_range(jointState))
        qjoint = configuration(jointState)
        global_coordinates!(jointState.joint, qjoint, q0joint, ϕjoint)
    end
end
