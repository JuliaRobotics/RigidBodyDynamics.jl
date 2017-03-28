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
    nontreejoints::Vector{Joint{M}}
    constraint_jacobian_structure::SparseMatrixCSC{Int64,Int64} # TODO: consider just using a Vector{Vector{Pair{Int64, Int64}}}

    # state vector
    q::Vector{X}
    v::Vector{X}

    # joint-specific
    qs::Vector{VectorSegment{X}}
    vs::Vector{VectorSegment{X}}
    joint_transforms::Vector{CacheElement{Transform3D{C}}}
    joint_twists::Vector{CacheElement{Twist{C}}}
    joint_bias_accelerations::Vector{CacheElement{SpatialAcceleration{C}}}
    motion_subspaces::Vector{CacheElement{MotionSubspace{C}}}
    motion_subspaces_in_world::Vector{CacheElement{MotionSubspace{C}}} # TODO: should this be here?

    # body-specific
    transforms_to_world::Vector{CacheElement{Transform3D{C}}}
    twists_wrt_world::Vector{CacheElement{Twist{C}}}
    bias_accelerations_wrt_world::Vector{CacheElement{SpatialAcceleration{C}}}
    inertias::Vector{CacheElement{SpatialInertia{C}}}
    crb_inertias::Vector{CacheElement{SpatialInertia{C}}}

    function (::Type{MechanismState{X, M, C}}){X<:Number, M<:Number, C<:Number}(mechanism::Mechanism{M})
        nb = num_bodies(mechanism)

        q = Vector{X}(num_positions(mechanism))
        v = zeros(X, num_velocities(mechanism))

        qs = Vector{VectorSegment{X}}(nb)
        vs = Vector{VectorSegment{X}}(nb)
        qstart = 1
        vstart = 1
        for joint in tree_joints(mechanism)
            qjoint = view(q, qstart : qstart + num_positions(joint) - 1)
            qs[tree_index(joint, mechanism)] = qjoint
            zero_configuration!(joint, qjoint)
            qstart += num_positions(joint)

            vjoint = view(v, vstart : vstart + num_velocities(joint) - 1)
            vs[tree_index(joint, mechanism)] = vjoint
            vstart += num_velocities(joint)
        end

        # joint-specific
        joint_transforms = [CacheElement{Transform3D{C}}() for i = 1 : nb]
        joint_twists = [CacheElement{Twist{C}}() for i = 1 : nb]
        joint_bias_accelerations = [CacheElement{SpatialAcceleration{C}}() for i = 1 : nb]
        motion_subspaces = [CacheElement{MotionSubspace{C}}() for i = 1 : nb]
        motion_subspaces_in_world = [CacheElement{MotionSubspace{C}}() for i = 1 : nb]

        # body-specific
        transforms_to_world = [CacheElement{Transform3D{C}}() for i = 1 : nb]
        twists_wrt_world = [CacheElement{Twist{C}}() for i = 1 : nb]
        bias_accelerations_wrt_world = [CacheElement{SpatialAcceleration{C}}() for i = 1 : nb]
        inertias = [CacheElement{SpatialInertia{C}}() for i = 1 : nb]
        crb_inertias = [CacheElement{SpatialInertia{C}}() for i = 1 : nb]

        # Set root-body related cache elements once and for all.
        rootindex = vertex_index(root_body(mechanism))
        rootframe = root_frame(mechanism)
        update!(transforms_to_world[rootindex], Transform3D(C, rootframe))
        update!(twists_wrt_world[rootindex], zero(Twist{C}, rootframe, rootframe, rootframe))
        update!(bias_accelerations_wrt_world[rootindex], zero(SpatialAcceleration{C}, rootframe, rootframe, rootframe))

        new{X, M, C}(mechanism, non_tree_joints(mechanism), constraint_jacobian_structure(mechanism),
            q, v, qs, vs,
            joint_transforms, joint_twists, joint_bias_accelerations, motion_subspaces, motion_subspaces_in_world,
            transforms_to_world, twists_wrt_world, bias_accelerations_wrt_world, inertias, crb_inertias)
    end
end
MechanismState{X, M}(::Type{X}, mechanism::Mechanism{M}) = MechanismState{X, M, promote_type(X, M)}(mechanism)

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

"""
$(SIGNATURES)

Return the `Joint`s that are not part of the underlying `Mechanism`'s spanning tree as an iterable collection.
"""
non_tree_joints(state::MechanismState) = state.nontreejoints

state_vector_eltype{X, M, C}(state::MechanismState{X, M, C}) = X
mechanism_eltype{X, M, C}(state::MechanismState{X, M, C}) = M
cache_eltype{X, M, C}(state::MechanismState{X, M, C}) = C

"""
$(SIGNATURES)

Return the part of the configuration vector ``q`` associated with `joint`.
"""
configuration(state::MechanismState, joint::Joint) = state.qs[tree_index(joint, state.mechanism)]

"""
$(SIGNATURES)

Return the part of the velocity vector ``v`` associated with `joint`.
"""
velocity(state::MechanismState, joint::Joint) = state.vs[tree_index(joint, state.mechanism)]

"""
$(SIGNATURES)

Invalidate all cache variables.
"""
function setdirty!(state::MechanismState)
    mechanism = state.mechanism
    for joint in tree_joints(mechanism)
        index = tree_index(joint, mechanism)
        setdirty!(state.joint_transforms[index])
        setdirty!(state.joint_twists[index])
        setdirty!(state.joint_bias_accelerations[index])
        setdirty!(state.motion_subspaces[index])
        setdirty!(state.motion_subspaces_in_world[index])
    end

    for body in bodies(mechanism)
        if !isroot(body, mechanism)
            index = vertex_index(body)
            setdirty!(state.transforms_to_world[index])
            setdirty!(state.twists_wrt_world[index])
            setdirty!(state.bias_accelerations_wrt_world[index])
            setdirty!(state.inertias[index])
            setdirty!(state.crb_inertias[index])
        end
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
    for joint in tree_joints(state.mechanism)
        zero_configuration!(joint, configuration(state, joint))
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
zero!(state::MechanismState) = (zero_configuration!(state); zero_velocity!(state))

"""
$(SIGNATURES)

Randomize the configuration vector ``q``. The distribution depends on
the particular joint types present in the associated `Mechanism`. The resulting
``q`` is guaranteed to be on the `Mechanism`'s configuration manifold.
Invalidates cache variables.
"""
function rand_configuration!(state::MechanismState)
    for joint in tree_joints(state.mechanism)
        rand_configuration!(joint, configuration(state, joint))
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

for fun in (:num_velocities, :num_positions)
    @eval function $fun{T}(path::TreePath{RigidBody{T}, Joint{T}})
        mapreduce($fun, +, 0, path.source_to_lca) + mapreduce($fun, +, 0, path.target_to_lca)
    end
end

function set_path_vector!{X, M, C}(ret::AbstractVector, state::MechanismState{X, M, C}, path::TreePath, fun)
    setvectorpart! = (out, part, startind) -> begin
        n = length(part)
        n > 0 && copy!(out, startind, part, 1, n)
        startind + n
    end
    startind = 1
    for joint in path.source_to_lca
        startind = setvectorpart!(ret, fun(state, joint), startind)
    end
    for i = length(path.target_to_lca) : -1 : 1
        joint = path.target_to_lca[i]
        startind = setvectorpart!(ret, fun(state, joint), startind)
    end
    ret
end

"""
$(SIGNATURES)

Return the part of the `Mechanism`'s configuration vector ``q`` associated with
the joints along `path`.
"""
function configuration{X, M, C}(state::MechanismState{X, M, C}, path::TreePath{RigidBody{M}, Joint{M}})
    set_path_vector!(Vector{X}(num_positions(path)), state, path, configuration)
end

"""
$(SIGNATURES)

Return the part of the `Mechanism`'s velocity vector ``v`` associated with
the joints along `path`.
"""
function velocity{X, M, C}(state::MechanismState{X, M, C}, path::TreePath{RigidBody{M}, Joint{M}})
    set_path_vector!(Vector{X}(num_velocities(path)), state, path, velocity)
end

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


configuration_range(state::MechanismState, joint::Joint) = first(parentindexes(configuration(state, joint)))
velocity_range(state::MechanismState, joint::Joint) = first(parentindexes(velocity(state, joint)))

"""
$(SIGNATURES)

Return the joint transform for the given joint, i.e. the transform from
`frame_after(joint)` to `frame_before(joint)`.
"""
function transform(state::MechanismState, joint::Joint)
    index = tree_index(joint, state.mechanism)
    @cache_element_get!(state.joint_transforms[index], begin
        q = state.qs[index]
        joint_transform(joint, q)
    end)
end

"""
$(SIGNATURES)

Return the joint twist for the given joint, i.e. the twist of
`frame_after(joint)` with respect to `frame_before(joint)`, expressed in the
root frame of the mechanism.
"""
function twist(state::MechanismState, joint::Joint)
    index = tree_index(joint, state.mechanism)
    @cache_element_get!(state.joint_twists[index], begin
        q = state.qs[index]
        v = state.vs[index]
        joint_twist(joint, q, v)
    end)
end

"""
$(SIGNATURES)

Return the bias acceleration across the given joint, i.e. the spatial acceleration
of `frame_after(joint)` with respect to `frame_before(joint)`, expressed in the
root frame of the mechanism when all joint accelerations are zero.
"""
function bias_acceleration(state::MechanismState, joint::Joint)
    index = tree_index(joint, state.mechanism)
    @cache_element_get!(state.joint_bias_accelerations[index], begin
        q = state.qs[index]
        v = state.vs[index]
        bias_acceleration(joint, q, v)
    end)
end

"""
$(SIGNATURES)

Return the motion subspace of the given joint expressed in `frame_after(joint)`.
"""
function motion_subspace(state::MechanismState, joint::Joint)
    index = tree_index(joint, state.mechanism)
    @cache_element_get!(state.motion_subspaces[index], begin
        q = state.qs[index]
        motion_subspace(joint, q)
    end)
end

"""
$(SIGNATURES)

Return the motion subspace of the given joint expressed in the root frame of
the mechanism.
"""
function motion_subspace_in_world(state::MechanismState, joint::Joint)
    index = tree_index(joint, state.mechanism)
    @cache_element_get!(state.motion_subspaces_in_world[index], begin
        body = successor(joint, state.mechanism)
        parentbody = predecessor(joint, state.mechanism)
        parentframe = default_frame(parentbody)
        motionsubspace = change_base(motion_subspace(state, joint), parentframe)
        transform(motionsubspace, transform_to_root(state, body))
    end)
end

"""
$(SIGNATURES)

Return the transform from `default_frame(body)` to the root frame of the
mechanism.
"""
function transform_to_root(state::MechanismState, body::RigidBody)
    index = vertex_index(body)
    @cache_element_get!(state.transforms_to_world[index], begin
        joint = joint_to_parent(body, state.mechanism)
        parentbody = predecessor(joint, state.mechanism)
        parent_to_root = transform_to_root(state, parentbody)
        before_joint_to_parent = frame_definition(parentbody, frame_before(joint)) # FIXME: slow!
        parent_to_root * before_joint_to_parent * transform(state, joint)
    end)
end

"""
$(SIGNATURES)

Return the twist of `default_frame(body)` with respect to the root frame of the
mechanism, expressed in the root frame.
"""
function twist_wrt_world(state::MechanismState, body::RigidBody)
    index = vertex_index(body)
    @cache_element_get!(state.twists_wrt_world[index], begin
        joint = joint_to_parent(body, state.mechanism)
        parentbody = predecessor(joint, state.mechanism)
        parenttwist = twist_wrt_world(state, parentbody)
        parentframe = default_frame(parentbody)
        jointtwist = change_base(twist(state, joint), parentframe) # to make frames line up
        parenttwist + transform(jointtwist, transform_to_root(state, body))
    end)
end

"""
$(SIGNATURES)

Return the bias acceleration of the given body with respect to the world,
i.e. the spatial acceleration of `default_frame(body)` with respect to the
root frame of the mechanism, expressed in the root frame, when all joint
accelerations are zero.
"""
function bias_acceleration(state::MechanismState, body::RigidBody)
    index = vertex_index(body)
    @cache_element_get!(state.bias_accelerations_wrt_world[index], begin
        joint = joint_to_parent(body, state.mechanism)
        parentbody = predecessor(joint, state.mechanism)
        parentbias = bias_acceleration(state, parentbody)
        parentframe = default_frame(parentbody)
        jointbias = change_base(bias_acceleration(state, joint), parentframe) # to make frames line up

         # TODO: awkward way of doing this:
        toroot = transform_to_root(state, body)
        twistwrtworld = transform(twist_wrt_world(state, body), inv(toroot))
        jointtwist = change_base(twist(state, joint), parentframe) # to make frames line up

        jointbias = transform(jointbias, toroot, twistwrtworld, jointtwist)
        parentbias + jointbias
    end)
end

"""
$(SIGNATURES)

Return the spatial inertia of `body` expressed in the root frame of the
mechanism.
"""
function spatial_inertia(state::MechanismState, body::RigidBody)
    index = vertex_index(body)
    @cache_element_get!(state.inertias[index], begin
        transform(spatial_inertia(body), transform_to_root(state, body))
    end)
end

"""
$(SIGNATURES)

Return the composite rigid body inertia `body` expressed in the root frame of the
mechanism.
"""
function crb_inertia(state::MechanismState, body::RigidBody)
    index = vertex_index(body)
    @cache_element_get!(state.crb_inertias[index], begin
        ret = spatial_inertia(state, body)
        for joint in joints_to_children(body, state.mechanism)
            child = successor(joint, state.mechanism)
            ret += crb_inertia(state, child)
        end
        ret
    end)
end

function newton_euler(state::MechanismState, body::RigidBody, accel::SpatialAcceleration)
    inertia = spatial_inertia(state, body)
    twist = twist_wrt_world(state, body)
    newton_euler(inertia, accel, twist)
end

momentum(state::MechanismState, body::RigidBody) = spatial_inertia(state, body) * twist_wrt_world(state, body)
momentum_rate_bias(state::MechanismState, body::RigidBody) = newton_euler(state, body, bias_acceleration(state, body))
kinetic_energy(state::MechanismState, body::RigidBody) = kinetic_energy(spatial_inertia(state, body), twist_wrt_world(state, body))

function configuration_derivative!{X}(out::AbstractVector{X}, state::MechanismState{X})
    for joint in tree_joints(state.mechanism)
        q = configuration(state, joint)
        v = velocity(state, joint)
        q̇ = UnsafeVectorView(out, configuration_range(state, joint))
        velocity_to_configuration_derivative!(joint, q̇, q, v)
    end
end

function configuration_derivative{X}(state::MechanismState{X})
    ret = Vector{X}(num_positions(state.mechanism))
    configuration_derivative!(ret, state)
    ret
end

function transform_to_root(state::MechanismState, frame::CartesianFrame3D)
    body = body_fixed_frame_to_body(state.mechanism, frame) # FIXME: expensive
    tf = transform_to_root(state, body)
    if tf.from != frame
        tf = tf * body_fixed_frame_definition(state.mechanism, frame) # TODO: consider caching
    end
    tf
end

for fun in (:momentum, :momentum_rate_bias, :kinetic_energy)
    @eval $fun(state::MechanismState) = sum($fun(state, body) for body in non_root_bodies(state.mechanism)) # TODO: allocations
    @eval $fun(state::MechanismState, body_itr) = sum($fun(state, body) for body in body_itr) # TODO: allocations
end

"""
$(SIGNATURES)

Return the homogeneous transform from `from` to `to`.
"""
function relative_transform(state::MechanismState, from::CartesianFrame3D, to::CartesianFrame3D)
    # TODO: check if this if-else is actually worth it
    rootframe = root_frame(state.mechanism)
    if to == rootframe
        return transform_to_root(state, from)
    elseif from == rootframe
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
    # TODO: check if this if-else is actually worth it
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
    for joint in tree_joints(mechanism)
        qrange = configuration_range(state, joint)
        vrange = velocity_range(state, joint)
        ϕjoint = UnsafeVectorView(ϕ, vrange)
        ϕdjoint = UnsafeVectorView(ϕd, vrange)
        q0joint = UnsafeVectorView(q0, qrange)
        qjoint = configuration(state, joint)
        vjoint = velocity(state, joint)
        local_coordinates!(joint, ϕjoint, ϕdjoint, q0joint, qjoint, vjoint)
    end
end

"""
$(SIGNATURES)

Convert local coordinates ``\phi`` centered around ``q_0`` to (global)
configuration vector ``q``.
""" # TODO: refer to the method that takes a joint once it's moved to its own Joints module
function global_coordinates!(state::MechanismState, q0::StridedVector, ϕ::StridedVector)
    mechanism = state.mechanism
    for joint in tree_joints(mechanism)
        q0joint = UnsafeVectorView(q0, configuration_range(state, joint))
        ϕjoint = UnsafeVectorView(ϕ, velocity_range(state, joint))
        qjoint = configuration(state, joint)
        global_coordinates!(joint, qjoint, q0joint, ϕjoint)
    end
end
