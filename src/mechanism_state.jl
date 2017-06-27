const BodyDict{T} = UnsafeFastDict{Graphs.vertex_index, RigidBody{T}}
const JointDict{T} = UnsafeFastDict{Graphs.edge_index, Joint{T}}

"""
$(TYPEDEF)

A `MechanismState` stores state information for an entire `Mechanism`. It
contains the joint configuration and velocity vectors ``q`` and ``v``, and
a vector of additional states ``s``. In addition, it stores cache
variables that depend on ``q`` and ``v`` and are aimed at preventing double work.

Type parameters:
* `X`: the scalar type of the ``q``, ``v``, and ``s`` vectors.
* `M`: the scalar type of the `Mechanism`
* `C`: the scalar type of the cache variables (`== promote_type(X, M)`)
"""
immutable MechanismState{X<:Number, M<:Number, C<:Number}
    mechanism::Mechanism{M}
    constraint_jacobian_structure::Vector{Tuple{Joint{M}, TreePath{RigidBody{M}, Joint{M}}}}

    # configurations, velocities
    q::Vector{X}
    v::Vector{X}

    # additional state
    s::Vector{X}

    # joint-specific
    qs::JointDict{M, VectorSegment{X}}
    vs::JointDict{M, VectorSegment{X}}
    joint_transforms::JointDict{M, CacheElement{Transform3DS{C}}}
    joint_twists::JointDict{M, CacheElement{Twist{C}}}
    joint_bias_accelerations::JointDict{M, CacheElement{SpatialAcceleration{C}}}
    motion_subspaces::JointDict{M, CacheElement{MotionSubspace{C}}}
    motion_subspaces_in_world::JointDict{M, CacheElement{MotionSubspace{C}}} # TODO: should this be here?

    # body-specific
    transforms_to_world::BodyDict{M, CacheElement{Transform3DS{C}}}
    twists_wrt_world::BodyDict{M, CacheElement{Twist{C}}}
    bias_accelerations_wrt_world::BodyDict{M, CacheElement{SpatialAcceleration{C}}}
    inertias::BodyDict{M, CacheElement{SpatialInertia{C}}}
    crb_inertias::BodyDict{M, CacheElement{SpatialInertia{C}}}
    contact_states::BodyDict{M, Vector{Vector{DefaultSoftContactState{X}}}} # TODO: consider moving to separate type

    function (::Type{MechanismState{X, M, C}}){X<:Number, M<:Number, C<:Number}(mechanism::Mechanism{M})
        nb = num_bodies(mechanism)

        q = Vector{X}(num_positions(mechanism))
        v = zeros(X, num_velocities(mechanism))
        s = zeros(X, num_additional_states(mechanism))

        qstart = 1
        qs = JointDict{M, VectorSegment{X}}(joint => begin
            qjoint = view(q, qstart : qstart + num_positions(joint) - 1)
            zero_configuration!(joint, qjoint)
            qstart += num_positions(joint)
            qjoint
        end for joint in tree_joints(mechanism))

        vstart = 1
        vs = JointDict{M, VectorSegment{X}}(joint => begin
            vjoint = view(v, vstart : vstart + num_velocities(joint) - 1)
            vstart += num_velocities(joint)
            vjoint
        end for joint in tree_joints(mechanism))

        # joint-specific
        joint_transforms = JointDict{M}(j => CacheElement{Transform3DS{C}}() for j in tree_joints(mechanism))
        joint_twists = JointDict{M}(j => CacheElement{Twist{C}}() for j in tree_joints(mechanism))
        joint_bias_accelerations = JointDict{M}(j => CacheElement{SpatialAcceleration{C}}() for j in tree_joints(mechanism))
        motion_subspaces = JointDict{M}(j => CacheElement{MotionSubspace{C}}() for j in tree_joints(mechanism))
        motion_subspaces_in_world = JointDict{M}(j => CacheElement{MotionSubspace{C}}() for j in tree_joints(mechanism))

        # body-specific
        transforms_to_world = BodyDict{M}(b => CacheElement{Transform3DS{C}}() for b in bodies(mechanism))
        twists_wrt_world = BodyDict{M}(b => CacheElement{Twist{C}}() for b in bodies(mechanism))
        bias_accelerations_wrt_world = BodyDict{M}(b => CacheElement{SpatialAcceleration{C}}() for b in bodies(mechanism))
        inertias = BodyDict{M}(b => CacheElement{SpatialInertia{C}}() for b in bodies(mechanism))
        crb_inertias = BodyDict{M}(b => CacheElement{SpatialInertia{C}}() for b in bodies(mechanism))
        contact_states = BodyDict{M, Vector{Vector{DefaultSoftContactState{C}}}}(b => Vector{Vector{DefaultSoftContactState{C}}}() for b in bodies(mechanism))
        startind = 1
        for body in bodies(mechanism), point in contact_points(body)
            model = contact_model(point)
            n = num_states(model)
            push!(contact_states[body], collect(begin
                s_part = view(s, startind : startind + n - 1)
                contact_state = SoftContactState(model, s_part, root_frame(mechanism))
                startind += n
                contact_state
            end for j = 1 : length(mechanism.environment)))
        end

        # Set root-body related cache elements once and for all.
        root = root_body(mechanism)
        rootframe = root_frame(mechanism)
        update!(transforms_to_world[root], eye(Transform3DS{C}, rootframe))
        update!(twists_wrt_world[root], zero(Twist{C}, rootframe, rootframe, rootframe))
        update!(bias_accelerations_wrt_world[root], zero(SpatialAcceleration{C}, rootframe, rootframe, rootframe))

        m = mechanism
        constraint_jacobian_structure = [(j, path(m, predecessor(j, m), successor(j, m))) for j in non_tree_joints(m)]

        new{X, M, C}(mechanism, constraint_jacobian_structure,
            q, v, s, qs, vs,
            joint_transforms, joint_twists, joint_bias_accelerations, motion_subspaces, motion_subspaces_in_world,
            transforms_to_world, twists_wrt_world, bias_accelerations_wrt_world, inertias, crb_inertias,
            contact_states)
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

Return the length of the vector of additional states ``s`` (currently used
for stateful contact models).
"""
num_additional_states(state::MechanismState) = length(state.s)


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
configuration(state::MechanismState, joint::Joint) = state.qs[joint]

"""
$(SIGNATURES)

Return the part of the velocity vector ``v`` associated with `joint`.
"""
velocity(state::MechanismState, joint::Joint) = state.vs[joint]

"""
$(SIGNATURES)

Invalidate all cache variables.
"""
function setdirty!(state::MechanismState)
    mechanism = state.mechanism
    for joint in tree_joints(mechanism)
        setdirty!(state.joint_transforms[joint])
        setdirty!(state.joint_twists[joint])
        setdirty!(state.joint_bias_accelerations[joint])
        setdirty!(state.motion_subspaces[joint])
        setdirty!(state.motion_subspaces_in_world[joint])
    end

    for body in bodies(mechanism)
        if !isroot(body, mechanism)
            setdirty!(state.transforms_to_world[body])
            setdirty!(state.twists_wrt_world[body])
            setdirty!(state.bias_accelerations_wrt_world[body])
            setdirty!(state.inertias[body])
            setdirty!(state.crb_inertias[body])
        end
    end
end

"""
$(SIGNATURES)

Reset all contact state variables.
"""
function reset_contact_state!(state::MechanismState)
    for states_for_body in values(state.contact_states)
        for states_for_point in states_for_body
            for contact_state in states_for_point
                reset!(contact_state)
            end
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
    reset_contact_state!(state)
    setdirty!(state)
end

"""
$(SIGNATURES)

Zero the velocity vector ``v``. Invalidates cache variables.
"""
function zero_velocity!(state::MechanismState)
    X = eltype(state.v)
    fill!(state.v,  zero(X))
    reset_contact_state!(state)
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
    reset_contact_state!(state)
    setdirty!(state)
end

"""
$(SIGNATURES)

Randomize the velocity vector ``v``.
Invalidates cache variables.
"""
function rand_velocity!(state::MechanismState)
    rand!(state.v)
    reset_contact_state!(state)
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

"""
$(SIGNATURES)

Return the vector of additional states ``s``.
"""
additional_state(state::MechanismState) = state.s

state_vector(state::MechanismState) = [configuration(state); velocity(state); additional_state(state)]

for fun in (:num_velocities, :num_positions)
    @eval function $fun{T}(path::TreePath{RigidBody{T}, Joint{T}})
        mapreduce(it -> $fun(first(it)), +, 0, path)
    end
end

function set_path_vector!{X, M, C}(ret::AbstractVector, state::MechanismState{X, M, C}, path::TreePath, fun)
    setvectorpart! = (out, part, startind) -> begin
        n = length(part)
        n > 0 && copy!(out, startind, part, 1, n)
        startind + n
    end
    startind = 1
    for (joint, direction) in path
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
    reset_contact_state!(state)
    setdirty!(state)
end

"""
$(SIGNATURES)

Set the part of the velocity vector associated with `joint`.
Invalidates cache variables.
"""
function set_velocity!(state::MechanismState, joint::Joint, v::AbstractVector)
    velocity(state, joint)[:] = v
    reset_contact_state!(state)
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

"""
$(SIGNATURES)

Set the vector of additional states ``s``.
"""
function set_additional_state!(state::MechanismState, s::AbstractVector)
    copy!(state.s, s)
    # note: setdirty! is currently not needed because no cache variables depend on s
end

function set!(state::MechanismState, x::AbstractVector)
    nq = num_positions(state)
    nv = num_velocities(state)
    ns = num_additional_states(state)
    length(x) == nq + nv + ns || error("wrong size")
    start = 1
    @inbounds copy!(state.q, 1, x, 1, nq)
    @inbounds copy!(state.v, 1, x, start += nq, nv)
    @inbounds copy!(state.s, 1, x, start += nv, ns)
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
    @cache_element_get!(state.joint_transforms[joint], joint_transform(joint, state.qs[joint]))
end

"""
$(SIGNATURES)

Return the joint twist for the given joint, i.e. the twist of
`frame_after(joint)` with respect to `frame_before(joint)`, expressed in the
root frame of the mechanism.
"""
function twist(state::MechanismState, joint::Joint)
    @cache_element_get!(state.joint_twists[joint], joint_twist(joint, state.qs[joint], state.vs[joint]))
end

"""
$(SIGNATURES)

Return the bias acceleration across the given joint, i.e. the spatial acceleration
of `frame_after(joint)` with respect to `frame_before(joint)`, expressed in the
root frame of the mechanism when all joint accelerations are zero.
"""
function bias_acceleration(state::MechanismState, joint::Joint)
    @cache_element_get!(state.joint_bias_accelerations[joint], bias_acceleration(joint, state.qs[joint], state.vs[joint]))
end

"""
$(SIGNATURES)

Return the motion subspace of the given joint expressed in `frame_after(joint)`.
"""
function motion_subspace(state::MechanismState, joint::Joint)
    @cache_element_get!(state.motion_subspaces[joint], motion_subspace(joint, state.qs[joint]))
end

"""
$(SIGNATURES)

Return the motion subspace of the given joint expressed in the root frame of
the mechanism.
"""
function motion_subspace_in_world(state::MechanismState, joint::Joint)
    @cache_element_get!(state.motion_subspaces_in_world[joint], begin
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
    @cache_element_get!(state.transforms_to_world[body], begin
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
    @cache_element_get!(state.twists_wrt_world[body], begin
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
    @cache_element_get!(state.bias_accelerations_wrt_world[body], begin
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
    @cache_element_get!(state.inertias[body], begin
        transform(spatial_inertia(body), transform_to_root(state, body))
    end)
end

"""
$(SIGNATURES)

Return the composite rigid body inertia `body` expressed in the root frame of the
mechanism.
"""
function crb_inertia(state::MechanismState, body::RigidBody)
    @cache_element_get!(state.crb_inertias[body], begin
        ret = spatial_inertia(state, body)
        for joint in joints_to_children(body, state.mechanism)
            child = successor(joint, state.mechanism)
            ret += crb_inertia(state, child)
        end
        ret
    end)
end

contact_states(state::MechanismState, body::RigidBody) = state.contact_states[body]

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
        q̇ = fastview(out, configuration_range(state, joint))
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

@inline function non_root_body_sum(state::MechanismState, start, fun, body_itr)
    ret = start
    for body in body_itr
        if !isroot(body, state.mechanism)
            ret += fun(state, body)
        end
    end
    ret
end

function momentum(state::MechanismState, body_itr)
    T = cache_eltype(state)
    non_root_body_sum(state, zero(Momentum{T}, root_frame(state.mechanism)), momentum, body_itr)
end

function momentum_rate_bias(state::MechanismState, body_itr)
    T = cache_eltype(state)
    non_root_body_sum(state, zero(Wrench{T}, root_frame(state.mechanism)), momentum_rate_bias, body_itr)
end

function kinetic_energy(state::MechanismState, body_itr)
    T = cache_eltype(state)
    non_root_body_sum(state, zero(T), kinetic_energy, body_itr)
end

for fun in (:momentum, :momentum_rate_bias, :kinetic_energy)
    @eval $fun(state::MechanismState) = $fun(state, bodies(state.mechanism))
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
        ϕjoint = fastview(ϕ, vrange)
        ϕdjoint = fastview(ϕd, vrange)
        q0joint = fastview(q0, qrange)
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
        q0joint = fastview(q0, configuration_range(state, joint))
        ϕjoint = fastview(ϕ, velocity_range(state, joint))
        qjoint = configuration(state, joint)
        global_coordinates!(joint, qjoint, q0joint, ϕjoint)
    end
end
