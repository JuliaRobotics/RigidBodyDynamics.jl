const BodyDict{V} = IndexDict{BodyID, V}
const BodyCacheDict{V} = CacheIndexDict{BodyID, V}
Base.@propagate_inbounds Base.getindex(d::AbstractIndexDict{BodyID}, key::RigidBody) = d[id(key)]
Base.@propagate_inbounds Base.setindex!(d::AbstractIndexDict{BodyID}, value, key::RigidBody) = d[id(key)] = value

const JointDict{V} = IndexDict{JointID, V}
const JointCacheDict{V} = CacheIndexDict{JointID, V}
Base.@propagate_inbounds Base.getindex(d::AbstractIndexDict{JointID}, key::Joint) = d[id(key)]
Base.@propagate_inbounds Base.setindex!(d::AbstractIndexDict{JointID}, value, key::Joint) = d[id(key)] = value

function motionsubspacetypes(JointTypes, ::Type{X}) where X
    Base.tuple_type_cons(motionsubspacetype(Base.tuple_type_head(JointTypes), X), motionsubspacetypes(Base.tuple_type_tail(JointTypes), X))
end
motionsubspacetypes(::Type{Tuple{}}, ::Type) = Tuple{}

function motionsubspacecollectiontype(::Type{TypeSortedCollection{D, N}}, ::Type{X}) where {D, N, X}
    TypeSortedCollection{vectortypes(motionsubspacetypes(eltypes(D), X)), N}
end

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
struct MechanismState{X, M, C, JointCollection, MotionSubspaceCollection}
    mechanism::Mechanism{M}
    modcount::Int
    type_sorted_tree_joints::JointCollection
    type_sorted_non_tree_joints::JointCollection
    ancestor_joint_masks::JointDict{JointDict{Bool}}
    constraint_jacobian_structure::JointDict{TreePath{RigidBody{M}, Joint{M}}} # TODO: switch to JointDict{JointDict{Vector{Bool}}}

    q::Vector{X} # configurations
    v::Vector{X} # velocities
    s::Vector{X} # additional state

    # joint-related fixed data
    jointids::Base.OneTo{JointID}
    treejointids::Base.OneTo{JointID}
    nontreejointids::UnitRange{JointID}
    qs::JointDict{VectorSegment{X}}
    vs::JointDict{VectorSegment{X}}
    predecessor_and_successor_ids::JointDict{Pair{BodyID, BodyID}}

    # joint-related cache
    joint_transforms::JointCacheDict{Transform3D{C}}
    tree_joint_transforms::VectorSegment{Transform3D{C}}
    joint_twists::JointCacheDict{Twist{C}}
    joint_bias_accelerations::JointCacheDict{SpatialAcceleration{C}}
    motion_subspaces::CacheElement{MotionSubspaceCollection} # TODO
    constraint_wrench_subspaces::JointCacheDict{WrenchSubspace{C}} # TODO: use a TSC. note: the ones corresponding to tree joints are never used (or set)

    # body-related cache
    transforms_to_root::BodyCacheDict{Transform3D{C}}
    twists_wrt_world::BodyCacheDict{Twist{C}}
    bias_accelerations_wrt_world::BodyCacheDict{SpatialAcceleration{C}}
    inertias::BodyCacheDict{SpatialInertia{C}}
    crb_inertias::BodyCacheDict{SpatialInertia{C}}
    contact_states::BodyCacheDict{Vector{Vector{DefaultSoftContactState{X}}}} # TODO: consider moving to separate type

    function MechanismState{X, M, C, JointCollection, MotionSubspaceCollection}(m::Mechanism{M}, q::Vector{X}, v::Vector{X}, s::Vector{X}) where {X, M, C, JointCollection, MotionSubspaceCollection}
        @assert length(q) == num_positions(m)
        @assert length(v) == num_velocities(m)
        @assert length(s) == num_additional_states(m)

        canonicalize_graph!(m)
        type_sorted_tree_joints = JointCollection(tree_joints(m))
        type_sorted_non_tree_joints = JointCollection(non_tree_joints(m))
        ancestor_joint_mask = joint -> JointDict{Bool}(id(j) => j ∈ path(m, successor(joint, m), root_body(m)) for j in tree_joints(m))
        ancestor_joint_masks = JointDict{JointDict{Bool}}(id(j) => ancestor_joint_mask(j) for j in tree_joints(m))

        # joint-specific
        lastjointid = isempty(joints(m)) ? JointID(0) : id(last(joints(m)))
        jointids = Base.OneTo(lastjointid)
        lasttreejointid = isempty(tree_joints(m)) ? JointID(0) : id(last(tree_joints(m)))
        treejointids = Base.OneTo(lasttreejointid)
        nontreejointids = lasttreejointid + 1 : lastjointid
        constraint_jacobian_structure = JointDict(id(j) => path(m, predecessor(j, m), successor(j, m))::TreePath{RigidBody{M}, Joint{M}} for j in joints(m))

        qstart, vstart = 1, 1
        qs = JointDict{VectorSegment{X}}(id(j) => view(q, qstart : (qstart += num_positions(j)) - 1) for j in tree_joints(m))
        vs = JointDict{VectorSegment{X}}(id(j) => view(v, vstart : (vstart += num_velocities(j)) - 1) for j in tree_joints(m))
        predecessor_and_successor_ids = JointDict{Pair{BodyID, BodyID}}(id(j) => (id(predecessor(j, m)) => id(successor(j, m))) for j in joints(m))

        joint_transforms = JointCacheDict{Transform3D{C}}(length(joints(m)))
        tree_joint_transforms = view(values(joint_transforms), 1 : length(tree_joints(m)))
        joint_twists = JointCacheDict{Twist{C}}(length(tree_joints(m)))
        joint_bias_accelerations = JointCacheDict{SpatialAcceleration{C}}(length(tree_joints(m)))
        motion_subspaces = CacheElement(MotionSubspaceCollection(indices(type_sorted_tree_joints)))
        constraint_wrench_subspaces = JointCacheDict{WrenchSubspace{C}}(length(joints(m)))

        # body-specific
        nbodies = length(bodies(m))
        transforms_to_root = BodyCacheDict{Transform3D{C}}(nbodies)
        twists_wrt_world = BodyCacheDict{Twist{C}}(nbodies)
        bias_accelerations_wrt_world = BodyCacheDict{SpatialAcceleration{C}}(nbodies)
        inertias = BodyCacheDict{SpatialInertia{C}}(nbodies)
        crb_inertias = BodyCacheDict{SpatialInertia{C}}(nbodies)
        contact_states = BodyCacheDict(id(b) => Vector{Vector{DefaultSoftContactState{X}}}() for b in bodies(m))
        startind = 1
        for body in bodies(m), point in contact_points(body)
            model = contact_model(point)
            n = num_states(model)
            push!(contact_states[body], collect(begin
                s_part = view(s, startind : startind + n - 1)
                contact_state = SoftContactState(model, s_part, root_frame(m))
                startind += n
                contact_state
            end for j = 1 : length(m.environment)))
        end

        # initialize state-independent cache data for root bodies:
        root = root_body(m)
        rootframe = default_frame(root)
        transforms_to_root[root] = eye(Transform3D{C}, rootframe)
        twists_wrt_world[root] = zero(Twist{C}, rootframe, rootframe, rootframe)
        bias_accelerations_wrt_world[root] = zero(SpatialAcceleration{C}, rootframe, rootframe, rootframe)
        inertias[root] = zero(SpatialInertia{C}, rootframe)

        new{X, M, C, JointCollection, MotionSubspaceCollection}(
            m, modcount(m), type_sorted_tree_joints, type_sorted_non_tree_joints, ancestor_joint_masks,
            constraint_jacobian_structure, q, v, s, jointids, treejointids, nontreejointids, qs, vs, predecessor_and_successor_ids,
            joint_transforms, tree_joint_transforms, joint_twists, joint_bias_accelerations, motion_subspaces, constraint_wrench_subspaces,
            transforms_to_root, twists_wrt_world, bias_accelerations_wrt_world, inertias, crb_inertias,
            contact_states)
    end

    function MechanismState{X, M, C, JointCollection}(mechanism::Mechanism{M}, q::Vector{X}, v::Vector{X}, s::Vector{X}) where {X, M, C, JointCollection}
        MotionSubspaceCollection = motionsubspacecollectiontype(JointCollection, X)
        MechanismState{X, M, C, JointCollection, MotionSubspaceCollection}(mechanism, q, v, s)
    end

    function MechanismState{X, M, C}(mechanism::Mechanism{M}, q::Vector{X}, v::Vector{X}, s::Vector{X}) where {X, M, C}
        JointCollection = typeof(TypeSortedCollection(joints(mechanism)))
        MechanismState{X, M, C, JointCollection}(mechanism, q, v, s)
    end

    function MechanismState{X, M}(mechanism::Mechanism{M}, q::Vector{X}, v::Vector{X}, s::Vector{X}) where {X, M}
        C = promote_type(X, M)
        MechanismState{X, M, C}(mechanism, q, v, s)
    end

    function MechanismState{X}(mechanism::Mechanism{M}) where {X, M}
        q = Vector{X}(num_positions(mechanism))
        v = Vector{X}(num_velocities(mechanism))
        s = Vector{X}(num_additional_states(mechanism))
        state = MechanismState{X, M}(mechanism, q, v, s)
        zero!(state)
        state
    end
end

MechanismState(mechanism::Mechanism{M}, q::Vector{X}, v::Vector{X}) where {X, M} = MechanismState{X, M}(mechanism, q, v, zeros(X, num_additional_states(mechanism)))
MechanismState(mechanism::Mechanism{M}) where {M} = MechanismState{M}(mechanism)

Base.show(io::IO, ::MechanismState{X, M, C}) where {X, M, C} = print(io, "MechanismState{$X, $M, $C, …}(…)")

modcount(state::MechanismState) = state.modcount
Base.@propagate_inbounds predsucc(id::JointID, state::MechanismState) = state.predecessor_and_successor_ids[id]
Base.@propagate_inbounds successorid(id::JointID, state::MechanismState) = last(state.predecessor_and_successor_ids[id])

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

state_vector_eltype(state::MechanismState{X, M, C}) where {X, M, C} = X
mechanism_eltype(state::MechanismState{X, M, C}) where {X, M, C} = M
cache_eltype(state::MechanismState{X, M, C}) where {X, M, C} = C

"""
$(SIGNATURES)

Return the part of the configuration vector ``q`` associated with `joint`.
"""
configuration(state::MechanismState, joint::Union{<:Joint, JointID}) = state.qs[joint]

"""
$(SIGNATURES)

Return the part of the velocity vector ``v`` associated with `joint`.
"""
velocity(state::MechanismState, joint::Union{<:Joint, JointID}) = state.vs[joint]

"""
$(SIGNATURES)

Invalidate all cache variables.
"""
function setdirty!(state::MechanismState)
    CustomCollections.setdirty!(state.joint_transforms)
    CustomCollections.setdirty!(state.joint_twists)
    CustomCollections.setdirty!(state.joint_bias_accelerations)
    CustomCollections.setdirty!(state.motion_subspaces)
    CustomCollections.setdirty!(state.constraint_wrench_subspaces)
    CustomCollections.setdirty!(state.transforms_to_root)
    CustomCollections.setdirty!(state.twists_wrt_world)
    CustomCollections.setdirty!(state.bias_accelerations_wrt_world)
    CustomCollections.setdirty!(state.inertias)
    CustomCollections.setdirty!(state.crb_inertias)
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
    foreach((joint, q) -> zero_configuration!(q, joint), state.type_sorted_tree_joints, values(state.qs))
    reset_contact_state!(state)
    setdirty!(state)
end

"""
$(SIGNATURES)

Zero the velocity vector ``v``. Invalidates cache variables.
"""
function zero_velocity!(state::MechanismState)
    state.v[:] = 0
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
    foreach((joint, q) -> rand_configuration!(q, joint), state.type_sorted_tree_joints, values(state.qs))
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

"""
$(SIGNATURES)

Return the velocity vector ``v``.

Note that this function returns a read-write reference to a field in `state`.
The user is responsible for calling [`setdirty!`](@ref) after modifying this
vector to ensure that dependent cache variables are invalidated.
"""
velocity(state::MechanismState) = state.v

"""
$(SIGNATURES)

Return the vector of additional states ``s``.
"""
additional_state(state::MechanismState) = state.s

state_vector(state::MechanismState) = [configuration(state); velocity(state); additional_state(state)]

for fun in (:num_velocities, :num_positions)
    @eval function $fun(p::TreePath{RigidBody{T}, <:Joint{T}} where {T})
        mapreduce($fun, +, 0, p)
    end
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

"""
$(SIGNATURES)

Project the configuration vector ``q`` onto the configuration manifold.

For example:
* for a part of ``q`` corresponding to a revolute joint, this method is a no-op;
* for a part of ``q`` corresponding to a spherical joint that uses a unit quaternion
to parameterize the orientation of its successor with respect to its predecessor,
`normalize_configuration!` will renormalize the quaternion so that it is indeed
of unit length.

!!! warning
This method does not ensure that the configuration or velocity satisfy joint
configuration or velocity limits/bounds.
"""
function normalize_configuration!(state::MechanismState)
    foreach(state.type_sorted_tree_joints, values(state.qs)) do joint, qjoint
        normalize_configuration!(qjoint, joint)
    end
end

"""
$(SIGNATURES)

Return the range of indices into the joint configuration vector ``q`` corresponding to joint `joint`.
"""
configuration_range(state::MechanismState, joint::Union{<:Joint, JointID}) = first(parentindexes(configuration(state, joint)))

"""
$(SIGNATURES)

Return the range of indices into the joint velocity vector ``v`` corresponding to joint `joint`.
"""
velocity_range(state::MechanismState, joint::Union{<:Joint, JointID}) = first(parentindexes(velocity(state, joint)))


## Accessor functions for cached variables

"""
$(SIGNATURES)

Return the joint transform for the given joint, i.e. the transform from
`frame_after(joint)` to `frame_before(joint)`.
"""
@inline function joint_transform(state::MechanismState, joint::Union{<:Joint, JointID})
    update_transforms!(state)
    state.joint_transforms[joint]
end

"""
$(SIGNATURES)
Return the joint twist for the given joint, i.e. the twist of
`frame_after(joint)` with respect to `frame_before(joint)`, expressed in the
root frame of the mechanism.
"""
@inline function twist(state::MechanismState, joint::Union{<:Joint, JointID})
    update_joint_twists!(state)
    state.joint_twists[joint]
end

"""
$(SIGNATURES)

Return the bias acceleration across the given joint, i.e. the spatial acceleration
of `frame_after(joint)` with respect to `frame_before(joint)`, expressed in the
root frame of the mechanism when all joint accelerations are zero.
"""
@inline function bias_acceleration(state::MechanismState, joint::Union{<:Joint, JointID})
    update_joint_bias_accelerations!(state)
    state.joint_bias_accelerations[joint]
end

"""
$(SIGNATURES)

Return the constraint wrench subspace of the given joint expressed in the frame after the joint.
"""
@inline function constraint_wrench_subspace(state::MechanismState, joint::Joint{T}) where {T}
    update_constraint_wrench_subspaces!(state)
    state.constraint_wrench_subspaces[joint]
end

@inline function constraint_wrench_subspace(state::MechanismState{X, M, C}, joint::Joint{M, JT}) where {X, M, C, JT}
    update_constraint_wrench_subspaces!(state)
    T = state.constraint_wrench_subspaces[joint]
    N = num_constraints(JT)
    WrenchMatrix(T.frame, SMatrix{3, N, C}(angular(T)), SMatrix{3, N, C}(linear(T)))
end

Base.@deprecate transform(state::MechanismState, joint::Joint) joint_transform(state, joint) # TODO: undeprecate?


"""
$(SIGNATURES)

Return the transform from `default_frame(body)` to the root frame of the
mechanism.
"""
@inline function transform_to_root(state::MechanismState, body::Union{<:RigidBody, BodyID})
    update_transforms!(state)
    state.transforms_to_root[body]
end

"""
$(SIGNATURES)

Return the twist of `default_frame(body)` with respect to the root frame of the
mechanism, expressed in the root frame.
"""
@inline function twist_wrt_world(state::MechanismState, body::Union{<:RigidBody, BodyID})
    update_twists_wrt_world!(state)
    state.twists_wrt_world[body]
end

"""
$(SIGNATURES)

Return the bias acceleration of the given body with respect to the world,
i.e. the spatial acceleration of `default_frame(body)` with respect to the
root frame of the mechanism, expressed in the root frame, when all joint
accelerations are zero.
"""
@inline function bias_acceleration(state::MechanismState, body::Union{<:RigidBody, BodyID})
    update_bias_accelerations_wrt_world!(state)
    state.bias_accelerations_wrt_world[body]
end

"""
$(SIGNATURES)

Return the spatial inertia of `body` expressed in the root frame of the
mechanism.
"""
@inline function spatial_inertia(state::MechanismState, body::Union{<:RigidBody, BodyID})
    update_spatial_inertias!(state)
    state.inertias[body]
end

"""
$(SIGNATURES)

Return the composite rigid body inertia `body` expressed in the root frame of the
mechanism.
"""
@inline function crb_inertia(state::MechanismState, body::Union{<:RigidBody, BodyID})
    update_crb_inertias!(state)
    state.crb_inertias[body]
end


# Cache variable update functions
@inline update_transforms!(state::MechanismState) = isdirty(state.transforms_to_root) && _update_transforms!(state)
@noinline function _update_transforms!(state::MechanismState)
    @modcountcheck state state.mechanism

    # update tree joint transforms
    map!(joint_transform, state.tree_joint_transforms, state.type_sorted_tree_joints, values(state.qs))

    # update transforms to root
    transforms_to_root = state.transforms_to_root
    for joint in tree_joints(state.mechanism)
        jointid = id(joint)
        parentid, bodyid = predsucc(jointid, state)
        transforms_to_root[bodyid] = transforms_to_root[parentid] * joint_to_predecessor(joint) * state.joint_transforms[jointid]
    end
    state.transforms_to_root.dirty = false

    # update non-tree joint transforms
    if length(state.type_sorted_non_tree_joints) > 0
        foreach_with_extra_args(state.joint_transforms, state, state.type_sorted_non_tree_joints) do joint_transforms, state, joint # TODO: use closure once it's fast
            pred = predecessor(joint, state.mechanism)
            succ = successor(joint, state.mechanism)
            before_to_root = transform_to_root(state, pred) * joint_to_predecessor(joint)
            after_to_root = transform_to_root(state, succ) * joint_to_successor(joint)
            joint_transforms[joint] = inv(before_to_root) * after_to_root
        end
    end
    state.joint_transforms.dirty = false
    nothing
end

@inline update_joint_twists!(state::MechanismState) = isdirty(state.joint_twists) && _update_joint_twists!(state)
@noinline function _update_joint_twists!(state::MechanismState)
    map!(joint_twist, values(state.joint_twists), state.type_sorted_tree_joints, values(state.qs), values(state.vs))
    state.joint_twists.dirty = false
    nothing
end

@inline update_joint_bias_accelerations!(state::MechanismState) = isdirty(state.joint_bias_accelerations) && _update_joint_bias_accelerations!(state)
@noinline function _update_joint_bias_accelerations!(state::MechanismState)
    map!(bias_acceleration, values(state.joint_bias_accelerations), state.type_sorted_tree_joints, values(state.qs), values(state.vs))
    state.joint_bias_accelerations.dirty = false
    nothing
end

@inline update_motion_subspaces!(state::MechanismState) = isdirty(state.motion_subspaces) && _update_motion_subspaces!(state)
@noinline function _update_motion_subspaces!(state::MechanismState)
    update_transforms!(state)
    map_with_extra_args!(state, state.motion_subspaces.data, state.type_sorted_tree_joints, values(state.qs)) do state, joint, qjoint
        jointid = id(joint)
        bodyid = successorid(jointid, state)
        transform(motion_subspace(joint, qjoint), transform_to_root(state, bodyid))
    end
    state.motion_subspaces.dirty = false
    nothing
end

@inline update_twists_wrt_world!(state::MechanismState) = isdirty(state.twists_wrt_world) && _update_twists_wrt_world!(state)
@noinline function _update_twists_wrt_world!(state::MechanismState)
    update_transforms!(state)
    update_joint_twists!(state)
    twists = state.twists_wrt_world
    for jointid in state.treejointids
        parentbodyid, bodyid = predsucc(jointid, state)
        jointtwist = state.joint_twists[jointid]
        twists[bodyid] = twists[parentbodyid] + transform(jointtwist, transform_to_root(state, bodyid))
    end
    state.twists_wrt_world.dirty = false
    nothing
end

@inline update_constraint_wrench_subspaces!(state::MechanismState) = isdirty(state.constraint_wrench_subspaces) && _update_constraint_wrench_subspaces!(state)
@noinline function _update_constraint_wrench_subspaces!(state::MechanismState)
    update_transforms!(state)
    foreach_with_extra_args(state.constraint_wrench_subspaces, state, state.type_sorted_non_tree_joints) do results, state, joint
        jointid = id(joint)
        bodyid = successorid(jointid, state)
        body = successor(joint, state.mechanism) # TODO
        tf = state.joint_transforms[jointid]
        T = constraint_wrench_subspace(joint, tf)
        toroot = state.transforms_to_root[bodyid] * frame_definition(body, T.frame) # TODO: expensive
        results[jointid] = WrenchSubspace(transform(T, toroot))
    end
    state.constraint_wrench_subspaces.dirty = false
    nothing
end

@inline update_bias_accelerations_wrt_world!(state::MechanismState) = isdirty(state.bias_accelerations_wrt_world) && _update_bias_accelerations_wrt_world!(state)
@noinline function _update_bias_accelerations_wrt_world!(state::MechanismState) # TODO: make more efficient
    update_transforms!(state)
    update_twists_wrt_world!(state)
    update_joint_bias_accelerations!(state)
    biasaccels = state.bias_accelerations_wrt_world
    for jointid in state.treejointids
        parentbodyid, bodyid = predsucc(jointid, state)
        jointbias = bias_acceleration(state, jointid)
        # TODO: awkward way of doing this:
        toroot = transform_to_root(state, bodyid)
        twistwrtworld = transform(twist_wrt_world(state, bodyid), inv(toroot))
        jointtwist = twist(state, jointid)
        biasaccels[bodyid] = biasaccels[parentbodyid] + transform(jointbias, toroot, twistwrtworld, jointtwist)
    end
    state.bias_accelerations_wrt_world.dirty = false
    nothing
end

@inline update_spatial_inertias!(state::MechanismState) = isdirty(state.inertias) && _update_spatial_inertias!(state)
@noinline function _update_spatial_inertias!(state::MechanismState)
    update_transforms!(state)
    mechanism = state.mechanism
    inertias = state.inertias
    for joint in tree_joints(mechanism)
        body = successor(joint, mechanism)
        bodyid = id(body)
        inertias[bodyid] = transform(spatial_inertia(body), transform_to_root(state, bodyid))
    end
    state.inertias.dirty = false
    nothing
end

@inline update_crb_inertias!(state::MechanismState) = isdirty(state.crb_inertias) && _update_crb_inertias!(state)
@noinline function _update_crb_inertias!(state::MechanismState)
    update_spatial_inertias!(state)
    mechanism = state.mechanism
    crb_inertias = state.crb_inertias
    rootbodyid = id(root_body(mechanism))
    crb_inertias[rootbodyid] = state.inertias[rootbodyid]
    for jointid in state.treejointids
        bodyid = successorid(jointid, state)
        crb_inertias[bodyid] = state.inertias[bodyid]
    end
    for jointid = reverse(state.treejointids)
        parentbodyid, bodyid = predsucc(jointid, state)
        crb_inertias[parentbodyid] += crb_inertias[bodyid]
    end
    state.crb_inertias.dirty = false
    nothing
end

contact_states(state::MechanismState, body::Union{<:RigidBody, BodyID}) = state.contact_states[body]

function newton_euler(state::MechanismState, body::Union{<:RigidBody, BodyID}, accel::SpatialAcceleration)
    inertia = spatial_inertia(state, body)
    twist = twist_wrt_world(state, body)
    newton_euler(inertia, accel, twist)
end

momentum(state::MechanismState, body::Union{<:RigidBody, BodyID}) = spatial_inertia(state, body) * twist_wrt_world(state, body)
momentum_rate_bias(state::MechanismState, body::Union{<:RigidBody, BodyID}) = newton_euler(state, body, bias_acceleration(state, body))
kinetic_energy(state::MechanismState, body::Union{<:RigidBody, BodyID}) = kinetic_energy(spatial_inertia(state, body), twist_wrt_world(state, body))

"""
$(SIGNATURES)

Return the gravitational potential energy in the given state, computed as the
negation of the dot product of the gravitational force and the center
of mass expressed in the `Mechanism`'s root frame.
"""
function gravitational_potential_energy(state::MechanismState, body::Union{<:RigidBody, BodyID})
    inertia = spatial_inertia(body)
    m = inertia.mass
    m > 0 || return zero(cache_eltype(state))
    com = transform_to_root(state, body) * center_of_mass(inertia)
    -m * dot(state.mechanism.gravitational_acceleration, FreeVector3D(com))
end

function configuration_derivative!(out::AbstractVector{X}, state::MechanismState{X}) where {X}
    # TODO: replace with plain foreach and closure once that doesn't allocate
    joint_configuration_derivative! = (qd, joint, qjoint, vjoint) -> begin
        qdjoint = fastview(qd, first(parentindexes(qjoint)))
        velocity_to_configuration_derivative!(qdjoint, joint, qjoint, vjoint)
    end
    foreach_with_extra_args(joint_configuration_derivative!, out, state.type_sorted_tree_joints, values(state.qs), values(state.vs))
end

function configuration_derivative_to_velocity_adjoint!(fq, state::MechanismState, fv)
    # TODO: replace with plain foreach and closure once that doesn't allocate
    foreach_with_extra_args(fq, state, fv, state.type_sorted_tree_joints, values(state.qs)) do fq, state, fv, joint, qjoint
        fqjoint = fastview(fq, configuration_range(state, joint))
        fvjoint = fastview(fv, velocity_range(state, joint))
        configuration_derivative_to_velocity_adjoint!(fqjoint, joint, qjoint, fvjoint)
    end
end

function configuration_derivative(state::MechanismState{X}) where {X}
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
    update_twists_wrt_world!(state)
    update_spatial_inertias!(state)
    non_root_body_sum(state, zero(Momentum{T}, root_frame(state.mechanism)), (state, body) -> momentum(state, body), body_itr)
end

function momentum_rate_bias(state::MechanismState, body_itr)
    T = cache_eltype(state)
    update_bias_accelerations_wrt_world!(state)
    update_spatial_inertias!(state)
    non_root_body_sum(state, zero(Wrench{T}, root_frame(state.mechanism)), (state, body) -> momentum_rate_bias(state, body), body_itr)
end

function kinetic_energy(state::MechanismState, body_itr)
    T = cache_eltype(state)
    update_twists_wrt_world!(state)
    update_spatial_inertias!(state)
    non_root_body_sum(state, zero(T), (state, body) -> kinetic_energy(state, body), body_itr)
end

function gravitational_potential_energy(state::MechanismState, body_itr)
    T = cache_eltype(state)
    update_transforms!(state)
    non_root_body_sum(state, zero(T), (state, body) -> gravitational_potential_energy(state, body), body_itr)
end

for fun in (:momentum, :momentum_rate_bias, :kinetic_energy, :gravitational_potential_energy)
    @eval $fun(state::MechanismState) = $fun(state, bodies(state.mechanism))
end

"""
$(SIGNATURES)

Return the homogeneous transform from `from` to `to`.
"""
function relative_transform(state::MechanismState, from::CartesianFrame3D, to::CartesianFrame3D)
    update_transforms!(state)
    inv(transform_to_root(state, to)) * transform_to_root(state, from)
end

"""
$(SIGNATURES)

Return the twist of `body` with respect to `base`, expressed in the
`Mechanism`'s root frame.
"""
function relative_twist(state::MechanismState, body::Union{<:RigidBody, BodyID}, base::Union{<:RigidBody, BodyID})
    -twist_wrt_world(state, base) + twist_wrt_world(state, body)
end

"""
$(SIGNATURES)

Return the twist of `body_frame` with respect to `base_frame`, expressed in the
`Mechanism`'s root frame.
"""
function relative_twist(state::MechanismState, body_frame::CartesianFrame3D, base_frame::CartesianFrame3D)
    twist = relative_twist(state, body_fixed_frame_to_body(state.mechanism, body_frame), body_fixed_frame_to_body(state.mechanism, base_frame))
    Twist(body_frame, base_frame, twist.frame, angular(twist), linear(twist))
end

for VectorType in (:Point3D, :FreeVector3D, :Twist, :Momentum, :Wrench)
    @eval begin
        function transform(state::MechanismState, v::$VectorType, to::CartesianFrame3D)
            # TODO: consider transforming in steps, so that computing the relative transform is not necessary
            transform(v, relative_transform(state, v.frame, to))
        end
    end
end

function transform(state::MechanismState, accel::SpatialAcceleration, to::CartesianFrame3D)
    old_to_root = transform_to_root(state, accel.frame)
    root_to_old = inv(old_to_root)
    twist_of_body_wrt_base = transform(relative_twist(state, accel.body, accel.base), root_to_old)
    twist_of_old_wrt_new = transform(relative_twist(state, accel.frame, to), root_to_old)
    old_to_new = inv(transform_to_root(state, to)) * old_to_root
    transform(accel, old_to_new, twist_of_old_wrt_new, twist_of_body_wrt_base)
end

"""
$(SIGNATURES)

Compute local coordinates ``\\phi`` centered around (global) configuration vector
``q_0``, as well as their time derivatives ``\\dot{\\phi}``.
""" # TODO: refer to the method that takes a joint once it's moved to its own Joints module
function local_coordinates!(ϕ::StridedVector, ϕd::StridedVector, state::MechanismState, q0::StridedVector)
    # TODO: replace with plain foreach and closure once that doesn't allocate
    joint_local_coordinates! = (ϕ, ϕd, q0, joint, qjoint, vjoint) -> begin
        qrange = first(parentindexes(qjoint))
        vrange = first(parentindexes(vjoint))
        ϕjoint = fastview(ϕ, vrange)
        ϕdjoint = fastview(ϕd, vrange)
        q0joint = fastview(q0, qrange)
        local_coordinates!(ϕjoint, ϕdjoint, joint, q0joint, qjoint, vjoint)
    end
    foreach_with_extra_args(joint_local_coordinates!, ϕ, ϕd, q0, state.type_sorted_tree_joints, values(state.qs), values(state.vs))
end

"""
$(SIGNATURES)

Convert local coordinates ``\\phi`` centered around ``q_0`` to (global)
configuration vector ``q``.
""" # TODO: refer to the method that takes a joint once it's moved to its own Joints module
function global_coordinates!(state::MechanismState, q0::StridedVector, ϕ::StridedVector)
    # TODO: replace with plain foreach and closure once that doesn't allocate
    joint_global_coordinates! = (q0, ϕ, joint, qjoint, vjoint) -> begin
        qrange = first(parentindexes(qjoint))
        vrange = first(parentindexes(vjoint))
        q0joint = fastview(q0, qrange)
        ϕjoint = fastview(ϕ, vrange)
        global_coordinates!(qjoint, joint, q0joint, ϕjoint)
    end
    foreach_with_extra_args(joint_global_coordinates!, q0, ϕ, state.type_sorted_tree_joints, values(state.qs), values(state.vs))
end
