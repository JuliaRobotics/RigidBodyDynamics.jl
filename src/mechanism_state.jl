## BodyDict, JointDict
const BodyDict{V} = IndexDict{BodyID, Base.OneTo{BodyID}, V}
const BodyCacheDict{V} = CacheIndexDict{BodyID, Base.OneTo{BodyID}, V}
@propagate_inbounds Base.getindex(d::AbstractIndexDict{BodyID}, key::RigidBody) = d[BodyID(key)]
@propagate_inbounds Base.setindex!(d::AbstractIndexDict{BodyID}, value, key::RigidBody) = d[BodyID(key)] = value

const JointDict{V} = IndexDict{JointID, Base.OneTo{JointID}, V}
const JointCacheDict{V} = CacheIndexDict{JointID, Base.OneTo{JointID}, V}
@propagate_inbounds Base.getindex(d::AbstractIndexDict{JointID}, key::Joint) = d[JointID(key)]
@propagate_inbounds Base.setindex!(d::AbstractIndexDict{JointID}, value, key::Joint) = d[JointID(key)] = value


## SegmentedVector method overloads
@propagate_inbounds Base.getindex(v::SegmentedVector{JointID}, id::JointID) = v.segments[id]
@propagate_inbounds Base.getindex(v::SegmentedVector{JointID}, joint::Joint) = v[JointID(joint)]
function SegmentedVector(parent::AbstractVector{T}, joints::AbstractVector{<:Joint}, viewlengthfun) where T
    SegmentedVector{JointID, T, Base.OneTo{JointID}}(parent, joints, viewlengthfun)
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
* `JointCollection`: the type of the `treejoints` and `nontreejoints` members (a `TypeSortedCollection` subtype)
"""
struct MechanismState{X, M, C, JointCollection}
    modcount::Int

    # mechanism layout
    mechanism::Mechanism{M}
    nonrootbodies::Vector{RigidBody{M}} # TODO: remove once https://github.com/JuliaLang/julia/issues/14955 is fixed
    treejoints::JointCollection
    nontreejoints::JointCollection
    jointids::Base.OneTo{JointID}
    treejointids::Base.OneTo{JointID}
    nontreejointids::UnitRange{JointID}
    predecessor_and_successor_ids::JointDict{Pair{BodyID, BodyID}}
    qranges::JointDict{UnitRange{Int}}
    vranges::JointDict{UnitRange{Int}}
    constraintranges::IndexDict{JointID, UnitRange{JointID}, UnitRange{Int}}
    support_set_masks::BodyDict{JointDict{Bool}} # TODO: use a Matrix-backed type
    constraint_jacobian_structure::JointDict{TreePath{RigidBody{M}, Joint{M}}} # TODO: use a Matrix-backed type
    q_index_to_joint_id::Vector{JointID}
    v_index_to_joint_id::Vector{JointID}

    # minimal representation of state
    q::SegmentedVector{JointID, X, Base.OneTo{JointID}, Vector{X}} # configurations
    v::SegmentedVector{JointID, X, Base.OneTo{JointID}, Vector{X}} # velocities
    s::Vector{X} # additional state

    # joint-related cache
    joint_transforms::JointCacheDict{Transform3D{C}}
    joint_twists::JointCacheDict{Twist{C}}
    joint_bias_accelerations::JointCacheDict{SpatialAcceleration{C}}
    tree_joint_transforms::VectorSegment{Transform3D{C}}
    non_tree_joint_transforms::VectorSegment{Transform3D{C}}

    #
    motion_subspaces::CacheElement{Vector{GeometricJacobian{SMatrix{3, 1, C, 3}}}} # TODO: use CacheIndexDict?
    constraint_wrench_subspaces::CacheElement{Vector{WrenchMatrix{SMatrix{3, 1, C, 3}}}} # TODO: use CacheIndexDict?

    # body-related cache
    transforms_to_root::BodyCacheDict{Transform3D{C}}
    twists_wrt_world::BodyCacheDict{Twist{C}}
    bias_accelerations_wrt_world::BodyCacheDict{SpatialAcceleration{C}}
    inertias::BodyCacheDict{SpatialInertia{C}}
    crb_inertias::BodyCacheDict{SpatialInertia{C}}
    contact_states::BodyCacheDict{Vector{Vector{DefaultSoftContactState{X}}}} # TODO: consider moving to separate type

    function MechanismState{X, M, C, JointCollection}(m::Mechanism{M}, q::Vector{X}, v::Vector{X}, s::Vector{X}) where {X, M, C, JointCollection}
        @assert length(q) == num_positions(m)
        @assert length(v) == num_velocities(m)
        @assert length(s) == num_additional_states(m)

        # mechanism layout
        nonrootbodies = collect(non_root_bodies(m))
        treejoints = JointCollection(tree_joints(m))
        nontreejoints = JointCollection(non_tree_joints(m))
        lastjointid = isempty(joints(m)) ? JointID(0) : JointID(last(joints(m)))
        lasttreejointid = isempty(tree_joints(m)) ? JointID(0) : JointID(last(tree_joints(m)))
        jointids = Base.OneTo(lastjointid)
        treejointids = Base.OneTo(lasttreejointid)
        nontreejointids = lasttreejointid + 1 : lastjointid
        predecessor_and_successor_ids = JointDict{Pair{BodyID, BodyID}}(
            JointID(j) => (BodyID(predecessor(j, m)) => BodyID(successor(j, m))) for j in joints(m))
        support_set = function (b::RigidBody)
            JointDict{Bool}(JointID(j) => (j ∈ tree_joints(m) ?  j ∈ path(m, b, root_body(m)) : false) for j in joints(m))
        end
        support_set_masks = BodyDict{JointDict{Bool}}(b => support_set(b) for b in bodies(m))
        constraint_jacobian_structure = JointDict{TreePath{RigidBody{M}, Joint{M}}}(
            JointID(j) => path(m, predecessor(j, m), successor(j, m)) for j in joints(m))
        qsegmented = SegmentedVector(q, tree_joints(m), num_positions)
        vsegmented = SegmentedVector(v, tree_joints(m), num_velocities)
        qranges = ranges(qsegmented)
        vranges = ranges(vsegmented)
        constraintranges = let start = 1
            rangevec = UnitRange{Int}[start : (start += num_constraints(j)) - 1 for j in non_tree_joints(m)]
            IndexDict(nontreejointids, rangevec)
        end
        q_index_to_joint_id = Vector{JointID}(undef, length(q))
        v_index_to_joint_id = Vector{JointID}(undef, length(v))
        for jointid in treejointids
            for qindex in qranges[jointid]
                q_index_to_joint_id[qindex] = jointid
            end
            for vindex in vranges[jointid]
                v_index_to_joint_id[vindex] = jointid
            end
        end

        # joint-related cache
        joint_transforms = JointCacheDict{Transform3D{C}}(jointids)
        joint_twists = JointCacheDict{Twist{C}}(treejointids)
        joint_bias_accelerations = JointCacheDict{SpatialAcceleration{C}}(treejointids)
        tree_joint_transforms = view(values(joint_transforms), 1 : Int(lasttreejointid))
        non_tree_joint_transforms = view(values(joint_transforms), Int(lasttreejointid) + 1 : Int(lastjointid))

        # motion subspaces, constraint wrench subspaces
        motion_subspaces = CacheElement(Vector{GeometricJacobian{SMatrix{3, 1, C, 3}}}(undef, num_velocities(m)))
        constraint_wrench_subspaces = CacheElement(Vector{WrenchMatrix{SMatrix{3, 1, C, 3}}}(undef, num_constraints(m)))

        # body-related cache
        bodyids = Base.OneTo(BodyID(last(bodies(m))))
        transforms_to_root = BodyCacheDict{Transform3D{C}}(bodyids)
        twists_wrt_world = BodyCacheDict{Twist{C}}(bodyids)
        bias_accelerations_wrt_world = BodyCacheDict{SpatialAcceleration{C}}(bodyids)
        inertias = BodyCacheDict{SpatialInertia{C}}(bodyids)
        crb_inertias = BodyCacheDict{SpatialInertia{C}}(bodyids)

        # contact. TODO: move out of MechanismState:
        contact_states = BodyCacheDict{Vector{Vector{DefaultSoftContactState{X}}}}(
            BodyID(b) => Vector{Vector{DefaultSoftContactState{X}}}() for b in bodies(m))
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
        transforms_to_root[root] = one(Transform3D{C}, rootframe)
        twists_wrt_world[root] = zero(Twist{C}, rootframe, rootframe, rootframe)
        bias_accelerations_wrt_world[root] = zero(SpatialAcceleration{C}, rootframe, rootframe, rootframe)
        inertias[root] = zero(SpatialInertia{C}, rootframe)

        new{X, M, C, JointCollection}(
            modcount(m), m, nonrootbodies, treejoints, nontreejoints,
            jointids, treejointids, nontreejointids,
            predecessor_and_successor_ids, qranges, vranges, constraintranges, support_set_masks, constraint_jacobian_structure,
            q_index_to_joint_id, v_index_to_joint_id,
            qsegmented, vsegmented, s,
            joint_transforms, joint_twists, joint_bias_accelerations, tree_joint_transforms, non_tree_joint_transforms,
            motion_subspaces, constraint_wrench_subspaces,
            transforms_to_root, twists_wrt_world, bias_accelerations_wrt_world, inertias, crb_inertias,
            contact_states)
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
        q = Vector{X}(undef, num_positions(mechanism))
        v = Vector{X}(undef, num_velocities(mechanism))
        s = Vector{X}(undef, num_additional_states(mechanism))
        state = MechanismState{X, M}(mechanism, q, v, s)
        zero!(state)
        state
    end
end

function MechanismState(mechanism::Mechanism{M}, q::Vector{X}, v::Vector{X}, s=zeros(X, num_additional_states(mechanism))) where {X, M}
    MechanismState{X, M}(mechanism, q, v, s)
end

MechanismState(mechanism::Mechanism{M}) where {M} = MechanismState{M}(mechanism)

Base.broadcastable(x::MechanismState) = Ref(x)

Base.show(io::IO, ::MechanismState{X, M, C}) where {X, M, C} = print(io, "MechanismState{$X, $M, $C, …}(…)")

modcount(state::MechanismState) = state.modcount
@propagate_inbounds predsucc(id::JointID, state::MechanismState) = state.predecessor_and_successor_ids[id]
@propagate_inbounds successorid(id::JointID, state::MechanismState) = last(state.predecessor_and_successor_ids[id])

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
configuration(state::MechanismState, joint::Union{<:Joint, JointID}) = state.q[joint]

"""
$(SIGNATURES)

Return the part of the velocity vector ``v`` associated with `joint`.
"""
velocity(state::MechanismState, joint::Union{<:Joint, JointID}) = state.v[joint]

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
    nothing
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
    foreach(state.treejoints, values(segments(state.q))) do joint, qjoint
        zero_configuration!(qjoint, joint)
    end
    reset_contact_state!(state)
    setdirty!(state)
end

"""
$(SIGNATURES)

Zero the velocity vector ``v``. Invalidates cache variables.
"""
function zero_velocity!(state::MechanismState)
    state.v .= 0
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
    foreach(state.treejoints, values(segments(state.q))) do joint, qjoint
        rand_configuration!(qjoint, joint)
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

for fun in (:num_velocities, :num_positions)
    @eval function $fun(p::TreePath{RigidBody{T}, <:Joint{T}} where {T})
        mapreduce($fun, +, p; init=0)
    end
end

"""
$(SIGNATURES)

Set the part of the configuration vector associated with `joint`.
Invalidates cache variables.
"""
function set_configuration!(state::MechanismState, joint::Joint, config)
    set_configuration!(configuration(state, joint), joint, config)
    reset_contact_state!(state)
    setdirty!(state)
end

"""
$(SIGNATURES)

Set the part of the velocity vector associated with `joint`.
Invalidates cache variables.
"""
function set_velocity!(state::MechanismState, joint::Joint, vel)
    set_velocity!(velocity(state, joint), joint, vel)
    reset_contact_state!(state)
    setdirty!(state)
end

"""
$(SIGNATURES)

Set the configuration vector ``q``. Invalidates cache variables.
"""
function set_configuration!(state::MechanismState, q::AbstractVector)
    copyto!(state.q, q)
    setdirty!(state)
end

"""
$(SIGNATURES)

Set the velocity vector ``v``. Invalidates cache variables.
"""
function set_velocity!(state::MechanismState, v::AbstractVector)
    copyto!(state.v, v)
    setdirty!(state)
end

"""
$(SIGNATURES)

Set the vector of additional states ``s``.
"""
function set_additional_state!(state::MechanismState, s::AbstractVector)
    copyto!(state.s, s)
    # note: setdirty! is currently not needed because no cache variables depend on s
end

"""
$(SIGNATURES)

Copy (minimal representation of) state `src` to state `dest`.
"""
function Base.copyto!(dest::MechanismState, src::MechanismState)
    dest.mechanism == src.mechanism || throw(ArgumentError("States are not associated with the same Mechanism."))
    @modcountcheck dest src
    copyto!(dest.q, src.q)
    copyto!(dest.v, src.v)
    copyto!(dest.s, src.s)
    setdirty!(dest)
    dest
end

"""
$(SIGNATURES)

Copy state information in vector `src` (ordered `[q; v; s]`) to state `dest`.
"""
function Base.copyto!(dest::MechanismState, src::AbstractVector)
    nq = num_positions(dest)
    nv = num_velocities(dest)
    ns = num_additional_states(dest)
    @boundscheck length(src) == nq + nv + ns || throw(DimensionMismatch())
    @inbounds copyto!(parent(dest.q), 1, src, 1, nq)
    @inbounds copyto!(parent(dest.v), 1, src, nq + 1, nv)
    @inbounds copyto!(dest.s, 1, src, nq + nv + 1, ns)
    setdirty!(dest)
    dest
end

"""
$(SIGNATURES)

Copy state information in state `dest` to vector `src` (ordered `[q; v; s]`).
"""
function Base.copyto!(dest::AbstractVector, src::MechanismState)
    nq = num_positions(src)
    nv = num_velocities(src)
    ns = num_additional_states(src)
    length(dest) == nq + nv + ns || throw(DimensionMismatch())
    @inbounds copyto!(dest, 1, src.q, 1, nq)
    @inbounds copyto!(dest, nq + 1, src.v, 1, nv)
    @inbounds copyto!(dest, nq + nv + 1, src.s, 1, ns)
    dest
end

"""
$(SIGNATURES)

Create a `Vector` that represents the same state as `state` (ordered `[q; v; s]`).
"""
function Base.Vector{T}(state::MechanismState) where T
    dest = Vector{T}(undef, num_positions(state) + num_velocities(state) + num_additional_states(state))
    copyto!(dest, state)
end

Base.Vector(state::MechanismState{X}) where {X} = Vector{X}(state)
Base.Array{T}(state::MechanismState) where {T} = Vector{T}(state)
Base.Array(state::MechanismState{X}) where {X} = Array{X}(state)
Base.convert(::Type{T}, state::MechanismState) where {T<:Array} = T(state)

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
    foreach(state.treejoints, values(segments(state.q))) do joint, qjoint
        normalize_configuration!(qjoint, joint)
    end
end

"""
$(SIGNATURES)

Applies the principal_value functions from [Rotations.jl](https://github.com/FugroRoames/Rotations.jl/blob/d080990517f89b56c37962ad53a7fd24bd94b9f7/src/principal_value.jl)
to joint angles. This currently only applies to `SPQuatFloating` joints.

For example:
* for a part of ``q`` corresponding to a revolute joint, this method is a no-op;
* for a part of ``q`` corresponding to a `SPQuatFloating` joint this function applies
`principal_value the orientation.
"""
function principal_value!(state::MechanismState)
    foreach(state.treejoints, values(segments(state.q))) do joint, qjoint
        principal_value!(qjoint, joint)
    end
end

"""
$(SIGNATURES)

Return the range of indices into the joint configuration vector ``q`` corresponding to joint `joint`.
"""
@propagate_inbounds configuration_range(state::MechanismState, joint::Union{<:Joint, JointID}) = state.qranges[joint]

"""
$(SIGNATURES)

Return the range of indices into the joint velocity vector ``v`` corresponding to joint `joint`.
"""
@propagate_inbounds velocity_range(state::MechanismState, joint::Union{<:Joint, JointID}) = state.vranges[joint]

"""
$(SIGNATURES)

Return the `JointID` of the joint associated with the given index into the configuration vector ``q``.
"""
@propagate_inbounds configuration_index_to_joint_id(state::MechanismState, qindex::Integer) = state.q_index_to_joint_id[qindex]

"""
$(SIGNATURES)

Return the `JointID` of the joint associated with the given index into the velocity vector ``v``.
"""
@propagate_inbounds velocity_index_to_joint_id(state::MechanismState, qindex::Integer) = state.v_index_to_joint_id[qindex]

"""
$(SIGNATURES)

Return the range of row indices into the constraint Jacobian corresponding to joint `joint`.
"""
@propagate_inbounds constraint_range(state::MechanismState, joint::Union{<:Joint, JointID}) = state.constraintranges[joint]


"""
$(SIGNATURES)

Return whether `joint` supports `body`, i.e., `joint` is a tree joint on the path between `body` and the root.
"""
@propagate_inbounds function supports(joint::Union{<:Joint, JointID}, body::Union{<:RigidBody, BodyID}, state::MechanismState)
    state.support_set_masks[body][joint]
end

## Accessor functions for cached variables
"""
$(SIGNATURES)

Return the joint transform for the given joint, i.e. the transform from
`frame_after(joint)` to `frame_before(joint)`.
"""
@propagate_inbounds function joint_transform(state::MechanismState, joint::Union{<:Joint, JointID}, safe=true)
    safe && update_transforms!(state)
    state.joint_transforms[joint]
end

"""
$(SIGNATURES)
Return the joint twist for the given joint, i.e. the twist of
`frame_after(joint)` with respect to `frame_before(joint)`, expressed in the
root frame of the mechanism.
"""
@propagate_inbounds function twist(state::MechanismState, joint::Union{<:Joint, JointID}, safe=true)
    safe && update_joint_twists!(state)
    state.joint_twists[joint]
end

"""
$(SIGNATURES)

Return the bias acceleration across the given joint, i.e. the spatial acceleration
of `frame_after(joint)` with respect to `frame_before(joint)`, expressed in the
root frame of the mechanism when all joint accelerations are zero.
"""
@propagate_inbounds function bias_acceleration(state::MechanismState, joint::Union{<:Joint, JointID}, safe=true)
    safe && update_joint_bias_accelerations!(state)
    state.joint_bias_accelerations[joint]
end

"""
$(SIGNATURES)

Return the transform from `default_frame(body)` to the root frame of the
mechanism.
"""
@propagate_inbounds function transform_to_root(state::MechanismState, body::Union{<:RigidBody, BodyID}, safe=true)
    safe && update_transforms!(state)
    state.transforms_to_root[body]
end

"""
$(SIGNATURES)

Return the twist of `default_frame(body)` with respect to the root frame of the
mechanism, expressed in the root frame.
"""
@propagate_inbounds function twist_wrt_world(state::MechanismState, body::Union{<:RigidBody, BodyID}, safe=true)
    safe && update_twists_wrt_world!(state)
    state.twists_wrt_world[body]
end

"""
$(SIGNATURES)

Return the bias acceleration of the given body with respect to the world,
i.e. the spatial acceleration of `default_frame(body)` with respect to the
root frame of the mechanism, expressed in the root frame, when all joint
accelerations are zero.
"""
@propagate_inbounds function bias_acceleration(state::MechanismState, body::Union{<:RigidBody, BodyID}, safe=true)
    safe && update_bias_accelerations_wrt_world!(state)
    state.bias_accelerations_wrt_world[body]
end

"""
$(SIGNATURES)

Return the spatial inertia of `body` expressed in the root frame of the
mechanism.
"""
@propagate_inbounds function spatial_inertia(state::MechanismState, body::Union{<:RigidBody, BodyID}, safe=true)
    safe && update_spatial_inertias!(state)
    state.inertias[body]
end

"""
$(SIGNATURES)

Return the composite rigid body inertia `body` expressed in the root frame of the
mechanism.
"""
@propagate_inbounds function crb_inertia(state::MechanismState, body::Union{<:RigidBody, BodyID}, safe=true)
    safe && update_crb_inertias!(state)
    state.crb_inertias[body]
end


# Cache variable update functions
@inline update_transforms!(state::MechanismState) = isdirty(state.transforms_to_root) && _update_transforms!(state)
@noinline function _update_transforms!(state::MechanismState)
    @modcountcheck state state.mechanism

    # update tree joint transforms
    qs = values(segments(state.q))
    @inbounds map!(joint_transform, state.tree_joint_transforms, state.treejoints, qs)

    # update transforms to root
    transforms_to_root = state.transforms_to_root
    for joint in tree_joints(state.mechanism)
        jointid = JointID(joint)
        parentid, bodyid = predsucc(jointid, state)
        transforms_to_root[bodyid] = transforms_to_root[parentid] * joint_to_predecessor(joint) * state.joint_transforms[jointid]
    end
    state.transforms_to_root.dirty = false

    # update non-tree joint transforms
    if !isempty(state.nontreejointids)
        broadcast!(state.non_tree_joint_transforms, state, state.nontreejoints) do state, joint
            predid, succid = predsucc(JointID(joint), state)
            before_to_root = state.transforms_to_root[predid] * joint_to_predecessor(joint)
            after_to_root = state.transforms_to_root[succid] * joint_to_successor(joint)
            inv(before_to_root) * after_to_root
        end
    end
    state.joint_transforms.dirty = false
    nothing
end

@inline function update_joint_twists!(state::MechanismState)
    isdirty(state.joint_twists) && _update_joint_twists!(state)
    nothing
end
@noinline function _update_joint_twists!(state::MechanismState)
    qs = values(segments(state.q))
    vs = values(segments(state.v))
    @inbounds map!(joint_twist, values(state.joint_twists), state.treejoints, qs, vs)
    state.joint_twists.dirty = false
    nothing
end

@inline function update_joint_bias_accelerations!(state::MechanismState)
    isdirty(state.joint_bias_accelerations) && _update_joint_bias_accelerations!(state)
    nothing
end
@noinline function _update_joint_bias_accelerations!(state::MechanismState)
    qs = values(segments(state.q))
    vs = values(segments(state.v))
    @inbounds map!(bias_acceleration, values(state.joint_bias_accelerations), state.treejoints, qs, vs)
    state.joint_bias_accelerations.dirty = false
    nothing
end

@inline function update_motion_subspaces!(state::MechanismState)
    isdirty(state.motion_subspaces) && _update_motion_subspaces!(state)
    nothing
end
@inline function _motion_subspace(state::MechanismState, joint::Joint, qjoint::AbstractVector)
    jointid = JointID(joint)
    bodyid = successorid(jointid, state)
    transform(motion_subspace(joint, qjoint), transform_to_root(state, bodyid, false))
end
@noinline function _update_motion_subspaces!(state::MechanismState)
    update_transforms!(state)
    qs = values(segments(state.q))
    foreach_with_extra_args(state, state.treejoints, qs) do state, joint, qjoint
        S = _motion_subspace(state, joint, qjoint)
        @inbounds vrange = velocity_range(state, joint)
        @inbounds for col = Base.OneTo(size(S, 2))
            Scol = GeometricJacobian(S.body, S.base, S.frame, SMatrix{3, 1}(S.angular[:, col]), SMatrix{3, 1}(S.linear[:, col]))
            vindex = vrange[col]
            state.motion_subspaces.data[vindex] = Scol
        end
    end
    state.motion_subspaces.dirty = false
    nothing
end

@inline function update_twists_wrt_world!(state::MechanismState)
    isdirty(state.twists_wrt_world) && _update_twists_wrt_world!(state)
    nothing
end
@noinline function _update_twists_wrt_world!(state::MechanismState)
    update_transforms!(state)
    update_joint_twists!(state)
    twists = state.twists_wrt_world
    @inbounds for jointid in state.treejointids
        parentbodyid, bodyid = predsucc(jointid, state)
        jointtwist = state.joint_twists[jointid]
        twists[bodyid] = twists[parentbodyid] + transform(jointtwist, transform_to_root(state, bodyid, false))
    end
    state.twists_wrt_world.dirty = false
    nothing
end

@inline function update_constraint_wrench_subspaces!(state::MechanismState)
    isdirty(state.constraint_wrench_subspaces) && _update_constraint_wrench_subspaces!(state)
    nothing
end
@inline function _constraint_wrench_subspace(state::MechanismState, joint::Joint)
    jointid = JointID(joint)
    tf = state.joint_transforms[jointid]
    T = constraint_wrench_subspace(joint, tf)
    bodyid = successorid(jointid, state)
    toroot = state.transforms_to_root[bodyid] * joint_to_successor(joint)
    transform(T, toroot)
end

@noinline function _update_constraint_wrench_subspaces!(state::MechanismState)
    update_transforms!(state)
    foreach_with_extra_args(state, state.nontreejoints) do state, joint
        T = _constraint_wrench_subspace(state, joint)
        @inbounds crange = constraint_range(state, joint)
        @inbounds for col = Base.OneTo(size(T, 2))
            Tcol = WrenchMatrix(T.frame, SMatrix{3, 1}(T.angular[:, col]), SMatrix{3, 1}(T.linear[:, col]))
            cindex = crange[col]
            state.constraint_wrench_subspaces.data[cindex] = Tcol
        end
    end
    state.constraint_wrench_subspaces.dirty = false
    nothing
end

@inline function update_bias_accelerations_wrt_world!(state::MechanismState)
    isdirty(state.bias_accelerations_wrt_world) && _update_bias_accelerations_wrt_world!(state)
    nothing
end
@noinline function _update_bias_accelerations_wrt_world!(state::MechanismState) # TODO: make more efficient
    update_transforms!(state)
    update_twists_wrt_world!(state)
    update_joint_bias_accelerations!(state)
    biasaccels = state.bias_accelerations_wrt_world
    for jointid in state.treejointids
        parentbodyid, bodyid = predsucc(jointid, state)
        jointbias = bias_acceleration(state, jointid)
        # TODO: awkward way of doing this:
        toroot = transform_to_root(state, bodyid, false)
        twistwrtworld = transform(twist_wrt_world(state, bodyid), inv(toroot))
        jointtwist = twist(state, jointid)
        biasaccels[bodyid] = biasaccels[parentbodyid] + transform(jointbias, toroot, twistwrtworld, jointtwist)
    end
    state.bias_accelerations_wrt_world.dirty = false
    nothing
end

@inline function update_spatial_inertias!(state::MechanismState)
    isdirty(state.inertias) && _update_spatial_inertias!(state)
    nothing
end
@noinline function _update_spatial_inertias!(state::MechanismState)
    update_transforms!(state)
    mechanism = state.mechanism
    inertias = state.inertias
    @inbounds for body in state.nonrootbodies
        bodyid = BodyID(body)
        inertias[bodyid] = transform(spatial_inertia(body), transform_to_root(state, bodyid, false))
    end
    state.inertias.dirty = false
    nothing
end

@inline function update_crb_inertias!(state::MechanismState)
    isdirty(state.crb_inertias) && _update_crb_inertias!(state)
    nothing
end
@noinline function _update_crb_inertias!(state::MechanismState)
    update_spatial_inertias!(state)
    mechanism = state.mechanism
    crb_inertias = state.crb_inertias
    rootbodyid = BodyID(root_body(mechanism))
    crb_inertias[rootbodyid] = state.inertias[rootbodyid]
    for jointid in state.treejointids
        bodyid = successorid(jointid, state)
        crb_inertias[bodyid] = state.inertias[bodyid]
    end
    for jointid in reverse(state.treejointids)# TODO: reverse is kind of expensive
        parentbodyid, bodyid = predsucc(jointid, state)
        crb_inertias[parentbodyid] += crb_inertias[bodyid]
    end
    state.crb_inertias.dirty = false
    nothing
end

contact_states(state::MechanismState, body::Union{<:RigidBody, BodyID}) = state.contact_states[body]

@propagate_inbounds function newton_euler(state::MechanismState, body::Union{<:RigidBody, BodyID}, accel::SpatialAcceleration, safe=true)
    inertia = spatial_inertia(state, body, safe)
    twist = twist_wrt_world(state, body, safe)
    newton_euler(inertia, accel, twist)
end

@propagate_inbounds function momentum(state::MechanismState, body::Union{<:RigidBody, BodyID}, safe=true)
    spatial_inertia(state, body, safe) * twist_wrt_world(state, body, safe)
end

@propagate_inbounds function momentum_rate_bias(state::MechanismState, body::Union{<:RigidBody, BodyID}, safe=true)
    newton_euler(state, body, bias_acceleration(state, body, safe), safe)
end

@propagate_inbounds function kinetic_energy(state::MechanismState, body::Union{<:RigidBody, BodyID}, safe=true)
    kinetic_energy(spatial_inertia(state, body, safe), twist_wrt_world(state, body, safe))
end

"""
$(SIGNATURES)

Return the gravitational potential energy in the given state, computed as the
negation of the dot product of the gravitational force and the center
of mass expressed in the `Mechanism`'s root frame.
"""
@propagate_inbounds function gravitational_potential_energy(state::MechanismState, body::Union{<:RigidBody, BodyID}, safe=true)
    inertia = spatial_inertia(body)
    m = inertia.mass
    m > 0 || return zero(cache_eltype(state))
    com = transform_to_root(state, body, safe) * center_of_mass(inertia)
    -m * dot(state.mechanism.gravitational_acceleration, FreeVector3D(com))
end

function configuration_derivative!(q̇::SegmentedVector{JointID}, state::MechanismState)
    q̇s = values(segments(q̇))
    qs = values(segments(state.q))
    vs = values(segments(state.v))
    foreach((joint, q̇, q, v) -> velocity_to_configuration_derivative!(q̇, joint, q, v), state.treejoints, q̇s, qs, vs)
end

function configuration_derivative_to_velocity_adjoint!(
        fq::SegmentedVector{JointID}, state::MechanismState, fv::SegmentedVector{JointID})
    fqs = values(segments(fq))
    qs = values(segments(state.q))
    fvs = values(segments(fv))
    foreach((joint, fq, q, fv) -> configuration_derivative_to_velocity_adjoint!(fq, joint, q, fv), state.treejoints, fqs, qs, fvs)
end

function configuration_derivative(state::MechanismState{X}) where {X}
    ret = SegmentedVector(Vector{X}(undef, num_positions(state.mechanism)), tree_joints(state.mechanism), num_positions)
    configuration_derivative!(ret, state)
    ret
end

function velocity_to_configuration_derivative_jacobian!(J::SegmentedBlockDiagonalMatrix, state::MechanismState)
    qs = values(segments(state.q))
    foreach(state.treejoints, qs, CustomCollections.blocks(J)) do joint, qjoint, block
        copyto!(block, velocity_to_configuration_derivative_jacobian(joint, qjoint))
    end
    nothing
end

function velocity_to_configuration_derivative_jacobian(state::MechanismState{X, M, C}) where {X, M, C}
    q_ranges = values(ranges(configuration(state)))
    v_ranges = values(ranges(velocity(state)))
    J = SegmentedBlockDiagonalMatrix(zeros(C, num_positions(state), num_velocities(state)), zip(q_ranges, v_ranges))
    velocity_to_configuration_derivative_jacobian!(J, state)
    J
end

function configuration_derivative_to_velocity_jacobian!(J::SegmentedBlockDiagonalMatrix, state::MechanismState)
    qs = values(segments(state.q))
    foreach(state.treejoints, qs, CustomCollections.blocks(J)) do joint, qjoint, block
        copyto!(block, configuration_derivative_to_velocity_jacobian(joint, qjoint))
    end
    nothing
end

function configuration_derivative_to_velocity_jacobian(state::MechanismState{X, M, C}) where {X, M, C}
    q_ranges = values(ranges(configuration(state)))
    v_ranges = values(ranges(velocity(state)))
    J = SegmentedBlockDiagonalMatrix(zeros(C, num_velocities(state), num_positions(state)), zip(v_ranges, q_ranges))
    configuration_derivative_to_velocity_jacobian!(J, state)
    J
end

function transform_to_root(state::MechanismState, frame::CartesianFrame3D, safe=true)
    body = body_fixed_frame_to_body(state.mechanism, frame) # FIXME: expensive
    tf = transform_to_root(state, body, safe)
    if tf.from != frame
        tf = tf * body_fixed_frame_definition(state.mechanism, frame) # TODO: consider caching
    end
    tf
end

@propagate_inbounds function statesum(fun::F, state::MechanismState, itr, init, safe=true) where F
    ret = init
    for x in itr
        ret += fun(state, x, safe)
    end
    ret
end

@propagate_inbounds function momentum(state::MechanismState, body_itr)
    T = cache_eltype(state)
    update_twists_wrt_world!(state)
    update_spatial_inertias!(state)
    statesum(momentum, state, body_itr, zero(Momentum{T}, root_frame(state.mechanism)), false)
end

@propagate_inbounds function momentum_rate_bias(state::MechanismState, body_itr)
    T = cache_eltype(state)
    update_bias_accelerations_wrt_world!(state)
    update_spatial_inertias!(state)
    statesum(momentum_rate_bias, state, body_itr, zero(Wrench{T}, root_frame(state.mechanism)), false)
end

@propagate_inbounds function kinetic_energy(state::MechanismState, body_itr)
    T = cache_eltype(state)
    update_twists_wrt_world!(state)
    update_spatial_inertias!(state)
    statesum(kinetic_energy, state, body_itr, zero(T), false)
end

@propagate_inbounds function gravitational_potential_energy(state::MechanismState, body_itr)
    T = cache_eltype(state)
    update_transforms!(state)
    statesum(gravitational_potential_energy, state, body_itr, zero(T), false)
end

for fun in (:momentum, :momentum_rate_bias, :kinetic_energy, :gravitational_potential_energy)
    @eval $fun(state::MechanismState) = @inbounds return $fun(state, state.nonrootbodies)
end

"""
$(SIGNATURES)

Return the homogeneous transform from `from` to `to`.
"""
function relative_transform(state::MechanismState, from::CartesianFrame3D, to::CartesianFrame3D)
    update_transforms!(state)
    inv(transform_to_root(state, to, false)) * transform_to_root(state, from, false)
end

"""
$(SIGNATURES)

Return the twist of `body` with respect to `base`, expressed in the
`Mechanism`'s root frame.
"""
function relative_twist(state::MechanismState, body::Union{<:RigidBody, BodyID}, base::Union{<:RigidBody, BodyID})
    update_twists_wrt_world!(state)
    -twist_wrt_world(state, base, false) + twist_wrt_world(state, body, false)
end

"""
$(SIGNATURES)

Return the twist of `body_frame` with respect to `base_frame`, expressed in the
`Mechanism`'s root frame.
"""
function relative_twist(state::MechanismState, body_frame::CartesianFrame3D, base_frame::CartesianFrame3D)
    body = body_fixed_frame_to_body(state.mechanism, body_frame)
    base = body_fixed_frame_to_body(state.mechanism, base_frame)
    twist = relative_twist(state, body, base)
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
function local_coordinates!(
        ϕ::SegmentedVector{JointID}, ϕd::SegmentedVector{JointID}, state::MechanismState, q0::SegmentedVector{JointID})
    ϕs = values(segments(ϕ))
    ϕds = values(segments(ϕd))
    q0s = values(segments(q0))
    qs = values(segments(state.q))
    vs = values(segments(state.v))
    foreach((joint, ϕ, ϕ̇, q0, q, v) -> local_coordinates!(ϕ, ϕ̇, joint, q0, q, v), state.treejoints, ϕs, ϕds, q0s, qs, vs)
end

"""
$(SIGNATURES)

Convert local coordinates ``\\phi`` centered around ``q_0`` to (global)
configuration vector ``q``.
""" # TODO: refer to the method that takes a joint once it's moved to its own Joints module
function global_coordinates!(state::MechanismState, q0::SegmentedVector{JointID}, ϕ::SegmentedVector{JointID})
    qs = values(segments(state.q))
    q0s = values(segments(q0))
    ϕs = values(segments(ϕ))
    foreach((joint, q, q0, ϕ) -> global_coordinates!(q, joint, q0, ϕ), state.treejoints, qs, q0s, ϕs)
end
