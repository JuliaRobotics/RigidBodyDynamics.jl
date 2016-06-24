immutable UpdateTransformToRoot{C}
    parentToRootCache::CacheElement{Transform3D{C}, UpdateTransformToRoot{C}}
    toParentCache::CacheElement{Transform3D{C}}
    UpdateTransformToRoot() = new()
    function UpdateTransformToRoot(parentToRootCache::CacheElement{Transform3D{C}, UpdateTransformToRoot{C}}, toParentCache::CacheElement{Transform3D{C}})
        new(parentToRootCache, toParentCache)
    end
end
function call(functor::UpdateTransformToRoot)
    get(functor.parentToRootCache) * get(functor.toParentCache)
end

immutable UpdateTwistWithRespectToWorld{C}
    parentFrame::CartesianFrame3D
    joint::Joint
    qJoint
    vJoint
    transformToRootCache::CacheElement{Transform3D{C}, UpdateTransformToRoot{C}}
    parentTwistCache::CacheElement{Twist{C}, UpdateTwistWithRespectToWorld{C}}

    UpdateTwistWithRespectToWorld() = new()
    function UpdateTwistWithRespectToWorld(parentFrame::CartesianFrame3D, joint::Joint, qJoint::AbstractVector, vJoint::AbstractVector, transformToRootCache::CacheElement{Transform3D{C}, UpdateTransformToRoot{C}}, parentTwistCache::CacheElement{Twist{C}, UpdateTwistWithRespectToWorld{C}})
        new(parentFrame, joint, qJoint, vJoint, transformToRootCache, parentTwistCache)
    end
end
function call{C}(functor::UpdateTwistWithRespectToWorld{C})
    parentFrame = functor.parentFrame
    joint = functor.joint
    parentTwist = get(functor.parentTwistCache)
    jointTwist = joint_twist(joint, functor.qJoint, functor.vJoint)::Twist{C}
    jointTwist = Twist(joint.frameAfter, parentFrame, jointTwist.frame, jointTwist.angular, jointTwist.linear) # to make the frames line up;
    parentTwist + transform(jointTwist, get(functor.transformToRootCache))
end

immutable UpdateSpatialInertiaInWorld{M, C}
    body::RigidBody{M}
    transformToRootCache::CacheElement{Transform3D{C}, UpdateTransformToRoot{C}}
end
function call(functor::UpdateSpatialInertiaInWorld)
    transform(functor.body.inertia, get(functor.transformToRootCache))
end

immutable UpdateCompositeRigidBodyInertia{M, C}
    this::CacheElement{SpatialInertia{C}, UpdateSpatialInertiaInWorld{M, C}}
    children::Vector{CacheElement{SpatialInertia{C}, UpdateCompositeRigidBodyInertia{M, C}}}
end
function call(functor::UpdateCompositeRigidBodyInertia)
    ret = get(functor.this)
    for child in functor.children
        ret += get(child)
    end
    ret
end

immutable MechanismState{X<:Real, M<:Real, C<:Real} # immutable, but can change q and v and use the setdirty! method to have cache elements recomputed
    mechanism::Mechanism{M}
    q::Vector{X}
    v::Vector{X}
    qRanges::Dict{Joint, UnitRange{Int64}}
    vRanges::Dict{Joint, UnitRange{Int64}}
    transformsToParent::Dict{CartesianFrame3D, CacheElement{Transform3D{C}}}
    transformsToRoot::Dict{CartesianFrame3D, CacheElement{Transform3D{C}, UpdateTransformToRoot{C}}}
    twistsWrtWorld::Dict{RigidBody{M}, CacheElement{Twist{C}, UpdateTwistWithRespectToWorld{C}}}
    motionSubspaces::Dict{Joint, CacheElement{GeometricJacobian{C}}}
    biasAccelerations::Dict{RigidBody{M}, CacheElement{SpatialAcceleration{C}}}
    spatialInertias::Dict{RigidBody{M}, CacheElement{SpatialInertia{C}, UpdateSpatialInertiaInWorld{M, C}}}
    crbInertias::Dict{RigidBody{M}, CacheElement{SpatialInertia{C}, UpdateCompositeRigidBodyInertia{M, C}}}

    function MechanismState(m::Mechanism{M})
        sortedjoints = [x.edgeToParentData for x in m.toposortedTree[2 : end]]
        q = zeros(X, num_positions(m))
        v = zeros(X, num_velocities(m))
        qRanges = Dict{Joint, UnitRange{Int64}}()
        vRanges = Dict{Joint, UnitRange{Int64}}()
        sortedjoints = [x.edgeToParentData for x in m.toposortedTree[2 : end]]
        qStart = vStart = 1
        for joint in sortedjoints
            num_q = num_positions(joint)
            qRanges[joint] = qStart : qStart + num_q - 1
            qStart += num_q

            num_v = num_velocities(joint)
            vRanges[joint] = vStart : vStart + num_v - 1
            vStart += num_v
        end
        transformsToParent = Dict{CartesianFrame3D, CacheElement{Transform3D{C}}}()
        transformsToRoot = Dict{CartesianFrame3D, CacheElement{Transform3D{C}, UpdateTransformToRoot{C}}}()
        twistsWrtWorld = Dict{RigidBody{M}, CacheElement{Twist{C}, UpdateTwistWithRespectToWorld{C}}}()
        motionSubspaces = Dict{Joint, CacheElement{GeometricJacobian}}()
        biasAccelerations = Dict{RigidBody{M}, CacheElement{SpatialAcceleration{C}}}()
        spatialInertias = Dict{RigidBody{M}, CacheElement{SpatialInertia{C}, UpdateSpatialInertiaInWorld{M, C}}}()
        crbInertias = Dict{RigidBody{M}, CacheElement{SpatialInertia{C}, UpdateCompositeRigidBodyInertia{M, C}}}()
        new(m, q, v, qRanges, vRanges, transformsToParent, transformsToRoot, twistsWrtWorld, motionSubspaces, biasAccelerations, spatialInertias, crbInertias)
    end
end

num_positions(state::MechanismState) = length(state.q)
num_velocities(state::MechanismState) = length(state.v)
state_vector_eltype{X, M, C}(state::MechanismState{X, M, C}) = X
mechanism_eltype{X, M, C}(state::MechanismState{X, M, C}) = M
cache_eltype{X, M, C}(state::MechanismState{X, M, C}) = C
configuration(state::MechanismState, joint::Joint) = state.q[state.qRanges[joint]] #slice(state.q, state.qRanges[joint])
velocity(state::MechanismState, joint::Joint) = state.v[state.vRanges[joint]] #slice(state.v, state.vRanges[joint])

function setdirty!(state::MechanismState)
    for element in values(state.transformsToParent) setdirty!(element) end
    for element in values(state.transformsToRoot) setdirty!(element) end
    for element in values(state.twistsWrtWorld) setdirty!(element) end
    for element in values(state.motionSubspaces) setdirty!(element) end
    for element in values(state.biasAccelerations) setdirty!(element) end
    for element in values(state.spatialInertias) setdirty!(element) end
    for element in values(state.crbInertias) setdirty!(element) end
end

function zero_configuration!(state::MechanismState)
    X = eltype(state.q)
    for joint in joints(state.mechanism) slice(state.q, state.qRanges[joint])[:] = zero_configuration(joint, X) end
    setdirty!(state)
end
function zero_velocity!(state::MechanismState)
    X = eltype(state.v)
    state.v[:] = zero(X)
    setdirty!(state)
end
zero!(state::MechanismState) = begin zero_configuration!(state); zero_velocity!(state) end

function rand_configuration!(state::MechanismState)
    X = eltype(state.q)
    for joint in joints(state.mechanism) slice(state.q, state.qRanges[joint])[:] = rand_configuration(joint, X) end
    setdirty!(state)
end
function rand_velocity!(state::MechanismState)
    rand!(state.v)
    setdirty!(state)
end
rand!(state::MechanismState) = begin rand_configuration!(state); rand_velocity!(state) end

configuration_vector(state::MechanismState) = state.q
velocity_vector(state::MechanismState) = state.v
state_vector(state::MechanismState) = [configuration_vector(state); velocity_vector(state)]

configuration_vector{T}(state::MechanismState, path::Path{RigidBody{T}, Joint}) = vcat([state.q[state.qRanges[joint]] for joint in path.edgeData]...)
velocity_vector{T}(state::MechanismState, path::Path{RigidBody{T}, Joint}) = vcat([state.v[state.vRanges[joint]] for joint in path.edgeData]...)

function set_configuration!(state::MechanismState, joint::Joint, q::Vector)
    slice(state.q, state.qRanges[joint])[:] = q
    setdirty!(state)
end

function set_velocity!(state::MechanismState, joint::Joint, v::Vector)
    slice(state.v, state.vRanges[joint])[:] = v
    setdirty!(state)
end

function set_configuration!(state::MechanismState, q::Vector)
    copy!(state.q, q)
    setdirty!(state)
end

function set_velocity!(state::MechanismState, v::Vector)
    copy!(state.v, v)
    setdirty!(state)
end

function set!(state::MechanismState, x::Vector)
    nq = num_positions(state)
    nv = num_velocities(state)
    state.q[:] = slice(x, 1 : nq)
    state.v[:] = slice(x, (nq + 1) : nq + nv)
    setdirty!(state)
end

function add_frame!{X, M, C, T<:Real}(state::MechanismState{X, M, C}, t::Transform3D{T})
    # for fixed frames
    to_parent = CacheElement(convert(Transform3D{C}, t))
    parent_to_root = state.transformsToRoot[t.to]
    state.transformsToParent[t.from] = to_parent
    state.transformsToRoot[t.from] = CacheElement(Transform3D{C}, UpdateTransformToRoot{C}(parent_to_root, to_parent))
    setdirty!(state)
    return state
end

function add_frame!{X, M, C}(state::MechanismState{X, M, C}, updateTransformToParent)
    # for non-fixed frames
    to_parent = CacheElement(Transform3D{C}, () -> convert(Transform3D{C}, updateTransformToParent()))
    t = to_parent.data
    parent_to_root = state.transformsToRoot[t.to]
    state.transformsToParent[t.from] = to_parent
    state.transformsToRoot[t.from] = CacheElement(Transform3D{C}, UpdateTransformToRoot{C}(parent_to_root, to_parent))

    setdirty!(state)
    return state
end

function MechanismState{X, M}(::Type{X}, m::Mechanism{M})
    typealias C promote_type(X, M)
    root = root_body(m)
    state = MechanismState{X, M, C}(m)
    zero!(state)

    state.biasAccelerations[root] = CacheElement(zero(SpatialAcceleration{C}, root.frame, root.frame, root.frame))

    for vertex in m.toposortedTree
        body = vertex.vertexData
        if !isroot(vertex)
            parentVertex = vertex.parent
            parentBody = parentVertex.vertexData
            joint = vertex.edgeToParentData
            parentFrame = default_frame(m, parentBody)

            qJoint = slice(state.q, state.qRanges[joint])
            vJoint = slice(state.v, state.vRanges[joint])

            # frames
            add_frame!(state, m.jointToJointTransforms[joint])
            add_frame!(state, () -> joint_transform(joint, qJoint))

            # twists
            transformToRootCache = state.transformsToRoot[joint.frameAfter]
            parentTwistCache = state.twistsWrtWorld[parentBody]
            twistCache = CacheElement(Twist{C}, UpdateTwistWithRespectToWorld{C}(parentFrame, joint, qJoint, vJoint, transformToRootCache, parentTwistCache))
            state.twistsWrtWorld[body] = twistCache

            # motion subspaces
            update_motion_subspace = () -> begin
                S = transform(motion_subspace(joint, qJoint), get(transformToRootCache))
                GeometricJacobian(S.body, parentFrame, S.frame, S.angular, S.linear) # to make frames line up
            end
            state.motionSubspaces[joint] = CacheElement(GeometricJacobian{C}, update_motion_subspace)

            # bias accelerations
            parentBiasAccelerationCache = state.biasAccelerations[parentBody]

            update_bias_acceleration = () -> begin
                bias = bias_acceleration(joint, qJoint, vJoint)
                bias = SpatialAcceleration(joint.frameAfter, parentFrame, bias.frame, bias.angular, bias.linear) # to make the frames line up

                jointTwist = joint_twist(joint, qJoint, vJoint)::Twist{C}
                jointTwist = Twist(joint.frameAfter, parentFrame, jointTwist.frame, jointTwist.angular, jointTwist.linear) # to make the frames line up;

                bodyToRoot = get(transformToRootCache)
                rootToBody = inv(bodyToRoot)
                twistOfBodyWrtRoot = transform(get(twistCache), rootToBody)

                return get(parentBiasAccelerationCache) + transform(bias, bodyToRoot, twistOfBodyWrtRoot, jointTwist)
            end
            state.biasAccelerations[body] = CacheElement(SpatialAcceleration{C}, update_bias_acceleration)
        else
            state.transformsToRoot[root.frame] = CacheElement(Transform3D{C}(root.frame, root.frame), UpdateTransformToRoot{C}())
            state.twistsWrtWorld[body] = CacheElement(zero(Twist{C}, root.frame, root.frame, root.frame), UpdateTwistWithRespectToWorld{C}())
        end

        # additional body fixed frames
        for tf in m.bodyFixedFrameDefinitions[body]
            if tf.from != tf.to
                add_frame!(state, tf)
            end
        end

        if !isroot(vertex)
            # spatial inertias needs to be done after adding additional body fixed frames
            # because they may be expressed in one of those frames
            parentBody = vertex.parent.vertexData

            # inertias
            transformBodyToRootCache = state.transformsToRoot[body.inertia.frame]

            state.spatialInertias[body] = CacheElement(SpatialInertia{C}, UpdateSpatialInertiaInWorld(body, transformBodyToRootCache))
        end
    end

    # crb inertias
    for i = length(m.toposortedTree) : -1 : 2
        vertex = m.toposortedTree[i]
        body = vertex.vertexData
        children = [state.crbInertias[v.vertexData] for v in vertex.children]
        state.crbInertias[body] = CacheElement(SpatialInertia{C}, UpdateCompositeRigidBodyInertia(state.spatialInertias[body], children))
    end

    setdirty!(state)
    return state
end
