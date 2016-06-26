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

immutable UpdateTwistAndBias{C}
    parentFrame::CartesianFrame3D
    joint::Joint
    qJoint
    vJoint
    transformToRootCache::CacheElement{Transform3D{C}, UpdateTransformToRoot{C}}
    parentCache::CacheElement{Tuple{Twist{C}, SpatialAcceleration{C}}, UpdateTwistAndBias{C}}

    UpdateTwistAndBias() = new()
    function UpdateTwistAndBias(parentFrame::CartesianFrame3D, joint::Joint, qJoint::AbstractVector, vJoint::AbstractVector, transformToRootCache::CacheElement{Transform3D{C}, UpdateTransformToRoot{C}}, parentCache::CacheElement{Tuple{Twist{C}, SpatialAcceleration{C}}, UpdateTwistAndBias{C}})
        new(parentFrame, joint, qJoint, vJoint, transformToRootCache, parentCache)
    end
end
function call{C}(functor::UpdateTwistAndBias{C})
    parentFrame = functor.parentFrame
    joint = functor.joint
    qJoint = functor.qJoint
    vJoint = functor.vJoint
    parent = get(functor.parentCache)
    parentTwist = parent[1]
    parentBias = parent[2]

    bodyToRoot = get(functor.transformToRootCache)
    jointTwist = joint_twist(joint, qJoint, vJoint)::Twist{C}
    jointTwist = Twist(joint.frameAfter, parentFrame, jointTwist.frame, jointTwist.angular, jointTwist.linear) # to make the frames line up;
    twist = parentTwist + transform(jointTwist, bodyToRoot)

    bias = bias_acceleration(joint, qJoint, vJoint)::SpatialAcceleration{C}
    bias = SpatialAcceleration(joint.frameAfter, parentFrame, bias.frame, bias.angular, bias.linear) # to make the frames line up
    rootToBody = inv(bodyToRoot)
    twistOfBodyWrtRoot = transform(twist, rootToBody)
    bias = parentBias + transform(bias, bodyToRoot, twistOfBodyWrtRoot, jointTwist)

    (twist, bias)
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
    twistsAndBiases::Dict{RigidBody{M}, CacheElement{Tuple{Twist{C}, SpatialAcceleration{C}}, UpdateTwistAndBias{C}}}
    motionSubspaces::Dict{Joint, CacheElement{GeometricJacobian{C}}}
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
        twistsAndBiases = Dict{RigidBody{M}, CacheElement{Tuple{Twist{C}, SpatialAcceleration{C}}, UpdateTwistAndBias{C}}}()
        motionSubspaces = Dict{Joint, CacheElement{GeometricJacobian}}()
        spatialInertias = Dict{RigidBody{M}, CacheElement{SpatialInertia{C}, UpdateSpatialInertiaInWorld{M, C}}}()
        crbInertias = Dict{RigidBody{M}, CacheElement{SpatialInertia{C}, UpdateCompositeRigidBodyInertia{M, C}}}()
        new(m, q, v, qRanges, vRanges, transformsToParent, transformsToRoot, twistsAndBiases, motionSubspaces, spatialInertias, crbInertias)
    end
end

num_positions(state::MechanismState) = length(state.q)
num_velocities(state::MechanismState) = length(state.v)
state_vector_eltype{X, M, C}(state::MechanismState{X, M, C}) = X
mechanism_eltype{X, M, C}(state::MechanismState{X, M, C}) = M
cache_eltype{X, M, C}(state::MechanismState{X, M, C}) = C
configuration(state::MechanismState, joint::Joint) = state.q[state.qRanges[joint]] #view(state.q, state.qRanges[joint])
velocity(state::MechanismState, joint::Joint) = state.v[state.vRanges[joint]] #view(state.v, state.vRanges[joint])

function setdirty!(state::MechanismState)
    for element in values(state.transformsToParent) setdirty!(element) end
    for element in values(state.transformsToRoot) setdirty!(element) end
    for element in values(state.twistsAndBiases) setdirty!(element) end
    for element in values(state.motionSubspaces) setdirty!(element) end
    for element in values(state.spatialInertias) setdirty!(element) end
    for element in values(state.crbInertias) setdirty!(element) end
end

function zero_configuration!(state::MechanismState)
    X = eltype(state.q)
    for joint in joints(state.mechanism) view(state.q, state.qRanges[joint])[:] = zero_configuration(joint, X) end
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
    for joint in joints(state.mechanism) view(state.q, state.qRanges[joint])[:] = rand_configuration(joint, X) end
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
    view(state.q, state.qRanges[joint])[:] = q
    setdirty!(state)
end

function set_velocity!(state::MechanismState, joint::Joint, v::Vector)
    view(state.v, state.vRanges[joint])[:] = v
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
    copy!(state.q, 1, x, 1, nq)
    copy!(state.v, 1, x, nq + 1, nv)
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

    for vertex in m.toposortedTree
        body = vertex.vertexData
        if !isroot(vertex)
            parentVertex = vertex.parent
            parentBody = parentVertex.vertexData
            joint = vertex.edgeToParentData
            parentFrame = default_frame(m, parentBody)

            qJoint = view(state.q, state.qRanges[joint])
            vJoint = view(state.v, state.vRanges[joint])

            # frames
            add_frame!(state, m.jointToJointTransforms[joint])
            add_frame!(state, () -> joint_transform(joint, qJoint))

            # twists and bias accelerations
            transformToRootCache = state.transformsToRoot[joint.frameAfter]
            parentCache = state.twistsAndBiases[parentBody]
            twistCache = CacheElement(Tuple{Twist{C}, SpatialAcceleration{C}}, UpdateTwistAndBias{C}(parentFrame, joint, qJoint, vJoint, transformToRootCache, parentCache))
            state.twistsAndBiases[body] = twistCache

            # motion subspaces
            update_motion_subspace = () -> begin
                S = transform(motion_subspace(joint, qJoint), get(transformToRootCache))
                GeometricJacobian(S.body, parentFrame, S.frame, S.angular, S.linear) # to make frames line up
            end
            state.motionSubspaces[joint] = CacheElement(GeometricJacobian{C}, update_motion_subspace)
        else
            state.transformsToRoot[root.frame] = CacheElement(Transform3D{C}(root.frame, root.frame), UpdateTransformToRoot{C}())
            rootTwist = zero(Twist{C}, root.frame, root.frame, root.frame)
            rootBias = zero(SpatialAcceleration{C}, root.frame, root.frame, root.frame)
            state.twistsAndBiases[body] = CacheElement((rootTwist, rootBias), UpdateTwistAndBias{C}())
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
