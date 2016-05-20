immutable MechanismState{X<:Real, M<:Real, C<:Real} # immutable, but can change q and v and use the setdirty! method to have cache elements recomputed
    mechanism::Mechanism{M}
    q::OrderedDict{Joint, Vector{X}}
    v::OrderedDict{Joint, Vector{X}}
    velocityVectorStartIndices::Dict{Joint, Int64}
    transformsToParent::Dict{CartesianFrame3D, CacheElement{Transform3D{C}}}
    transformsToRoot::Dict{CartesianFrame3D, CacheElement{Transform3D{C}}}
    twistsWrtWorld::Dict{RigidBody{M}, CacheElement{Twist{C}}}
    motionSubspaces::Dict{Joint, CacheElement{GeometricJacobian{C}}}
    biasAccelerations::Dict{RigidBody{M}, CacheElement{SpatialAcceleration{C}}}
    spatialInertias::Dict{RigidBody{M}, CacheElement{SpatialInertia{C}}}
    crbInertias::Dict{RigidBody{M}, CacheElement{SpatialInertia{C}}}

    function MechanismState(m::Mechanism{M})
        sortedjoints = [x.edgeToParentData for x in m.toposortedTree[2 : end]]
        q = OrderedDict{Joint, Vector{X}}()
        v = OrderedDict{Joint, Vector{X}}()
        velocityVectorStartIndices = Dict{Joint, Int64}()
        startIndex = 1
        for joint in sortedjoints
            velocityVectorStartIndices[joint] = startIndex
            startIndex += num_velocities(joint)
            q[joint] = Vector{X}(num_positions(joint))
            v[joint] = Vector{X}(num_velocities(joint))
        end
        transformsToParent = Dict{CartesianFrame3D, CacheElement{Transform3D{C}}}()
        transformsToRoot = Dict{CartesianFrame3D, CacheElement{Transform3D{C}}}()
        twistsWrtWorld = Dict{RigidBody{M}, CacheElement{Twist{C}}}()
        motionSubspaces = Dict{Joint, CacheElement{GeometricJacobian}}()
        biasAccelerations = Dict{RigidBody{M}, CacheElement{SpatialAcceleration{C}}}()
        spatialInertias = Dict{RigidBody{M}, CacheElement{SpatialInertia{C}}}()
        crbInertias = Dict{RigidBody{M}, CacheElement{SpatialInertia{C}}}()
        new(m, q, v, velocityVectorStartIndices, transformsToParent, transformsToRoot, twistsWrtWorld, motionSubspaces, biasAccelerations, spatialInertias, crbInertias)
    end
end

num_positions(state::MechanismState) = num_positions(keys(state.q))
num_velocities(state::MechanismState) = num_velocities(keys(state.v))

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
    for joint in keys(state.q) state.q[joint][:] = zero_configuration(joint, eltype(state.q[joint])) end
    setdirty!(state)
end
function zero_velocity!(state::MechanismState)
    for joint in keys(state.v) state.v[joint][:] = zero(state.v[joint]) end
    setdirty!(state)
end
zero!(state::MechanismState) = begin zero_configuration!(state); zero_velocity!(state) end

function rand_configuration!(state::MechanismState)
    for joint in keys(state.q) state.q[joint][:] = rand_configuration(joint, eltype(state.q[joint])) end
    setdirty!(state)
end
function rand_velocity!(state::MechanismState)
    for joint in keys(state.v) state.v[joint][:] = rand!(state.v[joint]) end
    setdirty!(state)
end
rand!(state::MechanismState) = begin rand_configuration!(state); rand_velocity!(state) end

configuration_vector(state::MechanismState) = vcat([last(kv) for kv in state.q]...)
velocity_vector(state::MechanismState) = vcat([last(kv) for kv in state.v]...)
state_vector(state::MechanismState) = [configuration_vector(state); velocity_vector(state)]

configuration_vector{T}(state::MechanismState, path::Path{RigidBody{T}, Joint}) = vcat([state.q[joint] for joint in path.edgeData]...)
velocity_vector{T}(state::MechanismState, path::Path{RigidBody{T}, Joint}) = vcat([state.v[joint] for joint in path.edgeData]...)

function set_configuration!(state::MechanismState, joint::Joint, q::Vector)
    state.q[joint][:] = q
    setdirty!(state)
end

function set_velocity!(state::MechanismState, joint::Joint, v::Vector)
    state.v[joint][:] = v
    setdirty!(state)
end

function set_configuration!(state::MechanismState, q::Vector)
    start = 1
    for joint in keys(state.q)
        state.q[joint][:] = q[start : start + num_positions(joint) - 1]
        start += num_positions(joint)
    end
    setdirty!(state)
end

function set_velocity!(state::MechanismState, v::Vector)
    start = 1
    for joint in keys(state.q)
        state.v[joint][:] = v[start : start + num_velocities(joint) - 1]
        start += num_velocities(joint)
    end
    setdirty!(state)
end

function set!(state::MechanismState, x::Vector)
    start = 1
    for joint in keys(state.q)
        state.q[joint][:] = x[start : start + num_positions(joint) - 1]
        start += num_positions(joint)
    end
    for joint in keys(state.v)
        state.v[joint][:] = x[start : start + num_velocities(joint) - 1]
        start += num_velocities(joint)
    end
    setdirty!(state)
end

function add_frame!{X, M, C, T<:Real}(state::MechanismState{X, M, C}, t::Transform3D{T})
    # for fixed frames
    to_parent = CacheElement(convert(Transform3D{C}, t))
    parent_to_root = state.transformsToRoot[t.to]
    state.transformsToParent[t.from] = to_parent
    state.transformsToRoot[t.from] = CacheElement(Transform3D{C}, () -> get(parent_to_root) * get(to_parent))
    setdirty!(state)
    return state
end

function add_frame!{X, M, C}(state::MechanismState{X, M, C}, updateTransformToParent::Function)
    # for non-fixed frames
    to_parent = CacheElement(Transform3D{C}, () -> convert(Transform3D{C}, updateTransformToParent()))
    t = to_parent.data
    parent_to_root = state.transformsToRoot[t.to]
    state.transformsToParent[t.from] = to_parent
    state.transformsToRoot[t.from] = CacheElement(Transform3D{C}, () -> get(parent_to_root) * get(to_parent))

    setdirty!(state)
    return state
end

function MechanismState{X, M}(::Type{X}, m::Mechanism{M})
    typealias C promote_type(X, M)
    root = root_body(m)
    state = MechanismState{X, M, C}(m)
    zero!(state)

    state.transformsToRoot[root.frame] = CacheElement(Transform3D{C}(root.frame, root.frame))
    state.twistsWrtWorld[root] = CacheElement(zero(Twist{C}, root.frame, root.frame, root.frame))
    state.biasAccelerations[root] = CacheElement(zero(SpatialAcceleration{C}, root.frame, root.frame, root.frame))

    for vertex in m.toposortedTree
        body = vertex.vertexData
        if !isroot(vertex)
            parentVertex = vertex.parent
            parentBody = parentVertex.vertexData
            joint = vertex.edgeToParentData
            parentFrame = default_frame(m, parentBody)

            qJoint = state.q[joint]
            vJoint = state.v[joint]

            # frames
            add_frame!(state, m.jointToJointTransforms[joint])
            add_frame!(state, () -> joint_transform(joint, qJoint))

            # twists
            transformToRootCache = state.transformsToRoot[joint.frameAfter]
            parentTwistCache = state.twistsWrtWorld[parentBody]
            update_twist_wrt_world = () -> begin
                parentTwist = get(parentTwistCache)
                jointTwist = joint_twist(joint, qJoint, vJoint)
                jointTwist = Twist(joint.frameAfter, parentFrame, jointTwist.frame, jointTwist.angular, jointTwist.linear) # to make the frames line up;
                return parentTwist + transform(jointTwist, get(transformToRootCache))
            end
            state.twistsWrtWorld[body] = CacheElement(Twist{C}, update_twist_wrt_world)

            # motion subspaces
            update_motion_subspace = () -> begin
                S = transform(motion_subspace(joint, qJoint), get(transformToRootCache))
                S.base = parentFrame # to make frames line up
                return S
            end
            state.motionSubspaces[joint] = CacheElement(GeometricJacobian{C}, update_motion_subspace)

            # bias accelerations
            parentBiasAccelerationCache = state.biasAccelerations[parentBody]
            update_bias_acceleration = () -> begin
                bias = bias_acceleration(joint, qJoint, vJoint)
                bias = SpatialAcceleration(bias.body, parentFrame, bias.frame, bias.angular, bias.linear) # to make the frames line up
                return get(parentBiasAccelerationCache) + transform(state, bias, root.frame)
            end
            state.biasAccelerations[body] = CacheElement(SpatialAcceleration{C}, update_bias_acceleration)
        end

        # additional body fixed frames
        for transform in m.bodyFixedFrameDefinitions[body]
            if transform.from != transform.to
                add_frame!(state, transform)
            end
        end

        if !isroot(vertex)
            # spatial inertias needs to be done after adding additional body fixed frames
            # because they may be expressed in one of those frames
            parentBody = vertex.parent.vertexData

            # inertias
            transformBodyToRootCache = state.transformsToRoot[body.inertia.frame]
            spatialInertiaCache = CacheElement(SpatialInertia{C}, () -> transform(body.inertia, get(transformBodyToRootCache)))
            state.spatialInertias[body] = spatialInertiaCache
        end
    end

    # crb inertias
    for i = length(m.toposortedTree) : -1 : 2
        vertex = m.toposortedTree[i]
        body = vertex.vertexData
        caches = [state.spatialInertias[body]; [state.crbInertias[v.vertexData] for v in vertex.children]...]
        state.crbInertias[body] = CacheElement(SpatialInertia{C}, () -> sum(get, caches))
    end

    setdirty!(state)
    return state
end
