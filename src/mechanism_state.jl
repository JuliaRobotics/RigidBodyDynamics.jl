immutable UpdateTwistAndBias{C}
    parentFrame::CartesianFrame3D
    joint::Joint
    qJoint::SubArray{C,1,Array{C,1},Tuple{UnitRange{Int64}},true}
    vJoint::SubArray{C,1,Array{C,1},Tuple{UnitRange{Int64}},true}
    transformToRootCache::CacheElement{Transform3D{C}, UpdateTransformToRoot{C}}
    parentCache::CacheElement{Tuple{Twist{C}, SpatialAcceleration{C}}, UpdateTwistAndBias{C}}

    UpdateTwistAndBias() = new()
    function UpdateTwistAndBias(parentFrame::CartesianFrame3D, joint::Joint, qJoint::SubArray{C,1,Array{C,1},Tuple{UnitRange{Int64}},true}, vJoint::SubArray{C,1,Array{C,1},Tuple{UnitRange{Int64}},true}, transformToRootCache::CacheElement{Transform3D{C}, UpdateTransformToRoot{C}}, parentCache::CacheElement{Tuple{Twist{C}, SpatialAcceleration{C}}, UpdateTwistAndBias{C}})
        new(parentFrame, joint, qJoint, vJoint, transformToRootCache, parentCache)
    end
end
@compat function (functor::UpdateTwistAndBias{C}){C}()
    parentFrame = functor.parentFrame
    joint = functor.joint
    qJoint = functor.qJoint
    vJoint = functor.vJoint
    parent = get(functor.parentCache)
    parentTwist = parent[1]
    parentBias = parent[2]

    bodyToRoot = get(functor.transformToRootCache)
    jointTwist = joint_twist(joint, qJoint, vJoint)#::Twist{C}
    jointTwist = Twist(joint.frameAfter, parentFrame, jointTwist.frame, jointTwist.angular, jointTwist.linear) # to make the frames line up;
    twist = parentTwist + transform(jointTwist, bodyToRoot)

    bias = bias_acceleration(joint, qJoint, vJoint)#::SpatialAcceleration{C}
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
@compat function (functor::UpdateSpatialInertiaInWorld)()
    transform(spatial_inertia(functor.body), get(functor.transformToRootCache))
end

immutable UpdateCompositeRigidBodyInertia{M, C}
    this::CacheElement{SpatialInertia{C}, UpdateSpatialInertiaInWorld{M, C}}
    children::Vector{CacheElement{SpatialInertia{C}, UpdateCompositeRigidBodyInertia{M, C}}}
end
@compat function (functor::UpdateCompositeRigidBodyInertia)()
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
    transformCache::TransformCache{C}
    twistsAndBiases::Dict{RigidBody{M}, CacheElement{Tuple{Twist{C}, SpatialAcceleration{C}}, UpdateTwistAndBias{C}}}
    motionSubspaces::Dict{Joint, CacheElement{GeometricJacobian}}
    spatialInertias::Dict{RigidBody{M}, CacheElement{SpatialInertia{C}, UpdateSpatialInertiaInWorld{M, C}}}
    crbInertias::Dict{RigidBody{M}, CacheElement{SpatialInertia{C}, UpdateCompositeRigidBodyInertia{M, C}}}

    function MechanismState(m::Mechanism{M})
        q = zeros(X, num_positions(m))
        v = zeros(X, num_velocities(m))
        transformCache = TransformCache(m, q)
        twistsAndBiases = Dict{RigidBody{M}, CacheElement{Tuple{Twist{C}, SpatialAcceleration{C}}, UpdateTwistAndBias{C}}}()
        motionSubspaces = Dict{Joint, CacheElement{GeometricJacobian}}()
        spatialInertias = Dict{RigidBody{M}, CacheElement{SpatialInertia{C}, UpdateSpatialInertiaInWorld{M, C}}}()
        crbInertias = Dict{RigidBody{M}, CacheElement{SpatialInertia{C}, UpdateCompositeRigidBodyInertia{M, C}}}()
        new(m, q, v, transformCache, twistsAndBiases, motionSubspaces, spatialInertias, crbInertias)
    end
end

show{X, M, C}(io::IO, ::MechanismState{X, M, C}) = print(io, "MechanismState{$X, $M, $C}(…)")
num_positions(state::MechanismState) = length(state.q)
num_velocities(state::MechanismState) = length(state.v)
state_vector_eltype{X, M, C}(state::MechanismState{X, M, C}) = X
mechanism_eltype{X, M, C}(state::MechanismState{X, M, C}) = M
cache_eltype{X, M, C}(state::MechanismState{X, M, C}) = C
configuration(state::MechanismState, joint::Joint) = view(state.q, state.mechanism.qRanges[joint])
velocity(state::MechanismState, joint::Joint) = view(state.v, state.mechanism.vRanges[joint])

function setdirty!(state::MechanismState)
    setdirty!(state.transformCache)
    for element in values(state.twistsAndBiases) setdirty!(element) end
    for element in values(state.motionSubspaces) setdirty!(element) end
    for element in values(state.spatialInertias) setdirty!(element) end
    for element in values(state.crbInertias) setdirty!(element) end
end

function zero_configuration!(state::MechanismState)
    X = eltype(state.q)
    for joint in joints(state.mechanism) view(state.q, state.mechanism.qRanges[joint])[:] = zero_configuration(joint, X) end
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
    for joint in joints(state.mechanism) view(state.q, state.mechanism.qRanges[joint])[:] = rand_configuration(joint, X) end
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

configuration_vector{T}(state::MechanismState, path::Path{RigidBody{T}, Joint}) = vcat([state.q[state.mechanism.qRanges[joint]] for joint in path.edgeData]...)
velocity_vector{T}(state::MechanismState, path::Path{RigidBody{T}, Joint}) = vcat([state.v[state.mechanism.vRanges[joint]] for joint in path.edgeData]...)

function set_configuration!(state::MechanismState, joint::Joint, q::AbstractVector)
    view(state.q, state.mechanism.qRanges[joint])[:] = q
    setdirty!(state)
end

function set_velocity!(state::MechanismState, joint::Joint, v::AbstractVector)
    view(state.v, state.mechanism.vRanges[joint])[:] = v
    setdirty!(state)
end

function set_configuration!(state::MechanismState, q::AbstractVector)
    copy!(state.q, q)
    setdirty!(state)
end

function set_velocity!(state::MechanismState, v::AbstractVector)
    copy!(state.v, v)
    setdirty!(state)
end

function set!(state::MechanismState, x::AbstractVector)
    nq = num_positions(state)
    nv = num_velocities(state)
    length(x) == nq + nv || error("wrong size")
    unsafe_copy!(state.q, 1, x, 1, nq)
    unsafe_copy!(state.v, 1, x, nq + 1, nv)
    setdirty!(state)
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

            qJoint = view(state.q, state.mechanism.qRanges[joint])
            vJoint = view(state.v, state.mechanism.vRanges[joint])

            # twists and bias accelerations
            transformToRootCache = state.transformCache.transformsToRoot[joint.frameAfter]
            parentCache = state.twistsAndBiases[parentBody]
            twistCache = CacheElement(Tuple{Twist{C}, SpatialAcceleration{C}}, UpdateTwistAndBias{C}(parentFrame, joint, qJoint, vJoint, transformToRootCache, parentCache))
            state.twistsAndBiases[body] = twistCache

            # motion subspaces
            update_motion_subspace = () -> begin
                S = transform(motion_subspace(joint, qJoint), get(transformToRootCache))
                GeometricJacobian(S.body, parentFrame, S.frame, S.angular, S.linear) # to make frames line up
            end
            state.motionSubspaces[joint] = CacheElement(GeometricJacobian, update_motion_subspace)
        else
            rootTwist = zero(Twist{C}, root.frame, root.frame, root.frame)
            rootBias = zero(SpatialAcceleration{C}, root.frame, root.frame, root.frame)
            state.twistsAndBiases[body] = CacheElement((rootTwist, rootBias), UpdateTwistAndBias{C}())
        end

        if !isroot(vertex)
            # spatial inertias needs to be done after adding additional body fixed frames
            # because they may be expressed in one of those frames
            parentBody = vertex.parent.vertexData

            # inertias
            transformBodyToRootCache = state.transformCache.transformsToRoot[spatial_inertia(body).frame]

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
