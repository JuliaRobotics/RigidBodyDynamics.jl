type MechanismStateCache{C<:Real, M<:Real}
    mechanism::Mechanism{M}
    velocityVectorStartIndices::Dict{Joint, Int64}
    transformsToParent::Dict{CartesianFrame3D, CacheElement{Transform3D{C}}}
    transformsToRoot::Dict{CartesianFrame3D, CacheElement{Transform3D{C}}}
    twistsWrtWorld::Dict{RigidBody{M}, CacheElement{Twist{C}}}
    motionSubspaces::Dict{Joint, CacheElement{GeometricJacobian{C}}}
    biasAccelerations::Dict{RigidBody{M}, CacheElement{SpatialAcceleration{C}}}
    spatialInertias::Dict{RigidBody{M}, CacheElement{SpatialInertia{C}}}
    crbInertias::Dict{RigidBody{M}, CacheElement{SpatialInertia{C}}}

    function MechanismStateCache(m::Mechanism{M})
        sortedjoints = [x.edgeToParentData for x in m.toposortedTree[2 : end]] # TODO: just get from state vector data once they're merged
        velocityVectorStartIndices = Dict{Joint, Int64}()
        startIndex = 1
        for joint in sortedjoints
            velocityVectorStartIndices[joint] = startIndex
            startIndex += num_velocities(joint)
        end
        transformsToParent = Dict{CartesianFrame3D, CacheElement{Transform3D{C}}}()
        transformsToRoot = Dict{CartesianFrame3D, CacheElement{Transform3D{C}}}()
        twistsWrtWorld = Dict{RigidBody{M}, CacheElement{Twist{C}}}()
        motionSubspaces = Dict{Joint, CacheElement{GeometricJacobian}}()
        biasAccelerations = Dict{RigidBody{M}, CacheElement{SpatialAcceleration{C}}}()
        spatialInertias = Dict{RigidBody{M}, CacheElement{SpatialInertia{C}}}()
        crbInertias = Dict{RigidBody{M}, CacheElement{SpatialInertia{C}}}()
        new(m, velocityVectorStartIndices, transformsToParent, transformsToRoot, twistsWrtWorld, motionSubspaces, biasAccelerations, spatialInertias, crbInertias)
    end
end

function setdirty!(cache::MechanismStateCache)
    for element in values(cache.transformsToParent) setdirty!(element) end
    for element in values(cache.transformsToRoot) setdirty!(element) end
    for element in values(cache.twistsWrtWorld) setdirty!(element) end
    for element in values(cache.motionSubspaces) setdirty!(element) end
    for element in values(cache.biasAccelerations) setdirty!(element) end
    for element in values(cache.spatialInertias) setdirty!(element) end
    for element in values(cache.crbInertias) setdirty!(element) end
end

function add_frame!{C, T<:Real}(cache::MechanismStateCache{C}, t::Transform3D{T})
    # for fixed frames
    to_parent = CacheElement(convert(Transform3D{C}, t))
    parent_to_root = cache.transformsToRoot[t.to]
    cache.transformsToParent[t.from] = to_parent
    cache.transformsToRoot[t.from] = CacheElement(Transform3D{C}, () -> get(parent_to_root) * get(to_parent))
    setdirty!(cache)
    return cache
end

function add_frame!{C}(cache::MechanismStateCache{C}, updateTransformToParent::Function)
    # for non-fixed frames
    to_parent = CacheElement(Transform3D{C}, () -> convert(Transform3D{C}, updateTransformToParent()))
    t = to_parent.data
    parent_to_root = cache.transformsToRoot[t.to]
    cache.transformsToParent[t.from] = to_parent
    cache.transformsToRoot[t.from] = CacheElement(Transform3D{C}, () -> get(parent_to_root) * get(to_parent))

    setdirty!(cache)
    return cache
end

transform_to_parent(cache::MechanismStateCache, frame::CartesianFrame3D) = get(cache.transformsToParent[frame])
transform_to_root(cache::MechanismStateCache, frame::CartesianFrame3D) = get(cache.transformsToRoot[frame])
relative_transform(cache::MechanismStateCache, from::CartesianFrame3D, to::CartesianFrame3D) = inv(transform_to_root(cache, to)) * transform_to_root(cache, from)
twist_wrt_world{C, M}(cache::MechanismStateCache{C, M}, body::RigidBody{M}) = get(cache.twistsWrtWorld[body])
relative_twist{C, M}(cache::MechanismStateCache{C, M}, body::RigidBody{M}, base::RigidBody{M}) = -get(cache.twistsWrtWorld[base]) + get(cache.twistsWrtWorld[body])
function relative_twist(cache::MechanismStateCache, bodyFrame::CartesianFrame3D, baseFrame::CartesianFrame3D)
    twist = relative_twist(cache, cache.mechanism.bodyFixedFrameToBody[bodyFrame],  cache.mechanism.bodyFixedFrameToBody[baseFrame])
    return Twist(bodyFrame, baseFrame, twist.frame, twist.angular, twist.linear)
end
bias_acceleration{C, M}(cache::MechanismStateCache{C, M}, body::RigidBody{M}) = get(cache.biasAccelerations[body])
motion_subspace(cache::MechanismStateCache, joint::Joint) = get(cache.motionSubspaces[joint])
spatial_inertia{C, M}(cache::MechanismStateCache{C, M}, body::RigidBody{M}) = get(cache.spatialInertias[body])
crb_inertia{C, M}(cache::MechanismStateCache{C, M}, body::RigidBody{M}) = get(cache.crbInertias[body])

function transform(cache::MechanismStateCache, point::Point3D, to::CartesianFrame3D)
    point.frame == to && return point # nothing to be done
    relative_transform(cache, point.frame, to) * point
end

function transform(cache::MechanismStateCache, vector::FreeVector3D, to::CartesianFrame3D)
    vector.frame == to && return vector # nothing to be done
    relative_transform(cache, vector.frame, to) * vector
end

function transform(cache::MechanismStateCache, twist::Twist, to::CartesianFrame3D)
    twist.frame == to && return twist # nothing to be done
    transform(twist, relative_transform(cache, twist.frame, to))
end

function transform(cache::MechanismStateCache, wrench::Wrench, to::CartesianFrame3D)
    wrench.frame == to && return wrench # nothing to be done
    transform(wrench, relative_transform(cache, wrench.frame, to))
end

function transform(cache::MechanismStateCache, accel::SpatialAcceleration, to::CartesianFrame3D)
    accel.frame == to && return accel # nothing to be done
    oldToRoot = transform_to_root(cache, accel.frame)
    rootToOld = inv(oldToRoot)
    twistOfBodyWrtBase = transform(relative_twist(cache, accel.body, accel.base), rootToOld)
    twistOfOldWrtNew = transform(relative_twist(cache, accel.frame, to), rootToOld)
    oldToNew = inv(transform_to_root(cache, to)) * oldToRoot
    return transform(accel, oldToNew, twistOfOldWrtNew, twistOfBodyWrtBase)
end

function MechanismStateCache{M, X}(m::Mechanism{M}, x::MechanismState{X})
    typealias C promote_type(M, X)
    root = root_body(m)
    cache = MechanismStateCache{C, M}(m)

    rootTransform = CacheElement(Transform3D{C}(root.frame, root.frame))
    cache.transformsToRoot[root.frame] = rootTransform

    rootTwist = zero(Twist{C}, root.frame, root.frame, root.frame)
    cache.twistsWrtWorld[root] = CacheElement(rootTwist)

    rootBiasAcceleration = SpatialAcceleration(root.frame, root.frame, root.frame, zero(Vec{3, C}), convert(Vec{3, C}, m.gravity))
    cache.biasAccelerations[root] = CacheElement(rootBiasAcceleration)

    for vertex in m.toposortedTree
        body = vertex.vertexData
        if !isroot(vertex)
            parentVertex = vertex.parent
            parentBody = parentVertex.vertexData
            joint = vertex.edgeToParentData
            parentFrame = default_frame(m, parentBody)

            qJoint = x.q[joint]
            vJoint = x.v[joint]

            # frames
            add_frame!(cache, m.jointToJointTransforms[joint])
            add_frame!(cache, () -> joint_transform(joint, qJoint))

            # twists
            transformToRootCache = cache.transformsToRoot[joint.frameAfter]
            parentTwistCache = cache.twistsWrtWorld[parentBody]
            update_twist_wrt_world = () -> begin
                parentTwist = get(parentTwistCache)
                jointTwist = joint_twist(joint, qJoint, vJoint)
                jointTwist = Twist(joint.frameAfter, parentFrame, jointTwist.frame, jointTwist.angular, jointTwist.linear) # to make the frames line up;
                return parentTwist + transform(jointTwist, get(transformToRootCache))
            end
            cache.twistsWrtWorld[body] = CacheElement(Twist{C}, update_twist_wrt_world)

            # motion subspaces
            update_motion_subspace = () -> begin
                S = transform(motion_subspace(joint, qJoint), get(transformToRootCache))
                S.base = parentFrame # to make frames line up
                return S
            end
            cache.motionSubspaces[joint] = CacheElement(GeometricJacobian{C}, update_motion_subspace)

            # bias accelerations
            # if isroot(parentBody)
            #     update_bias_acceleration = () -> begin
            #         bias = bias_acceleration(joint, qJoint, vJoint)
            #         bias = SpatialAcceleration(bias.body, parentFrame, bias.frame, bias.angular, bias.linear) # to make the frames line up
            #         return transform(cache, bias, root.frame)
            #     end
            # else
            parentBiasAccelerationCache = cache.biasAccelerations[parentBody]
            update_bias_acceleration = () -> begin
                bias = bias_acceleration(joint, qJoint, vJoint)
                bias = SpatialAcceleration(bias.body, parentFrame, bias.frame, bias.angular, bias.linear) # to make the frames line up
                return transform(cache, bias, root.frame)
            end
            # end
            cache.biasAccelerations[body] = CacheElement(SpatialAcceleration{C}, update_bias_acceleration)
        end

        # additional body fixed frames
        for transform in m.bodyFixedFrameDefinitions[body]
            if transform.from != transform.to
                add_frame!(cache, transform)
            end
        end

        if !isroot(vertex)
            # spatial inertias needs to be done after adding additional body fixed frames
            # because they may be expressed in one of those frames
            parentBody = vertex.parent.vertexData

            # inertias
            transformBodyToRootCache = cache.transformsToRoot[body.inertia.frame]
            spatialInertiaCache = CacheElement(SpatialInertia{C}, () -> transform(body.inertia, get(transformBodyToRootCache)))
            cache.spatialInertias[body] = spatialInertiaCache
        end
    end

    # crb inertias
    for i = length(m.toposortedTree) : -1 : 2
        vertex = m.toposortedTree[i]
        body = vertex.vertexData
        caches = [cache.spatialInertias[body]; [cache.crbInertias[v.vertexData] for v in vertex.children]...]
        cache.crbInertias[body] = CacheElement(SpatialInertia{C}, () -> sum(get, caches))
    end

    setdirty!(cache)
    return cache
end
