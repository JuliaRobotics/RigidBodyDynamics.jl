type MechanismStateCache{C<:Real, M<:Real}
    mechanism::Mechanism{M}
    velocityVectorStartIndices::Dict{Joint, Int64}
    transformsToParent::Dict{CartesianFrame3D, CacheElement{Transform3D{C}}}
    transformsToRoot::Dict{CartesianFrame3D, CacheElement{Transform3D{C}}}
    twistsWrtWorld::Dict{RigidBody{M}, CacheElement{Twist{C}}}
    motionSubspaces::Dict{Joint, MutableCacheElement{GeometricJacobian{C}}}
    spatialInertias::Dict{RigidBody{M}, MutableCacheElement{SpatialInertia{C}}}
    crbInertias::Dict{RigidBody{M}, MutableCacheElement{SpatialInertia{C}}}

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
        motionSubspaces = Dict{Joint, MutableCacheElement{GeometricJacobian}}()
        spatialInertias = Dict{RigidBody{M}, MutableCacheElement{SpatialInertia{C}}}()
        crbInertias = Dict{RigidBody{M}, MutableCacheElement{SpatialInertia{C}}}()
        new(m, velocityVectorStartIndices, transformsToParent, transformsToRoot, twistsWrtWorld, motionSubspaces, spatialInertias, crbInertias)
    end
end

function setdirty!(cache::MechanismStateCache)
    for element in values(cache.transformsToParent) setdirty!(element) end
    for element in values(cache.transformsToRoot) setdirty!(element) end
    for element in values(cache.twistsWrtWorld) setdirty!(element) end
    for element in values(cache.motionSubspaces) setdirty!(element) end
    for element in values(cache.spatialInertias) setdirty!(element) end
    for element in values(cache.crbInertias) setdirty!(element) end
end

function add_frame!{C}(cache::MechanismStateCache{C}, t::Transform3D{C})
    # for fixed frames
    to_parent = ImmutableCacheElement(t)
    parent_to_root = cache.transformsToRoot[t.to]
    cache.transformsToParent[t.from] = to_parent
    cache.transformsToRoot[t.from] = MutableCacheElement(() -> get(parent_to_root) * get(to_parent))
    setdirty!(cache)
    return cache
end

function add_frame!{C}(cache::MechanismStateCache{C}, updateTransformToParent::Function)
    # for non-fixed frames
    to_parent = MutableCacheElement(updateTransformToParent)
    t = to_parent.data
    parent_to_root = cache.transformsToRoot[t.to]
    cache.transformsToParent[t.from] = to_parent
    cache.transformsToRoot[t.from] = MutableCacheElement(() -> get(parent_to_root) * get(to_parent))

    setdirty!(cache)
    return cache
end

transform_to_parent(cache::MechanismStateCache, frame::CartesianFrame3D) = get(cache.transformsToParent[frame])
transform_to_root(cache::MechanismStateCache, frame::CartesianFrame3D) = get(cache.transformsToRoot[frame])
relative_transform(cache::MechanismStateCache, from::CartesianFrame3D, to::CartesianFrame3D) = inv(transform_to_root(cache, to)) * transform_to_root(cache, from)
twist_wrt_world(cache::MechanismStateCache, body::RigidBody) = get(cache.twistsWrtWorld[body])
relative_twist(cache::MechanismStateCache, body::RigidBody, base::RigidBody) = -get(cache.twistsWrtWorld[base]) + get(cache.twistsWrtWorld[body])
relative_twist(cache::MechanismStateCache, bodyFrame::CartesianFrame3D, baseFrame::CartesianFrame3D) = relative_twist(cache, cache.mechanism.bodyFixedFrameToBody[bodyFrame],  cache.mechanism.bodyFixedFrameToBody[baseFrame])
motion_subspace(cache::MechanismStateCache, joint::Joint) = get(cache.motionSubspaces[joint])
spatial_inertia(cache::MechanismStateCache, body::RigidBody) = get(cache.spatialInertias[body])
crb_inertia(cache::MechanismStateCache, body::RigidBody) = get(cache.crbInertias[body])

function transform{C}(cache::MechanismStateCache{C}, point::Point3D{C}, to::CartesianFrame3D)
    point.frame == to && return point # nothing to be done
    relative_transform(cache, point.frame, to) * point
end

function transform{C}(cache::MechanismStateCache{C}, twist::Twist{C}, to::CartesianFrame3D)
    twist.frame == to && return twist # nothing to be done
    transform(twist, relative_transform(cache, t.frame, to))
end

function transform{C}(cache::MechanismStateCache{C}, accel::SpatialAcceleration{C}, to::CartesianFrame3D)
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

    rootTransform = ImmutableCacheElement(Transform3D{C}(root.frame, root.frame))
    cache.transformsToRoot[root.frame] = rootTransform

    rootTwist = zero(Twist{C}, root.frame, root.frame, root.frame)
    cache.twistsWrtWorld[root] = ImmutableCacheElement(rootTwist)

    for vertex in m.toposortedTree
        body = vertex.vertexData
        if !isroot(vertex)
            parentVertex = vertex.parent
            parentBody = parentVertex.vertexData
            joint = vertex.edgeToParentData
            parentFrame = isroot(parentVertex) ? parentBody.frame : parentVertex.edgeToParentData.frameAfter

            qJoint = x.q[joint]
            vJoint = x.v[joint]

            # frames
            add_frame!(cache, m.jointToJointTransforms[joint])
            add_frame!(cache, () -> joint_transform(joint, qJoint))

            # twists
            transformToRootCache = cache.transformsToRoot[joint.frameAfter]
            parentTwistCache = cache.twistsWrtWorld[parentBody]
            updateTwistWrtWorld = () -> begin
                parentTwist = get(parentTwistCache)
                jointTwist = joint_twist(joint, qJoint, vJoint)
                jointTwist = Twist(joint.frameAfter, parentTwist.body, jointTwist.frame, jointTwist.angular, jointTwist.linear) # to make the frames line up;
                return parentTwist + transform(jointTwist, get(transformToRootCache))
            end
            cache.twistsWrtWorld[body] = MutableCacheElement(updateTwistWrtWorld)

            # motion subspaces
            updateMotionSubspace = () -> begin
                S = transform(motion_subspace(joint, qJoint), get(transformToRootCache))
                S.base = parentFrame # to make frames line up
                return S
            end
            cache.motionSubspaces[joint] = MutableCacheElement(updateMotionSubspace)
        end

        # additional body fixed frames
        for transform in m.bodyFixedFrameDefinitions[body]
            if transform.from != transform.to
                add_frame!(cache, transform)
            end
        end

        if !isroot(vertex)
            parentBody = vertex.parent.vertexData

            # inertias
            transformBodyToRootCache = cache.transformsToRoot[body.inertia.frame]
            updateSpatialInertia = () -> transform(body.inertia, get(transformBodyToRootCache))
            spatialInertiaCache = MutableCacheElement(updateSpatialInertia)
            cache.spatialInertias[body] = spatialInertiaCache

            # crb inertias
            if isroot(parentBody)
                updateCRBInertia = () -> get(spatialInertiaCache)
                cache.crbInertias[body] = MutableCacheElement(updateCRBInertia)
            else
                parentCRBInertia = cache.crbInertias[parentBody]
                updateCRBInertia = () -> get(parentCRBInertia) + get(spatialInertiaCache)
                cache.crbInertias[body] = MutableCacheElement(updateCRBInertia)
            end
        end
    end

    setdirty!(cache)
    return cache
end
