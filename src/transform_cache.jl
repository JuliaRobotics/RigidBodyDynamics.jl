immutable UpdateTransformToRoot{C}
    parentToRootCache::CacheElement{Transform3D{C}, UpdateTransformToRoot{C}}
    toParentCache::CacheElement{Transform3D{C}}
    UpdateTransformToRoot() = new()
    function UpdateTransformToRoot(parentToRootCache::CacheElement{Transform3D{C}, UpdateTransformToRoot{C}}, toParentCache::CacheElement{Transform3D{C}})
        new(parentToRootCache, toParentCache)
    end
end
@compat function (functor::UpdateTransformToRoot)()
    get(functor.parentToRootCache) * get(functor.toParentCache)
end

type TransformCache{C}
    transformsToParent::Dict{CartesianFrame3D, CacheElement{Transform3D{C}}}
    transformsToRoot::Dict{CartesianFrame3D, CacheElement{Transform3D{C}, UpdateTransformToRoot{C}}}

    function TransformCache()
        transformsToParent = Dict{CartesianFrame3D, CacheElement{Transform3D{C}}}()
        transformsToRoot = Dict{CartesianFrame3D, CacheElement{Transform3D{C}, UpdateTransformToRoot{C}}}()
        new(transformsToParent, transformsToRoot)
    end
end

transform_to_parent(cache::TransformCache, frame::CartesianFrame3D) = get(cache.transformsToParent[frame])
transform_to_root(cache::TransformCache, frame::CartesianFrame3D) = get(cache.transformsToRoot[frame])
relative_transform(cache::TransformCache, from::CartesianFrame3D, to::CartesianFrame3D) = inv(transform_to_root(cache, to)) * transform_to_root(cache, from)

function setdirty!(cache::TransformCache)
    for element in values(cache.transformsToParent) setdirty!(element) end
    for element in values(cache.transformsToRoot) setdirty!(element) end
end

function add_frame!{C, T<:Real}(cache::TransformCache{C}, t::Transform3D{T})
    # for fixed frames
    to_parent = CacheElement(convert(Transform3D{C}, t))
    parent_to_root = cache.transformsToRoot[t.to]
    cache.transformsToParent[t.from] = to_parent
    cache.transformsToRoot[t.from] = CacheElement(Transform3D{C}, UpdateTransformToRoot{C}(parent_to_root, to_parent))
    setdirty!(cache)
end

function add_frame!{C}(cache::TransformCache{C}, updateTransformToParent)
    # for non-fixed frames
    to_parent = CacheElement(Transform3D{C}, () -> convert(Transform3D{C}, updateTransformToParent()))
    t = to_parent.data
    parent_to_root = cache.transformsToRoot[t.to]
    cache.transformsToParent[t.from] = to_parent
    cache.transformsToRoot[t.from] = CacheElement(Transform3D{C}, UpdateTransformToRoot{C}(parent_to_root, to_parent))
    setdirty!(cache)
end

function TransformCache{M, Q}(m::Mechanism{M}, q::Vector{Q})
    C = promote_type(M, Q)
    cache = TransformCache{C}()

    for vertex in m.toposortedTree
        body = vertex.vertexData
        if !isroot(vertex)
            joint = vertex.edgeToParentData
            add_frame!(cache, m.jointToJointTransforms[joint])
            qJoint = view(q, m.qRanges[joint])
            add_frame!(cache, () -> joint_transform(joint, qJoint))
        else
            cache.transformsToRoot[body.frame] = CacheElement(Transform3D{C}(body.frame, body.frame), UpdateTransformToRoot{C}())
        end

        # additional body fixed frames
        for tf in m.bodyFixedFrameDefinitions[body]
            if tf.from != tf.to
                add_frame!(cache, tf)
            end
        end
    end
    cache
end
