type FrameCache{T}
    root::CartesianFrame3D
    transformsToParent::Dict{CartesianFrame3D, CacheElement{Transform3D{T}}}
    transformsToRoot::Dict{CartesianFrame3D, CacheElement{Transform3D{T}}}
    dependents::Dict{CacheElement{Transform3D{T}}, Vector{CacheElement{Transform3D{T}}}}

    function FrameCache(root::CartesianFrame3D)
        transformsToParent = Dict{CartesianFrame3D, CacheElement{Transform3D{T}}}()
        toRoot = ImmutableCacheElement(Transform3D{T}(root, root))
        transformsToRoot = Dict{CartesianFrame3D, CacheElement{Transform3D{T}}}(root => toRoot)
        dependents = Dict{CacheElement{Transform3D{T}}, Vector{CacheElement{Transform3D{T}}}}(toRoot => [])
        new(root, transformsToParent, transformsToRoot, dependents)
    end
end

function setdirty!{T}(cache::FrameCache{T}, element::CacheElement{Transform3D{T}})
    setdirty!(element)
    for dependent in cache.dependents[element]
        setdirty!(cache, dependent)
    end
end

function setdirty!{T}(cache::FrameCache{T})
    for element in values(cache.transformsToParent)
        setdirty!(element)
    end
    for element in values(cache.transformsToRoot)
        setdirty!(element)
    end
end

function add_frame!{T}(cache::FrameCache{T}, t::Transform3D{T})
    # for fixed frames
    to_parent = ImmutableCacheElement(t)
    cache.dependents[to_parent] = []
    parent_to_root = cache.transformsToRoot[t.to]
    cache.transformsToParent[t.from] = to_parent

    to_root = MutableCacheElement(() ->  get(parent_to_root) * get(to_parent))
    cache.dependents[to_root] = []
    push!(cache.dependents[parent_to_root], to_root)
    cache.transformsToRoot[t.from] = to_root

    setdirty!(cache)
    return cache
end

function add_frame!{T}(cache::FrameCache{T}, updateTransformToParent::Function, dependencies...)
    # for non-fixed frames
    to_parent = MutableCacheElement(updateTransformToParent)
    cache.dependents[to_parent] = []
    for dependency in dependencies
        push!(cache.dependents[dependency], to_parent)
    end
    t = to_parent.data
    parent_to_root = cache.transformsToRoot[t.to]
    cache.transformsToParent[t.from] = to_parent

    to_root = MutableCacheElement(() -> get(parent_to_root) * get(to_parent))
    cache.dependents[to_root] = []
    push!(cache.dependents[to_parent], to_root)
    push!(cache.dependents[parent_to_root], to_root)
    cache.transformsToRoot[t.from] = to_root

    setdirty!(cache)
    return cache
end

function transform_to_parent{T}(cache::FrameCache{T}, frame::CartesianFrame3D)
    frame == cache.root && error("frame has no parent")
    return get(cache.transformsToParent[frame])
end

transform_to_root{T}(cache::FrameCache{T}, frame::CartesianFrame3D) = get(cache.transformsToRoot[frame])

function relative_transform{T}(cache::FrameCache{T}, from::CartesianFrame3D, to::CartesianFrame3D)
    return inv(transform_to_root(cache, to)) * transform_to_root(cache, from)
end
