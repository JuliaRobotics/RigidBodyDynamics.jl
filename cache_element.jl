abstract CacheElement{T}

immutable ImmutableCacheElement{T} <: CacheElement{T}
    data::T
end

get{T}(element::ImmutableCacheElement{T}) = element.data
function setdirty!{T}(element::ImmutableCacheElement{T})
end

type MutableCacheElement{T} <: CacheElement{T}
    data::T
    updateFunction::Function
    dirty::Bool

    MutableCacheElement(data::T, updateFunction::Function) = new(data, updateFunction, true)
end
MutableCacheElement{T}(data::T, updateFunction::Function) = MutableCacheElement{T}(data, updateFunction)
MutableCacheElement(updateFunction::Function) = MutableCacheElement(updateFunction(), updateFunction)

function get{T}(element::MutableCacheElement{T})
    if element.dirty
        element.data = element.updateFunction()
        element.dirty = false
    end
    return element.data
end
function setdirty!{T}(element::MutableCacheElement{T})
    element.dirty = true
end
