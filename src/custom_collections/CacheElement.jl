## CacheElement
mutable struct CacheElement{T}
    data::T
    dirty::Bool
    CacheElement(data::T) where {T} = new{T}(data, true)
end

@inline setdirty!(element::CacheElement) = (element.dirty = true; nothing)
@inline isdirty(element::CacheElement) = element.dirty
