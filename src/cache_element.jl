mutable struct CacheElement{T}
    data::T
    dirty::Bool
    CacheElement(data::T) where {T} = new{T}(data, true)
end

@inline setdirty!(element::CacheElement) = (element.dirty = true; nothing)
@inline isdirty(element::CacheElement) = element.dirty

@inline function update!(element::CacheElement, f!, args...)
    if element.dirty
        f!(element.data, args...)
        element.dirty = false
    end
    nothing
end
