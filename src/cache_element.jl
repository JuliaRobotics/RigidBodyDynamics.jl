# immutable Identity{T}
#     value::T
# end
# call(id::Identity) = id.value

type CacheElement{T, F}
    updateFunction::F
    data::T
    dirty::Bool
end
CacheElement{T, F}(::Type{T}, updateFunction::F) = CacheElement{T, F}(updateFunction, updateFunction(), true)
# CacheElement{T}(value::T) = CacheElement(Identity(value), value, false)
CacheElement{T}(value::T) = CacheElement{T, Function}(() -> value, value, false)

function get{T, F}(element::CacheElement{T, F})
    if element.dirty
        element.data = element.updateFunction()
        element.dirty = false
    end
    element.data
end
setdirty!(element::CacheElement) = element.dirty = true
