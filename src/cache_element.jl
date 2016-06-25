type CacheElement{T, F}
    data::T
    updateFunction::F
    dirty::Bool
    constant::Bool
    CacheElement(value::T, updateFunction::F, constant::Bool) = new(value, updateFunction, !constant, constant)
end
CacheElement{T, F}(::Type{T}, updateFunction::F) = CacheElement{T, F}(updateFunction(), updateFunction, false)
CacheElement{T, F}(value::T, updateFunction::F) = CacheElement{T, F}(value, updateFunction, true)
CacheElement{T}(value::T) = CacheElement{T, Function}(value, () -> value, true) # TODO: remove

function get{T, F}(element::CacheElement{T, F})
    if element.dirty
        element.data = element.updateFunction()
        element.dirty = false
    end
    element.data
end
function setdirty!(element::CacheElement)
    if !element.constant
        element.dirty = true
    end
end
