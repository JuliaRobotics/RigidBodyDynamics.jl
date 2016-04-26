type CacheElement{T}
    data::T
    updateFunction::Function
    dirty::Bool

    CacheElement(t::T) = new(t, () -> t, false)
    CacheElement(updateFunction::Function) = new(updateFunction(), updateFunction, true)
end
CacheElement{T}(t::T) = CacheElement{T}(t)
CacheElement{T}(::Type{T}, updateFunction::Function) = CacheElement{T}(updateFunction)

function get{T}(element::CacheElement{T})
    if element.dirty
        element.data = element.updateFunction()
        element.dirty = false
    end
    return element.data
end
function setdirty!{T}(element::CacheElement{T})
    element.dirty = true
end
