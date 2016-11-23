type CacheElement{T}
    dirty::Bool
    data::T
    CacheElement() = new(true)
end

function update!{T}(element::CacheElement{T}, data::T)
    element.data = data
    element.dirty = false
end

function get(element::CacheElement)
    element.dirty && error("Cache dirty.")
    element.data
end

function setdirty!(element::CacheElement)
    element.dirty = true
end

@inline isdirty(element::CacheElement) = element.dirty

function get!(element::CacheElement, updateFunction)
    if isdirty(element)
        update!(element, updateFunction())
    end
    element.data
end
