type CacheElement{T}
    dirty::Bool
    data::T
    CacheElement() = new(true)
end

@inline function update!{T}(element::CacheElement{T}, data::T)
    element.data = data
    element.dirty = false
end

@inline function Base.get(element::CacheElement)
    element.dirty && error("Cache dirty.")
    element.data
end

@inline function setdirty!(element::CacheElement)
    element.dirty = true
end

@inline isdirty(element::CacheElement) = element.dirty

# The reason for doing this using a macro instead of using something like get!(element::CacheElement, callable)
# is that in all use cases so far, the callable would be a closure over some non-isbits types, meaning that
# creating the closure allocates.
# Example showing this allocation behavior: @benchmark (fun = () -> a.x) setup = a = Ref(rand(Int64))
macro cache_element_get!(element, updateExpr)
    ret = quote
        if isdirty($element)
            update!($element, $updateExpr)
        end
        get($element)
    end
    :($(esc(ret)))
end
