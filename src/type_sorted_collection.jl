function sort_by_type(A)
    ret = Dict()
    for x in A
        T = typeof(x)
        vector = get!(Vector{T}, ret, T)
        push!(vector, x)
    end
    ret
end

struct TypeSortedCollection{I, D<:Tuple{Vararg{Vector{T} where T}}}
    indexfun::I
    data::D

    function TypeSortedCollection(indexfun::I, x) where {I}
        dict = sort_by_type(x)
        data = tuple(values(dict)...)
        new{I, typeof(data)}(indexfun, data)
    end
end

Base.length(x::TypeSortedCollection) = sum(length, x.data)
indexfun(x::TypeSortedCollection) = x.indexfun

@generated function Base.map!(f, dest::AbstractVector, tv::TypeSortedCollection{I, D}, As...) where {I, D}
    expr = Expr(:block)
    push!(expr.args, :(Base.@_inline_meta))
    for i = 1 : nfields(D)
        push!(expr.args, quote
            vec = tv.data[$i]
            for element in vec
                index = tv.indexfun(element)
                dest[index] = f(element, getindex.(As, index)...)
            end
        end)
    end
    push!(expr.args, :(return nothing))
    expr
end
