function sort_by_type(A)
    ret = Dict()
    for x in A
        T = typeof(x)
        vector = get!(Vector{T}, ret, T)
        push!(vector, x)
    end
    ret
end

mutable struct TypeSortedCollection{Indexfun, D<:Tuple{Vararg{Vector{T} where T}}}
    data::D
end

function TypeSortedCollection(indexfun, x)
    dict = sort_by_type(x)
    data = tuple(values(dict)...)
    TypeSortedCollection{indexfun, typeof(data)}(data)
end

Base.length(x::TypeSortedCollection) = sum(length, x.data)
indexfun(x::TypeSortedCollection{I}) where {I} = I

@generated function Base.map!(f, dest::AbstractVector, tv::TypeSortedCollection{Indexfun, D}) where {Indexfun, D}
    n = nfields(D)
    expr = :()
    for i = 1 : n
        expr = quote
            $expr
            vec = tv.data[$i]
            for element in vec
                dest[Indexfun(element)] = f(element)
            end
        end
    end
    quote
        $expr
        return nothing
    end
end
