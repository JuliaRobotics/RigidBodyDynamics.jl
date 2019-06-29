struct CatVector{T, N, I<:Tuple{Vararg{AbstractVector{T}, N}}} <: AbstractVector{T}
    vecs::I
end

@inline Base.size(vec::CatVector) = (mapreduce(length, +, vec.vecs; init=0),)
Base.eltype(vec::CatVector) = eltype(eltype(vec.vecs))

# Note: getindex and setindex are pretty naive.
Base.@propagate_inbounds function Base.getindex(vec::CatVector, i::Int)
    @boundscheck checkbounds(vec, i)
    I = 1
    @inbounds while true
        subvec = vec.vecs[I]
        l = length(subvec)
        if i <= l
            return subvec[eachindex(subvec)[i]]
        else
            i -= l
            I += 1
        end
    end
    error()
end

Base.@propagate_inbounds function Base.setindex!(vec::CatVector, val, i::Int)
    @boundscheck checkbounds(vec, i)
    I = 1
    while true
        subvec = vec.vecs[I]
        l = length(subvec)
        if i <= l
            subvec[eachindex(subvec)[i]] = val
            return val
        else
            i -= l
            I += 1
        end
    end
    error()
end

Base.@propagate_inbounds function Base.copyto!(dest::AbstractVector, src::CatVector)
    @boundscheck length(dest) == length(src) || throw(DimensionMismatch())
    dest_indices = eachindex(dest)
    k = 1
    @inbounds for i in eachindex(src.vecs)
        vec = src.vecs[i]
        for j in eachindex(vec)
            dest[dest_indices[k]] = vec[j]
            k += 1
        end
    end
    return dest
end

Base.similar(vec::CatVector) = CatVector(map(similar, vec.vecs))
Base.similar(vec::CatVector, ::Type{T}) where {T} = CatVector(map(x -> similar(x, T), vec.vecs))
