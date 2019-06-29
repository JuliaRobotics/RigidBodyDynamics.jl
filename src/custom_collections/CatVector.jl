struct CatVector{T, N, V<:AbstractVector{T}} <: AbstractVector{T}
    vecs::NTuple{N, V}
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

Base.@propagate_inbounds function Base.copyto!(dest::AbstractVector{T}, src::CatVector{T}) where {T}
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

@inline function check_cat_vectors_line_up(x::CatVector, y::CatVector)
    length(x.vecs) == length(y.vecs) || throw(ArgumentError("Subvectors must line up"))
    for i in eachindex(x.vecs)
        length(x.vecs[i]) == length(y.vecs[i]) || throw(ArgumentError("Subvectors must line up"))
    end
    nothing
end

@inline check_cat_vectors_line_up(x::CatVector, y) = nothing
@inline check_cat_vectors_line_up(x::CatVector, y, tail...) = (check_cat_vectors_line_up(x, y); check_cat_vectors_line_up(x, tail...))

@inline function Base.copyto!(dest::CatVector, src::CatVector)
    @boundscheck check_cat_vectors_line_up(dest, src)
    @inbounds for i in eachindex(dest.vecs)
        copyto!(dest.vecs[i], src.vecs[i])
    end
    return dest
end

@inline function Base.map!(f::F, dest::CatVector, args::CatVector...) where F
    @boundscheck check_cat_vectors_line_up(dest, args...)
    @inbounds for i in eachindex(dest.vecs)
        map!(f, dest.vecs[i], map(arg -> arg.vecs[i], args)...)
    end
    return dest
end

Base.@propagate_inbounds catvec_broadcast_getindex(vec::CatVector, i::Int, j::Int, k::Int) = vec.vecs[i][j]
Base.@propagate_inbounds catvec_broadcast_getindex(x, i::Int, j::Int, k::Int) = Broadcast._broadcast_getindex(x, i)

@inline function Base.copyto!(dest::CatVector, bc::Broadcast.Broadcasted{Nothing})
    flat = Broadcast.flatten(bc)
    index = 1
    dest_vecs = dest.vecs
    @boundscheck check_cat_vectors_line_up(dest, bc.args...)
    @inbounds for i in eachindex(dest_vecs)
        vec = dest_vecs[i]
        for j in eachindex(vec)
            k = axes(flat)[1][index]
            let f = flat.f, args = flat.args, i = i, j = j, k = k
                vec[j] = Broadcast._broadcast_getindex_evalf(f, map(arg -> catvec_broadcast_getindex(arg, i, j, k), args)...)
            end
            index += 1
        end
    end
    return dest
end
