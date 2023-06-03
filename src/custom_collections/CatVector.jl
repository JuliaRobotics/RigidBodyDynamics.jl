"""
$(TYPEDEF)

An `AbstractVector` subtype that acts as a lazy concatenation of a number
of subvectors.
"""
struct CatVector{T, N, V<:AbstractVector{T}} <: AbstractVector{T}
    vecs::NTuple{N, V}
end

@inline Base.size(vec::CatVector) = (mapreduce(length, +, vec.vecs; init=0),)

# Note: getindex and setindex are pretty naive. Consider precomputing map from
# index to vector upon CatVector construction.
Base.@propagate_inbounds function Base.getindex(vec::CatVector, index::Int)
    @boundscheck index >= 1 || throw(BoundsError(vec, index))
    i = 1
    j = index
    @inbounds while true
        subvec = vec.vecs[i]
        l = length(subvec)
        if j <= l
            return subvec[eachindex(subvec)[j]]
        else
            j -= l
            i += 1
        end
    end
    throw(BoundsError(vec, index))
end

Base.@propagate_inbounds function Base.setindex!(vec::CatVector, val, index::Int)
    @boundscheck index >= 1 || throw(BoundsError(vec, index))
    i = 1
    j = index
    while true
        subvec = vec.vecs[i]
        l = length(subvec)
        if j <= l
            subvec[eachindex(subvec)[j]] = val
            return val
        else
            j -= l
            i += 1
        end
    end
    throw(BoundsError(vec, index))
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

@noinline cat_vectors_line_up_error() = throw(ArgumentError("Subvectors must line up"))

@inline function check_cat_vectors_line_up(x::CatVector, y::CatVector)
    length(x.vecs) == length(y.vecs) || cat_vectors_line_up_error()
    for i in eachindex(x.vecs)
        length(x.vecs[i]) == length(y.vecs[i]) || cat_vectors_line_up_error()
    end
    nothing
end

@inline check_cat_vectors_line_up(x::CatVector, y) = nothing
@inline function check_cat_vectors_line_up(x::CatVector, y, tail...)
    check_cat_vectors_line_up(x, y)
    check_cat_vectors_line_up(x, tail...)
end

@propagate_inbounds function Base.copyto!(dest::CatVector, src::CatVector)
    for i in eachindex(dest.vecs)
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

Base.@propagate_inbounds catvec_broadcast_vec(arg::CatVector, range::UnitRange, k::Int) = arg.vecs[k]
Base.@propagate_inbounds catvec_broadcast_vec(arg::AbstractVector, range::UnitRange, k::Int) = view(arg, range)
Base.@propagate_inbounds catvec_broadcast_vec(arg::Number, range::UnitRange, k::Int) = arg

@inline function Base.copyto!(dest::CatVector, bc::Broadcast.Broadcasted{Nothing})
    flat = Broadcast.flatten(bc)
    @boundscheck check_cat_vectors_line_up(dest, flat.args...)
    offset = 1
    @inbounds for i in eachindex(dest.vecs)
        let i = i, f = flat.f, args = flat.args
            dest′ = dest.vecs[i]
            range = offset : offset + length(dest′) - 1
            args′ = map(arg -> catvec_broadcast_vec(arg, range, i), args)
            axes′ = (eachindex(dest′),)
            copyto!(dest′, Broadcast.Broadcasted{Nothing}(f, args′, axes′))
            offset = last(range) + 1
        end
    end
    return dest
end
