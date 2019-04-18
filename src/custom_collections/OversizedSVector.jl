struct OversizedSVector{L, N, T} <: StaticVector{N, T}
    data::NTuple{L, T}

    @inline function OversizedSVector{L, N, T}(x::Tuple{Vararg{Any, L}}) where {L, N, T}
        new{L, N, T}(x)
    end

    @inline function OversizedSVector{L, N, T}(x::Tuple{Vararg{Any, M}}, padfun=zero) where {L, N, T, M}
        padding = ntuple(i -> padfun(T), Val(L - N))
        new{L, N, T}(tuple(x..., padding...))
    end
end

@inline function OversizedSVector{L, N}(x::Tuple{Vararg{Any, L}}) where {L, N}
    T = StaticArrays.promote_tuple_eltype(x)
    OversizedSVector{L, N, T}(x)
end

@inline function OversizedSVector{L}(x::Tuple{Vararg{Any, N}}) where {L, N}
    T = StaticArrays.promote_tuple_eltype(x)
    OversizedSVector{L, N, T}(x)
end

@inline (::Type{SV})(x...) where {SV<:OversizedSVector} = SV(x)

Base.@propagate_inbounds function Base.getindex(v::OversizedSVector{L, N}, i::Int) where {L, N}
    @boundscheck 1 <= i <= N || throw(BoundsError(v, i))
    return v.data[i]
end

function StaticArrays.similar_type(::Type{SV}, ::Type{T}, ::Size{S}) where {L, N, T, S, SV<:OversizedSVector{L, N}}
    if S === (N,)
        return OversizedSVector{L, N, T}
    else
        return StaticArrays.default_similar_type(SV, T, S)
    end
end

# Optimizations
for op in (:+, :-)
    @eval function Base.$op(x::OversizedSVector{L, N}, y::OversizedSVector{L, N}) where {L, N}
        OversizedSVector{L, N}(Tuple($op(SVector(x.data), SVector(y.data))))
    end
end

for op in (:*, :/)
    @eval function Base.$op(x::OversizedSVector{L, N}, y::Number) where {L, N}
        OversizedSVector{L, N}(Tuple($op(SVector(x.data), y)))
    end
end

for op in (:*, :\)
    @eval function Base.$op(x::Number, y::OversizedSVector{L, N}) where {L, N}
        OversizedSVector{L, N}(Tuple($op(x, SVector(y.data))))
    end
end
