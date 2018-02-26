module CustomCollections

using Compat
using TypeSortedCollections

export
    ConstVector,
    NullDict,
    UnsafeVectorView,
    CacheElement,
    AbstractIndexDict,
    IndexDict,
    CacheIndexDict,
    SegmentedVector,
    DiscardVector

export
    fastview,
    foreach_with_extra_args,
    isdirty,
    segments,
    ranges

## TypeSortedCollections addendum
# `foreach_with_extra_args` below is a hack to avoid allocations associated with creating closures over
# heap-allocated variables. Hopefully this will not be necessary in a future version of Julia.
for num_extra_args = 1 : 5
    extra_arg_syms = [Symbol("arg", i) for i = 1 : num_extra_args]
    @eval begin
        @generated function foreach_with_extra_args(f, $(extra_arg_syms...), A1::TypeSortedCollection{<:Any, N}, As::Union{<:TypeSortedCollection{<:Any, N}, AbstractVector}...) where {N}
            extra_args = $extra_arg_syms
            expr = Expr(:block)
            push!(expr.args, :(Base.@_inline_meta)) # required to achieve zero allocation
            push!(expr.args, :(leading_tsc = A1))
            push!(expr.args, :(@boundscheck TypeSortedCollections.lengths_match(A1, As...) || TypeSortedCollections.lengths_match_fail()))
            for i = 1 : N
                vali = Val(i)
                push!(expr.args, quote
                    let inds = leading_tsc.indices[$i]
                        @boundscheck TypeSortedCollections.indices_match($vali, inds, A1, As...) || TypeSortedCollections.indices_match_fail()
                        @inbounds for j in linearindices(inds)
                            vecindex = inds[j]
                            f($(extra_args...), TypeSortedCollections._getindex_all($vali, j, vecindex, A1, As...)...)
                        end
                    end
                end)
            end
            quote
                $expr
                nothing
            end
        end
    end
end


## ConstVector
"""
An immutable `AbstractVector` for which all elements are the same, represented
compactly and as an isbits type if the element type is `isbits`.
"""
struct ConstVector{T} <: AbstractVector{T}
    val::T
    length::Int64
end
Base.size(A::ConstVector) = (A.length, )
@inline Base.getindex(A::ConstVector, i::Int) = (@boundscheck checkbounds(A, i); A.val)
Base.IndexStyle(::Type{<:ConstVector}) = IndexLinear()


## NullDict
"""
An immutable associative type that signifies an empty dictionary and does not
allocate any memory.
"""
struct NullDict{K, V} <: Associative{K, V}
end
Base.haskey(::NullDict, k) = false
Base.length(::NullDict) = 0
Base.start(::NullDict) = nothing
Base.done(::NullDict, state) = true


## UnsafeVectorView
# TODO: remove
"""
Views in Julia still allocate some memory (since they need to keep
a reference to the original array). This type allocates no memory
and does no bounds checking. Use it with caution.

Originally from https://github.com/mlubin/ReverseDiffSparse.jl/commit/8e3ade867581aad6ade7c898ada2ed58e0ad42bb.
"""
struct UnsafeVectorView{T} <: AbstractVector{T}
    offset::Int
    len::Int
    ptr::Ptr{T}
end

@inline UnsafeVectorView(parent::Union{Vector, Base.FastContiguousSubArray}, range::UnitRange) = UnsafeVectorView(start(range) - 1, length(range), pointer(parent))
@inline Base.size(v::UnsafeVectorView) = (v.len,)
@inline Base.getindex(v::UnsafeVectorView, idx::Int) = unsafe_load(v.ptr, idx + v.offset)
@inline Base.setindex!(v::UnsafeVectorView, value, idx::Int) = unsafe_store!(v.ptr, value, idx + v.offset)
@inline Base.length(v::UnsafeVectorView) = v.len
Base.IndexStyle(::Type{<:UnsafeVectorView}) = IndexLinear()

"""
UnsafeVectorView only works for isbits types. For other types, we're already
allocating lots of memory elsewhere, so creating a new SubArray is fine.
This function looks type-unstable, but the isbits(T) test can be evaluated
by the compiler, so the result is actually type-stable.

From https://github.com/rdeits/NNLS.jl/blob/0a9bf56774595b5735bc738723bd3cb94138c5bd/src/NNLS.jl#L218.
"""
@inline function fastview(parent::Union{Vector{T}, Base.FastContiguousSubArray{T}}, range::UnitRange) where {T}
    if isbits(T)
        UnsafeVectorView(parent, range)
    else
        view(parent, range)
    end
end


## CacheElement
mutable struct CacheElement{T}
    data::T
    dirty::Bool
    CacheElement(data::T) where {T} = new{T}(data, true)
end

@inline setdirty!(element::CacheElement) = (element.dirty = true; nothing)
@inline isdirty(element::CacheElement) = element.dirty


## IndexDicts
abstract type AbstractIndexDict{K, V} <: Associative{K, V} end

makekeys(::Type{UnitRange{K}}, start::K, stop::K) where {K} = start : stop
function makekeys(::Type{Base.OneTo{K}}, start::K, stop::K) where {K}
    @boundscheck start === K(1) || error()
    Base.OneTo(stop)
end

struct IndexDict{K, KeyRange<:AbstractUnitRange{K}, V} <: AbstractIndexDict{K, V}
    keys::KeyRange
    values::Vector{V}
    IndexDict(keys::KeyRange, values::Vector{V}) where {K, V, KeyRange<:AbstractUnitRange{K}} = new{K, KeyRange, V}(keys, values)
end

mutable struct CacheIndexDict{K, KeyRange<:AbstractUnitRange{K}, V} <: AbstractIndexDict{K, V}
    keys::KeyRange
    values::Vector{V}
    dirty::Bool
    function CacheIndexDict(keys::KeyRange, values::Vector{V}) where {K, V, KeyRange<:AbstractUnitRange{K}}
        @boundscheck length(keys) == length(values) || error("Mismatch between keys and values.")
        new{K, KeyRange, V}(keys, values, true)
    end
end

setdirty!(d::CacheIndexDict) = (d.dirty = true)
isdirty(d::CacheIndexDict) = d.dirty

# Constructors
for IDict in (:IndexDict, :CacheIndexDict)
    @eval begin
        function (::Type{$IDict{K, KeyRange, V}})(keys::KeyRange) where {K, KeyRange<:AbstractUnitRange{K}, V}
            $IDict(keys, Vector{V}(uninitialized, length(keys)))
        end

        function (::Type{$IDict{K, KeyRange, V}})(keys::KeyRange, values::Vector{V}) where {K, KeyRange<:AbstractUnitRange{K}, V}
            $IDict(keys, values)
        end

        function (::Type{$IDict{K, KeyRange, V}})(kv::Vector{Pair{K, V}}) where {K, KeyRange<:AbstractUnitRange{K}, V}
            if !issorted(kv, by = first)
                sort!(kv; by = first)
            end
            start = first(first(kv))
            stop = first(last(kv))
            keys = makekeys(KeyRange, start, stop)
            for i in eachindex(kv)
                keys[i] === first(kv[i]) || error()
            end
            values = map(last, kv)
            $IDict(keys, values)
        end

        function (::Type{$IDict{K, KeyRange}})(kv::Vector{Pair{K, V}}) where {K, KeyRange<:AbstractUnitRange{K}, V}
            $IDict{K, KeyRange, V}(kv)
        end

        function (::Type{$IDict{K, KeyRange, V}})(itr) where {K, KeyRange<:AbstractUnitRange{K}, V}
            kv = map(x -> K(first(x)) => last(x)::V, itr)
            $IDict{K, KeyRange, V}(kv)
        end

        function (::Type{$IDict{K, KeyRange}})(itr) where {K, KeyRange<:AbstractUnitRange{K}}
            kv = map(x -> K(first(x)) => last(x), itr)
            $IDict{K, KeyRange}(kv)
        end
    end
end

@inline Base.isempty(d::AbstractIndexDict) = isempty(d.values)
@inline Base.length(d::AbstractIndexDict) = length(d.values)
@inline Base.start(d::AbstractIndexDict) = 1
@inline Base.next(d::AbstractIndexDict{K}, i) where {K} = (K(i) => d.values[i], i + 1)
@inline Base.done(d::AbstractIndexDict, i) = i == length(d) + 1
@inline Base.keys(d::AbstractIndexDict{K}) where {K} = d.keys
@inline Base.values(d::AbstractIndexDict) = d.values
@inline Base.haskey(d::AbstractIndexDict, key) = key ∈ d.keys
@inline keyindex(key::K, keyrange::Base.OneTo{K}) where {K} = Int(key)
@inline keyindex(key::K, keyrange::UnitRange{K}) where {K} = Int(key - first(keyrange) + 1)
Base.@propagate_inbounds Base.getindex(d::AbstractIndexDict{K}, key::K) where {K} = d.values[keyindex(key, d.keys)]
Base.@propagate_inbounds Base.setindex!(d::AbstractIndexDict{K}, value, key::K) where {K} = d.values[keyindex(key, d.keys)] = value


## SegmentedVector
const VectorSegment{T} = SubArray{T,1,Array{T, 1},Tuple{UnitRange{Int64}},true} # type of a n:m view into a Vector

struct SegmentedVector{I, T, P<:AbstractVector{T}} <: AbstractVector{T}
    parent::P
    segments::IndexDict{I, Base.OneTo{I}, VectorSegment{T}}

    function SegmentedVector(p::P, segments::IndexDict{I, Base.OneTo{I}, VectorSegment{T}}) where {I, T, P}
        @boundscheck begin
            start = 1
            for segment in values(segments)
                parent(segment) === p || error()
                indices = first(parentindexes(segment))
                first(indices) === start || error()
                start = last(indices) + 1
            end
            start === endof(p) + 1 || error("Segments do not cover input data.")
        end
        new{I, T, P}(p, segments)
    end
end

Base.size(v::SegmentedVector) = size(v.parent)
Base.@propagate_inbounds Base.getindex(v::SegmentedVector, i::Int) = v.parent[i]
Base.@propagate_inbounds Base.setindex!(v::SegmentedVector, value, i::Int) = v.parent[i] = value

Base.parent(v::SegmentedVector) = v.parent
segments(v::SegmentedVector) = v.segments
ranges(v::SegmentedVector{I}) where {I} = IndexDict(v.segments.keys, [first(parentindexes(view)) for view in v.segments.values])

struct DiscardVector <: AbstractVector{Any}
    length::Int
end
@inline Base.setindex!(v::DiscardVector, value, i::Int) = nothing
@inline Base.size(v::DiscardVector) = (v.length,)

end # module
