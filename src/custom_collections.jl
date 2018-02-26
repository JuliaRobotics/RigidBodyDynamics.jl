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
    segments

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

struct IndexDict{K, V} <: AbstractIndexDict{K, V}
    values::Vector{V}
    IndexDict{K, V}(values::Vector{V}) where {K, V} = new{K, V}(values)
end

mutable struct CacheIndexDict{K, V} <: AbstractIndexDict{K, V}
    values::Vector{V}
    dirty::Bool
    CacheIndexDict{K, V}(values::Vector{V}) where {K, V} = new{K, V}(values, true)
end

setdirty!(d::CacheIndexDict) = (d.dirty = true)
isdirty(d::CacheIndexDict) = d.dirty

# Constructors
for IDict in (:IndexDict, :CacheIndexDict)
    @eval begin
        (::Type{$IDict{K, V}})(n::Integer) where {K, V} = $IDict{K, V}(Vector{V}(uninitialized, n))
        (::Type{$IDict{K}})(values::Vector{V}) where {K, V} = $IDict{K, V}(values)

        function (::Type{$IDict{K, V}})(itr) where {K, V}
            ret = $IDict{K, V}(length(itr))
            for (k, v) in itr
                ret[k] = v
            end
            ret
        end

        (::Type{$IDict{K}})(dict::Associative{<:Any, V}) where {K, V} = $IDict{K, V}(dict)
        (::Type{$IDict})(dict::Associative{K, V}) where {K, V} = $IDict{K, V}(dict)
        (::Type{$IDict{K}})(itr) where {K} = $IDict(Dict(itr))
        (::Type{$IDict})(itr) = $IDict(Dict(itr))
    end
end

@inline Base.isempty(d::AbstractIndexDict) = isempty(d.values)
@inline Base.length(d::AbstractIndexDict) = length(d.values)
@inline Base.start(d::AbstractIndexDict) = 1
@inline Base.next(d::AbstractIndexDict{K}, i) where {K} = (K(i) => d.values[i], i + 1)
@inline Base.done(d::AbstractIndexDict, i) = i == length(d) + 1
@inline Base.keys(d::AbstractIndexDict{K}) where {K} = (K(i) for i in eachindex(d.values))
@inline Base.values(d::AbstractIndexDict) = d.values
@inline Base.haskey(d::AbstractIndexDict, key) = Int(key) ∈ 1 : length(d)
Base.@propagate_inbounds Base.getindex(d::AbstractIndexDict{K}, key::K) where {K} = d.values[Int(key)]
Base.@propagate_inbounds Base.setindex!(d::AbstractIndexDict{K}, value, key::K) where {K} = d.values[Int(key)] = value


## SegmentedVector
const VectorSegment{T} = SubArray{T,1,Array{T, 1},Tuple{UnitRange{Int64}},true} # type of a n:m view into a Vector

struct SegmentedVector{I, T, P<:AbstractVector{T}} <: AbstractVector{T}
    parent::P
    segments::IndexDict{I, VectorSegment{T}}

    function SegmentedVector(p::P, segments::IndexDict{I, VectorSegment{T}}) where {I, T, P}
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

function (::Type{SegmentedVector{I}})(parent::AbstractVector{T}, viewlengths) where {T, I}
    start = Ref(1)
    makeview = function (parent, viewlength)
        stop = start[] + viewlength - 1
        ret = view(parent, start[] : stop)
        start[] = stop + 1
        ret
    end
    segments = IndexDict{I, VectorSegment{T}}(i => makeview(parent, viewlength) for (i, viewlength) in viewlengths)
    SegmentedVector(parent, segments)
end

Base.size(v::SegmentedVector) = size(v.parent)
Base.@propagate_inbounds Base.getindex(v::SegmentedVector, i::Int) = v.parent[i]
Base.@propagate_inbounds Base.setindex!(v::SegmentedVector, value, i::Int) = v.parent[i] = value

Base.parent(v::SegmentedVector) = v.parent
segments(v::SegmentedVector) = v.segments

struct DiscardVector <: AbstractVector{Any}
    length::Int
end
Base.setindex!(v::DiscardVector, value, i::Int) = nothing
Base.size(v::DiscardVector) = (v.length,)

end # module
