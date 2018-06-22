module CustomCollections

using Compat
using TypeSortedCollections
using DocStringExtensions

export
    ConstVector,
    NullDict,
    CacheElement,
    AbstractIndexDict,
    IndexDict,
    CacheIndexDict,
    SegmentedVector,
    DiscardVector,
    SegmentedBlockDiagonalMatrix

export
    foreach_with_extra_args,
    isdirty,
    segments,
    ranges

@static if !isdefined(Base, :parentindices)
    parentindices(x) = Base.parentindexes(x)
end

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
$(TYPEDEF)

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
$(TYPEDEF)

An immutable associative type that signifies an empty dictionary and does not
allocate any memory.
"""
struct NullDict{K, V} <: AbstractDict{K, V}
end
Base.haskey(::NullDict, k) = false
Base.length(::NullDict) = 0
Base.start(::NullDict) = nothing
Base.done(::NullDict, state) = true


## CacheElement
mutable struct CacheElement{T}
    data::T
    dirty::Bool
    CacheElement(data::T) where {T} = new{T}(data, true)
end

@inline setdirty!(element::CacheElement) = (element.dirty = true; nothing)
@inline isdirty(element::CacheElement) = element.dirty


## IndexDicts
abstract type AbstractIndexDict{K, V} <: AbstractDict{K, V} end

makekeys(::Type{UnitRange{K}}, start::K, stop::K) where {K} = start : stop
function makekeys(::Type{Base.OneTo{K}}, start::K, stop::K) where {K}
    @boundscheck start === K(1) || error()
    Base.OneTo(stop)
end

"""
$(TYPEDEF)

An associative type whose keys are an `AbstractUnitRange`, and whose values are stored in
a `Vector`. `IndexDict` is an ordered associative collection, with the order determined by key range.
The nature of the keys enables very fast lookups and stores.

# Examples
```julia-repl
julia> IndexDict(2 : 4, [4, 5, 6])
RigidBodyDynamics.CustomCollections.IndexDict{Int64,UnitRange{Int64},Int64} with 3 entries:
  2 => 4
  3 => 5
  4 => 6

julia> IndexDict{Int32, UnitRange{Int32}}(i => 3 * i for i in Int32[4, 2, 3])
RigidBodyDynamics.CustomCollections.IndexDict{Int32,UnitRange{Int32},Int64} with 3 entries:
  2 => 6
  3 => 9
  4 => 12
```
"""
struct IndexDict{K, KeyRange<:AbstractUnitRange{K}, V} <: AbstractIndexDict{K, V}
    keys::KeyRange
    values::Vector{V}
end


"""
$(TYPEDEF)

Like [`IndexDict`](@ref), but contains an additional `Bool` dirty bit to be used in algorithms involving cached data.
"""
mutable struct CacheIndexDict{K, KeyRange<:AbstractUnitRange{K}, V} <: AbstractIndexDict{K, V}
    keys::KeyRange
    values::Vector{V}
    dirty::Bool
    function CacheIndexDict{K, KeyRange, V}(keys::KeyRange, values::Vector{V}) where {K, V, KeyRange<:AbstractUnitRange{K}}
        @boundscheck length(keys) == length(values) || error("Mismatch between keys and values.")
        new{K, KeyRange, V}(keys, values, true)
    end
end

setdirty!(d::CacheIndexDict) = (d.dirty = true)
isdirty(d::CacheIndexDict) = d.dirty

# Constructors
for IDict in (:IndexDict, :CacheIndexDict)
    @eval begin
        function $IDict(keys::KeyRange, values::Vector{V}) where {K, V, KeyRange<:AbstractUnitRange{K}}
            $IDict{K, KeyRange, V}(keys, values)
        end

        function $IDict{K, KeyRange, V}(keys::KeyRange) where {K, KeyRange<:AbstractUnitRange{K}, V}
            $IDict{K, KeyRange, V}(keys, Vector{V}(undef, length(keys)))
        end

        function $IDict{K, KeyRange, V}(kv::Vector{Pair{K, V}}) where {K, KeyRange<:AbstractUnitRange{K}, V}
            if !issorted(kv, by = first)
                sort!(kv; by = first)
            end
            start, stop = if isempty(kv)
                K(1), K(0)
            else
                first(first(kv)), first(last(kv))
            end
            keys = makekeys(KeyRange, start, stop)
            for i in eachindex(kv)
                keys[i] === first(kv[i]) || error()
            end
            values = map(last, kv)
            $IDict{K, KeyRange, V}(keys, values)
        end

        function $IDict{K, KeyRange}(kv::Vector{Pair{K, V}}) where {K, KeyRange<:AbstractUnitRange{K}, V}
            $IDict{K, KeyRange, V}(kv)
        end

        function $IDict{K, KeyRange, V}(itr) where {K, KeyRange<:AbstractUnitRange{K}, V}
            kv = Pair{K, V}[]
            for x in itr
                push!(kv, convert(K, first(x)) => convert(V, last(x)))
            end
            $IDict{K, KeyRange, V}(kv)
        end

        function $IDict{K, KeyRange}(itr) where {K, KeyRange<:AbstractUnitRange{K}}
            kv = [convert(K, first(x)) => last(x) for x in itr]
            $IDict{K, KeyRange}(kv)
        end
    end
end

@inline Base.isempty(d::AbstractIndexDict) = isempty(d.values)
@inline Base.length(d::AbstractIndexDict) = length(d.values)
@inline Base.start(d::AbstractIndexDict) = 1
@inline Base.next(d::AbstractIndexDict{K}, i) where {K} = (d.keys[i] => d.values[i], i + 1)
@inline Base.done(d::AbstractIndexDict, i) = i == length(d) + 1
@inline Base.keys(d::AbstractIndexDict{K}) where {K} = d.keys
@inline Base.values(d::AbstractIndexDict) = d.values
@inline Base.haskey(d::AbstractIndexDict, key) = key ∈ d.keys
@inline keyindex(key::K, keyrange::Base.OneTo{K}) where {K} = Int(key)
@inline keyindex(key::K, keyrange::UnitRange{K}) where {K} = Int(key - first(keyrange) + 1)
Base.@propagate_inbounds Base.getindex(d::AbstractIndexDict{K}, key::K) where {K} = d.values[keyindex(key, d.keys)]
Base.@propagate_inbounds Base.setindex!(d::AbstractIndexDict{K}, value, key::K) where {K} = d.values[keyindex(key, d.keys)] = value
Base.@propagate_inbounds Base.get(d::AbstractIndexDict{K}, key::K, default) where {K} = d[key]


## SegmentedVector
const VectorSegment{T} = SubArray{T,1,Array{T, 1},Tuple{UnitRange{Int}},true} # type of a n:m view into a Vector

"""
$(TYPEDEF)

`SegmentedVector` is an `AbstractVector` backed by another `AbstractVector` (its parent), which additionally stores an [`IndexDict`](@ref)
containing views into the parent. Together, these views cover the parent.

# Examples

```julia-repl
julia> x = [1., 2., 3., 4.]
4-element Array{Float64,1}:
 1.0
 2.0
 3.0
 4.0

julia> viewlength(i) = 2
viewlength (generic function with 1 method)

julia> xseg = SegmentedVector{Int}(x, 1 : 2, viewlength)
4-element RigidBodyDynamics.CustomCollections.SegmentedVector{Int64,Float64,Base.OneTo{Int64},Array{Float64,1}}:
 1.0
 2.0
 3.0
 4.0

julia> segments(xseg)[1]
2-element SubArray{Float64,1,Array{Float64,1},Tuple{UnitRange{Int64}},true}:
 1.0
 2.0

julia> yseg = similar(xseg, Int32); yseg .= 1 : 4 # same view ranges, different element type
4-element RigidBodyDynamics.CustomCollections.SegmentedVector{Int64,Int32,Base.OneTo{Int64},Array{Int32,1}}:
 1
 2
 3
 4

julia> segments(yseg)[2]
2-element SubArray{Int32,1,Array{Int32,1},Tuple{UnitRange{Int64}},true}:
 3
 4
```
"""
struct SegmentedVector{K, T, KeyRange<:AbstractRange{K}, P<:AbstractVector{T}} <: AbstractVector{T}
    parent::P
    segments::IndexDict{K, KeyRange, VectorSegment{T}}

    function SegmentedVector{K, T, KeyRange, P}(p::P, segments::IndexDict{K, KeyRange, VectorSegment{T}}) where {K, T, KeyRange<:AbstractRange{K}, P<:AbstractVector{T}}
        @boundscheck begin
            firstsegment = true
            start = 0
            l = 0
            for segment in values(segments)
                parent(segment) === parent(p) || error()
                indices = first(parentindices(segment))
                if firstsegment
                    start = first(indices)
                    firstsegment = false
                else
                    first(indices) === start || error()
                end
                start = last(indices) + 1
                l += length(indices)
            end
            l == length(p) || error("Segments do not cover input data.")
        end
        new{K, T, KeyRange, P}(p, segments)
    end
end

function SegmentedVector(p::P, segments::IndexDict{K, KeyRange, VectorSegment{T}}) where {K, T, KeyRange<:AbstractRange{K}, P<:AbstractVector{T}}
    SegmentedVector{K, T, KeyRange, P}(p, segments)
end

function SegmentedVector{K, T, KeyRange}(parent::P, keys, viewlengthfun) where {K, T, KeyRange<:AbstractRange{K}, P<:AbstractVector{T}}
    views = Vector{Pair{K, VectorSegment{T}}}()
    start = 1
    for key in keys
        stop = start[] + viewlengthfun(key) - 1
        push!(views, convert(K, key) => view(parent, start : stop))
        start = stop + 1
    end
    SegmentedVector{K, T, KeyRange, P}(parent, IndexDict{K, KeyRange, VectorSegment{T}}(views))
end

function SegmentedVector{K}(parent::P, keys, viewlengthfun) where {K, T, P<:AbstractVector{T}}
    SegmentedVector{K, T, Base.OneTo{K}}(parent, keys, viewlengthfun)
end

function SegmentedVector{K, T, KeyRange}(parent::P, ranges::AbstractDict{K, UnitRange{Int}}) where {K, T, KeyRange, P<:AbstractVector{T}}
    segs = IndexDict{K, KeyRange, VectorSegment{T}}(keys(ranges), [view(parent, range) for range in values(ranges)])
    SegmentedVector{K, T, KeyRange, P}(parent, IndexDict{K, KeyRange, VectorSegment{T}}(segs))
end

Base.size(v::SegmentedVector) = size(v.parent)
Base.@propagate_inbounds Base.getindex(v::SegmentedVector, i::Int) = v.parent[i]
Base.@propagate_inbounds Base.setindex!(v::SegmentedVector, value, i::Int) = v.parent[i] = value

Base.parent(v::SegmentedVector) = v.parent
segments(v::SegmentedVector) = v.segments
function ranges(v::SegmentedVector{K, <:Any, KeyRange}) where {K, KeyRange}
    segments = v.segments
    IndexDict{K, KeyRange, UnitRange{Int}}(segments.keys, map(segment -> first(parentindices(segment))::UnitRange{Int}, segments.values))
end

function Base.similar(v::SegmentedVector{K, T, KeyRange}, ::Type{S} = T) where {K, T, KeyRange, S}
    SegmentedVector{K, S, KeyRange}(similar(parent(v), S), ranges(v))
end

"""
$(TYPEDEF)

`DiscardVector` is an `AbstractVector` whose `setindex!` simply discards the value.
This is useful for `broadcast!` calls where the output of the broadcasted function is not interesting,
specifically when the broadcasted function is in-place and there are arguments that need to be treated
as scalars, so that a simple foreach doesn't do the job.
"""
struct DiscardVector <: AbstractVector{Any}
    length::Int
end
@inline Base.setindex!(v::DiscardVector, value, i::Int) = nothing
@inline Base.size(v::DiscardVector) = (v.length,)


const AbstractMatrixBlock{T, M} = SubArray{T,2,M,Tuple{UnitRange{Int},UnitRange{Int}},false}

function _is_contiguous_and_diagonal(parent::AbstractMatrix, block_indices)
    expected_starts = first.(indices(parent))
    for inds in block_indices
        if first.(inds) !== expected_starts
            return false
        end
        expected_starts = last.(inds) .+ 1
    end
    if expected_starts !== last.(indices(parent)) .+ 1
        return false
    end
    return true
end


function check_contiguous_block_ranges(parent::AbstractMatrix, block_indices)
    if !_is_contiguous_and_diagonal(parent, block_indices)
        throw(ArgumentError("The `block_indices` should be a vector of index ranges corresponding to non-overlapping contiguous diagonal blocks"))
    end
end

"""
$(TYPEDEF)

`SegmentedBlockDiagonalMatrix` is an `AbstractMatrix` backed by a parent `AbstractMatrix`, which
additionally stores a sequence of views into the diagonal blocks of the parent matrix. This
type is useful for storing and updating block-diagonal matrices whose block contents
may change but whose overall structure is fixed, such as configuration derivative <-> velocity
jacobians.
"""
struct SegmentedBlockDiagonalMatrix{T, M <: AbstractMatrix{T}} <: AbstractMatrix{T}
    parent::M
    blocks::Vector{AbstractMatrixBlock{T, M}}

    function SegmentedBlockDiagonalMatrix{T}(parent::AbstractMatrix{T}, block_indices) where T
        check_contiguous_block_ranges(parent, block_indices)
        blocks = map(block_indices) do indices
            view(parent, indices...)
        end
        new{T, typeof(parent)}(parent, blocks)
    end
end

SegmentedBlockDiagonalMatrix(parent::AbstractMatrix{T}, block_indices) where {T} = SegmentedBlockDiagonalMatrix{T}(parent, block_indices)

function SegmentedBlockDiagonalMatrix{T}(initializer, rows::Integer, cols::Integer, block_indices) where T
    parent = Matrix{T}(initializer, rows, cols)
    SegmentedBlockDiagonalMatrix{T}(parent, block_indices)
end

Base.parent(m::SegmentedBlockDiagonalMatrix) = m.parent
Base.size(m::SegmentedBlockDiagonalMatrix) = size(m.parent)
Base.@propagate_inbounds Base.getindex(v::SegmentedBlockDiagonalMatrix, i::Int) = v.parent[i]
Base.@propagate_inbounds Base.setindex!(v::SegmentedBlockDiagonalMatrix, value, i::Int) = v.parent[i] = value
Base.IndexStyle(::Type{<:SegmentedBlockDiagonalMatrix}) = IndexLinear()
blocks(m::SegmentedBlockDiagonalMatrix) = m.blocks

end # module
