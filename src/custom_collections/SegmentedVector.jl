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
@propagate_inbounds Base.getindex(v::SegmentedVector, i::Int) = v.parent[i]
@propagate_inbounds Base.setindex!(v::SegmentedVector, value, i::Int) = v.parent[i] = value
Base.dataids(x::SegmentedVector) = Base.dataids(x.parent)

Base.parent(v::SegmentedVector) = v.parent
segments(v::SegmentedVector) = v.segments
function ranges(v::SegmentedVector{K, <:Any, KeyRange}) where {K, KeyRange}
    segments = v.segments
    IndexDict{K, KeyRange, UnitRange{Int}}(segments.keys, map(segment -> first(parentindices(segment))::UnitRange{Int}, segments.values))
end

function Base.similar(v::SegmentedVector{K, T, KeyRange}, ::Type{S} = T) where {K, T, KeyRange, S}
    SegmentedVector{K, S, KeyRange}(similar(parent(v), S), ranges(v))
end
