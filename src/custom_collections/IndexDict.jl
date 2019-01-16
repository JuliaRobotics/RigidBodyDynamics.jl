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

Base.broadcastable(x::IndexDict) = Ref(x)

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

setdirty!(d::CacheIndexDict) = (d.dirty = true; nothing)
isdirty(d::CacheIndexDict) = d.dirty

# Constructors
for IDict in (:IndexDict, :CacheIndexDict)
    @eval begin
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
@inline Base.iterate(d::AbstractIndexDict, i = 1) = i > length(d) ? nothing : (d.keys[i] => d.values[i], i + 1)
@inline Base.keys(d::AbstractIndexDict{K}) where {K} = d.keys
@inline Base.values(d::AbstractIndexDict) = d.values
@inline Base.haskey(d::AbstractIndexDict, key) = key ∈ d.keys
@inline keyindex(key::K, keyrange::Base.OneTo{K}) where {K} = Int(key)
@inline keyindex(key::K, keyrange::UnitRange{K}) where {K} = Int(key - first(keyrange) + 1)
@propagate_inbounds Base.getindex(d::AbstractIndexDict{K}, key::K) where {K} = d.values[keyindex(key, d.keys)]
@propagate_inbounds Base.setindex!(d::AbstractIndexDict{K}, value, key::K) where {K} = d.values[keyindex(key, d.keys)] = value
@propagate_inbounds Base.get(d::AbstractIndexDict{K}, key::K, default) where {K} = d[key]
