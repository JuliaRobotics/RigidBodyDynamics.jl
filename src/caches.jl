"""
$(TYPEDEF)

Returns a `TypeDict` that manages the creation and storage of [`MechanismState`](@ref)
objects of various scalar types, associated with a given `Mechanism`.

A `StateCache` can be used to write generic functions that use `MechanismState`
objects, while avoiding overhead due to the construction of a new `MechanismState`
with a given scalar type every time the function is called.

# Examples
```julia-repl
julia> mechanism = rand_tree_mechanism(Float64, Revolute{Float64}, Prismatic{Float64}, QuaternionFloating{Float64});

julia> cache = StateCache(mechanism);

julia> state32 = cache[Float32]
MechanismState{Float32, Float64, Float64, …}(…)

julia> cache[Float32] === state32
true

julia> cache[Float64]
MechanismState{Float64, Float64, Float64, …}(…)
```
"""
function StateCache(mechanism::Mechanism{M}) where M
    value_constructor = let mechanism=mechanism
        @inline function (T)
            MechanismState{T}(mechanism)
        end
    end
    value_type_fun = let JointCollection=typeof(TypeSortedCollection(joints(mechanism)))
        @inline function (T)
            MechanismState{T, M, promote_type(T, M), JointCollection}
        end
    end
    TypeDict(value_constructor, value_type_fun)
end

"""
$(TYPEDEF)

Returns a `TypeDict` that manages the creation and storage of [`DynamicsResult`](@ref)
objects of various scalar types, associated with a given `Mechanism`.
Similar to [`StateCache`](@ref).
"""
function DynamicsResultCache(mechanism::Mechanism{M}) where M
    value_constructor = let mechanism=mechanism
        @inline function(T)
            DynamicsResult{T}(mechanism)
        end
    end
    value_type_fun = @inline function (T)
        DynamicsResult{T, M}
    end
    TypeDict(value_constructor, value_type_fun)
end

"""
$(TYPEDEF)

Returns a `TypeDict` that manages the creation and storage of heterogeneously typed [`SegmentedVector`](@ref)
objects. Similar to [`StateCache`](@ref).
"""
function SegmentedVectorCache(ranges::IndexDict{K, KeyRange, UnitRange{Int}}) where {K, KeyRange<:AbstractUnitRange{K}}
    value_constructor = let ranges=ranges, l=sum(length, values(ranges))
        @inline function (T)
            SegmentedVector{K, T, KeyRange}(Vector{T}(undef, l), ranges)
        end
    end
    value_type_fun = @inline function (T)
        SegmentedVector{K, T, KeyRange, Vector{T}}
    end
    TypeDict(value_constructor, value_type_fun)
end
