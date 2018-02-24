"""
$(TYPEDEF)

A container that manages the creation and storage of [`MechanismState`](@ref)
objects of various scalar types, associated with a given `Mechanism`.

A `StateCache` can be used to write generic functions that use `MechanismState`
objects, while avoiding overhead due to the construction of a new `MechanismState`
with a given scalar type every time the function is called.

# Examples
```julia-repl
julia> mechanism = rand_tree_mechanism(Float64, Revolute{Float64}, Prismatic{Float64}, QuaternionFloating{Float64});

julia> cache = StateCache(mechanism)
StateCache{…}

julia> state32 = cache[Float32]
MechanismState{Float32, Float64, Float64, …}(…)

julia> cache[Float32] === state32
true

julia> cache[Float64]
MechanismState{Float64, Float64, Float64, …}(…)
```
"""
struct StateCache{M, JointCollection}
    mechanism::Mechanism{M}
    keys::Vector{Tuple{UInt64, Int}}
    states::Vector{MechanismState}
end

Base.show(io::IO, ::StateCache) = print(io, "StateCache{…}(…)")

function StateCache(mechanism::Mechanism{M}) where M
    JointCollection = typeof(TypeSortedCollection(joints(mechanism)))
    StateCache{M, JointCollection}(mechanism, [], [])
end

function motionsubspacetypes(JointTypes, ::Type{X}) where X
    Base.tuple_type_cons(motionsubspacetype(Base.tuple_type_head(JointTypes), X), motionsubspacetypes(Base.tuple_type_tail(JointTypes), X))
end
motionsubspacetypes(::Type{Tuple{}}, ::Type) = Tuple{}

function motionsubspacecollectiontype(::Type{TypeSortedCollection{D, N}}, ::Type{X}) where {D, N, X}
    TypeSortedCollection{vectortypes(motionsubspacetypes(eltypes(D), X)), N}
end

@inline function Base.getindex(c::StateCache{M, JC}, ::Type{X}) where {M, JC, X}
    C = promote_type(X, M)
    MSC = motionsubspacecollectiontype(JC, X)
    ReturnType = MechanismState{X, M, C, JC, MSC}
    key = (object_id(X), Threads.threadid())
    @inbounds for i = 1 : length(c.keys)
        if c.keys[i] === key
            return c.states[i]::ReturnType
        end
    end
    state = MechanismState{X}(c.mechanism)
    push!(c.keys, key)
    push!(c.states, state)
    state::ReturnType
end
