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
Base.iterate(::NullDict) = nothing
