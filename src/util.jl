# associative type that signifies an empty dictionary and does not allocate memory
type NullDict{K, V} <: Associative{K, V}
end

import Base: length, start, done, get, haskey
length(::NullDict) = 0
start(::NullDict) = 0
done(::NullDict, state) = true
get(::NullDict, key, default) = default
haskey(::NullDict, k) = false

# convert{N, T}(::Type{SVector{N, T}}, v::SVector{N, T}) = v

# type of a view of a vector
# TODO: a bit too specific
typealias VectorSegment{T} SubArray{T,1,Array{T, 1},Tuple{UnitRange{Int64}},true}
