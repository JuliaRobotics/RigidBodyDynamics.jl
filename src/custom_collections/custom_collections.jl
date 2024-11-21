module CustomCollections

export
    ConstVector,
    ConstDict,
    NullDict,
    CacheElement,
    AbstractIndexDict,
    IndexDict,
    CacheIndexDict,
    SegmentedVector,
    SegmentedBlockDiagonalMatrix,
    UnorderedPair,
    CatVector

export
    foreach_with_extra_args,
    isdirty,
    segments,
    ranges

using TypeSortedCollections
using DocStringExtensions
using LinearAlgebra
using UnsafeArrays

using Base: @propagate_inbounds

include("foreach_with_extra_args.jl")
include("ConstVector.jl")
include("ConstDict.jl")
include("NullDict.jl")
include("CacheElement.jl")
include("IndexDict.jl")
include("SegmentedVector.jl")
include("SegmentedBlockDiagonalMatrix.jl")
include("UnorderedPair.jl")
include("CatVector.jl")

end # module
