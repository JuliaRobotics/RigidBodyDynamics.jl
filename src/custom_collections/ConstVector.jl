## ConstVector
"""
$(TYPEDEF)

An immutable `AbstractVector` for which all elements are the same, represented
compactly and as a bitstype if the element type is a bitstype.
"""
struct ConstVector{T} <: AbstractVector{T}
    val::T
    length::Int64
end
Base.size(A::ConstVector) = (A.length, )
@propagate_inbounds Base.getindex(A::ConstVector, i::Int) = (@boundscheck checkbounds(A, i); A.val)
Base.IndexStyle(::Type{<:ConstVector}) = IndexLinear()
Base.unalias(dest, x::ConstVector) = x
