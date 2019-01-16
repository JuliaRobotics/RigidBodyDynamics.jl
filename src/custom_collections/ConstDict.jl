## ConstDict
"""
An immutable `AbstractDict` for which the value is the same, no matter what the key is.
"""
struct ConstDict{K, V} <: AbstractDict{K, V}
    val::V
    ConstDict{K}(val::V) where {K, V} = new{K, V}(val)
end
Base.getindex(d::ConstDict{K}, key) where K = d.val
Base.show(io::IO, ::MIME"text/plain", d::ConstDict{K}) where {K} = show(io, d)
Base.show(io::IO, d::ConstDict{K}) where {K} = print(io, "$(typeof(d)) with fixed value $(d.val)")
