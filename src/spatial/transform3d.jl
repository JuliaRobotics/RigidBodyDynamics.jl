abstract type AbstractTransform3D{T} end
Base.eltype(::Type{<:AbstractTransform3D{T}}) where {T} = T

function print_transform_contents(io::IO, tf::AbstractTransform3D)
    angle_axis = AngleAxis(rotation(tf))
    angle = rotation_angle(angle_axis)
    axis = rotation_axis(angle_axis)
    print(io, "rotation: $(angle) rad about $(axis), translation: $(translation(tf))")
end


"""
$(TYPEDEF)

A homogeneous transformation matrix representing the transformation from one
three-dimensional Cartesian coordinate system to another.
"""
struct RawTransform3D{T} <: AbstractTransform3D{T}
    mat::SMatrix{4, 4, T, 16}
end

@inline RawTransform3D{T}(tf::RawTransform3D) where {T} = RawTransform3D{T}(similar_type(tf.mat, T)(tf.mat))
@inline RawTransform3D(tf::RawTransform3D) = RawTransform3D(copy(tf.mat))

@inline Base.convert(::Type{T}, tf::RawTransform3D) where {T<:RawTransform3D} = T(tf)

@inline function RawTransform3D{T}(rotation::Rotation{3}, translation::AbstractVector) where {T}
    R = convert(RotMatrix{3, T}, rotation)
    p = convert(SVector{3, T}, translation)
    @inbounds mat = @SMatrix [
        R[1] R[4] R[7] p[1];
        R[2] R[5] R[8] p[2];
        R[3] R[6] R[9] p[3];
        zero(T) zero(T) zero(T) one(T)]
    RawTransform3D{T}(mat)
end
@inline function RawTransform3D(rotation::Rotation{3}, translation::AbstractVector)
    T = promote_type(eltype(rotation), eltype(translation))
    RawTransform3D{T}(rotation, translation)
end

@inline RawTransform3D{T}(rotation::Rotation{3}) where {T} = RawTransform3D{T}(rotation, zero(SVector{3, T}))
@inline RawTransform3D(rotation::Rotation{3, T}) where {T} = RawTransform3D{T}(rotation)
@inline RawTransform3D{T}(translation::AbstractVector) where {T} = RawTransform3D{T}(one(RotMatrix{3, T}), translation)
@inline RawTransform3D(translation::AbstractVector{T}) where {T} = RawTransform3D{T}(translation)

@inline function rotation(tf::RawTransform3D{T}) where T
    @inbounds return RotMatrix{3, T}((tf.mat[1], tf.mat[2], tf.mat[3], tf.mat[5], tf.mat[6], tf.mat[7], tf.mat[9], tf.mat[10], tf.mat[11]))
end
@inline function translation(tf::RawTransform3D{T}) where T
    @inbounds return SVector{3, T}((tf.mat[13], tf.mat[14], tf.mat[15]))
end

@inline Base.one(::Type{RawTransform3D{T}}) where {T} = RawTransform3D(one(SMatrix{4, 4, T}))
@inline Base.one(::Type{RawTransform3D}) = one(RawTransform3D{Float64})

function Base.show(io::IO, tf::RawTransform3D{T}) where T
    println(io, "RawTransform3D{$T}:")
    print_transform_contents(io, tf)
end

@inline Base.:*(tf1::RawTransform3D, tf2::RawTransform3D) = RawTransform3D(tf1.mat * tf2.mat)
@inline function LinearAlgebra.inv(tf::RawTransform3D)
    rotinv = inv(rotation(tf))
    RawTransform3D(rotinv, -(rotinv * translation(tf)))
end

Random.rand(::Type{RawTransform3D{T}}) where {T} = RawTransform3D(rand(RotMatrix3{T}), rand(SVector{3, T}))
Random.rand(::Type{RawTransform3D}) = rand(RawTransform3D{Float64})

function Base.isapprox(x::RawTransform3D, y::RawTransform3D; atol::Real = 1e-12) # TODO: rtol, make default atol match Base
    isapprox(rotation(x), rotation(y), atol = atol) && isapprox(translation(x), translation(y), atol = atol)
end


"""
$(TYPEDEF)

A homogeneous transformation matrix representing the transformation from one
three-dimensional Cartesian coordinate system to another.

This type is annotated with [`CartesianFrame3D`](@ref)s. A `Transform3D` `tf`
with `tf.from == f1` `tf.to == f2` transforms spatial quantities (e.g. points)
expressed in `f1` to `f2`.
"""
struct Transform3D{T} <: AbstractTransform3D{T}
    raw::RawTransform3D{T}
    from::CartesianFrame3D
    to::CartesianFrame3D

    # Constructor takes frames and forwards remaining arguments to `RawTransform3D` constructor.
    # Note that the memory layout is different from the order of the arguments. This is for performance reasons.
    @inline function (::Type{T})(from::CartesianFrame3D, to::CartesianFrame3D, args...) where T<:Transform3D
        raw = rawtype(T)(args...)
        new{eltype(raw)}(raw, from, to)
    end
end

# TODO: deprecate.
@inline function Base.getproperty(tf::Transform3D, x::Symbol)
    x === :mat && return tf.raw.mat
    return getfield(tf, x)
end

rawtype(::Type{Transform3D{T}}) where {T} = RawTransform3D{T}
rawtype(::Type{Transform3D}) = RawTransform3D

@inline (::Type{T})(tf::Transform3D) where {T<:Transform3D} = T(tf.from, tf.to, rawtype(T)(tf.raw))

@inline Base.convert(::Type{T}, tf::Transform3D) where {T<:Transform3D} = T(tf)

@inline rotation(tf::Transform3D) = rotation(tf.raw)
@inline translation(tf::Transform3D) = translation(tf.raw)

@inline Base.one(::Type{T}, from::CartesianFrame3D, to::CartesianFrame3D) where {T<:Transform3D} = T(from, to, one(rawtype(T)))
@inline Base.one(::Type{T}, frame::CartesianFrame3D) where {T<:Transform3D} = one(T, frame, frame)

function Base.show(io::IO, tf::Transform3D{T}) where T
    println(io, "Transform3D{$T} from \"$(string(tf.from))\" to \"$(string(tf.to))\":")
    print_transform_contents(io, tf)
end

@inline Base.:*(tf1::Transform3D, tf2::Transform3D) = (@framecheck(tf1.from, tf2.to); Transform3D(tf2.from, tf1.to, tf1.raw * tf2.raw))
@inline LinearAlgebra.inv(tf::Transform3D) = Transform3D(tf.to, tf.from, inv(tf.raw))

Random.rand(::Type{T}, from::CartesianFrame3D, to::CartesianFrame3D) where {T<:Transform3D} = T(from, to, rand(rawtype(T)))

# TODO: rtol, make default atol match Base
function Base.isapprox(x::Transform3D, y::Transform3D; atol::Real = 1e-12)
    x.from == y.from && x.to == y.to && isapprox(x.raw, y.raw, atol = atol)
end
