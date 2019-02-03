abstract type AbstractTransform3D{T} end
Base.eltype(::Type{<:AbstractTransform3D{T}}) where {T} = T

function Base.show(io::IO, tf::AbstractTransform3D)
    angle_axis = AngleAxis(rotation(tf))
    angle = rotation_angle(angle_axis)
    axis = rotation_axis(angle_axis)
    print(io, "rotation: $(angle) rad about $(axis), translation: $(translation(tf))")
end

function Base.show(io::IO, ::MIME"text/plain", tf::AbstractTransform3D)
    print_short_type(io, typeof(tf))
    println(":")
    print(io, tf)
end

"""
$(TYPEDEF)

Represents the transformation from one three-dimensional Cartesian coordinate system to
another, stored as a ``4 \\times 4``  homogeneous transformation matrix.
"""
struct HomogeneousTransform3D{T} <: AbstractTransform3D{T}
    mat::SMatrix{4, 4, T, 16}
end

@inline HomogeneousTransform3D{T}(tf::HomogeneousTransform3D) where {T} = HomogeneousTransform3D{T}(similar_type(tf.mat, T)(tf.mat))
@inline HomogeneousTransform3D(tf::HomogeneousTransform3D) = HomogeneousTransform3D(copy(tf.mat))

@inline Base.convert(::Type{T}, tf::HomogeneousTransform3D) where {T<:HomogeneousTransform3D} = T(tf)

@inline function HomogeneousTransform3D{T}(rotation::Rotation{3}, translation::AbstractVector) where {T}
    R = convert(RotMatrix{3, T}, rotation)
    p = convert(SVector{3, T}, translation)
    @inbounds mat = @SMatrix [
        R[1] R[4] R[7] p[1];
        R[2] R[5] R[8] p[2];
        R[3] R[6] R[9] p[3];
        zero(T) zero(T) zero(T) one(T)]
    HomogeneousTransform3D{T}(mat)
end
@inline function HomogeneousTransform3D(rotation::Rotation{3}, translation::AbstractVector)
    T = promote_type(eltype(rotation), eltype(translation))
    HomogeneousTransform3D{T}(rotation, translation)
end

@inline HomogeneousTransform3D{T}(rotation::Rotation{3}) where {T} = HomogeneousTransform3D{T}(rotation, zero(SVector{3, T}))
@inline HomogeneousTransform3D(rotation::Rotation{3, T}) where {T} = HomogeneousTransform3D{T}(rotation)
@inline HomogeneousTransform3D{T}(translation::AbstractVector) where {T} = HomogeneousTransform3D{T}(one(RotMatrix{3, T}), translation)
@inline HomogeneousTransform3D(translation::AbstractVector{T}) where {T} = HomogeneousTransform3D{T}(translation)

@inline function rotation(tf::HomogeneousTransform3D{T}) where T
    @inbounds return RotMatrix{3, T}((tf.mat[1], tf.mat[2], tf.mat[3], tf.mat[5], tf.mat[6], tf.mat[7], tf.mat[9], tf.mat[10], tf.mat[11]))
end
@inline function translation(tf::HomogeneousTransform3D{T}) where T
    @inbounds return SVector{3, T}((tf.mat[13], tf.mat[14], tf.mat[15]))
end

@inline Base.one(::Type{HomogeneousTransform3D{T}}) where {T} = HomogeneousTransform3D(one(SMatrix{4, 4, T}))
@inline Base.one(::Type{HomogeneousTransform3D}) = one(HomogeneousTransform3D{Float64})

@inline Base.:*(tf1::HomogeneousTransform3D, tf2::HomogeneousTransform3D) = HomogeneousTransform3D(tf1.mat * tf2.mat)
@inline function LinearAlgebra.inv(tf::HomogeneousTransform3D)
    rotinv = inv(rotation(tf))
    HomogeneousTransform3D(rotinv, -(rotinv * translation(tf)))
end

Random.rand(::Type{HomogeneousTransform3D{T}}) where {T} = HomogeneousTransform3D(rand(RotMatrix3{T}), rand(SVector{3, T}))
Random.rand(::Type{HomogeneousTransform3D}) = rand(HomogeneousTransform3D{Float64})

function Base.isapprox(x::HomogeneousTransform3D, y::HomogeneousTransform3D; atol::Real = 1e-12) # TODO: rtol, make default atol match Base
    isapprox(rotation(x), rotation(y), atol = atol) && isapprox(translation(x), translation(y), atol = atol)
end


"""
$(TYPEDEF)

Represents the transformation from one three-dimensional Cartesian coordinate system to another.

This type is annotated with [`CartesianFrame3D`](@ref)s. A `FrameTransform3D` `tf`
with `tf.from == f1` `tf.to == f2` transforms spatial quantities (e.g. points)
expressed in `f1` to `f2`.
"""
struct FrameTransform3D{T, R<:AbstractTransform3D{T}} <: AbstractTransform3D{T}
    raw::R
    from::CartesianFrame3D
    to::CartesianFrame3D

    # Constructor takes frames and forwards remaining arguments to raw type constructor.
    # Note that the memory layout is different from the order of the arguments. This is for performance reasons.
    @inline function (::Type{T})(from::CartesianFrame3D, to::CartesianFrame3D, args...) where T<:FrameTransform3D
        raw = rawtype(T)(args...)
        new{eltype(raw), typeof(raw)}(raw, from, to)
    end
end

# TODO: deprecate.
@inline function Base.getproperty(tf::FrameTransform3D, x::Symbol)
    x === :mat && return tf.raw.mat
    return getfield(tf, x)
end

rawtype(::Type{FrameTransform3D{T, R}}) where {T, R} = R
rawtype(::Type{<:FrameTransform3D{T}}) where {T} = HomogeneousTransform3D{T}
rawtype(::Type{<:FrameTransform3D}) = HomogeneousTransform3D

@inline (::Type{T})(tf::FrameTransform3D) where {T<:FrameTransform3D} = T(tf.from, tf.to, rawtype(T)(tf.raw))

@inline Base.convert(::Type{T}, tf::FrameTransform3D) where {T<:FrameTransform3D} = T(tf)

@inline rotation(tf::FrameTransform3D) = rotation(tf.raw)
@inline translation(tf::FrameTransform3D) = translation(tf.raw)

@inline Base.one(::Type{T}, from::CartesianFrame3D, to::CartesianFrame3D) where {T<:FrameTransform3D} = T(from, to, one(rawtype(T)))
@inline Base.one(::Type{T}, frame::CartesianFrame3D) where {T<:FrameTransform3D} = one(T, frame, frame)

function Base.show(io::IO, ::MIME"text/plain", tf::FrameTransform3D{T}) where T
    print_short_type(io, typeof(tf))
    println(io, " from \"$(string(tf.from))\" to \"$(string(tf.to))\":")
    print(io, tf.raw)
end

@inline Base.:*(tf1::FrameTransform3D, tf2::FrameTransform3D) = (@framecheck(tf1.from, tf2.to); FrameTransform3D(tf2.from, tf1.to, tf1.raw * tf2.raw))
@inline LinearAlgebra.inv(tf::FrameTransform3D) = FrameTransform3D(tf.to, tf.from, inv(tf.raw))

Random.rand(::Type{T}, from::CartesianFrame3D, to::CartesianFrame3D) where {T<:FrameTransform3D} = T(from, to, rand(rawtype(T)))

# TODO: rtol, make default atol match Base
function Base.isapprox(x::FrameTransform3D, y::FrameTransform3D; atol::Real = 1e-12)
    x.from == y.from && x.to == y.to && isapprox(x.raw, y.raw, atol = atol)
end

const Transform3D{T} = FrameTransform3D{T, HomogeneousTransform3D{T}}
