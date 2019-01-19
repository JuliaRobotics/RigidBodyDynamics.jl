"""
$(TYPEDEF)

A homogeneous transformation matrix representing the transformation from one
three-dimensional Cartesian coordinate system to another.
"""
struct Transform3D{T, F<:Union{CartesianFrame3D, Nothing}}
    from::F
    to::F
    mat::SMatrix{4, 4, T, 16}
end

const RawTransform3D{T} = Transform3D{T, Nothing}
const FrameTransform3D{T} = Transform3D{T, CartesianFrame3D}

hasframes(::Type{<:FrameTransform3D}) = true

# Constructors
@inline function Transform3D{T, F}(from::F, to::F, rotation::RotMatrix{3}, translation::SVector{3}) where {T, F<:Union{CartesianFrame3D, Nothing}}
    R = convert(RotMatrix{3, T}, rotation)
    p = convert(SVector{3, T}, translation)
    @inbounds mat = @SMatrix [R[1] R[4] R[7] p[1];
                              R[2] R[5] R[8] p[2];
                              R[3] R[6] R[9] p[3];
                              zero(T) zero(T) zero(T) one(T)]
    Transform3D{T, F}(from, to, mat)
end

@inline function Transform3D{T}(from::F, to::F, rotation::RotMatrix{3}, translation::SVector{3}) where {T, F<:Union{CartesianFrame3D, Nothing}}
    Transform3D{T, F}(from, to, rotation, translation)
end

@inline function Transform3D(from::F, to::F, rotation::RotMatrix{3}, translation::SVector{3}) where F<:Union{CartesianFrame3D, Nothing}
    T = promote_type(eltype(typeof(rotation)), eltype(typeof(translation)))
    Transform3D{T}(from, rotation, translation)
end

@inline function (::Type{TF})(from::F, to::F, rotation::Rotation{3}, translation::AbstractVector) where {TF<:Transform3D, F<:Union{CartesianFrame3D, Nothing}}
    TF(from, to, RotMatrix(rotation), SVector{3}(translation))
end

@inline function (::Type{TF})(from::F, to::F, rotation::Rotation{3, T}) where {T, TF<:Transform3D, F<:Union{CartesianFrame3D, Nothing}}
    TF(from, to, rotation, zero(SVector{3, T}))
end

@inline function (::Type{TF})(from::F, to::F, translation::AbstractVector{T}) where {T, TF<:Transform3D, F<:Union{CartesianFrame3D, Nothing}}
    TF(from, to, one(RotMatrix3{T}), translation)
end

@inline (::Type{TF})(rotation::Rotation{3}, translation::AbstractVector) where {TF<:Transform3D} = TF(nothing, nothing, rotation, translation)
@inline (::Type{TF})(rotation::Rotation{3}) where {TF<:Transform3D} = TF(nothing, nothing, rotation)
@inline (::Type{TF})(translation::AbstractVector) where {TF<:Transform3D} = TF(nothing, nothing, translation)

@inline (::Type{TF})(tf::TF) where {TF<:Transform3D} = tf
@inline Transform3D{T, F}(tf::Transform3D) where {T, F<:Union{CartesianFrame3D, Nothing}} = Transform3D{T, F}(tf.from, tf.to, tf.mat)
@inline Transform3D{T}(tf::Transform3D) where {T} = Transform3D{T}(tf.from, tf.to, tf.mat)

# Conversion
@inline Base.convert(::Type{TF}, tf::Transform3D) where {TF<:Transform3D} = TF(tf)

# Getters
@inline rotation(tf::Transform3D) = @inbounds return RotMatrix(tf.mat[1], tf.mat[2], tf.mat[3], tf.mat[5], tf.mat[6], tf.mat[7], tf.mat[9], tf.mat[10], tf.mat[11])
@inline translation(tf::Transform3D) = @inbounds return SVector(tf.mat[13], tf.mat[14], tf.mat[15])

# Pretty-printing
function Base.show(io::IO, tf::Transform3D)
    print(io, "Transform3D")
    if hasframes(tf)
        print(io, "from \"$(string(tf.from))\" to \"$(string(tf.to))\"")
    end
    println(io, ":")
    angle_axis = AngleAxis(rotation(tf))
    angle = rotation_angle(angle_axis)
    axis = rotation_axis(angle_axis)
    print(io, "rotation: $(angle) rad about $(axis), translation: $(translation(tf))")
end

# Operations
@inline function Base.:*(tf1::Transform3D, tf2::Transform3D)
    if hasframes(tf1) && hasframes(tf2)
        @framecheck(tf1.from, tf2.to)
    end
    Transform3D(tf2.from, tf1.to, tf1.mat * tf2.mat)
end

@inline function Base.inv(tf::Transform3D)
    invrot = transpose(rotation(tf))
    Transform3D(tf.to, tf.from, invrot, -(invrot * translation(tf)))
end

function Base.one(::Type{TF}, from::F, to::F) where {T, TF<:Transform3D{T}, F<:Union{CartesianFrame3D, Nothing}}
    TF(from, to, one(SMatrix{4, 4, T}))
end

function Base.one(::Type{Transform3D}, from::F, to::F) where {T, TF<:Transform3D{T}, F<:Union{CartesianFrame3D, Nothing}}
    one(Transform3D{Float64}, from, to)
end

Base.one(::Type{TF}) where {TF<:Transform3D} = one(TF, nothing, nothing)

function Random.rand(::Type{TF}, from::F, to::F) where {T, TF<:Transform3D{T}, F<:Union{CartesianFrame3D, Nothing}}
    rotation = rand(RotMatrix3{T})
    translation = rand(SVector{3, T})
    TF(from, to, rotation, translation)
end

function Random.rand(::Type{Transform3D}, from::F, to::F) where F<:Union{CartesianFrame3D, Nothing}
    rand(Transform3D{Float64}, from, to)
end

Random.rand(::Type{TF}) where {TF<:Transform3D} = rand(TF, nothing, nothing)

# TODO: reltol, change default atol
function Base.isapprox(x::Transform3D, y::Transform3D; atol::Real = 1e-12)
    x.from == y.from && x.to == y.to && isapprox(rotation(x), rotation(y), atol = atol) && isapprox(translation(x), translation(y), atol = atol)
end
