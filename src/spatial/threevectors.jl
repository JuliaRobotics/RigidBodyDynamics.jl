"""
$(TYPEDEF)

Represents a point in 3D Euclidean space, i.e. a [bound vector](https://en.wikipedia.org/wiki/Euclidean_vector#Overview).
The difference between an `AbstractPoint3D` and an [`AbstractFreeVector3D`](@ref) is that transforming an
`AbstractPoint3D` to a different frame rotates *and translates* the point, whereas `AbstractFreeVector3D`s
are only rotated.
"""
abstract type AbstractPoint3D{T} end

similar_free_vector_type(::Type{V}) where {V<:AbstractPoint3D} = similar_free_vector_type(V, eltype(V))

function Base.:*(tf::AbstractTransform3D, x::AbstractPoint3D) where {V<:AbstractPoint3D}
    v = rotation(tf) * vec(x) + translation(tf)
    similar_type(typeof(x), eltype(v))(v)
end

function Base.:\(tf::AbstractTransform3D, x::AbstractPoint3D)
    v = rotation(tf) \ (vec(x) - translation(tf))
    similar_type(typeof(x), eltype(v))(v)
end

function Base.:-(x::AbstractPoint3D, y::AbstractPoint3D)
    fx = similar_free_vector_type(typeof(x))(x)
    fy = similar_free_vector_type(typeof(y))(y)
    fx - fy
end

"""
$(TYPEDEF)

Represents a [free vector](https://en.wikipedia.org/wiki/Euclidean_vector#Overview)
in 3D Euclidean space. The difference between an `AbstractFreeVector3D` and an
[`AbstractPoint3D`](@ref) is that transforming an `AbstractFreeVector3D` to a different frame
only rotates the vector, whereas `AbstractPoint3D`s are rotated and translated.
"""
abstract type AbstractFreeVector3D{T} end

similar_free_vector_type(::Type{V}) where {V<:AbstractFreeVector3D} = similar_free_vector_type(V, eltype(V))

(::Type{V})(x::AbstractPoint3D) where {V<:AbstractFreeVector3D} = V(vec(x))

function Base.:*(tf::AbstractTransform3D, x::AbstractFreeVector3D)
    v = rotation(tf) * vec(x)
    similar_type(typeof(x), eltype(v))(v)
end

function Base.:\(tf::AbstractTransform3D, x::AbstractFreeVector3D)
    v = rotation(tf) \ vec(x)
    similar_type(typeof(x), eltype(v))(v)
end

function Base.:-(x::AbstractFreeVector3D, y::AbstractFreeVector3D)
    # TODO: not so nice to only use typeof(x) here:
    v = vec(x) - vec(y)
    similar_type(typeof(x), eltype(v))(v)
end

function LinearAlgebra.dot(x::AbstractFreeVector3D, y::AbstractFreeVector3D)
    LinearAlgebra.dot(vec(x), vec(y))
end

function LinearAlgebra.cross(x::AbstractFreeVector3D, y::AbstractFreeVector3D)
    # TODO: not so nice to only use typeof(x) here:
    v = cross(vec(x), vec(y))
    similar_type(typeof(x), eltype(v))(v)
end

for VectorType in [:AbstractPoint3D, :AbstractFreeVector3D]
    @eval begin
        V(x::V) where {V<:$VectorType} = V(vec(x))

        Base.eltype(::Type{<:$VectorType{T}}) where {T} = T

        transform(x::$VectorType, tf::AbstractTransform3D) = tf * x

        # TODO: rtol, make default atol match Base
        function Base.isapprox(x::$VectorType, y::$VectorType; atol::Real = 1e-12)
            isapprox(vec(x), vec(y); atol = atol)
        end

        # TODO: Base.zero? Random.rand?

        function Base.:/(x::$VectorType, s::Number)
            v = vec(x) / s
            similar_type(typeof(x), eltype(v))(v)
        end

        function Base.:*(x::$VectorType, s::Number)
            v = vec(x) * s
            similar_type(typeof(x), eltype(v))(v)
        end

        function Base.:*(s::Number, x::$VectorType)
            v = s * vec(x)
            similar_type(typeof(x), eltype(v))(v)
        end

        function Base.:+(x::$VectorType)
            v = +vec(x)
            similar_type(typeof(x), eltype(v))(v)
        end

        function Base.:-(x::$VectorType)
            v = -vec(x)
            similar_type(typeof(x), eltype(v))(v)
        end

        LinearAlgebra.norm(x::$VectorType, p::Real=2) = norm(vec(x), p)

        function LinearAlgebra.normalize(x::$VectorType, p::Real=2)
            v = normalize(vec(x), p)
            similar_type(typeof(x), eltype(v))(v)
        end

        Base.show(io::IO, x::$VectorType) = print(io, vec(x))

        function Base.show(io::IO, ::MIME"text/plain", x::$VectorType)
            print_short_type(io, typeof(x))
            println(":")
            print(io, x)
        end
    end
end

# Mixed
function Base.:+(x::AbstractPoint3D, y::AbstractFreeVector3D)
    v = vec(x) + vec(y)
    similar_type(typeof(x), eltype(v))(v)
end

function Base.:+(x::AbstractFreeVector3D, y::AbstractPoint3D)
    v = vec(x) + vec(y)
    similar_type(typeof(y), eltype(v))(v)
end

function LinearAlgebra.cross(x::AbstractPoint3D, y::AbstractFreeVector3D)
    fx = similar_free_vector_type(typeof(x))(x)
    cross(fx, y)
end


"""
$(TYPEDEF)

An [`AbstractPoint3D`](@ref) subtype that stores its data as a homogeneous vector.
"""
struct HPoint3D{T} <: AbstractPoint3D{T}
    data::SVector{4, T}

    @inline function HPoint3D{T}(data::SVector{4}) where T
        # TODO: replace with @assert when it gets elided
        @boundscheck data[4] == one(T) || throw(ArgumentError("Last element must be one!"))
        new{T}(convert(SVector{4, T}, data))
    end
end

# Interface
function HPoint3D{T}(vec::AbstractVector) where T
    @boundscheck length(vec) == 3 || throw(DimensionMismatch())
    @inbounds return HPoint3D{T}(SVector{4, T}(vec[1], vec[2], vec[3], one(eltype(vec))))
end
HPoint3D(vec::AbstractVector{T}) where {T} = HPoint3D{T}(vec)

"""
$(TYPEDEF)

An [`AbstractFreeVector3D`](@ref) subtype that stores its data as a homogeneous vector.
"""
struct HFreeVector3D{T} <: AbstractFreeVector3D{T}
    data::SVector{4, T}

    @inline function HFreeVector3D{T}(data::SVector{4}) where T
        # TODO: replace with @assert when it gets elided
        @boundscheck data[4] == zero(T) || throw(ArgumentError())
        new{T}(convert(SVector{4, T}, data))
    end
end

# Interface
function HFreeVector3D{T}(vec::AbstractVector) where T
    @boundscheck length(vec) == 3 || throw(DimensionMismatch())
    @inbounds return HFreeVector3D{T}(SVector{4, T}(vec[1], vec[2], vec[3], zero(eltype(vec))))
end
HFreeVector3D(vec::AbstractVector{T}) where {T} = HFreeVector3D{T}(vec)

# similar_bound_vector_type(::Type{<:HFreeVector3D}, ::Type{T}) where {T} = HPoint3D{T}

for VectorType in (:HFreeVector3D, :HPoint3D)
    @eval begin
        # Interface
        StaticArrays.similar_type(::Type{<:$VectorType}, ::Type{T}) where {T} = $VectorType{T}
        similar_free_vector_type(::Type{<:$VectorType}, ::Type{T}) where {T} = HFreeVector3D{T}

        $VectorType{T}(x::$VectorType) where {T} = $VectorType{T}(similar_type(x.data, T)(x.data))
        $VectorType(x::$VectorType) = $VectorType(copy(x.data))
        Base.convert(::Type{T}, x::$VectorType) where {T<:$VectorType} = T(x)
        Base.vec(x::$VectorType{T}) where {T} = @inbounds return SVector{3, T}((x.data[1], x.data[2], x.data[3]))

        $VectorType{T}(x, y, z) where {T} = $VectorType(SVector{3, T}(x, y, z))
        $VectorType(x, y, z) where {T} = $VectorType(SVector{3}(x, y, z))

        Base.zero(::Type{$VectorType{T}}) where {T} = $VectorType{T}(zero(SVector{3, T}))
        Base.zero(::Type{$VectorType}) = zero($VectorType{Float64})

        Random.rand(rng::AbstractRNG, ::Type{$VectorType{T}}) where {T} = $VectorType{T}(rand(rng, SVector{3, T}))
        Random.rand(rng::AbstractRNG, ::Type{$VectorType}) = rand(rng, $VectorType{Float64})

        # Convenience
        $VectorType(data::SVector{4, T}) where {T} = $VectorType{T}(data)

        # Optimizations
        Base.:+(x::$VectorType) = $VectorType(+(x.data))
        Base.:-(x::$VectorType, y::$VectorType) = HFreeVector3D(x.data - y.data)
        Base.:*(tf::HTransform3D, x::$VectorType) = $VectorType(tf.mat * x.data)
        # TODO: \(tf, x), maybe.
        # TODO: cross, maybe
    end
end

# HFreeVector3D-specific optimizations
Base.:/(x::HFreeVector3D, s::Number) = HFreeVector3D(x.data / s)
Base.:*(x::HFreeVector3D, s::Number) = HFreeVector3D(x.data * s)
Base.:*(s::Number, x::HFreeVector3D) = HFreeVector3D(s * x.data)
Base.:-(x::HFreeVector3D) = HFreeVector3D(-(x.data))
LinearAlgebra.dot(x::HFreeVector3D, y::HFreeVector3D) = LinearAlgebra.dot(x.data, y.data)
LinearAlgebra.norm(x::HFreeVector3D, p::Real=2) = norm(x.data, p)
LinearAlgebra.normalize(x::HFreeVector3D, p::Real=2) = HFreeVector3D(normalize(x.data, p))

# Mixed HPoint3D and HFreeVector3D optimizations
Base.:+(x::HPoint3D, y::HFreeVector3D) = HPoint3D(x.data + y.data)
Base.:+(x::HFreeVector3D, y::HPoint3D) = HPoint3D(x.data + y.data)
Base.:-(x::HPoint3D, y::HFreeVector3D) = HPoint3D(x.data - y.data)


# Frame-annotated types
for BaseVectorType in [:Point3D, :FreeVector3D]
    FrameVectorType = Symbol("Frame$BaseVectorType")
    AbstractVectorType = Symbol("Abstract$BaseVectorType")
    DefaultRawVectorType = Symbol("H$BaseVectorType")

    @eval begin
        """
        $(TYPEDEF)

        A frame-annotated [`$($(string(AbstractVectorType)))`](@ref) subtype.
        """
        struct $FrameVectorType{T, R<:$AbstractVectorType{T}} <: $AbstractVectorType{T}
            raw::R
            frame::CartesianFrame3D

            # Constructor takes frames and forwards remaining arguments to raw type constructor.
            # Note that the memory layout is different from the order of the arguments. This is for performance reasons.
            @inline function (::Type{T})(frame::CartesianFrame3D, args...) where T<:$FrameVectorType
                raw = rawtype(T)(args...)
                new{eltype(raw), typeof(raw)}(raw, frame)
            end
        end

        rawtype(::Type{$FrameVectorType{T, R}}) where {T, R} = R
        rawtype(::Type{<:$FrameVectorType{T}}) where {T} = $DefaultRawVectorType{T}
        rawtype(::Type{<:$FrameVectorType}) = $DefaultRawVectorType

        function StaticArrays.similar_type(::Type{V}, ::Type{T}) where {V<:$FrameVectorType, T}
            $FrameVectorType{T, similar_type(rawtype(V), T)}
        end

        $FrameVectorType{T}(x::$FrameVectorType) where {T} = $FrameVectorType{T}(x.frame, similar_type(typeof(x.raw), T)(x.raw))
        $FrameVectorType(x::$FrameVectorType) = $FrameVectorType(x.frame, typeof(x.raw)(x.raw))

        # TODO: convert methods

        Base.vec(x::$FrameVectorType) = vec(x.raw)

        # Frame-annotated types cannot adhere to the `vec` constructor interface on purpose,
        # so reimplement, forward to raw types, and juggle frames.

        # TODO: rtol, make default atol match Base
        function Base.isapprox(x::$FrameVectorType, y::$FrameVectorType; atol::Real = 1e-12)
            x.frame == y.frame && isapprox(x.raw, y.raw; atol=atol)
        end

        function Base.zero(::Type{V}, frame::CartesianFrame3D) where {V<:$FrameVectorType}
            V(frame, zero(rawtype(V)))
        end

        Base.zero(x::V) where {V<:$FrameVectorType} = V(x.frame, zero(rawtype(V)))

        function Random.rand(rng::AbstractRNG, ::Type{V}, frame::CartesianFrame3D) where {V<:$FrameVectorType}
            V(frame, rand(rng, rawtype(V)))
        end
        Random.rand(::Type{V}, frame::CartesianFrame3D) where {V<:$FrameVectorType} = rand(Random.GLOBAL_RNG, V, frame)

        function Base.:*(tf::FrameTransform3D, x::$FrameVectorType)
            @framecheck tf.from x.frame
            $FrameVectorType(tf.to, tf.raw * x.raw)
        end

        function Base.:\(tf::FrameTransform3D, x::$FrameVectorType)
            @framecheck tf.to x.frame
            $FrameVectorType(tf.from, tf.raw \ x.raw)
        end

        Base.:/(x::$FrameVectorType, s::Number) = $FrameVectorType(x.frame, x.raw / s)
        Base.:*(x::$FrameVectorType, s::Number) = $FrameVectorType(x.frame, x.raw * s)
        Base.:*(s::Number, x::$FrameVectorType) = $FrameVectorType(x.frame, s * x.raw)
        Base.:+(x::$FrameVectorType) = $FrameVectorType(x.frame, +(x.raw))
        Base.:-(x::$FrameVectorType) = $FrameVectorType(x.frame, -(x.raw))

        function LinearAlgebra.normalize(x::$FrameVectorType, p::Real=2)
            $FrameVectorType(x.frame, normalize(x.raw, p))
        end

        function Base.show(io::IO, x::$FrameVectorType)
            invoke(Base.show, Tuple{typeof(io), $AbstractVectorType}, io, x.raw)
            print(io, " (in \"$(string(x.frame))\")")
        end

        # TODO: deprecate.
        @inline function Base.getproperty(x::$FrameVectorType, field::Symbol)
            field === :v && return vec(x.raw)
            return getfield(x, field)
        end
    end
end

# FramePoint3D-specific
function Base.:-(x::FramePoint3D, y::FramePoint3D)
    @framecheck x.frame y.frame
    FrameFreeVector3D(x.frame, x.raw - y.raw)
end

# FrameFreeVector3D-specific
(::Type{V})(x::FramePoint3D) where {V<:FrameFreeVector3D} = V(x.frame, vec(x))

function Base.:-(x::FrameFreeVector3D, y::FrameFreeVector3D)
    @framecheck x.frame y.frame
    FrameFreeVector3D(x.frame, x.raw - y.raw)
end

function LinearAlgebra.dot(x::FrameFreeVector3D, y::FrameFreeVector3D)
    @framecheck x.frame y.frame
    dot(x.raw, y.raw)
end

function LinearAlgebra.cross(x::FrameFreeVector3D, y::FrameFreeVector3D)
    @framecheck x.frame y.frame
    FrameFreeVector3D(x.frame, cross(x.raw, y.raw))
end

# Mixed FramePoint3D and FrameFreeVector3D
function Base.:+(x::FramePoint3D, y::FrameFreeVector3D)
    @framecheck x.frame y.frame
    FramePoint3D(x.frame, x.raw + y.raw)
end

function Base.:+(x::FrameFreeVector3D, y::FramePoint3D)
    @framecheck x.frame y.frame
    FramePoint3D(x.frame, x.raw + y.raw)
end

function Base.:-(x::FramePoint3D, y::FrameFreeVector3D)
    @framecheck x.frame y.frame
    FramePoint3D(x.frame, x.raw - y.raw)
end

function LinearAlgebra.cross(x::FramePoint3D, y::FrameFreeVector3D)
    @framecheck x.frame y.frame
    FrameFreeVector3D(x.frame, cross(x.raw, y.raw))
end

const Point3D{T} = FramePoint3D{T, HPoint3D{T}}
const FreeVector3D{T} = FrameFreeVector3D{T, HFreeVector3D{T}}
