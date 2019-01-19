# Point3D, FreeVector3D. The difference is that a FreeVector3D is only rotated when its frame is changed,
# whereas a Point3D is also translated
for VectorType in (:FreeVector3D, :Point3D)
    RawVectorType = Symbol("Raw$VectorType")
    FrameVectorType = Symbol("Frame$VectorType")
    @eval begin
        # TODO: consider storing as a homogeneous vector
        struct $VectorType{T, F<:FrameOrNothing}
            frame::F
            v::SVector{3, T} # TODO: rename to vec?

            @inline function $VectorType{T, F}(frame::F, v::SVector{3, T}) where {T, F<:FrameOrNothing}
                new{T, F}(frame, v)
            end
        end

        const $RawVectorType{T} = $VectorType{T, Nothing}
        const $FrameVectorType{T} = $VectorType{T, CartesianFrame3D}

        hasframes(::Type{<:$FrameVectorType}) = true

        # Constructors
        @inline function $VectorType{T, F}(frame::F, v::AbstractVector) where {T, F<:FrameOrNothing}
            $VectorType{T, F}(frame, SVector{3}(v))
        end

        @inline function $VectorType{T}(frame::F, v::AbstractVector) where {T, F<:FrameOrNothing}
            $VectorType{T, F}(frame, v)
        end

        @inline function $VectorType(frame::F, v::AbstractVector{T}) where {T, F<:FrameOrNothing}
            $VectorType{T, F}(frame, v)
        end

        @inline function (::Type{V})(frame::FrameOrNothing, x, y, z) where {V<:$VectorType}
            V(frame, SVector{3}(x, y, z))
        end

        @inline (::Type{V})(v::AbstractVector) where {V<:$VectorType} = V(nothing, v)
        @inline (::Type{V})(x, y, z) where {V<:$VectorType} = V(nothing, x, y, z)

        @inline (::Type{V})(v::V) where {V<:$VectorType} = v
        @inline $VectorType{T, F}(v::$VectorType) where {T, F<:FrameOrNothing} = $VectorType{T, F}(v.frame, v.v)
        @inline $VectorType{T}(v::$VectorType) where {T} = $VectorType{T}(v.frame, v.v)

        # TODO: deprecate:
        $VectorType(::Type{T}, frame::CartesianFrame3D) where {T} = $VectorType(frame, zero(SVector{3, T}))

        # Conversion
        @inline Base.convert(::Type{V}, v::$VectorType) where {V<:$VectorType} = V(v)

        # Getters
        @inline Base.vec(v::$VectorType) = v.v
        @propagate_inbounds Base.getindex(v::$VectorType, i::Integer) = getindex(v.v, i)
        Base.eltype(::Type{$VectorType{T}}) where {T} = T

        # Pretty-printing
        function Base.show(io::IO, v::$VectorType)
            print(io, "$($(string(VectorType)))")
            if hasframes(v)
                print(io, " in \"$(string(v.frame))\"")
            end
            print(io, ": $(v.v)")
        end

        # Operations
        Base.:/(v::$VectorType, s::Number) = $VectorType(v.frame, v.v / s)
        Base.:*(v::$VectorType, s::Number) = $VectorType(v.frame, v.v * s)
        Base.:*(s::Number, v::$VectorType) = $VectorType(v.frame, s * v.v)
        Base.:-(v::$VectorType) = $VectorType(v.frame, -v.v)
        Base.:+(v::$VectorType) = v

        function Random.rand(::Type{V}, frame::F) where {T, V<:$VectorType{T}, F<:FrameOrNothing}
            $VectorType(frame, rand(SVector{3, T}))
        end

        function Random.rand(::Type{$VectorType}, frame::F) where F<:FrameOrNothing
            rand($VectorType{Float64}, frame)
        end

        Random.rand(::Type{V}) where {V<:$VectorType} = rand(V, nothing)

        # TODO: deprecate:
        Random.rand(::Type{$VectorType}, ::Type{T}, frame::CartesianFrame3D) where {T} = $VectorType(frame, rand(SVector{3, T}))

        # TODO: reltol, change default atol
        Base.isapprox(x::$VectorType, y::$VectorType; atol::Real = 1e-12) = x.frame == y.frame && isapprox(x.v, y.v; atol = atol)

        """
        $(SIGNATURES)

        Return `x` transformed to `CartesianFrame3D` `t.from`.
        """
        transform(x::$VectorType, t::Transform3D) = t * x

        LinearAlgebra.norm(v::$VectorType) = norm(v.v)
        LinearAlgebra.normalize(v::$VectorType, p = 2) = $VectorType(v.frame, normalize(v.v, p))
    end
end

"""
$(TYPEDEF)

A `Point3D` represents a position in a given coordinate system.

A `Point3D` is a [bound vector](https://en.wikipedia.org/wiki/Euclidean_vector#Overview).
Applying a `Transform3D` to a `Point3D` both rotates and translates the
`Point3D`.
"""
Point3D

"""
$(TYPEDEF)

A `FreeVector3D` represents a [free vector](https://en.wikipedia.org/wiki/Euclidean_vector#Overview).

Examples of free vectors include displacements and velocities of points.

Applying a `Transform3D` to a `FreeVector3D` only rotates the `FreeVector3D`.
"""
FreeVector3D

# Point3D-specific
Base.:-(p1::Point3D, p2::Point3D) = FreeVector3D(@sameframe(p1.frame, p2.frame), p1.v - p2.v)

function Base.:*(tf::Transform3D, p::Point3D)
    if hasframes(tf) && hasframes(p)
        @framecheck(tf.from, p.frame)
    end
    Point3D(tf.to, rotation(tf) * p.v + translation(tf))
end

function Base.:\(tf::Transform3D, p::Point3D)
    if hasframes(tf) && hasframes(p)
        @framecheck(p.frame, tf.to)
    end
    Point3D(tf.from, rotation(tf) \ (p.v - translation(tf)))
end

# FreeVector3D-specific
FreeVector3D(p::Point3D) = FreeVector3D(p.frame, p.v)

for op in [:+, :-]
    @eval Base.$op(v1::FreeVector3D, v2::FreeVector3D) = FreeVector3D(@sameframe(v1.frame, v2.frame), $op(v1.v, v2.v))
end

function Base.:*(tf::Transform3D, v::FreeVector3D)
    if hasframes(tf) && hasframes(v)
        @framecheck(tf.from, v.frame)
    end
    FreeVector3D(tf.to, rotation(tf) * v.v)
end

function Base.:\(tf::Transform3D, v::FreeVector3D)
    if hasframes(tf) && hasframes(v)
        @framecheck v.frame tf.to
    end
    FreeVector3D(tf.from, rotation(tf) \ v.v)
end

# Mixed Point3D and FreeVector3D
LinearAlgebra.cross(v1::FreeVector3D, v2::FreeVector3D) = FreeVector3D(@sameframe(v1.frame, v2.frame), cross(v1.v, v2.v))
Base.:+(p::Point3D, v::FreeVector3D) = Point3D(@sameframe(p.frame, v.frame), p.v + v.v)
Base.:+(v::FreeVector3D, p::Point3D) = Point3D(@sameframe(v.frame, p.frame), v.v + p.v)
Base.:-(p::Point3D, v::FreeVector3D) = Point3D(@sameframe(p.frame, v.frame), p.v - v.v)
LinearAlgebra.cross(p::Point3D, v::FreeVector3D) = FreeVector3D(@sameframe(p.frame, v.frame), cross(p.v, v.v))
LinearAlgebra.cross(v::FreeVector3D, p::Point3D) = FreeVector3D(@sameframe(v.frame, p.frame), cross(v.v, p.v))
