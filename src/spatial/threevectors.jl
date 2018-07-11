# Point3D, FreeVector3D. The difference is that a FreeVector3D is only rotated when its frame is changed,
# whereas a Point3D is also translated
for VectorType in (:FreeVector3D, :Point3D)
    @eval begin
        # TODO: consider storing as a homogeneous vector
        struct $VectorType{V<:AbstractVector}
            v::V
            frame::CartesianFrame3D

            @inline function $VectorType(frame::CartesianFrame3D, v::V) where {V}
                @boundscheck length(v) == 3 || throw(DimensionMismatch())
                new{V}(v, frame)
            end
        end

        $VectorType(::Type{T}, frame::CartesianFrame3D) where {T} = $VectorType(frame, zero(SVector{3, T}))
        $VectorType(frame::CartesianFrame3D, x::Number, y::Number, z::Number) = $VectorType(frame, SVector(x, y, z))

        Base.convert(::Type{$VectorType{V}}, p::$VectorType{V}) where {V} = p
        Base.convert(::Type{$VectorType{V}}, p::$VectorType) where {V} = $VectorType(p.frame, convert(V, p.v))

        Base.zero(p::$VectorType) = $VectorType(p.frame, zero(p.v))

        Base.:/(p::$VectorType, s::Number) = $VectorType(p.frame, p.v / s)
        Base.:*(p::$VectorType, s::Number) = $VectorType(p.frame, p.v * s)
        Base.:*(s::Number, p::$VectorType) = $VectorType(p.frame, s * p.v)
        Base.:-(p::$VectorType) = $VectorType(p.frame, -p.v)

        Random.rand(::Type{$VectorType}, ::Type{T}, frame::CartesianFrame3D) where {T} = $VectorType(frame, rand(SVector{3, T}))
        Base.show(io::IO, p::$VectorType) = print(io, "$($(string(VectorType))) in \"$(string(p.frame))\": $(p.v)")
        Base.isapprox(x::$VectorType, y::$VectorType; atol::Real = 1e-12) = x.frame == y.frame && isapprox(x.v, y.v; atol = atol)

        """
        $(SIGNATURES)

        Return `x` transformed to `CartesianFrame3D` `t.from`.
        """
        transform(x::$VectorType, t::Transform3D) = t * x

        Base.eltype(::Type{$VectorType{V}}) where {V} = eltype(V)
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
Base.:-(p1::Point3D, p2::Point3D) = begin @framecheck(p1.frame, p2.frame); FreeVector3D(p1.frame, p1.v - p2.v) end
Base.:*(t::Transform3D, point::Point3D) = begin @framecheck(t.from, point.frame); Point3D(t.to, rotation(t) * point.v + translation(t)) end
Base.:\(t::Transform3D, point::Point3D) = begin @framecheck point.frame t.to; Point3D(t.from, rotation(t) \ (point.v - translation(t))) end

# FreeVector3D-specific
FreeVector3D(p::Point3D) = FreeVector3D(p.frame, p.v)
Base.:-(v1::FreeVector3D, v2::FreeVector3D) = begin @framecheck(v1.frame, v2.frame); FreeVector3D(v1.frame, v1.v - v2.v) end
LinearAlgebra.cross(v1::FreeVector3D, v2::FreeVector3D) = begin @framecheck(v1.frame, v2.frame); FreeVector3D(v1.frame, v1.v × v2.v) end
LinearAlgebra.dot(v1::FreeVector3D, v2::FreeVector3D) = begin @framecheck(v1.frame, v2.frame); v1.v ⋅ v2.v end
Base.:*(t::Transform3D, vector::FreeVector3D) = begin @framecheck(t.from, vector.frame); FreeVector3D(t.to, rotation(t) * vector.v) end
Base.:\(t::Transform3D, point::FreeVector3D) = begin @framecheck point.frame t.to; FreeVector3D(t.from, rotation(t) \ point.v) end
LinearAlgebra.norm(v::FreeVector3D) = norm(v.v)
LinearAlgebra.normalize(v::FreeVector3D, p = 2) = FreeVector3D(v.frame, normalize(v.v, p))

# Mixed Point3D and FreeVector3D
Base.:+(p1::FreeVector3D, p2::FreeVector3D) = begin @framecheck(p1.frame, p2.frame); FreeVector3D(p1.frame, p1.v + p2.v) end
Base.:+(p::Point3D, v::FreeVector3D) = begin @framecheck(p.frame, v.frame); Point3D(p.frame, p.v + v.v) end
Base.:+(v::FreeVector3D, p::Point3D) = p + v
Base.:-(p::Point3D, v::FreeVector3D) = begin @framecheck(p.frame, v.frame); Point3D(p.frame, p.v - v.v) end
LinearAlgebra.cross(p::Point3D, v::FreeVector3D) = begin @framecheck(p.frame, v.frame); FreeVector3D(p.frame, p.v × v.v) end
