# Point3D, FreeVector3D. The difference is that a FreeVector3D is only rotated when its frame is changed,
# whereas a Point3D is also translated
for Vector3D in (:FreeVector3D, :Point3D)
    RawVector3D = Symbol("Raw$Vector3D")
    FrameVector3D = Symbol("Frame$Vector3D")
    @eval begin
        # TODO: consider storing as a homogeneous vector
        struct $Vector3D{T, F<:FrameOrNothing}
            frame::F
            v::SVector{3, T} # TODO: rename to vec?

            @inline function $Vector3D{T, F}(frame::F, v::SVector{3, T}) where {T, F<:FrameOrNothing}
                new{T, F}(frame, v)
            end
        end

        const $RawVector3D{T} = $Vector3D{T, Nothing}
        const $FrameVector3D{T} = $Vector3D{T, CartesianFrame3D}

        hasframes(::Type{<:$FrameVector3D}) = true

        # Constructors: from AbstractVector
        @inline function $Vector3D{T, F}(frame::F, v::AbstractVector) where {T, F<:FrameOrNothing}
            $Vector3D{T, F}(frame, SVector{3}(v))
        end

        @inline function $Vector3D{T}(frame::F, v::AbstractVector) where {T, F<:FrameOrNothing}
            $Vector3D{T, F}(frame, v)
        end

        @inline function $Vector3D(frame::F, v::AbstractVector{T}) where {T, F<:FrameOrNothing}
            $Vector3D{T, F}(frame, v)
        end

        # Constructors: from three coordinates
        @inline function (::Type{V})(frame::FrameOrNothing, x, y, z) where {V<:$Vector3D}
            V(frame, SVector{3}(x, y, z))
        end

        # Constructors: from Vector3D
        @inline (::Type{V})(v::V) where {V<:$Vector3D} = v
        @inline $Vector3D{T, F}(v::$Vector3D) where {T, F<:FrameOrNothing} = $Vector3D{T, F}(v.frame, v.v)
        @inline $Vector3D{T}(v::$Vector3D) where {T} = $Vector3D{T}(v.frame, v.v)

        # Constructors: returning RawVector3D
        @inline (::Type{V})(v::AbstractVector) where {V<:$Vector3D} = V(nothing, v)
        @inline (::Type{V})(x, y, z) where {V<:$Vector3D} = V(nothing, x, y, z)

        # Constructors: RawVector3D to FrameVector3D
        @inline $Vector3D(frame::CartesianFrame3D, v::$RawVector3D) = $Vector3D(frame, v.v)

        # TODO: deprecate:
        $Vector3D(::Type{T}, frame::CartesianFrame3D) where {T} = $Vector3D(frame, zero(SVector{3, T}))

        # Conversion
        @inline Base.convert(::Type{V}, v::$Vector3D) where {V<:$Vector3D} = V(v)

        # Getters
        @inline Base.vec(v::$Vector3D) = v.v
        @propagate_inbounds Base.getindex(v::$Vector3D, i::Integer) = getindex(v.v, i)
        Base.eltype(::Type{$Vector3D{T}}) where {T} = T

        # Pretty-printing
        function Base.show(io::IO, v::$Vector3D)
            print(io, "$($(string(Vector3D)))")
            if hasframes(v)
                print(io, " in \"$(string(v.frame))\"")
            end
            print(io, ": $(v.v)")
        end

        # Operations
        Base.:/(v::$Vector3D, s::Number) = $Vector3D(v.frame, v.v / s)
        Base.:*(v::$Vector3D, s::Number) = $Vector3D(v.frame, v.v * s)
        Base.:*(s::Number, v::$Vector3D) = $Vector3D(v.frame, s * v.v)
        Base.:-(v::$Vector3D) = $Vector3D(v.frame, -v.v)
        Base.:+(v::$Vector3D) = v

        function Random.rand(::Type{V}, frame::F) where {T, V<:$Vector3D{T}, F<:FrameOrNothing}
            $Vector3D(frame, rand(SVector{3, T}))
        end

        function Random.rand(::Type{$Vector3D}, frame::F) where F<:FrameOrNothing
            rand($Vector3D{Float64}, frame)
        end

        Random.rand(::Type{V}) where {V<:$Vector3D} = rand(V, nothing)

        # TODO: deprecate:
        Random.rand(::Type{$Vector3D}, ::Type{T}, frame::CartesianFrame3D) where {T} = $Vector3D(frame, rand(SVector{3, T}))

        # TODO: reltol, change default atol
        Base.isapprox(x::$Vector3D, y::$Vector3D; atol::Real = 1e-12) = x.frame == y.frame && isapprox(x.v, y.v; atol = atol)

        """
        $(SIGNATURES)

        Return `x` transformed to `CartesianFrame3D` `t.from`.
        """
        transform(x::$Vector3D, t::Transform3D) = t * x

        LinearAlgebra.norm(v::$Vector3D) = norm(v.v)
        LinearAlgebra.normalize(v::$Vector3D, p = 2) = $Vector3D(v.frame, normalize(v.v, p))
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
