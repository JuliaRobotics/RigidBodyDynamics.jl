# NOTE: The `next_frame_id' and `frame_names' globals below are a hack, but they
# enable a significant reduction in allocations.
# Storing the names of all CartesianFrame3D objects in this frame_names vector instead
# of in the CartesianFrame3D and having CartesianFrame3D only contain an integer ID
# makes CartesianFrame3D an isbits (pointer free) type.
# This in turn makes it so that a lot of the geometry/dynamics types become isbits
# types, making them stack allocated and allowing all sorts of
# optimizations.
const next_frame_id = Ref(0)
const frame_names = Dict{Int64, String}()

"""
$(TYPEDEF)

A `CartesianFrame3D` identifies a three-dimensional Cartesian coordinate system.

`CartesianFrame3D`s are typically used to annotate the frame in which certain
quantities are expressed.
"""
immutable CartesianFrame3D
    id::Int64

    function CartesianFrame3D(name::String)
        ret = new(next_frame_id.x)
        next_frame_id.x = Base.Checked.checked_add(next_frame_id.x, 1)
        frame_names[ret.id] = name
        ret
    end

    function CartesianFrame3D()
        ret = new(next_frame_id.x)
        next_frame_id.x = Base.Checked.checked_add(next_frame_id.x, 1)
        ret
    end
end

name(frame::CartesianFrame3D) = get(frame_names, frame.id, "anonymous")
Base.show(io::IO, frame::CartesianFrame3D) = print(io, "CartesianFrame3D: \"$(name(frame))\" (id = $(frame.id))")

"""
$(SIGNATURES)

Check that `f1` and `f2` are identical (when bounds checks are enabled).

Throws an `ArgumentError` if `f1` is not identical to `f2` when bounds checks
are enabled. `\@framecheck` is a no-op when bounds checks are disabled.
"""
macro framecheck(f1, f2)
    symname1 = string(f1)
    symname2 = string(f2)
    ret = quote
        @boundscheck begin
            if $f1 != $f2
                name1, name2 = name($f1), name($f2)
                id1, id2 = ($f1).id, ($f2).id
                msg = "$($symname1) (\"$name1\", id = $id1) ≠ $($symname2) (\"$name2\", id = $id2)"
                throw(ArgumentError(msg))
            end
        end
    end
    :($(esc(ret)))
end


# Transform between frames
"""
$(TYPEDEF)

A homogeneous transformation matrix representing the transformation from one
three-dimensional Cartesian coordinate system to another.
"""
immutable Transform3D{A<:AbstractMatrix}
    from::CartesianFrame3D
    to::CartesianFrame3D
    mat::A

    function (::Type{Transform3D{A}}){A<:AbstractArray}(from::CartesianFrame3D, to::CartesianFrame3D, mat::A)
        @boundscheck size(mat) == (4, 4) || throw(DimensionMismatch())
        new{A}(from, to, mat)
    end
end

@inline Transform3D{A}(from::CartesianFrame3D, to::CartesianFrame3D, mat::A) = Transform3D{A}(from, to, mat)

Base.eltype{A}(::Type{Transform3D{A}}) = eltype(A)
@compat const Transform3DS{T} = Transform3D{SMatrix{4, 4, T, 16}}

@inline function Transform3D(from::CartesianFrame3D, to::CartesianFrame3D, rot::Rotation{3}, trans::SVector{3})
    T = promote_type(eltype(typeof(rot)), eltype(typeof(trans)))
    @inbounds mat = @SMatrix [rot[1] rot[4] rot[7] trans[1];
                              rot[2] rot[5] rot[8] trans[2];
                              rot[3] rot[6] rot[9] trans[3];
                              zero(T) zero(T) zero(T) one(T)]
   Transform3D(from, to, mat)
end

@inline function Transform3D{T}(from::CartesianFrame3D, to::CartesianFrame3D, rot::Rotation{3, T})
    @inbounds mat = @SMatrix [rot[1] rot[4] rot[7] zero(T);
                              rot[2] rot[5] rot[8] zero(T);
                              rot[3] rot[6] rot[9] zero(T);
                              zero(T) zero(T) zero(T) one(T)]
   Transform3D(from, to, mat)
end

@inline function Transform3D{T}(from::CartesianFrame3D, to::CartesianFrame3D, trans::SVector{3, T})
    @inbounds mat = @SMatrix [one(T) zero(T) zero(T) trans[1];
                              zero(T) one(T) zero(T) trans[2];
                              zero(T) zero(T) one(T) trans[3];
                              zero(T) zero(T) zero(T) one(T)]
    Transform3D(from, to, mat)
end

@inline Base.convert{A}(::Type{Transform3D{A}}, t::Transform3D{A}) = t
@inline Base.convert{A}(::Type{Transform3D{A}}, t::Transform3D) = Transform3D(t.from, t.to, convert(A, t.mat))

@inline rotation(t::Transform3D) = @inbounds return RotMatrix(t.mat[1], t.mat[2], t.mat[3], t.mat[5], t.mat[6], t.mat[7], t.mat[9], t.mat[10], t.mat[11])
@inline translation(t::Transform3D) = @inbounds return SVector(t.mat[13], t.mat[14], t.mat[15])

function Base.show(io::IO, t::Transform3D)
    println(io, "Transform3D from \"$(name(t.from))\" to \"$(name(t.to))\":")
    angleAxis = AngleAxis(rotation(t))
    angle = rotation_angle(angleAxis)
    axis = rotation_axis(angleAxis)
    println(io, "rotation: $(angle) rad about $(axis), translation: $(translation(t))") # TODO: use fixed Quaternions.jl version once it's updated
end

@inline function *(t1::Transform3D, t2::Transform3D)
    @framecheck(t1.from, t2.to)
    mat = t1.mat * t2.mat
    Transform3D(t2.from, t1.to, mat)
end

@inline function Base.inv(t::Transform3D)
    rotinv = inv(rotation(t))
    Transform3D(t.to, t.from, rotinv, -(rotinv * translation(t)))
end

@inline Base.eye{A<:StaticArray}(::Type{Transform3D{A}}, from::CartesianFrame3D, to::CartesianFrame3D) = Transform3D(from, to, eye(A))
@inline function Base.eye{A<:AbstractMatrix}(::Type{Transform3D{A}}, from::CartesianFrame3D, to::CartesianFrame3D)
    T = eltype(A)
    convert(Transform3D{A}, eye(Transform3DS{T}, from, to))
end
@inline Base.eye(::Type{Transform3D}, from::CartesianFrame3D, to::CartesianFrame3D) = eye(Transform3DS{Float64}, from, to)
@inline Base.eye{T<:Transform3D}(::Type{T}, frame::CartesianFrame3D) = eye(T, frame, frame)

function Random.rand{A}(::Type{Transform3D{A}}, from::CartesianFrame3D, to::CartesianFrame3D)
    T = eltype(A)
    rot = rand(RotMatrix3{T})
    trans = rand(SVector{3, T})
    convert(Transform3D{A}, Transform3D(from, to, rot, trans))
end

Random.rand(::Type{Transform3D}, from::CartesianFrame3D, to::CartesianFrame3D) = rand(Transform3DS{Float64}, from, to)

function Base.isapprox(x::Transform3D, y::Transform3D; atol::Real = 1e-12)
    x.from == y.from && x.to == y.to && isapprox(rotation(x), rotation(y), atol = atol) && isapprox(translation(x), translation(y), atol = atol)
end


# Point3D, FreeVector3D. The difference is that a FreeVector3D is only rotated when its frame is changed,
# whereas a Point3D is also translated
for VectorType in (:FreeVector3D, :Point3D)
    @eval begin
        # TODO: consider storing as a homogeneous vector
        immutable $VectorType{V<:AbstractVector}
            frame::CartesianFrame3D
            v::V

            function (::Type{$VectorType{V}}){V}(frame::CartesianFrame3D, v::V)
                @boundscheck length(v) == 3 || throw(DimensionMismatch())
                new{V}(frame, v)
            end
        end

        $VectorType{V}(frame::CartesianFrame3D, v::V) = $VectorType{V}(frame, v)
        $VectorType{T<:Number}(::Type{T}, frame::CartesianFrame3D) = $VectorType(frame, zeros(SVector{3, T}))
        $VectorType(frame::CartesianFrame3D, x::Number, y::Number, z::Number) = $VectorType(frame, SVector(x, y, z))

        Base.convert{V}(::Type{$VectorType{V}}, p::$VectorType{V}) = p
        Base.convert{V}(::Type{$VectorType{V}}, p::$VectorType) = $VectorType(p.frame, convert(V, p.v))

        Base.zero(p::$VectorType) = $VectorType(p.frame, zero(p.v))

        (/){S<:Number}(p::$VectorType, s::S) = $VectorType(p.frame, p.v / s)
        (*){S<:Number}(p::$VectorType, s::S) = $VectorType(p.frame, p.v * s)
        (*){S<:Number}(s::S, p::$VectorType) = $VectorType(p.frame, s * p.v)
        (-)(p::$VectorType) = $VectorType(p.frame, -p.v)

        Random.rand{T}(::Type{$VectorType}, ::Type{T}, frame::CartesianFrame3D) = $VectorType(frame, rand(SVector{3, T}))
        Base.show(io::IO, p::$VectorType) = print(io, "$($(string(VectorType))) in \"$(name(p.frame))\": $(p.v)")
        Base.isapprox(x::$VectorType, y::$VectorType; atol::Real = 1e-12) = x.frame == y.frame && isapprox(x.v, y.v; atol = atol)

        """
        $(SIGNATURES)

        Return `x` transformed to `CartesianFrame3D` `t.from`.
        """
        transform(x::$VectorType, t::Transform3D) = t * x

        """
        $(SIGNATURES)

        Apply the inverse transform, i.e. return `x`, originally expressed in
        CartesianFrame3D `t.from`, transformed to `t.to`.
        """
        invtransform(x::$VectorType, t::Transform3D) = t \ x

        Base.eltype{V}(::Type{$VectorType{V}}) = eltype(V)
        StaticArrays.similar_type{V, T}(x::Type{$VectorType{V}}, ::Type{T}) = $VectorType{SVector{3, T}}
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
(-)(p1::Point3D, p2::Point3D) = begin @framecheck(p1.frame, p2.frame); FreeVector3D(p1.frame, p1.v - p2.v) end
(*)(t::Transform3D, point::Point3D) = begin @framecheck(t.from, point.frame); Point3D(t.to, rotation(t) * point.v + translation(t)) end
(\)(t::Transform3D, point::Point3D) = begin @framecheck point.frame t.to; Point3D(t.from, At_mul_B(rotation(t), point.v - translation(t))) end

# FreeVector3D-specific
FreeVector3D(p::Point3D) = FreeVector3D(p.frame, p.v)
(-)(v1::FreeVector3D, v2::FreeVector3D) = begin @framecheck(v1.frame, v2.frame); FreeVector3D(v1.frame, v1.v - v2.v) end
Base.cross(v1::FreeVector3D, v2::FreeVector3D) = begin @framecheck(v1.frame, v2.frame); FreeVector3D(v1.frame, cross(v1.v, v2.v)) end
Base.dot(v1::FreeVector3D, v2::FreeVector3D) = begin @framecheck(v1.frame, v2.frame); dot(v1.v, v2.v) end
(*)(t::Transform3D, vector::FreeVector3D) = begin @framecheck(t.from, vector.frame); FreeVector3D(t.to, rotation(t) * vector.v) end
(\)(t::Transform3D, point::FreeVector3D) = begin @framecheck point.frame t.to; FreeVector3D(t.from, At_mul_B(rotation(t), point.v)) end
Base.norm(v::FreeVector3D) = norm(v.v)
Base.normalize(v::FreeVector3D, p = 2) = FreeVector3D(v.frame, normalize(v.v, p))

# Mixed Point3D and FreeVector3D
(+)(p1::FreeVector3D, p2::FreeVector3D) = begin @framecheck(p1.frame, p2.frame); FreeVector3D(p1.frame, p1.v + p2.v) end
(+)(p::Point3D, v::FreeVector3D) = begin @framecheck(p.frame, v.frame); Point3D(p.frame, p.v + v.v) end
(+)(v::FreeVector3D, p::Point3D) = p + v
(-)(p::Point3D, v::FreeVector3D) = begin @framecheck(p.frame, v.frame); Point3D(p.frame, p.v - v.v) end
Base.cross(p::Point3D, v::FreeVector3D) = begin @framecheck(p.frame, v.frame); FreeVector3D(p.frame, cross(p.v, v.v)) end
