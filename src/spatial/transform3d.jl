"""
$(TYPEDEF)

A homogeneous transformation matrix representing the transformation from one
three-dimensional Cartesian coordinate system to another.
"""
struct Transform3D{T}
    mat::SMatrix{4, 4, T, 16}
    from::CartesianFrame3D
    to::CartesianFrame3D

    @inline function Transform3D(from::CartesianFrame3D, to::CartesianFrame3D, mat::AbstractMatrix{T}) where T
        new{T}(mat, from, to)
    end
end

Base.eltype(::Type{Transform3D{T}}) where {T} = T

@inline function Transform3D(from::CartesianFrame3D, to::CartesianFrame3D, rot::Rotation{3}, trans::SVector{3})
    T = promote_type(eltype(typeof(rot)), eltype(typeof(trans)))
    R = convert(RotMatrix3{T}, rot)
    @inbounds mat = @SMatrix [R[1] R[4] R[7] trans[1];
                              R[2] R[5] R[8] trans[2];
                              R[3] R[6] R[9] trans[3];
                              zero(T) zero(T) zero(T) one(T)]
   Transform3D(from, to, mat)
end

@inline function Transform3D(from::CartesianFrame3D, to::CartesianFrame3D, rot::Rotation{3, T}) where {T}
    R = convert(RotMatrix3{T}, rot)
    @inbounds mat = @SMatrix [R[1] R[4] R[7] zero(T);
                              R[2] R[5] R[8] zero(T);
                              R[3] R[6] R[9] zero(T);
                              zero(T) zero(T) zero(T) one(T)]
   Transform3D(from, to, mat)
end

@inline function Transform3D(from::CartesianFrame3D, to::CartesianFrame3D, trans::SVector{3, T}) where {T}
    @inbounds mat = @SMatrix [one(T) zero(T) zero(T) trans[1];
                              zero(T) one(T) zero(T) trans[2];
                              zero(T) zero(T) one(T) trans[3];
                              zero(T) zero(T) zero(T) one(T)]
    Transform3D(from, to, mat)
end

@inline Base.convert(::Type{Transform3D{T}}, t::Transform3D{T}) where {T} = t
@inline Base.convert(::Type{Transform3D{T}}, t::Transform3D) where {T} = Transform3D(t.from, t.to, similar_type(t.mat, T)(t.mat))

@inline rotation(t::Transform3D) = @inbounds return RotMatrix(t.mat[1], t.mat[2], t.mat[3], t.mat[5], t.mat[6], t.mat[7], t.mat[9], t.mat[10], t.mat[11])
@inline translation(t::Transform3D) = @inbounds return SVector(t.mat[13], t.mat[14], t.mat[15])

function Base.show(io::IO, t::Transform3D)
    println(io, "Transform3D from \"$(string(t.from))\" to \"$(string(t.to))\":")
    angle_axis = AngleAxis(rotation(t))
    angle = rotation_angle(angle_axis)
    axis = rotation_axis(angle_axis)
    print(io, "rotation: $(angle) rad about $(axis), translation: $(translation(t))") # TODO: use fixed Quaternions.jl version once it's updated
end

@inline function Base.:*(t1::Transform3D, t2::Transform3D)
    @framecheck(t1.from, t2.to)
    mat = t1.mat * t2.mat
    Transform3D(t2.from, t1.to, mat)
end

@inline function LinearAlgebra.inv(t::Transform3D)
    rotinv = inv(rotation(t))
    Transform3D(t.to, t.from, rotinv, -(rotinv * translation(t)))
end

Base.one(::Type{Transform3D{T}}, from::CartesianFrame3D, to::CartesianFrame3D) where {T} =
    Transform3D(from, to, one(SMatrix{4, 4, T}))
Base.one(::Type{Transform3D}, from::CartesianFrame3D, to::CartesianFrame3D) = one(Transform3D{Float64}, from, to)
Base.one(::Type{T}, frame::CartesianFrame3D) where {T<:Transform3D} = one(T, frame, frame)

function Random.rand(::Type{Transform3D{T}}, from::CartesianFrame3D, to::CartesianFrame3D) where T
    rot = rand(RotMatrix3{T})
    trans = rand(SVector{3, T})
    Transform3D(from, to, rot, trans)
end

Random.rand(::Type{Transform3D}, from::CartesianFrame3D, to::CartesianFrame3D) = rand(Transform3D{Float64}, from, to)

function Base.isapprox(x::Transform3D, y::Transform3D; atol::Real = 1e-12)
    x.from == y.from && x.to == y.to && isapprox(rotation(x), rotation(y), atol = atol) && isapprox(translation(x), translation(y), atol = atol)
end
