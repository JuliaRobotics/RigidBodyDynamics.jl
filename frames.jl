using FixedSizeArrays
using Quaternions
import Base: convert, one, *, +, call, inv, get

function rotate{N, T}(x::Mat{3, N, T}, q::Quaternion{T})
    # TODO: efficiency?
    return Mat(rotationmatrix(q)) * x
end

function rotate{T}(x::Vec{3, T}, q::Quaternion{T})
    # TODO: efficiency?
    return Mat(rotationmatrix(q)) * x
end

immutable CartesianFrame3D
    name::ASCIIString
end

immutable Point3D{T}
    frame::CartesianFrame3D
    v::Vec{3, T}
end

immutable Transform3D{T}
    from::CartesianFrame3D
    to::CartesianFrame3D
    rot::Quaternion{T}
    trans::Vec{3, T}

    Transform3D(from::CartesianFrame3D, to::CartesianFrame3D, rot::Quaternion{T}, trans::Vec{3, T}) = new(from, to, rot, trans)
    Transform3D(from::CartesianFrame3D, to::CartesianFrame3D) = new(from, to, one(Quaternion{T}), zero(Vec{3, T}))
    Transform3D(frame::CartesianFrame3D) = new(frame, frame, one(Quaternion{T}), zero(Vec{3, T}))
end

Transform3D{T}(from::CartesianFrame3D, to::CartesianFrame3D, rot::Quaternion{T}, trans::Vec{3, T}) = Transform3D{T}(from, to, rot, trans)
Transform3D{T}(from::CartesianFrame3D, to::CartesianFrame3D, rot::Quaternion{T}) = Transform3D{T}(from, to, rot, zero(Vec{3, T}))
Transform3D{T}(from::CartesianFrame3D, to::CartesianFrame3D, trans::Vec{3, T}) = Transform3D{T}(from, to, one(Quaternion{T}), trans)

function *(t1::Transform3D, t2::Transform3D)
    @assert t1.from == t2.to
    return Transform3D(t2.from, t1.to, t1.rot * t2.rot, t1.trans + rotate(t2.trans, t1.rot))
end

function *(t::Transform3D, point::Point3D)
    @assert t1.from == point.frame
    return Point3D(t.to, rotate(point.v, t.rot) + t.trans)
end

function inv{T}(t::Transform3D{T})
    rotinv = inv(t.rot)
    transinv = -rotate(t.trans, rotinv)
    return Transform3D(t.to, t.from, rotinv, transinv)
end
