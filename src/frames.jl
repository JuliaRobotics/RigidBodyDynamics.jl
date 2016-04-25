immutable CartesianFrame3D
    name::ASCIIString
end
show(io::IO, frame::CartesianFrame3D) = print(io, "CartesianFrame3D: \"$(frame.name)\"")

immutable Point3D{T<:Real}
    frame::CartesianFrame3D
    v::Vec{3, T}

    Point3D(frame::CartesianFrame3D, v::Vec{3, T}) = new(frame, v)
    Point3D(frame::CartesianFrame3D) = new(frame, zero(Vec{3, T}))
end
Point3D{T}(frame::CartesianFrame3D, v::Vec{3, T}) = Point3D{T}(frame, v)
(+)(p1::Point3D, p2::Point3D) = begin @assert p1.frame == p2.frame; return Point3D(p1.frame, p1.v + p2.v) end
(/)(p::Point3D, s::Real) = Point3D(p.frame, p.v / s)
(*)(p::Point3D, s::Real) = Point3D(p.frame, p.v * s)
(*)(s::Real, p::Point3D) = Point3D(p.frame, s * p.v)
rand{T}(::Type{Point3D{T}}, frame::CartesianFrame3D) = Point3D(frame, rand(Vec{3, T}))
show(io::IO, p::Point3D) = print(io, "Point3D in \"$(p.frame.name)\": $(p.v)")
isapprox{T}(x::Point3D{T}, y::Point3D{T}; atol::Real = 1e-12) = x.frame == y.frame && isapprox_tol(x.v, y.v; atol = atol)

immutable FreeVector3D{T<:Real}
    frame::CartesianFrame3D
    v::Vec{3, T}

    FreeVector3D(frame::CartesianFrame3D, v::Vec{3, T}) = new(frame, v)
    FreeVector3D(frame::CartesianFrame3D) = new(frame, zero(Vec{3, T}))
end
FreeVector3D{T}(frame::CartesianFrame3D, v::Vec{3, T}) = FreeVector3D{T}(frame, v)
(+)(v1::FreeVector3D, v2::FreeVector3D) = begin @assert v1.frame == v2.frame; return FreeVector3D(v1.frame, v1.v + v2.v) end
(-)(v1::FreeVector3D, v2::FreeVector3D) = begin @assert v1.frame == v2.frame; return FreeVector3D(v1.frame, v1.v - v2.v) end
(/)(v::FreeVector3D, s::Real) = FreeVector3D(v.frame, v.v / s)
(*)(v::FreeVector3D, s::Real) = FreeVector3D(v.frame, v.v * s)
(*)(s::Real, v::FreeVector3D) = FreeVector3D(v.frame, s * v.v)
rand{T}(::Type{FreeVector3D{T}}, frame::CartesianFrame3D) = FreeVector3D(frame, rand(Vec{3, T}))
show(io::IO, v::FreeVector3D) = print(io, "FreeVector3D in \"$(v.frame.name)\": $(v.v)")
isapprox{T}(x::FreeVector3D{T}, y::FreeVector3D{T}; atol::Real = 1e-12) = x.frame == y.frame && isapprox_tol(x.v, y.v; atol = atol)

# Mixed Point and FreeVector
(+)(p1::Point3D, v2::FreeVector3D) = begin @assert p1.frame == v2.frame; return Point3D(p1.frame, p1.v + v2.v) end
(+)(v1::FreeVector3D, p2::Point3D) = p2 + v1
(-)(p1::Point3D, v2::FreeVector3D) = begin @assert p1.frame == v2.frame; return Point3D(p1.frame, p1.v - v2.v) end
(-)(p1::FreeVector3D, p2::Point3D) = begin @assert p1.frame == p2.frame; return FreeVector3D(p1.frame, p1.v - p2.v) end

immutable Transform3D{T<:Real}
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
Transform3D{T}(::Type{T}, frame::CartesianFrame3D) = Transform3D{T}(frame, frame, one(Quaternion{T}), zero(Vec{3, T}))
function show(io::IO, t::Transform3D)
    println(io, "Transform3D from \"$(t.from.name)\" to \"$(t.to.name)\":")
    angle, axis = angleaxis(t.rot)
    println(io, "rotation: $(2 * angle) rad about $(axis), translation: $(t.trans)") # TODO: 2 * angle because Quaternions.jl is wrong... need to notify
end

function *(t1::Transform3D, t2::Transform3D)
    @assert t1.from == t2.to
    return Transform3D(t2.from, t1.to, t1.rot * t2.rot, t1.trans + rotate(t2.trans, t1.rot))
end

function inv{T}(t::Transform3D{T})
    rotinv = inv(t.rot)
    return Transform3D(t.to, t.from, rotinv, -rotate(t.trans, rotinv))
end

rand(::Type{Transform3D{Float64}}, from::CartesianFrame3D, to::CartesianFrame3D) = Transform3D(from, to, nquatrand(), rand(Vec{3, Float64}))

function isapprox{T}(x::Transform3D{T}, y::Transform3D{T}; atol::Real = 1e-12)
    theta = 2 * angle(x.rot / y.rot)
    return x.from == y.from && x.to == y.to && isapprox(theta, zero(T), atol = atol) && isapprox_tol(x.trans, y.trans, atol = atol)
end

function *{T}(t::Transform3D{T}, point::Point3D{T})
    @assert t.from == point.frame
    return Point3D(t.to, rotate(point.v, t.rot) + t.trans)
end

function *{T}(t::Transform3D{T}, vector::FreeVector3D{T})
    @assert t.from == vector.frame
    return FreeVector3D(t.to, rotate(vector.v, t.rot))
end
