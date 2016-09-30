# NOTE: The `next_frame_id' and `frame_names' globals below are a hack, but they
# significantly reduce allocation.
# Storing the names of all CartesianFrame3D objects in this frame_names vector instead
# of in the CartesianFrame3D and having CartesianFrame3D only contain an integer ID
# makes CartesianFram3D an isbits type.
# This in turn makes it so that a lot of the geometry/dynamics types become isbits
# (pointer free) types, making them stack allocated and allowing all sorts of
# optimizations.
const next_frame_id = Ref(0)
const frame_names = Dict{Int64, String}()

immutable CartesianFrame3D
    id::Int64
    function CartesianFrame3D(name::String)
        ret = new(next_frame_id.x)
        next_frame_id.x += 1
        frame_names[ret.id] = name
        ret
    end
end
name(frame::CartesianFrame3D) = frame_names[frame.id]
show(io::IO, frame::CartesianFrame3D) = print(io, "CartesianFrame3D: \"$(name(frame))\"")

# enable/disable frame checks
if isdefined(Main, :RIGID_BODY_DYNAMICS_RELEASE)
    framecheck(f1::CartesianFrame3D, f2::CartesianFrame3D) = nothing
else
    framecheck(f1::CartesianFrame3D, f2::CartesianFrame3D) = f1 != f2 && throw(ArgumentError(("$f1 doesn't match $f2")))
end


for VectorType in (:FreeVector3D, :Point3D)
    @eval begin
        type $VectorType{V <:AbstractVector}
            frame::CartesianFrame3D
            v::V

            $VectorType(frame::CartesianFrame3D, v::V) = begin @boundscheck length(v) == 3; new(frame, v) end
        end

        $VectorType{V}(frame::CartesianFrame3D, v::V) = $VectorType{V}(frame, v)
        $VectorType{T<:Real}(::Type{T}, frame::CartesianFrame3D) = $VectorType(frame, zeros(SVector{3, T}))

        convert{V}(::Type{$VectorType{V}}, p::$VectorType{V}) = p
        convert{V}(::Type{$VectorType{V}}, p::$VectorType) = $VectorType(p.frame, convert(V, p.v))

        (/){S<:Real}(p::$VectorType, s::S) = $VectorType(p.frame, p.v / s)
        (*){S<:Real}(p::$VectorType, s::S) = $VectorType(p.frame, p.v * s)
        (*){S<:Real}(s::S, p::$VectorType) = $VectorType(p.frame, s * p.v)

        rand{T}(::Type{$VectorType}, ::Type{T}, frame::CartesianFrame3D) = $VectorType(frame, rand(SVector{3, T}))
        show(io::IO, p::$VectorType) = print(io, "$($(VectorType).name.name) in \"$(name(p.frame))\": $(p.v)")
        isapprox(x::$VectorType, y::$VectorType; atol::Real = 1e-12) = x.frame == y.frame && isapprox(x.v, y.v; atol = atol)
    end
end

# Point specific
(-)(p1::Point3D, p2::Point3D) = begin framecheck(p1.frame, p2.frame); FreeVector3D(p1.frame, p1.v - p2.v) end

# FreeVector specific
FreeVector3D(p::Point3D) = FreeVector3D(p.frame, p.v)
cross(v1::FreeVector3D, v2::FreeVector3D) = begin framecheck(v1.frame, v2.frame); FreeVector3D(v1.frame, cross(v1.v, v2.v)) end
dot(v1::FreeVector3D, v2::FreeVector3D) = begin framecheck(v1.frame, v2.frame); dot(v1.v, v2.v) end

# Mixed Point and FreeVector
(+)(p1::FreeVector3D, p2::FreeVector3D) = begin framecheck(p1.frame, p2.frame); FreeVector3D(p1.frame, p1.v + p2.v) end
(+)(p::Point3D, v::FreeVector3D) = begin framecheck(p.frame, v.frame); Point3D(p.frame, p.v + v.v) end
(+)(v::FreeVector3D, p::Point3D) = p + v
(-)(p::Point3D, v::FreeVector3D) = begin framecheck(p.frame, v.frame); Point3D(p.frame, p.v - v.v) end
cross(p::Point3D, v::FreeVector3D) = begin framecheck(p.frame, v.frame); FreeVector3D(p.frame, cross(p.v, v.v)) end

immutable Transform3D{T<:Real}
    from::CartesianFrame3D
    to::CartesianFrame3D
    rot::Quaternion{T}
    trans::SVector{3, T}

    Transform3D(from::CartesianFrame3D, to::CartesianFrame3D, rot::Quaternion{T}, trans::SVector{3, T}) = new(from, to, rot, trans)
    Transform3D(from::CartesianFrame3D, to::CartesianFrame3D) = new(from, to, one(Quaternion{T}), zeros(SVector{3, T}))
    Transform3D(frame::CartesianFrame3D) = new(frame, frame, one(Quaternion{T}), zeros(SVector{3, T}))
end
Transform3D{T}(from::CartesianFrame3D, to::CartesianFrame3D, rot::Quaternion{T}, trans::SVector{3, T}) = Transform3D{T}(from, to, rot, trans)
Transform3D{T}(from::CartesianFrame3D, to::CartesianFrame3D, rot::Quaternion{T}) = Transform3D{T}(from, to, rot, zeros(SVector{3, T}))
Transform3D{T}(from::CartesianFrame3D, to::CartesianFrame3D, trans::SVector{3, T}) = Transform3D{T}(from, to, one(Quaternion{T}), trans)
Transform3D{T}(::Type{T}, from::CartesianFrame3D, to::CartesianFrame3D) = Transform3D{T}(from, to, one(Quaternion{T}), zeros(SVector{3, T}))
Transform3D{T}(::Type{T}, frame::CartesianFrame3D) = Transform3D{T}(frame, frame, one(Quaternion{T}), zeros(SVector{3, T}))

convert{T}(::Type{Transform3D{T}}, t::Transform3D{T}) = t
convert{T}(::Type{Transform3D{T}}, t::Transform3D) = Transform3D(t.from, t.to, convert(Quaternion{T}, t.rot), convert(SVector{3, T}, t.trans))

function show(io::IO, t::Transform3D)
    println(io, "Transform3D from \"$(name(t.from))\" to \"$(name(t.to))\":")
    angle, axis = angle_axis_proper(t.rot)
    println(io, "rotation: $(angle) rad about $(axis), translation: $(t.trans)") # TODO: use fixed Quaternions.jl version once it's updated
end

function *(t1::Transform3D, t2::Transform3D)
    framecheck(t1.from, t2.to)
    return Transform3D(t2.from, t1.to, t1.rot * t2.rot, t1.trans + rotate(t2.trans, t1.rot))
end

function inv{T}(t::Transform3D{T})
    rotinv = inv(t.rot)
    return Transform3D{T}(t.to, t.from, rotinv, -rotate(t.trans, rotinv))
end

rand(::Type{Transform3D{Float64}}, from::CartesianFrame3D, to::CartesianFrame3D) = Transform3D(from, to, nquatrand(), rand(SVector{3, Float64}))

function isapprox{T}(x::Transform3D{T}, y::Transform3D{T}; atol::Real = 1e-12)
    theta = 2 * angle(x.rot / y.rot)
    return x.from == y.from && x.to == y.to && isapprox(theta, zero(T), atol = atol) && isapprox(x.trans, y.trans, atol = atol)
end

function *(t::Transform3D, point::Point3D)
    framecheck(t.from, point.frame)
    return Point3D(t.to, rotate(point.v, t.rot) + t.trans)
end

function *(t::Transform3D, vector::FreeVector3D)
    framecheck(t.from, vector.frame)
    return FreeVector3D(t.to, rotate(vector.v, t.rot))
end
