mutable struct HalfSpace3D{T}
    point::Point3D{SVector{3, T}}
    outward_normal::FreeVector3D{SVector{3, T}}

    function HalfSpace3D(point::Point3D{SVector{3, T}}, outward_normal::FreeVector3D{SVector{3, T}}) where {T}
        @framecheck point.frame outward_normal.frame
        new{T}(point, normalize(outward_normal))
    end
end

function HalfSpace3D(point::Point3D, outward_normal::FreeVector3D)
    T = promote_type(eltype(point), eltype(outward_normal))
    HalfSpace3D(convert(Point3D{SVector{3, T}}, point), convert(FreeVector3D{SVector{3, T}}, outward_normal))
end

Base.eltype(::Type{HalfSpace3D{T}}) where {T} = T
separation(halfspace::HalfSpace3D, p::Point3D) = dot(p - halfspace.point, halfspace.outward_normal)
point_inside(halfspace::HalfSpace3D, p::Point3D) = separation(halfspace, p) <= 0
detect_contact(halfspace::HalfSpace3D, p::Point3D) = separation(halfspace, p), halfspace.outward_normal
