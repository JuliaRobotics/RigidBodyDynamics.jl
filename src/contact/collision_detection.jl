# TODO: move this somewhere else
AffineMap(tf::Transform3D) = AffineMap(rotation(tf), translation(tf))

function extract_normal(result::GJKResult)
    # IMPORTANT: this only works when the closest face of the simplex corresponds
    # to the closest face of the Minkowski sum
    @inbounds begin
        simplex = result.simplex
        closest_face = nothing
        closest_point = nothing
        closest_distsq = nothing
        for i in eachindex(result.simplex)
            face = EnhancedGJK.simplex_face(simplex, i)
            weights = EnhancedGJK.projection_weights(face)
            closest_point_on_face = EnhancedGJK.linear_combination(weights, face)
            distsq = dot(closest_point_on_face, closest_point_on_face)
            if closest_face === nothing || distsq < closest_distsq
                closest_face = face
                closest_point = closest_point_on_face
                closest_distsq = distsq
            end
        end
        v1 = closest_face[2] - closest_face[1]
        v2 = closest_face[3] - closest_face[1]
        normal = normalize(v1 × v2)
        return dot(closest_point, normal) < 0 ? normal : -normal
    end
end

@inline function detect_contact(cache::CollisionCache, pose_a::Transform3D, pose_b::Transform3D)
    result = gjk!(cache, AffineMap(pose_a), AffineMap(pose_b))
    separation = result.signed_distance
    if separation < 0
        # FIXME: normal computation currently relies on the fact that the closest
        # face of the simplex corresponds to the closest face of the Minkowski sum.
        # This will not be the case with large penetrations.
        # Should cache previous closest face to be accurate with large penetrations.
        # For now, check that penetration isn't too big.
        @assert separation > -5e3
        normal = extract_normal(result)
    else
        normal = nothing
    end
    closest_in_a = result.closest_point_in_body.a
    closest_in_b = result.closest_point_in_body.b
    separation, normal, closest_in_a, closest_in_b
end

# mutable struct HalfSpace3D{T}
#     point::Point3D{SVector{3, T}}
#     outward_normal::FreeVector3D{SVector{3, T}}

#     function HalfSpace3D(point::Point3D{SVector{3, T}}, outward_normal::FreeVector3D{SVector{3, T}}) where {T}
#         @framecheck point.frame outward_normal.frame
#         new{T}(point, normalize(outward_normal))
#     end
# end

# function HalfSpace3D(point::Point3D, outward_normal::FreeVector3D)
#     T = promote_type(eltype(point), eltype(outward_normal))
#     HalfSpace3D(convert(Point3D{SVector{3, T}}, point), convert(FreeVector3D{SVector{3, T}}, outward_normal))
# end

# # Base.eltype(::Type{HalfSpace3D{T}}) where {T} = T
# separation(halfspace::HalfSpace3D, p::Point3D) = dot(p - halfspace.point, halfspace.outward_normal)
# # point_inside(halfspace::HalfSpace3D, p::Point3D) = separation(halfspace, p) <= 0
# detect_contact(halfspace::HalfSpace3D, p::Point3D) = separation(halfspace, p), halfspace.outward_normal
