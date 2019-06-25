struct CollidablePair{A<:CollisionElement, B<:CollisionElement, M<:ContactForceModel}
    a::A
    b::B
    model::M
end

# struct GJKCollisionCache{G<:EnhancedGJK.CollisionCache}
#     gjk_cache::G
#     penetrated_face::TODO
# end
# PointGJKCollisionCache(gjk_cache::EnhancedGJK.CollisionCache) = PointGJKCollisionCache(gjk_cache, Ref(zero(UInt8)))
# function PointGJKCollisionCache(pair::CollidablePair)
#     @assert pair.a.geometry isa GeometryTypes.Point || pair.b.geometry isa GeometryTypes.Point
#     PointGJKCollisionCache(CollisionCache(pair.a.geometry, pair.b.geometry))
# end

"""
    $(SIGNATURES)

Return a suitable object to do collision checking for a `CollidablePair`.

By default, this returns an `EnhancedGJK.CollisionCache`.
"""
function collision_cache end

collision_cache(pair::CollidablePair) = CollisionCache(pair.a.geometry, pair.b.geometry)

# TODO: move this somewhere else
AffineMap(tf::Transform3D) = AffineMap(rotation(tf), translation(tf))

function extract_normal(result::GJKResult)
    # IMPORTANT: this only works when the closest face of the simplex corresponds
    # to the closest face of the Minkowski sum
    simplex = result.simplex
    closest_face_index = nothing
    closest_distsq = nothing
    @inbounds begin
        for i in eachindex(result.simplex)
            face = EnhancedGJK.simplex_face(simplex, i)
            weights = EnhancedGJK.projection_weights(face)
            closest_point = EnhancedGJK.linear_combination(weights, face)
            distsq = dot(closest_point, closest_point)
            if closest_face_index === nothing || distsq < closest_distsq
                closest_face_index = i
                closest_distsq = distsq
            end
        end
        closest_face = EnhancedGJK.simplex_face(simplex, closest_face_index)
        v1 = closest_face[2] - closest_face[1]
        v2 = closest_face[3] - closest_face[1]
        return normalize(v1 × v2)
    end
end

function detect_contact(cache::CollisionCache, pose_a::Transform3D, pose_b::Transform3D)
    result = gjk!(cache, AffineMap(pose_a), AffineMap(pose_b))
    separation = result.signed_distance
    normal = extract_normal(result)
    closest_in_a = result.closest_point_in_body.a
    closest_in_b = result.closest_point_in_body.b
    separation, normal, closest_in_a, closest_in_b
end