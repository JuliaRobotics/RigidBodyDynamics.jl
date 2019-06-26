
"""
$(SIGNATURES)

Return a suitable object to do collision checking for the given geometries.
By default this returns an `EnhancedGJK.CollisionCache`.
"""
function collision_cache end

collision_cache(a, b) = CollisionCache(a, b)
reset!(cache::CollisionCache) = EnhancedGJK.reset!(cache)

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

@inline function detect_contact(cache::CollisionCache, pose_a::Transformation, pose_b::Transformation)
    result = gjk!(cache, pose_a, pose_b)
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
    closest_in_a = pose_a(result.closest_point_in_body.a)
    closest_in_b = pose_b(result.closest_point_in_body.b)
    separation, normal, closest_in_a, closest_in_b
end

struct HalfSpace{N, T}
    outward_normal::SVector{N, T}
    offset::T

    function HalfSpace(outward_normal::SVector{N, T}, offset::T) where {N, T}
        new{N, T}(normalize(outward_normal), offset)
    end
end

function HalfSpace(outward_normal::StaticVector{N}, offset) where {N}
    T = promote_type(typeof(offset), eltype(outward_normal))
    HalfSpace(SVector{N, T}(outward_normal), T(offset))
end

function HalfSpace(outward_normal::StaticVector{N}, point::StaticVector{N}) where N
    HalfSpace(outward_normal, point ⋅ outward_normal)
end

function (tf::AffineMap)(halfspace::HalfSpace)
    linear = LinearMap(transform_deriv(tf, 0))
    normal = linear(halfspace.outward_normal)
    offset = halfspace.offset + normal ⋅ tf.translation
    HalfSpace(normal, offset)
end

function (tf::Translation)(halfspace::HalfSpace)
    normal = halfspace.outward_normal
    offset = halfspace.offset + normal ⋅ tf.translation
    HalfSpace(normal, offset)
end

collision_cache(a::GeometryTypes.Point{N}, b::HalfSpace{N}) where {N} = a => b
collision_cache(a::HalfSpace{N}, b::GeometryTypes.Point{N}) where {N} = a => b

reset!(::Pair{<:GeometryTypes.Point, <:HalfSpace}) = nothing
reset!(::Pair{<:HalfSpace, <:GeometryTypes.Point}) = nothing

@inline function detect_contact(geometries::Pair{<:GeometryTypes.Point, <:HalfSpace}, pose_a::Transformation, pose_b::Transformation)
    point = pose_a(geometries[1])
    halfspace = pose_b(geometries[2])
    separation = point ⋅ halfspace.outward_normal - halfspace.offset
    closest_in_halfspace = separation <= 0 ? point : point - halfspace.offset * halfspace.outward_normal
    separation, halfspace.outward_normal, point, closest_in_halfspace
end

@inline function detect_contact(geometries::Pair{<:HalfSpace, <:GeometryTypes.Point}, pose_a::Transformation, pose_b::Transformation)
    separation, inward_normal, point_in_b, point_in_a = detect_contact(Base.reverse(geometries), pose_b, pose_a)
    separation, -inward_normal, point_in_a, point_in_b
end
