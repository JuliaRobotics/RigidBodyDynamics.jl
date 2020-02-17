
"""
$(SIGNATURES)

Return a suitable object to do collision checking for the given geometries.
"""
function collision_cache end

mutable struct DefaultCollisionCache{T, C<:EnhancedGJK.CollisionCache}
    gjkcache::C
    halfspace_in_b::Union{HalfSpace{3, T}, Nothing} # fixed in frame of body B; normal points outward from B
end

DefaultCollisionCache{T}(gjkcache::C) where {T, C<:EnhancedGJK.CollisionCache} = DefaultCollisionCache{T, C}(gjkcache, nothing)
DefaultCollisionCache{T}(a, b) where {T} = DefaultCollisionCache{T}(EnhancedGJK.CollisionCache(a, b))

collision_cache(::Type{T}, a, b) where {T} = DefaultCollisionCache{T}(a, b)

function reset!(cache::DefaultCollisionCache)
    EnhancedGJK.reset!(cache.gjkcache)
    cache.halfspace_in_b = nothing
    return cache
end

function extract_halfspace(result::GJKResult, closest_in_b::SVector{3})
    # IMPORTANT: this only works when the closest face of the simplex corresponds
    # to the closest face of the Minkowski sum. Should instead use EPA to get the
    # halfspace and cache it for performance and to avoid tunelling.
    simplex = result.simplex
    best_dist_squared = nothing
    best_face = nothing
    best_point = nothing
    for i in eachindex(simplex)
        face = EnhancedGJK.simplex_face(simplex, i)
        weights = EnhancedGJK.projection_weights(face)
        point = EnhancedGJK.linear_combination(weights, face)
        dist_squared = point ⋅ point
        if best_dist_squared === nothing || dist_squared < best_dist_squared
            best_dist_squared = dist_squared
            best_face = face
            best_point = point
        end
    end
    # @show typeof(best_face)
    # error()
    normal = normalize(EnhancedGJK.normal(best_face))
    signed_distance = best_point ⋅ normal
    if signed_distance > 0
        signed_distance = -signed_distance
        normal = -normal
    end
    HalfSpace(normal, normal ⋅ closest_in_b - signed_distance)
end

@inline function detect_contact(cache::DefaultCollisionCache, pose_a::Transformation, pose_b::Transformation)
    result = gjk!(cache.gjkcache, pose_a, pose_b)
    closest_in_a = pose_a(result.closest_point_in_body.a)
    closest_in_b = pose_b(result.closest_point_in_body.b)
    if result.in_collision
        # NOTE: normal computation currently relies on the fact that the closest
        # face of the simplex corresponds to the closest face of the Minkowski sum.
        # This will not be the case with large penetrations.
        # Caching the penetration halfspace from the previous result is meant to
        # circumvent this.
        if cache.halfspace_in_b === nothing
            halfspace_in_world = extract_halfspace(result, closest_in_b)
            cache.halfspace_in_b = inv(pose_b)(halfspace_in_world)
        else
            halfspace_in_world = pose_b(cache.halfspace_in_b)
        end
        separation = closest_in_b ⋅ halfspace_in_world.outward_normal - halfspace_in_world.offset
        normal = halfspace_in_world.outward_normal
    else
        cache.halfspace_in_b = nothing
        separation = separation_distance(result)
        normal = nothing
    end
    separation, normal, closest_in_a, closest_in_b
end


collision_cache(::Type{T}, a::GeometryTypes.Point{N, T}, b::HalfSpace{N, T}) where {T, N} = a => b
collision_cache(::Type{T}, a::HalfSpace{N, T}, b::GeometryTypes.Point{N, T}) where {T, N} = a => b

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


struct PointToHRepCollisionCache{F, M, T} # F: whether or not the order of point and hrep is flipped
    point::GeometryTypes.Point{3, T}
    hrep::HRep{M, 3, T}
    penetrated_halfspace_index::Base.RefValue{Int} # 0: uninitialized
end

PointToHRepCollisionCache(point::GeometryTypes.Point{3, T}, hrep::HRep{M, 3, T}) where {M, T} = PointToHRepCollisionCache{false, M, T}(point, hrep, Ref(0))
PointToHRepCollisionCache(hrep::HRep{M, 3, T}, point::GeometryTypes.Point{3, T}) where {M, T} = PointToHRepCollisionCache{true, M, T}(point, hrep, Ref(0))

collision_cache(::Type{T}, point::GeometryTypes.Point{3, T}, hrep::HRep{M, 3, T}) where {M, T} = PointToHRepCollisionCache(point, hrep)
collision_cache(::Type{T}, hrep::HRep{M, 3, T}, point::GeometryTypes.Point{3, T}) where {M, T} = PointToHRepCollisionCache(hrep, point)

reset!(cache::PointToHRepCollisionCache) = (cache.penetrated_halfspace_index[] = 0; return nothing)

@inline function detect_contact(cache::PointToHRepCollisionCache{F, M}, pose_a::Transformation, pose_b::Transformation) where {F, M}
    point = SVector(cache.point)
    hrep = cache.hrep
    flip = F

    # Handle flipping of a and b
    point_to_world, hrep_to_world = flip ? (pose_b, pose_a) : (pose_a, pose_b)
    point_world = point_to_world(point)
    point_hrep = inv(hrep_to_world)(point_world)

    # Check all of the half-spaces (in hrep frame)
    in_penetration = true
    closest_index = 0
    T = eltype(point) # TODO
    closest_distance = typemax(T)
    halfspaces = hrep.halfspaces
    point_hrep_homogeneous = SVector(tuple(point_hrep[1], point_hrep[2], point_hrep[3], -one(eltype(point_hrep)))) # TODO: make nicer
    @inbounds for i in 1 : M
        halfspace = halfspaces[i]
        normal = halfspace.outward_normal
        sep = SVector(tuple(normal[1], normal[2], normal[3], halfspace.offset)) ⋅ point_hrep_homogeneous # TODO: make nicer
        in_penetration &= sep < 0
        distance = abs(sep)
        if i === 1 || distance < closest_distance
            closest_index = i
            closest_distance = distance
        end
    end

    if in_penetration
        # Determine which half-space to use
        if cache.penetrated_halfspace_index[] != 0
            closest_index = cache.penetrated_halfspace_index[]
        end
    end

    # Compute normal based on closest plane
    @inbounds halfspace = halfspaces[closest_index]
    rotation_to_world = LinearMap(transform_deriv(hrep_to_world, 0))
    normal_world = rotation_to_world(halfspace.outward_normal)

    # compute separation, closest point, caching
    if in_penetration
        separation = let normal = halfspace.outward_normal
            SVector(tuple(normal[1], normal[2], normal[3], halfspace.offset)) ⋅ point_hrep_homogeneous
        end
        @assert separation < 0
        closest_in_hrep = point_world
        cache.penetrated_halfspace_index[] = closest_index
    else
        separation = closest_distance # kind of bogus...
        closest_in_hrep = fill(NaN, typeof(point_world))
        cache.penetrated_halfspace_index[] = 0
    end

    # Handle flipping of a and b and return
    if flip
        return separation, -normal_world, closest_in_hrep, point_world
    else
        return separation, normal_world, point_world, closest_in_hrep
    end
end
