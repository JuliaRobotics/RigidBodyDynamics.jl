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
