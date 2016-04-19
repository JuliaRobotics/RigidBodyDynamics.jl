immutable SpatialMotionMatrix{N, T}
    body::RigidBody
    base::RigidBody
    frame::CartesianFrame3D
    angular::Mat{3, N, T}
    linear::Mat{3, N, T}
end
typealias SpatialMotionVector{T} SpatialMotionMatrix{1, T}

function transform{N, T}(m::SpatialMotionMatrix{N, T}, t::Transform3D{T})
    @assert m.frame == t.from
    angular = rotate(m.angular, t.rot)
    linear = rotate(m.linear, t.rot) + broadcast(cross, t.trans, angular(:, i))
    return SpatialMotionMatrix(m.body, m.base, t.to, angular, linear)
end

immutable SpatialForceMatrix{N, T}
    body::RigidBody
    base::RigidBody
    frame::CartesianFrame3D
    angular::Mat{3, N, T}
    linear::Mat{3, N, T}
end
typealias SpatialForceVector{T} SpatialForceMatrix{1, T}

function transform{N, T}(f::SpatialForceMatrix{N, T}, t::Transform3D{T})
    @assert f.frame == t.from
    linear = rotate(f.linear, t.rot)
    angular = rotate(f.angular, t.rot) + broadcast(cross, t.trans, linear)
    return SpatialForceMatrix(f.body, f.base, t.to, angular, linear)
end
