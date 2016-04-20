immutable Twist{T}
    # describes motion of body w.r.t. base, expressed in frame
    body::CartesianFrame3D
    base::CartesianFrame3D
    frame::CartesianFrame3D
    angular::Vec{3, T}
    linear::Vec{3, T}
end

function (+){T}(t1::Twist{T}, t2::Twist{T})
    @assert t1.body == t2.base
    @assert t1.frame == t2.frame
    return Twist(t2.body, t1.base, frame, t1.angular + t2.angular, t1.linear + t2.linear)
end

function (-){T}(t::Twist{T})
    return Twist(t.base, t.body, t.frame, -angular, -linear)
end

function transform{T}(m::Twist{T}, transform::Transform3D{T})
    @assert m.frame == transform.from
    R = rotationmatrix(transform.rot)
    angular = R * m.angular
    linear = R * m.linear + cross(transform.trans, angular)
    return Twist(m.body, m.base, transform.to, angular, linear)
end

# immutable MotionSubspaceBasis{N, T}
#     body::RigidBody
#     base::RigidBody
#     frame::CartesianFrame3D
#     angular::Mat{3, N, T}
#     linear::Mat{3, N, T}
# end
#
# function transform{N, T}(m::MotionSubspaceBasis{N, T}, t::Transform3D{T})
#     @assert m.frame == t.from
#     R = rotationmatrix(t.rot)
#     angular = R * m.angular
#     linear = R * m.linear + broadcast(cross, t.trans, angular)
#     return MotionSubspaceBasis(m.body, m.base, t.to, angular, linear)
# end

# immutable SpatialForceMatrix{N, T}
#     frame::CartesianFrame3D
#     angular::Mat{3, N, T}
#     linear::Mat{3, N, T}
#     body::RigidBody
#     base::RigidBody
# end
# typealias SpatialForceVector{T} SpatialForceMatrix{1, T}
#
# function transform{N, T}(f::SpatialForceMatrix{N, T}, t::Transform3D{T})
#     @assert f.frame == t.from
#     R = rotationmatrix(t.rot)
#     linear = R * f.linear
#     angular = R * f.angular + broadcast(cross, t.trans, linear)
#     return SpatialForceMatrix(t.to, angular, linear, f.body, f.base)
# end
