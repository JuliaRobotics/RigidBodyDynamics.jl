immutable Twist{T}
    # describes motion of body w.r.t. base, expressed in frame
    body::CartesianFrame3D
    base::CartesianFrame3D
    frame::CartesianFrame3D
    angular::Vec{3, T}
    linear::Vec{3, T}
end

function (+){T}(twist1::Twist{T}, twist2::Twist{T})
    @assert twist1.body == twist2.base
    @assert twist1.frame == twist2.frame
    return Twist(twist2.body, twist1.base, twist1.frame, twist1.angular + twist2.angular, twist1.linear + twist2.linear)
end

function (-){T}(t::Twist{T})
    return Twist(t.base, t.body, t.frame, -t.angular, -t.linear)
end

function transform{T}(m::Twist{T}, transform::Transform3D{T})
    @assert m.frame == transform.from
    R = Mat(rotationmatrix(transform.rot))
    angular = R * m.angular
    linear = R * m.linear + cross(transform.trans, angular)
    return Twist(m.body, m.base, transform.to, angular, linear)
end

change_base_no_relative_motion(t::Twist, base::CartesianFrame3D) = Twist(t.body, base, t.frame, t.angular, t.linear)
change_body_no_relative_motion(t::Twist, body::CartesianFrame3D) = Twist(body, t.base, t.frame, t.angular, t.linear)
zero{T}(::Type{Twist{T}}, body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D) = Twist(body, base, frame, zero(Vec{3, T}), zero(Vec{3, T}))
rand{T}(::Type{Twist{T}}, body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D) = Twist(body, base, frame, rand(Vec{3, T}), rand(Vec{3, T}))

immutable GeometricJacobian{T}
    body::CartesianFrame3D
    base::CartesianFrame3D
    frame::CartesianFrame3D
    mat::Array{T}
end
angularPart(jac::GeometricJacobian) = jac.mat[1 : 3, :]
linearPart(jac::GeometricJacobian) = jac.mat[4 : 6, :]

function transform{T}(jac::GeometricJacobian{T}, transform::Transform3D{T})
    @assert jac.frame == transform.from
    R = rotationmatrix(transform.rot)
    angular = R * angularPart(jac)
    linear = R * linearPart(jac)
    transArray = Array(transform.trans)
    for i = 1 : size(angular, 2)
        linear[:, i] += cross(transArray, angular[:, i])
    end
    return GeometricJacobian(jac.body, jac.base, transform.to, [angular; linear])
end

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
