immutable Twist{T<:Real}
    # describes motion of body w.r.t. base, expressed in frame
    body::CartesianFrame3D
    base::CartesianFrame3D
    frame::CartesianFrame3D
    angular::Vec{3, T}
    linear::Vec{3, T}
end
Twist{T<:Real}(body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D, vec::Vector{T}) = Twist(body, base, frame, vec[1 : 3], vec[4 : 6])

function (+)(twist1::Twist, twist2::Twist)
    @assert twist1.body == twist2.base
    @assert twist1.frame == twist2.frame
    return Twist(twist2.body, twist1.base, twist1.frame, twist1.angular + twist2.angular, twist1.linear + twist2.linear)
end
(-)(t::Twist) = Twist(t.base, t.body, t.frame, -t.angular, -t.linear)

function transform(twist::Twist, transform::Transform3D)
    @assert twist.frame == transform.from
    R = Mat(rotationmatrix(transform.rot))
    angular = R * twist.angular
    linear = R * twist.linear + cross(transform.trans, angular)
    return Twist(twist.body, twist.base, transform.to, angular, linear)
end

change_base_no_relative_motion(t::Twist, base::CartesianFrame3D) = Twist(t.body, base, t.frame, t.angular, t.linear)
change_body_no_relative_motion(t::Twist, body::CartesianFrame3D) = Twist(body, t.base, t.frame, t.angular, t.linear)
zero{T}(::Type{Twist{T}}, body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D) = Twist(body, base, frame, zero(Vec{3, T}), zero(Vec{3, T}))
rand{T}(::Type{Twist{T}}, body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D) = Twist(body, base, frame, rand(Vec{3, T}), rand(Vec{3, T}))

type GeometricJacobian{T<:Real}
    body::CartesianFrame3D
    base::CartesianFrame3D
    frame::CartesianFrame3D
    mat::Array{T}
end
angularPart(jac::GeometricJacobian) = jac.mat[1 : 3, :]
linearPart(jac::GeometricJacobian) = jac.mat[4 : 6, :]
(*){R<:Real}(jac::GeometricJacobian, v::Vector{R}) = Twist(jac.body, jac.base, jac.frame, jac.mat * v)
(-)(jac::GeometricJacobian) = GeometricJacobian(jac.base, jac.body, jac.frame, -jac.mat)

function hcat{T}(jacobians::GeometricJacobian{T}...)
    frame = jacobians[1].frame
    for j = 2 : length(jacobians)
        @assert jacobians[j].frame == frame
        @assert jacobians[j].base == jacobians[j - 1].body
    end
    return GeometricJacobian(jacobians[end].body, jacobians[1].base, frame, hcat([jac.mat for jac in jacobians]...))
end

function transform(jac::GeometricJacobian, transform::Transform3D)
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

abstract ForceSpaceElement{T<:Real}

immutable Wrench{T<:Real} <: ForceSpaceElement{T}
    frame::CartesianFrame3D
    angular::Vec{3, T}
    linear::Vec{3, T}
end
Wrench{T}(frame::CartesianFrame3D, vec::Vector{T}) = Wrench{T}(frame, vec[1 : 3], vec[4 : 6])

immutable Momentum{T<:Real} <: ForceSpaceElement{T}
    frame::CartesianFrame3D
    angular::Vec{3, T}
    linear::Vec{3, T}
end
Momentum{T}(frame::CartesianFrame3D, vec::Vector{T}) = Momentum{T}(frame, vec[1 : 3], vec[4 : 6])

zero{F<:ForceSpaceElement}(::Type{F}, frame::CartesianFrame3D) = F(frame, zero(Vec{3, T}), zero(Vec{3, T}))

function transform{F<:ForceSpaceElement}(f::F, transform::Transform3D)
    @assert f.frame == transform.from
    R = Mat(rotationmatrix(transform.rot))
    linear = R * f.linear
    angular = R * f.angular + cross(transform.trans, linear)
    return F(transform.to, angular, linear)
end

function (+){F<:ForceSpaceElement}(f1::F, f2::F)
    @assert f1.frame == f2.frame
    return F(f1.frame, f1.angular + f2.angular, f1.linear + f2.linear)
end
(-){F<:ForceSpaceElement}(f::F) = F(f.frame, -f.angular, -f.linear)

type MomentumMatrix{T<:Real}
    frame::CartesianFrame3D
    mat::Array{T}
end
function (*){T}(inertia::SpatialInertia{T}, jac::GeometricJacobian{T})
    @assert inertia.frame == jac.frame

    Jω = angularPart(jac)
    Jv = linearPart(jac)
    I = Array(inertia.moment)
    c = Array(inertia.centerOfMass)
    m = inertia.mass
    angular = I * Jω
    linear = m * Jv
    for i = 1 : size(Jω, 2)
        mc = m * c
        angular[:, i] += cross(mc, Jv[:, i])
        linear[:, i] -= cross(mc, Jω[:, i])
    end
    return MomentumMatrix(inertia.frame, [linear; angular])
end

function hcat{T}(mats::MomentumMatrix{T}...)
    frame = mats[1].frame
    for j = 2 : length(mats)
        @assert mats[j].frame == frame
    end
    return MomentumMatrix(frame, hcat([m.mat for m in mats]...))
end

(*){T, R<:Real}(mat::MomentumMatrix{T}, v::Vector{R}) = Momentum(mat.frame, mat.mat * v)

immutable SpatialAcceleration{T<:Real}
    body::CartesianFrame3D
    base::CartesianFrame3D
    frame::CartesianFrame3D
    angular::Vec{3, T}
    linear::Vec{3, T}
end

function (+)(accel1::SpatialAcceleration, accel2::SpatialAcceleration)
    @assert accel1.body == accel2.base
    @assert accel1.frame == accel2.frame
    return SpatialAcceleration(accel2.body, accel1.base, accel1.frame, accel1.angular + accel2.angular, accel1.linear + accel2.linear)
end
(-)(accel::SpatialAcceleration) = SpatialAcceleration(accel.base, accel.body, accel.frame, -accel.angular, -accel.linear)
