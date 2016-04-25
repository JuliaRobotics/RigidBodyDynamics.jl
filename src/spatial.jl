immutable SpatialInertia{T<:Real}
    frame::CartesianFrame3D
    moment::Mat{3, 3, T}
    centerOfMass::Vec{3, T}
    mass::T
end
function show(io::IO, inertia::SpatialInertia)
    println(io, "SpatialInertia expressed in \"$(inertia.frame.name)\":")
    println(io, "mass: $(inertia.mass)")
    println(io, "center of mass: $(inertia.centerOfMass)")
    print(io, "moment of inertia:\n$(inertia.moment)")
end

function (+){T}(inertia1::SpatialInertia{T}, inertia2::SpatialInertia{T})
    @assert inertia1.frame == inertia2.frame
    moment = inertia1.moment + inertia2.moment
    mass = inertia1.mass + inertia2.mass
    centerOfMass = (inertia1.centerOfMass * inertia1.mass + inertia2.centerOfMass * inertia2.mass) / mass
    return SpatialInertia(inertia1.frame, moment, centerOfMass, mass)
end

function vector_to_skew_symmetric(v::Vec{3})
    return [
    0 -v[3] v[2];
    v[3] 0 -v[1];
    -v[2] v[1] 0];
end

function to_matrix(inertia::SpatialInertia)
    J = Array(inertia.moment)
    m = inertia.mass
    C = Array(vector_to_skew_symmetric(m * inertia.centerOfMass))
    return [J  C;
            C' m * eye(3)]
end


function transform{T}(inertia::SpatialInertia{T}, t::Transform3D{T})
    @assert t.from == inertia.frame

    function vector_to_skew_symmetric_squared(a::Vec{3, T})
        aSq = a .* a
        b11 = -aSq[2] - aSq[3]
        b12 = a[1] * a[2]
        b13 = a[1] * a[3]
        b22 = -aSq[1] - aSq[3]
        b23 = a[2] * a[3]
        b33 = -aSq[1] - aSq[2]
        return Mat((b11, b12, b13), (b12, b22, b23), (b13, b23, b33))
    end

    J = inertia.moment
    m = inertia.mass
    c = inertia.centerOfMass

    R = Mat(rotationmatrix(t.rot))
    p = t.trans

    cnew = R * (c * m)
    Jnew = vector_to_skew_symmetric_squared(cnew)
    cnew += m * p
    Jnew -= vector_to_skew_symmetric_squared(cnew)
    Jnew /= m
    Jnew += R * J * R'
    cnew /= m

    return SpatialInertia(t.to, Jnew, cnew, m)
end

rand{T}(::Type{SpatialInertia{T}}, frame::CartesianFrame3D) = SpatialInertia(frame, rand(Mat{3, 3, T}), rand(Vec{3, T}), rand(T))

immutable Twist{T<:Real}
    # describes motion of body w.r.t. base, expressed in frame
    body::CartesianFrame3D
    base::CartesianFrame3D
    frame::CartesianFrame3D
    angular::Vec{3, T}
    linear::Vec{3, T}
end
Twist{T<:Real}(body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D, vec::Vector{T}) = Twist(body, base, frame, vec[1 : 3], vec[4 : 6])
function show(io::IO, t::Twist)
    print(io, "Twist of \"$(t.body.name)\" w.r.t \"$(t.base.name)\" in \"$(t.frame.name)\":\nangular: $(t.angular), linear: $(t.linear)")
end

function (+)(twist1::Twist, twist2::Twist)
    @assert twist1.body == twist2.base
    @assert twist1.frame == twist2.frame
    return Twist(twist2.body, twist1.base, twist1.frame, twist1.angular + twist2.angular, twist1.linear + twist2.linear)
end
(-)(t::Twist) = Twist(t.base, t.body, t.frame, -t.angular, -t.linear)

function transform_spatial_motion(angular::Vec{3}, linear::Vec{3}, R::Mat{3, 3}, p::Vec{3}) # TODO: version that works directly with quaternion rotation parameterization
    angular = R * angular
    linear = R * linear + cross(p, angular)
    return angular, linear
end

function transform(twist::Twist, transform::Transform3D)
    @assert twist.frame == transform.from
    R = Mat(rotationmatrix(transform.rot))
    p = transform.trans
    angular, linear = transform_spatial_motion(twist.angular, twist.linear, R, p)
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
function show(io::IO, jac::GeometricJacobian)
    print(io, "GeometricJacobian: body: \"$(jac.body.name)\", base: \"$(jac.base.name)\", expressed in \"$(jac.frame.name)\":\n$(jac.mat)")
end

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
Wrench{T}(frame::CartesianFrame3D, vec::Vector{T}) = Wrench{T}(frame, Vec(vec[1 : 3]), Vec(vec[4 : 6]))
show(io::IO, w::Wrench) = print(io, "Wrench expressed in \"$(w.frame.name)\":\nangular: $(w.angular), linear: $(w.linear)")

immutable Momentum{T<:Real} <: ForceSpaceElement{T}
    frame::CartesianFrame3D
    angular::Vec{3, T}
    linear::Vec{3, T}
end
Momentum{T}(frame::CartesianFrame3D, vec::Vector{T}) = Momentum{T}(frame, Vec(vec[1 : 3]), Vec(vec[4 : 6]))
show(io::IO, m::Momentum) = print(io, "Momentum expressed in \"$(m.frame.name)\":\nangular: $(m.angular), linear: $(m.linear)")

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

function mul_inertia{T}(J::Mat{3, 3, T}, c::Vec{3, T}, m::T, ω::Vec{3, T}, v::Vec{3, T})
    mc = m * c
    angular = J * ω + cross(mc, v)
    linear = m * v - cross(mc, ω)
    return angular, linear
end

function (*){T}(inertia::SpatialInertia{T}, twist::Twist{T})
    @assert inertia.frame == twist.frame
    return Momentum(inertia.frame, mul_inertia(inertia.moment, inertia.centerOfMass, inertia.mass, twist.angular, twist.linear)...)
end


type MomentumMatrix{T<:Real}
    frame::CartesianFrame3D
    mat::Array{T}
end
show(io::IO, m::MomentumMatrix) = print(io, "MomentumMatrix expressed in \"$(m.frame.name)\":\n$(m.mat)")

function (*){T}(inertia::SpatialInertia{T}, jac::GeometricJacobian{T})
    @assert inertia.frame == jac.frame

    Jω = angularPart(jac)
    Jv = linearPart(jac)
    J = Array(inertia.moment)
    c = Array(inertia.centerOfMass)
    m = inertia.mass
    angular = J * Jω
    linear = m * Jv
    for i = 1 : size(Jω, 2)
        mc = m * c
        angular[:, i] += cross(mc, Jv[:, i])
        linear[:, i] -= cross(mc, Jω[:, i])
    end
    return MomentumMatrix(inertia.frame, [angular; linear])
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

function SpatialAcceleration{T}(body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D, vec::Vector{T})
    return SpatialAcceleration(body, base, frame, Vec(vec[1 : 3]), Vec(vec[4 : 6]))
end

function (+)(accel1::SpatialAcceleration, accel2::SpatialAcceleration)
    @assert accel1.frame == accel2.frame
    @assert accel1.body == accel2.base
    return SpatialAcceleration(accel2.body, accel1.base, accel1.frame, accel1.angular + accel2.angular, accel1.linear + accel2.linear)
end

(-)(accel::SpatialAcceleration) = SpatialAcceleration(accel.base, accel.body, accel.frame, -accel.angular, -accel.linear)

function show(io::IO, a::SpatialAcceleration)
    print(io, "SpatialAcceleration of \"$(a.body.name)\" w.r.t \"$(a.base.name)\" in \"$(a.frame.name)\":\nangular: $(a.angular), linear: $(a.linear)")
end

function transform{T}(accel::SpatialAcceleration{T}, oldToNew::Transform3D{T}, twistOfCurrentWrtNew::Twist{T}, twistOfBodyWrtBase::Twist{T})
    # trivial case
    accel.frame == oldToNew.to && return accel

    # frame checks
    @assert oldToNew.from == accel.frame
    @assert twistOfCurrentWrtNew.frame == accel.frame
    @assert twistOfCurrentWrtNew.body == accel.frame
    @assert twistOfCurrentWrtNew.base == oldToNew.to
    @assert twistOfBodyWrtBase.frame == accel.frame
    @assert twistOfBodyWrtBase.body == accel.body
    @assert twistOfBodyWrtBase.base == accel.base

    # spatial motion cross product:
    angular = cross(twistOfCurrentWrtNew.angular, twistOfBodyWrtBase.angular)
    linear = cross(twistOfCurrentWrtNew.linear, twistOfBodyWrtBase.angular)
    linear += cross(twistOfCurrentWrtNew.angular, twistOfBodyWrtBase.linear)

    # add current acceleration:
    angular += accel.angular
    linear += accel.linear

    # transform to new frame
    R = Mat(rotationmatrix(oldToNew.rot))
    p = oldToNew.trans
    angular, linear = transform_spatial_motion(angular, linear, R, p)

    return SpatialAcceleration(accel.body, accel.base, oldToNew.to, angular, linear)
end

zero{T}(::Type{SpatialAcceleration{T}}, body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D) = SpatialAcceleration(body, base, frame, zero(Vec{3, T}), zero(Vec{3, T}))
rand{T}(::Type{SpatialAcceleration{T}}, body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D) = SpatialAcceleration(body, base, frame, rand(Vec{3, T}), rand(Vec{3, T}))

function newton_euler(I::SpatialInertia, Ṫ::SpatialAcceleration, T::Twist)
    body = Ṫ.body
    base = Ṫ.base # TODO: should assert that this is an inertial frame somehow
    frame = Ṫ.frame

    @assert I.frame == frame
    @assert T.body == body
    @assert T.base == base
    @assert T.frame == frame

    angular, linear = mul_inertia(I.moment, I.centerOfMass, I.mass, Ṫ.angular, Ṫ.linear)
    angularMomentum, linearMomentum = mul_inertia(I.moment, I.centerOfMass, I.mass, T.angular, T.linear)
    angular += cross(T.angular, angularMomentum) + cross(T.linear, linearMomentum)
    linear += cross(T.angular, linearMomentum)
    return Wrench(frame, angular, linear)
end

function joint_torque(jac::GeometricJacobian, wrench::Wrench)
    @assert jac.frame == wrench.frame
    return jac.mat' * [wrench.angular...; wrench.linear...]
end
