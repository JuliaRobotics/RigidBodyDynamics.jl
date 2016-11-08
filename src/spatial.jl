immutable SpatialInertia{T<:Real}
    frame::CartesianFrame3D
    moment::SMatrix{3, 3, T, 9}
    crossPart::SVector{3, T} # mass times center of mass
    mass::T
end

convert{T}(::Type{SpatialInertia{T}}, inertia::SpatialInertia{T}) = inertia

function convert{T<:Real}(::Type{SpatialInertia{T}}, inertia::SpatialInertia)
    SpatialInertia(inertia.frame, convert(SMatrix{3, 3, T}, inertia.moment), convert(SVector{3, T}, inertia.crossPart), convert(T, inertia.mass))
end

function convert{T}(::Type{SMatrix{6, 6, T}}, inertia::SpatialInertia)
    J = inertia.moment
    C = hat(inertia.crossPart)
    m = inertia.mass
    [J  C; C' m * eye(SMatrix{3, 3, T})]
end

convert{T<:Matrix}(::Type{T}, inertia::SpatialInertia) = convert(T, convert(SMatrix{6, 6, eltype(T)}, inertia))

Array{T}(inertia::SpatialInertia{T}) = convert(Matrix{T}, inertia)  # TODO: clean up

center_of_mass(inertia::SpatialInertia) = Point3D(inertia.frame, inertia.crossPart / inertia.mass)

function show(io::IO, inertia::SpatialInertia)
    println(io, "SpatialInertia expressed in \"$(name(inertia.frame))\":")
    println(io, "mass: $(inertia.mass)")
    println(io, "center of mass: $(center_of_mass(inertia))")
    print(io, "moment of inertia:\n$(inertia.moment)")
end

zero{T}(::Type{SpatialInertia{T}}, frame::CartesianFrame3D) = SpatialInertia(frame, zeros(SMatrix{3, 3, T}), zeros(SVector{3, T}), zero(T))

function isapprox(x::SpatialInertia, y::SpatialInertia; atol = 1e-12)
    x.frame == y.frame && isapprox(x.moment, y.moment; atol = atol) && isapprox(x.crossPart, y.crossPart; atol = atol) && isapprox(x.mass, y.mass; atol = atol)
end

function (+){T}(inertia1::SpatialInertia{T}, inertia2::SpatialInertia{T})
    framecheck(inertia1.frame, inertia2.frame)
    moment = inertia1.moment + inertia2.moment
    crossPart = inertia1.crossPart + inertia2.crossPart
    mass = inertia1.mass + inertia2.mass
    SpatialInertia(inertia1.frame, moment, crossPart, mass)
end

function transform{I, T}(inertia::SpatialInertia{I}, t::Transform3D{T})::SpatialInertia{promote_type(I, T)}
    framecheck(t.from, inertia.frame)
    S = promote_type(I, T)

    if t.from == t.to
        return convert(SpatialInertia{S}, inertia)
    elseif inertia.mass == zero(I)
        return zero(SpatialInertia{S}, t.to)
    else
        J = convert(SMatrix{3, 3, S}, inertia.moment)
        m = convert(S, inertia.mass)
        c = convert(SVector{3, S}, inertia.crossPart)

        R = rotation_matrix(convert(Quaternion{S}, t.rot))
        p = convert(SVector{3, S}, t.trans)

        cnew = R * c
        Jnew = hat_squared(cnew)
        cnew += m * p
        Jnew -= hat_squared(cnew)
        mInv = inv(m)
        Jnew *= mInv
        Jnew += R * J * R'
        return SpatialInertia{S}(t.to, Jnew, cnew, m)
    end
end

function rand{T}(::Type{SpatialInertia{T}}, frame::CartesianFrame3D)
    # Try to generate a random but physical moment of inertia
    # by constructing it from its eigendecomposition
    Q = rotationmatrix(qrotation(rand(T, 3) * 2*pi))
    principalMoments = Vector{T}(3)

    # Scale the inertias to make the length scale of the
    # equivalent inertial ellipsoid roughly ~1 unit
    principalMoments[1:2] = rand(2) / 10.

    # Ensure that the principal moments of inertia obey the triangle
    # inequalities:
    # http://www.mathworks.com/help/physmod/sm/mech/vis/about-body-color-and-geometry.html
    lb = abs(principalMoments[1] - principalMoments[2])
    ub = principalMoments[1] + principalMoments[2]
    principalMoments[3] = rand() * (ub - lb) + lb

    # Construct the moment of inertia tensor
    J = SMatrix{3, 3, T}(Q * diagm(principalMoments) * Q')

    # Construct the inertia in CoM frame
    comFrame = CartesianFrame3D("com")
    spatialInertia = SpatialInertia(comFrame, J, zeros(SVector{3, T}), rand(T))

    # Put the center of mass at a random offset
    comFrameToDesiredFrame = Transform3D(comFrame, frame, rand(SVector{3, T}) - 0.5)
    transform(spatialInertia, comFrameToDesiredFrame)
end

immutable Twist{T<:Real}
    # describes motion of body w.r.t. base, expressed in frame
    body::CartesianFrame3D
    base::CartesianFrame3D
    frame::CartesianFrame3D
    angular::SVector{3, T}
    linear::SVector{3, T}
end
Twist{T<:Real}(body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D, vec::AbstractVector{T}) = Twist(body, base, frame, SVector(vec[1], vec[2], vec[3]), SVector(vec[4], vec[5], vec[6]))
convert{T<:Real}(::Type{Twist{T}}, t::Twist{T}) = t
convert{T<:Real}(::Type{Twist{T}}, t::Twist) = Twist(t.body, t.base, t.frame, convert(SVector{3, T}, t.angular), convert(SVector{3, T}, t.linear))
convert{T}(::Type{Vector{T}}, twist::Twist{T}) = [twist.angular...; twist.linear...] # TODO: clean up
Array{T}(twist::Twist{T}) = convert(Vector{T}, twist) # TODO: clean up

function show(io::IO, t::Twist)
    print(io, "Twist of \"$(name(t.body))\" w.r.t \"$(name(t.base))\" in \"$(name(t.frame))\":\nangular: $(t.angular), linear: $(t.linear)")
end

function isapprox(x::Twist, y::Twist; atol = 1e-12)
    x.body == y.body && x.base == y.base && x.frame == y.frame && isapprox(x.angular, y.angular; atol = atol) && isapprox(x.linear, y.linear; atol = atol)
end


function (+)(twist1::Twist, twist2::Twist)
    framecheck(twist1.frame, twist2.frame)
    if twist1.body == twist2.base
        return Twist(twist2.body, twist1.base, twist1.frame, twist1.angular + twist2.angular, twist1.linear + twist2.linear)
    elseif twist1.base == twist2.body
        return Twist(twist1.body, twist2.base, twist1.frame, twist1.angular + twist2.angular, twist1.linear + twist2.linear)
    else
        throw(ArgumentError("frame mismatch"))
    end
end
(-)(t::Twist) = Twist(t.base, t.body, t.frame, -t.angular, -t.linear)

function transform_spatial_motion(angular::SVector{3}, linear::SVector{3}, rot::Quaternion, p::SVector{3})
    angular = rotate(angular, rot)
    linear = rotate(linear, rot) + cross(p, angular)
    angular, linear
end

function transform(twist::Twist, transform::Transform3D)
    framecheck(twist.frame, transform.from)
    angular, linear = transform_spatial_motion(twist.angular, twist.linear, transform.rot, transform.trans)
    Twist(twist.body, twist.base, transform.to, angular, linear)
end

change_base_no_relative_motion(t::Twist, base::CartesianFrame3D) = Twist(t.body, base, t.frame, t.angular, t.linear)
change_body_no_relative_motion(t::Twist, body::CartesianFrame3D) = Twist(body, t.base, t.frame, t.angular, t.linear)
zero{T}(::Type{Twist{T}}, body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D) = Twist(body, base, frame, zeros(SVector{3, T}), zeros(SVector{3, T}))
rand{T}(::Type{Twist{T}}, body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D) = Twist(body, base, frame, rand(SVector{3, T}), rand(SVector{3, T}))

immutable GeometricJacobian{A<:AbstractMatrix}
    body::CartesianFrame3D
    base::CartesianFrame3D
    frame::CartesianFrame3D
    angular::A
    linear::A

    function GeometricJacobian(body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D, angular::A, linear::A)
        @boundscheck size(angular, 1) == 3 || error("size mismatch")
        @boundscheck size(linear, 1) == 3 || error("size mismatch")
        @boundscheck size(angular, 2) == size(linear, 2) || error("size mismatch")
        new(body, base, frame, angular, linear)
    end
end

function GeometricJacobian{A<:AbstractMatrix}(body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D, angular::A, linear::A)
    GeometricJacobian{A}(body, base, frame, angular, linear)
end

# MotionSubspace is the return type of the motion_subspace(::Joint, ...) method. Defining it as a
# GeometricJacobian with a view of a 3×6 SMatrix as the underlying data type gets around type
# instabilities in motion_subspace while still using an isbits type.
# See https://github.com/tkoolen/RigidBodyDynamics.jl/issues/84.
typealias MotionSubspace{T} GeometricJacobian{ContiguousSMatrixColumnView{3, 6, T, 18}}

@generated function MotionSubspace{N, T}(body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D, angular::SMatrix{3, N, T}, linear::SMatrix{3, N, T})
    fillerSize = 6 - N
    return quote
        $(Expr(:meta, :inline))
        filler = fill(NaN, SMatrix{3, $fillerSize, T})
        angularData = hcat(angular, filler)::SMatrix{3, 6, T, 18}
        linearData = hcat(linear, filler)::SMatrix{3, 6, T, 18}
        MotionSubspace{T}(body, base, frame, view(angularData, :, 1 : N), view(linearData, :, 1 : N))
    end
end

function MotionSubspace{T}(body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D, angular::SMatrix{3, 0, T}, linear::SMatrix{3, 0, T})
    angularData = fill(NaN, SMatrix{3, 6, T})
    linearData = fill(NaN, SMatrix{3, 6, T})
    MotionSubspace{T}(body, base, frame, view(angularData, :, 1 : 0), view(linearData, :, 1 : 0))
end

function MotionSubspace{T}(body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D, angular::SMatrix{3, 6, T}, linear::SMatrix{3, 6, T})
    MotionSubspace{T}(body, base, frame, view(angular, :, 1 : 6), view(linear, :, 1 : 6))
end

convert{A}(::Type{GeometricJacobian{A}}, jac::GeometricJacobian{A}) = jac
convert{A}(::Type{GeometricJacobian{A}}, jac::GeometricJacobian) = GeometricJacobian(jac.body, jac.base, jac.frame, convert(A, jac.angular), convert(A, jac.linear))
Array(jac::GeometricJacobian) = [Array(jac.angular); Array(jac.linear)]

eltype{A}(::Type{GeometricJacobian{A}}) = eltype(A)
num_cols(jac::GeometricJacobian) = size(jac.angular, 2)
angular_part(jac::GeometricJacobian) = jac.angular
linear_part(jac::GeometricJacobian) = jac.linear

function Twist(jac::GeometricJacobian, v::AbstractVector)
    angular = convert(SVector{3}, _mul(jac.angular, v))
    linear = convert(SVector{3}, _mul(jac.linear, v))
    Twist(jac.body, jac.base, jac.frame, angular, linear)
end

(-)(jac::GeometricJacobian) = GeometricJacobian(jac.base, jac.body, jac.frame, -jac.angular, -jac.linear)
function show(io::IO, jac::GeometricJacobian)
    print(io, "GeometricJacobian: body: \"$(name(jac.body))\", base: \"$(name(jac.base))\", expressed in \"$(name(jac.frame))\":\n$(Array(jac))")
end

function hcat(jacobians::GeometricJacobian...)
    frame = jacobians[1].frame
    for j = 2 : length(jacobians)
        framecheck(jacobians[j].frame, frame)
        framecheck(jacobians[j].base, jacobians[j - 1].body)
    end
    angular = hcat((jac.angular for jac in jacobians)...)
    linear = hcat((jac.linear for jac in jacobians)...)
    GeometricJacobian(jacobians[end].body, jacobians[1].base, frame, angular, linear)
end

function transform(jac::GeometricJacobian, transform::Transform3D)
    framecheck(jac.frame, transform.from)
    R = rotation_matrix(transform.rot)
    angular = R * jac.angular
    linear = R * jac.linear + cross(transform.trans, angular)
    GeometricJacobian(jac.body, jac.base, transform.to, angular, linear)
end

abstract ForceSpaceElement{T<:Real}

immutable Wrench{T<:Real} <: ForceSpaceElement{T}
    frame::CartesianFrame3D
    angular::SVector{3, T}
    linear::SVector{3, T}
end
Wrench{T}(frame::CartesianFrame3D, vec::AbstractVector{T}) = Wrench{T}(frame, SVector(vec[1], vec[2], vec[3]), SVector(vec[4], vec[5], vec[6]))
convert{T<:Real}(::Type{Wrench{T}}, wrench::Wrench{T}) = wrench
convert{T<:Real}(::Type{Wrench{T}}, wrench::Wrench) = Wrench(wrench.frame, convert(SVector{3, T}, wrench.angular), convert(SVector{3, T}, wrench.linear))

show(io::IO, w::Wrench) = print(io, "Wrench expressed in \"$(name(w.frame))\":\nangular: $(w.angular), linear: $(w.linear)")
zero{T}(::Type{Wrench{T}}, frame::CartesianFrame3D) = Wrench(frame, zeros(SVector{3, T}), zeros(SVector{3, T}))
rand{T}(::Type{Wrench{T}}, frame::CartesianFrame3D) = Wrench(frame, rand(SVector{3, T}), rand(SVector{3, T}))

dot(w::Wrench, t::Twist) = begin framecheck(w.frame, t.frame); dot(w.angular, t.angular) + dot(w.linear, t.linear) end
dot(t::Twist, w::Wrench) = dot(w, t)

immutable Momentum{T<:Real} <: ForceSpaceElement{T}
    frame::CartesianFrame3D
    angular::SVector{3, T}
    linear::SVector{3, T}
end
Momentum{T}(frame::CartesianFrame3D, vec::AbstractVector{T}) = Momentum{T}(frame, SVector(vec[1], vec[2], vec[3]), SVector(vec[4], vec[5], vec[6]))
convert{T<:Real}(::Type{Wrench{T}}, momentum::Momentum{T}) = momentum
convert{T<:Real}(::Type{Wrench{T}}, momentum::Momentum) = Momentum(momentum.frame, convert(SVector{3, T}, momentum.angular), convert(SVector{3, T}, momentum.linear))

show(io::IO, m::Momentum) = print(io, "Momentum expressed in \"$(name(m.frame))\":\nangular: $(m.angular), linear: $(m.linear)")
zero{T}(::Type{Momentum{T}}, frame::CartesianFrame3D) = Momentum(frame, zeros(SVector{3, T}), zeros(SVector{3, T}))
rand{T}(::Type{Momentum{T}}, frame::CartesianFrame3D) = Momentum(frame, rand(SVector{3, T}), rand(SVector{3, T}))


function transform{F<:ForceSpaceElement}(f::F, transform::Transform3D)
    framecheck(f.frame, transform.from)
    linear = rotate(f.linear, transform.rot)
    angular = rotate(f.angular, transform.rot) + cross(transform.trans, linear)
    F(transform.to, angular, linear)
end

function (+){F<:ForceSpaceElement}(f1::F, f2::F)
    framecheck(f1.frame, f2.frame)
    F(f1.frame, f1.angular + f2.angular, f1.linear + f2.linear)
end

function (-){F<:ForceSpaceElement}(f1::F, f2::F)
    framecheck(f1.frame, f2.frame)
    F(f1.frame, f1.angular - f2.angular, f1.linear - f2.linear)
end

(-){F<:ForceSpaceElement}(f::F) = F(f.frame, -f.angular, -f.linear)

Array{F<:ForceSpaceElement}(f::F) = [f.angular...; f.linear...]
isapprox{F<:ForceSpaceElement}(x::F, y::F; atol = 1e-12) = x.frame == y.frame && isapprox(x.angular, y.angular, atol = atol) && isapprox(x.linear, y.linear, atol = atol)

function mul_inertia(J, c, m, ω, v)
    angular = J * ω + cross(c, v)
    linear = m * v - cross(c, ω)
    angular, linear
end

function (*)(inertia::SpatialInertia, twist::Twist)
    framecheck(inertia.frame, twist.frame)
    Momentum(inertia.frame, mul_inertia(inertia.moment, inertia.crossPart, inertia.mass, twist.angular, twist.linear)...)
end


immutable MomentumMatrix{A<:AbstractMatrix}
    frame::CartesianFrame3D
    angular::A
    linear::A

    function MomentumMatrix(frame::CartesianFrame3D, angular::A, linear::A)
        @assert size(angular, 1) == 3
        @assert size(linear, 1) == 3
        @assert size(angular, 2) == size(linear, 2)
        new(frame, angular, linear)
    end
end

function MomentumMatrix{A<:AbstractMatrix}(frame::CartesianFrame3D, angular::A, linear::A)
    MomentumMatrix{A}(frame, angular, linear)
end

# function MomentumMatrix{A<:AbstractMatrix}(frame::CartesianFrame3D, mat::A) # TODO: remove?
#     @assert size(mat, 1) == 6
#     @inbounds angular = mat[1 : 3, :]
#     @inbounds linear = mat[4 : 6, :]
#     MomentumMatrix(frame, angular, linear)
# end

convert{A}(::Type{MomentumMatrix{A}}, mat::MomentumMatrix{A}) = mat
convert{A}(::Type{MomentumMatrix{A}}, mat::MomentumMatrix) = MomentumMatrix(mat.frame, convert(A, mat.angular), convert(A, mat.linear))
Array(mat::MomentumMatrix) = [Array(mat.angular); Array(mat.linear)]

eltype{A}(::Type{MomentumMatrix{A}}) = eltype(A)
num_cols(mat::MomentumMatrix) = size(mat.angular, 2)
angular_part(mat::MomentumMatrix) = mat.angular
linear_part(mat::MomentumMatrix) = mat.linear

show(io::IO, m::MomentumMatrix) = print(io, "MomentumMatrix expressed in \"$(name(m.frame))\":\n$(Array(m))")

function (*)(inertia::SpatialInertia, jac::GeometricJacobian)
    framecheck(inertia.frame, jac.frame)
    Jω = jac.angular
    Jv = jac.linear
    J = inertia.moment
    m = inertia.mass
    c = inertia.crossPart
    angular = J * Jω + cross(c, Jv)
    linear = m * Jv - cross(c, Jω)
    MomentumMatrix(inertia.frame, angular, linear)
end

function hcat(mats::MomentumMatrix...)
    frame = mats[1].frame
    for j = 2 : length(mats)
        framecheck(mats[j].frame, frame)
    end
    angular = hcat((m.angular for m in mats)...)
    linear = hcat((m.linear for m in mats)...)
    MomentumMatrix(frame, angular, linear)
end

function Momentum(mat::MomentumMatrix, v::AbstractVector)
    Momentum(mat.frame, convert(SVector{3}, mat.angular * v), convert(SVector{3}, mat.linear * v))
end

function Wrench(mat::MomentumMatrix, v̇::AbstractVector)
    Wrench(mat.frame, convert(SVector{3}, mat.angular * v̇), convert(SVector{3}, mat.linear * v̇))
end

function transform(mat::MomentumMatrix, transform::Transform3D)
    framecheck(mat.frame, transform.from)
    R = rotation_matrix(transform.rot)
    linear = R * linear_part(mat)
    T = eltype(linear)
    angular = R * angular_part(mat) + cross(transform.trans, linear)
    MomentumMatrix(transform.to, angular, linear)
end

immutable SpatialAcceleration{T<:Real}
    body::CartesianFrame3D
    base::CartesianFrame3D
    frame::CartesianFrame3D
    angular::SVector{3, T}
    linear::SVector{3, T}
end

convert{T<:Real}(::Type{SpatialAcceleration{T}}, accel::SpatialAcceleration{T}) = accel
convert{T<:Real}(::Type{SpatialAcceleration{T}}, accel::SpatialAcceleration) = SpatialAcceleration(accel.body, accel.base, accel.frame, convert(SVector{3, T}, accel.angular), convert(SVector{3, T}, accel.linear))

function SpatialAcceleration(body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D, vec::AbstractVector)
    SpatialAcceleration(body, base, frame, SVector(vec[1], vec[2], vec[3]), SVector(vec[4], vec[5], vec[6]))
end

function SpatialAcceleration(jac::GeometricJacobian, v̇::AbstractVector)
    angular = convert(SVector{3}, _mul(jac.angular, v̇))
    linear = convert(SVector{3}, _mul(jac.linear, v̇))
    SpatialAcceleration(jac.body, jac.base, jac.frame, angular, linear)
end

function isapprox(x::SpatialAcceleration, y::SpatialAcceleration; atol = 1e-12)
    x.body == y.body && x.base == y.base && x.frame == y.frame && isapprox(x.angular, y.angular; atol = atol) && isapprox(x.linear, y.linear; atol = atol)
end

function (+)(accel1::SpatialAcceleration, accel2::SpatialAcceleration)
    framecheck(accel1.frame, accel2.frame)
    if accel1.body == accel2.base
        return SpatialAcceleration(accel2.body, accel1.base, accel1.frame, accel1.angular + accel2.angular, accel1.linear + accel2.linear)
    elseif accel1.body == accel2.body && accel1.base == accel2.base
        return SpatialAcceleration(accel1.body, accel1.base, accel1.frame, accel1.angular + accel2.angular, accel1.linear + accel2.linear)
    end
    @assert false
end

(-)(accel::SpatialAcceleration) = SpatialAcceleration(accel.base, accel.body, accel.frame, -accel.angular, -accel.linear)

Array(accel::SpatialAcceleration) = [accel.angular...; accel.linear...]

function show(io::IO, a::SpatialAcceleration)
    print(io, "SpatialAcceleration of \"$(name(a.body))\" w.r.t \"$(name(a.base))\" in \"$(name(a.frame))\":\nangular: $(a.angular), linear: $(a.linear)")
end

# also known as 'spatial motion cross product'
@inline function se3_commutator(xω, xv, yω, yv)
    angular = cross(xω, yω)
    linear = cross(xω, yv) + cross(xv, yω)
    angular, linear
end

function transform(accel::SpatialAcceleration, oldToNew::Transform3D, twistOfCurrentWrtNew::Twist, twistOfBodyWrtBase::Twist)
    # trivial case
    accel.frame == oldToNew.to && return accel

    # frame checks
    framecheck(oldToNew.from, accel.frame)
    framecheck(twistOfCurrentWrtNew.frame, accel.frame)
    framecheck(twistOfCurrentWrtNew.body, accel.frame)
    framecheck(twistOfCurrentWrtNew.base, oldToNew.to)
    framecheck(twistOfBodyWrtBase.frame, accel.frame)
    framecheck(twistOfBodyWrtBase.body, accel.body)
    framecheck(twistOfBodyWrtBase.base, accel.base)

    # 'cross term':
    angular, linear = se3_commutator(
        twistOfCurrentWrtNew.angular, twistOfCurrentWrtNew.linear,
        twistOfBodyWrtBase.angular, twistOfBodyWrtBase.linear)

    # add current acceleration:
    angular += accel.angular
    linear += accel.linear

    # transform to new frame
    angular, linear = transform_spatial_motion(angular, linear, oldToNew.rot, oldToNew.trans)

    SpatialAcceleration(accel.body, accel.base, oldToNew.to, angular, linear)
end

zero{T}(::Type{SpatialAcceleration{T}}, body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D) = SpatialAcceleration(body, base, frame, zeros(SVector{3, T}), zeros(SVector{3, T}))
rand{T}(::Type{SpatialAcceleration{T}}, body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D) = SpatialAcceleration(body, base, frame, rand(SVector{3, T}), rand(SVector{3, T}))

function newton_euler(I::SpatialInertia, Ṫ::SpatialAcceleration, T::Twist)
    body = Ṫ.body
    base = Ṫ.base # TODO: should assert that this is an inertial frame somehow
    frame = Ṫ.frame

    framecheck(I.frame, frame)
    framecheck(T.body, body)
    framecheck(T.base, base)
    framecheck(T.frame, frame)

    angular, linear = mul_inertia(I.moment, I.crossPart, I.mass, Ṫ.angular, Ṫ.linear)
    angularMomentum, linearMomentum = mul_inertia(I.moment, I.crossPart, I.mass, T.angular, T.linear)
    angular += cross(T.angular, angularMomentum) + cross(T.linear, linearMomentum)
    linear += cross(T.angular, linearMomentum)
    Wrench(frame, angular, linear)
end

function joint_torque(jac::GeometricJacobian, wrench::Wrench)
    framecheck(jac.frame, wrench.frame)
    jac.angular' * wrench.angular + jac.linear' * wrench.linear
end

function kinetic_energy(I::SpatialInertia, twist::Twist)
    framecheck(I.frame, twist.frame)
    # TODO: should assert that twist.base is an inertial frame somehow
    ω = twist.angular
    v = twist.linear
    J = I.moment
    c = I.crossPart
    m = I.mass
    1/2 * (dot(ω, J * ω) + dot(v, m * v + 2 * cross(ω, c)))
end

# log(::Transform3D) + some extra outputs that make log_with_time_derivative faster
function _log(t::Transform3D)
    # Proposition 2.9 in Murray et al, "A mathematical introduction to robotic manipulation."
    rot = t.rot
    p = t.trans

    # Rotational part of local coordinates is simply the rotation vector.
    θ, axis = angle_axis_proper(rot)
    ϕrot = θ * axis

    # Translational part from Bullo and Murray, "Proportional derivative (PD) control on the Euclidean group.",
    # (2.4) and (2.5), which provide a closed form solution of the inverse of the A matrix in proposition 2.9 of Murray et al.

    θ_over_2 = θ / 2
    sθ_over_2 = sin(θ_over_2)
    cθ_over_2 = cos(θ_over_2)
    θ_squared = θ^2
    if abs(angle_difference(θ, zero(θ))) < eps(θ)
        α = one(θ_over_2)
        ϕtrans = p
    else
        α = θ_over_2 * cθ_over_2 / sθ_over_2
        ϕtrans = p - ϕrot × p / 2 + (1 - α) / θ_squared * ϕrot × (ϕrot × p) # Bullo, Murray, (2.5)
    end

    ξ = Twist(t.from, t.to, t.to, ϕrot, ϕtrans) # twist in base frame; see section 4.3
    ξ, θ, θ_squared, θ_over_2, sθ_over_2, cθ_over_2, α
end

function log(t::Transform3D)
    first(_log(t))
end

# Compute exponential coordinates as well as their time derivatives in one shot.
# This mainly exists because ForwardDiff won't work at the singularity of log.
# It is also ~50% faster than ForwardDiff in this case.
function log_with_time_derivative(t::Transform3D, twist::Twist)
    # See Bullo and Murray, "Proportional derivative (PD) control on the Euclidean group.", Lemma 4.
    # This is truely magic.
    # Notation matches Bullo and Murray.

    framecheck(twist.body, t.from)
    framecheck(twist.base, t.to)
    framecheck(twist.frame, twist.body) # required by Lemma 4.

    X, θ, θ_squared, θ_over_2, sθ_over_2, cθ_over_2, α = _log(t)

    ψ = X.angular
    q = X.linear

    ω = twist.angular
    v = twist.linear

    ψ̇ = ω
    q̇ = v
    if abs(angle_difference(θ, zero(θ))) > eps(θ)
        β = θ_over_2^2 / sθ_over_2^2
        A = (2 * (1 - α) + (α - β) / 2) / θ_squared
        B = ((1 - α) + (α - β) / 2) / θ_squared^2
        adψ̇, adq̇ = se3_commutator(ψ, q, ω, v)
        ad2ψ̇, ad2q̇ = se3_commutator(ψ, q, adψ̇, adq̇)
        ad3ψ̇, ad3q̇ = se3_commutator(ψ, q, ad2ψ̇, ad2q̇)
        ad4ψ̇, ad4q̇ = se3_commutator(ψ, q, ad3ψ̇, ad3q̇)
        ψ̇ += adψ̇ / 2 + A * ad2ψ̇ + B * ad4ψ̇
        q̇ += adq̇ / 2 + A * ad2q̇ + B * ad4q̇
    end
    Ẋ = SpatialAcceleration(X.body, X.base, X.frame, ψ̇, q̇)

    X, Ẋ
end

function exp(twist::Twist)
    # See Murray et al, "A mathematical introduction to robotic manipulation."
    framecheck(twist.frame, twist.base) # twist in base frame; see section 4.3
    ϕrot = twist.angular
    ϕtrans = twist.linear
    θ = norm(ϕrot)
    if abs(angle_difference(θ, zero(θ))) < eps(θ)
        # (2.32)
        rot = Quaternion{typeof(θ)}(one(θ), zero(θ), zero(θ), zero(θ), true)
        trans = ϕtrans
    else
        # (2.36)
        ω = ϕrot / θ
        rot = angle_axis_to_quaternion(θ, ω)
        v = ϕtrans / θ
        trans = ω × v
        trans -= rotate(trans, rot)
        trans += ω * dot(ω, v) * θ
    end
    Transform3D(twist.body, twist.base, rot, trans)
end
