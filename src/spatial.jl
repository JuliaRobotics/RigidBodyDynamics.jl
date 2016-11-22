# Types
immutable SpatialInertia{T<:Real}
    frame::CartesianFrame3D
    moment::SMatrix{3, 3, T, 9}
    crossPart::SVector{3, T} # mass times center of mass
    mass::T
end

for MotionSpaceElement in (:Twist, :SpatialAcceleration)
    @eval begin
        immutable $MotionSpaceElement{T<:Real}
            # describes motion of body w.r.t. base, expressed in frame
            body::CartesianFrame3D
            base::CartesianFrame3D
            frame::CartesianFrame3D
            angular::SVector{3, T}
            linear::SVector{3, T}
        end
    end
end

for ForceSpaceElement in (:Momentum, :Wrench)
    @eval begin
        immutable $ForceSpaceElement{T<:Real}
            frame::CartesianFrame3D
            angular::SVector{3, T}
            linear::SVector{3, T}
        end
    end
end

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


# SpatialInertia-specific functions
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

Array{T}(inertia::SpatialInertia{T}) = convert(Matrix{T}, inertia)

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


# MotionSpaceElement-specific
for MotionSpaceElement in (:Twist, :SpatialAcceleration)
    @eval begin
        function $MotionSpaceElement{T<:Real}(body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D, vec::AbstractVector{T})
            @boundscheck length(vec) == 6 || error("size mismatch")
            @inbounds angular = SVector(vec[1], vec[2], vec[3])
            @inbounds linear = SVector(vec[4], vec[5], vec[6])
            $MotionSpaceElement(body, base, frame, angular, linear)
        end

        convert{T<:Real}(::Type{$MotionSpaceElement{T}}, m::$MotionSpaceElement{T}) = m
        convert{T<:Real}(::Type{$MotionSpaceElement{T}}, m::$MotionSpaceElement) = $MotionSpaceElement(m.body, m.base, m.frame, convert(SVector{3, T}, m.angular), convert(SVector{3, T}, m.linear))
        convert{T}(::Type{Vector{T}}, m::$MotionSpaceElement{T}) = [m.angular...; m.linear...]
        Array{T}(m::$MotionSpaceElement{T}) = convert(Vector{T}, m)

        eltype{T}(::Type{$MotionSpaceElement{T}}) = T
        similar_type{T1, T2}(::Type{$MotionSpaceElement{T1}}, ::Type{T2}) = $MotionSpaceElement{T2}

        function show(io::IO, m::$MotionSpaceElement)
            print(io, "$($(MotionSpaceElement).name.name) of \"$(name(m.body))\" w.r.t \"$(name(m.base))\" in \"$(name(m.frame))\":\nangular: $(m.angular), linear: $(m.linear)")
        end

        function isapprox(x::$MotionSpaceElement, y::$MotionSpaceElement; atol = 1e-12)
            x.body == y.body && x.base == y.base && x.frame == y.frame && isapprox(x.angular, y.angular; atol = atol) && isapprox(x.linear, y.linear; atol = atol)
        end

        function (+)(m1::$MotionSpaceElement, m2::$MotionSpaceElement)
            framecheck(m1.frame, m2.frame)
            angular = m1.angular + m2.angular
            linear = m1.linear + m2.linear
            if m1.body == m2.body && m1.base == m2.base
                return $MotionSpaceElement(m1.body, m1.base, m1.frame, angular, linear)
            elseif m1.body == m2.base
                return $MotionSpaceElement(m2.body, m1.base, m1.frame, angular, linear)
            elseif m1.base == m2.body
                return $MotionSpaceElement(m1.body, m2.base, m1.frame, angular, linear)
            else
                throw(ArgumentError("frame mismatch"))
            end
        end

        (-)(m::$MotionSpaceElement) = $MotionSpaceElement(m.base, m.body, m.frame, -m.angular, -m.linear)

        change_base(m::$MotionSpaceElement, base::CartesianFrame3D) = $MotionSpaceElement(m.body, base, m.frame, m.angular, m.linear)

        function zero{T}(::Type{$MotionSpaceElement{T}}, body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D)
            $MotionSpaceElement(body, base, frame, zeros(SVector{3, T}), zeros(SVector{3, T}))
        end

        function rand{T}(::Type{$MotionSpaceElement{T}}, body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D)
            $MotionSpaceElement(body, base, frame, rand(SVector{3, T}), rand(SVector{3, T}))
        end
    end
end

# Twist-specific functions
function transform(twist::Twist, transform::Transform3D)
    framecheck(twist.frame, transform.from)
    angular, linear = transform_spatial_motion(twist.angular, twist.linear, transform.rot, transform.trans)
    Twist(twist.body, twist.base, transform.to, angular, linear)
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


# SpatialAcceleration-specific functions
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


# ForceSpaceElement-specific
for ForceSpaceElement in (:Momentum, :Wrench)
    @eval begin
        function $ForceSpaceElement{T}(frame::CartesianFrame3D, vec::AbstractVector{T})
            @boundscheck length(vec) == 6 || error("size mismatch")
            @inbounds angular = SVector(vec[1], vec[2], vec[3])
            @inbounds linear = SVector(vec[4], vec[5], vec[6])
            $ForceSpaceElement{T}(frame, angular, linear)
        end

        convert{T<:Real}(::Type{$ForceSpaceElement{T}}, f::$ForceSpaceElement{T}) = f

        function convert{T<:Real}(::Type{$ForceSpaceElement{T}}, f::$ForceSpaceElement)
            $ForceSpaceElement(f.frame, convert(SVector{3, T}, f.angular), convert(SVector{3, T}, f.linear))
        end

        eltype{T}(::Type{$ForceSpaceElement{T}}) = T
        similar_type{T1, T2}(::Type{$ForceSpaceElement{T1}}, ::Type{T2}) = $ForceSpaceElement{T2}

        show(io::IO, f::$ForceSpaceElement) = print(io, "$($(ForceSpaceElement).name.name) expressed in \"$(name(f.frame))\":\nangular: $(f.angular), linear: $(f.linear)")
        zero{T}(::Type{$ForceSpaceElement{T}}, frame::CartesianFrame3D) = $ForceSpaceElement(frame, zeros(SVector{3, T}), zeros(SVector{3, T}))
        rand{T}(::Type{$ForceSpaceElement{T}}, frame::CartesianFrame3D) = $ForceSpaceElement(frame, rand(SVector{3, T}), rand(SVector{3, T}))

        function transform(f::$ForceSpaceElement, transform::Transform3D)
            framecheck(f.frame, transform.from)
            linear = rotate(f.linear, transform.rot)
            angular = rotate(f.angular, transform.rot) + cross(transform.trans, linear)
            $ForceSpaceElement(transform.to, angular, linear)
        end

        function (+)(f1::$ForceSpaceElement, f2::$ForceSpaceElement)
            framecheck(f1.frame, f2.frame)
            $ForceSpaceElement(f1.frame, f1.angular + f2.angular, f1.linear + f2.linear)
        end

        function (-)(f1::$ForceSpaceElement, f2::$ForceSpaceElement)
            framecheck(f1.frame, f2.frame)
            $ForceSpaceElement(f1.frame, f1.angular - f2.angular, f1.linear - f2.linear)
        end

        (-)(f::$ForceSpaceElement) = $ForceSpaceElement(f.frame, -f.angular, -f.linear)

        Array(f::$ForceSpaceElement) = [f.angular...; f.linear...]
        isapprox(x::$ForceSpaceElement, y::$ForceSpaceElement; atol = 1e-12) = x.frame == y.frame && isapprox(x.angular, y.angular, atol = atol) && isapprox(x.linear, y.linear, atol = atol)
    end
end


# GeometricJacobian-specific functions
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

change_base(jac::GeometricJacobian, base::CartesianFrame3D) = GeometricJacobian(jac.body, base, jac.frame, jac.angular, jac.linear)

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


# Wrench-specific functions
dot(w::Wrench, t::Twist) = begin framecheck(w.frame, t.frame); dot(w.angular, t.angular) + dot(w.linear, t.linear) end
dot(t::Twist, w::Wrench) = dot(w, t)


# MomentumMatrix-specific functions
function MomentumMatrix{A<:AbstractMatrix}(frame::CartesianFrame3D, angular::A, linear::A)
    MomentumMatrix{A}(frame, angular, linear)
end

function MomentumMatrix{A<:AbstractMatrix}(frame::CartesianFrame3D, mat::A)
    @assert size(mat, 1) == 6
    @inbounds angular = mat[1 : 3, :]
    @inbounds linear = mat[4 : 6, :]
    MomentumMatrix(frame, angular, linear)
end

convert{A}(::Type{MomentumMatrix{A}}, mat::MomentumMatrix{A}) = mat
convert{A}(::Type{MomentumMatrix{A}}, mat::MomentumMatrix) = MomentumMatrix(mat.frame, convert(A, mat.angular), convert(A, mat.linear))
Array(mat::MomentumMatrix) = [Array(mat.angular); Array(mat.linear)]

eltype{A}(::Type{MomentumMatrix{A}}) = eltype(A)
num_cols(mat::MomentumMatrix) = size(mat.angular, 2)
angular_part(mat::MomentumMatrix) = mat.angular
linear_part(mat::MomentumMatrix) = mat.linear

show(io::IO, m::MomentumMatrix) = print(io, "MomentumMatrix expressed in \"$(name(m.frame))\":\n$(Array(m))")

function hcat(mats::MomentumMatrix...)
    frame = mats[1].frame
    for j = 2 : length(mats)
        framecheck(mats[j].frame, frame)
    end
    angular = hcat((m.angular for m in mats)...)
    linear = hcat((m.linear for m in mats)...)
    MomentumMatrix(frame, angular, linear)
end

function transform(mat::MomentumMatrix, transform::Transform3D)
    framecheck(mat.frame, transform.from)
    R = rotation_matrix(transform.rot)
    linear = R * linear_part(mat)
    T = eltype(linear)
    angular = R * angular_part(mat) + cross(transform.trans, linear)
    MomentumMatrix(transform.to, angular, linear)
end


# Interactions between spatial types
for MotionSpaceElement in (:Twist, :SpatialAcceleration)
    # GeometricJacobian * velocity vector --> Twist
    # GeometricJacobian * acceleration vector --> SpatialAcceleration
    @eval function $MotionSpaceElement(jac::GeometricJacobian, x::AbstractVector)
        angular = convert(SVector{3}, _mul(jac.angular, x))
        linear = convert(SVector{3}, _mul(jac.linear, x))
        $MotionSpaceElement(jac.body, jac.base, jac.frame, angular, linear)
    end
end

for ForceSpaceElement in (:Momentum, :Wrench)
    # MomentumMatrix * velocity vector --> Momentum
    # MomentumMatrix * acceleration vector --> Wrench
    @eval function $ForceSpaceElement(mat::MomentumMatrix, x::AbstractVector)
        angular = convert(SVector{3}, _mul(mat.angular, x))
        linear = convert(SVector{3}, _mul(mat.linear, x))
        $ForceSpaceElement(mat.frame, angular, linear)
    end
end

function (*)(inertia::SpatialInertia, twist::Twist)
    framecheck(inertia.frame, twist.frame)
    Momentum(inertia.frame, mul_inertia(inertia.moment, inertia.crossPart, inertia.mass, twist.angular, twist.linear)...)
end

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

function torque(jac::GeometricJacobian, wrench::Wrench)
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
