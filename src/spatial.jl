# Types

"""
$(TYPEDEF)

A spatial inertia, or inertia matrix, represents the mass distribution of a
rigid body.

A spatial inertia expressed in frame ``i`` is defined as:
```math
I^i =
\\int_{B}\\rho\\left(x\\right)\\left[\\begin{array}{cc}
\\hat{p}^{T}\\left(x\\right)\\hat{p}\\left(x\\right) & \\hat{p}\\left(x\\right)\\\\
\\hat{p}^{T}\\left(x\\right) & I
\\end{array}\\right]dx=\\left[\\begin{array}{cc}
J & \\hat{c}\\\\
\\hat{c}^{T} & mI
\\end{array}\\right]
```
where ``\\rho(x)`` is the density of point ``x``, and ``p(x)`` are the coordinates
of point ``x`` expressed in frame ``i``.
``J`` is the mass moment of inertia, ``m`` is the total mass, and ``c`` is the
'cross part', center of mass position scaled by ``m``.
"""
struct SpatialInertia{T<:Number}
    frame::CartesianFrame3D
    moment::SMatrix{3, 3, T, 9}
    crossPart::SVector{3, T} # mass times center of mass
    mass::T
end

for MotionSpaceElement in (:Twist, :SpatialAcceleration)
    @eval struct $MotionSpaceElement{T<:Number}
        # describes motion of body w.r.t. base, expressed in frame
        body::CartesianFrame3D
        base::CartesianFrame3D
        frame::CartesianFrame3D
        angular::SVector{3, T}
        linear::SVector{3, T}
    end
end

"""
$(TYPEDEF)

A twist represents the relative angular and linear motion between two bodies.

The twist of frame ``j`` with respect to frame ``i``, expressed in frame ``k``
is defined as
```math
T_{j}^{k,i}=\\left(\\begin{array}{c}
\\omega_{j}^{k,i}\\\\
v_{j}^{k,i}
\\end{array}\\right)\\in\\mathbb{R}^{6}
```
such that
```math
\\left[\\begin{array}{cc}
\\hat{\\omega}_{j}^{k,i} & v_{j}^{k,i}\\\\
0 & 0
\\end{array}\\right]=H_{i}^{k}\\dot{H}_{j}^{i}H_{k}^{j}
```
where ``H^{\\beta}_{\\alpha}`` is the homogeneous transform from frame
``\\alpha`` to frame ``\\beta``, and ``\\hat{x}`` is the ``3 \\times 3`` skew
symmetric matrix that satisfies ``\\hat{x} y = x \\times y`` for all
``y \\in \\mathbb{R}^3``.

Here, ``\\omega_{j}^{k,i}`` is the angular part and ``v_{j}^{k,i}`` is the
linear part. Note that the linear part is not in general the same as the
linear velocity of the origin of frame ``j``.
"""
Twist

"""
$(TYPEDEF)

A spatial acceleration is the time derivative of a twist.

See [`Twist`](@ref).
"""
SpatialAcceleration

for ForceSpaceElement in (:Momentum, :Wrench)
    @eval struct $ForceSpaceElement{T<:Number}
        frame::CartesianFrame3D
        angular::SVector{3, T}
        linear::SVector{3, T}
    end
end

"""
$(TYPEDEF)

A `Momentum` is the product of a `SpatialInertia` and a `Twist`, i.e.
```math
h^i =
\\left(\\begin{array}{c}
k^{i}\\\\
l^{i}
\\end{array}\\right) =
I^i T^{i, j}_k
```
where ``I^i`` is the spatial inertia of a given body expressed in frame ``i``,
and ``T^{i, j}_k`` is the twist of frame ``k`` (attached to the body) with
respect to inertial frame ``j``, expressed in frame ``i``. ``k^i`` is the
angular momentum and ``l^i`` is the linear momentum.
"""
Momentum

"""
$(TYPEDEF)

A wrench represents a system of forces.

The wrench ``w^i`` expressed in frame ``i`` is defined as
```math
w^{i} =
\\left(\\begin{array}{c}
\\tau^{i}\\\\
f^{i}
\\end{array}\\right) =
\\sum_{j}\\left(\\begin{array}{c}
r_{j}^{i}\\times f_{j}^{i}\\\\
f_{j}^{i}
\\end{array}\\right)
```
where the ``f_{j}^{i}`` are forces expressed in frame ``i``, exerted at
positions ``r_{j}^{i}``. ``\\tau^i`` is the total torque and ``f^i`` is the
total force.
"""
Wrench

"""
$(TYPEDEF)

A geometric Jacobian (also known as basic, or spatial Jacobian) maps a vector
of joint velocities to a twist.
"""
struct GeometricJacobian{A<:AbstractMatrix}
    body::CartesianFrame3D
    base::CartesianFrame3D
    frame::CartesianFrame3D
    angular::A
    linear::A

    function GeometricJacobian(body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D, angular::A, linear::A) where {A<:AbstractMatrix}
        @boundscheck size(angular, 1) == 3 || error("size mismatch")
        @boundscheck size(linear, 1) == 3 || error("size mismatch")
        @boundscheck size(angular, 2) == size(linear, 2) || error("size mismatch")
        new{A}(body, base, frame, angular, linear)
    end
end

for ForceSpaceMatrix in (:MomentumMatrix, :WrenchMatrix)
    @eval struct $ForceSpaceMatrix{A<:AbstractMatrix}
        frame::CartesianFrame3D
        angular::A
        linear::A

        function $ForceSpaceMatrix(frame::CartesianFrame3D, angular::A, linear::A) where {A<:AbstractMatrix}
            @boundscheck size(angular, 1) == 3 || error("size mismatch")
            @boundscheck size(linear, 1) == 3 || error("size mismatch")
            @boundscheck size(angular, 2) == size(linear, 2) || error("size mismatch")
            new{A}(frame, angular, linear)
        end
    end
end

"""
$(TYPEDEF)

A momentum matrix maps a joint velocity vector to momentum.

This is a slight generalization of the centroidal momentum matrix
(Orin, Goswami, "Centroidal momentum matrix of a humanoid robot: Structure and properties.")
in that the matrix (and hence the corresponding total momentum) need not be
expressed in a centroidal frame.
"""
MomentumMatrix

# SpatialInertia-specific functions
Base.eltype(::Type{SpatialInertia{T}}) where {T} = T
@inline Base.convert(::SpatialInertia{T}, inertia::SpatialInertia{T}) where {T<:Number} = inertia
@inline function Base.convert(::Type{SpatialInertia{T}}, inertia::SpatialInertia) where {T<:Number}
    SpatialInertia(inertia.frame, convert(SMatrix{3, 3, T}, inertia.moment), convert(SVector{3, T}, inertia.crossPart), convert(T, inertia.mass))
end
function Base.convert(::Type{SMatrix{6, 6, T}}, inertia::SpatialInertia) where {T}
    J = inertia.moment
    C = hat(inertia.crossPart)
    m = inertia.mass
    [J  C; C' m * eye(SMatrix{3, 3, T})]
end
Base.convert(::Type{T}, inertia::SpatialInertia) where {T<:Matrix} = convert(T, convert(SMatrix{6, 6, eltype(T)}, inertia))

Base.Array(inertia::SpatialInertia{T}) where {T} = convert(Matrix{T}, inertia)

"""
$(SIGNATURES)

Return the center of mass of the `SpatialInertia` as a [`Point3D`](@ref).
"""
center_of_mass(inertia::SpatialInertia) = Point3D(inertia.frame, inertia.crossPart / inertia.mass)

function Base.show(io::IO, inertia::SpatialInertia)
    println(io, "SpatialInertia expressed in \"$(name(inertia.frame))\":")
    println(io, "mass: $(inertia.mass)")
    println(io, "center of mass: $(center_of_mass(inertia))")
    print(io, "moment of inertia:\n$(inertia.moment)")
end

Base.zero(::Type{SpatialInertia{T}}, frame::CartesianFrame3D) where {T} = SpatialInertia(frame, zeros(SMatrix{3, 3, T}), zeros(SVector{3, T}), zero(T))
Base.zero(inertia::SpatialInertia) = zero(typeof(inertia), inertia.frame)

function Base.isapprox(x::SpatialInertia, y::SpatialInertia; atol = 1e-12)
    x.frame == y.frame && isapprox(x.moment, y.moment; atol = atol) && isapprox(x.crossPart, y.crossPart; atol = atol) && isapprox(x.mass, y.mass; atol = atol)
end

function (+)(inertia1::SpatialInertia, inertia2::SpatialInertia)
    @framecheck(inertia1.frame, inertia2.frame)
    moment = inertia1.moment + inertia2.moment
    crossPart = inertia1.crossPart + inertia2.crossPart
    mass = inertia1.mass + inertia2.mass
    SpatialInertia(inertia1.frame, moment, crossPart, mass)
end

"""
$(SIGNATURES)

Transform the `SpatialInertia` to a different frame.
"""
function transform(inertia::SpatialInertia, t::Transform3D)
    @framecheck(t.from, inertia.frame)
    T = promote_type(eltype(typeof(inertia)), eltype(typeof(t)))

    if t.from == t.to
        return convert(SpatialInertia{T}, inertia)
    elseif inertia.mass == 0
        return zero(SpatialInertia{T}, t.to)
    else
        J = inertia.moment
        m = inertia.mass
        c = inertia.crossPart

        R = rotation(t)
        p = translation(t)

        cnew = R * c
        Jnew = hat_squared(cnew)
        cnew += m * p
        Jnew -= hat_squared(cnew)
        mInv = inv(m)
        Jnew *= mInv
        Jnew += R * J * R'
        return SpatialInertia{T}(t.to, Jnew, cnew, m)
    end
end

function Random.rand(::Type{<:SpatialInertia{T}}, frame::CartesianFrame3D) where {T}
    # Try to generate a random but physical moment of inertia
    # by constructing it from its eigendecomposition
    Q = rand(RotMatrix3{T}).mat
    principalMoments = Vector{T}(3)

    # Scale the inertias to make the length scale of the
    # equivalent inertial ellipsoid roughly ~1 unit
    principalMoments[1:2] = rand(T, 2) / 10.

    # Ensure that the principal moments of inertia obey the triangle
    # inequalities:
    # http://www.mathworks.com/help/physmod/sm/mech/vis/about-body-color-and-geometry.html
    lb = abs(principalMoments[1] - principalMoments[2])
    ub = principalMoments[1] + principalMoments[2]
    principalMoments[3] = rand(T) * (ub - lb) + lb

    # Construct the moment of inertia tensor
    J = SMatrix{3, 3, T}(Q * diagm(principalMoments) * Q')

    # Construct the inertia in CoM frame
    comFrame = CartesianFrame3D()
    spatialInertia = SpatialInertia(comFrame, J, zeros(SVector{3, T}), rand(T))

    # Put the center of mass at a random offset
    comFrameToDesiredFrame = Transform3D(comFrame, frame, rand(SVector{3, T}) - T(0.5))
    transform(spatialInertia, comFrameToDesiredFrame)
end


# MotionSpaceElement-specific
for MotionSpaceElement in (:Twist, :SpatialAcceleration)
    @eval begin
        Base.convert(::Type{$MotionSpaceElement{T}}, m::$MotionSpaceElement{T}) where {T<:Number} = m
        function Base.convert(::Type{$MotionSpaceElement{T}}, m::$MotionSpaceElement) where {T<:Number}
            $MotionSpaceElement(m.body, m.base, m.frame, convert(SVector{3, T}, m.angular), convert(SVector{3, T}, m.linear))
        end
        Base.convert(::Type{Vector{T}}, m::$MotionSpaceElement{T}) where {T} = [m.angular...; m.linear...]
        Base.Array(m::$MotionSpaceElement{T}) where {T} = convert(Vector{T}, m)
        Base.eltype(::Type{$MotionSpaceElement{T}}) where {T} = T
        StaticArrays.similar_type(::Type{$MotionSpaceElement{T1}}, ::Type{T2}) where {T1, T2} = $MotionSpaceElement{T2} # FIXME: lose this

        function Base.show(io::IO, m::$MotionSpaceElement)
            print(io, "$($(string(MotionSpaceElement))) of \"$(name(m.body))\" w.r.t \"$(name(m.base))\" in \"$(name(m.frame))\":\nangular: $(m.angular), linear: $(m.linear)")
        end

        function Base.isapprox(x::$MotionSpaceElement, y::$MotionSpaceElement; atol = 1e-12)
            x.body == y.body && x.base == y.base && x.frame == y.frame && isapprox(x.angular, y.angular; atol = atol) && isapprox(x.linear, y.linear; atol = atol)
        end

        function (+)(m1::$MotionSpaceElement, m2::$MotionSpaceElement)
            @framecheck(m1.frame, m2.frame)
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

        function Base.zero(::Type{$MotionSpaceElement{T}}, body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D) where {T}
            $MotionSpaceElement(body, base, frame, zeros(SVector{3, T}), zeros(SVector{3, T}))
        end

        Base.zero(m::$MotionSpaceElement) = zero(typeof(m), m.body, m.base, m.frame)

        function Random.rand(::Type{$MotionSpaceElement{T}}, body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D) where {T}
            $MotionSpaceElement(body, base, frame, rand(SVector{3, T}), rand(SVector{3, T}))
        end
    end
end

# Twist-specific functions

"""
$(SIGNATURES)

Transform the `Twist` to a different frame.
"""
function transform(twist::Twist, transform::Transform3D)
    @framecheck(twist.frame, transform.from)
    angular, linear = transform_spatial_motion(twist.angular, twist.linear, rotation(transform), translation(transform))
    Twist(twist.body, twist.base, transform.to, angular, linear)
end

# log(::Transform3D) + some extra outputs that make log_with_time_derivative faster
function _log(t::Transform3D)
    # Proposition 2.9 in Murray et al, "A mathematical introduction to robotic manipulation."
    rot = rotation(t)
    p = translation(t)

    # Rotational part of local coordinates is simply the rotation vector.
    aa = AngleAxis(rot)
    θ, axis = rotation_angle(aa), rotation_axis(aa)
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

"""
$(SIGNATURES)

Express a homogeneous transform in exponential coordinates centered around the
identity.
"""
function Base.log(t::Transform3D)
    first(_log(t))
end


"""
$(SIGNATURES)

Compute exponential coordinates as well as their time derivatives in one shot.
This mainly exists because ForwardDiff won't work at the singularity of `log`.
It is also ~50% faster than ForwardDiff in this case.
"""
function log_with_time_derivative(t::Transform3D, twist::Twist)
    # See Bullo and Murray, "Proportional derivative (PD) control on the Euclidean group.", Lemma 4.
    # This is truely magic.
    # Notation matches Bullo and Murray.

    @framecheck(twist.body, t.from)
    @framecheck(twist.base, t.to)
    @framecheck(twist.frame, twist.body) # required by Lemma 4.

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

"""
$(SIGNATURES)

Convert exponential coordinates to a homogeneous transform.
"""
function Base.exp(twist::Twist)
    # See Murray et al, "A mathematical introduction to robotic manipulation."
    @framecheck(twist.frame, twist.base) # twist in base frame; see section 4.3
    ϕrot = twist.angular
    ϕtrans = twist.linear
    θ = norm(ϕrot)
    if abs(angle_difference(θ, zero(θ))) < eps(θ)
        # (2.32)
        rot = eye(RotMatrix3{typeof(θ)})
        trans = ϕtrans
    else
        # (2.36)
        ω = ϕrot / θ
        rot = RotMatrix(AngleAxis(θ, ω[1], ω[2], ω[3]))
        v = ϕtrans / θ
        trans = ω × v
        trans -= rot * trans
        trans += ω * dot(ω, v) * θ
    end
    Transform3D(twist.body, twist.base, rot, trans)
end

function Base.cross(twist1::Twist, twist2::Twist)
    @framecheck(twist1.frame, twist2.frame)
    angular, linear = se3_commutator(twist1.angular, twist1.linear, twist2.angular, twist2.linear)
    SpatialAcceleration(twist2.body, twist2.base, twist2.frame, angular, linear)
end

# SpatialAcceleration-specific functions
"""
$(SIGNATURES)

Transform the `SpatialAcceleration` to a different frame.

The transformation rule is obtained by differentiating the transformation rule
for twists.
"""
function transform(accel::SpatialAcceleration, oldToNew::Transform3D, twistOfCurrentWrtNew::Twist, twistOfBodyWrtBase::Twist)
    # trivial case
    accel.frame == oldToNew.to && return accel

    # frame checks
    @framecheck(oldToNew.from, accel.frame)
    @framecheck(twistOfCurrentWrtNew.frame, accel.frame)
    @framecheck(twistOfCurrentWrtNew.body, accel.frame)
    @framecheck(twistOfCurrentWrtNew.base, oldToNew.to)
    @framecheck(twistOfBodyWrtBase.frame, accel.frame)
    @framecheck(twistOfBodyWrtBase.body, accel.body)
    @framecheck(twistOfBodyWrtBase.base, accel.base)

    # 'cross term':
    angular, linear = se3_commutator(
        twistOfCurrentWrtNew.angular, twistOfCurrentWrtNew.linear,
        twistOfBodyWrtBase.angular, twistOfBodyWrtBase.linear)

    # add current acceleration:
    angular += accel.angular
    linear += accel.linear

    # transform to new frame
    angular, linear = transform_spatial_motion(angular, linear, rotation(oldToNew), translation(oldToNew))

    SpatialAcceleration(accel.body, accel.base, oldToNew.to, angular, linear)
end


# ForceSpaceElement-specific
for ForceSpaceElement in (:Momentum, :Wrench)
    @eval begin
        """
        $(SIGNATURES)

        Create a $($(string(ForceSpaceElement))) given the angular and linear
        components, which should be expressed in the same frame.
        """
        function $ForceSpaceElement(angular::FreeVector3D, linear::FreeVector3D)
            @framecheck angular.frame linear.frame
            $ForceSpaceElement(angular.frame, angular.v, linear.v)
        end

        Base.convert(::Type{$ForceSpaceElement{T}}, f::$ForceSpaceElement{T}) where {T<:Number} = f

        function Base.convert(::Type{$ForceSpaceElement{T}}, f::$ForceSpaceElement) where {T<:Number}
            $ForceSpaceElement(f.frame, convert(SVector{3, T}, f.angular), convert(SVector{3, T}, f.linear))
        end

        Base.eltype(::Type{$ForceSpaceElement{T}}) where {T} = T
        StaticArrays.similar_type(::Type{$ForceSpaceElement{T1}}, ::Type{T2}) where {T1, T2} = $ForceSpaceElement{T2} # FIXME: lose this

        function Base.show(io::IO, f::$ForceSpaceElement)
            print(io, "$($(string(ForceSpaceElement))) expressed in \"$(name(f.frame))\":\nangular: $(f.angular), linear: $(f.linear)")
        end

        function Base.zero(::Type{$ForceSpaceElement{T}}, frame::CartesianFrame3D) where {T}
            $ForceSpaceElement(frame, zeros(SVector{3, T}), zeros(SVector{3, T}))
        end

        Base.zero(f::$ForceSpaceElement) = zero(typeof(f), f.frame)

        function Random.rand(::Type{$ForceSpaceElement{T}}, frame::CartesianFrame3D) where {T}
            $ForceSpaceElement(frame, rand(SVector{3, T}), rand(SVector{3, T}))
        end

        """
        $(SIGNATURES)

        Transform the $($(string(ForceSpaceElement))) to a different frame.
        """
        function transform(f::$ForceSpaceElement, transform::Transform3D)
            @framecheck(f.frame, transform.from)
            rot = rotation(transform)
            linear = rot * f.linear
            angular = rot * f.angular + cross(translation(transform), linear)
            $ForceSpaceElement(transform.to, angular, linear)
        end

        function (+)(f1::$ForceSpaceElement, f2::$ForceSpaceElement)
            @framecheck(f1.frame, f2.frame)
            $ForceSpaceElement(f1.frame, f1.angular + f2.angular, f1.linear + f2.linear)
        end

        function (-)(f1::$ForceSpaceElement, f2::$ForceSpaceElement)
            @framecheck(f1.frame, f2.frame)
            $ForceSpaceElement(f1.frame, f1.angular - f2.angular, f1.linear - f2.linear)
        end

        (-)(f::$ForceSpaceElement) = $ForceSpaceElement(f.frame, -f.angular, -f.linear)

        Base.Array(f::$ForceSpaceElement) = [f.angular...; f.linear...]
        Base.isapprox(x::$ForceSpaceElement, y::$ForceSpaceElement; atol = 1e-12) = x.frame == y.frame && isapprox(x.angular, y.angular, atol = atol) && isapprox(x.linear, y.linear, atol = atol)
    end
end

# Wrench-specific functions
"""
$(SIGNATURES)

Create a Wrench from a force, ``f``, and the application point of the force, ``r``.
The torque part of the wrench will be computed as ``r \\times f``. The force
and the application point should be expressed in the same frame.
"""
Wrench(application_point::Point3D, force::FreeVector3D) = Wrench(application_point × force, force)

# WrenchSubspace is the return type of e.g. constraint_wrench_subspace(::Joint, ...)
const WrenchSubspace{T} = WrenchMatrix{ContiguousSMatrixColumnView{3, 6, T, 18}}
function WrenchSubspace(frame::CartesianFrame3D, angular, linear)
    WrenchMatrix(frame, smatrix3x6view(angular), smatrix3x6view(linear))
end

# MotionSubspace is the return type of motion_subspace(::Joint, ...)
const MotionSubspace{T} = GeometricJacobian{ContiguousSMatrixColumnView{3, 6, T, 18}}
function MotionSubspace(body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D, angular, linear)
    GeometricJacobian(body, base, frame, smatrix3x6view(angular), smatrix3x6view(linear))
end

# GeometricJacobian-specific functions
Base.convert(::Type{GeometricJacobian{A}}, jac::GeometricJacobian{A}) where {A} = jac
function Base.convert(::Type{GeometricJacobian{A}}, jac::GeometricJacobian) where {A}
    GeometricJacobian(jac.body, jac.base, jac.frame, convert(A, jac.angular), convert(A, jac.linear))
end
Base.Array(jac::GeometricJacobian) = [Array(jac.angular); Array(jac.linear)]
Base.eltype(::Type{GeometricJacobian{A}}) where {A} = eltype(A)

"""
$(SIGNATURES)

Return the number of columns of the `GeometricJacobian`.
"""
num_cols(jac::GeometricJacobian) = size(jac.angular, 2)
angular_part(jac::GeometricJacobian) = jac.angular
linear_part(jac::GeometricJacobian) = jac.linear
change_base(jac::GeometricJacobian, base::CartesianFrame3D) = GeometricJacobian(jac.body, base, jac.frame, jac.angular, jac.linear)

(-)(jac::GeometricJacobian) = GeometricJacobian(jac.base, jac.body, jac.frame, -jac.angular, -jac.linear)

function Base.show(io::IO, jac::GeometricJacobian)
    print(io, "GeometricJacobian: body: \"$(name(jac.body))\", base: \"$(name(jac.base))\", expressed in \"$(name(jac.frame))\":\n$(Array(jac))")
end

"""
$(SIGNATURES)

Transform the `GeometricJacobian` to a different frame.
"""
function transform(jac::GeometricJacobian, transform::Transform3D)
    @framecheck(jac.frame, transform.from)
    R = rotation(transform)
    angular = R * jac.angular
    linear = R * jac.linear + colwise(cross, translation(transform), angular)
    GeometricJacobian(jac.body, jac.base, transform.to, angular, linear)
end


# Force space matrix-specific functions
for ForceSpaceMatrix in (:MomentumMatrix, :WrenchMatrix)
    @eval begin
        Base.convert(::Type{$ForceSpaceMatrix{A}}, mat::$ForceSpaceMatrix{A}) where {A} = mat
        function Base.convert(::Type{$ForceSpaceMatrix{A}}, mat::$ForceSpaceMatrix) where {A}
            $ForceSpaceMatrix(mat.frame, convert(A, mat.angular), convert(A, mat.linear))
        end
        Base.Array(mat::$ForceSpaceMatrix) = [Array(mat.angular); Array(mat.linear)]

        Base.eltype(::Type{$ForceSpaceMatrix{A}}) where {A} = eltype(A)
        num_cols(mat::$ForceSpaceMatrix) = size(mat.angular, 2)
        angular_part(mat::$ForceSpaceMatrix) = mat.angular
        linear_part(mat::$ForceSpaceMatrix) = mat.linear

        function Base.show(io::IO, m::$ForceSpaceMatrix)
            print(io, "$($(string(ForceSpaceMatrix))) expressed in \"$(name(m.frame))\":\n$(Array(m))")
        end

        function Base.hcat(mats::$ForceSpaceMatrix...)
            frame = mats[1].frame
            for j = 2 : length(mats)
                @framecheck(mats[j].frame, frame)
            end
            angular = hcat((m.angular for m in mats)...)
            linear = hcat((m.linear for m in mats)...)
            $ForceSpaceMatrix(frame, angular, linear)
        end

        function transform(mat::$ForceSpaceMatrix, transform::Transform3D)
            @framecheck(mat.frame, transform.from)
            R = rotation(transform)
            linear = R * linear_part(mat)
            T = eltype(linear)
            angular = R * angular_part(mat) + colwise(cross, translation(transform), linear)
            $ForceSpaceMatrix(transform.to, angular, linear)
        end
    end
end


# Interactions between spatial types

"""
$(SIGNATURES)

Compute the mechanical power associated with a pairing of a wrench and a twist.
"""
Base.dot(w::Wrench, t::Twist) = begin @framecheck(w.frame, t.frame); dot(w.angular, t.angular) + dot(w.linear, t.linear) end
Base.dot(t::Twist, w::Wrench) = dot(w, t)

for MotionSpaceElement in (:Twist, :SpatialAcceleration)
    # GeometricJacobian * velocity vector --> Twist
    # GeometricJacobian * acceleration vector --> SpatialAcceleration
    @eval function $MotionSpaceElement(jac::GeometricJacobian, x::AbstractVector)
        angular = convert(SVector{3}, _mul(jac.angular, x))
        linear = convert(SVector{3}, _mul(jac.linear, x))
        $MotionSpaceElement(jac.body, jac.base, jac.frame, angular, linear)
    end
end

for (ForceSpaceMatrix, ForceSpaceElement) in (:MomentumMatrix => :Momentum, :MomentumMatrix => :Wrench, :WrenchMatrix => :Wrench)
    # MomentumMatrix * velocity vector --> Momentum
    # MomentumMatrix * acceleration vector --> Wrench
    # WrenchMatrix * dimensionless multipliers --> Wrench
    @eval function $ForceSpaceElement(mat::$ForceSpaceMatrix, x::AbstractVector)
        angular = convert(SVector{3}, _mul(mat.angular, x))
        linear = convert(SVector{3}, _mul(mat.linear, x))
        $ForceSpaceElement(mat.frame, angular, linear)
    end
end

function (*)(inertia::SpatialInertia, twist::Twist)
    @framecheck(inertia.frame, twist.frame)
    angular, linear = mul_inertia(inertia.moment, inertia.crossPart, inertia.mass, twist.angular, twist.linear)
    Momentum(inertia.frame, angular, linear)
end

function (*)(inertia::SpatialInertia, jac::GeometricJacobian)
    @framecheck(inertia.frame, jac.frame)
    Jω = jac.angular
    Jv = jac.linear
    J = inertia.moment
    m = inertia.mass
    c = inertia.crossPart
    angular = J * Jω + colwise(cross, c, Jv)
    linear = m * Jv - colwise(cross, c, Jω)
    MomentumMatrix(inertia.frame, angular, linear)
end

"""
$(SIGNATURES)

Apply the Newton-Euler equations to find the external wrench required to
make a body with spatial inertia ``I``, which has twist ``T`` with respect
to an inertial frame, achieve spatial acceleration ``\\dot{T}``.

This wrench is also equal to the rate of change of momentum of the body.
"""
function newton_euler(I::SpatialInertia, Ṫ::SpatialAcceleration, T::Twist)
    body = Ṫ.body
    base = Ṫ.base # TODO: should assert that this is an inertial frame somehow
    frame = Ṫ.frame

    @framecheck(I.frame, frame)
    @framecheck(T.body, body)
    @framecheck(T.base, base)
    @framecheck(T.frame, frame)

    angular, linear = mul_inertia(I.moment, I.crossPart, I.mass, Ṫ.angular, Ṫ.linear)
    angularMomentum, linearMomentum = mul_inertia(I.moment, I.crossPart, I.mass, T.angular, T.linear)
    angular += cross(T.angular, angularMomentum) + cross(T.linear, linearMomentum)
    linear += cross(T.angular, linearMomentum)
    Wrench(frame, angular, linear)
end

torque!(τ::AbstractVector, jac::GeometricJacobian, wrench::Wrench) = At_mul_B!(τ, jac, wrench)

function torque(jac::GeometricJacobian, wrench::Wrench)
    τ = Vector{promote_type(eltype(jac), eltype(wrench))}(num_cols(jac))
    torque!(τ, jac, wrench)
    τ
end

for (MatrixType, VectorType) in (:WrenchMatrix => :(Union{Twist, SpatialAcceleration}), :GeometricJacobian => :(Union{Momentum, Wrench}))
    @eval function Base.At_mul_B!(x::AbstractVector, mat::$MatrixType, vec::$VectorType)
        @boundscheck length(x) == num_cols(mat) || error("size mismatch")
        @framecheck mat.frame vec.frame
        @simd for row in eachindex(x)
            @inbounds begin
                x[row] =
                mat.angular[1, row] * vec.angular[1] +
                mat.angular[2, row] * vec.angular[2] +
                mat.angular[3, row] * vec.angular[3] +
                mat.linear[1, row] * vec.linear[1] +
                mat.linear[2, row] * vec.linear[2] +
                mat.linear[3, row] * vec.linear[3]
            end
        end
    end
end

"""
$(SIGNATURES)

Compute the kinetic energy of a body with spatial inertia ``I``, which has
twist ``T`` with respect to an inertial frame.
"""
function kinetic_energy(I::SpatialInertia, twist::Twist)
    @framecheck(I.frame, twist.frame)
    # TODO: should assert that twist.base is an inertial frame somehow
    ω = twist.angular
    v = twist.linear
    J = I.moment
    c = I.crossPart
    m = I.mass
    (dot(ω, J * ω) + dot(v, m * v + 2 * cross(ω, c))) / 2
end

"""
$(SIGNATURES)

Given a twist ``T_{j}^{k,i}`` of frame ``j`` with respect to frame ``i``, expressed in frame ``k``,
and the location of a point fixed in frame ``j``, also expressed in frame ``k``, compute the velocity
of the point relative to frame ``i``.
"""
function point_velocity(twist::Twist, point::Point3D)
    @framecheck twist.frame point.frame
    FreeVector3D(twist.frame, twist.angular × point.v + twist.linear)
end
