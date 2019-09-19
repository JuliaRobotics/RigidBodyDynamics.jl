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

    @inline function GeometricJacobian{A}(
            body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D,
            angular::AbstractMatrix, linear::AbstractMatrix) where A<:AbstractMatrix
        @boundscheck size(angular, 1) == 3 || throw(DimensionMismatch())
        @boundscheck size(linear, 1) == 3 || throw(DimensionMismatch())
        @boundscheck size(angular, 2) == size(linear, 2) || throw(DimensionMismatch())
        new{A}(body, base, frame, angular, linear)
    end
end

# GeometricJacobian-specific functions
@inline function GeometricJacobian(
        body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D,
        angular::A, linear::A) where {A<:AbstractMatrix}
    GeometricJacobian{A}(body, base, frame, angular, linear)
end

@inline function GeometricJacobian(
        body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D,
        angular::A1, linear::A2) where {A1<:AbstractMatrix, A2<:AbstractMatrix}
    GeometricJacobian(body, base, frame, promote(angular, linear)...)
end

@inline function GeometricJacobian{A}(jac::GeometricJacobian) where A
    GeometricJacobian(jac.body, jac.base, jac.frame, A(angular(jac)), A(linear(jac)))
end

change_base(jac::GeometricJacobian, base::CartesianFrame3D) = GeometricJacobian(jac.body, base, jac.frame, angular(jac), linear(jac))

Base.:-(jac::GeometricJacobian) = GeometricJacobian(jac.base, jac.body, jac.frame, -angular(jac), -linear(jac))

function Base.show(io::IO, jac::GeometricJacobian)
    print(io, "GeometricJacobian: body: \"$(string(jac.body))\", base: \"$(string(jac.base))\", expressed in \"$(string(jac.frame))\":\n$(Array(jac))")
end

"""
$(SIGNATURES)

Transform the `GeometricJacobian` to a different frame.
"""
@inline function transform(jac::GeometricJacobian, tf::Transform3D)
    @framecheck(jac.frame, tf.from)
    R = rotation(tf)
    ang = R * angular(jac)
    lin = R * linear(jac) + colwise(×, translation(tf), ang)
    GeometricJacobian(jac.body, jac.base, tf.to, ang, lin)
end

struct PointJacobian{M <: AbstractMatrix}
    frame::CartesianFrame3D
    J::M
end

Base.@deprecate PointJacobian{M}(J::M, frame::CartesianFrame3D) where {M<:AbstractMatrix} PointJacobian(frame, J)
Base.@deprecate PointJacobian(J::AbstractMatrix, frame::CartesianFrame3D) PointJacobian(frame, J)

# Construct/convert to Matrix
(::Type{A})(jac::PointJacobian) where {A<:Array} = A(jac.J)
Base.convert(::Type{A}, jac::PointJacobian) where {A<:Array} = A(jac)
Base.eltype(::Type{PointJacobian{M}}) where {M} = eltype(M)

Base.transpose(jac::PointJacobian) = Transpose(jac)

function point_velocity(jac::PointJacobian, v::AbstractVector)
    FreeVector3D(jac.frame, jac.J * v)
end

function LinearAlgebra.mul!(τ::AbstractVector, jac_transpose::Transpose{<:Any, <:PointJacobian}, force::FreeVector3D)
    jac = parent(jac_transpose)
    @framecheck jac.frame force.frame
    mul!(τ, transpose(jac.J), force.v)
end

function Base.:*(jac_transpose::Transpose{<:Any, <:PointJacobian}, force::FreeVector3D)
    jac = parent(jac_transpose)
    @framecheck jac.frame force.frame
    transpose(jac.J) * force.v
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
struct Twist{T}
    body::CartesianFrame3D
    base::CartesianFrame3D
    frame::CartesianFrame3D
    angular::SVector{3, T}
    linear::SVector{3, T}

    @inline function Twist{T}(body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D,
            angular::AbstractVector, linear::AbstractVector) where T
        new{T}(body, base, frame, angular, linear)
    end
end

"""
$(TYPEDEF)

A spatial acceleration is the time derivative of a twist.

See [`Twist`](@ref).
"""
struct SpatialAcceleration{T}
    body::CartesianFrame3D
    base::CartesianFrame3D
    frame::CartesianFrame3D
    angular::SVector{3, T}
    linear::SVector{3, T}

    @inline function SpatialAcceleration{T}(body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D,
            angular::AbstractVector, linear::AbstractVector) where T
        new{T}(body, base, frame, angular, linear)
    end
end


for MotionSpaceElement in (:Twist, :SpatialAcceleration)
    @eval begin
        # Construct with possibly eltype-heterogeneous inputs
        @inline function $MotionSpaceElement(body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D,
                angular::AbstractVector, linear::AbstractVector)
            T = promote_eltype(angular, linear)
            $MotionSpaceElement{T}(body, base, frame, angular, linear)
        end

        # Construct given FreeVector3Ds
        function $MotionSpaceElement(body::CartesianFrame3D, base::CartesianFrame3D, angular::FreeVector3D, linear::FreeVector3D)
            @framecheck angular.frame linear.frame
            $MotionSpaceElement(body, base, angular.frame, angular.v, linear.v)
        end

        # Construct/convert given another $MotionSpaceElement
        function $MotionSpaceElement{T}(m::$MotionSpaceElement) where T
            $MotionSpaceElement(m.body, m.base, m.frame, SVector{3, T}(angular(m)), SVector{3, T}(linear(m)))
        end

        function Base.show(io::IO, m::$MotionSpaceElement)
            print(io, "$($(string(MotionSpaceElement))) of \"$(string(m.body))\" w.r.t \"$(string(m.base))\" in \"$(string(m.frame))\":\nangular: $(angular(m)), linear: $(linear(m))")
        end

        function Base.isapprox(x::$MotionSpaceElement, y::$MotionSpaceElement; atol = 1e-12)
            x.body == y.body && x.base == y.base && x.frame == y.frame && isapprox(angular(x), angular(y); atol = atol) && isapprox(linear(x), linear(y); atol = atol)
        end

        @inline function Base.:+(m1::$MotionSpaceElement, m2::$MotionSpaceElement)
            @framecheck(m1.frame, m2.frame)
            @boundscheck begin
                ((m1.body == m2.body && m1.base == m2.base) || m1.body == m2.base) || throw(ArgumentError("frame mismatch"))
            end
            $MotionSpaceElement(m2.body, m1.base, m1.frame, angular(m1) + angular(m2), linear(m1) + linear(m2))
        end

        Base.:-(m::$MotionSpaceElement) = $MotionSpaceElement(m.base, m.body, m.frame, -angular(m), -linear(m))

        change_base(m::$MotionSpaceElement, base::CartesianFrame3D) = $MotionSpaceElement(m.body, base, m.frame, angular(m), linear(m))

        function Base.zero(::Type{$MotionSpaceElement{T}}, body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D) where {T}
            $MotionSpaceElement(body, base, frame, zero(SVector{3, T}), zero(SVector{3, T}))
        end

        Base.zero(m::$MotionSpaceElement) = zero(typeof(m), m.body, m.base, m.frame)

        function Random.rand(::Type{$MotionSpaceElement{T}}, body::CartesianFrame3D, base::CartesianFrame3D, frame::CartesianFrame3D) where {T}
            $MotionSpaceElement(body, base, frame, rand(SVector{3, T}), rand(SVector{3, T}))
        end

        # GeometricJacobian * velocity vector --> Twist
        # GeometricJacobian * acceleration vector --> SpatialAcceleration
        function $MotionSpaceElement(jac::GeometricJacobian, x::AbstractVector)
            $MotionSpaceElement(jac.body, jac.base, jac.frame, convert(SVector{3}, angular(jac) * x), convert(SVector{3}, linear(jac) * x))
        end
    end
end

"""
$(SIGNATURES)

Transform the `Twist` to a different frame.
"""
@inline function transform(twist::Twist, tf::Transform3D)
    @framecheck(twist.frame, tf.from)
    ang, lin = transform_spatial_motion(angular(twist), linear(twist), rotation(tf), translation(tf))
    Twist(twist.body, twist.base, tf.to, ang, lin)
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
    θ_2 = θ / 2
    sθ_2, cθ_2 = sincos(θ_2)
    θ_squared = θ^2
    if abs(rem2pi(θ, RoundNearest)) < eps(typeof(θ))
        α = one(θ_2)
        ϕtrans = p
    else
        α = θ_2 * cθ_2 / sθ_2
        ϕtrans = p - ϕrot × p / 2 + (1 - α) / θ_squared * ϕrot × (ϕrot × p) # Bullo, Murray, (2.5)
    end

    ξ = Twist(t.from, t.to, t.to, ϕrot, ϕtrans) # twist in base frame; see section 4.3
    ξ, θ, θ_squared, θ_2, sθ_2, cθ_2, α
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

    ψ = angular(X)
    q = linear(X)

    ω = angular(twist)
    v = linear(twist)

    ψ̇ = ω
    q̇ = v
    if abs(rem2pi(θ, RoundNearest)) > eps(typeof(θ))
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
    ϕrot = angular(twist)
    ϕtrans = linear(twist)
    θ = norm(ϕrot)
    if abs(rem2pi(θ, RoundNearest)) < eps(typeof(θ))
        # (2.32)
        rot = one(RotMatrix3{typeof(θ)})
        trans = ϕtrans
    else
        # (2.36)
        ω = ϕrot / θ
        rot = RotMatrix(AngleAxis(θ, ω[1], ω[2], ω[3], false))
        v = ϕtrans / θ
        trans = ω × v
        trans -= rot * trans
        trans += ω * dot(ω, v) * θ
    end
    Transform3D(twist.body, twist.base, rot, trans)
end

@inline function LinearAlgebra.cross(twist1::Twist, twist2::Twist)
    @framecheck(twist1.frame, twist2.frame)
    ang, lin = se3_commutator(angular(twist1), linear(twist1), angular(twist2), linear(twist2))
    SpatialAcceleration(twist2.body, twist2.base, twist2.frame, ang, lin)
end

"""
$(SIGNATURES)

Given the twist ``T_{j}^{k,i}`` of frame ``j`` with respect to frame ``i``, expressed in frame ``k``,
and the location of a point fixed in frame ``j``, also expressed in frame ``k``, compute the velocity
of the point relative to frame ``i``.
"""
function point_velocity(twist::Twist, point::Point3D)
    @framecheck twist.frame point.frame
    FreeVector3D(twist.frame, angular(twist) × point.v + linear(twist))
end

"""
$(SIGNATURES)

Given the twist ``dot{T}_{j}^{k,i}`` of frame ``j`` with respect to frame ``i``, expressed in frame ``k``
and its time derivative (a spatial acceleration), as well as the location of a point fixed in frame ``j``,
also expressed in frame ``k``, compute the acceleration of the point relative to frame ``i``.
"""
function point_acceleration(twist::Twist, accel::SpatialAcceleration, point::Point3D)
    @framecheck twist.base accel.base
    @framecheck twist.body accel.body
    @framecheck twist.frame accel.frame
    FreeVector3D(accel.frame, angular(accel) × point.v + linear(accel) + angular(twist) × point_velocity(twist, point).v)
end


# SpatialAcceleration-specific functions
"""
$(SIGNATURES)

Transform the `SpatialAcceleration` to a different frame.

The transformation rule is obtained by differentiating the transformation rule
for twists.
"""
function transform(accel::SpatialAcceleration, old_to_new::Transform3D, twist_of_current_wrt_new::Twist, twist_of_body_wrt_base::Twist)
    # trivial case
    accel.frame == old_to_new.to && return accel

    # frame checks
    @framecheck(old_to_new.from, accel.frame)
    @framecheck(twist_of_current_wrt_new.frame, accel.frame)
    @framecheck(twist_of_current_wrt_new.body, accel.frame)
    @framecheck(twist_of_current_wrt_new.base, old_to_new.to)
    @framecheck(twist_of_body_wrt_base.frame, accel.frame)
    @framecheck(twist_of_body_wrt_base.body, accel.body)
    @framecheck(twist_of_body_wrt_base.base, accel.base)

    # 'cross term':
    ang, lin = se3_commutator(
        angular(twist_of_current_wrt_new), linear(twist_of_current_wrt_new),
        angular(twist_of_body_wrt_base), linear(twist_of_body_wrt_base))

    # add current acceleration:
    ang += angular(accel)
    lin += linear(accel)

    # transform to new frame
    ang, lin = transform_spatial_motion(ang, lin, rotation(old_to_new), translation(old_to_new))

    SpatialAcceleration(accel.body, accel.base, old_to_new.to, ang, lin)
end
