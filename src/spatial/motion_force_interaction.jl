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
struct SpatialInertia{T}
    frame::CartesianFrame3D
    moment::SMatrix{3, 3, T, 9}
    cross_part::SVector{3, T} # mass times center of mass
    mass::T

    @inline function SpatialInertia(frame::CartesianFrame3D, moment::AbstractMatrix{T}, cross_part::AbstractVector{T}, mass::T) where T
        @boundscheck size(moment) == (3, 3) || error("size mismatch")
        @boundscheck size(cross_part) == (3,) || error("size mismatch")
        new{T}(frame, moment, cross_part, mass)
    end
end

# SpatialInertia-specific functions
Base.eltype(::Type{SpatialInertia{T}}) where {T} = T
@inline Base.convert(::SpatialInertia{T}, inertia::SpatialInertia{T}) where {T} = inertia
@inline function Base.convert(::Type{SpatialInertia{T}}, inertia::SpatialInertia) where {T}
    SpatialInertia(inertia.frame, convert(SMatrix{3, 3, T}, inertia.moment), convert(SVector{3, T}, inertia.cross_part), convert(T, inertia.mass))
end
function Base.convert(::Type{SMatrix{6, 6, T}}, inertia::SpatialInertia) where {T}
    J = inertia.moment
    C = hat(inertia.cross_part)
    m = inertia.mass
    [J  C; C' m * eye(SMatrix{3, 3, T})]
end
Base.convert(::Type{T}, inertia::SpatialInertia) where {T<:Matrix} = convert(T, convert(SMatrix{6, 6, eltype(T)}, inertia))

Base.Array(inertia::SpatialInertia{T}) where {T} = convert(Matrix{T}, inertia)

"""
$(SIGNATURES)

Return the center of mass of the `SpatialInertia` as a [`Point3D`](@ref).
"""
center_of_mass(inertia::SpatialInertia) = Point3D(inertia.frame, inertia.cross_part / inertia.mass)

function Base.show(io::IO, inertia::SpatialInertia)
    println(io, "SpatialInertia expressed in \"$(string(inertia.frame))\":")
    println(io, "mass: $(inertia.mass)")
    println(io, "center of mass: $(center_of_mass(inertia))")
    print(io, "moment of inertia:\n$(inertia.moment)")
end

Base.zero(::Type{SpatialInertia{T}}, frame::CartesianFrame3D) where {T} = SpatialInertia(frame, zeros(SMatrix{3, 3, T}), zeros(SVector{3, T}), zero(T))
Base.zero(inertia::SpatialInertia) = zero(typeof(inertia), inertia.frame)

function Base.isapprox(x::SpatialInertia, y::SpatialInertia; atol = 1e-12)
    x.frame == y.frame && isapprox(x.moment, y.moment; atol = atol) && isapprox(x.cross_part, y.cross_part; atol = atol) && isapprox(x.mass, y.mass; atol = atol)
end

function Base.:+(inertia1::SpatialInertia, inertia2::SpatialInertia)
    @framecheck(inertia1.frame, inertia2.frame)
    moment = inertia1.moment + inertia2.moment
    cross_part = inertia1.cross_part + inertia2.cross_part
    mass = inertia1.mass + inertia2.mass
    SpatialInertia(inertia1.frame, moment, cross_part, mass)
end

"""
$(SIGNATURES)

Transform the `SpatialInertia` to a different frame.
"""
function transform(inertia::SpatialInertia, t::Transform3D)
    @framecheck(t.from, inertia.frame)
    J = inertia.moment
    m = inertia.mass
    c = inertia.cross_part
    R = rotation(t)
    p = translation(t)
    cnew = R * c
    Jnew = hat_squared(cnew)
    cnew += m * p
    Jnew -= hat_squared(cnew)
    minv = ifelse(m > 0, inv(m), zero(m))
    Jnew *= minv
    Jnew += R * J * R'
    SpatialInertia(t.to, Jnew, cnew, convert(eltype(Jnew), m))
end

function Random.rand(::Type{<:SpatialInertia{T}}, frame::CartesianFrame3D) where {T}
    # Try to generate a random but physical moment of inertia
    # by constructing it from its eigendecomposition
    Q = rand(RotMatrix3{T}).mat
    principal_moments = Vector{T}(3)

    # Scale the inertias to make the length scale of the
    # equivalent inertial ellipsoid roughly ~1 unit
    principal_moments[1:2] = rand(T, 2) / 10.

    # Ensure that the principal moments of inertia obey the triangle
    # inequalities:
    # http://www.mathworks.com/help/physmod/sm/mech/vis/about-body-color-and-geometry.html
    lb = abs(principal_moments[1] - principal_moments[2])
    ub = principal_moments[1] + principal_moments[2]
    principal_moments[3] = rand(T) * (ub - lb) + lb

    # Construct the moment of inertia tensor
    J = SMatrix{3, 3, T}(Q * diagm(principal_moments) * Q')

    # Construct the inertia in CoM frame
    com_frame = CartesianFrame3D()
    spatial_inertia = SpatialInertia(com_frame, J, zeros(SVector{3, T}), rand(T))

    # Put the center of mass at a random offset
    com_frame_to_desired_frame = Transform3D(com_frame, frame, rand(SVector{3, T}) - T(0.5))
    transform(spatial_inertia, com_frame_to_desired_frame)
end

"""
$(SIGNATURES)

Compute the mechanical power associated with a pairing of a wrench and a twist.
"""
Base.dot(w::Wrench, t::Twist) = begin @framecheck(w.frame, t.frame); dot(angular(w), angular(t)) + dot(linear(w), linear(t)) end
Base.dot(t::Twist, w::Wrench) = dot(w, t)


for (ForceSpaceMatrix, ForceSpaceElement) in (:MomentumMatrix => :Momentum, :MomentumMatrix => :Wrench, :WrenchMatrix => :Wrench)
    # MomentumMatrix * velocity vector --> Momentum
    # MomentumMatrix * acceleration vector --> Wrench
    # WrenchMatrix * dimensionless multipliers --> Wrench
    @eval function $ForceSpaceElement(mat::$ForceSpaceMatrix, x::AbstractVector)
        $ForceSpaceElement(mat.frame, convert(SVector{3}, angular(mat) * x), convert(SVector{3}, linear(mat) * x))
    end
end

function Base.:*(inertia::SpatialInertia, twist::Twist)
    @framecheck(inertia.frame, twist.frame)
    ang, lin = mul_inertia(inertia.moment, inertia.cross_part, inertia.mass, angular(twist), linear(twist))
    Momentum(inertia.frame, ang, lin)
end

function Base.:*(inertia::SpatialInertia, jac::GeometricJacobian)
    @framecheck(inertia.frame, jac.frame)
    Jω = angular(jac)
    Jv = linear(jac)
    J = inertia.moment
    m = inertia.mass
    c = inertia.cross_part
    ang = J * Jω + colwise(cross, c, Jv)
    lin = m * Jv - colwise(cross, c, Jω)
    MomentumMatrix(inertia.frame, ang, lin)
end

"""
$(SIGNATURES)

Apply the Newton-Euler equations to find the external wrench required to
make a body with spatial inertia ``I``, which has twist ``T`` with respect
to an inertial frame, achieve spatial acceleration ``\\dot{T}``.

This wrench is also equal to the rate of change of momentum of the body.
"""
function newton_euler(inertia::SpatialInertia, spatial_accel::SpatialAcceleration, twist::Twist)
    I = inertia
    T = twist
    Ṫ = spatial_accel

    body = Ṫ.body
    base = Ṫ.base # TODO: should assert that this is an inertial frame somehow
    frame = Ṫ.frame

    @framecheck(I.frame, frame)
    @framecheck(T.body, body)
    @framecheck(T.base, base)
    @framecheck(T.frame, frame)

    ang, lin = mul_inertia(I.moment, I.cross_part, I.mass, angular(Ṫ), linear(Ṫ))
    angular_momentum, linear_momentum = mul_inertia(I.moment, I.cross_part, I.mass, angular(T), linear(T))
    ang += cross(angular(T), angular_momentum) + cross(linear(T), linear_momentum)
    lin += cross(angular(T), linear_momentum)
    Wrench(frame, ang, lin)
end

torque!(τ::AbstractVector, jac::GeometricJacobian, wrench::Wrench) = At_mul_B!(τ, jac, wrench)

function torque(jac::GeometricJacobian, wrench::Wrench)
    τ = Vector{promote_type(eltype(jac), eltype(wrench))}(size(jac, 2))
    torque!(τ, jac, wrench)
    τ
end

for (MatrixType, VectorType) in (:WrenchMatrix => :(Union{Twist, SpatialAcceleration}), :GeometricJacobian => :(Union{Momentum, Wrench}))
    @eval @inline function Base.At_mul_B!(x::AbstractVector, mat::$MatrixType, vec::$VectorType)
        @boundscheck length(x) == size(mat, 2) || error("size mismatch")
        @framecheck mat.frame vec.frame
        @simd for row in eachindex(x)
            @inbounds begin
                x[row] =
                angular(mat)[1, row] * angular(vec)[1] +
                angular(mat)[2, row] * angular(vec)[2] +
                angular(mat)[3, row] * angular(vec)[3] +
                linear(mat)[1, row] * linear(vec)[1] +
                linear(mat)[2, row] * linear(vec)[2] +
                linear(mat)[3, row] * linear(vec)[3]
            end
        end
    end
end



"""
$(SIGNATURES)

Compute the kinetic energy of a body with spatial inertia ``I``, which has
twist ``T`` with respect to an inertial frame.
"""
function kinetic_energy(inertia::SpatialInertia, twist::Twist)
    @framecheck(inertia.frame, twist.frame)
    # TODO: should assert that twist.base is an inertial frame somehow
    ω = angular(twist)
    v = linear(twist)
    J = inertia.moment
    c = inertia.cross_part
    m = inertia.mass
    (dot(ω, J * ω) + dot(v, m * v + 2 * cross(ω, c))) / 2
end
