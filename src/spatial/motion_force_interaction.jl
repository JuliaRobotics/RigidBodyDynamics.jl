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

!!! warning

    The `moment` field of a `SpatialInertia` is the moment of inertia **about the origin of its `frame`**,
    not about the center of mass.
"""
struct SpatialInertia{T}
    frame::CartesianFrame3D
    moment::SMatrix{3, 3, T, 9}
    cross_part::SVector{3, T} # mass times center of mass
    mass::T

    @inline function SpatialInertia{T}(frame::CartesianFrame3D, moment::AbstractMatrix, cross_part::AbstractVector, mass) where T
        new{T}(frame, moment, cross_part, mass)
    end
end

"""
$(SIGNATURES)

Construct a `SpatialInertia` by specifying:

* `frame`: the frame in which the spatial inertia is expressed.
* `moment`: the moment of inertia expressed in `frame` (i.e., in `frame`'s axes and about the origin of `frame`).
* `cross_part`: the center of mass expressed in `frame`, multiplied by the mass.
* `mass`: the total mass.

For more convenient construction of `SpatialInertia`s, consider using the keyword argument constructor instead.
"""
@inline function SpatialInertia(frame::CartesianFrame3D, moment::AbstractMatrix, cross_part::AbstractVector, mass)
    T = promote_eltype(moment, cross_part, mass)
    SpatialInertia{T}(frame, moment, cross_part, mass)
end

"""
$(SIGNATURES)

Construct a `SpatialInertia` by specifying:

* `frame`: the frame in which the spatial inertia is expressed.
* one of:
  * `moment`: the moment of inertia expressed in `frame` (i.e., about the origin of `frame` and in `frame`'s axes).
  * `moment_about_com`: the moment of inertia about the center of mass, in `frame`'s axes.
* `com`: the center of mass expressed in `frame`.
* `mass`: the total mass.

The `com` and `mass` keyword arguments are required, as well as exactly one of `moment` and `moment_about_com`
"""
@inline function SpatialInertia(frame::CartesianFrame3D;
        moment::Union{AbstractMatrix, Nothing}=nothing,
        moment_about_com::Union{AbstractMatrix, Nothing}=nothing,
        com::AbstractVector,
        mass)
    if !((moment isa AbstractMatrix) ⊻ (moment_about_com isa AbstractMatrix))
        throw(ArgumentError("Exactly one of `moment` or `moment_about_com` is required."))
    end
    _com = SVector{3}(com)
    if moment !== nothing
        _moment = moment
    else
        _moment = SMatrix{3, 3}(moment_about_com)
        _moment -= mass * hat_squared(_com) # parallel axis theorem
    end
    SpatialInertia(frame, _moment, mass * _com, mass)
end

# SpatialInertia-specific functions
Base.eltype(::Type{SpatialInertia{T}}) where {T} = T

# Construct/convert given another SpatialInertia
SpatialInertia{T}(inertia::SpatialInertia{T}) where {T} = inertia
@inline function SpatialInertia{T}(inertia::SpatialInertia) where {T}
    SpatialInertia{T}(inertia.frame,
        SMatrix{3, 3, T}(inertia.moment),
        SVector{3, T}(inertia.cross_part),
        T(inertia.mass))
end
@inline Base.convert(::Type{S}, inertia::SpatialInertia) where {S<:SpatialInertia} = S(inertia)

# Construct/convert to SMatrix
function StaticArrays.SMatrix{6, 6, T, 36}(inertia::SpatialInertia) where {T}
    J = SMatrix{3, 3, T}(inertia.moment)
    C = hat(SVector{3, T}(inertia.cross_part))
    m = T(inertia.mass)
    [[J C]; [C' SMatrix{3, 3}(m * I)]]
end
StaticArrays.SMatrix{6, 6, T}(inertia::SpatialInertia) where {T} = SMatrix{6, 6, T, 36}(inertia)
StaticArrays.SMatrix{6, 6}(inertia::SpatialInertia{T}) where {T} = SMatrix{6, 6, T}(inertia)
StaticArrays.SMatrix(inertia::SpatialInertia) = SMatrix{6, 6}(inertia)
StaticArrays.SArray(inertia::SpatialInertia) = SMatrix(inertia)
Base.convert(::Type{A}, inertia::SpatialInertia) where {A<:SArray} = A(inertia)

function Base.convert(::Type{Matrix}, inertia::SpatialInertia) where {T<:Matrix}
    Base.depwarn("This convert method is deprecated. Please use `$T(SMatrix(inertia))` instead or
    reconsider whether conversion to Matrix is necessary.", :convert)
    T(SMatrix(inertia))
end

function Base.Array(inertia::SpatialInertia)
    Base.depwarn("This Array constructor is deprecated. Please use `Matrix(SMatrix(inertia))` instead or
    reconsider whether conversion to Matrix is necessary.", :Array)
    Matrix(SMatrix(inertia))
end

"""
$(SIGNATURES)

Return the center of mass of the `SpatialInertia` as a [`Point3D`](@ref).
"""
center_of_mass(inertia::SpatialInertia) = Point3D(inertia.frame, inertia.cross_part / inertia.mass)

function Base.show(io::IO, inertia::SpatialInertia)
    println(io, "SpatialInertia expressed in $(name_and_id(inertia.frame)):")
    println(io, "mass: $(inertia.mass)")
    println(io, "center of mass: $(center_of_mass(inertia))")
    print(io, "moment of inertia (about origin of $(name_and_id(inertia.frame)):\n$(inertia.moment)")
end

Base.zero(::Type{SpatialInertia{T}}, frame::CartesianFrame3D) where {T} = SpatialInertia(frame, zero(SMatrix{3, 3, T}), zero(SVector{3, T}), zero(T))
Base.zero(inertia::SpatialInertia) = zero(typeof(inertia), inertia.frame)

function Base.isapprox(x::SpatialInertia, y::SpatialInertia; atol = 1e-12)
    x.frame == y.frame && isapprox(x.moment, y.moment; atol = atol) && isapprox(x.cross_part, y.cross_part; atol = atol) && isapprox(x.mass, y.mass; atol = atol)
end

@inline function Base.:+(inertia1::SpatialInertia, inertia2::SpatialInertia)
    @framecheck(inertia1.frame, inertia2.frame)
    SpatialInertia(inertia1.frame,
        inertia1.moment + inertia2.moment,
        inertia1.cross_part + inertia2.cross_part,
        inertia1.mass + inertia2.mass)
end

"""
$(SIGNATURES)

Transform the `SpatialInertia` to a different frame.
"""
@inline function transform(inertia::SpatialInertia, t::Transform3D)
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
    minv = m > 0 ? inv(m) : zero(m)
    Jnew *= minv
    Jnew += R * J * R'
    SpatialInertia(t.to, Jnew, cnew, convert(eltype(Jnew), m))
end

function Random.rand(::Type{<:SpatialInertia{T}}, frame::CartesianFrame3D) where {T}
    # Try to generate a random but physical moment of inertia by constructing it from its eigendecomposition.

    # Create random principal moments of inertia (about the center of mass).
    # Scale the inertias to make the length scale of the equivalent inertial ellipsoid roughly ~1 unit
    ixx = rand(T) / 10.
    iyy = rand(T) / 10.

    # Ensure that the principal moments of inertia obey the triangle inequalities:
    # http://www.mathworks.com/help/physmod/sm/mech/vis/about-body-color-and-geometry.html
    lb = abs(ixx - iyy)
    ub = ixx + iyy
    izz = rand(T) * (ub - lb) + lb

    # Randomly rotate the principal axes.
    R = rand(RotMatrix3{T})
    moment_about_com = R * Diagonal(SVector(ixx, iyy, izz)) * R'

    SpatialInertia(frame, moment_about_com=moment_about_com, com=rand(SVector{3, T}) .- T(0.5), mass=rand(T))
end

"""
$(SIGNATURES)

Compute the mechanical power associated with a pairing of a wrench and a twist.
"""
LinearAlgebra.dot(w::Wrench, t::Twist) = begin @framecheck(w.frame, t.frame); dot(angular(w), angular(t)) + dot(linear(w), linear(t)) end
LinearAlgebra.dot(t::Twist, w::Wrench) = dot(w, t)


for (ForceSpaceMatrix, ForceSpaceElement) in (:MomentumMatrix => :Momentum, :MomentumMatrix => :Wrench, :WrenchMatrix => :Wrench)
    # MomentumMatrix * velocity vector --> Momentum
    # MomentumMatrix * acceleration vector --> Wrench
    # WrenchMatrix * dimensionless multipliers --> Wrench
    @eval function $ForceSpaceElement(mat::$ForceSpaceMatrix, x::AbstractVector)
        $ForceSpaceElement(mat.frame, SVector{3}(angular(mat) * x), SVector{3}(linear(mat) * x))
    end
end

@inline function Base.:*(inertia::SpatialInertia, twist::Twist)
    @framecheck(inertia.frame, twist.frame)
    ang, lin = mul_inertia(inertia.moment, inertia.cross_part, inertia.mass, angular(twist), linear(twist))
    Momentum(inertia.frame, ang, lin)
end

@inline function Base.:*(inertia::SpatialInertia, jac::GeometricJacobian)
    @framecheck(inertia.frame, jac.frame)
    Jω = angular(jac)
    Jv = linear(jac)
    J = inertia.moment
    m = inertia.mass
    c = inertia.cross_part
    ang = J * Jω + colwise(×, c, Jv)
    lin = m * Jv - colwise(×, c, Jω)
    MomentumMatrix(inertia.frame, ang, lin)
end

"""
$(SIGNATURES)

Apply the Newton-Euler equations to find the external wrench required to
make a body with spatial inertia ``I``, which has twist ``T`` with respect
to an inertial frame, achieve spatial acceleration ``\\dot{T}``.

This wrench is also equal to the rate of change of momentum of the body.
"""
@inline function newton_euler(inertia::SpatialInertia, spatial_accel::SpatialAcceleration, twist::Twist)
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
    ang += angular(T) × angular_momentum + linear(T) × linear_momentum
    lin += angular(T) × linear_momentum
    Wrench(frame, ang, lin)
end

@inline torque!(τ::AbstractVector, jac::GeometricJacobian, wrench::Wrench) = mul!(τ, transpose(jac), wrench)

@inline function torque(jac::GeometricJacobian, wrench::Wrench)
    T = promote_eltype(jac, wrench)
    τ = Vector{T}(undef, size(jac, 2))
    torque!(τ, jac, wrench)
    τ
end

for (MatrixType, VectorType) in (:WrenchMatrix => :(Union{Twist, SpatialAcceleration}), :GeometricJacobian => :(Union{Momentum, Wrench}))
    @eval begin
        @inline function LinearAlgebra.mul!(
                dest::AbstractVector{T},
                transposed_mat::LinearAlgebra.Transpose{<:Any, <:$MatrixType},
                vec::$VectorType) where T
            mat = parent(transposed_mat)
            @boundscheck length(dest) == size(mat, 2) || throw(DimensionMismatch())
            @framecheck mat.frame vec.frame

            mat_angular = angular(mat)
            mat_linear = linear(mat)
            vec_angular = angular(vec)
            vec_linear = linear(vec)

            @inbounds begin
                @simd for row in eachindex(dest)
                    dest[row] =
                        mat_angular[1, row] * vec_angular[1] +
                        mat_angular[2, row] * vec_angular[2] +
                        mat_angular[3, row] * vec_angular[3]
                end
                @simd for row in eachindex(dest)
                    dest[row] +=
                        mat_linear[1, row] * vec_linear[1] +
                        mat_linear[2, row] * vec_linear[2] +
                        mat_linear[3, row] * vec_linear[3]
                end
            end
            dest
        end

        function Base.:*(transposed_mat::LinearAlgebra.Transpose{<:Any, <:$MatrixType}, vec::$VectorType)
            mat = parent(transposed_mat)
            @framecheck mat.frame vec.frame
            transpose(angular(mat)) * angular(vec) + transpose(linear(mat)) * linear(vec)
        end
    end
end

for ForceSpaceMatrix in (:MomentumMatrix, :WrenchMatrix)
    for (A, B) in ((ForceSpaceMatrix, :GeometricJacobian), (:GeometricJacobian, ForceSpaceMatrix))
        @eval begin
            function Base.:*(at::LinearAlgebra.Transpose{<:Any, <:$A}, b::$B)
                a = parent(at)
                @framecheck a.frame b.frame
                transpose(angular(a)) * angular(b) + transpose(linear(a)) * linear(b)
            end
            function Base.:*(a::$A, bt::LinearAlgebra.Transpose{<:Any, <:$B})
                b = parent(bt)
                @framecheck a.frame b.frame
                angular(a) * transpose(angular(b)) + linear(a) * transpose(linear(b))
            end
        end
    end
end

"""
$(SIGNATURES)

Compute the kinetic energy of a body with spatial inertia ``I``, which has
twist ``T`` with respect to an inertial frame.
"""
@inline function kinetic_energy(inertia::SpatialInertia, twist::Twist)
    @framecheck(inertia.frame, twist.frame)
    # TODO: should assert that twist.base is an inertial frame somehow
    ω = angular(twist)
    v = linear(twist)
    J = inertia.moment
    c = inertia.cross_part
    m = inertia.mass
    (ω ⋅ (J * ω) + v ⋅ (m * v + 2 * (ω × c))) / 2
end
