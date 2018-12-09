## Colwise
# TODO: replace with future mapslices specialization, see https://github.com/JuliaArrays/StaticArrays.jl/pull/99
"""
    $(SIGNATURES)

Equivalent to one of

```julia
mapslices(x -> f(a, x), B, dims=1)
mapslices(x -> f(x, b), A, dims=1)
```

but optimized for statically-sized matrices.
"""
function colwise end

colwise(f, a::AbstractVector, B::AbstractMatrix) = mapslices(x -> f(a, x), B, dims=1)
colwise(f, A::AbstractMatrix, b::AbstractVector) = mapslices(x -> f(x, b), A, dims=1)

@inline function colwise(f, a::StaticVector, B::StaticMatrix)
    Sa = Size(a)
    SB = Size(B)
    Sa[1] === SB[1] || throw(DimensionMismatch())
    _colwise(f, Val(SB[2]), a, B)
end

@inline function _colwise(f, ::Val{0}, a::StaticVector, B::StaticMatrix)
    zero(similar_type(B, promote_type(eltype(a), eltype(B))))
end

@inline function _colwise(f, M::Val, a::StaticVector, B::StaticMatrix)
    cols = ntuple(i -> f(a, B[:, i]), M)
    hcat(cols...)
end

@inline function colwise(f, A::StaticMatrix, b::StaticVector)
    SA = Size(A)
    Sb = Size(b)
    SA[1] === Sb[1] || throw(DimensionMismatch())
    _colwise(f, Val(SA[2]), A, b)
end

@inline function _colwise(f, ::Val{0}, A::StaticMatrix, b::StaticVector)
    zero(similar_type(A, promote_type(eltype(A), eltype(b))))
end

@inline function _colwise(f, M::Val, A::StaticMatrix, b::StaticVector)
    cols = ntuple(i -> f(A[:, i], b), M)
    hcat(cols...)
end


@inline function vector_to_skew_symmetric(v::SVector{3, T}) where {T}
    @SMatrix [zero(T) -v[3] v[2];
            v[3] zero(T) -v[1];
            -v[2] v[1] zero(T)]
end
const hat = vector_to_skew_symmetric

@inline function vector_to_skew_symmetric_squared(a::SVector{3})
    a1² = a[1] * a[1]
    a2² = a[2] * a[2]
    a3² = a[3] * a[3]
    b11 = -a2² - a3²
    b12 = a[1] * a[2]
    b13 = a[1] * a[3]
    b22 = -a1² - a3²
    b23 = a[2] * a[3]
    b33 = -a1² - a2²
    @SMatrix [b11 b12 b13;
            b12 b22 b23;
            b13 b23 b33]
end
const hat_squared = vector_to_skew_symmetric_squared


# The 'Bortz equation'.
# Bortz, John E. "A new mathematical formulation for strapdown inertial navigation."
# IEEE transactions on aerospace and electronic systems 1 (1971): 61-66.
#
# Or, interpreted in a Lie setting:
# d/dt(exp(ϕ(t))) = ̂(dexp(ϕ(t)) * ϕ̇(t)) * exp(ϕ(t)) (where dexp is the 'right trivialized' tangent of the exponential map)
# ̂(dexp(ϕ(t)) * ϕ̇(t)) = d/dt(exp(ϕ(t))) * exp(ϕ(t))⁻¹  (hat form of angular velocity in world frame)
#                      = ̂(exp(ϕ(t)) ω)      (with ω angular velocity in body frame)
# ϕ̇(t) = dexp⁻¹(ϕ(t)) * exp(ϕ(t)) * ω
function rotation_vector_rate(rotation_vector::AbstractVector{T}, angular_velocity_in_body::AbstractVector{T}) where {T}
    ϕ = rotation_vector
    ω = angular_velocity_in_body
    @boundscheck length(ϕ) == 3 || error("ϕ has wrong length")
    @boundscheck length(ω) == 3 || error("ω has wrong length")
    θ = norm(ϕ)
    ϕ̇ = ω
    if θ > eps(θ)
        s, c = sincos(θ)
        ϕ̇ += (ϕ × ω) / 2 + 1 / θ^2 * (1 - (θ * s) / (2 * (1 - c))) * ϕ × (ϕ × ω)
    end
    ϕ̇
end


@inline function transform_spatial_motion(angular::SVector{3}, linear::SVector{3}, rot::R, trans::SVector{3}) where {R <: Rotation{3}}
    angular = rot * angular
    linear = rot * linear + trans × angular
    angular, linear
end

@inline function mul_inertia(J, c, m, ω, v)
    angular = J * ω + c × v
    linear = m * v - c × ω
    angular, linear
end

# also known as 'spatial motion cross product'
@inline function se3_commutator(xω, xv, yω, yv)
    angular = xω × yω
    linear = xω × yv + xv × yω
    angular, linear
end

function quaternion_derivative end
function spquat_derivative end
function angular_velocity_in_body end

@inline function velocity_jacobian(::typeof(quaternion_derivative), q::Quat)
    (@SMatrix [
        -q.x -q.y -q.z;
        q.w -q.z  q.y;
        q.z  q.w -q.x;
        -q.y  q.x  q.w]) / 2
end

@inline function velocity_jacobian(::typeof(spquat_derivative), q::SPQuat)
    quat = Quat(q)
    dQuat_dW = velocity_jacobian(quaternion_derivative, quat)
    dSPQuat_dQuat = Rotations.jacobian(SPQuat, quat)
    dSPQuat_dQuat * dQuat_dW
end

@inline function velocity_jacobian(::typeof(angular_velocity_in_body), q::Quat)
    2 * @SMatrix [
    -q.x  q.w  q.z -q.y;
    -q.y -q.z  q.w  q.x;
    -q.z  q.y -q.x  q.w]
end

@inline function velocity_jacobian(::typeof(angular_velocity_in_body), q::SPQuat)
    quat = Quat(q)
    dW_dQuat = velocity_jacobian(angular_velocity_in_body, quat)
    dQuat_dSPQuat = Rotations.jacobian(Quat, q)
    dW_dQuat * dQuat_dSPQuat
end

@inline function quaternion_derivative(q::Quat, angular_velocity_in_body::AbstractVector)
    @boundscheck length(angular_velocity_in_body) == 3 || error("size mismatch")
    velocity_jacobian(quaternion_derivative, q) * angular_velocity_in_body
end

@inline function spquat_derivative(q::SPQuat, angular_velocity_in_body::AbstractVector)
    @boundscheck length(angular_velocity_in_body) == 3 || error("size mismatch")
    velocity_jacobian(spquat_derivative, q) * angular_velocity_in_body
end

@inline function angular_velocity_in_body(q::Quat, quat_derivative::AbstractVector)
    @boundscheck length(quat_derivative) == 4 || error("size mismatch")
    velocity_jacobian(angular_velocity_in_body, q) * quat_derivative
end

@inline function angular_velocity_in_body(q::SPQuat, spq_derivative::AbstractVector)
    @boundscheck length(spq_derivative) == 3 || error("size mismatch")
    velocity_jacobian(angular_velocity_in_body, q) * spq_derivative
end

function linearized_rodrigues_vec(r::RotMatrix) # TODO: consider moving to Rotations
    x = (r[3, 2] - r[2, 3]) / 2
    y = (r[1, 3] - r[3, 1]) / 2
    z = (r[2, 1] - r[1, 2]) / 2
    RodriguesVec(x, y, z)
end
