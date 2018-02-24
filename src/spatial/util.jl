## Colwise
# TODO: replace with future mapslices specialization, see https://github.com/JuliaArrays/StaticArrays.jl/pull/99
"""
colwise(f, vec, mat)
Return a matrix `A` such that `A[:, i] == f(vec, mat[:, i])`.
"""
@generated function colwise(f, vec::StaticVector, mat::StaticArray)
    length(vec) == size(mat, 1) || throw(DimensionMismatch())
    if size(mat, 2) == 0
        T = similar_type(mat, promote_type(eltype(vec), eltype(mat)))
        quote
            $(Expr(:meta, :inline))
            zeros($T)
        end
    else
        exprs = [:(f(vec, mat[:, $j])) for j = 1 : size(mat, 2)]
        quote
            $(Expr(:meta, :inline))
            @inbounds return $(Expr(:call, hcat, exprs...))
        end
    end
end

"""
colwise(f, vec, mat)
Return a matrix `A` such that `A[:, i] == f(vec, mat[:, i])`.
"""
function colwise(f, vec::AbstractVector, mat::AbstractMatrix)
    mapslices(x -> f(vec, x), mat, (1,))
end

"""
colwise(f, mat, vec)
Return a matrix `A` such that `A[:, i] == f(mat[:, i], vec)`.
"""
@generated function colwise(f, mat::StaticArray, vec::StaticVector)
    length(vec) == size(mat, 1) || throw(DimensionMismatch())
    if size(mat, 2) == 0
        T = similar_type(mat, promote_type(eltype(vec), eltype(mat)))
        quote
            $(Expr(:meta, :inline))
            zeros($T)
        end
    else
        exprs = [:(f(mat[:, $j], vec)) for j = 1 : size(mat, 2)]
        quote
            $(Expr(:meta, :inline))
            @inbounds return $(Expr(:call, hcat, exprs...))
        end
    end
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
    Θ = norm(ϕ)
    ϕ̇ = ω
    if Θ > eps(Θ)
        ϕ̇ += T(0.5) * (ϕ × ω) + 1 / Θ^2 * (1 - (Θ * sin(Θ)) / (2 * (1 - cos(Θ)))) * ϕ × (ϕ × ω)
    end
    ϕ̇
end


function transform_spatial_motion(angular::SVector{3}, linear::SVector{3}, rot::R, trans::SVector{3}) where {R <: Rotation{3}}
    angular = rot * angular
    linear = rot * linear + cross(trans, angular)
    angular, linear
end

function mul_inertia(J, c, m, ω, v)
    angular = J * ω + cross(c, v)
    linear = m * v - cross(c, ω)
    angular, linear
end

# also known as 'spatial motion cross product'
@inline function se3_commutator(xω, xv, yω, yv)
    angular = cross(xω, yω)
    linear = cross(xω, yv) + cross(xv, yω)
    angular, linear
end

function quaternion_derivative end
function angular_velocity_in_body end

@inline function velocity_jacobian(::typeof(quaternion_derivative), q::Quat)
    (@SMatrix [
        -q.x -q.y -q.z;
        q.w -q.z  q.y;
        q.z  q.w -q.x;
        -q.y  q.x  q.w]) / 2
end

@inline function velocity_jacobian(::typeof(angular_velocity_in_body), q::Quat)
    2 * @SMatrix [
    -q.x  q.w  q.z -q.y;
    -q.y -q.z  q.w  q.x;
    -q.z  q.y -q.x  q.w]
end

@inline function quaternion_derivative(q::Quat, angular_velocity_in_body::AbstractVector)
    @boundscheck length(angular_velocity_in_body) == 3 || error("size mismatch")
    velocity_jacobian(quaternion_derivative, q) * angular_velocity_in_body
end

@inline function angular_velocity_in_body(q::Quat, quat_derivative::AbstractVector)
    @boundscheck length(quat_derivative) == 4 || error("size mismatch")
    velocity_jacobian(angular_velocity_in_body, q) * quat_derivative
end

function linearized_rodrigues_vec(r::RotMatrix) # TODO: consider moving to Rotations
    x = (r[3, 2] - r[2, 3]) / 2
    y = (r[1, 3] - r[3, 1]) / 2
    z = (r[2, 1] - r[1, 2]) / 2
    RodriguesVec(x, y, z)
end
