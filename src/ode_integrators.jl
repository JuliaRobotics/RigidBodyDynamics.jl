# Described in
# Andrle, Michael S., and John L. Crassidis.
# "Geometric integration of quaternions."
# Journal of Guidance, Control, and Dynamics 36.6 (2013): 1762-1767.

# A crucial step is ‘pulling back’ the equation from G to g using exp.
# need function: exp!(joint, q, ϕ); rotation vector to quaternion; 'canonical coordinates of the first kind'
# need function: log!(joint, ϕ, q); (local) quaternion to rotation vector
# need function: dexp_inv!(joint, ϕ̇, ϕ, v); ϕ being local coordinates centred about the original configuration

immutable ButcherTableau{N, T}
    a::SMatrix{N, N, T}
    b::SVector{N, T}
    c::SVector{N, T}
    explicit::Bool

    function ButcherTableau(a::AbstractMatrix{T}, b::AbstractVector{T})
        @assert size(a, 1) == N
        @assert size(a, 2) == N
        @assert length(b) == N
        c = vec(sum(a, 2))
        explicit = all(triu(a) .== 0)
        new(SMatrix{N, N}(a), SVector{N}(b), SVector{N}(c), explicit)
    end
end

ButcherTableau{T}(a::Matrix{T}, b::Vector{T}) = ButcherTableau{length(b), T}(a, b)

num_stages{N, T}(::ButcherTableau{N, T}) = N
isexplicit(tableau::ButcherTableau) = tableau.explicit

function CrouchGrossman3{T}(::Type{T})
    a = zeros(T, 3, 3)
    a[2, 1] = 3 / 4
    a[3, 1] = 119 / 216
    a[3, 2] = 17 / 108
    b = T[13 / 51, -2 / 3, 24 / 17]
    ButcherTableau(a, b)
end

type LieIntegrator{X, M, C}
    state::MechanismState{X, M, C}
    tableau::ExplicitButcherTableau{C} #TODO: assert isexplicit
end

function step(integrator::LieIntegrator, Δt::Real)
end

function (integrator::LieCrouchGrossman3)()
    state = integrator.state
    q = configuration_vector(state)
    v = velocity_vector(state)
end
