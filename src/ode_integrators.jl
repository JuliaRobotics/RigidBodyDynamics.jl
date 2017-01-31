module OdeIntegrators

using RigidBodyDynamics
using StaticArrays

export runge_kutta_4,
    MuntheKaasIntegrator,
    OdeResultsSink,
    OdeRingBufferStorage,
    integrate,
    step

import Base: eltype, length, step
import RigidBodyDynamics: scaleadd!

immutable ButcherTableau{N, T<:Number, L}
    a::SMatrix{N, N, T, L}
    b::SVector{N, T}
    c::SVector{N, T}
    explicit::Bool

    function ButcherTableau(a::AbstractMatrix{T}, b::AbstractVector{T})
        @assert N > 0
        @assert size(a, 1) == N
        @assert size(a, 2) == N
        @assert length(b) == N
        c = vec(sum(a, 2))
        explicit = all(triu(a) .== 0)
        new(SMatrix{N, N}(a), SVector{N}(b), SVector{N}(c), explicit)
    end
end
ButcherTableau{T}(a::Matrix{T}, b::Vector{T}) = ButcherTableau{length(b), T, length(b)^2}(a, b)
num_stages{N, T}(::ButcherTableau{N, T}) = N
isexplicit(tableau::ButcherTableau) = tableau.explicit

function runge_kutta_4{T}(::Type{T})
    a = zeros(T, 4, 4)
    a[2, 1] = 1/2
    a[3, 2] = 1/2
    a[4, 3] = 1
    b = T[1/6, 1/3, 1/3, 1/6]
    ButcherTableau(a, b)
end

abstract OdeResultsSink
initialize(::OdeResultsSink, state) = error("concrete subtypes must implement")
process(::OdeResultsSink, t, state) = error("concrete subtypes must implement")

type OdeRingBufferStorage{T} <: OdeResultsSink
    ts::Vector{T}
    qs::Vector{Vector{T}}
    vs::Vector{Vector{T}}
    nextIndex::Int64

    function OdeRingBufferStorage(n::Int64)
        ts = Vector{T}(n)
        qs = [Vector{T}() for i in 1 : n]
        vs = [Vector{T}() for i in 1 : n]
        new(ts, qs, vs, 1)
    end
end
Base.eltype{T}(storage::OdeRingBufferStorage{T}) = T
Base.length(storage::OdeRingBufferStorage) = length(storage.ts)
set_num_positions!(storage::OdeRingBufferStorage, n::Int64) = for q in storage.qs resize!(q, n) end
set_num_velocities!(storage::OdeRingBufferStorage, n::Int64) = for v in storage.vs resize!(v, n) end

function initialize{T}(storage::OdeRingBufferStorage{T}, t::T, state)
    set_num_positions!(storage, length(configuration_vector(state)))
    set_num_velocities!(storage, length(velocity_vector(state)))
    process(storage, t, state)
end

function process{T}(storage::OdeRingBufferStorage{T}, t::T, state)
    index = storage.nextIndex
    storage.ts[index] = t
    copy!(storage.qs[index], configuration_vector(state))
    copy!(storage.vs[index], velocity_vector(state))
    storage.nextIndex = index % length(storage) + 1
    nothing
end

type MuntheKaasStageCache{N, T<:Number}
    q0::Vector{T} # global coordinates
    vs::SVector{N, Vector{T}} # velocity vector for each stage
    vds::SVector{N, Vector{T}} # time derivatives of vs
    ϕs::SVector{N, Vector{T}} # local coordinates around q0 for each stage
    ϕds::SVector{N, Vector{T}} # time derivatives of ϕs
    ϕstep::Vector{T}
    vstep::Vector{T}

    function MuntheKaasStageCache()
        q0 = Vector{T}()
        vs = SVector{N, Vector{T}}((Vector{T}() for i in 1 : N)...)
        vds = SVector{N, Vector{T}}((Vector{T}() for i in 1 : N)...)
        ϕs = SVector{N, Vector{T}}((Vector{T}() for i in 1 : N)...)
        ϕds = SVector{N, Vector{T}}((Vector{T}() for i in 1 : N)...)
        ϕstep = Vector{T}()
        vstep = Vector{T}()
        new(q0, vs, vds, ϕs, ϕds, ϕstep, vstep)
    end
end
set_num_positions!(cache::MuntheKaasStageCache, n::Int64) = resize!(cache.q0, n)
function set_num_velocities!(cache::MuntheKaasStageCache, n::Int64)
    for v in cache.vs resize!(v, n) end
    for vd in cache.vds resize!(vd, n) end
    for ϕ in cache.ϕs resize!(ϕ, n) end
    for ϕd in cache.ϕds resize!(ϕd, n) end
    resize!(cache.ϕstep, n)
    resize!(cache.vstep, n)
end

# A Lie-group-aware ODE integrator.
#
# MuntheKaasIntegrator is used to properly integrate the kinematics of globally
# parameterized rigid joints (Duindam, Port-Based Modeling and Control for
# Efficient Bipedal Walking Robots, 2006, Definition 2.9). Global parameterizations of e.g. SO(3)
# are needed to avoid singularities, but this leads to the problem that the tangent
# space no longer has the same dimension as the ambient space of the global parameterization.
# A Munthe-Kaas integrator solves this problem by converting back and forth
# between local and global coordinates at every integration time step.
#
# The idea is to do the dynamics and compute the stages of the integration scheme
# in terms of local coordinates centered around the global parameterization of
# the configuration at the end of the previous time step (e.g. exponential coordinates),
# combine the stages into a new set of local coordinates as usual for Runge-Kutta methods,
# and then convert the local coordinates back to global coordinates.
#
# From Iserles et al., 'Lie-group methods' (2000).
# https://hal.archives-ouvertes.fr/hal-01328729
#
# Another useful reference is Park and Chung,
# 'Geometric Integration on Euclidean Group with Application to Articulated Multibody Systems' (2005).
# http://www.ent.mrt.ac.lk/iml/paperbase/TRO%20Collection/TRO/2005/october/7.pdf
immutable MuntheKaasIntegrator{N, T<:Number, F, S<:OdeResultsSink, L}
    dynamics!::F # dynamics!(vd, t, state), sets vd (time derivative of v) given time t and state
    tableau::ButcherTableau{N, T, L}
    sink::S
    stages::MuntheKaasStageCache{N, T}

    function MuntheKaasIntegrator(dynamics!::F, tableau::ButcherTableau{N, T, L}, sink::S)
        @assert isexplicit(tableau)
        stages = MuntheKaasStageCache{N, T}()
        new(dynamics!, tableau, sink, stages)
    end
end
function MuntheKaasIntegrator{N, T<:Number, F, S<:OdeResultsSink, L}(dynamics!::F, tableau::ButcherTableau{N, T, L}, sink::S)
    MuntheKaasIntegrator{N, T, F, S, L}(dynamics!, tableau, sink)
end
num_stages{N}(::MuntheKaasIntegrator{N}) = N
eltype{N, T}(::MuntheKaasIntegrator{N, T}) = T

# state must be a type for which the following functions are defined:
# - configuration_vector(state), returns the configuration vector in global coordinates
# - velocity_vector(state), returns the velocity vector
# - set_velocity!(state, v), sets velocity vector to v
# - global_coordinates!(state, q0, ϕ), sets global coordinates in state based on local coordinates ϕ centered around global coordinates q0
# - local_coordinates!(state, ϕ, ϕd, q0), converts state's global configuration q and velocity v to local coordinates centered around global coordinates q0
function step(integrator::MuntheKaasIntegrator, t::Real, state, Δt::Real)
    tableau = integrator.tableau
    stages = integrator.stages
    n = num_stages(integrator)

    # Use current configuration as the configuration around which the local coordinates for this step will be centered.
    q0, v0 = stages.q0, stages.vstep
    copy!(q0, configuration_vector(state))
    copy!(v0, velocity_vector(state))

    # Compute integrator stages.
    for i = 1 : n
        # Update local coordinates and velocities
        ϕ = stages.ϕs[i]
        v = stages.vs[i]
        fill!(ϕ, zero(eltype(ϕ)))
        copy!(v, v0)
        for j = 1 : i - 1
            aij = tableau.a[i, j]
            if aij != zero(aij)
                weight = Δt * aij
                scaleadd!(ϕ, stages.ϕds[j], weight)
                scaleadd!(v, stages.vds[j], weight)
            end
        end

        # Convert from local to global coordinates and set state
        global_coordinates!(state, q0, ϕ)
        set_velocity!(state, v)

        # Dynamics in global coordinates
        vd = stages.vds[i]
        integrator.dynamics!(vd, t + Δt * tableau.c[i], state)

        # Convert back to local coordinates
        ϕd = stages.ϕds[i] # TODO: ϕ not actually needed, just ϕd!
        local_coordinates!(state, ϕ, ϕd, q0)
    end

    # Combine stages
    ϕ = stages.ϕstep
    fill!(ϕ, zero(eltype(ϕ)))
    v = stages.vstep # already initialized to v0
    for i = 1 : n
        weight = tableau.b[i] * Δt
        scaleadd!(ϕ, stages.ϕds[i], weight)
        scaleadd!(v, stages.vds[i], weight)
    end

    # Convert from local to global coordinates
    global_coordinates!(state, q0, ϕ)
    set_velocity!(state, v)

    nothing
end

function integrate(integrator::MuntheKaasIntegrator, state0, finalTime, Δt)
    T = eltype(integrator)
    t = zero(T)
    state = state0
    set_num_positions!(integrator.stages, length(configuration_vector(state)))
    set_num_velocities!(integrator.stages, length(velocity_vector(state)))
    initialize(integrator.sink, t, state)
    while t < finalTime
        step(integrator, t, state, Δt)
        t += Δt
        process(integrator.sink, t, state)
    end
end

end # module
