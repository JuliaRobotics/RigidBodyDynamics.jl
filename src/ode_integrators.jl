immutable ButcherTableau{N, T<:Real, L}
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
    a[4, 3] = 1/2
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

immutable MuntheKaasStageCache{N, T<:Real}
    q0::Vector{T} # global coordinates
    vs::SVector{N, Vector{T}} # velocity vector for each stage
    vds::SVector{N, Vector{T}} # time derivatives of vs
    ϕs::SVector{N, Vector{T}} # local coordinates around q0 for each stage
    ϕds::SVector{N, Vector{T}} # time derivatives of ϕs

    function MuntheKaasStageCache()
        q0 = Vector{T}()
        vs = SVector{N, Vector{T}}((Vector{T}() for i in 1 : N)...)
        vds = SVector{N, Vector{T}}((Vector{T}() for i in 1 : N)...)
        ϕs = SVector{N, Vector{T}}((Vector{T}() for i in 1 : N)...)
        ϕds = SVector{N, Vector{T}}((Vector{T}() for i in 1 : N)...)
        new(q0, vs, vds, ϕs, ϕds)
    end
end
set_num_positions!(cache::MuntheKaasStageCache, n::Int64) = resize!(cache.q0, n)
function set_num_velocities!(cache::MuntheKaasStageCache, n::Int64)
    for v in cache.vs resize!(v, n) end
    for vd in cache.vds resize!(vd, n) end
    for ϕ in cache.ϕs resize!(ϕ, n) end
    for ϕd in cache.ϕds resize!(ϕd, n) end
end

immutable MuntheKaasIntegrator{N, T<:Real, F, S<:OdeResultsSink, L}
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
function MuntheKaasIntegrator{N, T<:Real, F, S<:OdeResultsSink, L}(dynamics!::F, tableau::ButcherTableau{N, T, L}, sink::S)
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
    q0 = stages.q0
    copy!(q0, configuration_vector(state))

    # Compute integrator stages.
    for i = 1 : n
        # Update local coordinates and velocities
        ϕ = stages.ϕs[i]
        v = stages.vs[i]
        fill!(ϕ, 0)
        copy!(v, velocity_vector(state))
        for j = 1 : i - 1
            aij = tableau.a[i, j]
            if aij != zero(aij)
                weight = Δt * aij
                scaleadd!(ϕ, stages.ϕds[j], weight) # TODO: use fusing broadcast in 0.6
                scaleadd!(v, stages.vds[j], weight) # TODO: use fusing broadcast in 0.6
            end
        end

        # Convert from local to global coordinates and set state
        global_coordinates!(state, q0, ϕ)
        set_velocity!(state, v)

        # Dynamics in global coordinates
        vd = stages.vds[i]
        tstage = t + Δt * tableau.c[i]
        integrator.dynamics!(vd, tstage, state)

        # Convert back to local coordinates TODO: ϕ not needed, just ϕd!
        ϕd = stages.ϕds[i]
        local_coordinates!(state, ϕ, ϕd, q0)
    end

    # Combine stages (store in vector for first step) # TODO: don't do that to make code simpler
    ϕ = stages.ϕs[1]
    v = stages.vs[1]
    scale!(ϕ, tableau.b[1] * Δt)
    scale!(v, tableau.b[1] * Δt)
    for i = 2 : n
        weight = tableau.b[i] * Δt
        scaleadd!(ϕ, stages.ϕs[i], weight) # TODO: use fusing broadcast in 0.6
        scaleadd!(v, stages.vs[i], weight) # TODO: use fusing broadcast in 0.6
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
