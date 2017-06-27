module OdeIntegrators

using RigidBodyDynamics
using StaticArrays
using DocStringExtensions

export runge_kutta_4,
    MuntheKaasIntegrator,
    OdeResultsSink,
    RingBufferStorage,
    ExpandingStorage,
    integrate,
    step

import Base: eltype, length, step
import RigidBodyDynamics: scaleadd!

"""
$(TYPEDEF)

A [Butcher tableau](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods#Explicit_Runge.E2.80.93Kutta_methods).
"""
immutable ButcherTableau{N, T<:Number, L}
    a::SMatrix{N, N, T, L}
    b::SVector{N, T}
    c::SVector{N, T}
    explicit::Bool

    function (::Type{ButcherTableau{N, T, L}}){N, T<:Number, L}(a::AbstractMatrix{T}, b::AbstractVector{T})
        @assert N > 0
        @assert size(a, 1) == N
        @assert size(a, 2) == N
        @assert length(b) == N
        c = vec(sum(a, 2))
        explicit = all(triu(a) .== 0)
        new{N, T, L}(SMatrix{N, N}(a), SVector{N}(b), SVector{N}(c), explicit)
    end
end
ButcherTableau{T}(a::Matrix{T}, b::Vector{T}) = ButcherTableau{length(b), T, length(b)^2}(a, b)
num_stages{N, T}(::ButcherTableau{N, T}) = N
isexplicit(tableau::ButcherTableau) = tableau.explicit

"""
$(SIGNATURES)

Return the Butcher tableau for the standard fourth order Runge-Kutta integrator.
"""
function runge_kutta_4{T}(scalartype::Type{T})
    a = zeros(T, 4, 4)
    a[2, 1] = 1/2
    a[3, 2] = 1/2
    a[4, 3] = 1
    b = T[1/6, 1/3, 1/3, 1/6]
    ButcherTableau(a, b)
end

"""
$(TYPEDEF)

Does 'something' with the results of an ODE integration (e.g. storing results,
visualizing, etc.). Subtypes must implement:
* `initialize(sink, state)`: called with the initial state when integration begins.
* `process(sink, t, state)`: called at every integration time step with the current state and time.
"""
abstract type OdeResultsSink end
initialize(::OdeResultsSink, state) = error("concrete subtypes must implement")
process(::OdeResultsSink, t, state) = error("concrete subtypes must implement")

"""
$(TYPEDEF)

An `OdeResultsSink` that stores the state at each integration time step in a
ring buffer.
"""
type RingBufferStorage{T} <: OdeResultsSink
    ts::Vector{T}
    qs::Vector{Vector{T}}
    vs::Vector{Vector{T}}
    lastIndex::Int64

    function (::Type{RingBufferStorage{T}}){T}(n::Int64)
        ts = Vector{T}(n)
        qs = [Vector{T}() for i in 1 : n]
        vs = [Vector{T}() for i in 1 : n]
        new{T}(ts, qs, vs, 0)
    end
end
Base.eltype{T}(storage::RingBufferStorage{T}) = T
Base.length(storage::RingBufferStorage) = length(storage.ts)
set_num_positions!(storage::RingBufferStorage, n::Int64) = for q in storage.qs resize!(q, n) end
set_num_velocities!(storage::RingBufferStorage, n::Int64) = for v in storage.vs resize!(v, n) end

function initialize{T}(storage::RingBufferStorage{T}, t::T, state)
    set_num_positions!(storage, length(configuration(state)))
    set_num_velocities!(storage, length(velocity(state)))
    process(storage, t, state)
end

function process{T}(storage::RingBufferStorage{T}, t::T, state)
    index = storage.lastIndex % length(storage) + 1
    storage.ts[index] = t
    copy!(storage.qs[index], configuration(state))
    copy!(storage.vs[index], velocity(state))
    storage.lastIndex = index
    nothing
end

"""
$(TYPEDEF)

An `OdeResultsSink` that stores the state at each integration time step in
`Vectors` that may expand.
"""
type ExpandingStorage{T} <: OdeResultsSink
    ts::Vector{T}
    qs::Vector{Vector{T}}
    vs::Vector{Vector{T}}

    function (::Type{ExpandingStorage{T}}){T}(n::Int64)
        ts = Vector{T}(); sizehint!(ts, n)
        qs = Vector{T}[]; sizehint!(qs, n)
        vs = Vector{T}[]; sizehint!(vs, n)
        new{T}(ts, qs, vs)
    end
end
Base.eltype{T}(storage::ExpandingStorage{T}) = T
Base.length(storage::ExpandingStorage) = length(storage.ts)

initialize{T}(storage::ExpandingStorage{T}, t::T, state) = process(storage, t, state)

function process{T}(storage::ExpandingStorage{T}, t::T, state)
    push!(storage.ts, t)
    push!(storage.qs, copy(configuration(state)))
    push!(storage.vs, copy(velocity(state)))
    nothing
end

type MuntheKaasStageCache{N, T<:Number}
    q0::Vector{T} # global coordinates
    vs::SVector{N, Vector{T}} # velocity vector for each stage
    vds::SVector{N, Vector{T}} # time derivatives of vs
    ϕs::SVector{N, Vector{T}} # local coordinates around q0 for each stage
    ϕds::SVector{N, Vector{T}} # time derivatives of ϕs
    ss::SVector{N, Vector{T}} # additional state for each stage
    sds::SVector{N, Vector{T}} # time derivatives of ss
    ϕstep::Vector{T} # local coordinates around q0 after complete step
    vstep::Vector{T} # velocity after complete step
    sstep::Vector{T} # additional state after complete step

    function (::Type{MuntheKaasStageCache{N, T}}){N, T<:Number}()
        q0 = Vector{T}()
        vs = SVector{N, Vector{T}}((Vector{T}() for i in 1 : N)...)
        vds = SVector{N, Vector{T}}((Vector{T}() for i in 1 : N)...)
        ϕs = SVector{N, Vector{T}}((Vector{T}() for i in 1 : N)...)
        ϕds = SVector{N, Vector{T}}((Vector{T}() for i in 1 : N)...)
        ss = SVector{N, Vector{T}}((Vector{T}() for i in 1 : N)...)
        sds = SVector{N, Vector{T}}((Vector{T}() for i in 1 : N)...)
        ϕstep = Vector{T}()
        vstep = Vector{T}()
        sstep = Vector{T}()
        new{N, T}(q0, vs, vds, ϕs, ϕds, ss, sds, ϕstep, vstep, sstep)
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

function set_num_additional_states!(cache::MuntheKaasStageCache, n::Int64)
    for s in cache.ss resize!(s, n) end
    for sd in cache.sds resize!(sd, n) end
    resize!(cache.sstep, n)
end

"""
A Lie-group-aware ODE integrator.

`MuntheKaasIntegrator` is used to properly integrate the dynamics of globally
parameterized rigid joints (Duindam, Port-Based Modeling and Control for
Efficient Bipedal Walking Robots, 2006, Definition 2.9). Global parameterizations of e.g. ``SO(3)``
are needed to avoid singularities, but this leads to the problem that the tangent
space no longer has the same dimension as the ambient space of the global parameterization.
A Munthe-Kaas integrator solves this problem by converting back and forth
between local and global coordinates at every integration time step.

The idea is to do the dynamics and compute the stages of the integration scheme
in terms of local coordinates centered around the global parameterization of
the configuration at the end of the previous time step (e.g. exponential coordinates),
combine the stages into a new set of local coordinates as usual for Runge-Kutta methods,
and then convert the local coordinates back to global coordinates.

From [Iserles et al., 'Lie-group methods' (2000)](https://hal.archives-ouvertes.fr/hal-01328729).

Another useful reference is [Park and Chung, 'Geometric Integration on Euclidean Group with Application to Articulated Multibody Systems' (2005)](http://www.ent.mrt.ac.lk/iml/paperbase/TRO%20Collection/TRO/2005/october/7.pdf).
"""
immutable MuntheKaasIntegrator{N, T<:Number, F, S<:OdeResultsSink, L}
    dynamics!::F # dynamics!(vd, sd, t, state), sets vd (time derivative of v) and sd (time derivative of s) given time t and state
    tableau::ButcherTableau{N, T, L}
    sink::S
    stages::MuntheKaasStageCache{N, T}

    function (::Type{MuntheKaasIntegrator{N, T, F, S, L}}){N, T<:Number, F, S<:OdeResultsSink, L}(dynamics!::F, tableau::ButcherTableau{N, T, L}, sink::S)
        @assert isexplicit(tableau)
        stages = MuntheKaasStageCache{N, T}()
        new{N, T, F, S, L}(dynamics!, tableau, sink, stages)
    end
end

"""
Create a `MuntheKaasIntegrator` given:
* a callable `dynamics!(vd, t, state)` that updates the joint acceleration vector `vd` at time `t` and in state `state`;
* a [`ButcherTableau`](@ref) `tableau`, specifying the integrator coefficients;
* an [`OdeResultsSink`](@ref) `sink` which processes the results of the integration procedure at each time step.
"""
function MuntheKaasIntegrator{N, T<:Number, F, S<:OdeResultsSink, L}(dynamics!::F, tableau::ButcherTableau{N, T, L}, sink::S)
    MuntheKaasIntegrator{N, T, F, S, L}(dynamics!, tableau, sink)
end

num_stages{N}(::MuntheKaasIntegrator{N}) = N
eltype{N, T}(::MuntheKaasIntegrator{N, T}) = T

"""
$(SIGNATURES)

Take a single integration step.

`state` must be of a type for which the following functions are defined:
* `configuration(state)`, returns the configuration vector in global coordinates;
* `velocity(state)`, returns the velocity vector;
* `additional_state(state)`, returns the vector of additional states;
* `set_velocity!(state, v)`, sets velocity vector to `v`;
* `set_additional_state!(state, s)`, sets vector of additional states to `s`;
* `global_coordinates!(state, q0, ϕ)`, sets global coordinates in state based on local coordinates `ϕ` centered around global coordinates `q0`;
* `local_coordinates!(state, ϕ, ϕd, q0)`, converts state's global configuration `q` and velocity `v` to local coordinates centered around global coordinates `q0`.
"""
function step(integrator::MuntheKaasIntegrator, t::Real, state, Δt::Real)
    tableau = integrator.tableau
    stages = integrator.stages
    n = num_stages(integrator)

    # Use current configuration as the configuration around which the local coordinates for this step will be centered.
    q0, v0, s0 = stages.q0, stages.vstep, stages.sstep
    copy!(q0, configuration(state))
    copy!(v0, velocity(state))
    copy!(s0, additional_state(state))

    # Compute integrator stages.
    for i = 1 : n
        # Update local coordinates and velocities
        ϕ = stages.ϕs[i]
        v = stages.vs[i]
        s = stages.ss[i]
        fill!(ϕ, zero(eltype(ϕ)))
        copy!(v, v0)
        copy!(s, s0)
        for j = 1 : i - 1
            aij = tableau.a[i, j]
            if aij != zero(aij)
                weight = Δt * aij
                scaleadd!(ϕ, stages.ϕds[j], weight)
                scaleadd!(v, stages.vds[j], weight)
                scaleadd!(s, stages.sds[j], weight)
            end
        end

        # Convert from local to global coordinates and set state
        # TODO: multiple setdirty! calls when using MechanismState:
        global_coordinates!(state, q0, ϕ)
        set_velocity!(state, v)
        set_additional_state!(state, s)

        # Dynamics in global coordinates
        vd = stages.vds[i]
        sd = stages.sds[i]
        integrator.dynamics!(vd, sd, t + Δt * tableau.c[i], state)

        # Convert back to local coordinates
        ϕd = stages.ϕds[i] # TODO: ϕ not actually needed, just ϕd!
        local_coordinates!(state, ϕ, ϕd, q0)
    end

    # Combine stages
    ϕ = stages.ϕstep
    fill!(ϕ, zero(eltype(ϕ)))
    v = stages.vstep # already initialized to v0
    s = stages.sstep # already initialized to s0
    for i = 1 : n
        weight = tableau.b[i] * Δt
        scaleadd!(ϕ, stages.ϕds[i], weight)
        scaleadd!(v, stages.vds[i], weight)
        scaleadd!(s, stages.sds[i], weight)
    end

    # Convert from local to global coordinates
    # TODO: multiple setdirty! calls when using MechanismState:
    global_coordinates!(state, q0, ϕ)
    set_velocity!(state, v)
    set_additional_state!(state, s)

    nothing
end

# Throttle a loop by sleeping periodically (with minimum duration `minsleeptime`), so that `t` doesn't increase faster than `maxrate`.
# Note: sleep function rounds to 1e-3. Decreasing minsleeptime makes throttling smoother (more frequent, shorter pauses), but reduces accuracy due to rounding error.
macro throttle(t, maxrate, minsleeptime, loopexpr)
    loopcondition = :($(esc(loopexpr.args[1])))
    loopbody = quote
        $(esc(loopexpr.args[2]))

        if !isinf($(esc(maxrate)))
            Δwalltime = time() - prev_walltime
            Δt = $(esc(t)) - prev_t
            sleeptime = Δt / $(esc(maxrate)) - Δwalltime
            if sleeptime > $(esc(minsleeptime))
                sleep(sleeptime)
                prev_walltime = time()
                prev_t = $(esc(t))
            end
        end
    end
    loop = Expr(loopexpr.head, loopcondition, loopbody)

    quote
        prev_walltime = time()
        prev_t = $(esc(t))
        $loop
    end
end

"""
$(SIGNATURES)

Integrate dynamics from the initial state `state0` at time ``0`` to `finalTime`
using step size `Δt`.
"""
function integrate(integrator::MuntheKaasIntegrator, state0, finalTime, Δt; maxRealtimeRate::Float64 = Inf)
    T = eltype(integrator)
    t = zero(T)
    state = state0
    set_num_positions!(integrator.stages, length(configuration(state)))
    set_num_velocities!(integrator.stages, length(velocity(state)))
    set_num_additional_states!(integrator.stages, length(additional_state(state)))
    initialize(integrator.sink, t, state)
    minSleepTime = 1. / 60. # based on visualizer fps

    @throttle t maxRealtimeRate minSleepTime while t < finalTime
        step(integrator, t, state, Δt)
        t += Δt
        process(integrator.sink, t, state)
    end
end

end # module
