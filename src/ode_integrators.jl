module OdeIntegrators

using LinearAlgebra
using RigidBodyDynamics
using StaticArrays
using DocStringExtensions
using LoopThrottle

export runge_kutta_4,
    MuntheKaasIntegrator,
    ButcherTableau,
    OdeResultsSink,
    RingBufferStorage,
    ExpandingStorage,
    integrate,
    step

"""
$(TYPEDEF)

A [Butcher tableau](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods#Explicit_Runge.E2.80.93Kutta_methods).
"""
struct ButcherTableau{N, T, L}
    a::SMatrix{N, N, T, L}
    b::SVector{N, T}
    c::SVector{N, T}
    explicit::Bool

    function ButcherTableau(a::AbstractMatrix{T}, b::AbstractVector{T}) where {T}
        N = size(a, 1)
        L = N^2
        @assert N > 0
        @assert size(a, 2) == N
        @assert length(b) == N
        c = vec(sum(a, dims=2))
        explicit = all(triu(a) .== 0)
        new{N, T, L}(SMatrix{N, N}(a), SVector{N}(b), SVector{N}(c), explicit)
    end
end
num_stages(::ButcherTableau{N}) where {N} = N
isexplicit(tableau::ButcherTableau) = tableau.explicit

"""
$(SIGNATURES)

Return the Butcher tableau for the standard fourth order Runge-Kutta integrator.
"""
function runge_kutta_4(scalar_type::Type{T}) where {T}
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
mutable struct RingBufferStorage{T, Q<:AbstractVector, V<:AbstractVector} <: OdeResultsSink
    ts::Vector{T}
    qs::Vector{Q}
    vs::Vector{V}
    last_index::Int64

    function RingBufferStorage{T}(state, n) where {T}
        Q = typeof(configuration(state))
        V = typeof(velocity(state))
        ts = Vector{T}(undef, n)
        qs = [similar(configuration(state)) for i in 1 : n]
        vs = [similar(velocity(state)) for i in 1 : n]
        new{T, Q, V}(ts, qs, vs, 0)
    end
end
Base.eltype(storage::RingBufferStorage{T}) where {T} = T
Base.length(storage::RingBufferStorage) = length(storage.ts)

function initialize(storage::RingBufferStorage, t, state)
    process(storage, t, state)
end

function process(storage::RingBufferStorage, t, state)
    index = storage.last_index % length(storage) + 1
    storage.ts[index] = t
    copyto!(storage.qs[index], configuration(state))
    copyto!(storage.vs[index], velocity(state))
    storage.last_index = index
    nothing
end

"""
$(TYPEDEF)

An `OdeResultsSink` that stores the state at each integration time step in
`Vectors` that may expand.
"""
mutable struct ExpandingStorage{T, Q<:AbstractVector, V<:AbstractVector} <: OdeResultsSink
    ts::Vector{T}
    qs::Vector{Q}
    vs::Vector{V}

    function ExpandingStorage{T}(state, n) where {T}
        Q = typeof(configuration(state))
        V = typeof(velocity(state))
        ts = T[]; sizehint!(ts, n)
        qs = Q[]; sizehint!(qs, n)
        vs = V[]; sizehint!(vs, n)
        new{T, Q, V}(ts, qs, vs)
    end
end
Base.eltype(storage::ExpandingStorage{T}) where {T} = T
Base.length(storage::ExpandingStorage) = length(storage.ts)

initialize(storage::ExpandingStorage, t, state) = process(storage, t, state)

function process(storage::ExpandingStorage, t, state)
    push!(storage.ts, t)
    q = similar(configuration(state))
    copyto!(q, configuration(state))
    push!(storage.qs, q)
    v = similar(velocity(state))
    copyto!(v, velocity(state))
    push!(storage.vs, v)
    nothing
end

mutable struct MuntheKaasStageCache{N, T, Q<:AbstractVector, V<:AbstractVector, S<:AbstractVector}
    q0::Q # global coordinates
    vs::SVector{N, V} # velocity vector for each stage
    vds::SVector{N, V} # time derivatives of vs
    ϕs::SVector{N, V} # local coordinates around q0 for each stage
    ϕds::SVector{N, V} # time derivatives of ϕs
    ss::SVector{N, S} # additional state for each stage
    sds::SVector{N, S} # time derivatives of ss
    ϕstep::V # local coordinates around q0 after complete step
    vstep::V # velocity after complete step
    sstep::S # additional state after complete step

    function MuntheKaasStageCache{N, T}(state) where {N, T}
        q0 = similar(configuration(state), T)
        vs = SVector{N}((similar(velocity(state), T) for i in 1 : N)...)
        vds = SVector{N}((similar(velocity(state), T) for i in 1 : N)...)
        ϕs = SVector{N}((similar(velocity(state), T) for i in 1 : N)...)
        ϕds = SVector{N}((similar(velocity(state), T) for i in 1 : N)...)
        ss = SVector{N}((similar(additional_state(state), T) for i in 1 : N)...)
        sds = SVector{N}((similar(additional_state(state), T) for i in 1 : N)...)
        ϕstep = similar(velocity(state), T)
        vstep = similar(velocity(state), T)
        sstep = similar(additional_state(state), T)
        new{N, T, typeof(q0), typeof(ϕstep), typeof(sstep)}(q0, vs, vds, ϕs, ϕds, ss, sds, ϕstep, vstep, sstep)
    end
end

"""
$(TYPEDEF)

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
struct MuntheKaasIntegrator{N, T, F, S<:OdeResultsSink, X, L, M<:MuntheKaasStageCache{N, T}}
    dynamics!::F # dynamics!(vd, sd, t, state), sets vd (time derivative of v) and sd (time derivative of s) given time t and state
    tableau::ButcherTableau{N, T, L}
    sink::S
    state::X
    stages::M

    @doc """
    $(SIGNATURES)

    Create a `MuntheKaasIntegrator` given:

    * a callable `dynamics!(vd, t, state)` that updates the joint acceleration vector `vd` at time `t` and in state `state`;
    * a [`ButcherTableau`](@ref) `tableau`, specifying the integrator coefficients;
    * an [`OdeResultsSink`](@ref) `sink` which processes the results of the integration procedure at each time step.

    `state` must be of a type for which the following functions are defined:

    * `configuration(state)`, returns the configuration vector in global coordinates;
    * `velocity(state)`, returns the velocity vector;
    * `additional_state(state)`, returns the vector of additional states;
    * `set_velocity!(state, v)`, sets velocity vector to `v`;
    * `set_additional_state!(state, s)`, sets vector of additional states to `s`;
    * `global_coordinates!(state, q0, ϕ)`, sets global coordinates in state based on local coordinates `ϕ` centered around global coordinates `q0`;
    * `local_coordinates!(ϕ, ϕd, state, q0)`, converts state's global configuration `q` and velocity `v` to local coordinates centered around global coordinates `q0`.
    """ ->
    function MuntheKaasIntegrator(state::X, dynamics!::F, tableau::ButcherTableau{N, T, L}, sink::S) where {N, T, F, S<:OdeResultsSink, X, L}
        @assert isexplicit(tableau)
        stages = MuntheKaasStageCache{N, T}(state)
        new{N, T, F, S, X, L, typeof(stages)}(dynamics!, tableau, sink, state, stages)
    end
end

num_stages(::MuntheKaasIntegrator{N}) where {N} = N
Base.eltype(::MuntheKaasIntegrator{N, T}) where {N, T} = T

"""
$(SIGNATURES)

Take a single integration step.
"""
function step(integrator::MuntheKaasIntegrator, t::Real, Δt::Real)
    tableau = integrator.tableau
    stages = integrator.stages
    n = num_stages(integrator)

    # Use current configuration as the configuration around which the local coordinates for this step will be centered.
    q0, v0, s0 = stages.q0, stages.vstep, stages.sstep
    state = integrator.state
    q0 .= configuration(state)
    v0 .= velocity(state)
    s0 .= additional_state(state)

    # Compute integrator stages.
    for i = 1 : n
        # Update local coordinates and velocities
        ϕ = stages.ϕs[i]
        v = stages.vs[i]
        s = stages.ss[i]
        ϕ .= 0
        v .= v0
        s .= s0
        for j = 1 : i - 1
            aij = tableau.a[i, j]
            if aij != zero(aij)
                weight = Δt * aij
                ϕ .= muladd.(weight, stages.ϕds[j], ϕ)
                v .= muladd.(weight, stages.vds[j], v)
                s .= muladd.(weight, stages.sds[j], s)
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
        integrator.dynamics!(vd, sd, muladd(tableau.c[i], Δt, t), state)

        # Convert back to local coordinates
        ϕd = stages.ϕds[i] # TODO: ϕ not actually needed, just ϕd!
        local_coordinates!(ϕ, ϕd, state, q0)
    end

    # Combine stages
    ϕ = stages.ϕstep
    ϕ .= 0
    v = stages.vstep # already initialized to v0
    s = stages.sstep # already initialized to s0
    for i = 1 : n
        weight = tableau.b[i] * Δt
        ϕ .= muladd.(weight, stages.ϕds[i], ϕ)
        v .= muladd.(weight, stages.vds[i], v)
        s .= muladd.(weight, stages.sds[i], s)
    end

    # Convert from local to global coordinates
    # TODO: multiple setdirty! calls when using MechanismState:
    global_coordinates!(state, q0, ϕ)
    set_velocity!(state, v)
    set_additional_state!(state, s)

    nothing
end

"""
$(SIGNATURES)

Integrate dynamics from the initial state at time ``0`` to `final_time`
using step size `Δt`.
"""
function integrate(integrator::MuntheKaasIntegrator, final_time, Δt; max_realtime_rate::Float64 = Inf)
    t = zero(eltype(integrator))
    state = integrator.state
    initialize(integrator.sink, t, state)
    @throttle t while t < final_time
        step(integrator, t, Δt)
        t += Δt
        process(integrator.sink, t, state)
    end max_rate=max_realtime_rate min_sleep_time=1/60.
end

end # module
