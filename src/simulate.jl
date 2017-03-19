using RigidBodyDynamics.OdeIntegrators

"""
$(SIGNATURES)

Basic `Mechanism` simulation: integrate the state from time ``0`` to `finalTime`
starting from the initial state `state0`. Return a `Vector` of times, as well as
`Vector`s of configuration vectors and velocity vectors at these times.

Uses `MuntheKaasIntegrator`. See [`MuntheKaasIntegrator`](@ref) for a lower
level interface with more options.
"""
function simulate(state0::MechanismState, finalTime; Δt = 1e-4)
    T = cache_eltype(state0)
    result = DynamicsResult(T, state0.mechanism)
    passive_dynamics! = (vd::AbstractArray, sd::AbstractArray, t, state) -> begin
        dynamics!(result, state)
        copy!(vd, result.v̇)
        copy!(sd, result.ṡ)
        nothing
    end
    tableau = runge_kutta_4(Float64)
    storage = ExpandingStorage{Float64}(ceil(Int64, finalTime / Δt * 1.001)) # very rough overestimate of number of time steps
    integrator = MuntheKaasIntegrator(passive_dynamics!, tableau, storage)
    integrate(integrator, state0, finalTime, Δt)
    storage.ts, storage.qs, storage.vs
end
