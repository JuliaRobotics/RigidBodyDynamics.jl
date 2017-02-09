using RigidBodyDynamics.OdeIntegrators

"""
$(SIGNATURES)

Simple `Mechanism` simulation: integrate the state from time ``0`` to `finalTime`
starting from the initial state `state0`. Return a `Vector` of times, as well as
`Vector`s of configuration vectors and velocity vectors at these times.

Uses `MuntheKaasIntegrator`. See [`MuntheKaasIntegrator`](@ref) for a lower
level interface with more options.
"""
function simulate(state0::MechanismState, finalTime)
    T = cache_eltype(state0)
    result = DynamicsResult(T, state0.mechanism)
    passive_dynamics! = (vd::AbstractArray, t, state) -> begin
        dynamics!(result, state)
        copy!(vd, result.v̇)
        nothing
    end
    Δt = 1e-4
    tableau = runge_kutta_4(Float64)
    storage = OdeRingBufferStorage{Float64}(ceil(Int64, finalTime / Δt) + 2)
    integrator = MuntheKaasIntegrator(passive_dynamics!, tableau, storage)
    integrate(integrator, state0, finalTime, Δt)
    storage.ts, storage.qs, storage.vs
end
