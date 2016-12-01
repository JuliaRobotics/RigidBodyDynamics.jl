using RigidBodyDynamics.OdeIntegrators

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
    storage = OdeRingBufferStorage{Float64}(ceil(Int64, finalTime / Δt) + 1)
    integrator = MuntheKaasIntegrator(passive_dynamics!, tableau, storage)
    integrate(integrator, state0, finalTime, Δt)
    storage.ts, storage.qs, storage.vs
end
