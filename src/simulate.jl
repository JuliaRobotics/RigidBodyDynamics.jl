function simulate(state0::MechanismState, tspan; integrator = ode45, kwargs...)
    q0 = configuration_vector(state0)
    v0 = velocity_vector(state0)
    x0 = [q0; v0]
    T = cache_eltype(state0)
    state = state0
    mechanism = state.mechanism
    result = DynamicsResult(T, mechanism)
    ẋ = Vector{T}(num_positions(mechanism) + num_velocities(mechanism))
    odefun(t, x) = dynamics!(ẋ, result, state, x)
    times, states = integrator(odefun, x0, tspan; kwargs...)
end
