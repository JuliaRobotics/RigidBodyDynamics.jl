using RigidBodyDynamics.OdeIntegrators

"""
$(SIGNATURES)

A trivial controller that simply sets the torques to zero.
"""
function zero_torque!(torques::AbstractVector, t, state::MechanismState)
    torques .= 0
end

"""
$(SIGNATURES)

Basic `Mechanism` simulation: integrate the state from time ``0`` to `final_time`
starting from the initial state `state0`. Return a `Vector` of times, as well as
`Vector`s of configuration vectors and velocity vectors at these times.

Optionally, a function (or other callable) can be passed in as the third argument (`control!`).
`control!` will be called at each time step of the simulation and allows you to specify joint torques
given the time and the state of the `Mechanism`. It should look like this:

```julia
function control!(torques::AbstractVector, t, state::MechanismState)
    rand!(torques) # for example
end
```

The integration time step can be specified using the `Δt` keyword argument (defaults to `1e-4`).

$stabilization_gains_doc

Uses `MuntheKaasIntegrator`. See [`RigidBodyDynamics.OdeIntegrators.MuntheKaasIntegrator`](@ref) for a lower
level interface with more options.
"""
function simulate(state0::MechanismState{X}, final_time, control! = zero_torque!;
        Δt = 1e-4, stabilization_gains=default_constraint_stabilization_gains(X)) where X
    T = cache_eltype(state0)
    result = DynamicsResult{T}(state0.mechanism)
    control_torques = similar(velocity(state0))
    closed_loop_dynamics! = let result=result, control_torques=control_torques, stabilization_gains=stabilization_gains # https://github.com/JuliaLang/julia/issues/15276
        function (v̇::AbstractArray, ṡ::AbstractArray, t, state)
            control!(control_torques, t, state)
            dynamics!(result, state, control_torques; stabilization_gains=stabilization_gains)
            copyto!(v̇, result.v̇)
            copyto!(ṡ, result.ṡ)
            nothing
        end
    end
    tableau = runge_kutta_4(T)
    storage = ExpandingStorage{T}(state0, ceil(Int64, final_time / Δt * 1.001)) # very rough overestimate of number of time steps
    integrator = MuntheKaasIntegrator(state0, closed_loop_dynamics!, tableau, storage)
    integrate(integrator, final_time, Δt)
    storage.ts, storage.qs, storage.vs
end
