# Simulation

## Index

```@index
Pages   = ["simulation.md"]
Order   = [:type, :function]
```

## Basic simulation

```@docs
simulate
```

## Lower level ODE integration interface

```@docs
MuntheKaasIntegrator
MuntheKaasIntegrator(state::X, dynamics!::F, tableau::ButcherTableau{N, T, L}, sink::S) where {N, T, F, S<:OdeResultsSink, X, L}
ButcherTableau
OdeResultsSink
RingBufferStorage
ExpandingStorage
```

```@autodocs
Modules = [RigidBodyDynamics.OdeIntegrators]
Order   = [:function]
Pages   = ["ode_integrators.jl"]
```
