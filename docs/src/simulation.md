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
