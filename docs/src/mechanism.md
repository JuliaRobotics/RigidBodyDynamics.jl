# Mechanisms

## Index

```@index
Pages   = ["mechanism.md"]
Order   = [:type, :function]
```

## The `Mechanism` type

```@docs
Mechanism
```

## [Creating and modifying `Mechanism`s](@id mechanism_create)

See also [URDF parsing and writing](@ref) for URDF file format support.

```@docs
Mechanism(root_body; gravity)
```

```@autodocs
Modules = [RigidBodyDynamics]
Order   = [:function]
Pages   = ["mechanism_modification.jl"]
```

## Basic functionality

```@autodocs
Modules = [RigidBodyDynamics]
Order   = [:function]
Pages   = ["mechanism.jl"]
```
