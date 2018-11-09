# Joints

## Index

```@index
Pages   = ["joints.md"]
Order   = [:type, :function]
```

## The `Joint` type

```@docs
Joint
```

## Functions

```@autodocs
Modules = [RigidBodyDynamics]
Order   = [:function]
Pages   = ["joint.jl"]
```

## `JointType`s

```@docs
JointType
```

### Fixed

```@docs
Fixed
```

### Revolute

```@docs
Revolute
Revolute(axis)
```

### Prismatic

```@docs
Prismatic
Prismatic(axis)
```

### Planar

```@docs
Planar
Planar{T}(x_axis::AbstractVector, y_axis::AbstractVector) where {T}
```

### QuaternionSpherical

```@docs
QuaternionSpherical
```

### QuaternionFloating

```@docs
QuaternionFloating
```

### SPQuatFloating

```@docs
SPQuatFloating
```

### SinCosRevolute

```@docs
SinCosRevolute
SinCosRevolute(axis)
```
