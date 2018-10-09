# Spatial vector algebra

## Index

```@index
Pages   = ["spatial.md"]
Order   = [:type, :function, :macro]
```

## Types

### Coordinate frames

```@docs
CartesianFrame3D
CartesianFrame3D(::String)
CartesianFrame3D()
```

### Transforms

```@docs
Transform3D
```

### Points, free vectors

```@docs
Point3D
FreeVector3D
```

### Inertias

```@docs
SpatialInertia
```

### Twists, spatial accelerations

```@docs
Twist
SpatialAcceleration
```

### Momenta, wrenches

```@docs
Momentum
Wrench
```

### Geometric Jacobians

```@docs
GeometricJacobian
```

### Momentum matrices

```@docs
MomentumMatrix
```

## The `@framecheck` macro

```@docs
@framecheck
```

## Functions

```@autodocs
Modules = [RigidBodyDynamics.Spatial]
Order   = [:function]
```
