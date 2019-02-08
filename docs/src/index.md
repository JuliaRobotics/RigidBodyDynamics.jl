# RigidBodyDynamics

RigidBodyDynamics implements various rigid body dynamics and kinematics algorithms.

## Design features

Some of the key design features of this package are:

* pure Julia implementation, enabling seamless support for e.g. automatic differentiation using [ForwardDiff.jl](https://github.com/JuliaDiff/ForwardDiff.jl) and symbolic dynamics using [SymPy.jl](https://github.com/JuliaPy/SymPy.jl).
* easy creation and modification of general rigid body mechanisms.
* basic parsing of and writing to the [URDF](http://wiki.ros.org/urdf) file format.
* extensive checks that verify that coordinate systems match before computation, with the goal of making reference frame mistakes impossible
* flexible caching of intermediate results to prevent doing double work
* fairly small codebase and few dependencies
* singularity-free rotation parameterizations

## Functionality

Current functionality of RigidBodyDynamics.jl includes:

* kinematics/transforming points and free vectors from one coordinate system to another
* transforming wrenches, momenta (spatial force vectors) and twists and their derivatives (spatial motion vectors) from one coordinate system to another
* relative twists/spatial accelerations between bodies
* kinetic/potential energy
* center of mass
* geometric/basic/spatial Jacobians
* momentum
* momentum matrix
* momentum rate bias (= momentum matrix time derivative multiplied by joint velocity vector)
* mass matrix (composite rigid body algorithm)
* inverse dynamics (recursive Newton-Euler)
* dynamics
* simulation, either using an off-the-shelf ODE integrator or using an included custom Munthe-Kaas integrator that properly handles second-order ODEs defined on a manifold.

Closed-loop systems (parallel mechanisms) are supported, with optional Baumgarte stabilization of the loop joint constraints. Support for contact is very limited (possibly subject to major changes in the future), implemented using penalty methods.

## Installation

### Installing Julia

Download links and more detailed instructions are available on the [Julia website](http://julialang.org/). The latest version of RigidBodyDynamics.jl requires Julia 0.7, but we recommend downloading 1.0 (the latest stable Julia release at the time of writing). Version 0.7 of RigidBodyDynamics.jl is the last to support Julia 0.6.

!!! warning

    Do **not** use `apt-get` or `brew` to install Julia, as the versions provided by these package managers tend to be out of date.

### Installing RigidBodyDynamics

To install the latest tagged release of RigidBodyDynamics, start Julia and enter `Pkg` mode by pressing `]`. Then simply run

```julia
add RigidBodyDynamics
```

To use the latest master version and work on the bleeding edge (generally, not recommended), instead run

```julia
add RigidBodyDynamics#master
```

A third option is to clone the repository (to the directory printed by `julia -e 'import Pkg; println(Pkg.devdir())'`):

```julia
dev RigidBodyDynamics
```

## About

This library was inspired by [IHMCRoboticsToolkit](https://bitbucket.org/ihmcrobotics/ihmc-open-robotics-software) and by [Drake](http://drake.mit.edu).

Most of the nomenclature used and algorithms implemented by this package stem
from the following resources:

* Murray, Richard M., et al. *A mathematical introduction to robotic manipulation*. CRC press, 1994.
* Featherstone, Roy. *Rigid body dynamics algorithms*. Springer, 2008.
* Duindam, Vincent. *Port-based modeling and control for efficient bipedal walking robots*. Diss. University of Twente, 2006.


## Contents

```@contents
Pages = [
  "spatial.md",
  "joints.md",
  "rigidbody.md",
  "mechanism.md",
  "mechanismstate.md",
  "algorithms.md",
  "caches.md",
  "simulation.md",
  "urdf.md",
  "benchmarks.md"]
Depth = 2
```

## Citing this library

```bibtex
@misc{rigidbodydynamicsjl,
 author = "Twan Koolen and contributors",
 title = "RigidBodyDynamics.jl",
 year = 2016,
 url = "https://github.com/JuliaRobotics/RigidBodyDynamics.jl"
}
```
