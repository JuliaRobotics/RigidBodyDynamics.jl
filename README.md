# RigidBodyDynamics.jl

[![Build Status](https://travis-ci.org/tkoolen/RigidBodyDynamics.jl.svg?branch=master)](https://travis-ci.org/tkoolen/RigidBodyDynamics.jl)
[![codecov.io](https://codecov.io/github/tkoolen/RigidBodyDynamics.jl/coverage.svg?branch=master)](https://codecov.io/github/tkoolen/RigidBodyDynamics.jl?branch=master)

RigidBodyDynamics.jl is a small rigid body dynamics library for Julia. It was inspired by the [IHMCRoboticsToolkit](https://bitbucket.org/ihmcrobotics/ihmc-open-robotics-software) from the Institute of Human and Machine Cognition, and by [Drake](http://drake.mit.edu).

## Key features
* easy creation of general rigid body mechanisms
* extensive checks that verify that coordinate systems match before computation: the goal is to make reference frame mistakes impossible
* support for automatic differentiation using e.g. [ForwardDiff.jl](https://github.com/JuliaDiff/ForwardDiff.jl)
* flexible caching of intermediate results to prevent doing double work
* fairly small codebase and few dependencies

## Current functionality
* kinematics/transforming points and free vectors from one coordinate system to another
* transforming wrenches, momenta (spatial force vectors) and twists and their derivatives (spatial motion vectors) from one coordinate system to another
* kinetic/potential energy
* center of mass
* geometric/basic/spatial Jacobians
* momentum matrix
* mass matrix (composite rigid body algorithm)
* inverse dynamics (recursive Newton-Euler)

Since the inverse dynamics algorithm can also be used to compute just the Coriolis and gravitational torques, it could be used in conjunction with the mass matrix algorithm to do forward dynamics. The articulated-body algorithm for forward dynamics has not yet been implemented.

Closed loop systems are not yet supported.

## Citing this library
```
@misc{rigidbodydynamicsjl,
 author = "Twan Koolen",
 title = "RigidBodyDynamics.jl",
 year = 2016,
 url = "https://github.com/tkoolen/RigidBodyDynamics.jl"
}
```
