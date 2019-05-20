# RigidBodyDynamics.jl

[![Build Status](https://travis-ci.org/JuliaRobotics/RigidBodyDynamics.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/RigidBodyDynamics.jl)
[![codecov.io](https://codecov.io/github/JuliaRobotics/RigidBodyDynamics.jl/coverage.svg?branch=master)](https://codecov.io/github/JuliaRobotics/RigidBodyDynamics.jl?branch=master)
[![](https://img.shields.io/badge/docs-latest-blue.svg)](https://JuliaRobotics.github.io/RigidBodyDynamics.jl/dev)
[![](https://img.shields.io/badge/docs-stable-blue.svg)](https://JuliaRobotics.github.io/RigidBodyDynamics.jl/stable)

RigidBodyDynamics.jl is a rigid body dynamics library in pure Julia. It aims to be **user friendly** and [**performant**](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/blob/master/docs/src/benchmarks.md), but also **generic** in the sense that the algorithms can be called with inputs of any (suitable) scalar types. This means that if fast numeric dynamics evaluations are required, a user can supply `Float64` or `Float32` inputs. However, if symbolic quantities are desired for analysis purposes, they can be obtained by calling the algorithms with e.g. [`SymPy.Sym`](https://github.com/JuliaPy/SymPy.jl) inputs. If gradients are required, e.g. the [`ForwardDiff.Dual`](https://github.com/JuliaDiff/ForwardDiff.jl) type, which implements forward-mode [automatic differentiation](https://en.wikipedia.org/wiki/Automatic_differentiation), can be used.

See the [latest stable documentation](https://JuliaRobotics.github.io/RigidBodyDynamics.jl/stable/) for a list of features, installation instructions, and a quick-start guide. Installation should only take a couple of minutes, including installing Julia itself. See the [notebooks directory](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/tree/master/notebooks) for some usage examples.


## Related packages

RigidBodyDynamics.jl is part of the [JuliaRobotics GitHub organization](http://www.juliarobotics.org/).

Packages built on top of RigidBodyDynamics.jl include:

* [RigidBodySim.jl](https://github.com/JuliaRobotics/RigidBodySim.jl) - simulator built on top of RigidBodyDynamics.jl.
* [MeshCatMechanisms.jl](https://github.com/JuliaRobotics/MeshCatMechanisms.jl) - 3D visualization of articulated mechanisms using MeshCat.jl (built on top of [three.js](https://threejs.org/)) and RigidBodyDynamics.jl.
* [RigidBodyTreeInspector.jl](https://github.com/rdeits/RigidBodyTreeInspector.jl) - 3D visualization of RigidBodyDynamics.jl `Mechanism`s using [Director](https://github.com/RobotLocomotion/director).
* [MotionCaptureJointCalibration.jl](https://github.com/JuliaRobotics/MotionCaptureJointCalibration.jl) - kinematic calibration for robots using motion capture data, built on top of RigidBodyDynamics.jl
* [QPControl.jl](https://github.com/tkoolen/QPControl.jl) - quadratic-programming-based robot controllers implemented using RigidBodyDynamics.jl.
* [StrandbeestRobot.jl](https://github.com/rdeits/StrandbeestRobot.jl) - simulations of a 12-legged parallel walking mechanism inspired by Theo Jansens's [Strandbeest](https://www.strandbeest.com/) using RigidBodyDynamics.jl.


## Talks / publications

* May 20, 2019: paper at ICRA 2019: [Julia for robotics: simulation and real-time control in a
high-level programming language](https://www.researchgate.net/publication/331983442_Julia_for_robotics_simulation_and_real-time_control_in_a_high-level_programming_language).
* August 10, 2018: Robin Deits gave [a talk](https://www.youtube.com/watch?v=dmWQtI3DFFo) at JuliaCon 2018 demonstrating RigidBodyDynamics.jl and related packages.
* August 23, 2017: a video of a JuliaCon 2017 talk given by Robin Deits and Twan Koolen on using Julia for robotics [has been uploaded](https://www.youtube.com/watch?v=gPYc77M90Qg). It includes a brief demo of RigidBodyDynamics.jl and RigidBodyTreeInspector.jl. Note that RigidBodyDynamics.jl performance has significantly improved since this talk. The margins of the slides have unfortunately been cut off somewhat in the video.
