# RigidBodyDynamics.jl

[![Build Status](https://travis-ci.org/JuliaRobotics/RigidBodyDynamics.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/RigidBodyDynamics.jl)
[![codecov.io](https://codecov.io/github/JuliaRobotics/RigidBodyDynamics.jl/coverage.svg?branch=master)](https://codecov.io/github/JuliaRobotics/RigidBodyDynamics.jl?branch=master)
[![](https://img.shields.io/badge/docs-latest-blue.svg)](https://JuliaRobotics.github.io/RigidBodyDynamics.jl/latest)
[![](https://img.shields.io/badge/docs-stable-blue.svg)](https://JuliaRobotics.github.io/RigidBodyDynamics.jl/stable)

RigidBodyDynamics.jl is a rigid body dynamics library in pure Julia. It aims to be **user friendly** and [**performant**](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/blob/master/docs/src/benchmarks.md), but also **generic** in the sense that the algorithms can be called with inputs of any (suitable) scalar types. This means that if fast numeric dynamics evaluations are required, a user can supply `Float64` or `Float32` inputs. However, if symbolic quantities are desired for analysis purposes, they can be obtained by calling the algorithms with e.g. [`SymPy.Sym`](https://github.com/JuliaPy/SymPy.jl) inputs. If gradients are required, e.g. the [`ForwardDiff.Dual`](https://github.com/JuliaDiff/ForwardDiff.jl) type, which implements forward-mode [automatic differentiation](https://en.wikipedia.org/wiki/Automatic_differentiation), can be used.

See the [latest stable documentation](https://JuliaRobotics.github.io/RigidBodyDynamics.jl/stable/) for a list of features, installation instructions, and a quick-start guide. Installation should only take a couple of minutes, including installing Julia itself. See the [notebooks directory](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/tree/master/notebooks) for some usage examples.

## News
* July 10, 2018: [tagged version 0.8.0](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/releases/tag/v0.8.0). Drops support for Julia 0.6. Supports Julia 0.7 with no deprecation warnings (requires at least Julia 0.7.0-beta.105, *does not work on 0.7.0-beta*). Note that a few example notebooks don't work yet due to because of additional dependencies that haven't been updated for Julia 0.7. Also note the significant performance improvements (e.g., ~32% for `mass_matrix!`).
* July 9, 2018: [tagged version 0.7.0](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/releases/tag/v0.7.0). Supports Julia 0.7 (with some deprecation warnings). This is the last version to support Julia 0.6.
* May 14, 2018: [tagged version 0.6.1](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/releases/tag/v0.6.1).
* April 26, 2018: [tagged version 0.6.0](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/releases/tag/v0.6.0).
* March 1, 2018: [tagged version 0.5.0](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/releases/tag/v0.5.0).
* September 20, 2017: [tagged version 0.4.0](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/releases/tag/v0.4.0).
* August 23, 2017: a video of a JuliaCon 2017 talk given by Robin Deits and Twan Koolen on using Julia for robotics [has been uploaded](https://www.youtube.com/watch?v=gPYc77M90Qg). It includes a brief demo of RigidBodyDynamics.jl and RigidBodyTreeInspector.jl. Note that RigidBodyDynamics.jl performance has significantly improved since this talk. The margins of the slides have unfortunately been cut off somewhat in the video.
* August 22, 2017: [tagged version 0.3.0](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/releases/tag/v0.3.0). Drops Julia 0.5 support.
* June 18, 2017: [tagged version 0.2.0](https://github.com/JuliaLang/METADATA.jl/pull/9814). Supports Julia 0.6. This is the last version to support Julia 0.5.
* March 20, 2017: [tagged version 0.1.0](https://github.com/JuliaLang/METADATA.jl/pull/8431).
* February 16, 2017: [tagged version 0.0.6](https://github.com/JuliaLang/METADATA.jl/pull/7989).
* February 14, 2017: [tagged version 0.0.5](https://github.com/JuliaLang/METADATA.jl/pull/7953).
* December 12, 2016: [tagged version 0.0.4](https://github.com/JuliaLang/METADATA.jl/pull/7256).
* December 6, 2016: [tagged version 0.0.3](https://github.com/JuliaLang/METADATA.jl/pull/7183).
* October 28, 2016: [tagged version 0.0.2](https://github.com/JuliaLang/METADATA.jl/pull/6896).
* October 24, 2016: [tagged version 0.0.1](https://github.com/JuliaLang/METADATA.jl/pull/6831).
