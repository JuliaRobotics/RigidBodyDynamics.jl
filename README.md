# RigidBodyDynamics.jl

[![Build Status](https://travis-ci.org/tkoolen/RigidBodyDynamics.jl.svg?branch=master)](https://travis-ci.org/tkoolen/RigidBodyDynamics.jl)
[![codecov.io](https://codecov.io/github/tkoolen/RigidBodyDynamics.jl/coverage.svg?branch=master)](https://codecov.io/github/tkoolen/RigidBodyDynamics.jl?branch=master)

RigidBodyDynamics.jl is a small rigid body dynamics library for Julia. It was inspired by the [IHMCRoboticsToolkit](https://bitbucket.org/ihmcrobotics/ihmc-open-robotics-software) from the Institute of Human and Machine Cognition, and by [Drake](http://drake.mit.edu).

## News
* October 24, 2016: [tagged version 0.0.1](https://github.com/JuliaLang/METADATA.jl/pull/6831).

## Key features
* easy creation of general rigid body mechanisms (including basic [URDF](http://wiki.ros.org/urdf) parsing)
* extensive checks that verify that coordinate systems match before computation: the goal is to make reference frame mistakes impossible
* support for automatic differentiation using e.g. [ForwardDiff.jl](https://github.com/JuliaDiff/ForwardDiff.jl)
* flexible caching of intermediate results to prevent doing double work
* fairly small codebase and few dependencies

## Current functionality
* kinematics/transforming points and free vectors from one coordinate system to another
* transforming wrenches, momenta (spatial force vectors) and twists and their derivatives (spatial motion vectors) from one coordinate system to another
* relative twists/spatial accelerations between bodies
* kinetic/potential energy
* center of mass
* geometric/basic/spatial Jacobians
* momentum matrix
* mass matrix (composite rigid body algorithm)
* inverse dynamics (recursive Newton-Euler)
* dynamics

The (forward) dynamics algorithm is currently rudimentary; it just computes the mass matrix and the bias term (using the inverse dynamics algorithm), and then solves for joint accelerations without exploiting sparsity.

Closed loop systems and contact are not yet supported.

## Benchmarks
Run `perf/runbenchmarks.jl` (`-O3` and `--check-bounds=no` flags recommended) to see benchmark results for the Atlas robot (v5) in the following scenarios:

1. Compute the joint-space mass matrix.
1. Do inverse dynamics.
1. Do forward dynamics.

Note that results on Travis builds are **not at all** representative because of code coverage. Results on a recent, fast machine with version 0.0.1:

Output of `versioninfo()`:
```
Julia Version 0.5.0
Commit 3c9d753 (2016-09-19 18:14 UTC)
Platform Info:
  System: Linux (x86_64-pc-linux-gnu)
  CPU: Intel(R) Core(TM) i7-6950X CPU @ 3.00GHz
  WORD_SIZE: 64
  BLAS: libopenblas (USE64BITINT DYNAMIC_ARCH NO_AFFINITY Haswell)
  LAPACK: libopenblas64_
  LIBM: libopenlibm
  LLVM: libLLVM-3.7.1 (ORCJIT, broadwell)
```

Mass matrix:
```
  samples:          10000
  evals/sample:     1
  time tolerance:   5.00%
  memory tolerance: 1.00%
  memory estimate:  115.80 kb
  allocs estimate:  2913
  minimum time:     200.72 μs (0.00% GC)
  median time:      215.32 μs (0.00% GC)
  mean time:        230.30 μs (5.96% GC)
  maximum time:     3.45 ms (92.18% GC)
```

Inverse dynamics:
```
  "inverse_dynamics" => BenchmarkTools.Trial: 
  samples:          10000
  evals/sample:     1
  time tolerance:   5.00%
  memory tolerance: 1.00%
  memory estimate:  60.67 kb
  allocs estimate:  1246
  minimum time:     243.40 μs (0.00% GC)
  median time:      250.57 μs (0.00% GC)
  mean time:        258.38 μs (2.53% GC)
  maximum time:     2.96 ms (90.01% GC)
```

Forward dynamics:
```
  samples:          10000
  evals/sample:     1
  time tolerance:   5.00%
  memory tolerance: 1.00%
  memory estimate:  133.08 kb
  allocs estimate:  3355
  minimum time:     283.42 μs (0.00% GC)
  median time:      297.46 μs (0.00% GC)
  mean time:        313.83 μs (5.05% GC)
  maximum time:     3.91 ms (90.38% GC)
```


## Related packages
* [RigidBodyTreeInspector.jl](https://github.com/rdeits/RigidBodyTreeInspector.jl) - 3D visualization of RigidBodyDynamics.jl `Mechanism`s using [Director](https://github.com/RobotLocomotion/director).

## Citing this library
```
@misc{rigidbodydynamicsjl,
 author = "Twan Koolen and contributors",
 title = "RigidBodyDynamics.jl",
 year = 2016,
 url = "https://github.com/tkoolen/RigidBodyDynamics.jl"
}
```
