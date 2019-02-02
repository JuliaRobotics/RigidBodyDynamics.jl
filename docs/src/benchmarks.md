# Benchmarks

To attain maximal performance, it is recommended to pass `-O3`, `--check-bounds=no` as command line flags to `julia`. As of Julia 1.1, maximizing performance for the `dynamics!` algorithm requires either setting the number of BLAS threads to 1 (`using LinearAlgebra; BLAS.set_num_threads(1)`) if using OpenBLAS (the default), or compiling Julia with MKL. See [this issue](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/issues/500) for more information.

Run `perf/runbenchmarks.jl` to see benchmark results for the Atlas robot (v5). Results below are for the following scenarios:

1. Compute the joint-space mass matrix.
2. Compute both the mass matrix and a geometric Jacobian from the left hand to the right foot.
3. Do inverse dynamics.
4. Do forward dynamics.

Note that results on Travis builds are **not at all** representative because of code coverage. Results on a reasonably fast machine at commit [32a2ccb](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/tree/32a2ccbdf0e432bfdde77e24feabc2b641a3565a):

Output of `versioninfo()`:

```
Julia Version 1.1.0
Commit 80516ca202 (2019-01-21 21:24 UTC)
Platform Info:
  OS: Linux (x86_64-pc-linux-gnu)
  CPU: Intel(R) Core(TM) i7-6950X CPU @ 3.00GHz
  WORD_SIZE: 64
  LIBM: libopenlibm
  LLVM: libLLVM-6.0.1 (ORCJIT, broadwell)
```

Mass matrix:

```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     5.379 μs (0.00% GC)
  median time:      5.795 μs (0.00% GC)
  mean time:        5.770 μs (0.00% GC)
  maximum time:     9.258 μs (0.00% GC)
```

Mass matrix and Jacobian from left hand to right foot:

```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     5.754 μs (0.00% GC)
  median time:      5.856 μs (0.00% GC)
  mean time:        5.940 μs (0.00% GC)
  maximum time:     10.100 μs (0.00% GC)
```

Note the low additional cost of computing a Jacobian when the mass matrix is already computed. This is because RigidBodyDynamics.jl caches intermediate computation results.

Inverse dynamics:

```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     5.623 μs (0.00% GC)
  median time:      5.727 μs (0.00% GC)
  mean time:        5.886 μs (0.00% GC)
  maximum time:     9.637 μs (0.00% GC)
```

Forward dynamics:

```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     17.721 μs (0.00% GC)
  median time:      18.862 μs (0.00% GC)
  mean time:        18.755 μs (0.00% GC)
  maximum time:     24.910 μs (0.00% GC)
```
