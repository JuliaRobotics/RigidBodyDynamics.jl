# Benchmarks

To attain maximal performance, it is recommended to pass `-O3`, `--check-bounds=no` as command line flags to `julia`. As of Julia 1.1, maximizing performance for the `dynamics!` algorithm requires either setting the number of BLAS threads to 1 (`using LinearAlgebra; BLAS.set_num_threads(1)`) if using OpenBLAS (the default), or compiling Julia with MKL. See [this issue](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/issues/500) for more information.

Run `perf/runbenchmarks.jl` to see benchmark results for the Atlas robot (v5). Results below are for the following scenarios:

1. Compute the joint-space mass matrix.
2. Compute both the mass matrix and a geometric Jacobian from the left hand to the right foot.
3. Do inverse dynamics.
4. Do forward dynamics.

Note that results on CI builds are **not at all** representative because of code coverage. Results on a reasonably fast laptop at commit [870bea6](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/commit/870bea668d5b11ce0555fa0552592d2c3cb15c54):

Output of `versioninfo()`:

```
Julia Version 1.5.3
Commit 788b2c77c1 (2020-11-09 13:37 UTC)
Platform Info:
  OS: macOS (x86_64-apple-darwin18.7.0)
  CPU: Intel(R) Core(TM) i7-8850H CPU @ 2.60GHz
  WORD_SIZE: 64
  LIBM: libopenlibm
  LLVM: libLLVM-9.0.1 (ORCJIT, skylake)
```

Note that this is a different machine than the one that was used for earlier benchmarks.

Mass matrix:

```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     4.415 μs (0.00% GC)
  median time:      4.579 μs (0.00% GC)
  mean time:        4.916 μs (0.00% GC)
  maximum time:     19.794 μs (0.00% GC)
```

Mass matrix and Jacobian from left hand to right foot:

```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     4.860 μs (0.00% GC)
  median time:      4.982 μs (0.00% GC)
  mean time:        5.399 μs (0.00% GC)
  maximum time:     24.712 μs (0.00% GC)
```

Note the low additional cost of computing a Jacobian when the mass matrix is already computed. This is because RigidBodyDynamics.jl caches intermediate computation results.

Inverse dynamics:

```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     4.256 μs (0.00% GC)
  median time:      4.541 μs (0.00% GC)
  mean time:        4.831 μs (0.00% GC)
  maximum time:     21.625 μs (0.00% GC)
```

Forward dynamics:

```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     13.600 μs (0.00% GC)
  median time:      14.419 μs (0.00% GC)
  mean time:        16.071 μs (0.00% GC)
  maximum time:     55.328 μs (0.00% GC)
```
