# Benchmarks

To attain maximal performance, it is recommended to pass `-O3` and `--check-bounds=no` as command line flags to `julia`.

> **Warning**
> For Julia versions previous to `v1.8`, maximizing performance for the `dynamics!` algorithm requires either setting the number of BLAS threads to 1 (`using LinearAlgebra; BLAS.set_num_threads(1)`) if using OpenBLAS (the default), or compiling Julia with MKL. See [this issue](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/issues/500) for more information.

Run `perf/runbenchmarks.jl` to see benchmark results for the Atlas robot (v5). Results below are for the following scenarios:

1. Compute the joint-space mass matrix.
2. Compute both the mass matrix and a geometric Jacobian from the left hand to the right foot.
3. Do inverse dynamics.
4. Do forward dynamics.

> **Note**
> Results on CI builds are **not at all** representative because of code coverage.

Below are the results for **RBD.jl 2.3.2** (commit [`b9ef1d`](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/commit/b9ef1d6974beff4d4fbe7dffc6dbfa65f71e0132)) using **Julia 1.8.5** on an **Apple MacBook Air (M1, 2020)** (8GB RAM, 512GB SSD):

Output of `versioninfo()`:
```
Julia Version 1.8.5
Commit 17cfb8e65e* (2023-01-08 06:45 UTC)
Platform Info:
  OS: macOS (arm64-apple-darwin22.1.0)
  CPU: 8 × Apple M1
  WORD_SIZE: 64
  LIBM: libopenlibm
  LLVM: libLLVM-13.0.1 (ORCJIT, apple-m1)
  Threads: 1 on 4 virtual cores
```

> **Note**
> This is a different machine than the one that was used for earlier benchmarks.

Mass matrix ([`mass_matrix!`](@ref)):
```
BenchmarkTools.Trial: 10000 samples with 10 evaluations.
 Range (min … max):  3.796 μs …  6.183 μs  ┊ GC (min … max): 0.00% … 0.00%
 Time  (median):     3.858 μs              ┊ GC (median):    0.00%
 Time  (mean ± σ):   3.879 μs ± 93.310 ns  ┊ GC (mean ± σ):  0.00% ± 0.00%
 Memory estimate: 0 bytes, allocs estimate: 0.
```

Mass matrix ([`mass_matrix!`](@ref)) and Jacobian ([`geometric_jacobian!`](@ref)) from left hand to right foot:
```
BenchmarkTools.Trial: 10000 samples with 10 evaluations.
 Range (min … max):  4.042 μs …   6.763 μs  ┊ GC (min … max): 0.00% … 0.00%
 Time  (median):     4.167 μs               ┊ GC (median):    0.00%
 Time  (mean ± σ):   4.187 μs ± 115.303 ns  ┊ GC (mean ± σ):  0.00% ± 0.00%
 Memory estimate: 0 bytes, allocs estimate: 0.
```

Note the low additional cost of computing a Jacobian when the mass matrix is already computed. This is because RigidBodyDynamics.jl caches intermediate computation results.

Inverse dynamics ([`inverse_dynamics!`](@ref)):
```
BenchmarkTools.Trial: 10000 samples with 10 evaluations.
 Range (min … max):  2.700 μs …  5.700 μs  ┊ GC (min … max): 0.00% … 0.00%
 Time  (median):     2.767 μs              ┊ GC (median):    0.00%
 Time  (mean ± σ):   2.773 μs ± 79.491 ns  ┊ GC (mean ± σ):  0.00% ± 0.00%
 Memory estimate: 0 bytes, allocs estimate: 0.
```

Forward dynamics ([`dynamics!`](@ref)):
```
BenchmarkTools.Trial: 10000 samples with 10 evaluations.
 Range (min … max):  10.554 μs …  23.692 μs  ┊ GC (min … max): 0.00% … 0.00%
 Time  (median):     10.667 μs               ┊ GC (median):    0.00%
 Time  (mean ± σ):   10.703 μs ± 212.557 ns  ┊ GC (mean ± σ):  0.00% ± 0.00%
 Memory estimate: 0 bytes, allocs estimate: 0.
```
