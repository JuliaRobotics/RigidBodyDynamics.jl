# Benchmarks

To attain maximal performance, it is recommended to pass `-O3` and `--check-bounds=no` as command line flags to `julia`:

```bash
cd RigidBodyDynamics.jl
julia -O3 --check-bounds=no perf/runbenchmarks.jl
```

> **Warning**
> For Julia versions previous to `v1.8`, maximizing performance for the `dynamics!` algorithm requires either setting the number of BLAS threads to 1 (`using LinearAlgebra; BLAS.set_num_threads(1)`) if using OpenBLAS (the default), or compiling Julia with MKL. See [this issue](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/issues/500) for more information.

Run `perf/runbenchmarks.jl` to see benchmark results for the Atlas robot (v5). Results below are for the following scenarios:

1. Compute the joint-space mass matrix.
2. Compute both the mass matrix and a geometric Jacobian from the left hand to the right foot.
3. Do inverse dynamics.
4. Do forward dynamics.

> **Note**
> Results on CI builds are **not at all** representative because of code coverage.

Below are the results for **RBD.jl 2.5.0** (commit [`93a5ea`](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/commit/93a5eaf15a5f6714b1ec1ce621b053542dcb721d)) using **Julia 1.11.1** on an **Apple MacBook Air (M2, 2023)** (16GB RAM, 512GB SSD):

Output of `versioninfo()`:
```
Julia Version 1.11.1
Commit 8f5b7ca12ad (2024-10-16 10:53 UTC)
Build Info:
  Official https://julialang.org/ release
Platform Info:
  OS: macOS (arm64-apple-darwin22.4.0)
  CPU: 8 × Apple M2
  WORD_SIZE: 64
  LLVM: libLLVM-16.0.6 (ORCJIT, apple-m2)
Threads: 1 default, 0 interactive, 1 GC (on 4 virtual cores)
```

> **Note**
> This is a different machine than the one that was used for earlier benchmarks.

Mass matrix ([`mass_matrix!`](@ref)):
```
BenchmarkTools.Trial: 10000 samples with 10 evaluations.
 Range (min … max):  3.728 μs …  10.024 μs  ┊ GC (min … max): 0.00% … 0.00%
 Time  (median):     3.874 μs               ┊ GC (median):    0.00%
 Time  (mean ± σ):   3.903 μs ± 180.208 ns  ┊ GC (mean ± σ):  0.00% ± 0.00%
 Memory estimate: 0 bytes, allocs estimate: 0.
```

Mass matrix ([`mass_matrix!`](@ref)) and Jacobian ([`geometric_jacobian!`](@ref)) from left hand to right foot:
```
BenchmarkTools.Trial: 10000 samples with 10 evaluations.
 Range (min … max):  3.941 μs …   9.020 μs  ┊ GC (min … max): 0.00% … 0.00%
 Time  (median):     4.103 μs               ┊ GC (median):    0.00%
 Time  (mean ± σ):   4.135 μs ± 196.842 ns  ┊ GC (mean ± σ):  0.00% ± 0.00%
 Memory estimate: 0 bytes, allocs estimate: 0.
```

Note the low additional cost of computing a Jacobian when the mass matrix is already computed. This is because RigidBodyDynamics.jl caches intermediate computation results.

Inverse dynamics ([`inverse_dynamics!`](@ref)):
```
BenchmarkTools.Trial: 10000 samples with 10 evaluations.
 Range (min … max):  2.736 μs …   5.666 μs  ┊ GC (min … max): 0.00% … 0.00%
 Time  (median):     2.866 μs               ┊ GC (median):    0.00%
 Time  (mean ± σ):   2.882 μs ± 119.781 ns  ┊ GC (mean ± σ):  0.00% ± 0.00%
 Memory estimate: 0 bytes, allocs estimate: 0.
```

Forward dynamics ([`dynamics!`](@ref)):
```
BenchmarkTools.Trial: 10000 samples with 10 evaluations.
 Range (min … max):  9.791 μs …  13.899 μs  ┊ GC (min … max): 0.00% … 0.00%
 Time  (median):     9.874 μs               ┊ GC (median):    0.00%
 Time  (mean ± σ):   9.942 μs ± 292.126 ns  ┊ GC (mean ± σ):  0.00% ± 0.00%
 Memory estimate: 0 bytes, allocs estimate: 0.
```
