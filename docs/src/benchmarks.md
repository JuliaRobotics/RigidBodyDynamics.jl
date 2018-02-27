# Benchmarks

To get maximal performance, it is recommended to:

1. [Rebuild the Julia system image for your CPU architecture](https://docs.julialang.org/en/stable/devdocs/sysimg/#)
1. Pass `-O3`, `--check-bounds=no`, and `--math-mode=fast` as command line flags to `julia`.

Run `perf/runbenchmarks.jl` to see benchmark results for the Atlas robot (v5) in the following scenarios:

1. Compute the joint-space mass matrix.
1. Compute both the mass matrix and a geometric Jacobian from the left hand to the right foot.
1. Do inverse dynamics.
1. Do forward dynamics.

Note that results on Travis builds are **not at all** representative because of code coverage. Results on a reasonably fast machine at commit [8f70a47bcd](https://github.com/tkoolen/RigidBodyDynamics.jl/tree/8f70a47bcd6ed4baca9d3ea4f304dc4f1df787d7):

Output of `versioninfo()`:

```
Julia Version 0.6.2
Commit d386e40c17 (2017-12-13 18:08 UTC)
Platform Info:
  OS: Linux (x86_64-pc-linux-gnu)
  CPU: Intel(R) Core(TM) i7-6950X CPU @ 3.00GHz
  WORD_SIZE: 64
  BLAS: libopenblas (USE64BITINT DYNAMIC_ARCH NO_AFFINITY Haswell)
  LAPACK: libopenblas64_
  LIBM: libopenlibm
  LLVM: libLLVM-3.9.1 (ORCJIT, broadwell)
```

Mass matrix:

```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     9.697 μs (0.00% GC)
  median time:      10.003 μs (0.00% GC)
  mean time:        10.076 μs (0.00% GC)
  maximum time:     47.473 μs (0.00% GC)
```

Mass matrix and Jacobian from left hand to right foot:

```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     10.426 μs (0.00% GC)
  median time:      10.737 μs (0.00% GC)
  mean time:        10.754 μs (0.00% GC)
  maximum time:     49.830 μs (0.00% GC)
```

Note the low additional cost of computing a Jacobian when the mass matrix is already computed. This is because RigidBodyDynamics.jl caches intermediate computation results.

Inverse dynamics:

```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     10.988 μs (0.00% GC)
  median time:      11.294 μs (0.00% GC)
  mean time:        11.383 μs (0.00% GC)
  maximum time:     33.164 μs (0.00% GC)
```

Forward dynamics:

```
  memory estimate:  64 bytes
  allocs estimate:  3
  --------------
  minimum time:     39.481 μs (0.00% GC)
  median time:      54.874 μs (0.00% GC)
  mean time:        55.230 μs (0.00% GC)
  maximum time:     594.235 μs (0.00% GC)
```
