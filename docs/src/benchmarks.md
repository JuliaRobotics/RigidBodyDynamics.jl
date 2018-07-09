# Benchmarks

To get maximal performance, it is recommended to:

1. [Rebuild the Julia system image for your CPU architecture](https://docs.julialang.org/en/stable/devdocs/sysimg/#)
1. Pass `-O3`, `--check-bounds=no`, and `--math-mode=fast` as command line flags to `julia`.

Run `perf/runbenchmarks.jl` to see benchmark results for the Atlas robot (v5) in the following scenarios:

1. Compute the joint-space mass matrix.
1. Compute both the mass matrix and a geometric Jacobian from the left hand to the right foot.
1. Do inverse dynamics.
1. Do forward dynamics.

Note that results on Travis builds are **not at all** representative because of code coverage. Results on a reasonably fast machine at commit [8f70a47bcd](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/tree/8f70a47bcd6ed4baca9d3ea4f304dc4f1df787d7):

Output of `versioninfo()`:

```
Julia Version 0.7.0-beta.133
Commit 60174a9 (2018-07-03 20:03 UTC)
Platform Info:
  OS: Linux (x86_64-linux-gnu)
  CPU: Intel(R) Core(TM) i7-6950X CPU @ 3.00GHz
  WORD_SIZE: 64
  LIBM: libopenlibm
  LLVM: libLLVM-6.0.0 (ORCJIT, broadwell)
```

Mass matrix:

```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     6.618 μs (0.00% GC)
  median time:      6.883 μs (0.00% GC)
  mean time:        7.158 μs (0.00% GC)
  maximum time:     40.992 μs (0.00% GC)
```

Mass matrix and Jacobian from left hand to right foot:

```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     7.442 μs (0.00% GC)
  median time:      7.839 μs (0.00% GC)
  mean time:        7.840 μs (0.00% GC)
  maximum time:     43.941 μs (0.00% GC)
```

Note the low additional cost of computing a Jacobian when the mass matrix is already computed. This is because RigidBodyDynamics.jl caches intermediate computation results.

Inverse dynamics:

```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     7.954 μs (0.00% GC)
  median time:      8.246 μs (0.00% GC)
  mean time:        8.456 μs (0.00% GC)
  maximum time:     34.537 μs (0.00% GC)
```

Forward dynamics:

```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     29.740 μs (0.00% GC)
  median time:      36.015 μs (0.00% GC)
  mean time:        36.014 μs (0.00% GC)
  maximum time:     186.809 μs (0.00% GC)
```
