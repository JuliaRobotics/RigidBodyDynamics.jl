
# Benchmarks
To get maximal performance, it is recommended to:
1. [Rebuild the Julia system image for your CPU architecture](https://docs.julialang.org/en/stable/devdocs/sysimg/#)
1. Pass `-O3` and `--check-bounds=no` as command line flags to `julia`.

Run `perf/runbenchmarks.jl` to see benchmark results for the Atlas robot (v5) in the following scenarios:

1. Compute the joint-space mass matrix.
1. Do inverse dynamics.
1. Do forward dynamics.

Note that results on Travis builds are **not at all** representative because of code coverage. Results on a recent, fast machine with version 0.2.0:

Output of `versioninfo()`:
```
Julia Version 0.6.0
Commit 9036443 (2017-06-19 13:05 UTC)
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
  minimum time:     13.790 μs (0.00% GC)
  median time:      14.263 μs (0.00% GC)
  mean time:        14.340 μs (0.00% GC)
  maximum time:     71.598 μs (0.00% GC)
```

Inverse dynamics:
```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     12.491 μs (0.00% GC)
  median time:      12.870 μs (0.00% GC)
  mean time:        60.109 μs (0.00% GC)
  maximum time:     471.387 ms (0.00% GC)
```

Forward dynamics:
```
  memory estimate:  64 bytes
  allocs estimate:  3
  --------------
  minimum time:     47.291 μs (0.00% GC)
  median time:      50.349 μs (0.00% GC)
  mean time:        153.675 μs (0.00% GC)
  maximum time:     1.033 s (0.00% GC)
```
