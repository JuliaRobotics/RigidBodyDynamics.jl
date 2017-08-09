
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
  minimum time:     14.112 μs (0.00% GC)
  median time:      14.836 μs (0.00% GC)
  mean time:        14.795 μs (0.00% GC)
  maximum time:     68.117 μs (0.00% GC)
```

Inverse dynamics:
```
  memory estimate:  0 bytes
  allocs estimate:  0
  --------------
  minimum time:     12.957 μs (0.00% GC)
  median time:      13.408 μs (0.00% GC)
  mean time:        13.707 μs (0.00% GC)
  maximum time:     33.793 μs (0.00% GC)
```

Forward dynamics:
```
  memory estimate:  64 bytes
  allocs estimate:  3
  --------------
  minimum time:     45.531 μs (0.00% GC)
  median time:      49.981 μs (0.00% GC)
  mean time:        58.648 μs (0.00% GC)
  maximum time:     1.323 ms (0.00% GC)
```
