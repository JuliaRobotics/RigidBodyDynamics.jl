
# Benchmarks
Run `perf/runbenchmarks.jl` (`-O3` and `--check-bounds=no` flags recommended) to see benchmark results for the Atlas robot (v5) in the following scenarios:

1. Compute the joint-space mass matrix.
1. Do inverse dynamics.
1. Do forward dynamics.

Note that results on Travis builds are **not at all** representative because of code coverage. Results on a recent, fast machine with version 0.0.4:

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
  memory estimate:  0.00 bytes
  allocs estimate:  0
  --------------
  minimum time:     23.034 μs (0.00% GC)
  median time:      23.364 μs (0.00% GC)
  mean time:        23.546 μs (0.00% GC)
  maximum time:     52.605 μs (0.00% GC)
  --------------
  samples:          10000
  evals/sample:     1
  time tolerance:   5.00%
  memory tolerance: 1.00%
```

Inverse dynamics:
```
  memory estimate:  0.00 bytes
  allocs estimate:  0
  --------------
  minimum time:     29.178 μs (0.00% GC)
  median time:      29.704 μs (0.00% GC)
  mean time:        30.276 μs (0.00% GC)
  maximum time:     65.232 μs (0.00% GC)
  --------------
  samples:          10000
  evals/sample:     1
  time tolerance:   5.00%
  memory tolerance: 1.00%
```

Forward dynamics:
```
  memory estimate:  48.00 bytes
  allocs estimate:  2
  --------------
  minimum time:     53.336 μs (0.00% GC)
  median time:      82.928 μs (0.00% GC)
  mean time:        83.334 μs (0.00% GC)
  maximum time:     208.453 μs (0.00% GC)
  --------------
  samples:          10000
  evals/sample:     1
  time tolerance:   5.00%
  memory tolerance: 1.00%
```
