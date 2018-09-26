# Static and ahead-of-time compiled executable using RigidBodyDynamics.jl

Steps:

1. From `static` directory, run `julia --project=. --color=yes -e 'import Pkg; Pkg.instantiate(); include("build.jl")'`
2. Run `cd build && cc '-DJULIAC_PROGRAM_LIBNAME="main.so"' -o driver ../driver.c main.so -std=gnu99 -I/opt/julia-1.0/include/julia -DJULIA_ENABLE_THREADING=1 -fPIC -L/opt/julia-1.0/lib -Wl,--export-dynamic -Wl,-rpath,/opt/julia-1.0/lib -Wl,-rpath,/opt/julia-1.0/lib/julia -ljulia -m64 -O3 '-Wl,-rpath,$ORIGIN' && cd ..`. You may have to adjust the paths
3. Run `build/driver doublependulum.urdf test.csv 1`




The `julia_main` function is located in `main.jl`. It can be run using `julia snoopfile.jl`, which is also run during the ahead-of-time compilation process to determine which functions should be baked into the system image.

The compilation process takes 3:10.51 on my Linux machine.

Note that there is also a `build_shared_lib` function in `PackageCompiler`.
