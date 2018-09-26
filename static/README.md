# Static and ahead-of-time compiled executable using RigidBodyDynamics.jl

Steps:

1. From `static` directory, run `julia --project=. --color=yes -e 'import Pkg; Pkg.instantiate(); include("build.jl")'`. This creates the `build` directory containing (among other things) the `staticrbdjl.so` library with a C interface that corresponds to the functions in `staticrbd.jl` that are marked `Base.@ccallable`.
2. `cd build`
3. `cc '-DJULIAC_PROGRAM_LIBNAME="staticrbdjl.so"' -o driver ../driver.c staticrbdjl.so -std=gnu99 -I/opt/julia-1.0/include/julia -DJULIA_ENABLE_THREADING=1 -fPIC -L/opt/julia-1.0/lib -Wl,--export-dynamic -Wl,-rpath,/opt/julia-1.0/lib -Wl,-rpath,/opt/julia-1.0/lib/julia -ljulia -m64 -O3 '-Wl,-rpath,$ORIGIN'`. You may have to adjust the paths.
4. `./driver -u ../doublependulum.urdf -c ../test.csv`

The compilation process takes 3:10.51 on my Linux machine.


# TODO:

* create snoop file by running the driver on a couple of different inputs, then recompile with JIT disabled
* handle floating joints (`-f` flag)
* parse CSV
* actual timing code
* makefile
