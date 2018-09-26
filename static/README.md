# Static and ahead-of-time compiled executable using RigidBodyDynamics.jl

Steps:

1. From `static` directory, run `julia --project=. --color=yes -e 'import Pkg; Pkg.instantiate(); include("build.jl")'`
2. `cd build`
3. `cc '-DJULIAC_PROGRAM_LIBNAME="main.so"' -o driver ../driver.c main.so -std=gnu99 -I/opt/julia-1.0/include/julia -DJULIA_ENABLE_THREADING=1 -fPIC -L/opt/julia-1.0/lib -Wl,--export-dynamic -Wl,-rpath,/opt/julia-1.0/lib -Wl,-rpath,/opt/julia-1.0/lib/julia -ljulia -m64 -O3 '-Wl,-rpath,$ORIGIN'`. You may have to adjust the paths
4. `./driver ../doublependulum.urdf ../test.csv 1`

The compilation process takes 3:10.51 on my Linux machine.



# TODO:

* create snoop file by running the driver on a couple of different inputs, then recompile with JIT disabled
* actual timing code
