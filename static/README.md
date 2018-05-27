# Static and ahead-of-time compiled executable using RigidBodyDynamics.jl

Steps:

1. `Pkg.add("PackageCompiler")`
2. `Pkg.checkout("PackageCompiler")`
3. From `static` directory, run `julia --color=yes build.jl`
4. From `static` directory, run `build/main`

The `julia_main` function is located in `main.jl`. It can be run using `julia snoopfile.jl`, which is also run during the ahead-of-time compilation process to determine which functions should be baked into the system image.

The compilation process takes 3m40.950s on my OSX machine.

Note that there is also a `build_shared_lib` function in `PackageCompiler`.
