# RigidBodyDynamics.jl examples

This directory contains various RigidBodyDynamics.jl usage examples.
The `.jl` files in each subdirectory are meant to be processed using [Literate.jl](https://github.com/fredrikekre/Literate.jl).

During the documentation build process, the `.jl` files are converted to markdown
files that end up in the package documentation. Jupyter notebooks are also
generated, and can be viewed online on [`nbviewer.jupyter.org`](https://nbviewer.jupyter.org/)
and ran using [`Binder`](https://mybinder.org/); links are available from the generated documentation page.

You can also run the notebooks locally by performing the following steps:

1. [install RigidBodyDynamics.jl](http://www.juliarobotics.org/RigidBodyDynamics.jl/stable/#Installation-1)
2. [install IJulia](https://github.com/JuliaLang/IJulia.jl) (`add` it to the default project)
3. in the Julia REPL, run
   ```
   using IJulia, RigidBodyDynamics
   cd(joinpath(dirname(pathof(RigidBodyDynamics)), "..", "examples"))
   using Pkg
   Pkg.activate(".")
   include("generate.jl")
   notebook(dir=".")
   ```
