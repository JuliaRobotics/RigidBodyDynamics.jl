append!(empty!(LOAD_PATH), Base.DEFAULT_LOAD_PATH)
using Pkg
exampledir = @__DIR__
Pkg.activate(exampledir)
Pkg.instantiate()
include(joinpath(exampledir, "generate_notebooks.jl"))
