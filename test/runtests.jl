using Base.Test

using RigidBodyDynamics
using Quaternions
using StaticArrays
using FactCheck
using IJulia
using Compat
import ForwardDiff

import FactCheck: roughly
roughly{T}(x::T, atol) = y -> isapprox(y, x; atol = atol)
roughly{T}(x::T; kvtols...) = y -> isapprox(y, x; kvtols...)

include("test_frames.jl")
include("test_spatial.jl")
include("test_double_pendulum.jl")
include("test_mechanism.jl")

# run notebooks
jupyter = IJulia.jupyter
for f in filter(x -> endswith(x, "ipynb"), readdir("../examples"))
    notebook = "../examples/" * f
    run(`$jupyter nbconvert --to notebook --execute $notebook --output $notebook`)
end
