using Base.Test

using RigidBodyDynamics
using RigidBodyDynamics.TreeDataStructure
using Quaternions
using StaticArrays
using Compat
import ForwardDiff

include("test_tree.jl")
include("test_frames.jl")
include("test_spatial.jl")
include("test_double_pendulum.jl")
include("test_mechanism_algorithms.jl")
include("test_mechanism_manipulation.jl")

if VERSION < v"0.6-dev"
    outputdir = RigidBodyDynamics.module_tempdir
    if !isdir(outputdir)
        mkpath(outputdir)
    end
    @testset "example notebooks" begin
        using IJulia
        jupyter = IJulia.jupyter
        for f in filter(x -> endswith(x, "ipynb"), readdir("../examples"))
            notebook = joinpath("..", "examples", f)
            output = joinpath(outputdir, f)
            run(`$jupyter nbconvert --to notebook --execute $notebook --output $output`)
        end
    end
end

@testset "benchmarks" begin
    include("../perf/runbenchmarks.jl")
end
