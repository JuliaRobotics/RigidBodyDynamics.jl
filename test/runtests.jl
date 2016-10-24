using Base.Test

using RigidBodyDynamics
using RigidBodyDynamics.TreeDataStructure
using Quaternions
using StaticArrays
import ForwardDiff

include("test_tree.jl")
include("test_frames.jl")
include("test_spatial.jl")
include("test_double_pendulum.jl")
include("test_mechanism_algorithms.jl")
include("test_mechanism_manipulation.jl")

macro test_skip_nightly(ex)
    if VERSION < v"0.6-dev"
        return :(@test $(esc(ex)))
    else
        return :(@test_skip $(esc(ex)))
    end
end

# notebooks
@testset "example notebooks" begin
    using IJulia

    outputdir = RigidBodyDynamics.module_tempdir
    if !isdir(outputdir)
        mkpath(outputdir)
    end
    jupyter = IJulia.jupyter
    for f in filter(x -> endswith(x, "ipynb"), readdir("../examples"))
        notebook = joinpath("..", "examples", f)
        output = joinpath(outputdir, f)
        @test_skip_nightly begin run(`$jupyter nbconvert --to notebook --execute $notebook --output $output`); true end
    end
end

# benchmarks
@testset "benchmarks" begin
    # skip on nightly due to SSL failure with nightly while downloading Atlas URDF on Travis
    @test_skip_nightly evalfile("../perf/runbenchmarks.jl")
end
