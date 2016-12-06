using Base.Test

using RigidBodyDynamics
using RigidBodyDynamics.TreeDataStructure
using Rotations
using StaticArrays
using ODE
using ForwardDiff

if VERSION > v"0.5"
    import Base.Iterators: filter
end

# useful utility function for computing time derivatives.
create_autodiff(x, dx) = [ForwardDiff.Dual(x[i], dx[i]) for i in 1 : length(x)]

# TODO: open a PR with ForwardDiff:
Base.mod{T<:ForwardDiff.Dual}(x::T, y::T) = ForwardDiff.Dual(mod(ForwardDiff.value(x), ForwardDiff.value(y)), ForwardDiff.partials(x))
@inline Base.rem(x::ForwardDiff.Dual, n::Real) = ForwardDiff.Dual(rem(ForwardDiff.value(x), n), ForwardDiff.partials(x))

include("test_util.jl")
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
        # skip on nightly because notebooks specify version 0.5
        @test_skip_nightly begin run(`$jupyter nbconvert --to notebook --execute $notebook --output $output`); true end
    end
end

# benchmarks
@testset "benchmarks" begin
    # skip on nightly due to SSL failure with nightly while downloading Atlas URDF on Travis
    @test_skip_nightly begin include("../perf/runbenchmarks.jl"); true end
end
