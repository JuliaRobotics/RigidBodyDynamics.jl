using Base.Test

using RigidBodyDynamics
using RigidBodyDynamics.TreeDataStructure
using Rotations
using StaticArrays
using ForwardDiff

import Compat.Iterators: filter

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

# notebooks
@testset "example notebooks" begin
    using NBInclude
    notebookdir = joinpath("..", "notebooks")
    for file in readdir(notebookdir)
        name, ext = splitext(file)
        if lowercase(ext) == ".ipynb"
            @testset "$name" begin
                nbinclude(joinpath(notebookdir, file))
            end
        end
    end
end

# benchmarks
@testset "benchmarks" begin
    @test begin include("../perf/runbenchmarks.jl"); true end
end
