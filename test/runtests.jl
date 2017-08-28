using Base.Test

using RigidBodyDynamics
using RigidBodyDynamics.Graphs
using RigidBodyDynamics.Contact
using Rotations
using StaticArrays
using ForwardDiff

import Base.Iterators: filter

# useful utility function for computing time derivatives.
create_autodiff(x, dx) = [ForwardDiff.Dual(x[i], dx[i]) for i in 1 : length(x)]

# TODO: open a PR with ForwardDiff:
@inline Base.mod2pi{T<:ForwardDiff.Dual}(x::T) = ForwardDiff.Dual(mod2pi(ForwardDiff.value(x)), ForwardDiff.partials(x))
@inline Base.rem(x::ForwardDiff.Dual, n::Real) = ForwardDiff.Dual(rem(ForwardDiff.value(x), n), ForwardDiff.partials(x))

include("test_graph.jl")
include("test_util.jl")
include("test_frames.jl")
include("test_spatial.jl")
include("test_contact.jl")
include("test_urdf_parser.jl")
include("test_double_pendulum.jl")
include("test_mechanism_algorithms.jl")
include("test_simulate.jl")
include("test_mechanism_modification.jl")

# notebooks
@testset "example notebooks" begin
    using NBInclude
    notebookdir = joinpath("..", "notebooks")
    for file in readdir(notebookdir)
        name, ext = splitext(file)
        if lowercase(ext) == ".ipynb"
            @testset "$name" begin
                println("Testing $name.")
                nbinclude(joinpath(notebookdir, file), regex = r"^((?!\#NBSKIP).)*$"s)
            end
        end
    end
end

# benchmarks
@testset "benchmarks" begin
    @test begin include("../perf/runbenchmarks.jl"); true end
end
