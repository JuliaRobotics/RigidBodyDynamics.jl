using Base.Test

using RigidBodyDynamics
using RigidBodyDynamics.Graphs
using RigidBodyDynamics.Contact
using RigidBodyDynamics.PDControl
using Rotations
using StaticArrays
using ForwardDiff

import Base.Iterators: filter

# useful utility function for computing time derivatives.
create_autodiff(x, dx) = [ForwardDiff.Dual(x[i], dx[i]) for i in 1 : length(x)]

# TODO: https://github.com/JuliaDiff/DiffBase.jl/pull/19
@inline Base.mod2pi(x::ForwardDiff.Dual) = ForwardDiff.Dual(mod2pi(ForwardDiff.value(x)), ForwardDiff.partials(x))
@inline Base.rem2pi(x::ForwardDiff.Dual, roundingmode::RoundingMode) = ForwardDiff.Dual(rem2pi(ForwardDiff.value(x), roundingmode), ForwardDiff.partials(x))

include("test_graph.jl")
include("test_custom_collections.jl")
include("test_frames.jl")
include("test_spatial.jl")
include("test_contact.jl")
include("test_urdf_parser.jl")
include("test_double_pendulum.jl")
include("test_mechanism_algorithms.jl")
include("test_simulate.jl")
include("test_mechanism_modification.jl")
include("test_pd_control.jl")

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
