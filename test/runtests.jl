using Compat
using Test
using Compat.LinearAlgebra
using Compat.Random

using RigidBodyDynamics
using RigidBodyDynamics.Graphs
using RigidBodyDynamics.Contact
using RigidBodyDynamics.PDControl
using Rotations
using StaticArrays

import Base.Iterators: filter
import ForwardDiff
import LightXML

include("test_exports.jl")
include("test_graph.jl")
include("test_custom_collections.jl")
include("test_frames.jl")
include("test_spatial.jl")
include("test_contact.jl")
include("test_urdf_parser.jl")
include("test_double_pendulum.jl")
include("test_caches.jl")
include("test_mechanism_algorithms.jl")
include("test_simulate.jl")
include("test_mechanism_modification.jl")
include("test_pd_control.jl")

# notebooks
notebookdir = joinpath(@__DIR__, "..", "notebooks")
for file in readdir(notebookdir)
    name, ext = splitext(file)
    lowercase(ext) == ".ipynb" || continue
    @eval module $(gensym())
    using Test
    using NBInclude
    @testset "$($name)" begin
        nbinclude(joinpath($notebookdir, $file), regex = r"^((?!\#NBSKIP).)*$"s)
    end
    end #module
end

# benchmarks
@testset "benchmarks" begin
    @test begin include("../perf/runbenchmarks.jl"); true end
end
