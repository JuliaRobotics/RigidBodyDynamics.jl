using Test
using LinearAlgebra
using Random

using RigidBodyDynamics
using RigidBodyDynamics.Graphs
using RigidBodyDynamics.Contact
using RigidBodyDynamics.PDControl
using Rotations
using StaticArrays

import Base.Iterators: filter
import ForwardDiff
import LightXML

using RigidBodyDynamics: ModifiedRodriguesParam
using BenchmarkTools: @ballocated

include("test_exports.jl")
include("test_graph.jl")
include("test_custom_collections.jl")
include("test_frames.jl")
include("test_spatial.jl")
include("test_contact.jl")
include("test_urdf.jl")
include("test_double_pendulum.jl")
include("test_caches.jl")
include("test_mechanism_algorithms.jl")
include("test_simulate.jl")
include("test_mechanism_modification.jl")
include("test_pd_control.jl")

if v"1.11" <= VERSION < v"1.12"  # Test notebooks on Julia v1.11
    # The notebook tests rely on instantiating specific project manifests.
    # Attempting to do so on a version of Julia older than the one used to
    # create those manifests can cause errors in `Pkg.instantiate()`.
    include("test_notebooks.jl")

    @testset "benchmarks" begin
        @test begin include("../perf/runbenchmarks.jl"); true end
    end
end
