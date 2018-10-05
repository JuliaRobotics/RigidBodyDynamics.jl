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
include("test_notebooks.jl")

@testset "benchmarks" begin
    @test begin include("../perf/runbenchmarks.jl"); true end
end
