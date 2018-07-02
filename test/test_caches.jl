module TestCaches

using Test
using RigidBodyDynamics

function randmech()
    rand_tree_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 5]; [Fixed{Float64} for i = 1 : 5]; [QuaternionSpherical{Float64} for i = 1 : 5]; [Prismatic{Float64} for i = 1 : 5]; [Planar{Float64} for i = 1 : 5]]...)
end

function cachetest(cache, eltypefun)
    x64 = @inferred cache[Float64]
    @test eltypefun(x64) == Float64
    @test cache[Float64] === x64
    @test @allocated(cache[Float64]) == 0

    x32 = @inferred cache[Float32]
    @test eltypefun(x32) == Float32
    @test cache[Float32] === x32
    @test @allocated(cache[Float32]) == 0

    @test cache[Float64] === x64
    @test @allocated(cache[Float64]) == 0
end

@testset "StateCache" begin
    mechanism = randmech()
    cache = StateCache(mechanism)
    cachetest(cache, RigidBodyDynamics.state_vector_eltype)
end

@testset "DynamicsResultCache" begin
    mechanism = randmech()
    cache = DynamicsResultCache(mechanism)
    cachetest(cache, result -> eltype(result.v̇))
end

@testset "SegmentedVectorCache" begin
    mechanism = randmech()
    state = MechanismState(mechanism)
    cache = SegmentedVectorCache(RigidBodyDynamics.ranges(velocity(state)))
    cachetest(cache, eltype)
end

end # module
