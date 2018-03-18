module TestCaches

using Compat.Test
using RigidBodyDynamics

function randmech()
    rand_tree_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 5]; [Fixed{Float64} for i = 1 : 5]; [QuaternionSpherical{Float64} for i = 1 : 5]; [Prismatic{Float64} for i = 1 : 5]; [Planar{Float64} for i = 1 : 5]]...)
end

@testset "StateCache" begin
    mechanism = randmech()
    cache = StateCache(mechanism)

    state64 = @inferred cache[Float64]
    @test RigidBodyDynamics.state_vector_eltype(state64) == Float64
    @test cache[Float64] === state64
    @test @allocated(cache[Float64]) == 0

    state32 = @inferred cache[Float32]
    @test RigidBodyDynamics.state_vector_eltype(state32) == Float32
    @test cache[Float32] === state32
    @test @allocated(cache[Float32]) == 0

    @test cache[Float64] === state64
    @test @allocated(cache[Float64]) == 0
end

@testset "DynamicsResultCache" begin
    mechanism = randmech()
    cache = DynamicsResultCache(mechanism)

    result64 = @inferred cache[Float64]
    @test eltype(result64.v̇) == Float64
    @test cache[Float64] === result64
    @test @allocated(cache[Float64]) == 0

    result32 = @inferred cache[Float32]
    @test eltype(result32.v̇) == Float32
    @test cache[Float32] === result32
    @test @allocated(cache[Float32]) == 0

    @test cache[Float64] === result64
    @test @allocated(cache[Float64]) == 0
end

end # module
