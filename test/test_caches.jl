module TestCaches

using Test
using RigidBodyDynamics
using RigidBodyDynamics.Contact
import Random

function randmech()
    rand_tree_mechanism(Float64,
        QuaternionFloating{Float64},
        [Revolute{Float64} for i = 1 : 5]...,
        [Fixed{Float64} for i = 1 : 5]...,
        [Prismatic{Float64} for i = 1 : 5]...,
        [Planar{Float64} for i = 1 : 5]...,
        [SPQuatFloating{Float64} for i = 1:2]...,
        [SinCosRevolute{Float64} for i = 1:2]...
    )
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

@testset "caches (nthreads = $(Threads.nthreads()))" begin
    @testset "StateCache" begin
        Random.seed!(1)
        mechanism = randmech()
        cache = StateCache(mechanism)
        cachetest(cache, RigidBodyDynamics.state_vector_eltype)
    end

    @testset "DynamicsResultCache" begin
        @testset "Basic mechanism" begin
            Random.seed!(2)
            mechanism = randmech()
            cache = DynamicsResultCache(mechanism)
            cachetest(cache, result -> eltype(result.v̇))
        end

        @testset "Mechanism with contact points (Issue #483)" begin
            Random.seed!(3)
            mechanism = randmech()
            contactmodel = SoftContactModel(hunt_crossley_hertz(k = 500e3),
                ViscoelasticCoulombModel(0.8, 20e3, 100.))
            body = rand(bodies(mechanism))
            add_contact_point!(body,
                ContactPoint(Point3D(default_frame(body), 0.0, 0.0, 0.0), contactmodel))
            cache = DynamicsResultCache(mechanism)
            cachetest(cache, result -> eltype(result.v̇))
        end
    end

    @testset "SegmentedVectorCache" begin
        Random.seed!(3)
        mechanism = randmech()
        state = MechanismState(mechanism)
        cache = SegmentedVectorCache(RigidBodyDynamics.ranges(velocity(state)))
        cachetest(cache, eltype)
    end

    @testset "StateCache multi-threaded (#548)" begin
        N = 2
        mechanism = randmech()

        state_caches = [StateCache(mechanism) for _ = 1 : N]
        qs = let state = MechanismState(mechanism)
            [(rand_configuration!(state); copy(configuration(state))) for _ = 1 : N]
        end
        vs = [rand(num_velocities(mechanism)) for _ = 1 : N]

        Threads.@threads for i = 1 : N
            state = state_caches[i][Float64]
            set_configuration!(state, qs[i])
            set_velocity!(state, vs[i])
        end

        for i = 1 : N
            @test configuration(state_caches[i][Float64]) == qs[i]
            @test velocity(state_caches[i][Float64]) == vs[i]
        end

        Threads.@threads for i = 1 : N
            @test configuration(state_caches[i][Float64]) == qs[i]
            @test velocity(state_caches[i][Float64]) == vs[i]
        end
    end
end

end # module
