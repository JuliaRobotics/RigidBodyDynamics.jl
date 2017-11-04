@testset "custom collections" begin
    @testset "fastview" begin
        x1 = rand(5)
        x2 = [BigFloat(rand()) for i = 1 : 5]
        for x in (x1, x2)
            for range in (i : j for i in 1 : length(x), j in 1 : length(x) if j >= i)
                @test view(x, range) == RigidBodyDynamics.fastview(x, range)
            end
        end
    end

    @testset "UnsafeFastDict" begin
        d1 = RigidBodyDynamics.UnsafeFastDict{identity}(i => 3. * i for i in 1 : 3)
        show(DevNull, d1)
        @test eltype(d1) == Pair{Int64, Float64}
        @test all(keys(d1) .== 1 : 3)
        @test all(values(d1) .== 3. * (1 : 3))

        d2 = RigidBodyDynamics.UnsafeFastDict{x -> round(Int64, x), Number}(i => 3 * i for i in 1 : 3)
        @test all(keys(d2) .== 1 : 3)
        @test all(values(d2) .== 3 * (1 : 3))
    end

    @testset "nulldict" begin
        nd = RigidBodyDynamics.NullDict{Int, Int}()
        @test isempty(nd)
        @test length(nd) == 0
        for element in nd
            @test false # should never be reached, since the nulldict is always empty
        end
        show(IOBuffer(), nd)
    end
end
