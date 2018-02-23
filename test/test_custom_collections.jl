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
