@testset "custom collections" begin
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
