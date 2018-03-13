using Compat.Test
using RigidBodyDynamics

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

    @testset "IndexDict" begin
        Int32Dict{V} = RigidBodyDynamics.IndexDict{Int32, Base.OneTo{Int32}, V}
        dict = Dict(Int32(2) => 4., Int32(1) => 3.)
        expected = Int32Dict{Float64}(Base.OneTo(Int32(2)), [3., 4.])
        i32dict1 = @inferred Int32Dict(dict)
        @test i32dict1 == dict == expected
        @test keys(expected) === keys(i32dict1)
        @test values(expected) == values(i32dict1)
        i32dict2 = @inferred Int32Dict{Float64}(dict)
        @test i32dict2 == dict == expected
        @test keys(expected) === keys(i32dict2)
        @test values(expected) == values(i32dict2)
    end

    @testset "SegmentedVector" begin
        x = [1., 2., 3., 4.]
        viewlength = i -> 2
        xseg = SegmentedVector{Int}(x, 1 : 2, viewlength)
        @test segments(xseg)[1] == [1., 2.]
        @test segments(xseg)[2] == [3., 4.]
        @test length(segments(xseg)) == 2
        yseg = similar(xseg, Int32)
        yseg .= 1 : 4
        @test segments(yseg)[1] == [1, 2]
        @test segments(yseg)[2] == [3, 4]
        @test length(segments(yseg)) == 2

        xseg2 = SegmentedVector(x, RigidBodyDynamics.IndexDict(Base.OneTo(2), [view(x, 1 : 3), view(x, 4 : 4)]))
        @test xseg2 isa SegmentedVector
        ranges2 = RigidBodyDynamics.CustomCollections.ranges(xseg2)
        @test ranges2[1] == 1 : 3
        @test ranges2[2] == 4 : 4
        @test xseg2 == xseg

        xseg3 = copy(xseg)
        @test xseg3 == xseg
        @test xseg3 isa SegmentedVector
    end
end
