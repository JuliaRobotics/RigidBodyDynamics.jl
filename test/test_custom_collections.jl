using Test
using RigidBodyDynamics
import Random

# A pathologically weird matrix which uses base -1 indexing
# for its first dimension and base 2 indexing for its second
struct NonOneBasedMatrix <: AbstractMatrix{Float64}
    m::Int
    n::Int
end

Base.size(m::NonOneBasedMatrix) = (m.m, m.n)
Base.axes(m::NonOneBasedMatrix) = ((1:m.m) .- 2, (1:m.n) .+ 1)

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

    @testset "ConstDict" begin
        c = RigidBodyDynamics.ConstDict{Int}(2.0)
        @test c[1] == 2.0
        @test c[-1] == 2.0
        show(IOBuffer(), c)
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

    @testset "SegmentedBlockDiagonalMatrix" begin
        Random.seed!(5)
        A = rand(10, 10)
        block_indices = [(1:1, 1:1),  # square
                         (2:4, 2:2),  # non-square
                         (5:4, 3:2),  # empty
                         (5:7, 3:6),  # 2x2
                         (8:10, 7:10)]
        RigidBodyDynamics.CustomCollections.check_contiguous_block_ranges(A, block_indices)
        S = RigidBodyDynamics.SegmentedBlockDiagonalMatrix(A, block_indices)
        for (i, block) in enumerate(block_indices)
            @test S[block...] == RigidBodyDynamics.CustomCollections.blocks(S)[i]
        end
        A .= 0
        for block in RigidBodyDynamics.CustomCollections.blocks(S)
            Random.rand!(block)
        end

        @testset "Malformed blocks" begin
            @testset "overlap" begin
            block_indices = [(1:1, 1:1),
                             (2:4, 2:3),
                             (5:4, 3:2),
                             (5:7, 3:6),
                             (8:10, 7:10)]
                @test_throws ArgumentError RigidBodyDynamics.CustomCollections.check_contiguous_block_ranges(A, block_indices)
            end

            @testset "out of bounds" begin
                block_indices = [(1:1, 0:1),
                                 (2:4, 2:2),
                                 (5:4, 3:2),
                                 (5:7, 3:6),
                                 (8:10, 7:10)]
                @test_throws ArgumentError RigidBodyDynamics.CustomCollections.check_contiguous_block_ranges(A, block_indices)

                block_indices = [(1:1, 1:1),
                                 (2:4, 2:2),
                                 (5:4, 3:2),
                                 (5:7, 3:6),
                                 (8:12, 7:10)]
                @test_throws ArgumentError RigidBodyDynamics.CustomCollections.check_contiguous_block_ranges(A, block_indices)
            end

            @testset "gap" begin
                block_indices = [(1:1, 1:1),
                                 (5:4, 3:2),
                                 (5:7, 3:6),
                                 (8:10, 7:10)]
                @test_throws ArgumentError RigidBodyDynamics.CustomCollections.check_contiguous_block_ranges(A, block_indices)
            end
        end

        @testset "Nonstandard indexing" begin
            M = NonOneBasedMatrix(5, 5)
            block_indices = [(-1:1, 2:3),
                             (2:3,  4:6)]
            RigidBodyDynamics.CustomCollections.check_contiguous_block_ranges(M, block_indices)
        end

        @testset "mul! specialization" begin
            M = rand(20, size(S, 1))
            C = M * S
            @test C ≈ M * parent(S) atol=1e-15

            M = rand(size(S, 2), 20)
            C = S * M
            @test C ≈ parent(S) * M atol=1e-15

            @test_throws DimensionMismatch rand(20, size(S, 1) + 1) * M
            @test_throws DimensionMismatch rand(size(A, 2) + 1, 20) * M
        end
    end
end
