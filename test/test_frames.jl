@testset "frames" begin
    f1name = "1"
    f1 = CartesianFrame3D(f1name)
    f2 = CartesianFrame3D()
    f3 = CartesianFrame3D()
    @test name(f1) == f1name
    name(f2) # just to make sure it doesn't crash
    @test f2 != f3
    @boundscheck begin
        # only throws when bounds checks are enabled:
        @test_throws ArgumentError @framecheck(f1, f2)
        @test_throws ArgumentError @framecheck(f2, f3)
    end
    @framecheck(f1, f1)

    t1 = rand(Transform3D{Float64}, f2, f1)
    @test isapprox(t1 * inv(t1), Transform3D{Float64}(f1))
    @test isapprox(inv(t1) * t1, Transform3D{Float64}(f2))

    @test isapprox(t1 * Point3D(Float64, f2), Point3D(f1, t1.trans))

    p = rand(Point3D, Float64, f2)
    v = FreeVector3D(f2, p.v)
    @test isapprox(inv(t1) * (t1 * p), p)
    @test isapprox(inv(t1) * (t1 * v), v)
    @test isapprox(t1 * p - t1 * v, Point3D(f1, t1.trans))

    @test_throws DimensionMismatch Point3D(f2, rand(2))
    @test_throws DimensionMismatch Point3D(f2, rand(4))

    show(DevNull, t1)
    show(DevNull, p)
    show(DevNull, v)
end
