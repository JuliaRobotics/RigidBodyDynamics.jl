@testset "frames" begin
    f1 = CartesianFrame3D("1")
    f2 = CartesianFrame3D("2")

    t1 = rand(Transform3D{Float64}, f2, f1)
    @test isapprox(t1 * inv(t1), Transform3D{Float64}(f1))
    @test isapprox(inv(t1) * t1, Transform3D{Float64}(f2))

    @test isapprox(t1 * Point3D(Float64, f2), Point3D(f1, t1.trans))

    p = rand(Point3D, Float64, f2)
    v = FreeVector3D(f2, p.v)
    @test isapprox(inv(t1) * (t1 * p), p)
    @test isapprox(inv(t1) * (t1 * v), v)
    @test isapprox(t1 * p - t1 * v, Point3D(f1, t1.trans))
end
