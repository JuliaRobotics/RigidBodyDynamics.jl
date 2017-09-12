@testset "contact" begin
    @testset "HalfSpace3D" begin
        frame = CartesianFrame3D()
        point = Point3D(frame, rand(), rand(), rand())
        normal = FreeVector3D(frame, 0., 0., 1.)
        halfspace = HalfSpace3D(point, normal)

        for i = 1 : 100
            x = rand(Point3D, frame)
            @test point_inside(halfspace, x) == (x.v[3] <= point.v[3])
            ϕ, normal = detect_contact(halfspace, x)
            @test ϕ == separation(halfspace, x)
            @test isapprox(normal.v, ForwardDiff.gradient(xyz -> separation(halfspace, Point3D(frame, xyz)), x.v))
        end
    end
end
