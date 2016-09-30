f1 = CartesianFrame3D("1")
f2 = CartesianFrame3D("2")

t1 = rand(Transform3D{Float64}, f2, f1)
@fact t1 * inv(t1) --> roughly(Transform3D{Float64}(f1))
@fact inv(t1) * t1 --> roughly(Transform3D{Float64}(f2))

@fact t1 * Point3D(Float64, f2) --> roughly(Point3D(f1, t1.trans))

p = rand(Point3D, Float64, f2)
v = FreeVector3D(f2, p.v)
@fact inv(t1) * (t1 * p) --> roughly(p)
@fact inv(t1) * (t1 * v) --> roughly(v)
@fact t1 * p - t1 * v --> roughly(Point3D(f1, t1.trans))
