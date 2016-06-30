function Ad(H::Transform3D)
    R = rotationmatrix(H.rot)
    pHat = Array(RigidBodyDynamics.vector_to_skew_symmetric(H.trans))
    return [R zeros(3, 3); pHat * R R]
end

f1 = CartesianFrame3D("1")
f2 = CartesianFrame3D("2")
f3 = CartesianFrame3D("3")
f4 = CartesianFrame3D("4")

facts("spatial inertia") do
    I2 = rand(SpatialInertia{Float64}, f2)
    H21 = rand(Transform3D{Float64}, f2, f1)
    I1 = transform(I2, H21)
    I3 = rand(SpatialInertia{Float64}, f2)
    @fact I2.mass --> I1.mass
    @fact Array(I1) --> roughly(Ad(inv(H21))' * Array(I2) * Ad(inv(H21)); atol = 1e-12)
    @fact I2 --> roughly(transform(I1, inv(H21)))
    @fact Array(I2) + Array(I3) --> roughly(Array(I2 + I3); atol = 1e-12)
end

facts("twist") do
    T1 = rand(Twist{Float64}, f2, f1, f3)
    T2 = rand(Twist{Float64}, f3, f2, f3)
    T3 = T1 + T2
    H31 = rand(Transform3D{Float64}, f3, f1)
    @fact T3.body --> T2.body
    @fact T3.base --> T1.base
    @fact T3.frame --> f3
    @fact T2 + T1 --> roughly(T3)
    @fact_throws ArgumentError T1 + rand(Twist{Float64}, f3, f2, f4) # wrong frame
    @fact_throws ArgumentError T1 + rand(Twist{Float64}, f3, f4, f3) # wrong base
    @fact Array(transform(T1, H31)) --> roughly(Ad(H31) * Array(T1))
end

facts("wrench") do
    W = rand(Wrench{Float64}, f2)
    H21 = rand(Transform3D{Float64}, f2, f1)
    @fact Array(transform(W, H21)) --> roughly(Ad(inv(H21))' * Array(W))
    @fact_throws ArgumentError transform(W, inv(H21)) # wrong frame
end

facts("momentum") do
    T = rand(Twist{Float64}, f2, f1, f2)
    I = rand(SpatialInertia{Float64}, f2)
    T2 = rand(Twist{Float64}, f2, f1, f1)
    H21 = rand(Transform3D{Float64}, f2, f1)
    h = I * T
    @fact Array(I) * Array(T) --> roughly(Array(h); atol = 1e-12)
    @fact_throws ArgumentError I * T2 # wrong frame
    @fact transform(I, H21) * transform(T, H21) --> roughly(transform(h, H21))
    @fact Array(transform(h, H21)) --> roughly(Ad(inv(H21))' * Array(h))
    @fact_throws ArgumentError transform(h, inv(H21)) # wrong frame
end

facts("geometric jacobian, power") do
    J = GeometricJacobian(f2, f1, f3, rand(6, 14))
    v = rand(num_cols(J))
    W = rand(Wrench{Float64}, f3)
    T = Twist(J, v)
    H = rand(Transform3D{Float64}, f3, f1)
    τ = joint_torque(J, W)
    @fact J.body --> T.body
    @fact J.base --> T.base
    @fact J.frame --> T.frame
    @fact Twist(transform(J, H), v) --> roughly(transform(T, H))
    @fact dot(Array(τ), v) --> roughly(dot(T, W); atol = 1e-12) # power equality
    @fact_throws ArgumentError dot(transform(T, H), W)
    @fact_throws ArgumentError joint_torque(transform(J, H), W)
end

facts("momentum matrix") do
    A = MomentumMatrix(f3, rand(6, 13))
    v = rand(num_cols(A))
    h = Momentum(A, v)
    H = rand(Transform3D{Float64}, f3, f1)
    @fact h.frame --> A.frame
    @fact Momentum(transform(A, H), v) --> roughly(transform(h, H))
end

facts("spatial acceleration") do
    I = rand(SpatialInertia{Float64}, f2)
    Ṫ = rand(SpatialAcceleration{Float64}, f2, f1, f2)
    T = rand(Twist{Float64}, f2, f1, f2)
    W = newton_euler(I, Ṫ, T)
    H = rand(Transform3D{Float64}, f2, f1)
    @fact transform(newton_euler(transform(I, H), transform(Ṫ, H, T, T), transform(T, H)), inv(H)) --> roughly(W)
end

facts("other functionality") do
    I = rand(SpatialInertia{Float64}, f2)
    T = rand(Twist{Float64}, f2, f1, f2)
    H = rand(Transform3D{Float64}, f2, f1)
    Ek = kinetic_energy(I, T)
    @fact (1//2 * Array(T)' * Array(I) * Array(T))[1] --> roughly(Ek; atol = 1e-12)
    @fact kinetic_energy(transform(I, H), transform(T, H)) --> roughly(Ek; atol = 1e-12)
end
