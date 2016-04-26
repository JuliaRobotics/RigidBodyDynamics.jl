function Ad(H::Transform3D)
    R = rotationmatrix(H.rot)
    pHat = Array(RigidBodyDynamics.vector_to_skew_symmetric(H.trans))
    return [R zeros(3, 3); pHat * R R]
end

function test_spatial()
    f1 = CartesianFrame3D("1")
    f2 = CartesianFrame3D("2")
    f3 = CartesianFrame3D("3")
    f4 = CartesianFrame3D("4")

    let # spatial inertia
        I2 = rand(SpatialInertia{Float64}, f2)
        H21 = rand(Transform3D{Float64}, f2, f1)
        I1 = transform(I2, H21)
        I3 = rand(SpatialInertia{Float64}, f2)
        @test I2.mass == I1.mass
        @test isapprox(Ad(inv(H21))' * to_array(I2) * Ad(inv(H21)), to_array(I1); atol = 1e-12)
        @test isapprox(transform(I1, inv(H21)), I2)
        @test isapprox(to_array(I2 + I3), to_array(I2) + to_array(I3))
    end

    let # twist
        T1 = rand(Twist{Float64}, f2, f1, f3)
        T2 = rand(Twist{Float64}, f3, f2, f3)
        T3 = T1 + T2
        H31 = rand(Transform3D{Float64}, f3, f1)
        @test T3.body == T2.body
        @test T3.base == T1.base
        @test T3.frame == f3
        @test isapprox(T3, T2 + T1)
        @test_throws AssertionError T1 + rand(Twist{Float64}, f3, f2, f4) # wrong frame
        @test_throws ArgumentError T1 + rand(Twist{Float64}, f3, f4, f3) # wrong base
        @test isapprox(Ad(H31) * to_array(T1), to_array(transform(T1, H31)))
    end

    let # wrench
        W = rand(Wrench{Float64}, f2)
        H21 = rand(Transform3D{Float64}, f2, f1)
        @test isapprox(Ad(inv(H21))' * to_array(W), to_array(transform(W, H21)))
        @test_throws AssertionError transform(W, inv(H21)) # wrong frame
    end

    let # momentum
        T = rand(Twist{Float64}, f2, f1, f2)
        I = rand(SpatialInertia{Float64}, f2)
        T2 = rand(Twist{Float64}, f2, f1, f1)
        H21 = rand(Transform3D{Float64}, f2, f1)
        h = I * T
        @test isapprox(to_array(h), to_array(I) * to_array(T); atol = 1e-12)
        @test_throws AssertionError I * T2 # wrong frame
        @test isapprox(transform(h, H21), transform(I, H21) * transform(T, H21))
        @test isapprox(Ad(inv(H21))' * to_array(h), to_array(transform(h, H21)))
        @test_throws AssertionError transform(h, inv(H21)) # wrong frame
    end

    let # geometric jacobian, power
        J = GeometricJacobian(f2, f1, f3, rand(6, 14))
        v = rand(size(J.mat, 2))
        W = rand(Wrench{Float64}, f3)
        T = J * v
        H = rand(Transform3D{Float64}, f3, f1)
        τ = joint_torque(J, W)
        @test J.body == T.body
        @test J.base == T.base
        @test J.frame == T.frame
        @test isapprox(transform(T, H), transform(J, H) * v)
        @test isapprox(dot(T, W), dot(τ, v), atol = 1e-12) # power equality
        @test_throws AssertionError dot(transform(T, H), W)
        @test_throws AssertionError joint_torque(transform(J, H), W)
    end

    let # momentum matrix
        A = MomentumMatrix(f3, rand(6, 13))
        v = rand(size(A.mat, 2))
        h = A * v
        H = rand(Transform3D{Float64}, f3, f1)
        @test h.frame == A.frame
        @test isapprox(transform(h, H), transform(A, H) * v)
    end

    let # spatial acceleration
        I = rand(SpatialInertia{Float64}, f2)
        Ṫ = rand(SpatialAcceleration{Float64}, f2, f1, f2)
        T = rand(Twist{Float64}, f2, f1, f2)
        W = newton_euler(I, Ṫ, T)
        H = rand(Transform3D{Float64}, f2, f1)
        @test isapprox(W, transform(newton_euler(transform(I, H), transform(Ṫ, H, T, T), transform(T, H)), inv(H)))
    end

    let # other functionality
        I = rand(SpatialInertia{Float64}, f2)
        T = rand(Twist{Float64}, f2, f1, f2)
        H = rand(Transform3D{Float64}, f2, f1)
        Ek = kinetic_energy(I, T)
        @test isapprox(Ek, (1//2 * to_array(T)' * to_array(I) * to_array(T))[1]; atol = 1e-12)
        @test isapprox(Ek, kinetic_energy(transform(I, H), transform(T, H)); atol = 1e-12)
    end
end
