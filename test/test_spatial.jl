function Ad(H::Transform3D)
    R = rotationmatrix(H.rot)
    pHat = Array(RigidBodyDynamics.hat(H.trans))
    return [R zeros(3, 3); pHat * R R]
end

@testset "spatial" begin
    f1 = CartesianFrame3D("1")
    f2 = CartesianFrame3D("2")
    f3 = CartesianFrame3D("3")
    f4 = CartesianFrame3D("4")

    @testset "spatial inertia" begin
        I2 = rand(SpatialInertia{Float64}, f2)
        H21 = rand(Transform3D{Float64}, f2, f1)
        I1 = transform(I2, H21)
        I3 = rand(SpatialInertia{Float64}, f2)
        @test I2.mass == I1.mass
        @test isapprox(Array(I1), Ad(inv(H21))' * Array(I2) * Ad(inv(H21)); atol = 1e-12)
        @test isapprox(I2, transform(I1, inv(H21)))
        @test isapprox(Array(I2) + Array(I3), Array(I2 + I3); atol = 1e-12)
        @inferred transform(zero(SpatialInertia{Float32}, f1), Transform3D{Float64}(f1))
    end

    @testset "twist" begin
        T1 = rand(Twist{Float64}, f2, f1, f3)
        T2 = rand(Twist{Float64}, f3, f2, f3)
        T3 = T1 + T2
        H31 = rand(Transform3D{Float64}, f3, f1)
        @test T3.body == T2.body
        @test T3.base == T1.base
        @test T3.frame == f3
        @test isapprox(T2 + T1, T3)
        @test_throws ArgumentError T1 + rand(Twist{Float64}, f3, f2, f4) # wrong frame
        @test_throws ArgumentError T1 + rand(Twist{Float64}, f3, f4, f3) # wrong base
        @test isapprox(Array(transform(T1, H31)), Ad(H31) * Array(T1))
    end

    @testset "wrench" begin
        W = rand(Wrench{Float64}, f2)
        H21 = rand(Transform3D{Float64}, f2, f1)
        @test isapprox(Array(transform(W, H21)), Ad(inv(H21))' * Array(W))
        @test_throws ArgumentError transform(W, inv(H21)) # wrong frame
    end

    @testset "momentum" begin
        T = rand(Twist{Float64}, f2, f1, f2)
        I = rand(SpatialInertia{Float64}, f2)
        T2 = rand(Twist{Float64}, f2, f1, f1)
        H21 = rand(Transform3D{Float64}, f2, f1)
        h = I * T
        @test isapprox(Array(I) * Array(T), Array(h); atol = 1e-12)
        @test_throws ArgumentError I * T2 # wrong frame
        @test isapprox(transform(I, H21) * transform(T, H21), transform(h, H21))
        @test isapprox(Array(transform(h, H21)), Ad(inv(H21))' * Array(h))
        @test_throws ArgumentError transform(h, inv(H21)) # wrong frame
    end

    @testset "geometric jacobian, power" begin
        n = 14
        J = GeometricJacobian(f2, f1, f3, rand(SMatrix{3, n}), rand(SMatrix{3, n}))
        v = rand(num_cols(J))
        W = rand(Wrench{Float64}, f3)
        T = Twist(J, v)
        H = rand(Transform3D{Float64}, f3, f1)
        τ = torque(J, W)
        @test J.body == T.body
        @test J.base == T.base
        @test J.frame == T.frame
        @test isapprox(Twist(transform(J, H), v), transform(T, H))
        @test isapprox(dot(Array(τ), v), dot(T, W); atol = 1e-12) # power equality
        @test_throws ArgumentError dot(transform(T, H), W)
        @test_throws ArgumentError torque(transform(J, H), W)
    end

    @testset "momentum matrix" begin
        n = 13
        A = MomentumMatrix(f3, rand(SMatrix{3, n}), rand(SMatrix{3, n}))
        v = rand(num_cols(A))
        h = Momentum(A, v)
        H = rand(Transform3D{Float64}, f3, f1)
        @test h.frame == A.frame
        @test isapprox(Momentum(transform(A, H), v), transform(h, H))
    end

    @testset "spatial acceleration" begin
        I = rand(SpatialInertia{Float64}, f2)
        Ṫ = rand(SpatialAcceleration{Float64}, f2, f1, f2)
        T = rand(Twist{Float64}, f2, f1, f2)
        W = newton_euler(I, Ṫ, T)
        H = rand(Transform3D{Float64}, f2, f1)
        @test isapprox(transform(newton_euler(transform(I, H), transform(Ṫ, H, T, T), transform(T, H)), inv(H)), W)
    end

    @testset "kinetic energy" begin
        I = rand(SpatialInertia{Float64}, f2)
        T = rand(Twist{Float64}, f2, f1, f2)
        H = rand(Transform3D{Float64}, f2, f1)
        Ek = kinetic_energy(I, T)
        @test isapprox((1//2 * Array(T)' * Array(I) * Array(T))[1], Ek; atol = 1e-12)
        @test isapprox(kinetic_energy(transform(I, H), transform(T, H)), Ek; atol = 1e-12)
    end

    @testset "log / exp" begin
        for θ in [linspace(0., 10 * eps(), 100); linspace(0., 2 * π - eps(), 100)]
            # have magnitude of parts of twist be bounded by θ to check for numerical issues
            ϕrot = normalize(rand(SVector{3})) * θ * 2 * (rand() - 0.5)
            ϕtrans = normalize(rand(SVector{3})) * θ * 2 * (rand() - 0.5)
            ξ = Twist{Float64}(f2, f1, f1, ϕrot, ϕtrans)
            H = exp(ξ)
            @test isapprox(ξ, log(H))

            ξhat = [RigidBodyDynamics.hat(ϕrot) ϕtrans]
            ξhat = [ξhat; zeros(1, 4)]
            H_mat = [RigidBodyDynamics.rotation_matrix(H.rot) H.trans]
            H_mat = [H_mat; zeros(1, 3) 1.]
            @test isapprox(expm(ξhat), H_mat; atol = 1e-10)
        end

        # test without rotation but with nonzero translation:
        ξ = Twist{Float64}(f2, f1, f1, zeros(SVector{3}), rand(SVector{3}))
        H = exp(ξ)
        @test isapprox(ξ, log(H))

        # test rotation for θ > 2 * π
        for θ in [linspace(2 * π - 10 * eps(), 2 * π + 10 * eps(), 100) linspace(2 * π, 6 * π, 100)]
            ω = normalize(rand(SVector{3}))
            ξ1 = Twist(f2, f1, f1, ω * θ, zeros(SVector{3}))
            ξ2 = Twist(f2, f1, f1, ω * mod(θ, 2 * π), zeros(SVector{3}))
            @test isapprox(exp(ξ1), exp(ξ2); atol = 1e-10)
        end

        # derivative
        for θ in linspace(1e-3, 2 * π - 1e-3, 100) # autodiff doesn't work close to the identity rotation
            ξ = Twist{Float64}(f2, f1, f1, θ * normalize(rand(SVector{3})), θ * normalize(rand(SVector{3})))
            H = exp(ξ)
            T = Twist{Float64}(f2, f1, f2, rand(SVector{3}), rand(SVector{3}))
            ξ2, ξ̇ = RigidBodyDynamics.log_with_time_derivative(H, T)
            @test isapprox(ξ, ξ2)
            # autodiff log. Need time derivative of transform in ForwardDiff form, so need to basically v_to_qdot for quaternion floating joint

            ωQuat = Quaternion{Float64}(0., T.angular[1], T.angular[2], T.angular[3], false)
            linear = T.linear
            rotdot = 0.5 * H.rot * ωQuat
            transdot = RigidBodyDynamics.rotate(linear, H.rot)

            trans_autodiff = @SVector [ForwardDiff.Dual(H.trans[i], transdot[i]) for i in 1 : 3]
            rot_autodiff = Quaternion{eltype(trans_autodiff)}(
                ForwardDiff.Dual(H.rot.s, rotdot.s),
                ForwardDiff.Dual(H.rot.v1, rotdot.v1),
                ForwardDiff.Dual(H.rot.v2, rotdot.v2),
                ForwardDiff.Dual(H.rot.v3, rotdot.v3), true)
            H_autodiff = Transform3D(H.from, H.to, rot_autodiff, trans_autodiff)
            ξ_autodiff = log(H_autodiff)
            ξ̇rot_from_autodiff = @SVector [ForwardDiff.partials(ξ_autodiff.angular[i])[1] for i in 1 : 3]
            ξ̇trans_from_autodiff = @SVector [ForwardDiff.partials(ξ_autodiff.linear[i])[1] for i in 1 : 3]
            ξ̇_from_autodiff = SpatialAcceleration(ξ.body, ξ.base, ξ.frame, ξ̇rot_from_autodiff, ξ̇trans_from_autodiff)
            @test isapprox(ξ̇, ξ̇_from_autodiff; atol = 1e-10)
        end
    end
end
