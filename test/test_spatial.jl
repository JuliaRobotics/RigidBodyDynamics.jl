function Ad(H::Transform3D)
    hat = RigidBodyDynamics.Spatial.hat
    p_hat = hat(translation(H))
    [[rotation(H) zeros(SMatrix{3, 3, eltype(H)})]; [p_hat * rotation(H) rotation(H)]]
end

@testset "spatial" begin
    f1 = CartesianFrame3D("1")
    f2 = CartesianFrame3D("2")
    f3 = CartesianFrame3D("3")
    f4 = CartesianFrame3D("4")

    @testset "rotation vector rate" begin
        hat = RigidBodyDynamics.Spatial.hat
        rotation_vector_rate = RigidBodyDynamics.Spatial.rotation_vector_rate
        for ϕ in (rand(SVector{3}), zeros(SVector{3})) # exponential coordinates (rotation vector)
            ω = rand(SVector{3}) # angular velocity in body frame
            R = RotMatrix(RodriguesVec(ϕ...))
            Ṙ = R * hat(ω)
            ϕ̇ = rotation_vector_rate(ϕ, ω)
            Θ = norm(ϕ)
            if Θ > eps(Θ)
                ϕ_autodiff = SVector{3}(create_autodiff(ϕ, ϕ̇))
                R_autodiff = RotMatrix(RodriguesVec(ϕ_autodiff...))
                Ṙ_from_autodiff = map(x -> ForwardDiff.partials(x)[1], R_autodiff)
                @test isapprox(Ṙ_from_autodiff, Ṙ)
            else
                @test isapprox(ϕ̇, ω) # limit case; hard to test using autodiff because of division by zero
            end
        end
    end

    @testset "colwise" begin
        colwise = RigidBodyDynamics.Spatial.colwise
        v = @SVector [2, 4, 6]
        M = @SMatrix [1 2 3; 4 5 6; 7 8 9]
        T = eltype(v)
        vcross = @SMatrix [zero(T) -v[3] v[2];
                    v[3] zero(T) -v[1];
                    -v[2] v[1] zero(T)]
        @test vcross * M == colwise(cross, v, M)
        @test colwise(cross, M, v) == -colwise(cross, v, M)
        @test colwise(+, M, v) == broadcast(+, M, v)
        v2 = @SVector [1, 2, 3, 4]
        @test_throws DimensionMismatch colwise(+, M, v2)
    end

    @testset "show" begin
        show(DevNull, rand(SpatialInertia{Float64}, f1))
        show(DevNull, rand(Twist{Float64}, f2, f1, f3))
        show(DevNull, rand(SpatialAcceleration{Float64}, f2, f1, f3))
        show(DevNull, rand(Wrench{Float64}, f2))
        show(DevNull, rand(Momentum{Float64}, f2))

        n = 5
        show(DevNull, GeometricJacobian(f2, f1, f3, rand(SMatrix{3, n}), rand(SMatrix{3, n})))
        show(DevNull, MomentumMatrix(f2, rand(SMatrix{3, n}), rand(SMatrix{3, n})))
        show(DevNull, WrenchMatrix(f2, rand(SMatrix{3, n}), rand(SMatrix{3, n})))
    end

    @testset "spatial inertia" begin
        I2 = rand(SpatialInertia{Float64}, f2)
        H21 = rand(Transform3D, f2, f1)
        I1 = transform(I2, H21)
        I3 = rand(SpatialInertia{Float64}, f2)
        @test I2.mass == I1.mass
        @test isapprox(Array(I1), Ad(inv(H21))' * Array(I2) * Ad(inv(H21)); atol = 1e-12)
        @test isapprox(I2, transform(I1, inv(H21)))
        @test isapprox(Array(I2) + Array(I3), Array(I2 + I3); atol = 1e-12)
        @inferred transform(zero(SpatialInertia{Float32}, f1), eye(Transform3D, f1))
        @test I2 + zero(I2) == I2

        # Test that the constructor works with dynamic arrays (which are
        # converted to static arrays internally)
        I4 = @inferred(SpatialInertia(f2, eye(3), zeros(3), 1.0))
    end

    @testset "twist" begin
        T1 = rand(Twist{Float64}, f2, f1, f3)
        T2 = rand(Twist{Float64}, f3, f2, f3)
        T3 = T1 + T2
        H31 = rand(Transform3D, f3, f1)
        @test T3.body == T2.body
        @test T3.base == T1.base
        @test T3.frame == f3
        # @test isapprox(T2 + T1, T3) # used to be allowed, but makes code slower; just switch T1 and T2 around
        @test_throws ArgumentError T1 + rand(Twist{Float64}, f3, f2, f4) # wrong frame
        @test_throws ArgumentError T1 + rand(Twist{Float64}, f3, f4, f3) # wrong base
        @test isapprox(Array(transform(T1, H31)), Ad(H31) * Array(T1))
        @test T3 + zero(T3) == T3

        # 2.17 in Duindam:
        f0 = CartesianFrame3D("0")
        fi = CartesianFrame3D("i")
        Qi = Point3D(fi, rand(SVector{3}))
        H = rand(Transform3D, fi, f0)
        T0 = log(H)
        Q0 = H * Qi
        Q̇0 = point_velocity(T0, Q0)
        f = t -> Array((exp(Twist(T0.body, T0.base, T0.frame, t * T0.angular, t * T0.linear)) * Qi).v)
        Q̇0check = ForwardDiff.derivative(f, 1.)
        @test isapprox(Q̇0.v, Q̇0check)
    end

    @testset "wrench" begin
        W = rand(Wrench{Float64}, f2)
        H21 = rand(Transform3D, f2, f1)
        @test isapprox(Array(transform(W, H21)), Ad(inv(H21))' * Array(W))
        @test_throws ArgumentError transform(W, inv(H21)) # wrong frame
        @test W + zero(W) == W

        point2 = Point3D(f2, zeros(SVector{3}))
        force2 = FreeVector3D(f2, rand(SVector{3}))
        W2 = Wrench(point2, force2)
        @test isapprox(W2.angular, zeros(SVector{3}))
        @test isapprox(W2.linear, force2.v)
        @test W2.frame == force2.frame

        point1 = H21 * point2
        force1 = H21 * force2
        W1 = Wrench(point1, force1)
        @test W1.frame == f1
        @test isapprox(W1, transform(W2, H21))
        @test_throws ArgumentError Wrench(point1, force2) # wrong frame

        @testset "wrench constructed from plain Vectors" begin
            point1 = Point3D(f1, [1., 0., 0.])
            force1 = FreeVector3D(f1, [0, 1, 0])
            w = @inferred(Wrench(point1, force1))
            @test isa(w, Wrench{Float64})
        end

    end

    @testset "momentum" begin
        T = rand(Twist{Float64}, f2, f1, f2)
        I = rand(SpatialInertia{Float64}, f2)
        T2 = rand(Twist{Float64}, f2, f1, f1)
        H21 = rand(Transform3D, f2, f1)
        h = I * T
        @test isapprox(Array(I) * Array(T), Array(h); atol = 1e-12)
        @test_throws ArgumentError I * T2 # wrong frame
        @test isapprox(transform(I, H21) * transform(T, H21), transform(h, H21))
        @test isapprox(Array(transform(h, H21)), Ad(inv(H21))' * Array(h))
        @test_throws ArgumentError transform(h, inv(H21)) # wrong frame
        @test h + zero(h) == h
    end

    @testset "geometric jacobian, power" begin
        n = 14
        J = GeometricJacobian(f2, f1, f3, rand(SMatrix{3, n}), rand(SMatrix{3, n}))
        v = rand(size(J, 2))
        W = rand(Wrench{Float64}, f3)
        T = Twist(J, v)
        H = rand(Transform3D, f3, f1)
        τ = torque(J, W)
        @test J.body == T.body
        @test J.base == T.base
        @test J.frame == T.frame
        @test isapprox(Twist(transform(J, H), v), transform(T, H))
        @test isapprox(dot(Array(τ), v), dot(T, W); atol = 1e-12) # power equality
        @test_throws ArgumentError dot(transform(T, H), W)
        @test_throws ArgumentError torque(transform(J, H), W)

        Jmutable = GeometricJacobian(f2, f1, f3, rand(3, n), rand(3, n))
        @test isapprox(Twist(transform(Jmutable, H), v), transform(Twist(Jmutable, v), H))
    end

    @testset "At_mul_B!" begin
        mat = WrenchMatrix(f1, rand(SMatrix{3, 4}), rand(SMatrix{3, 4}))
        vec = rand(SpatialAcceleration{Float64}, f2, f3, f1)
        k = fill(NaN, size(mat, 2))
        At_mul_B!(k, mat, vec)
        @test isapprox(k, mat.angular' * vec.angular + mat.linear' * vec.linear, atol = 1e-14)
    end

    @testset "momentum matrix" begin
        n = 13
        A = MomentumMatrix(f3, rand(SMatrix{3, n}), rand(SMatrix{3, n}))
        v = rand(size(A, 2))
        h = Momentum(A, v)
        H = rand(Transform3D, f3, f1)
        @test h.frame == A.frame
        @test isapprox(Momentum(transform(A, H), v), transform(h, H))
    end

    @testset "spatial acceleration" begin
        I = rand(SpatialInertia{Float64}, f2)
        Ṫ = rand(SpatialAcceleration{Float64}, f2, f1, f2)
        T = rand(Twist{Float64}, f2, f1, f2)
        W = newton_euler(I, Ṫ, T)
        H = rand(Transform3D, f2, f1)
        @test isapprox(transform(newton_euler(transform(I, H), transform(Ṫ, H, T, T), transform(T, H)), inv(H)), W)
        @test Ṫ + zero(Ṫ) == Ṫ
    end

    @testset "kinetic energy" begin
        I = rand(SpatialInertia{Float64}, f2)
        T = rand(Twist{Float64}, f2, f1, f2)
        H = rand(Transform3D, f2, f1)
        Ek = kinetic_energy(I, T)
        @test isapprox((1//2 * Array(T)' * Array(I) * Array(T))[1], Ek; atol = 1e-12)
        @test isapprox(kinetic_energy(transform(I, H), transform(T, H)), Ek; atol = 1e-12)
    end

    @testset "log / exp" begin
        hat = RigidBodyDynamics.Spatial.hat
        srand(1) # TODO: https://github.com/tkoolen/RigidBodyDynamics.jl/issues/135
        for θ in [linspace(0., 10 * eps(), 100); linspace(0., π - eps(), 100)]
            # have magnitude of parts of twist be bounded by θ to check for numerical issues
            ϕrot = normalize(rand(SVector{3})) * θ * 2 * (rand() - 0.5)
            ϕtrans = normalize(rand(SVector{3})) * θ * 2 * (rand() - 0.5)
            ξ = Twist{Float64}(f2, f1, f1, ϕrot, ϕtrans)
            H = exp(ξ)
            @test isapprox(ξ, log(H))

            ξhat = [hat(ϕrot) ϕtrans]
            ξhat = [ξhat; zeros(1, 4)]
            H_mat = [rotation(H) translation(H)]
            H_mat = [H_mat; zeros(1, 3) 1.]
            @test isapprox(expm(ξhat), H_mat)
        end

        # test without rotation but with nonzero translation:
        ξ = Twist{Float64}(f2, f1, f1, zeros(SVector{3}), rand(SVector{3}))
        H = exp(ξ)
        @test isapprox(ξ, log(H))

        # test rotation for θ > π
        for θ in [linspace(π - 10 * eps(), π + 10 * eps(), 100) linspace(π, 6 * π, 100)]
            ω = normalize(rand(SVector{3}))
            ξ1 = Twist(f2, f1, f1, ω * θ, zeros(SVector{3}))
            ξ2 = Twist(f2, f1, f1, ω * mod(θ, 2 * π), zeros(SVector{3}))
            @test isapprox(exp(ξ1), exp(ξ2))
        end

        # derivative
        for θ in linspace(1e-3, π - 1e-3, 100) # autodiff doesn't work close to the identity rotation
            hat = RigidBodyDynamics.Spatial.hat
            ξ = Twist{Float64}(f2, f1, f1, θ * normalize(rand(SVector{3})), θ * normalize(rand(SVector{3})))
            H = exp(ξ)
            T = Twist{Float64}(f2, f1, f2, rand(SVector{3}), rand(SVector{3}))
            ξ2, ξ̇ = RigidBodyDynamics.log_with_time_derivative(H, T)
            @test isapprox(ξ, ξ2)
            # autodiff log. Need time derivative of transform in ForwardDiff form, so need to basically v_to_qdot for quaternion floating joint

            ω = SVector(T.angular[1], T.angular[2], T.angular[3])
            linear = T.linear

            rotdot = rotation(H) * hat(ω)
            transdot = rotation(H) * linear

            trans_autodiff = @SVector [ForwardDiff.Dual(translation(H)[i], transdot[i]) for i in 1 : 3]
            rot_autodiff = RotMatrix(@SMatrix [ForwardDiff.Dual(rotation(H)[i, j], rotdot[i, j]) for i in 1 : 3, j in 1 : 3])
            H_autodiff = Transform3D(H.from, H.to, rot_autodiff, trans_autodiff)
            ξ_autodiff = log(H_autodiff)
            ξ̇rot_from_autodiff = @SVector [ForwardDiff.partials(ξ_autodiff.angular[i])[1] for i in 1 : 3]
            ξ̇trans_from_autodiff = @SVector [ForwardDiff.partials(ξ_autodiff.linear[i])[1] for i in 1 : 3]
            ξ̇_from_autodiff = SpatialAcceleration(ξ.body, ξ.base, ξ.frame, ξ̇rot_from_autodiff, ξ̇trans_from_autodiff)
            @test isapprox(ξ̇, ξ̇_from_autodiff)
        end
    end
end
