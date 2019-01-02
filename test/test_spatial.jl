function Ad(H::Transform3D)
    hat = RigidBodyDynamics.Spatial.hat
    p_hat = hat(translation(H))
    [[rotation(H) zero(SMatrix{3, 3, eltype(H)})]; [p_hat * rotation(H) rotation(H)]]
end

@testset "spatial" begin
    f1 = CartesianFrame3D("1")
    f2 = CartesianFrame3D("2")
    f3 = CartesianFrame3D("3")
    f4 = CartesianFrame3D("4")

    @testset "rotation vector rate" begin
        Random.seed!(62)
        hat = RigidBodyDynamics.Spatial.hat
        rotation_vector_rate = RigidBodyDynamics.Spatial.rotation_vector_rate
        for ϕ in (rand(SVector{3}), zero(SVector{3})) # exponential coordinates (rotation vector)
            ω = rand(SVector{3}) # angular velocity in body frame
            R = RotMatrix(RodriguesVec(ϕ...))
            Ṙ = R * hat(ω)
            ϕ̇ = rotation_vector_rate(ϕ, ω)
            Θ = norm(ϕ)
            if Θ > eps(Θ)
                ϕ_autodiff = ForwardDiff.Dual.(ϕ, ϕ̇)
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
        @test vcross * M == colwise(×, v, M)
        @test colwise(×, M, v) == -colwise(×, v, M)
        @test colwise(+, M, v) == broadcast(+, M, v)
        v2 = @SVector [1, 2, 3, 4]
        @test_throws DimensionMismatch colwise(+, M, v2)
    end

    @testset "show" begin
        Random.seed!(63)
        show(devnull, rand(SpatialInertia{Float64}, f1))
        show(devnull, rand(Twist{Float64}, f2, f1, f3))
        show(devnull, rand(SpatialAcceleration{Float64}, f2, f1, f3))
        show(devnull, rand(Wrench{Float64}, f2))
        show(devnull, rand(Momentum{Float64}, f2))

        n = 5
        show(devnull, GeometricJacobian(f2, f1, f3, rand(SMatrix{3, n}), rand(SMatrix{3, n})))
        show(devnull, MomentumMatrix(f2, rand(SMatrix{3, n}), rand(SMatrix{3, n})))
        show(devnull, WrenchMatrix(f2, rand(SMatrix{3, n}), rand(SMatrix{3, n})))
    end

    @testset "spatial inertia" begin
        Random.seed!(64)
        I2 = rand(SpatialInertia{Float64}, f2)
        H21 = rand(Transform3D, f2, f1)
        I1 = transform(I2, H21)
        I3 = rand(SpatialInertia{Float64}, f2)
        @test I2.mass == I1.mass
        @test isapprox(SMatrix(I1), Ad(inv(H21))' * SMatrix(I2) * Ad(inv(H21)); atol = 1e-12)
        @test isapprox(I2, transform(I1, inv(H21)))
        @test isapprox(SMatrix(I2) + SMatrix(I3), SMatrix(I2 + I3); atol = 1e-12)
        @inferred transform(zero(SpatialInertia{Float32}, f1), one(Transform3D, f1))
        @test I2 + zero(I2) == I2

        # Test that the constructor works with dynamic arrays (which are
        # converted to static arrays internally)
        I4 = @inferred(SpatialInertia(f2, Matrix(1.0I, 3, 3), zeros(3), 1.0))

        # Ensure that the kwarg constructor matches the positional argument constructor.
        inertia = rand(SpatialInertia{Float64}, f1)
        centroidal = CartesianFrame3D("centroidal")
        to_centroidal = Transform3D(f1, centroidal, -center_of_mass(inertia).v)
        inertia_centroidal = transform(inertia, to_centroidal)
        @test inertia_centroidal.frame == centroidal
        @test center_of_mass(inertia_centroidal) ≈ Point3D(Float64, centroidal) atol=1e-12
        inertia_back = SpatialInertia(inertia.frame, moment_about_com=inertia_centroidal.moment, com=center_of_mass(inertia).v, mass=inertia.mass)
        @test inertia ≈ inertia_back atol=1e-12
    end

    @testset "twist" begin
        Random.seed!(65)
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
        @test isapprox(SVector(transform(T1, H31)), Ad(H31) * SVector(T1))
        @test T3 + zero(T3) == T3

        # 2.17 in Duindam:
        f0 = CartesianFrame3D("0")
        fi = CartesianFrame3D("i")
        Qi = Point3D(fi, rand(SVector{3}))
        H = rand(Transform3D, fi, f0)
        T0 = log(H)
        Q0 = H * Qi
        Q̇0 = point_velocity(T0, Q0)
        f = t -> Array((exp(Twist(T0.body, T0.base, T0.frame, t * angular(T0), t * linear(T0))) * Qi).v)
        Q̇0check = ForwardDiff.derivative(f, 1.)
        @test isapprox(Q̇0.v, Q̇0check)
    end

    @testset "wrench" begin
        Random.seed!(66)
        W = rand(Wrench{Float64}, f2)
        H21 = rand(Transform3D, f2, f1)
        @test isapprox(SVector(transform(W, H21)), Ad(inv(H21))' * SVector(W))
        @test_throws ArgumentError transform(W, inv(H21)) # wrong frame
        @test W + zero(W) == W

        point2 = Point3D(f2, zero(SVector{3}))
        force2 = FreeVector3D(f2, rand(SVector{3}))
        W2 = Wrench(point2, force2)
        @test isapprox(angular(W2), zero(SVector{3}))
        @test isapprox(linear(W2), force2.v)
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
        Random.seed!(67)
        T = rand(Twist{Float64}, f2, f1, f2)
        I = rand(SpatialInertia{Float64}, f2)
        T2 = rand(Twist{Float64}, f2, f1, f1)
        H21 = rand(Transform3D, f2, f1)
        h = I * T
        @test isapprox(SMatrix(I) * SVector(T), SVector(h); atol = 1e-12)
        @test_throws ArgumentError I * T2 # wrong frame
        @test isapprox(transform(I, H21) * transform(T, H21), transform(h, H21))
        @test isapprox(SVector(transform(h, H21)), Ad(inv(H21))' * SVector(h))
        @test_throws ArgumentError transform(h, inv(H21)) # wrong frame
        @test h + zero(h) == h
    end

    @testset "geometric jacobian, power" begin
        Random.seed!(68)
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

    @testset "mul! with transpose(mat)" begin
        Random.seed!(69)
        mat = WrenchMatrix(f1, rand(SMatrix{3, 4}), rand(SMatrix{3, 4}))
        vec = rand(SpatialAcceleration{Float64}, f2, f3, f1)
        k = fill(NaN, size(mat, 2))
        mul!(k, transpose(mat), vec)
        @test isapprox(k, angular(mat)' * angular(vec) + linear(mat)' * linear(vec), atol = 1e-14)
        @test k == transpose(mat) * vec
    end

    @testset "momentum matrix" begin
        Random.seed!(70)
        n = 13
        A = MomentumMatrix(f3, rand(SMatrix{3, n}), rand(SMatrix{3, n}))
        v = rand(size(A, 2))
        h = Momentum(A, v)
        H = rand(Transform3D, f3, f1)
        @test h.frame == A.frame
        @test isapprox(Momentum(transform(A, H), v), transform(h, H))
    end

    @testset "spatial acceleration" begin
        Random.seed!(71)
        I = rand(SpatialInertia{Float64}, f2)
        Ṫ = rand(SpatialAcceleration{Float64}, f2, f1, f2)
        T = rand(Twist{Float64}, f2, f1, f2)
        W = newton_euler(I, Ṫ, T)
        H = rand(Transform3D, f2, f1)
        @test isapprox(transform(newton_euler(transform(I, H), transform(Ṫ, H, T, T), transform(T, H)), inv(H)), W)
        @test Ṫ + zero(Ṫ) == Ṫ
    end

    @testset "point_velocity, point_acceleration" begin
        body = CartesianFrame3D("body")
        base = CartesianFrame3D("base")
        frame = CartesianFrame3D("some other frame") # yes, the math does check out if this is different from the other two; ṗ will just be rotated to this frame
        T = rand(Twist{Float64}, body, base, frame)
        Ṫ = rand(SpatialAcceleration{Float64}, body, base, frame)
        p = Point3D(frame, rand(SVector{3}))
        ṗ = point_velocity(T, p)
        p̈ = point_acceleration(T, Ṫ, p)
        @test p̈.frame == frame
        p_dual = Point3D(p.frame, ForwardDiff.Dual.(p.v, ṗ.v))
        T_dual = Twist(T.body, T.base, T.frame, ForwardDiff.Dual.(angular(T), angular(Ṫ)), ForwardDiff.Dual.(linear(T), linear(Ṫ)))
        ṗ_dual = point_velocity(T_dual, p_dual)
        p̈_check = FreeVector3D(ṗ_dual.frame, map(x -> ForwardDiff.partials(x, 1), ṗ_dual.v))
        @test p̈ ≈ p̈_check atol=1e-12
    end

    @testset "kinetic energy" begin
        Random.seed!(72)
        I = rand(SpatialInertia{Float64}, f2)
        T = rand(Twist{Float64}, f2, f1, f2)
        H = rand(Transform3D, f2, f1)
        Ek = kinetic_energy(I, T)
        @test isapprox((1//2 * SVector(T)' * SMatrix(I) * SVector(T))[1], Ek; atol = 1e-12)
        @test isapprox(kinetic_energy(transform(I, H), transform(T, H)), Ek; atol = 1e-12)
    end

    @testset "log / exp" begin
        Random.seed!(74) # TODO: https://github.com/JuliaRobotics/RigidBodyDynamics.jl/issues/135
        hat = RigidBodyDynamics.Spatial.hat
        for θ in [LinRange(0., 10 * eps(), 100);
                  LinRange(0., π - eps(), 100)]
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
            @test isapprox(exp(ξhat), H_mat)
        end

        # test without rotation but with nonzero translation:
        ξ = Twist{Float64}(f2, f1, f1, zero(SVector{3}), rand(SVector{3}))
        H = exp(ξ)
        @test isapprox(ξ, log(H))

        # test rotation for θ > π
        for θ in [LinRange(π - 10 * eps(), π + 10 * eps(), 100);
                  LinRange(π, 6 * π, 100)]
            ω = normalize(rand(SVector{3}))
            ξ1 = Twist(f2, f1, f1, ω * θ, zero(SVector{3}))
            ξ2 = Twist(f2, f1, f1, ω * mod(θ, 2 * π), zero(SVector{3}))
            @test isapprox(exp(ξ1), exp(ξ2))
        end

        # derivative
        for θ in LinRange(1e-3, π - 1e-3, 100) # autodiff doesn't work close to the identity rotation
            hat = RigidBodyDynamics.Spatial.hat
            ξ = Twist{Float64}(f2, f1, f1, θ * normalize(rand(SVector{3})), θ * normalize(rand(SVector{3})))
            H = exp(ξ)
            T = Twist{Float64}(f2, f1, f2, rand(SVector{3}), rand(SVector{3}))
            ξ2, ξ̇ = RigidBodyDynamics.log_with_time_derivative(H, T)
            @test isapprox(ξ, ξ2)
            # autodiff log. Need time derivative of transform in ForwardDiff form, so need to basically v_to_qdot for quaternion floating joint
            rotdot = rotation(H) * hat(angular(T))
            transdot = rotation(H) * linear(T)

            trans_autodiff = @SVector [ForwardDiff.Dual(translation(H)[i], transdot[i]) for i in 1 : 3]
            rot_autodiff = RotMatrix(@SMatrix [ForwardDiff.Dual(rotation(H)[i, j], rotdot[i, j]) for i in 1 : 3, j in 1 : 3])
            H_autodiff = Transform3D(H.from, H.to, rot_autodiff, trans_autodiff)
            ξ_autodiff = log(H_autodiff)
            ξ̇rot_from_autodiff = @SVector [ForwardDiff.partials(angular(ξ_autodiff)[i])[1] for i in 1 : 3]
            ξ̇trans_from_autodiff = @SVector [ForwardDiff.partials(linear(ξ_autodiff)[i])[1] for i in 1 : 3]
            ξ̇_from_autodiff = SpatialAcceleration(ξ.body, ξ.base, ξ.frame, ξ̇rot_from_autodiff, ξ̇trans_from_autodiff)
            @test isapprox(ξ̇, ξ̇_from_autodiff)
        end
    end

    @testset "RodriguesVec linearization" begin
        Random.seed!(75)
        ϵ = 1e-3
        for i = 1 : 100
            e = RotMatrix(AngleAxis(ϵ, randn(), randn(), randn()))
            rv = RodriguesVec(e)
            rv_lin = linearized_rodrigues_vec(e)
            lin_error = AngleAxis(rv \ rv_lin)
            @test rotation_angle(lin_error) ≈ 0 atol = 1e-8
        end
    end

    @testset "Conversions to vector/matrix" begin
        f = CartesianFrame3D()
        angular = [1, 2, 3]
        linear = [4, 5, 6]
        twist = Twist(f, f, f, angular, linear)
        svec = SVector(angular..., linear...)
        twist64 = Twist(f, f, f, Float64.(angular), Float64.(linear))
        svec64 = SVector{6, Float64}(svec)
        @test twist isa Twist{Int}
        @test Twist{Float64}(twist) === twist64
        @test convert(Twist{Float64}, twist) === twist64
        @test SVector{6, Float64}(twist) === svec64
        @test convert(SVector{6, Float64}, twist) === svec64
        @test SVector{6}(twist) === svec
        @test convert(SVector{6}, twist) === svec
        @test SArray(twist) === svec
        @test convert(SArray, twist) === svec

        moment = [1 2 3;
                  2 4 5;
                  3 5 6]
        crosspart = [7, 8, 9]
        mass = 10
        inertia = SpatialInertia(f, moment, crosspart, mass)
        inertia64 = SpatialInertia(f, Float64.(moment), Float64.(crosspart), Float64(mass))
        mat = SMatrix(inertia) # already tested above
        mat64 = SMatrix{6, 6, Float64}(mat)
        @test inertia isa SpatialInertia{Int}
        @test SpatialInertia{Float64}(inertia) === inertia64
        @test convert(SpatialInertia{Float64}, inertia) === inertia64
        @test SMatrix{6, 6}(inertia) === mat
        @test convert(SMatrix{6, 6}, inertia) === mat
        @test SMatrix{6, 6, Float64}(inertia) === mat64
        @test convert(SMatrix{6, 6, Float64}, inertia) === mat64
        @test SMatrix(inertia64) === mat64
        @test convert(SMatrix, inertia64) === mat64

        J = GeometricJacobian(f, f, f, rand(1:10, 3, 4), rand(1:10, 3, 4))
        @test convert(Array, J) == Array(J) == Matrix(J) == [J.angular; J.linear]
        @test convert(Array{Float64}, J) == Array{Float64}(J) == Matrix{Float64}(J) == Float64[J.angular; J.linear]
        @test Matrix(J) == [J.angular; J.linear]
        @test Matrix(J) == [J.angular; J.linear]
    end
end
