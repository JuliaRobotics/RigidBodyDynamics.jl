@testset "pd" begin
    @testset "scalar" begin
        @test pd(PDGains(1, 2), 3, 4) == -11
    end

    @testset "vector" begin
        gains = PDGains(1, 2)
        e = [3; 4; 5]
        ė = [6; 7; 8]
        @test pd(gains, e, ė) == ((e, ė) -> pd(gains, e, ė)).(e, ė)
    end

    @testset "specifying desireds" begin
        gains = PDGains(5, 6)
        x = SVector(1, 2)
        ẋ = SVector(5, 6)
        @test pd(gains, x, ẋ) == pd(gains, x, zero(x), ẋ, zero(ẋ))
    end

    @testset "x-axis rotation" begin
        Random.seed!(56)
        gains = PDGains(2, 3)
        e = RotX(rand())
        ė = rand() * SVector(1, 0, 0)
        @test pd(gains, e, ė)[1] ≈ pd(gains, e.theta, ė[1])
    end

    @testset "orientation control" begin
        Random.seed!(57)
        mechanism = rand_floating_tree_mechanism(Float64) # single floating body
        joint = first(tree_joints(mechanism))
        body = successor(joint, mechanism)
        base = root_body(mechanism)

        state = MechanismState(mechanism)
        rand!(state)

        Rdes = rand(RotMatrix{3})
        ωdes = zero(SVector{3})
        gains = PDGains(100, 20)
        v̇des = similar(velocity(state))

        control_dynamics_result = DynamicsResult(mechanism)
        function control!(torques::SegmentedVector, t, state::MechanismState)
            H = transform_to_root(state, body)
            T = transform(twist_wrt_world(state, body), inv(H))
            R = rotation(H)
            ω = T.angular
            ωddes = pd(gains, R, Rdes, ω, ωdes)
            v̇desjoint = v̇des[joint]
            v̇desjoint .= SVector([ωddes; zero(ωddes)])
            wrenches = control_dynamics_result.jointwrenches
            accelerations = control_dynamics_result.accelerations
            inverse_dynamics!(torques, wrenches, accelerations, state, v̇des)
        end

        final_time = 3.
        simulate(state, final_time, control!; Δt = 1e-3)

        H = transform_to_root(state, body)
        T = transform(twist_wrt_world(state, body), inv(H))
        R = rotation(H)
        ω = T.angular

        @test isapprox(R * Rdes', one(typeof(Rdes)); atol = 1e-8)
        @test isapprox(ω, zero(ω); atol = 1e-8)
    end

    @testset "pose control" begin
        Random.seed!(58)
        mechanism = rand_floating_tree_mechanism(Float64) # single floating body
        joint = first(tree_joints(mechanism))
        body = successor(joint, mechanism)
        base = root_body(mechanism)

        baseframe = default_frame(base)
        desiredframe = CartesianFrame3D("desired")
        actualframe = frame_after(joint)

        state = MechanismState(mechanism)
        rand!(state)

        xdes = rand(Transform3D, desiredframe, baseframe)
        vdes = zero(Twist{Float64}, desiredframe, baseframe, actualframe)
        v̇des = similar(velocity(state))
        gains = SE3PDGains(baseframe, PDGains(100 * one(SMatrix{3, 3}), 20), PDGains(100., 20.)) # world-fixed gains

        control_dynamics_result = DynamicsResult(mechanism)
        function control!(torques::SegmentedVector, t, state::MechanismState)
            x = transform_to_root(state, body)
            invx = inv(x)
            v = transform(twist_wrt_world(state, body), invx)
            v̇desjoint = v̇des[joint]
            v̇desjoint .= SVector(pd(transform(gains, invx), x, xdes, v, vdes))
            wrenches = control_dynamics_result.jointwrenches
            accelerations = control_dynamics_result.accelerations
            inverse_dynamics!(torques, wrenches, accelerations, state, v̇des)
        end

        final_time = 3.
        simulate(state, final_time, control!; Δt = 1e-3)

        x = transform_to_root(state, body)
        v = transform(twist_wrt_world(state, body), inv(x))
        @test isapprox(x, xdes * one(Transform3D, actualframe, desiredframe), atol = 1e-6)
        @test isapprox(v, vdes + zero(Twist{Float64}, actualframe, desiredframe, actualframe), atol = 1e-6)
    end

    @testset "linearized SE(3) control" begin
        Random.seed!(59)
        for i = 1 : 100
            randpsd3() = (x = rand(SMatrix{3, 3}); x' * x)
            baseframe = CartesianFrame3D("base")
            bodyframe = CartesianFrame3D("body")
            gains = SE3PDGains(bodyframe, PDGains(randpsd3(), randpsd3()), PDGains(randpsd3(), randpsd3()))

            xdes = rand(Transform3D{Float64}, bodyframe, baseframe)
            Tdes = rand(Twist{Float64}, bodyframe, baseframe, bodyframe)

            ϵ = 1e-2
            x = xdes * Transform3D(bodyframe, bodyframe, AngleAxis(ϵ, randn(), randn(), randn()), ϵ * randn(SVector{3}))
            T = rand(Twist{Float64}, bodyframe, baseframe, bodyframe)

            accel_nonlin = pd(gains, x, xdes, T, Tdes)
            accel_lin = pd(gains, x, xdes, T, Tdes, SE3PDMethod{:Linearized}())
            @test isapprox(accel_nonlin, accel_lin; atol = 1e-6)

            T = Tdes
            accel_nonlin = pd(gains, x, xdes, T, Tdes)
            accel_lin = pd(gains, x, xdes, T, Tdes, SE3PDMethod{:Linearized}())
            @test isapprox(accel_nonlin, accel_lin; atol = 1e-6)
        end
    end
end
