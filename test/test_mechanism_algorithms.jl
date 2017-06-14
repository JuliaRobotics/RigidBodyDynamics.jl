# @testset "mechanism algorithms" begin
    mechanism = rand_tree_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; [Fixed{Float64} for i = 1 : 5]; [Prismatic{Float64} for i = 1 : 10]]...)
    x = MechanismState(Float64, mechanism)
    rand!(x)

    @testset "show" begin
        mechanism_with_loops = deepcopy(mechanism)
        for i = 1 : 5
            pred = rand(bodies(mechanism_with_loops))
            succ = rand(bodies(mechanism_with_loops))
            joint = Joint("non-tree-$i", Fixed{Float64}())
            attach!(mechanism_with_loops, pred, joint, eye(Transform3D, frame_before(joint), default_frame(pred)), succ)
        end

        show(DevNull, mechanism_with_loops)
        for joint in joints(mechanism_with_loops)
            show(DevNull, joint)
            showcompact(DevNull, joint)
        end
        for body in bodies(mechanism_with_loops)
            show(DevNull, body)
            showcompact(DevNull, body)
        end
        show(DevNull, x)
    end

    @testset "basic stuff" begin
        q = vcat([configuration(x, joint) for joint in tree_joints(mechanism)]...)
        v = vcat([velocity(x, joint) for joint in tree_joints(mechanism)]...)

        @test q == configuration(x)
        @test v == velocity(x)

        zero_configuration!(x)
        set_configuration!(x, q)
        @test q == configuration(x)

        zero_velocity!(x)
        set_velocity!(x, v)
        @test v == velocity(x)

        qcopy = copy(configuration(x))
        zero_configuration!(x)
        for joint in joints(mechanism)
            set_configuration!(x, joint, qcopy[configuration_range(x, joint)])
        end
        @test q == configuration(x)

        vcopy = copy(velocity(x))
        zero_velocity!(x)
        for joint in joints(mechanism)
            set_velocity!(x, joint, vcopy[velocity_range(x, joint)])
        end
        @test v == velocity(x)

        zero!(x)
        set!(x, [q; v])

        @test q == configuration(x)
        @test v == velocity(x)
    end

    @testset "q̇ <-> v" begin
        q = configuration(x)
        q̇ = configuration_derivative(x)
        v = velocity(x)
        for joint in joints(mechanism)
            qJoint = configuration(x, joint)
            q̇Joint = q̇[configuration_range(x, joint)]
            vJoint = velocity(x, joint)
            vJointFromq̇ = similar(vJoint)
            configuration_derivative_to_velocity!(joint, vJointFromq̇, qJoint, q̇Joint)
            @test isapprox(vJoint, vJointFromq̇; atol = 1e-12)
        end
    end

    @testset "joint_torque! / motion_subspace" begin
        for joint in tree_joints(mechanism)
            qjoint = configuration(x, joint)
            wrench = rand(Wrench{Float64}, frame_after(joint))
            τ = Vector{Float64}(num_velocities(joint))
            joint_torque!(joint, τ, qjoint, wrench)
            S = motion_subspace(joint, qjoint)
            # @test isapprox(τ, torque(S, wrench)) # TODO: https://github.com/JuliaLang/julia/issues/20034
        end
    end

    @testset "geometric_jacobian / relative_twist" begin
        frame = CartesianFrame3D()
        for i = 1 : 100
            bs = Set(bodies(mechanism))
            body = rand([bs...])
            delete!(bs, body)
            base = rand([bs...])
            p = RigidBodyDynamics.path(mechanism, base, body)
            vpath = velocity(x, p)

            J = geometric_jacobian(x, p)
            T = relative_twist(x, body, base)
            @test isapprox(Twist(J, vpath), T; atol = 1e-12)

            J1 = GeometricJacobian(J.body, J.base, J.frame, similar(J.angular), similar(J.linear))
            geometric_jacobian!(J1, x, p)
            @test isapprox(Twist(J1, vpath), T; atol = 1e-12)

            H = rand(Transform3D, root_frame(mechanism), frame)
            J2 = GeometricJacobian(J.body, J.base, frame, similar(J.angular), similar(J.linear))
            @test_throws ArgumentError geometric_jacobian!(J, x, p, H)
            geometric_jacobian!(J2, x, p, H)
            @test isapprox(Twist(J2, vpath), transform(T, H); atol = 1e-12)

            J3 = GeometricJacobian(J.body, J.base, default_frame(body), similar(J.angular), similar(J.linear))
            geometric_jacobian!(J3, x, p)
            @test isapprox(Twist(J3, vpath), transform(x, T, default_frame(body)); atol = 1e-12)
        end
    end

    @testset "motion_subspace / constraint_wrench_subspace" begin
        for joint in tree_joints(mechanism)
            qjoint = configuration(x, joint)
            S = motion_subspace(joint, qjoint)
            jointTransform = joint_transform(joint, qjoint)
            T = constraint_wrench_subspace(joint, jointTransform)::RigidBodyDynamics.WrenchSubspace{Float64} # TODO
            # @test isapprox(T.angular' * S.angular + T.linear' * S.linear, zeros(6 - num_velocities(joint), num_velocities(joint)); atol = 1e-14) # TODO: https://github.com/JuliaLang/julia/issues/20034
        end
    end

    # TODO: good test, but currently don't want to support joints with variable wrench subspaces:
    # @testset "constraint_bias" begin
    #     for joint in joints(mechanism)
    #         qjoint = configuration(x, joint)
    #         vjoint = velocity(x, joint)
    #         q̇joint = similar(qjoint)
    #         velocity_to_configuration_derivative!(joint, q̇joint, qjoint, vjoint)
    #         qjointAutodiff = create_autodiff(qjoint, q̇joint)
    #         TAutodiff = constraint_wrench_subspace(joint, qjointAutodiff)#::RigidBodyDynamics.WrenchSubspace{eltype(qjointAutodiff)} # TODO
    #         angular = map(x -> ForwardDiff.partials(x, 1), TAutodiff.angular)
    #         linear = map(x -> ForwardDiff.partials(x, 1), TAutodiff.linear)
    #         Ṫ = WrenchMatrix(frame_after(joint), angular, linear)
    #         jointTwist = transform(x, relative_twist(x, frame_after(joint), frame_before(joint)), frame_after(joint))
    #         bias = fill(NaN, 6 - num_velocities(joint))
    #         constraint_bias!(joint, bias, jointTwist)
    #         @test isapprox(Ṫ.angular' * jointTwist.angular + Ṫ.linear' * jointTwist.linear, bias; atol = 1e-14)
    #     end
    # end

    @testset "relative_acceleration" begin
        for body in bodies(mechanism)
            for base in bodies(mechanism)
                v̇ = rand(num_velocities(mechanism))
                Ṫ = relative_acceleration(x, body, base, v̇)
                q = configuration(x)
                v = velocity(x)
                q̇ = configuration_derivative(x)
                q_autodiff = create_autodiff(q, q̇)
                v_autodiff = create_autodiff(v, v̇)
                x_autodiff = MechanismState(eltype(q_autodiff), mechanism)
                set_configuration!(x_autodiff, q_autodiff)
                set_velocity!(x_autodiff, v_autodiff)
                twist_autodiff = relative_twist(x_autodiff, body, base)
                accel_vec = [ForwardDiff.partials(x, 1)::Float64 for x in (Array(twist_autodiff))]
                @test isapprox(Array(Ṫ), accel_vec; atol = 1e-12)

                root = root_body(mechanism)
                f = default_frame(body)
                Ṫbody = transform(x, relative_acceleration(x, body, root, v̇), f)
                Ṫbase = transform(x, relative_acceleration(x, base, root, v̇), f)
                @test isapprox(transform(x, -Ṫbase + Ṫbody, Ṫ.frame), Ṫ; atol = 1e-12)
            end
        end
    end

    @testset "motion subspace / twist wrt world" begin
        for joint in tree_joints(mechanism)
            body = successor(joint, mechanism)
            parentBody = predecessor(joint, mechanism)
            @test isapprox(relative_twist(x, body, parentBody), Twist(motion_subspace_in_world(x, joint), velocity(x, joint)); atol = 1e-12)
        end
    end

    @testset "composite rigid body inertias" begin
        for joint in tree_joints(mechanism)
            body = successor(joint, mechanism)
            crb = crb_inertia(x, body)
            stack = [body]
            subtree = typeof(body)[]
            while !isempty(stack)
                b = pop!(stack)
                push!(subtree, b)
                for joint in out_joints(b, mechanism)
                    push!(stack, successor(joint, mechanism))
                end
            end
            @test isapprox(sum((b::RigidBody) -> spatial_inertia(x, b), subtree), crb; atol = 1e-12)
        end
    end

    @testset "momentum_matrix / summing momenta" begin
        A = momentum_matrix(x)
        Amat = Array(A)
        for joint in tree_joints(mechanism)
            body = successor(joint, mechanism)
            Ajoint = Amat[:, velocity_range(x, joint)]
            @test isapprox(Array(crb_inertia(x, body) * motion_subspace_in_world(x, joint)), Ajoint; atol = 1e-12)
        end

        v = velocity(x)
        hSum = sum(b -> spatial_inertia(x, b) * twist_wrt_world(x, b), non_root_bodies(mechanism))
        @test isapprox(Momentum(A, v), hSum; atol = 1e-12)

        A1 = MomentumMatrix(A.frame, similar(A.angular), similar(A.linear))
        momentum_matrix!(A1, x)
        @test isapprox(Momentum(A1, v), hSum; atol = 1e-12)

        frame = CartesianFrame3D()
        A2 = MomentumMatrix(frame, similar(A.angular), similar(A.linear))
        H = rand(Transform3D, root_frame(mechanism), frame)
        @test_throws ArgumentError momentum_matrix!(A, x, H)
        momentum_matrix!(A2, x, H)
        @test isapprox(Momentum(A2, v), transform(hSum, H); atol = 1e-12)

        body = rand(collect(bodies(mechanism)))
        A3 = MomentumMatrix(default_frame(body), similar(A.angular), similar(A.linear))
        momentum_matrix!(A3, x)
        @test isapprox(Momentum(A3, v), transform(x, hSum, default_frame(body)); atol = 1e-12)
    end

    @testset "mass matrix / kinetic energy" begin
        Ek = kinetic_energy(x)
        M = mass_matrix(x)
        v = velocity(x)
        @test isapprox(1/2 * dot(v, M * v), Ek; atol = 1e-12)

        q = configuration(x)
        kinetic_energy_fun = v -> begin
            local x = MechanismState(eltype(v), mechanism)
            set_configuration!(x, q)
            set_velocity!(x, v)
            kinetic_energy(x)
        end

        M2 = similar(M.data)
        # FIXME: changed after updating to ForwardDiff 0.2: chunk size 1 necessary because creating a MechanismState with a max size Dual takes forever...
        M2 = ForwardDiff.hessian!(M2, kinetic_energy_fun, v, ForwardDiff.HessianConfig{1}(v))
        @test isapprox(M2, M; atol = 1e-12)
    end

    @testset "spatial_inertia!" begin
        body = rand(collect(non_root_bodies(mechanism)))
        newinertia = rand(SpatialInertia{eltype(mechanism)}, spatial_inertia(body).frame)
        spatial_inertia!(body, newinertia)
        @assert spatial_inertia(body) == newinertia
    end

    @testset "inverse dynamics / acceleration term" begin
        M = mass_matrix(x)

        function v̇_to_τ(v̇)
            inverse_dynamics(x, v̇)
        end
        M2 = similar(M.data)
        ForwardDiff.jacobian!(M2, v̇_to_τ, zeros(Float64, num_velocities(mechanism)))
        @test isapprox(M2, M; atol = 1e-12)
    end

    @testset "inverse dynamics / Coriolis term" begin
        mechanism = rand_tree_mechanism(Float64, [[Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...) # skew symmetry property tested later on doesn't hold when q̇ ≠ v
        x = MechanismState(Float64, mechanism)
        rand!(x)

        function q_to_M(q)
            local x = MechanismState(eltype(q), mechanism)
            set_configuration!(x, q)
            zero_velocity!(x)
            vec(mass_matrix(x))
        end
        nv = num_velocities(mechanism)
        nq = num_positions(mechanism)
        dMdq = zeros(nv * nv, nq)
        ForwardDiff.jacobian!(dMdq, q_to_M, configuration(x))
        q̇ = velocity(x)
        Ṁ = reshape(dMdq * q̇, num_velocities(mechanism), num_velocities(mechanism))

        q = configuration(x)
        v̇ = zeros(num_velocities(mechanism))
        function v_to_c(v)
            local x = MechanismState(eltype(v), mechanism)
            set_configuration!(x, q)
            set_velocity!(x, v)
            inverse_dynamics(x, v̇)
        end
        C = similar(Ṁ)
        ForwardDiff.jacobian!(C, v_to_c, q̇)
        C *= 1/2

        skew = Ṁ - 2 * C;
        @test isapprox(skew + skew', zeros(size(skew)); atol = 1e-12)
    end

    @testset "inverse dynamics / gravity term" begin
        mechanism = rand_tree_mechanism(Float64, [[Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...)
        x = MechanismState(Float64, mechanism)
        rand!(x)
        v̇ = zeros(num_velocities(mechanism))
        zero_velocity!(x)
        g = inverse_dynamics(x, v̇)

        function q_to_potential(q)
            x = MechanismState(eltype(q), mechanism)
            set_configuration!(x, q)
            zero_velocity!(x)
            return [gravitational_potential_energy(x)]
        end

        g2 = similar(g')
        ForwardDiff.jacobian!(g2, q_to_potential, configuration(x))
        @test isapprox(g2, g'; atol = 1e-12)
    end

    @testset "momentum matrix" begin
        mechanism = rand_chain_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...)
        x = MechanismState(Float64, mechanism)
        rand_configuration!(x)
        rand_velocity!(x)
        q = configuration(x)
        q̇ = configuration_derivative(x)
        v = velocity(x)
        v̇ = rand(num_velocities(mechanism))

        # momentum computed two ways
        @test isapprox(Momentum(momentum_matrix(x), v), momentum(x))

        # rate of change of momentum computed using autodiff:
        q_autodiff = create_autodiff(q, q̇)
        v_autodiff = create_autodiff(v, v̇)
        x_autodiff = MechanismState(eltype(q_autodiff), mechanism)
        set_configuration!(x_autodiff, q_autodiff)
        set_velocity!(x_autodiff, v_autodiff)
        A_autodiff = Array(momentum_matrix(x_autodiff))
        A = [ForwardDiff.value(A_autodiff[i, j])::Float64 for i = 1 : size(A_autodiff, 1), j = 1 : size(A_autodiff, 2)]
        Ȧ = [ForwardDiff.partials(A_autodiff[i, j], 1)::Float64 for i = 1 : size(A_autodiff, 1), j = 1 : size(A_autodiff, 2)]
        ḣArray = A * v̇ + Ȧ * v

        # rate of change of momentum computed without autodiff:
        ḣ = Wrench(momentum_matrix(x), v̇) + momentum_rate_bias(x)
        @test isapprox(ḣArray, Array(ḣ); atol = 1e-12)
    end

    @testset "inverse dynamics / external wrenches" begin
        mechanism = rand_chain_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...) # what really matters is that there's a floating joint first
        x = MechanismState(Float64, mechanism)
        rand_configuration!(x)
        rand_velocity!(x)

        v̇ = rand(num_velocities(mechanism))
        externalwrenches = Dict(body => rand(Wrench{Float64}, root_frame(mechanism)) for body in bodies(mechanism))
        τ = inverse_dynamics(x, v̇, externalwrenches)
        floatingjoint = first(out_joints(root_body(mechanism), mechanism))
        τfloating = τ[velocity_range(x, floatingjoint)]
        floatingjointwrench = Wrench(frame_after(floatingjoint), SVector{3}(τfloating[1 : 3]), SVector{3}(τfloating[4 : 6]))
        floatingjointwrench = transform(x, floatingjointwrench, root_frame(mechanism))
        ḣ = Wrench(momentum_matrix(x), v̇) + momentum_rate_bias(x) # momentum rate of change
        gravitational_force = mass(mechanism) * mechanism.gravitationalAcceleration
        com = center_of_mass(x)
        gravitational_wrench = Wrench(gravitational_force.frame, cross(com, gravitational_force).v, gravitational_force.v)
        total_wrench = floatingjointwrench + gravitational_wrench + sum((b) -> transform(x, externalwrenches[b], root_frame(mechanism)), non_root_bodies(mechanism))
        @test isapprox(total_wrench, ḣ; atol = 1e-12)
    end

    @testset "dynamics / inverse dynamics" begin
        mechanism = rand_tree_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...)
        x = MechanismState(Float64, mechanism)
        rand!(x)
        externalTorques = rand(num_velocities(mechanism))
        externalwrenches = Dict(body => rand(Wrench{Float64}, root_frame(mechanism)) for body in bodies(mechanism))

        result = DynamicsResult(Float64, mechanism)
        dynamics!(result, x, externalTorques, externalwrenches)
        τ = inverse_dynamics(x, result.v̇, externalwrenches) - externalTorques
        @test isapprox(τ, zeros(num_velocities(mechanism)); atol = 1e-10)
    end

    @testset "dynamics ode method" begin
        mechanism = rand_tree_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...)
        x = MechanismState(Float64, mechanism)
        rand!(x)
        torques = rand(num_velocities(mechanism))
        externalwrenches = Dict(body => rand(Wrench{Float64}, root_frame(mechanism)) for body in bodies(mechanism))

        result1 = DynamicsResult(Float64, mechanism)
        ẋ = Vector{Float64}(length(state_vector(x)))
        dynamics!(ẋ, result1, x, state_vector(x), torques, externalwrenches)

        result2 = DynamicsResult(Float64, mechanism)
        dynamics!(result2, x, torques, externalwrenches)

        @test isapprox([configuration_derivative(x); result2.v̇], ẋ)
    end

    @testset "power flow" begin
        mechanism = rand_chain_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...) # what really matters is that there's a floating joint first
        x = MechanismState(Float64, mechanism)
        rand_configuration!(x)
        rand_velocity!(x)
        externalwrenches = Dict(body => rand(Wrench{Float64}, root_frame(mechanism)) for body in bodies(mechanism))
        τ = rand(num_velocities(mechanism))
        result = DynamicsResult(Float64, mechanism)
        dynamics!(result, x, τ, externalwrenches)

        q = configuration(x)
        q̇ = configuration_derivative(x)
        v = velocity(x)
        v̇ = result.v̇
        power = τ ⋅ v + sum(body -> externalwrenches[body] ⋅ twist_wrt_world(x, body), non_root_bodies(mechanism))

        q_autodiff = create_autodiff(q, q̇)
        v_autodiff = create_autodiff(v, v̇)
        x_autodiff = MechanismState(eltype(q_autodiff), mechanism)
        set_configuration!(x_autodiff, q_autodiff)
        set_velocity!(x_autodiff, v_autodiff)
        energy_autodiff = gravitational_potential_energy(x_autodiff) + kinetic_energy(x_autodiff)
        energy_derivative = ForwardDiff.partials(energy_autodiff)[1]
        @test isapprox(power, energy_derivative, atol = 1e-10)
    end

    @testset "local / global coordinates" begin
        mechanism = rand_tree_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...)
        state = MechanismState(Float64, mechanism)
        rand!(state)
        for joint in joints(mechanism)
            # back and forth between local and global
            ϕ = Vector{Float64}(num_velocities(joint))
            ϕ̇ = Vector{Float64}(num_velocities(joint))
            q0 = Vector{Float64}(num_positions(joint))
            q = configuration(state, joint)
            v = velocity(state, joint)
            rand_configuration!(joint, q0)
            local_coordinates!(joint, ϕ, ϕ̇, q0, q, v)
            q_back = Vector{Float64}(num_positions(joint))
            global_coordinates!(joint, q_back, q0, ϕ)
            @test isapprox(q, q_back)

            # compare ϕ̇ to autodiff
            q̇ = Vector{Float64}(num_positions(joint))
            velocity_to_configuration_derivative!(joint, q̇, q, v)
            v̇ = rand(num_velocities(joint))
            q_autodiff = create_autodiff(q, q̇)
            v_autodiff = create_autodiff(v, v̇)
            q0_autodiff = create_autodiff(q0, zeros(length(q0)))
            T = eltype(q_autodiff)
            ϕ_autodiff = Vector{T}(length(ϕ))
            ϕ̇_autodiff = Vector{T}(length(ϕ̇))
            local_coordinates!(joint, ϕ_autodiff, ϕ̇_autodiff, q0_autodiff, q_autodiff, v_autodiff)
            ϕ̇_from_autodiff = [ForwardDiff.partials(x)[1] for x in ϕ_autodiff]
            @test isapprox(ϕ̇, ϕ̇_from_autodiff)

            # local coordinates should be zero when q = q0
            # Definition 2.9 in Duindam, "Port-Based Modeling and Control for Efficient Bipedal Walking Robots"
            copy!(q, q0)
            local_coordinates!(joint, ϕ, ϕ̇, q0, q, v)
            @test isapprox(ϕ, zeros(num_velocities(joint)); atol = 1e-6) # FIXME: tolerance is way too high
        end
    end
# end
