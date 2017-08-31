function RigidBodyDynamics.rand_tree_mechanism()
    rand_tree_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 5]; [Fixed{Float64} for i = 1 : 5]; [QuaternionSpherical{Float64} for i = 1 : 5]; [Prismatic{Float64} for i = 1 : 5]; [Planar{Float64} for i = 1 : 5]]...)
end

@testset "mechanism algorithms" begin
    @testset "show" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
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
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
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

        q2 = rand(num_positions(mechanism))
        v2 = rand(num_velocities(mechanism))
        q2copy = deepcopy(q2)
        v2copy = deepcopy(v2)
        x2 = MechanismState(mechanism, q2, v2)
        @test configuration(x2) === q2
        @test velocity(x2) === v2
        @test all(configuration(x2) .== q2copy)
        @test all(velocity(x2) .== v2copy)

        @test @inferred(num_positions(mechanism)) == num_positions(x)
        @test @inferred(num_velocities(mechanism)) == num_velocities(x)
    end

    @testset "q̇ <-> v" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
        q = configuration(x)
        q̇ = configuration_derivative(x)
        v = velocity(x)
        for joint in joints(mechanism)
            qjoint = configuration(x, joint)
            q̇joint = q̇[configuration_range(x, joint)]
            vjoint = velocity(x, joint)
            vjoint_from_q̇ = similar(vjoint)
            configuration_derivative_to_velocity!(vjoint_from_q̇, joint, qjoint, q̇joint)
            @test isapprox(vjoint, vjoint_from_q̇; atol = 1e-12)
        end
    end

    @testset "joint_torque! / motion_subspace" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
        for joint in tree_joints(mechanism)
            body = successor(joint, mechanism)
            qjoint = configuration(x, joint)
            wrench = rand(Wrench{Float64}, frame_after(joint))
            τ = Vector{Float64}(num_velocities(joint))
            joint_torque!(τ, joint, qjoint, wrench)
            S = motion_subspace_in_world(x, joint)
            @test isapprox(τ, torque(S, transform(wrench, transform_to_root(x, body))))
        end
    end

    @testset "isfloating" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
        for joint in joints(mechanism)
            S = motion_subspace_in_world(x, joint)
            @test isfloating(joint) == (rank(Array(S)) == 6)
            @test isfloating(joint) == (num_constraints(joint) == 0)
        end
    end

    @testset "geometric_jacobian / relative_twist" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
        frame = CartesianFrame3D()
        for i = 1 : 100
            bs = Set(bodies(mechanism))
            body = rand([bs...])
            delete!(bs, body)
            base = rand([bs...])
            p = path(mechanism, base, body)
            v = velocity(x)

            J = geometric_jacobian(x, p)
            T = relative_twist(x, body, base)
            @test isapprox(Twist(J, v), T; atol = 1e-12)

            J1 = GeometricJacobian(J.body, J.base, J.frame, similar(J.angular), similar(J.linear))
            geometric_jacobian!(J1, x, p)
            @test isapprox(Twist(J1, v), T; atol = 1e-12)

            H = rand(Transform3D, root_frame(mechanism), frame)
            J2 = GeometricJacobian(J.body, J.base, frame, similar(J.angular), similar(J.linear))
            if num_velocities(p) > 0
                @test_throws ArgumentError geometric_jacobian!(J, x, p, H)
            end
            geometric_jacobian!(J2, x, p, H)
            @test isapprox(Twist(J2, v), transform(T, H); atol = 1e-12)

            J3 = GeometricJacobian(J.body, J.base, default_frame(body), similar(J.angular), similar(J.linear))
            geometric_jacobian!(J3, x, p)
            @test isapprox(Twist(J3, v), transform(x, T, default_frame(body)); atol = 1e-12)
        end
    end

    @testset "motion_subspace / constraint_wrench_subspace" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
        for joint in tree_joints(mechanism)
            body = successor(joint, mechanism)
            qjoint = configuration(x, joint)
            S = motion_subspace_in_world(x, joint)
            tf = joint_transform(joint, qjoint)
            T = transform(constraint_wrench_subspace(joint, tf), transform_to_root(x, body))
            @test isapprox(T.angular' * S.angular + T.linear' * S.linear, zeros(num_constraints(joint), num_velocities(joint)); atol = 1e-12)
        end
    end

    # TODO: good test, but currently don't want to support joints with variable wrench subspaces:
    # @testset "constraint_bias" begin
    #     for joint in joints(mechanism)
    #         qjoint = configuration(x, joint)
    #         vjoint = velocity(x, joint)
    #         q̇joint = similar(qjoint)
    #         velocity_to_configuration_derivative!(joint, q̇joint, qjoint, vjoint)
    #         qjoint_autodiff = create_autodiff(qjoint, q̇joint)
    #         TAutodiff = constraint_wrench_subspace(joint, qjoint_autodiff)#::RigidBodyDynamics.WrenchSubspace{eltype(qjoint_autodiff)} # TODO
    #         angular = map(x -> ForwardDiff.partials(x, 1), TAutodiff.angular)
    #         linear = map(x -> ForwardDiff.partials(x, 1), TAutodiff.linear)
    #         Ṫ = WrenchMatrix(frame_after(joint), angular, linear)
    #         joint_twist = transform(x, relative_twist(x, frame_after(joint), frame_before(joint)), frame_after(joint))
    #         bias = fill(NaN, 6 - num_velocities(joint))
    #         constraint_bias!(joint, bias, joint_twist)
    #         @test isapprox(Ṫ.angular' * joint_twist.angular + Ṫ.linear' * joint_twist.linear, bias; atol = 1e-14)
    #     end
    # end

    @testset "relative_acceleration" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
        result = DynamicsResult(mechanism)
        for body in bodies(mechanism)
            for base in bodies(mechanism)
                v̇ = rand(num_velocities(mechanism))
                spatial_accelerations!(result.accelerations, x, v̇)
                Ṫ = relative_acceleration(result, body, base)
                q = configuration(x)
                v = velocity(x)
                q̇ = configuration_derivative(x)
                q_autodiff = create_autodiff(q, q̇)
                v_autodiff = create_autodiff(v, v̇)
                x_autodiff = MechanismState{eltype(q_autodiff)}(mechanism)
                set_configuration!(x_autodiff, q_autodiff)
                set_velocity!(x_autodiff, v_autodiff)
                twist_autodiff = relative_twist(x_autodiff, body, base)
                accel_vec = [ForwardDiff.partials(x, 1)::Float64 for x in (Array(twist_autodiff))]
                @test isapprox(Array(Ṫ), accel_vec; atol = 1e-12)

                root = root_body(mechanism)
                f = default_frame(body)
                Ṫbody = transform(x, relative_acceleration(result, body, root), f)
                Ṫbase = transform(x, relative_acceleration(result, base, root), f)
                @test isapprox(transform(x, -Ṫbase + Ṫbody, Ṫ.frame), Ṫ; atol = 1e-12)
            end
        end
    end

    @testset "motion subspace / twist wrt world" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
        for joint in tree_joints(mechanism)
            body = successor(joint, mechanism)
            parent_body = predecessor(joint, mechanism)
            @test isapprox(relative_twist(x, body, parent_body), Twist(motion_subspace_in_world(x, joint), velocity(x, joint)); atol = 1e-12)
        end
    end

    @testset "composite rigid body inertias" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
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
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
        A = momentum_matrix(x)
        Amat = Array(A)
        for joint in tree_joints(mechanism)
            if num_velocities(joint) > 0 # TODO: Base.mapslices can't handle matrices with zero columns
                body = successor(joint, mechanism)
                Ajoint = Amat[:, velocity_range(x, joint)]
                @test isapprox(Array(crb_inertia(x, body) * motion_subspace_in_world(x, joint)), Ajoint; atol = 1e-12)
            end
        end

        v = velocity(x)
        hsum = sum(b -> spatial_inertia(x, b) * twist_wrt_world(x, b), non_root_bodies(mechanism))
        @test isapprox(Momentum(A, v), hsum; atol = 1e-12)

        A1 = MomentumMatrix(A.frame, similar(A.angular), similar(A.linear))
        momentum_matrix!(A1, x)
        @test isapprox(Momentum(A1, v), hsum; atol = 1e-12)

        frame = CartesianFrame3D()
        A2 = MomentumMatrix(frame, similar(A.angular), similar(A.linear))
        H = rand(Transform3D, root_frame(mechanism), frame)
        @test_throws ArgumentError momentum_matrix!(A, x, H)
        momentum_matrix!(A2, x, H)
        @test isapprox(Momentum(A2, v), transform(hsum, H); atol = 1e-12)

        body = rand(collect(bodies(mechanism)))
        A3 = MomentumMatrix(default_frame(body), similar(A.angular), similar(A.linear))
        momentum_matrix!(A3, x)
        @test isapprox(Momentum(A3, v), transform(x, hsum, default_frame(body)); atol = 1e-12)
    end

    @testset "mass matrix / kinetic energy" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
        Ek = kinetic_energy(x)
        M = mass_matrix(x)
        v = velocity(x)
        @test isapprox(1/2 * dot(v, M * v), Ek; atol = 1e-12)

        q = configuration(x)
        kinetic_energy_fun = v -> begin
            local x = MechanismState{eltype(v)}(mechanism)
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
        mechanism = rand_tree_mechanism()
        body = rand(collect(non_root_bodies(mechanism)))
        newinertia = rand(SpatialInertia{eltype(mechanism)}, spatial_inertia(body).frame)
        spatial_inertia!(body, newinertia)
        @assert spatial_inertia(body) == newinertia
    end

    @testset "inverse dynamics / acceleration term" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
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
        x = MechanismState{Float64}(mechanism)
        rand!(x)

        function q_to_M(q)
            local x = MechanismState(mechanism, q, zeros(eltype(q), num_velocities(mechanism)))
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
            local x = MechanismState{eltype(v)}(mechanism)
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
        x = MechanismState(mechanism)
        rand!(x)
        v̇ = zeros(num_velocities(mechanism))
        zero_velocity!(x)
        g = inverse_dynamics(x, v̇)

        function q_to_potential(q)
            x = MechanismState{eltype(q)}(mechanism)
            set_configuration!(x, q)
            zero_velocity!(x)
            return [gravitational_potential_energy(x)]
        end

        g2 = similar(g')
        ForwardDiff.jacobian!(g2, q_to_potential, configuration(x))
        @test isapprox(g2, g'; atol = 1e-12)
    end

    @testset "momentum matrix" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
        q = configuration(x)
        q̇ = configuration_derivative(x)
        v = velocity(x)
        v̇ = rand(num_velocities(mechanism))

        # momentum computed two ways
        @test isapprox(Momentum(momentum_matrix(x), v), momentum(x))

        # rate of change of momentum computed using autodiff:
        q_autodiff = create_autodiff(q, q̇)
        v_autodiff = create_autodiff(v, v̇)
        x_autodiff = MechanismState{eltype(q_autodiff)}(mechanism)
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
        mechanism = rand_chain_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; Planar{Float64}; [Prismatic{Float64} for i = 1 : 10]]...) # what really matters is that there's a floating joint first
        x = MechanismState(mechanism)
        rand!(x)
        v̇ = rand(num_velocities(mechanism))
        externalwrenches = Dict(body => rand(Wrench{Float64}, root_frame(mechanism)) for body in bodies(mechanism))
        τ = inverse_dynamics(x, v̇, externalwrenches)
        floatingjoint = first(out_joints(root_body(mechanism), mechanism))
        τfloating = τ[velocity_range(x, floatingjoint)]
        floatingjointwrench = Wrench(frame_after(floatingjoint), SVector{3}(τfloating[1 : 3]), SVector{3}(τfloating[4 : 6]))
        floatingjointwrench = transform(x, floatingjointwrench, root_frame(mechanism))
        ḣ = Wrench(momentum_matrix(x), v̇) + momentum_rate_bias(x) # momentum rate of change
        gravitational_force = mass(mechanism) * mechanism.gravitational_acceleration
        com = center_of_mass(x)
        gravitational_wrench = Wrench(gravitational_force.frame, cross(com, gravitational_force).v, gravitational_force.v)
        total_wrench = floatingjointwrench + gravitational_wrench + sum((b) -> transform(x, externalwrenches[b], root_frame(mechanism)), non_root_bodies(mechanism))
        @test isapprox(total_wrench, ḣ; atol = 1e-12)
    end

    @testset "dynamics / inverse dynamics" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
        external_torques = rand(num_velocities(mechanism))
        externalwrenches = Dict(body => rand(Wrench{Float64}, root_frame(mechanism)) for body in bodies(mechanism))
        result = DynamicsResult(mechanism)
        dynamics!(result, x, external_torques, externalwrenches)
        τ = inverse_dynamics(x, result.v̇, externalwrenches) - external_torques
        @test isapprox(τ, zeros(num_velocities(mechanism)); atol = 1e-10)
    end

    @testset "dynamics_bias / inverse_dynamics" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
        externalwrenches = Dict(body => rand(Wrench{Float64}, root_frame(mechanism)) for body in bodies(mechanism))
        v̇ = zeros(num_velocities(x))
        τ1 = inverse_dynamics(x, v̇, externalwrenches)
        τ2 = dynamics_bias(x, externalwrenches)
        @test τ1 ≈ τ2
    end

    @testset "dynamics ode method" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
        torques = rand(num_velocities(mechanism))
        externalwrenches = Dict(body => rand(Wrench{Float64}, root_frame(mechanism)) for body in bodies(mechanism))

        result1 = DynamicsResult(mechanism)
        ẋ = Vector{Float64}(length(state_vector(x)))
        dynamics!(ẋ, result1, x, state_vector(x), torques, externalwrenches)

        result2 = DynamicsResult(mechanism)
        dynamics!(result2, x, torques, externalwrenches)

        @test isapprox([configuration_derivative(x); result2.v̇], ẋ)
    end

    @testset "power flow" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        rand!(x)
        externalwrenches = Dict(body => rand(Wrench{Float64}, root_frame(mechanism)) for body in bodies(mechanism))
        τ = rand(num_velocities(mechanism))
        result = DynamicsResult(mechanism)
        dynamics!(result, x, τ, externalwrenches)

        q = configuration(x)
        q̇ = configuration_derivative(x)
        v = velocity(x)
        v̇ = result.v̇
        power = τ ⋅ v + sum(body -> externalwrenches[body] ⋅ twist_wrt_world(x, body), non_root_bodies(mechanism))

        q_autodiff = create_autodiff(q, q̇)
        v_autodiff = create_autodiff(v, v̇)
        x_autodiff = MechanismState{eltype(q_autodiff)}(mechanism)
        set_configuration!(x_autodiff, q_autodiff)
        set_velocity!(x_autodiff, v_autodiff)
        energy_autodiff = gravitational_potential_energy(x_autodiff) + kinetic_energy(x_autodiff)
        energy_derivative = ForwardDiff.partials(energy_autodiff)[1]
        @test isapprox(power, energy_derivative, atol = 1e-10)
    end

    @testset "local / global coordinates" begin
        mechanism = rand_tree_mechanism()
        state = MechanismState(mechanism)
        rand!(state)
        for joint in joints(mechanism)
            # back and forth between local and global
            ϕ = Vector{Float64}(num_velocities(joint))
            ϕ̇ = Vector{Float64}(num_velocities(joint))
            q0 = Vector{Float64}(num_positions(joint))
            q = configuration(state, joint)
            v = velocity(state, joint)
            rand_configuration!(q0, joint)
            local_coordinates!(ϕ, ϕ̇, joint, q0, q, v)
            q_back = Vector{Float64}(num_positions(joint))
            global_coordinates!(q_back, joint, q0, ϕ)
            @test isapprox(q, q_back)

            # compare ϕ̇ to autodiff
            q̇ = Vector{Float64}(num_positions(joint))
            velocity_to_configuration_derivative!(q̇, joint, q, v)
            v̇ = rand(num_velocities(joint))
            q_autodiff = create_autodiff(q, q̇)
            v_autodiff = create_autodiff(v, v̇)
            q0_autodiff = create_autodiff(q0, zeros(length(q0)))
            T = eltype(q_autodiff)
            ϕ_autodiff = Vector{T}(length(ϕ))
            ϕ̇_autodiff = Vector{T}(length(ϕ̇))
            local_coordinates!(ϕ_autodiff, ϕ̇_autodiff, joint, q0_autodiff, q_autodiff, v_autodiff)
            ϕ̇_from_autodiff = [ForwardDiff.partials(x)[1] for x in ϕ_autodiff]
            @test isapprox(ϕ̇, ϕ̇_from_autodiff)

            # local coordinates should be zero when q = q0
            # Definition 2.9 in Duindam, "Port-Based Modeling and Control for Efficient Bipedal Walking Robots"
            copy!(q, q0)
            local_coordinates!(ϕ, ϕ̇, joint, q0, q, v)
            @test isapprox(ϕ, zeros(num_velocities(joint)); atol = 1e-6) # FIXME: tolerance is way too high
        end
    end

    @testset "configuration_derivative_to_velocity_adjoint!" begin
        mechanism = rand_tree_mechanism()
        x = MechanismState(mechanism)
        configuration(x) .= rand(num_positions(x)) # needs to work for configuration vectors that do not satisfy the state constraints as well
        fv = rand(num_velocities(x))
        fq = zeros(num_positions(x))
        configuration_derivative_to_velocity_adjoint!(fq, x, fv)
        @test dot(fv, velocity(x)) ≈ dot(fq, configuration_derivative(x))
    end

    @testset "joint bounds" begin
        @test @inferred(RigidBodyDynamics.Bounds{Float64}()).lower == -Inf
        @test @inferred(RigidBodyDynamics.Bounds{Float64}()).upper == Inf
        @test @inferred(RigidBodyDynamics.Bounds(-1, 2.0)).lower == -1
        @test @inferred(RigidBodyDynamics.Bounds(-1, 2.0)).upper == 2
        @test isa(RigidBodyDynamics.Bounds(-1, 1.0).lower, Float64)
        @test isa(RigidBodyDynamics.Bounds(-1, 1.0).upper, Float64)
        @test @inferred(clamp(1.5, RigidBodyDynamics.Bounds(-1, 1))) == RigidBodyDynamics.upper(RigidBodyDynamics.Bounds(-1, 1))
        @test @inferred(clamp(-3, RigidBodyDynamics.Bounds(-2, 1))) == RigidBodyDynamics.lower(RigidBodyDynamics.Bounds(-2, 1))
        @test @inferred(clamp(0, RigidBodyDynamics.Bounds(-1, 1))) == 0
        @test @inferred(intersect(RigidBodyDynamics.Bounds(-1, 1), RigidBodyDynamics.Bounds(-0.5, 2.0))) == RigidBodyDynamics.Bounds(-0.5, 1.0)
    end
end
