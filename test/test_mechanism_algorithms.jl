@testset "mechanism algorithms" begin
    mechanism = rand_tree_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; [Fixed{Float64} for i = 1 : 5]; [Prismatic{Float64} for i = 1 : 10]]...)
    x = MechanismState(Float64, mechanism)
    rand!(x)

    @testset "basic stuff" begin
        q = vcat([configuration(x, vertex.edgeToParentData) for vertex in non_root_vertices(mechanism)]...)
        v = vcat([velocity(x, vertex.edgeToParentData) for vertex in non_root_vertices(mechanism)]...)

        @test q == configuration_vector(x)
        @test v == velocity_vector(x)

        zero_configuration!(x)
        set_configuration!(x, q)
        @test q == configuration_vector(x)

        zero_velocity!(x)
        set_velocity!(x, v)
        @test v == velocity_vector(x)

        qcopy = copy(configuration_vector(x))
        zero_configuration!(x)
        for joint in joints(mechanism)
            set_configuration!(x, joint, qcopy[mechanism.qRanges[joint]])
        end
        @test q == configuration_vector(x)

        vcopy = copy(velocity_vector(x))
        zero_velocity!(x)
        for joint in joints(mechanism)
            set_velocity!(x, joint, vcopy[mechanism.vRanges[joint]])
        end
        @test v == velocity_vector(x)

        zero!(x)
        set!(x, [q; v])

        @test q == configuration_vector(x)
        @test v == velocity_vector(x)
    end

    @testset "q̇ <-> v" begin
        q = configuration_vector(x)
        q̇ = configuration_derivative(x)
        v = velocity_vector(x)
        for joint in joints(mechanism)
            qJoint = q[mechanism.qRanges[joint]]
            q̇Joint = q̇[mechanism.qRanges[joint]]
            vJoint = velocity(x, joint)
            vJointFromq̇ = similar(vJoint)
            configuration_derivative_to_velocity!(joint, vJointFromq̇, qJoint, q̇Joint)
            @test isapprox(vJoint, vJointFromq̇; atol = 1e-12)
        end
    end

    @testset "joint_torque! / geometric_jacobian" begin
        for joint in joints(mechanism)
            qjoint = configuration(x, joint)
            wrench = rand(Wrench{Float64}, joint.frameAfter)
            τ = Vector{Float64}(num_velocities(joint))
            joint_torque!(joint, τ, qjoint, wrench)
            S = motion_subspace(joint, qjoint)
            @test isapprox(τ, joint_torque(S, wrench))
        end
    end

    @testset "geometric_jacobian / relative_twist" begin
        bs = Set(bodies(mechanism))
        body = rand([bs...])
        delete!(bs, body)
        base = rand([bs...])
        p = path(mechanism, base, body)
        J = geometric_jacobian(x, p)
        vpath = velocity_vector(x, p)
        T = relative_twist(x, body, base)
        @test isapprox(Twist(J, vpath), T; atol = 1e-12)
    end

    @testset "relative_acceleration" begin
        bs = Set(bodies(mechanism))
        body = rand([bs...])
        delete!(bs, body)
        base = rand([bs...])
        js = joints(mechanism)
        v̇ = rand(num_velocities(mechanism))
        Ṫ = relative_acceleration(x, body, base, v̇)

        q = configuration_vector(x)
        v = velocity_vector(x)
        q̇ = configuration_derivative(x)
        create_autodiff = (z, dz) -> [ForwardDiff.Dual(z[i]::Float64, dz[i]::Float64) for i in 1 : length(z)]
        q_autodiff = create_autodiff(q, q̇)
        v_autodiff = create_autodiff(v, v̇)
        x_autodiff = MechanismState(eltype(q_autodiff), mechanism)
        set_configuration!(x_autodiff, q_autodiff)
        set_velocity!(x_autodiff, v_autodiff)
        twist_autodiff = relative_twist(x_autodiff, body, base)
        accel_vec = [ForwardDiff.partials(x, 1)::Float64 for x in (Array(twist_autodiff))]

        @test isapprox(Array(Ṫ), accel_vec; atol = 1e-12)
    end

    @testset "motion subspace / twist wrt world" begin
        for vertex in non_root_vertices(mechanism)
            body = vertex.vertexData
            joint = vertex.edgeToParentData
            parentBody = vertex.parent.vertexData
            @test isapprox(relative_twist(x, body, parentBody), Twist(motion_subspace(x, joint), velocity(x, joint)); atol = 1e-12)
        end
    end

    @testset "composite rigid body inertias" begin
        for vertex in non_root_vertices(mechanism)
            body = vertex.vertexData
            crb = crb_inertia(x, body)
            subtree = toposort(vertex)
            @test isapprox(sum((b::RigidBody) -> spatial_inertia(x, b), [v.vertexData for v in subtree]), crb; atol = 1e-12)
        end
    end

    @testset "momentum_matrix / summing momenta" begin
        A = momentum_matrix(x)
        Amat = Array(A)
        for vertex in non_root_vertices(mechanism)
            body = vertex.vertexData
            joint = vertex.edgeToParentData
            Ajoint = Amat[:, mechanism.vRanges[joint]]
            @test isapprox(Array(crb_inertia(x, body) * motion_subspace(x, joint)), Ajoint; atol = 1e-12)
        end

        v = velocity_vector(x)
        h = Momentum(A, v)
        hSum = sum(b -> spatial_inertia(x, b) * twist_wrt_world(x, b), non_root_bodies(mechanism))
        @test isapprox(h, hSum; atol = 1e-12)
    end


    @testset "mass matrix / kinetic energy" begin
        Ek = kinetic_energy(x)
        M = mass_matrix(x)
        v = velocity_vector(x)
        @test isapprox(1/2 * dot(v, M * v), Ek; atol = 1e-12)

        q = configuration_vector(x)
        kinetic_energy_fun = v -> begin
            local x = MechanismState(eltype(v), mechanism)
            set_configuration!(x, q)
            set_velocity!(x, v)
            kinetic_energy(x)
        end

        M2 = similar(M.data)
        # FIXME: changed after updating to ForwardDiff 0.2: chunk size 1 necessary because creating a MechanismState with a max size Dual takes forever...
        M2 = ForwardDiff.hessian!(M2, kinetic_energy_fun, velocity_vector(x), ForwardDiff.Chunk{1}())
        @test isapprox(M2, M; atol = 1e-12)
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
        ForwardDiff.jacobian!(dMdq, q_to_M, configuration_vector(x))
        q̇ = velocity_vector(x)
        Ṁ = reshape(dMdq * q̇, num_velocities(mechanism), num_velocities(mechanism))

        q = configuration_vector(x)
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
            return [potential_energy(x)]
        end

        g2 = similar(g')
        ForwardDiff.jacobian!(g2, q_to_potential, configuration_vector(x))
        @test isapprox(g2, g'; atol = 1e-12)
    end

    @testset "inverse dynamics / external wrenches" begin
        mechanism = rand_chain_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...) # what really matters is that there's a floating joint first
        x = MechanismState(Float64, mechanism)
        rand_configuration!(x)
        rand_velocity!(x)

        q = configuration_vector(x)
        q̇ = configuration_derivative(x)
        v = velocity_vector(x)
        v̇ = rand(num_velocities(mechanism))
        externalWrenches = Dict(body => rand(Wrench{Float64}, root_frame(mechanism)) for body in non_root_bodies(mechanism))
        τ = inverse_dynamics(x, v̇, externalWrenches)
        floatingBodyVertex = root_vertex(mechanism).children[1]
        floatingJoint = floatingBodyVertex.edgeToParentData
        floatingJointWrench = Wrench(floatingBodyVertex.edgeToParentData.frameAfter, τ[mechanism.vRanges[floatingJoint]])
        floatingJointWrench = transform(x, floatingJointWrench, root_frame(mechanism))

        create_autodiff = (z, dz) -> [ForwardDiff.Dual(z[i]::Float64, dz[i]::Float64) for i in 1 : length(z)]
        q_autodiff = create_autodiff(q, q̇)
        v_autodiff = create_autodiff(v, v̇)
        x_autodiff = MechanismState(eltype(q_autodiff), mechanism)
        set_configuration!(x_autodiff, q_autodiff)
        set_velocity!(x_autodiff, v_autodiff)
        A_autodiff = Array(momentum_matrix(x_autodiff))
        A = [ForwardDiff.value(A_autodiff[i, j])::Float64 for i = 1 : size(A_autodiff, 1), j = 1 : size(A_autodiff, 2)]
        Ȧ = [ForwardDiff.partials(A_autodiff[i, j], 1)::Float64 for i = 1 : size(A_autodiff, 1), j = 1 : size(A_autodiff, 2)]
        ḣ = A * v̇ + Ȧ * v # rate of change of momentum

        gravitational_force = mass(mechanism) * mechanism.gravitationalAcceleration
        com = center_of_mass(x)
        gravitational_wrench = Wrench(gravitational_force.frame, cross(com, gravitational_force).v, gravitational_force.v)
        total_wrench = floatingJointWrench + gravitational_wrench + sum((w) -> transform(x, w, root_frame(mechanism)), values(externalWrenches))
        @test isapprox(Array(total_wrench), ḣ; atol = 1e-12)
    end

    @testset "dynamics / inverse dynamics" begin
        mechanism = rand_tree_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...)
        x = MechanismState(Float64, mechanism)
        rand!(x)

        js = joints(mechanism)
        externalWrenches = Dict(body => rand(Wrench{Float64}, root_frame(mechanism)) for body in non_root_bodies(mechanism))
        stateVector = state_vector(x)

        result = DynamicsResult(Float64, mechanism)
        dynamics!(result, x, stateVector, externalWrenches)
        τ = inverse_dynamics(x, result.v̇, externalWrenches)
        @test isapprox(τ, zeros(num_velocities(mechanism)); atol = 1e-12)
    end

    @testset "simulate" begin
        acrobot = parse_urdf(Float64, "urdf/Acrobot.urdf")
        x = MechanismState(Float64, acrobot)
        rand!(x)
        total_energy_before = potential_energy(x) + kinetic_energy(x)
        tspan = [0.; 1.]
        times, states = simulate(x, tspan)
        set!(x, states[end])
        total_energy_after = potential_energy(x) + kinetic_energy(x)
        @test isapprox(total_energy_after, total_energy_before, atol = 1e-3) # fairly loose tolerance here
    end
end
