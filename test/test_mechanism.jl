function test_mechanism()
    mechanism = rand_tree_mechanism(Float64, [QuaternionFloating; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...)

    x = MechanismState{Float64}(mechanism)
    rand!(x)
    cache = MechanismStateCache(mechanism, x)

    # basic stuff
    let
        v = vcat([x.v[vertex.edgeToParentData] for vertex in mechanism.toposortedTree[2 : end]]...)
        @test isapprox(v, velocity_vector(x))
        q = vcat([x.q[vertex.edgeToParentData] for vertex in mechanism.toposortedTree[2 : end]]...)
        @test isapprox(q, configuration_vector(x))
    end

    # q̇ <-> v
    let
        qdot = velocity_to_configuration_derivative(mechanism, x.q, x.v)
        vCheck = configuration_derivative_to_velocity(mechanism, x.q, qdot)
        for joint in joints(mechanism)
            @test isapprox(x.v[joint], vCheck[joint]; atol = 1e-12)
        end
    end


    # geometric_jacobian / relative_twist
    let
        bs = Set(bodies(mechanism))
        body = rand([bs...])
        delete!(bs, body)
        base = rand([bs...])
        p = path(mechanism, base, body)
        J = geometric_jacobian(cache, p)
        vpath = vcat([x.v[joint] for joint in p.edgeData]...)
        T = relative_twist(cache, body, base)
        @test isapprox(J * vpath, T; atol = 1e-12)
    end

    # motion subspace / twist wrt world
    let
        for vertex in mechanism.toposortedTree[2 : end]
            body = vertex.vertexData
            joint = vertex.edgeToParentData
            parentBody = vertex.parent.vertexData
            @test isapprox(motion_subspace(cache, joint) * x.v[joint], relative_twist(cache, body, parentBody))
        end
    end

    # crb inertias
    let
        for vertex in mechanism.toposortedTree[2 : end]
            body = vertex.vertexData
            crb = crb_inertia(cache, body)
            subtree = toposort(vertex)
            crbCheck = sum((b::RigidBody) -> spatial_inertia(cache, b), [v.vertexData for v in subtree])
            @test isapprox(crb, crbCheck)
        end
    end

    # momentum_matrix / summing momenta
    let
        A = momentum_matrix(cache)
        for vertex in mechanism.toposortedTree[2 : end]
            body = vertex.vertexData
            joint = vertex.edgeToParentData
            start = cache.velocityVectorStartIndices[joint]
            Ajoint = A.mat[:, start : start + num_velocities(joint) - 1]
            @test isapprox(Ajoint, (crb_inertia(cache, body) * motion_subspace(cache, joint)).mat)
        end

        v = velocity_vector(x)
        h = A * v
        hSum = sum((b::RigidBody) -> isroot(b) ? zero(Momentum{Float64}, A.frame) : spatial_inertia(cache, b) * twist_wrt_world(cache, b), bodies(mechanism))
        @test isapprox(hSum, h)
    end

    # mass matrix / kinetic energy
    let
        Ek = kinetic_energy(cache)
        M = mass_matrix(cache)
        v = velocity_vector(x)
        @test isapprox(Ek, 1/2 * dot(v, M * v); atol = 1e-12)

        q = configuration_vector(x)
        kinetic_energy_fun = v -> begin
            local x = MechanismState{eltype(v)}(mechanism)
            set_configuration!(x, q)
            set_velocity!(x, v)
            local cache = MechanismStateCache(mechanism, x)
            return kinetic_energy(cache)
        end
        M2 = ForwardDiff.hessian(kinetic_energy_fun, velocity_vector(x))
        @test isapprox(M, M2; atol = 1e-12)
    end

    # inverse dynamics
    let
        # test acceleration terms
        let
            v̇_to_τ = v̇ -> begin
                v̇Dict = Dict{Joint, Vector{eltype(v̇)}}()
                for joint in keys(x.v)
                    v̇Dict[joint] = v̇[cache.velocityVectorStartIndices[joint] : cache.velocityVectorStartIndices[joint] + num_velocities(joint) - 1]
                end
                return inverse_dynamics(cache, v̇Dict)
            end
            M = ForwardDiff.jacobian(v̇_to_τ, zeros(Float64, num_velocities(mechanism)))
            @test isapprox(M, mass_matrix(cache); atol = 1e-12)
        end

        # test Coriolis term
        mechanism = rand_tree_mechanism(Float64, [[Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...) # skew symmetry property tested later on doesn't hold when q̇ ≠ v
        x = MechanismState{Float64}(mechanism)
        rand!(x)
        q_to_M = q -> begin
            local x = MechanismState{eltype(q)}(mechanism)
            set_configuration!(x, q)
            zero_velocity!(x)
            local cache = MechanismStateCache(mechanism, x)
            return vec(mass_matrix(cache))
        end
        dMdq = ForwardDiff.jacobian(q_to_M, configuration_vector(x))
        q̇ = velocity_vector(x)
        Ṁ = reshape(dMdq * q̇, num_velocities(mechanism), num_velocities(mechanism))

        q = configuration_vector(x)
        v̇ = Dict([j::Joint => zeros(num_velocities(j))::Vector{Float64} for j in joints(mechanism)])
        v_to_c = v -> begin
            local x = MechanismState{eltype(v)}(mechanism)
            set_configuration!(x, q)
            set_velocity!(x, v)
            local cache = MechanismStateCache(mechanism, x)
            return inverse_dynamics(cache, v̇)
        end
        C = 1/2 * ForwardDiff.jacobian(v_to_c, q̇)

        skew = Ṁ - 2 * C;
        @test isapprox(skew + skew', zeros(size(skew)); atol = 1e-10)

        # test gravitational term
        v̇ = Dict([j::Joint => zeros(num_velocities(j))::Vector{Float64} for j in joints(mechanism)])
        zero_velocity!(x)
        cache = MechanismStateCache(mechanism, x)
        g = inverse_dynamics(cache, v̇)
        q_to_potential = q -> begin
            local x = MechanismState{eltype(q)}(mechanism)
            set_configuration!(x, q)
            zero_velocity!(x)
            local cache = MechanismStateCache(mechanism, x)
            return [potential_energy(cache)]
        end
        @test isapprox(g, -ForwardDiff.jacobian(q_to_potential, configuration_vector(x))'; atol = 1e-12)


        # TODO: test external wrenches
        # println(dM)
        # Ṁ =

        # nonRootBodies = filter(b -> !isroot(b), bodies(mechanism))
        # v̇ = Dict([joint::Joint => zeros(num_velocities(joint))::Vector{Float64} for joint in joints(mechanism)])
        # externalWrenches = Dict{RigidBody{Float64}, Wrench{Float64}}()
        # c = inverse_dynamics(cache, v̇, externalWrenches)

        # A = momentum_matrix(cache)
        # v = velocity_vector(x)
        # h = A * v
        # TODO
        # v̇ = Dict([joint::Joint => rand(num_velocities(joint))::Vector{Float64} for joint in joints(mechanism)])
        # externalWrenches = Dict(([body::RigidBody{Float64} => rand(Wrench{Float64}, body.frame)::Wrench{Float64} for body in nonRootBodies]))
        # power = sum(body -> dot(transform(cache, externalWrenches[body], root_frame(mechanism)), twist_wrt_world(cache, body)), nonRootBodies)
        # τ = inverse_dynamics(cache, v̇, externalWrenches)
        # println(power)
        # println(dot(τ, v))
        # Δt = 1e-6
        # v =

        # inverse_dynamics
    end

end
