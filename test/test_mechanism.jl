function test_mechanism()
    # mechanism = rand_tree_mechanism(Float64, [QuaternionFloating; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...)
    mechanism = rand_chain_mechanism(Float64, Revolute{Float64}, Revolute{Float64})#, Revolute{Float64})
    # mechanism = rand_chain_mechanism(Float64, Prismatic{Float64}, Prismatic{Float64})#, Revolute{Float64})

    x = MechanismState{Float64}(mechanism)
    rand!(x)
    # RigidBodyDynamics.set_velocity!(x, [0.; 1.])
    cache = MechanismStateCache(mechanism, x)

    # basic stuff
    let
        v = vcat([x.v[vertex.edgeToParentData] for vertex in mechanism.toposortedTree[2 : end]]...)
        @test isapprox(v, velocity_vector(x))
        q = vcat([x.q[vertex.edgeToParentData] for vertex in mechanism.toposortedTree[2 : end]]...)
        @test isapprox(q, configuration_vector(x))
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
        println(Ek - 1/2 * dot(v, M * v))
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
        println(M2)
        @test isapprox(M, M2; atol = 1e-12)
    end

    # inverse dynamics
    let
        nonRootBodies = filter(b -> !isroot(b), bodies(mechanism))
        v̇ = Dict([joint::Joint => zeros(num_velocities(joint))::Vector{Float64} for joint in joints(mechanism)])
        externalWrenches = Dict{RigidBody{Float64}, Wrench{Float64}}()
        C = inverse_dynamics(cache, v̇, externalWrenches)

        A = momentum_matrix(cache)
        v = velocity_vector(x)
        h = A * v
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
