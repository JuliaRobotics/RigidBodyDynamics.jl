@testset "simulation" begin
    @testset "simulate" begin
        # use simulate function (Munthe-Kaas integrator)
        acrobot = parse_urdf(Float64, "urdf/Acrobot.urdf")
        x = MechanismState{Float64}(acrobot)
        rand!(x)
        total_energy_before = gravitational_potential_energy(x) + kinetic_energy(x)
        times, qs, vs = simulate(x, 0.1; Δt = 1e-2)
        set_configuration!(x, qs[end])
        set_velocity!(x, vs[end])
        total_energy_after = gravitational_potential_energy(x) + kinetic_energy(x)
        @test isapprox(total_energy_after, total_energy_before, atol = 1e-3)

        total_energy_before = gravitational_potential_energy(x) + kinetic_energy(x)
        result = DynamicsResult{Float64}(acrobot)

        # use RingBufferStorage
        using RigidBodyDynamics.OdeIntegrators
        passive_dynamics! = (vd::AbstractArray, sd::AbstractArray, t, state) -> begin
            dynamics!(result, state)
            copy!(vd, result.v̇)
            copy!(sd, result.ṡ)
            nothing
        end
        storage = RingBufferStorage{Float64}(3)
        integrator = MuntheKaasIntegrator(passive_dynamics!, runge_kutta_4(Float64), storage)
        integrate(integrator, x, 0.1, 1e-2, maxRealtimeRate = 2.0)
        set_configuration!(x, storage.qs[storage.lastIndex])
        set_velocity!(x, storage.vs[storage.lastIndex])
        @test isapprox(gravitational_potential_energy(x) + kinetic_energy(x), total_energy_before, atol = 1e-3)
    end

    @testset "elastic ball drop" begin
        # Drop a single rigid body with a contact point at the center of mass
        # onto the floor with a conservative contact model. Check energy
        # balances and bounce heights.

        world = RigidBody{Float64}("world")
        mechanism = Mechanism(world)
        bodyframe = CartesianFrame3D()
        body = RigidBody("body", rand(SpatialInertia{Float64}, bodyframe))
        floatingjoint = Joint("floating", QuaternionFloating{Float64}())
        attach!(mechanism, world, floatingjoint, eye(Transform3D, frame_before(floatingjoint), root_frame(mechanism)), body)

        com = center_of_mass(spatial_inertia(body))
        model = SoftContactModel(hunt_crossley_hertz(; α = 0.), ViscoelasticCoulombModel(0.5, 1e3, 1e3))
        contactpoint = ContactPoint(com, model)
        add_contact_point!(body, contactpoint)

        point = Point3D(root_frame(mechanism), zeros(SVector{3}))
        normal = FreeVector3D(root_frame(mechanism), 0., 0., 1.)
        halfspace = HalfSpace3D(point, normal)
        add_environment_primitive!(mechanism, halfspace)

        state = MechanismState{Float64}(mechanism)
        z0 = 0.05
        zero!(state)
        RigidBodyDynamics.translation!(floatingjoint.jointType, configuration(state, floatingjoint), SVector(1., 2., z0 - com.v[3]))
        setdirty!(state)

        energy0 = gravitational_potential_energy(state)
        com_world = transform_to_root(state, body) * com

        ts, qs, vs = simulate(state, 0.5; Δt = 1e-3)

        currentsign = 0.
        switches = 0
        for (t, q, v) in zip(ts, qs, vs)
            set_configuration!(state, q)
            set_velocity!(state, v)
            twist = twist_wrt_world(state, body)
            com_world = transform_to_root(state, body) * com
            velocity = point_velocity(twist, com_world)

            z = com_world.v[3]
            penetration = max(-z, zero(z))
            n = model.normal.n
            elastic_potential_energy = model.normal.k * penetration^(n + 1) / (n + 1)
            energy = elastic_potential_energy + kinetic_energy(state) + gravitational_potential_energy(state)
            @test isapprox(energy0, energy; atol = 1e-2)

            newsign = sign(velocity.v[3])
            (newsign != currentsign) && (switches += 1)
            currentsign = newsign
        end
        @test switches > 3
    end

    @testset "inclined plane" begin
        θ = 0.5 # plane angle
        μcrit = tan(θ) # μ > μcrit should show sticking behavior, μ < μcrit should show sliding behavior

        # set up point mass + inclined plane
        world = RigidBody{Float64}("world")
        mechanism = Mechanism(world)
        floatingjoint = Joint("floating", QuaternionFloating{Float64}())
        body = RigidBody("body", SpatialInertia(CartesianFrame3D("inertia"), eye(SMatrix{3, 3}), zeros(SVector{3}), 2.))
        attach!(mechanism, world, floatingjoint, eye(Transform3D, frame_before(floatingjoint), default_frame(world)), body)
        worldframe = root_frame(mechanism)
        inclinedplane = HalfSpace3D(Point3D(worldframe, zeros(SVector{3})), FreeVector3D(worldframe, sin(θ), 0., cos(θ)))
        add_environment_primitive!(mechanism, inclinedplane)
        irrelevantplane = HalfSpace3D(Point3D(worldframe, 0., 0., -100.), FreeVector3D(worldframe, 0., 0., 1.)) # #211
        add_environment_primitive!(mechanism, irrelevantplane)

        # simulate inclined plane friction experiments
        normalmodel = hunt_crossley_hertz(k = 50e3; α = 1.)
        contactlocation = Point3D(default_frame(body), 0., 0., 0.)
        for μ in (μcrit + 1e-2, μcrit - 1e-2)
            frictionmodel = ViscoelasticCoulombModel(μ, 50e3, 1e4)
            m, b = deepcopy((mechanism, body))
            add_contact_point!(b, ContactPoint(contactlocation, SoftContactModel(normalmodel, frictionmodel)))
            state = MechanismState{Float64}(m)
            simulate(state, 1., Δt = 1e-3) # settle into steady state
            x1 = transform(state, contactlocation, worldframe)
            simulate(state, 0.5, Δt = 1e-3)
            x2 = transform(state, contactlocation, worldframe)
            if μ > μcrit # should stick
                @test isapprox(x1, x2, atol = 1e-4)
            else # should slip
                @test !isapprox(x1, x2, atol = 5e-2)
            end
        end
    end
end
