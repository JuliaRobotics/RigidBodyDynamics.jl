@testset "simulation" begin
    @testset "simulate" begin
        Random.seed!(60)
        # use simulate function (Munthe-Kaas integrator)
        acrobot = parse_urdf(joinpath(@__DIR__, "urdf", "Acrobot.urdf"), remove_fixed_tree_joints=false)
        x = MechanismState(acrobot)
        rand!(x)
        total_energy_before = gravitational_potential_energy(x) + kinetic_energy(x)
        times, qs, vs = simulate(x, 0.1; Δt = 1e-2)
        set_configuration!(x, qs[end])
        set_velocity!(x, vs[end])
        total_energy_after = gravitational_potential_energy(x) + kinetic_energy(x)
        @test isapprox(total_energy_after, total_energy_before, atol = 1e-3)

        total_energy_before = gravitational_potential_energy(x) + kinetic_energy(x)
        result = DynamicsResult(acrobot)

        # use RingBufferStorage
        using RigidBodyDynamics.OdeIntegrators
        passive_dynamics! = (vd::AbstractArray, sd::AbstractArray, t, state) -> begin
            dynamics!(result, state)
            copyto!(vd, result.v̇)
            copyto!(sd, result.ṡ)
            nothing
        end
        storage = RingBufferStorage{Float64}(x, 3)
        integrator = MuntheKaasIntegrator(x, passive_dynamics!, runge_kutta_4(Float64), storage)
        integrate(integrator, 0.1, 1e-2, max_realtime_rate = 2.0)
        set_configuration!(x, storage.qs[storage.last_index])
        set_velocity!(x, storage.vs[storage.last_index])
        @test isapprox(gravitational_potential_energy(x) + kinetic_energy(x), total_energy_before, atol = 1e-3)
    end

    @testset "elastic ball drop" begin
        # Drop a single rigid body with a contact point at the center of mass
        # onto the floor with a conservative contact model. Check energy
        # balances and bounce heights.
        Random.seed!(61)
        world = RigidBody{Float64}("world")
        mechanism = Mechanism(world)
        bodyframe = CartesianFrame3D()
        body = RigidBody("body", rand(SpatialInertia{Float64}, bodyframe))
        floatingjoint = Joint("floating", QuaternionFloating{Float64}())
        attach!(mechanism, world, body, floatingjoint)

        com = center_of_mass(spatial_inertia(body))
        model = SoftContactModel(hunt_crossley_hertz(; α = 0.), ViscoelasticCoulombModel(0.5, 1e3, 1e3))
        contactpoint = ContactPoint(com, model)
        add_contact_point!(body, contactpoint)

        point = Point3D(root_frame(mechanism), zero(SVector{3}))
        normal = FreeVector3D(root_frame(mechanism), 0., 0., 1.)
        halfspace = HalfSpace3D(point, normal)
        add_environment_primitive!(mechanism, halfspace)

        state = MechanismState(mechanism)
        z0 = 0.05
        zero!(state)
        tf = transform_to_root(state, body)
        tf = Transform3D(frame_after(floatingjoint), frame_before(floatingjoint), rotation(tf), SVector(1., 2., z0 - com.v[3]))
        set_configuration!(state, floatingjoint, tf)

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
        body = RigidBody("body", SpatialInertia(CartesianFrame3D("inertia"), SMatrix{3, 3}(1.0I), zero(SVector{3}), 2.))
        attach!(mechanism, world, body, floatingjoint)
        worldframe = root_frame(mechanism)
        inclinedplane = HalfSpace3D(Point3D(worldframe, zero(SVector{3})), FreeVector3D(worldframe, sin(θ), 0., cos(θ)))
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
            state = MechanismState(m)
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

    @testset "four-bar linkage" begin
        # gravitational acceleration
        g = -9.81

        # link lengths
        l_0 = 1.10
        l_1 = 0.5
        l_2 = 1.20
        l_3 = 0.75

        # link masses
        m_1 = 0.5
        m_2 = 1.0
        m_3 = 0.75

        # link center of mass offsets from the preceding joint axes
        c_1 = 0.25
        c_2 = 0.60
        c_3 = 0.375

        # moments of inertia about the center of mass of each link
        I_1 = 0.333
        I_2 = 0.537
        I_3 = 0.4

        # scalar type
        T = Float64

        # Rotation axis: negative y-axis
        axis = SVector(zero(T), -one(T), zero(T))

        world = RigidBody{T}("world")
        mechanism = Mechanism(world; gravity = SVector(0., 0., g))
        rootframe = root_frame(mechanism)

        # link1 and joint1
        joint1 = Joint("joint1", Revolute(axis))
        inertia1 = SpatialInertia(CartesianFrame3D("inertia1_centroidal"), moment=I_1*axis*axis', com=zero(SVector{3, T}), mass=m_1)
        link1 = RigidBody(inertia1)
        before_joint1_to_world = one(Transform3D, frame_before(joint1), default_frame(world))
        c1_to_joint = Transform3D(inertia1.frame, frame_after(joint1), SVector(c_1, 0, 0))
        attach!(mechanism, world, link1, joint1, joint_pose = before_joint1_to_world, successor_pose = c1_to_joint)

        # link2 and joint2
        joint2 = Joint("joint2", Revolute(axis))
        inertia2 = SpatialInertia(CartesianFrame3D("inertia2_centroidal"), moment=I_2*axis*axis', com=zero(SVector{3, T}), mass=m_2)
        link2 = RigidBody(inertia2)
        before_joint2_to_after_joint1 = Transform3D(frame_before(joint2), frame_after(joint1), SVector(l_1, 0., 0.))
        c2_to_joint = Transform3D(inertia2.frame, frame_after(joint2), SVector(c_2, 0, 0))
        attach!(mechanism, link1, link2, joint2, joint_pose = before_joint2_to_after_joint1, successor_pose = c2_to_joint)

        # link3 and joint3
        joint3 = Joint("joint3", Revolute(axis))
        inertia3 = SpatialInertia(CartesianFrame3D("inertia3_centroidal"), moment=I_3*axis*axis', com=zero(SVector{3, T}), mass=m_3)
        link3 = RigidBody(inertia3)
        before_joint3_to_world = Transform3D(frame_before(joint3), default_frame(world), SVector(l_0, 0., 0.))
        c3_to_joint = Transform3D(inertia3.frame, frame_after(joint3), SVector(c_3, 0, 0))
        attach!(mechanism, world, link3, joint3, joint_pose = before_joint3_to_world, successor_pose = c3_to_joint)

        # loop joint between link2 and link3
        joint4 = Joint("joint4", Revolute(axis))
        before_joint4_to_joint2 = Transform3D(frame_before(joint4), frame_after(joint2), SVector(l_2, 0., 0.))
        joint3_to_after_joint4 = Transform3D(frame_after(joint3), frame_after(joint4), SVector(-l_3, 0., 0.))
        attach!(mechanism, link2, link3, joint4, joint_pose = before_joint4_to_joint2, successor_pose = joint3_to_after_joint4)

        # initial state
        set_initial_state! = function (state::MechanismState)
            # found through nonlinear optimization. Slightly inaccurate.
            set_configuration!(state, joint1, 1.6707963267948966) # θ
            set_configuration!(state, joint2, -1.4591054166649482) # γ
            set_configuration!(state, joint3, 1.5397303602625536) # ϕ
            set_velocity!(state, joint1, 0.5)
            set_velocity!(state, joint2, -0.47295)
            set_velocity!(state, joint3, 0.341)
        end

        # no stabilization
        state = MechanismState(mechanism)
        set_initial_state!(state)
        zero_velocity!(state)
        energy0 = gravitational_potential_energy(state)
        ts, qs, vs = simulate(state, 1., Δt = 1e-3, stabilization_gains=nothing)
        @test kinetic_energy(state) ≉ 0 atol=1e-2
        energy1 = gravitational_potential_energy(state) + kinetic_energy(state)
        @test energy0 ≈ energy1 atol=1e-8
        @test transform(state, Point3D(Float64, frame_before(joint4)), rootframe) ≈
              transform(state, Point3D(Float64, frame_after(joint4)), rootframe) atol=1e-10 # no significant separation after a short simulation

        # with default stabilization: start with some separation
        set_initial_state!(state)
        set_configuration!(state, joint1, 1.7)
        @test transform(state, Point3D(Float64, frame_before(joint4)), rootframe) ≉
              transform(state, Point3D(Float64, frame_after(joint4)), rootframe) atol=1e-2 # significant separation initially
        simulate(state, 15., Δt = 1e-3)
        @test transform(state, Point3D(Float64, frame_before(joint4)), rootframe) ≈
              transform(state, Point3D(Float64, frame_after(joint4)), rootframe) atol=1e-5 # reduced separation after 15 seconds
        energy15 = gravitational_potential_energy(state) + kinetic_energy(state)
        simulate(state, 10, Δt = 1e-3)
        energy20 = gravitational_potential_energy(state) + kinetic_energy(state)
        @test energy20 ≈ energy15 atol=1e-5 # stabilization doesn't significantly affect energy after converging
    end
end
