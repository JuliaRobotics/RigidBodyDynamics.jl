function floating_joint_transform_to_configuration!(joint::Joint, q::AbstractVector, joint_transform::Transform3D)
    @framecheck frame_before(joint) joint_transform.to
    @framecheck frame_after(joint) joint_transform.from
    joint_type(joint)::QuaternionFloating
    RigidBodyDynamics.rotation!(q, joint_type(joint), rotation(joint_transform))
    RigidBodyDynamics.translation!(q, joint_type(joint), translation(joint_transform))
end

function floating_joint_twist_to_velocity!(joint::Joint, v::AbstractVector, joint_twist::Twist)
    @framecheck frame_before(joint) joint_twist.base
    @framecheck frame_after(joint) joint_twist.body
    @framecheck frame_after(joint) joint_twist.frame
    joint_type(joint)::QuaternionFloating
    RigidBodyDynamics.angular_velocity!(v, joint_type(joint), angular(joint_twist))
    RigidBodyDynamics.linear_velocity!(v, joint_type(joint), linear(joint_twist))
end

@testset "mechanism modification" begin
    @testset "attach!" begin
        body0 = RigidBody{Float64}("root")
        mechanism = Mechanism(body0)
        joint1 = Joint("joint1", rand(Revolute{Float64}))
        joint1_pose = rand(Transform3D{Float64}, frame_before(joint1), default_frame(body0))
        body1 = RigidBody(rand(SpatialInertia{Float64}, frame_after(joint1)))
        joint2 = Joint("joint2", QuaternionFloating{Float64}())
        joint2_pose = rand(Transform3D{Float64}, frame_before(joint2), default_frame(body1))
        body2 = RigidBody(rand(SpatialInertia{Float64}, CartesianFrame3D("2")))

        # can't attach if predecessor is not among bodies of mechanism
        @test_throws AssertionError attach!(mechanism, body1, body2, joint2, joint_pose = joint2_pose)

        # attach body1
        attach!(mechanism, body0, body1, joint1, joint_pose = joint1_pose)
        @test length(bodies(mechanism)) == 2
        @test body1 ∈ bodies(mechanism)
        @test length(joints(mechanism)) == 1
        @test joint1 ∈ joints(mechanism)
        @test isapprox(fixed_transform(mechanism, frame_before(joint1), joint1_pose.to), joint1_pose)

        # can't use the same joint twice
        @test_throws AssertionError attach!(mechanism, body0, body1, joint1, joint_pose = joint1_pose)

        # attach body2
        attach!(mechanism, body1, body2, joint2, joint_pose = joint2_pose)
        @test length(bodies(mechanism)) == 3
        @test body2 ∈ bodies(mechanism)
        @test length(joints(mechanism)) == 2
        @test joint2 ∈ joints(mechanism)
        @test isapprox(fixed_transform(mechanism, frame_before(joint2), joint2_pose.to), joint2_pose)
    end

    @testset "attach! mechanism" begin
        mechanism = rand_tree_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; QuaternionSpherical{Float64}; Planar{Float64}; [Prismatic{Float64} for i = 1 : 10]]...)
        nq = num_positions(mechanism)
        nv = num_velocities(mechanism)

        mechanism2 = rand_tree_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 5]; QuaternionSpherical{Float64}; Planar{Float64}; [Prismatic{Float64} for i = 1 : 5]]...)
        additional_frames = Dict{CartesianFrame3D, RigidBody{Float64}}()
        for body in bodies(mechanism2)
            for i = 1 : 5
                frame = CartesianFrame3D("frame_$i")
                tf = rand(Transform3D, frame, default_frame(body))
                add_frame!(body, tf)
                additional_frames[frame] = body
            end
        end

        parent_body = rand(collect(bodies(mechanism)))
        attach!(mechanism, parent_body, mechanism2)

        @test num_positions(mechanism) == nq + num_positions(mechanism2)
        @test num_velocities(mechanism) == nv + num_velocities(mechanism2)

        # make sure all of the frame definitions got copied over
        for frame in keys(additional_frames)
            body = additional_frames[frame]
            if body == root_body(mechanism2)
                body = parent_body
            end
            @test RigidBodyDynamics.is_fixed_to_body(body, frame)
        end

        state = MechanismState(mechanism) # issue 63
        rand!(state)
        M = mass_matrix(state)

        # independent acrobots in the same configuration
        # make sure mass matrix is block diagonal, and that blocks on diagonal are the same
        double_acrobot = parse_urdf(Float64, "urdf/Acrobot.urdf")
        acrobot2 = parse_urdf(Float64, "urdf/Acrobot.urdf")
        xsingle = MechanismState(acrobot2)
        rand!(xsingle)
        qsingle = configuration(xsingle)
        nq_single = length(qsingle)
        parent_body = root_body(double_acrobot)
        attach!(double_acrobot, parent_body, acrobot2)
        x = MechanismState(double_acrobot)
        set_configuration!(x, [qsingle; qsingle])
        H = mass_matrix(x)
        H11 = H[1 : nq_single, 1 : nq_single]
        H12 = H[1 : nq_single, nq_single + 1 : end]
        H21 = H[nq_single + 1 : end, 1 : nq_single]
        H22 = H[nq_single + 1 : end, nq_single + 1 : end]
        @test isapprox(H11, H22)
        @test isapprox(H12, zeros(H12))
        @test isapprox(H21, zeros(H21))
    end

    @testset "remove fixed joints" begin
        joint_types = [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; QuaternionSpherical{Float64}; Planar{Float64}; [Fixed{Float64} for i = 1 : 10]]
        shuffle!(joint_types)
        mechanism = rand_tree_mechanism(Float64, joint_types...)
        state = MechanismState(mechanism)
        rand!(state)
        q = configuration(state)
        M = mass_matrix(state)
        nonfixedjoints = collect(filter(j -> !(joint_type(j) isa Fixed), tree_joints(mechanism)))

        remove_fixed_tree_joints!(mechanism)
        @test tree_joints(mechanism) == nonfixedjoints
        state_no_fixed_joints = MechanismState(mechanism)
        set_configuration!(state_no_fixed_joints, q)
        M_no_fixed_joints = mass_matrix(state_no_fixed_joints)
        @test isapprox(M_no_fixed_joints, M, atol = 1e-12)
    end

    @testset "replace joint" begin
        jointtypes = [QuaternionFloating{Float64}; QuaternionSpherical{Float64}; Planar{Float64}; [Revolute{Float64} for i = 1 : 10]]
        shuffle!(jointtypes)
        mechanism = rand_tree_mechanism(Float64, jointtypes...)

        for m in [mechanism; maximal_coordinates(mechanism)]
            for i = 1 : 10
                oldjoint = rand(joints(mechanism))
                newjoint = Joint("new", frame_before(oldjoint), frame_after(oldjoint), Fixed{Float64}())
                replace_joint!(mechanism, oldjoint, newjoint)
                @test newjoint ∈ joints(mechanism)
                @test oldjoint ∉ joints(mechanism)
            end
        end
    end

    @testset "submechanism" begin
        for testnum = 1 : 100
            # joint_types = [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; QuaternionSpherical{Float64}; Planar{Float64}; [Fixed{Float64} for i = 1 : 10]] # FIXME: use this
            joint_types = [Revolute{Float64} for i = 1 : 4]
            shuffle!(joint_types)
            mechanism = rand_tree_mechanism(Float64, joint_types...)
            state = MechanismState(mechanism)
            rand!(state)
            M = mass_matrix(state)

            submechanism_root = rand(collect(bodies(mechanism)))
            mechanism_part, bodymap, jointmap = submechanism(mechanism, submechanism_root)
            @test root_body(mechanism_part) == bodymap[submechanism_root]
            @test mechanism.gravitational_acceleration.v == mechanism_part.gravitational_acceleration.v

            substate = MechanismState(mechanism_part)
            for (oldjoint, newjoint) in jointmap
                set_configuration!(substate, newjoint, configuration(state, oldjoint))
                set_velocity!(substate, newjoint, velocity(state, oldjoint))
            end
            Msub = mass_matrix(substate)
            if num_velocities(mechanism_part) > 0
                indices = vcat([velocity_range(state, joint) for joint in tree_joints(mechanism) if joint ∈ keys(jointmap)]...)
                @test isapprox(M[indices, indices], Msub, atol = 1e-10)
            end
        end
    end

    @testset "reattach" begin
        for testnum = 1 : 10
            # create random floating mechanism
            joint_types = [[Prismatic{Float64} for i = 1 : 10]; [Revolute{Float64} for i = 1 : 10]; [Fixed{Float64} for i = 1 : 10]]
            shuffle!(joint_types)
            mechanism1 = rand_floating_tree_mechanism(Float64, joint_types...)

            # random state
            x1 = MechanismState(mechanism1)
            rand!(x1)

            # copy and set up mapping from bodies and joints of mechanism1 to those of mechanism2
            bodies1 = collect(bodies(mechanism1))
            joints1 = collect(joints(mechanism1))
            (bodies2, joints2, mechanism2) = deepcopy((bodies1, joints1, mechanism1))
            bodymap = Dict(zip(bodies1, bodies2))
            jointmap = Dict(zip(joints1, joints2))

            # find world, floating joint, floating body of mechanism1, and determine a new floating body
            world = root_body(mechanism1)
            floatingjoint = first(out_joints(world, mechanism1))
            floatingbody = successor(floatingjoint, mechanism1)
            newfloatingbody = rand(collect(non_root_bodies(mechanism1)))

            # reroot mechanism2
            newfloatingjoint = Joint("new_floating", QuaternionFloating{Float64}())
            joint_to_world = eye(Transform3D, frame_before(newfloatingjoint), default_frame(world))
            body_to_joint = eye(Transform3D, default_frame(newfloatingbody), frame_after(newfloatingjoint))
            attach!(mechanism2, bodymap[world], bodymap[newfloatingbody], newfloatingjoint, joint_pose = joint_to_world, successor_pose = body_to_joint)
            flipped_joint_map = Dict()
            remove_joint!(mechanism2, jointmap[floatingjoint]; flipped_joint_map = flipped_joint_map)

            # mimic the same state for the rerooted mechanism
            # copy non-floating joint configurations and velocities
            x2 = MechanismState(mechanism2)
            for (joint1, joint2) in jointmap
                if joint1 != floatingjoint
                    joint2_rerooted = get(flipped_joint_map, joint2, joint2)
                    set_configuration!(x2, joint2_rerooted, configuration(x1, joint1))
                    set_velocity!(x2, joint2_rerooted, velocity(x1, joint1))
                end
            end

            # set configuration and velocity of new floating joint
            newfloatingjoint_transform = inv(joint_to_world) * relative_transform(x1, body_to_joint.from, joint_to_world.to) * inv(body_to_joint)
            floating_joint_transform_to_configuration!(newfloatingjoint, configuration(x2, newfloatingjoint), newfloatingjoint_transform)

            newfloatingjoint_twist = transform(x1, relative_twist(x1, newfloatingbody, world), body_to_joint.from)
            newfloatingjoint_twist = transform(newfloatingjoint_twist, body_to_joint)
            newfloatingjoint_twist = Twist(body_to_joint.to, joint_to_world.from, newfloatingjoint_twist.frame, angular(newfloatingjoint_twist), linear(newfloatingjoint_twist))
            floating_joint_twist_to_velocity!(newfloatingjoint, velocity(x2, newfloatingjoint), newfloatingjoint_twist)

            # do dynamics and compute spatial accelerations
            result1 = DynamicsResult(mechanism1)
            dynamics!(result1, x1)
            spatial_accelerations!(result1, x1)
            result2 = DynamicsResult(mechanism2)
            dynamics!(result2, x2)
            spatial_accelerations!(result2, x2)

            # make sure that joint accelerations for non-floating joints are the same
            for (joint1, joint2) in jointmap
                if joint1 != floatingjoint
                    joint2_rerooted = get(flipped_joint_map, joint2, joint2)
                    v̇1 = view(result1.v̇, velocity_range(x1, joint1))
                    v̇2 = view(result2.v̇, velocity_range(x2, joint2_rerooted))
                    @test isapprox(v̇1, v̇2)
                end
            end

            # make sure that body spatial accelerations are the same
            for (body1, body2) in bodymap
                accel1 = relative_acceleration(result1, body1, world)
                accel2 = relative_acceleration(result2, body2, bodymap[world])
                @test isapprox(angular.((accel1, accel2))...)
                @test isapprox(linear.((accel1, accel2))...)
            end
        end # for
    end # reattach

    @testset "maximal coordinates" begin
        # create random tree mechanism and equivalent mechanism in maximal coordinates
        tree_mechanism = rand_tree_mechanism(Float64, [QuaternionFloating{Float64}; QuaternionSpherical{Float64}; Planar{Float64}; [Revolute{Float64} for i = 1 : 10]; [Fixed{Float64} for i = 1 : 5]; [Prismatic{Float64} for i = 1 : 10]]...);
        mc_mechanism, newfloatingjoints, bodymap, jointmap = maximal_coordinates(tree_mechanism)

        # randomize state of tree mechanism
        tree_state = MechanismState(tree_mechanism)
        rand!(tree_state)

        # put maximal coordinate system in state that is equivalent to tree mechanism state
        mc_state = MechanismState(mc_mechanism)
        for oldbody in non_root_bodies(tree_mechanism)
            newbody = bodymap[oldbody]
            joint = newfloatingjoints[newbody]

            tf = relative_transform(tree_state, frame_after(joint), frame_before(joint))
            floating_joint_transform_to_configuration!(joint, configuration(mc_state, joint), tf)

            twist = transform(relative_twist(tree_state, frame_after(joint), frame_before(joint)), inv(tf))
            floating_joint_twist_to_velocity!(joint, velocity(mc_state, joint), twist)
        end
        setdirty!(mc_state)

        # do dynamics and compute spatial accelerations
        tree_dynamics_result = DynamicsResult(tree_mechanism);
        dynamics!(tree_dynamics_result, tree_state)
        spatial_accelerations!(tree_dynamics_result, tree_state)

        mc_dynamics_result = DynamicsResult(mc_mechanism);
        dynamics!(mc_dynamics_result, mc_state)
        spatial_accelerations!(mc_dynamics_result, mc_state)

        # compare spatial accelerations of bodies
        for (treebody, mcbody) in bodymap
            tree_accel = relative_acceleration(tree_dynamics_result, treebody, root_body(tree_mechanism))
            mc_accel = relative_acceleration(mc_dynamics_result, mcbody, root_body(mc_mechanism))
            @test isapprox(tree_accel, mc_accel; atol = 1e-12)
        end
    end # maximal coordinates

    @testset "generic scalar dynamics" begin # TODO: move to a better place
        mechanism = rand_tree_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; QuaternionSpherical{Float64}; Planar{Float64}; [Fixed{Float64} for i = 1 : 5]; [Prismatic{Float64} for i = 1 : 10]]...);
        mechanism, newfloatingjoints, bodymap, jointmap = maximal_coordinates(mechanism)

        state_float64 = MechanismState(mechanism)
        rand!(state_float64)
        NullDual = typeof(ForwardDiff.Dual(0., ()))
        state_dual = MechanismState{NullDual}(mechanism)
        configuration(state_dual)[:] = configuration(state_float64)
        velocity(state_dual)[:] = velocity(state_float64)

        dynamics_result_float64 = DynamicsResult(mechanism)
        dynamics!(dynamics_result_float64, state_float64)

        dynamics_result_dual = DynamicsResult{NullDual}(mechanism)
        dynamics!(dynamics_result_dual, state_dual)

        @test isapprox(dynamics_result_float64.v̇, dynamics_result_dual.v̇; atol = 1e-3)
        @test isapprox(dynamics_result_float64.λ, dynamics_result_dual.λ; atol = 1e-3)
    end
end # mechanism modification
