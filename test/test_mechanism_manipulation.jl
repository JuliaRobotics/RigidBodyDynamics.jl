@testset "mechanism manipulation" begin
    @testset "attach mechanism" begin
        mechanism = rand_tree_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...)
        nq = num_positions(mechanism)
        nv = num_velocities(mechanism)

        mechanism2 = rand_tree_mechanism(Float64, [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 5]; [Prismatic{Float64} for i = 1 : 5]]...)
        additionalFrames = Dict{CartesianFrame3D, RigidBody{Float64}}()
        for body in bodies(mechanism2)
            for i = 1 : 5
                frame = CartesianFrame3D("frame_$i")
                tf = rand(Transform3D{Float64}, frame, default_frame(body))
                add_frame!(body, tf)
                additionalFrames[frame] = body
            end
        end

        parentBody = rand(collect(bodies(mechanism)))
        attach!(mechanism, parentBody, mechanism2)

        @test num_positions(mechanism) == nq + num_positions(mechanism2)
        @test num_velocities(mechanism) == nv + num_velocities(mechanism2)

        # make sure all of the frame definitions got copied over
        for frame in keys(additionalFrames)
            body = additionalFrames[frame]
            if body == root_body(mechanism2)
                body = parentBody
            end
            @test RigidBodyDynamics.is_fixed_to_body(body, frame)
        end

        state = MechanismState(Float64, mechanism) # issue 63
        rand!(state)
        M = mass_matrix(state)

        # independent acrobots in the same configuration
        # make sure mass matrix is block diagonal, and that blocks on diagonal are the same
        doubleAcrobot = parse_urdf(Float64, "urdf/Acrobot.urdf")
        acrobot2 = parse_urdf(Float64, "urdf/Acrobot.urdf")
        xSingle = MechanismState(Float64, acrobot2)
        rand!(xSingle)
        qSingle = configuration_vector(xSingle)
        nqSingle = length(qSingle)
        parentBody = root_body(doubleAcrobot)
        attach!(doubleAcrobot, parentBody, acrobot2)
        x = MechanismState(Float64, doubleAcrobot)
        set_configuration!(x, [qSingle; qSingle])
        H = mass_matrix(x)
        H11 = H[1 : nqSingle, 1 : nqSingle]
        H12 = H[1 : nqSingle, nqSingle + 1 : end]
        H21 = H[nqSingle + 1 : end, 1 : nqSingle]
        H22 = H[nqSingle + 1 : end, nqSingle + 1 : end]
        @test isapprox(H11, H22)
        @test isapprox(H12, zeros(H12))
        @test isapprox(H21, zeros(H21))
    end

    @testset "remove fixed joints" begin
        jointTypes = [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; [Fixed{Float64} for i = 1 : 10]]
        shuffle!(jointTypes)
        mechanism = rand_tree_mechanism(Float64, jointTypes...)
        state = MechanismState(Float64, mechanism)
        rand!(state)
        q = configuration_vector(state)
        M = mass_matrix(state)
        nonFixedJointVertices = collect(filter(v -> !isa(edge_to_parent_data(v).jointType, Fixed), non_root_vertices(mechanism)))

        remove_fixed_joints!(mechanism)
        @test non_root_vertices(mechanism) == nonFixedJointVertices
        state_no_fixed_joints = MechanismState(Float64, mechanism)
        set_configuration!(state_no_fixed_joints, q)
        M_no_fixed_joints = mass_matrix(state)
        @test isapprox(M_no_fixed_joints, M, atol = 1e-12)
    end

    @testset "submechanism" begin
        for testnum = 1 : 10
            jointTypes = [QuaternionFloating{Float64}; [Revolute{Float64} for i = 1 : 10]; [Fixed{Float64} for i = 1 : 10]]
            shuffle!(jointTypes)
            mechanism = rand_tree_mechanism(Float64, jointTypes...)
            state = MechanismState(Float64, mechanism)
            rand!(state)
            M = mass_matrix(state)

            submechanismRoot = rand(collect(bodies(mechanism)))
            mechanismPart = submechanism(mechanism, submechanismRoot)
            @test root_body(mechanismPart) == submechanismRoot
            @test mechanism.gravitationalAcceleration.v == mechanismPart.gravitationalAcceleration.v

            substate = MechanismState(Float64, mechanismPart)
            for joint in tree_joints(mechanismPart)
                set_configuration!(substate, joint, configuration(state, joint))
                set_velocity!(substate, joint, velocity(state, joint))
            end
            Msub = mass_matrix(substate)
            if !isleaf(root_vertex(mechanismPart))
                firstJoint = edge_to_parent_data(children(root_vertex(mechanismPart))[1])
                offset = first(velocity_range(state, firstJoint)) - 1
                vRange = (1 : num_velocities(mechanismPart)) + offset
                @test isapprox(M[vRange, vRange] - Msub, zeros(Msub.data), atol = 1e-10)
            end
        end
    end

    @testset "reattach" begin
        for testnum = 1 : 10
            # create random floating mechanism
            jointTypes = [[Prismatic{Float64} for i = 1 : 10]; [Revolute{Float64} for i = 1 : 10]; [Fixed{Float64} for i = 1 : 10]]
            shuffle!(jointTypes)
            mechanism1 = rand_floating_tree_mechanism(Float64, jointTypes)

            # random state
            x1 = MechanismState(Float64, mechanism1)
            rand!(x1)

            # copy and set up mapping from bodies and joints of mechanism1 to those of mechanism2
            bodies1 = collect(bodies(mechanism1))
            joints1 = collect(tree_joints(mechanism1))
            (bodies2, joints2, mechanism2) = deepcopy((bodies1, joints1, mechanism1))
            bodymap = Dict(zip(bodies1, bodies2))
            jointmap = Dict(zip(joints1, joints2))

            # find world, floating joint, floating body of mechanism1, and determine a new floating body
            world = root_body(mechanism1)
            subtreeRootVertex = children(root_vertex(mechanism1))[1]
            floatingJoint = edge_to_parent_data(subtreeRootVertex)
            floatingBody = vertex_data(subtreeRootVertex)
            newFloatingBody = rand(collect(non_root_bodies(mechanism1)))

            # reroot mechanism2
            newFloatingJoint = Joint("newFloating", QuaternionFloating{Float64}())
            jointToWorld = Transform3D{Float64}(newFloatingJoint.frameBefore, default_frame(world))
            bodyToJoint = Transform3D{Float64}(default_frame(newFloatingBody), newFloatingJoint.frameAfter)
            flippedJointMapping = reattach!(mechanism2, bodymap[floatingBody], bodymap[world], newFloatingJoint, jointToWorld, bodymap[newFloatingBody], bodyToJoint)

            # mimic the same state for the rerooted mechanism
            # copy non-floating joint configurations and velocities
            x2 = MechanismState(Float64, mechanism2)
            for (joint1, joint2) in jointmap
                if joint1 != floatingJoint
                    joint2Rerooted = get(flippedJointMapping, joint2, joint2)
                    set_configuration!(x2, joint2Rerooted, configuration(x1, joint1))
                    set_velocity!(x2, joint2Rerooted, velocity(x1, joint1))
                end
            end

            # set configuration and velocity of new floating joint
            newFloatingJointTransform = inv(jointToWorld) * relative_transform(x1, bodyToJoint.from, jointToWorld.to) * inv(bodyToJoint)
            quat = Quat(newFloatingJointTransform.rot)
            trans = newFloatingJointTransform.trans
            set_configuration!(x2, newFloatingJoint, [quat.w; quat.x; quat.y; quat.z; trans])# TODO: add Joint function that does mapping

            newFloatingJointTwist = transform(x1, relative_twist(x1, newFloatingBody, world), bodyToJoint.from)
            newFloatingJointTwist = transform(newFloatingJointTwist, bodyToJoint)
            set_velocity!(x2, newFloatingJoint, [newFloatingJointTwist.angular; newFloatingJointTwist.linear]) # TODO: add Joint function that does mapping

            # do dynamics
            result1 = DynamicsResult(Float64, mechanism1)
            dynamics!(result1, x1)
            result2 = DynamicsResult(Float64, mechanism2)
            dynamics!(result2, x2)

            # make sure that joint accelerations for non-floating joints are the same
            for (joint1, joint2) in jointmap
                if joint1 != floatingJoint
                    joint2Rerooted = get(flippedJointMapping, joint2, joint2)
                    v̇1 = view(result1.v̇, velocity_range(x1, joint1))
                    v̇2 = view(result2.v̇, velocity_range(x2, joint2Rerooted))
                    @test isapprox(v̇1, v̇2)
                end
            end

            # make sure that body spatial accelerations are the same
            for (body1, body2) in bodymap
                accel1 = relative_acceleration(x1, body1, world, result1.v̇)
                accel2 = relative_acceleration(x2, body2, bodymap[world], result2.v̇)
                @test isapprox(accel1.angular, accel2.angular)
                @test isapprox(accel1.linear, accel2.linear)
            end
        end # for
    end # reattach
end # mechanism manipulation
