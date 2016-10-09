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
                transform = rand(Transform3D{Float64}, frame, body.frame)
                add_body_fixed_frame!(mechanism2, body, transform)
                additionalFrames[frame] = body
            end
        end

        parentBody = rand(bodies(mechanism))
        attach!(mechanism, parentBody, mechanism2)

        @test num_positions(mechanism) == nq + num_positions(mechanism2)
        @test num_velocities(mechanism) == nv + num_velocities(mechanism2)

        # make sure all of the frame definitions got copied over
        for frame in keys(additionalFrames)
            body = additionalFrames[frame]
            if body == root_body(mechanism2)
                body = parentBody
            end
            @test RigidBodyDynamics.is_fixed_to_body(mechanism, frame, body)
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
        nonFixedJointVertices = filter(v -> !isa(v.edgeToParentData.jointType, Fixed), non_root_vertices(mechanism))

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

            submechanismRoot = rand(bodies(mechanism))
            mechanismPart = submechanism(mechanism, submechanismRoot)
            @test root_body(mechanismPart) == submechanismRoot
            @test mechanism.gravitationalAcceleration.v == mechanismPart.gravitationalAcceleration.v

            substate = MechanismState(Float64, mechanismPart)
            for joint in joints(mechanismPart)
                set_configuration!(substate, joint, configuration(state, joint))
                set_velocity!(substate, joint, velocity(state, joint))
            end
            Msub = mass_matrix(substate)
            if !isleaf(root_vertex(mechanismPart))

                firstJoint = root_vertex(mechanismPart).children[1].edgeToParentData
                offset = first(mechanism.vRanges[firstJoint]) - 1

                vRange = (1 : num_velocities(mechanismPart)) + offset
                @test isapprox(M[vRange, vRange] - Msub, zeros(Msub.data), atol = 1e-10)
            end
        end
    end
end
