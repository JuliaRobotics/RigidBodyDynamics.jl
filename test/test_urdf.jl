@testset "URDF parse" begin
    @testset "joint bounds" begin
        acrobot = parse_urdf(joinpath(@__DIR__, "urdf", "Acrobot.urdf"), remove_fixed_tree_joints=false)
        @test position_bounds(findjoint(acrobot, "shoulder")) == [RigidBodyDynamics.Bounds(-Inf, Inf)]
        @test velocity_bounds(findjoint(acrobot, "shoulder")) == [RigidBodyDynamics.Bounds(-Inf, Inf)]
        @test effort_bounds(findjoint(acrobot, "shoulder")) == [RigidBodyDynamics.Bounds(-Inf, Inf)]
        @test position_bounds(findjoint(acrobot, "elbow")) == [RigidBodyDynamics.Bounds(-Inf, Inf)]
        @test velocity_bounds(findjoint(acrobot, "elbow")) == [RigidBodyDynamics.Bounds(-Inf, Inf)]
        @test effort_bounds(findjoint(acrobot, "elbow")) == [RigidBodyDynamics.Bounds(-Inf, Inf)]

        acrobot_with_limits = parse_urdf(joinpath(@__DIR__, "urdf", "Acrobot_with_limits.urdf"), remove_fixed_tree_joints=false)
        @test position_bounds(findjoint(acrobot_with_limits, "shoulder")) == [RigidBodyDynamics.Bounds(-6.28, 6.28)]
        @test velocity_bounds(findjoint(acrobot_with_limits, "shoulder")) == [RigidBodyDynamics.Bounds(-10, 10)]
        @test effort_bounds(findjoint(acrobot_with_limits, "shoulder")) == [RigidBodyDynamics.Bounds(0, 0)]
        @test position_bounds(findjoint(acrobot_with_limits, "elbow")) == [RigidBodyDynamics.Bounds(-6.28, 6.28)]
        @test velocity_bounds(findjoint(acrobot_with_limits, "elbow")) == [RigidBodyDynamics.Bounds(-10, 10)]
        @test effort_bounds(findjoint(acrobot_with_limits, "elbow")) == [RigidBodyDynamics.Bounds(-5, 5)]
    end

    @testset "planar joints" begin
        robot = parse_urdf(joinpath(@__DIR__, "urdf", "planar_slider.urdf"), remove_fixed_tree_joints=false)
        # the first joint is the fixed joint between the world and the base link
        jt = joint_type(joints(robot)[1])
        @test jt isa Fixed

        # the 2nd joint is planar with <axis xyz="1 0 0"/>
        jt = joint_type(joints(robot)[2])
        @test jt isa Planar
        @test jt.x_axis ≈ SVector(0, 0, -1)
        @test jt.y_axis ≈ SVector(0, 1, 0)

        # the 3rd joint is planar with <axis xyz="0 1 0"/>
        jt = joint_type(joints(robot)[3])
        @test jt isa Planar
        @test jt.x_axis ≈ SVector(1, 0, 0)
        @test jt.y_axis ≈ SVector(0, 0, -1)

        # the 4th joint is planar with <axis xyz="0 0 1"/>
        jt = joint_type(joints(robot)[4])
        @test jt isa Planar
        @test jt.x_axis ≈ SVector(1, 0, 0)
        @test jt.y_axis ≈ SVector(0, 1, 0)
    end

    @testset "rotation handling" begin
        # https://github.com/JuliaRobotics/RigidBodyDynamics.jl/issues/429
        xml = """
            <origin rpy="0 0 0"/>
        """
        xpose = LightXML.root(LightXML.parse_string(xml))
        rot, trans = RigidBodyDynamics.parse_pose(Float64, xpose)
        @test rot ≈ SDiagonal(1, 1, 1)
        @test trans ≈ SVector(0, 0, 0)

        xml = """
            <origin rpy="0.3 0 0"/>
        """
        xpose = LightXML.root(LightXML.parse_string(xml))
        rot, trans = RigidBodyDynamics.parse_pose(Float64, xpose)
        @test rot ≈ RotMatrix(RotX(0.3))
        @test trans ≈ SVector(0, 0, 0)

        xml = """
            <origin rpy="0 0.2 0"/>
        """
        xpose = LightXML.root(LightXML.parse_string(xml))
        rot, trans = RigidBodyDynamics.parse_pose(Float64, xpose)
        @test rot ≈ RotMatrix(RotY(0.2))
        @test trans ≈ SVector(0, 0, 0)

        xml = """
            <origin rpy="0 0 0.1"/>
        """
        xpose = LightXML.root(LightXML.parse_string(xml))
        rot, trans = RigidBodyDynamics.parse_pose(Float64, xpose)
        @test rot ≈ RotMatrix(RotZ(0.1))
        @test trans ≈ SVector(0, 0, 0)

        # Comparison against ROS's tf.transformations.quaternion_from_euler
        xml = """
            <origin rpy="1 2 3"/>
        """
        xpose = LightXML.root(LightXML.parse_string(xml))
        rot, trans = RigidBodyDynamics.parse_pose(Float64, xpose)
        @test rot ≈ [0.41198225 -0.83373765 -0.36763046;
                    -0.05872664 -0.42691762  0.90238159;
                    -0.90929743 -0.35017549 -0.2248451] atol=1e-7
        @test rot ≈ RotZ(3) * RotY(2) * RotX(1)
        @test trans ≈ SVector(0, 0, 0)


        # Comparison against ROS's tf.transformations.quaternion_from_euler
        xml = """
            <origin rpy="0.5 0.1 0.2"/>
        """
        xpose = LightXML.root(LightXML.parse_string(xml))
        rot, trans = RigidBodyDynamics.parse_pose(Float64, xpose)
        @test rot ≈ [0.97517033 -0.12744012  0.18111281;
                     0.19767681  0.86959819 -0.45246312;
                    -0.09983342  0.47703041  0.8731983] atol=1e-7
        @test rot ≈ RotZ(0.2) * RotY(0.1) * RotX(0.5)
        @test trans ≈ SVector(0, 0, 0)
    end
end

function test_inverse_dynamics_match(mechanism1::Mechanism, mechanism2::Mechanism; atol::Real=0, rtol::Real=atol>0 ? 0 : √eps)
    state1 = MechanismState(mechanism1)
    v̇_vec = rand(num_velocities(mechanism1))
    rand!(state1)
    v̇1 = copyto!(similar(velocity(state1)), v̇_vec)
    τ1 = inverse_dynamics(state1, v̇1)

    state2 = MechanismState(mechanism2)
    copyto!(state2, Vector(state1))
    v̇2 = copyto!(similar(velocity(state2)), v̇_vec)
    τ2 = inverse_dynamics(state2, v̇2)

    @test τ1 ≈ τ2 rtol=rtol atol=atol
end

function test_kinematic_graph_layout_match(mechanism1::Mechanism, mechanism2::Mechanism)
    for (j1, j2) in zip(joints(mechanism1), joints(mechanism2))
        @test string(j1) == string(j2)
        @test string(predecessor(j1, mechanism1)) == string(predecessor(j2, mechanism2))
        @test string(successor(j1, mechanism1)) == string(successor(j2, mechanism2))
        @test joint_type(j1) == joint_type(j2)
    end
end

function test_urdf_serialize_deserialize(mechanism1::Mechanism; remove_fixed_tree_joints::Bool)
    state1 = MechanismState(mechanism1)

    mktempdir() do dir
        urdf2 = joinpath(dir, "test.urdf")
        write_urdf(urdf2, mechanism1, robot_name="test")
        mechanism2 = parse_urdf(urdf2, remove_fixed_tree_joints=remove_fixed_tree_joints)
        test_inverse_dynamics_match(mechanism1, mechanism2, atol=1e-10)

        for joint1 in tree_joints(mechanism1)
            if remove_fixed_tree_joints && joint_type(joint1) isa Fixed
                continue
            else
                joint2 = findjoint(mechanism2, string(joint1))
                @test position_bounds(joint1) == position_bounds(joint2)
                @test velocity_bounds(joint1) == velocity_bounds(joint2)
                @test effort_bounds(joint1) == effort_bounds(joint2)
            end
        end
    end
end

@testset "URDF write" begin
    @testset "Basics" begin
        Random.seed!(124)
        urdfdir = joinpath(@__DIR__, "urdf")
        for basename in readdir(urdfdir)
            last(splitext(basename)) == ".urdf" || continue
            urdf = joinpath(urdfdir, basename)
            for floating in (true, false)
                for remove_fixed_joints_before in (true, false)
                    mechanism = parse_urdf(urdf, remove_fixed_tree_joints=remove_fixed_joints_before, floating=floating)
                    for remove_fixed_joints_after in (true, false)
                        test_urdf_serialize_deserialize(mechanism, remove_fixed_tree_joints=remove_fixed_joints_after)
                    end
                end
            end
        end
    end

    @testset "include_root" begin
        Random.seed!(1245)
        urdfdir = joinpath(@__DIR__, "urdf")
        for basename in readdir(urdfdir)
            last(splitext(basename)) == ".urdf" || continue
            urdf = joinpath(urdfdir, basename)

            unmodified_mechanism = parse_urdf(urdf, remove_fixed_tree_joints=false)
            floating_mechanism = parse_urdf(urdf, remove_fixed_tree_joints=false, floating=true)

            # Write unmodified mechanism to URDF using `include_root=false`, then parse
            # using the default `Fixed` joint type as the root joint type. Ensure we get
            # the same kinematic tree.
            mktempdir() do dir
                urdf2 = joinpath(dir, "test.urdf")
                write_urdf(urdf2, unmodified_mechanism, include_root=false)
                unmodified_mechanism_back = parse_urdf(urdf2, remove_fixed_tree_joints=false)
                test_kinematic_graph_layout_match(unmodified_mechanism, unmodified_mechanism_back)
                test_inverse_dynamics_match(unmodified_mechanism, unmodified_mechanism_back, atol=1e-10)
            end

            # Write floating-base mechanism to URDF using `include_root=true` (default), then parse
            # using the default `Fixed` joint type as the root joint type:
            mktempdir() do dir
                urdf2 = joinpath(dir, "test.urdf")
                write_urdf(urdf2, floating_mechanism)
                floating_mechanism_back = parse_urdf(urdf2)
                # Note: floating_mechanism_back has an extra fixed joint at the root, compared to floating_mechanism,
                # but is dynamically equivalent.
                test_inverse_dynamics_match(floating_mechanism, floating_mechanism_back, atol=1e-10)
            end

            # Write floating-base mechanism to URDF using `include_root=false`, then parse and compare
            # to unmodified mechanism:
            mktempdir() do dir
                urdf2 = joinpath(dir, "test.urdf")
                write_urdf(urdf2, floating_mechanism, include_root=false)
                unmodified_mechanism_back = parse_urdf(urdf2, remove_fixed_tree_joints=false)
                test_kinematic_graph_layout_match(unmodified_mechanism, unmodified_mechanism_back)
                test_inverse_dynamics_match(unmodified_mechanism, unmodified_mechanism_back, atol=1e-10)
            end
        end
    end
end
