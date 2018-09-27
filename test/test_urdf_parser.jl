@testset "URDF parser" begin
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

