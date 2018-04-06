@testset "URDF parser" begin
    @testset "joint bounds" begin
        acrobot = parse_urdf(Float64, joinpath(@__DIR__, "urdf", "Acrobot.urdf"))
        @test position_bounds(findjoint(acrobot, "shoulder")) == [RigidBodyDynamics.Bounds(-Inf, Inf)]
        @test velocity_bounds(findjoint(acrobot, "shoulder")) == [RigidBodyDynamics.Bounds(-Inf, Inf)]
        @test effort_bounds(findjoint(acrobot, "shoulder")) == [RigidBodyDynamics.Bounds(-Inf, Inf)]
        @test position_bounds(findjoint(acrobot, "elbow")) == [RigidBodyDynamics.Bounds(-Inf, Inf)]
        @test velocity_bounds(findjoint(acrobot, "elbow")) == [RigidBodyDynamics.Bounds(-Inf, Inf)]
        @test effort_bounds(findjoint(acrobot, "elbow")) == [RigidBodyDynamics.Bounds(-Inf, Inf)]

        acrobot_with_limits = parse_urdf(Float64, joinpath(@__DIR__, "urdf", "Acrobot_with_limits.urdf"))
        @test position_bounds(findjoint(acrobot_with_limits, "shoulder")) == [RigidBodyDynamics.Bounds(-6.28, 6.28)]
        @test velocity_bounds(findjoint(acrobot_with_limits, "shoulder")) == [RigidBodyDynamics.Bounds(-10, 10)]
        @test effort_bounds(findjoint(acrobot_with_limits, "shoulder")) == [RigidBodyDynamics.Bounds(0, 0)]
        @test position_bounds(findjoint(acrobot_with_limits, "elbow")) == [RigidBodyDynamics.Bounds(-6.28, 6.28)]
        @test velocity_bounds(findjoint(acrobot_with_limits, "elbow")) == [RigidBodyDynamics.Bounds(-10, 10)]
        @test effort_bounds(findjoint(acrobot_with_limits, "elbow")) == [RigidBodyDynamics.Bounds(-5, 5)]
    end

    @testset "planar joints" begin
        robot = parse_urdf(Float64, joinpath(@__DIR__, "urdf", "planar_slider.urdf"))
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
end

