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

        base_link = findbody(acrobot_with_limits, "base_link")
        floating_base = joint_to_parent(base_link, acrobot_with_limits)
        set_joint_type!(floating_base, QuaternionFloating{Float64}())
        @test floating_base.position_bounds == fill(RigidBodyDynamics.Bounds{Float64}(), 7)
        @test floating_base.velocity_bounds == fill(RigidBodyDynamics.Bounds{Float64}(), 6)
        @test floating_base.effort_bounds == fill(RigidBodyDynamics.Bounds{Float64}(), 6)
    end
end

