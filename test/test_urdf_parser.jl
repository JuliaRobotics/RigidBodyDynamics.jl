@testset "URDF parser" begin
    @testset "joint bounds" begin
        acrobot = parse_urdf(Float64, joinpath(@__DIR__, "urdf", "Acrobot.urdf"))
        @test findjoint(acrobot, "shoulder").bounds.position == [RigidBodyDynamics.Bounds(-Inf, Inf)]
        @test findjoint(acrobot, "shoulder").bounds.velocity == [RigidBodyDynamics.Bounds(-Inf, Inf)]
        @test findjoint(acrobot, "shoulder").bounds.effort == [RigidBodyDynamics.Bounds(-Inf, Inf)]
        @test findjoint(acrobot, "elbow").bounds.position == [RigidBodyDynamics.Bounds(-Inf, Inf)]
        @test findjoint(acrobot, "elbow").bounds.velocity == [RigidBodyDynamics.Bounds(-Inf, Inf)]
        @test findjoint(acrobot, "elbow").bounds.effort == [RigidBodyDynamics.Bounds(-Inf, Inf)]

        acrobot_with_limits = parse_urdf(Float64, joinpath(@__DIR__, "urdf", "Acrobot_with_limits.urdf"))
        @test findjoint(acrobot_with_limits, "shoulder").bounds.position == [RigidBodyDynamics.Bounds(-6.28, 6.28)]
        @test findjoint(acrobot_with_limits, "shoulder").bounds.velocity == [RigidBodyDynamics.Bounds(-10, 10)]
        @test findjoint(acrobot_with_limits, "shoulder").bounds.effort == [RigidBodyDynamics.Bounds(0, 0)]
        @test findjoint(acrobot_with_limits, "elbow").bounds.position == [RigidBodyDynamics.Bounds(-6.28, 6.28)]
        @test findjoint(acrobot_with_limits, "elbow").bounds.velocity == [RigidBodyDynamics.Bounds(-10, 10)]
        @test findjoint(acrobot_with_limits, "elbow").bounds.effort == [RigidBodyDynamics.Bounds(-5, 5)]
    end
end

