@testset "URDF parser" begin
    @testset "joint bounds" begin
        acrobot = parse_urdf(Float64, joinpath(@__DIR__, "urdf", "Acrobot.urdf"))
        @test findjoint(acrobot, "shoulder").bounds.position.lower == [-Inf]
        @test findjoint(acrobot, "shoulder").bounds.position.upper == [Inf]
        @test findjoint(acrobot, "shoulder").bounds.velocity.lower == [-Inf]
        @test findjoint(acrobot, "shoulder").bounds.velocity.upper == [Inf]
        @test findjoint(acrobot, "shoulder").bounds.effort.lower == [-Inf]
        @test findjoint(acrobot, "shoulder").bounds.effort.upper == [Inf]
        @test findjoint(acrobot, "elbow").bounds.position.lower == [-Inf]
        @test findjoint(acrobot, "elbow").bounds.position.upper == [Inf]
        @test findjoint(acrobot, "elbow").bounds.velocity.lower == [-Inf]
        @test findjoint(acrobot, "elbow").bounds.velocity.upper == [Inf]
        @test findjoint(acrobot, "elbow").bounds.effort.lower == [-Inf]
        @test findjoint(acrobot, "elbow").bounds.effort.upper == [Inf]

        acrobot_with_limits = parse_urdf(Float64, joinpath(@__DIR__, "urdf", "Acrobot_with_limits.urdf"))
        @test findjoint(acrobot_with_limits, "shoulder").bounds.position.lower == [-6.28]
        @test findjoint(acrobot_with_limits, "shoulder").bounds.position.upper == [6.28]
        @test findjoint(acrobot_with_limits, "shoulder").bounds.velocity.lower == [-10]
        @test findjoint(acrobot_with_limits, "shoulder").bounds.velocity.upper == [10]
        @test findjoint(acrobot_with_limits, "shoulder").bounds.effort.lower == [0]
        @test findjoint(acrobot_with_limits, "shoulder").bounds.effort.upper == [0]
        @test findjoint(acrobot_with_limits, "elbow").bounds.position.lower == [-6.28]
        @test findjoint(acrobot_with_limits, "elbow").bounds.position.upper == [6.28]
        @test findjoint(acrobot_with_limits, "elbow").bounds.velocity.lower == [-10]
        @test findjoint(acrobot_with_limits, "elbow").bounds.velocity.upper == [10]
        @test findjoint(acrobot_with_limits, "elbow").bounds.effort.lower == [-5]
        @test findjoint(acrobot_with_limits, "elbow").bounds.effort.upper == [5]
    end
end

