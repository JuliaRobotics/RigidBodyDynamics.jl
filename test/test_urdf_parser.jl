@testset "URDF parser" begin
    @testset "joint limits" begin
        acrobot = parse_urdf(Float64, joinpath(@__DIR__, "urdf", "Acrobot.urdf"))
        @test findjoint(acrobot, "shoulder").limits.position.first == [-Inf]
        @test findjoint(acrobot, "shoulder").limits.position.second == [Inf]
        @test findjoint(acrobot, "shoulder").limits.velocity.first == [-Inf]
        @test findjoint(acrobot, "shoulder").limits.velocity.second == [Inf]
        @test findjoint(acrobot, "shoulder").limits.effort.first == [-Inf]
        @test findjoint(acrobot, "shoulder").limits.effort.second == [Inf]
        @test findjoint(acrobot, "elbow").limits.position.first == [-Inf]
        @test findjoint(acrobot, "elbow").limits.position.second == [Inf]
        @test findjoint(acrobot, "elbow").limits.velocity.first == [-Inf]
        @test findjoint(acrobot, "elbow").limits.velocity.second == [Inf]
        @test findjoint(acrobot, "elbow").limits.effort.first == [-Inf]
        @test findjoint(acrobot, "elbow").limits.effort.second == [Inf]

        acrobot_with_limits = parse_urdf(Float64, joinpath(@__DIR__, "urdf", "Acrobot_with_limits.urdf"))
        @test findjoint(acrobot_with_limits, "shoulder").limits.position.first == [-6.28]
        @test findjoint(acrobot_with_limits, "shoulder").limits.position.second == [6.28]
        @test findjoint(acrobot_with_limits, "shoulder").limits.velocity.first == [-10]
        @test findjoint(acrobot_with_limits, "shoulder").limits.velocity.second == [10]
        @test findjoint(acrobot_with_limits, "shoulder").limits.effort.first == [0]
        @test findjoint(acrobot_with_limits, "shoulder").limits.effort.second == [0]
        @test findjoint(acrobot_with_limits, "elbow").limits.position.first == [-6.28]
        @test findjoint(acrobot_with_limits, "elbow").limits.position.second == [6.28]
        @test findjoint(acrobot_with_limits, "elbow").limits.velocity.first == [-10]
        @test findjoint(acrobot_with_limits, "elbow").limits.velocity.second == [10]
        @test findjoint(acrobot_with_limits, "elbow").limits.effort.first == [-5]
        @test findjoint(acrobot_with_limits, "elbow").limits.effort.second == [5]
    end
end

