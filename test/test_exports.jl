@testset "exports" begin
    # Ensure that every exported name is actually defined
    for name in names(RigidBodyDynamics)
        @test isdefined(RigidBodyDynamics, name)
    end
end
