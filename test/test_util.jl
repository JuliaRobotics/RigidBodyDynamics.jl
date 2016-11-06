import RigidBodyDynamics: angle_axis_proper, hat, rotation_matrix

@testset "util" begin
    @testset "rotation" begin
        quat = nquatrand()
        R = rotation_matrix(quat)
        angle, axis = angle_axis_proper(quat)
        rotationvector = angle * axis
        @test isapprox(R, expm(hat(rotationvector)); atol = 1e-10)
    end
end
