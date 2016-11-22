import RigidBodyDynamics: angle_axis_proper, hat, rotation_matrix, rotation_vector, rotation_vector_rate
import RigidBodyDynamics: angle_axis_to_rotation_matrix, rotation_vector_to_rotation_matrix

@testset "util" begin
    @testset "conversions" begin
        for quat in (nquatrand(), Quaternion(1., 0., 0., 0.))
            R = rotation_matrix(quat)
            angle, axis = angle_axis_proper(quat)
            ϕ = rotation_vector(quat)
            @test isapprox(R, SMatrix{3, 3}(expm(Array(hat(ϕ)))); atol = 1e-10)
            @test isapprox(R, angle_axis_to_rotation_matrix(angle, axis))
            @test isapprox(R, rotation_vector_to_rotation_matrix(ϕ))
        end
    end

    @testset "rotation vector rate" begin
        for ϕ in (rand(SVector{3}), zeros(SVector{3})) # exponential coordinates (rotation vector)
            ω = rand(SVector{3}) # angular velocity in body frame
            R = SMatrix{3, 3}(expm(Array(hat(ϕ)))) # rotation matrix TODO: use dedicated function
            Ṙ = R * hat(ω)
            ϕ̇ = rotation_vector_rate(ϕ, ω)
            Θ = norm(ϕ)
            if Θ > eps(Θ)
                ϕ_autodiff = SVector{3}(create_autodiff(ϕ, ϕ̇))
                R_autodiff = rotation_vector_to_rotation_matrix(ϕ_autodiff)
                Ṙ_from_autodiff = map(x -> ForwardDiff.partials(x)[1], R_autodiff)
                @test isapprox(Ṙ_from_autodiff, Ṙ)
            else
                @test isapprox(ϕ̇, ω) # limit case; hard to test using autodiff because of division by zero
            end
        end
    end
end
