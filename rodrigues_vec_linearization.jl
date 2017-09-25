using RigidBodyDynamics.PDControl
using Rotations
using StaticArrays
using Base.Test

function linearized_rodrigues_vec(e::RotMatrix)
    x = (e[3, 2] - e[2, 3]) / 2
    y = (e[1, 3] - e[3, 1]) / 2
    z = (e[2, 1] - e[1, 2]) / 2
    RodriguesVec(x, y, z)
end

@testset "Rodrigues vec linearization" begin
    ϵ = 1e-3
    for i = 1 : 100
        e = RotMatrix(AngleAxis(ϵ, randn(), randn(), randn()))
        rv = RodriguesVec(e)
        rv_lin = linearized_rodrigues_vec(e)
        lin_error = AngleAxis(rv \ rv_lin)
        @test rotation_angle(lin_error) ≈ 0 atol = 1e-8
    end
end
