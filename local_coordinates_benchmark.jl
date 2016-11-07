using RigidBodyDynamics
using BenchmarkTools
using ForwardDiff


const joint = Joint("bla", QuaternionFloating{Float64}());

const q̇ = Vector{Float64}(num_positions(joint))
const q_autodiff = Vector{ForwardDiff.Dual{1,Float64}}(num_positions(joint))
const q0_autodiff = Vector{ForwardDiff.Dual{1,Float64}}(num_positions(joint))
const ϕ_autodiff = Vector{ForwardDiff.Dual{1,Float64}}(num_velocities(joint))
function local_coordinates2!(jt::QuaternionFloating,
        ϕ::AbstractVector, ϕ̇::AbstractVector,
        q0::AbstractVector, q::AbstractVector, v::AbstractVector)
    RigidBodyDynamics._velocity_to_configuration_derivative!(jt, q̇, q, v)
    for i in eachindex(q)
        @inbounds q_autodiff = ForwardDiff.Dual(q[i], q̇[i])
        @inbounds q0_autodiff = ForwardDiff.Dual(q0[i], 0.)
    end
    RigidBodyDynamics._local_coordinates2!(jt, ϕ_autodiff, q0_autodiff, q_autodiff)
    for i in eachindex(ϕ_autodiff)
        @inbounds ϕ[i] = ForwardDiff.value(ϕ_autodiff[i])
        @inbounds ϕ̇[i] = ForwardDiff.partials(ϕ_autodiff[i])[1]
    end
end

function create_benchmark_suite()
    suite = BenchmarkGroup()

    suite["without autodiff"] = @benchmarkable(RigidBodyDynamics._local_coordinates!($(joint.jointType), ϕ, ϕ̇, q0, q, v), setup = (
            q = Vector{Float64}(num_positions(joint));
            rand_configuration!(joint, q);
            q0 = Vector{Float64}(num_positions(joint));
            rand_configuration!(joint, q0);
            v = rand(num_velocities(joint));
            ϕ = Vector{Float64}(num_velocities(joint));
            ϕ̇ = Vector{Float64}(num_velocities(joint))
        )
    )

    suite["with autodiff"] = @benchmarkable(local_coordinates2!($(joint.jointType), ϕ, ϕ̇, q0, q, v), setup = (
            q = Vector{Float64}(num_positions(joint));
            rand_configuration!(joint, q);
            q0 = Vector{Float64}(num_positions(joint));
            rand_configuration!(joint, q0);
            v = rand(num_velocities(joint));
            ϕ = Vector{Float64}(num_velocities(joint));
            ϕ̇ = Vector{Float64}(num_velocities(joint))
        )
    )
    suite
end

function runbenchmarks()
    suite = create_benchmark_suite()
    tune!(suite)
    Profile.clear_malloc_data()
    results = run(suite, verbose = true)
    showall(results)
    println()
end

runbenchmarks()
