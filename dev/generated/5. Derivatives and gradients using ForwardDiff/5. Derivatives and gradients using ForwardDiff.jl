using Pkg # hide
Pkg.activate("/home/travis/build/JuliaRobotics/RigidBodyDynamics.jl/docs/../examples/5. Derivatives and gradients using ForwardDiff") # hide
Pkg.instantiate() # hide
using RigidBodyDynamics, StaticArrays, ForwardDiff
using Test, Random
Random.seed!(1); # to get repeatable results

srcdir = dirname(pathof(RigidBodyDynamics))
urdf = joinpath(srcdir, "..", "test", "urdf", "Acrobot.urdf")
mechanism = parse_urdf(urdf)

float64state = MechanismState(mechanism)
rand!(float64state)
momentum(float64state)

q = configuration(float64state)
function momentum_vec(v::AbstractVector{T}) where T
    # create a `MechanismState` that can handle the element type of `v` (which will be some `ForwardDiff.Dual`):
    state = MechanismState{T}(mechanism)

    # set the state variables:
    set_configuration!(state, q)
    set_velocity!(state, v)

    # return momentum converted to an `SVector` (as ForwardDiff expects an `AbstractVector`)
    Vector(SVector(momentum(state)))
end

v = velocity(float64state)
@test momentum_vec(v) == SVector(momentum(float64state))

J = ForwardDiff.jacobian(momentum_vec, v)

A = momentum_matrix(float64state)
@test J ≈ Array(A) atol = 1e-12

using BenchmarkTools
@benchmark ForwardDiff.jacobian($momentum_vec, $v)

const statecache = StateCache(mechanism)

float32state = statecache[Float32]
@test float32state === statecache[Float32]

function momentum_vec!(out::AbstractVector, v::AbstractVector{T}) where T
    # retrieve a `MechanismState` that can handle the element type of `v`:
    state = statecache[T]

    # set the state variables:
    set_configuration!(state, q)
    set_velocity!(state, v)

    # compute momentum and store it in `out`
    m = momentum(state)
    copyto!(out, SVector(m))
end

const out = zeros(6) # where we'll be storing our results
momentum_vec!(out, v)
@test out == SVector(momentum(float64state))

const result = DiffResults.JacobianResult(out, v)
const config = ForwardDiff.JacobianConfig(momentum_vec!, out, v)
ForwardDiff.jacobian!(result, momentum_vec!, out, v, config)
J = DiffResults.jacobian(result)
@test J ≈ Array(A) atol = 1e-12

@benchmark ForwardDiff.jacobian!($result, $momentum_vec!, $out, $v, $config)

q = copy(configuration(float64state))
@benchmark begin
    set_configuration!($float64state, $q)
    momentum_matrix!($A, $float64state)
end

dynamicsresult = DynamicsResult(mechanism)
set_configuration!(float64state, q)
set_velocity!(float64state, v)
dynamics!(dynamicsresult, float64state)
v̇ = dynamicsresult.v̇

q̇ = v
q_dual = ForwardDiff.Dual.(q, q̇)

v_dual = ForwardDiff.Dual.(v, v̇)

T = eltype(q_dual)
state = statecache[T]
set_configuration!(state, q_dual)
set_velocity!(state, v_dual)
energy_dual = kinetic_energy(state) + gravitational_potential_energy(state)

energy = ForwardDiff.value(energy_dual)
partials = ForwardDiff.partials(energy_dual)
power = partials[1];

energy

@test power ≈ 0 atol = 1e-14

# This file was generated using Literate.jl, https://github.com/fredrikekre/Literate.jl

