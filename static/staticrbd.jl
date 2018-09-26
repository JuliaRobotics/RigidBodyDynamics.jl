import Pkg
Pkg.activate(@__DIR__)

module StaticRBD

using RigidBodyDynamics
using DelimitedFiles
using LinearAlgebra

function parse_args(args::Vector{String})
    @assert length(args) == 3
    urdf = args[1]
    csvfile = args[2]
    num_passes = parse(Int, args[3])
    mechanism = parse_urdf(Float64, urdf)
    n = num_positions(mechanism) + num_velocities(mechanism)
    statedata, header = readdlm(csvfile, ',', Float64, '\n'; header=true)
    statevecs = Vector{Float64}[statedata[i, :] for i in 1 : size(statedata, 1)]
    mechanism, statevecs, num_passes
end

Base.@ccallable function create_mechanism(c_urdf::Cstring, c_floating::Cint)::Any
    urdf = unsafe_string(c_urdf)
    floating = Bool(c_floating)
    mechanism = parse_urdf(Float64, urdf)
    @assert !floating # TODO
    remove_fixed_tree_joints!(mechanism)
    mechanism
end

Base.@ccallable function create_state(mechanism::Any)::Any
    mechanism::Mechanism{Float64}
    MechanismState(mechanism)
end

Base.@ccallable function create_dynamics_result(mechanism::Any)::Any
    mechanism::Mechanism{Float64}
    DynamicsResult(mechanism)
end

Base.@ccallable function inverse_dynamics(τ::Any, jointwrenches::Any, accelerations::Any, state::Any, v̇::Any)::Cvoid
    setdirty!(state)
    inverse_dynamics!(τ, jointwrenches, accelerations, state, v̇)
    nothing
end

Base.@ccallable function mass_matrix(M::Any, state::Any)::Cvoid
    setdirty!(state)
    mass_matrix!(M, state)
    nothing
end

Base.@ccallable function dynamics(result::Any, state::Any, τ::Any)::Cvoid
    setdirty!(state)
    dynamics!(result, state, τ)
    nothing
end

end
