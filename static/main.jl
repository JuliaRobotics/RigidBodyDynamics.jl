import Pkg
Pkg.activate(@__DIR__)

module StaticRBD

using RigidBodyDynamics
using DelimitedFiles

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

function run_inverse_dynamics_benchmark(result::DynamicsResult, state::MechanismState, statevecs::Vector{Vector{Float64}}, num_passes::Integer)
    # FIXME: take torque vector, actually call inverse_dynamics! instead of dynamics_bias!
    for _ in Base.OneTo(num_passes)
        @time for vec in statevecs
            copyto!(state, vec)
            dynamics_bias!(result, state)
        end
    end
end

function run_mass_matrix_benchmark(M::AbstractMatrix, state::MechanismState, statevecs::Vector{Vector{Float64}}, num_passes::Integer)
    for _ in Base.OneTo(num_passes)
        @time for vec in statevecs
            copyto!(state, vec)
            mass_matrix!(M, state)
        end
    end
end

function run_dynamics_benchmark(result::DynamicsResult, state::MechanismState, statevecs::Vector{Vector{Float64}}, num_passes::Integer)
    for _ in Base.OneTo(num_passes)
        @time for vec in statevecs
            copyto!(state, vec)
            dynamics!(result, state)
        end
    end
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
    @show mechanism
    println("creating result")
    DynamicsResult(mechanism)
end

Base.@ccallable function inverse_dynamics(τ::Vector{Float64}, state::Any, result::Any, q::Vector{Float64}, v::Vector{Float64}, v̇::Vector{Float64})::Cvoid
    state::MechanismState{Float64, Float64, Float64} # TODO
    result::DynamicsResult{Float64, Float64}
    @inbounds copyto!(configuration(state), q)
    @inbounds copyto!(velocity(state), v)
    setdirty!(state)
    inverse_dynamics!(τ, result.jointwrenches, result.accelerations, state, v̇)
    nothing
end

Base.@ccallable function inverse_dynamics_benchmark(args::Vector{String})::Cvoid
    mechanism, statevecs, num_passes = parse_args(args)
    state = MechanismState(mechanism)
    result = DynamicsResult(mechanism)
    run_inverse_dynamics_benchmark(result, state, statevecs, num_passes)
    nothing
end

Base.@ccallable function mass_matrix_benchmark(args::Vector{String})::Cvoid
    mechanism, statevecs, num_passes = parse_args(args)
    state = MechanismState(mechanism)
    M = mass_matrix(state)
    run_mass_matrix_benchmark(M, state, statevecs, num_passes)
    nothing
end

Base.@ccallable function dynamics_benchmark(args::Vector{String})::Cvoid
    mechanism, statevecs, num_passes = parse_args(args)
    state = MechanismState(mechanism)
    result = DynamicsResult(mechanism)
    run_dynamics_benchmark(result, state, statevecs, num_passes)
    nothing
end

Base.@ccallable function foo(x::Cint, y::Cint)::Cint
    x + y
end

Base.@ccallable function printstring(c_string::Cstring)::Cvoid
    string = unsafe_string(c_string)
    println(string)
    nothing
end

end
