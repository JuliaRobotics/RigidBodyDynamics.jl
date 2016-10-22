using RigidBodyDynamics
using BenchmarkTools
import RigidBodyDynamics.TreeDataStructure: children, edge_to_parent_data

function create_floating_atlas()
    atlasUrdfUrl = "https://raw.githubusercontent.com/RobotLocomotion/drake/6e3ca768cbaabf15d0f2bed0fb5bd703fa022aa5/drake/examples/Atlas/urdf/atlas_minimal_contact.urdf"
    atlasUrdf = RigidBodyDynamics.cached_download(atlasUrdfUrl, "atlas.urdf")
    atlas = parse_urdf(Float64, atlasUrdf)
    for child in children(root_vertex(atlas))
        joint = edge_to_parent_data(child)
        change_joint_type!(atlas, joint, QuaternionFloating{Float64}())
    end
    atlas
end

function create_benchmark_suite()
    suite = BenchmarkGroup()
    mechanism = create_floating_atlas()
    remove_fixed_joints!(mechanism)

    let
        state = MechanismState(Float64, mechanism)
        result = DynamicsResult(Float64, mechanism)
        suite["mass_matrix"] = @benchmarkable mass_matrix!($(result.massMatrix), $state) setup = rand!($state)
    end

    let
        state = MechanismState(Float64, mechanism)
        result = DynamicsResult(Float64, mechanism)
        torques = Vector{Float64}(num_velocities(mechanism))
        suite["inverse_dynamics"] = @benchmarkable(
            inverse_dynamics!($torques, $(result.jointWrenches), $(result.accelerations), $state, v̇, externalWrenches),
            setup = (
                v̇ = rand(num_velocities($mechanism));
                externalWrenches = Dict(body => rand(Wrench{Float64}, root_frame($mechanism)) for body in non_root_bodies($mechanism));
                rand!($state)
            )
        )
    end

    let
        state = MechanismState(Float64, mechanism)
        result = DynamicsResult(Float64, mechanism)
        suite["dynamics"] = @benchmarkable(dynamics!($result, $state, externalWrenches),
            setup=(
                rand!($state);
                externalWrenches = Dict(body => rand(Wrench{Float64}, root_frame($mechanism)) for body in non_root_bodies($mechanism))
            )
        )
    end

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
