using RigidBodyDynamics
using BenchmarkTools

function get_atlas_urdf()
    atlas_urdf_filename = "atlas.urdf"
    atlas_urdf_dir = Base.tempdir() * "/RigidBodyDynamics/urdf/"
    atlas_urdf_local = atlas_urdf_dir * atlas_urdf_filename
    if !isfile(atlas_urdf_local)
        atlas_urdf_remote = "https://raw.githubusercontent.com/RobotLocomotion/drake/6e3ca768cbaabf15d0f2bed0fb5bd703fa022aa5/drake/examples/Atlas/urdf/atlas_minimal_contact.urdf"
        download(atlas_urdf_remote, atlas_urdf_filename)
        if !isdir(atlas_urdf_dir)
            mkpath(atlas_urdf_dir)
        end
        mv(atlas_urdf_filename, atlas_urdf_local)
    end
    atlas_urdf_local
end

function create_floating_atlas()
    atlas = parse_urdf(Float64, get_atlas_urdf())
    for child in root_vertex(atlas).children
        joint = child.edgeToParentData
        change_joint_type!(atlas, joint, QuaternionFloating())
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
    results = run(suite, verbose = true)
    showall(results)
    println()
end

runbenchmarks()
