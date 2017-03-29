using RigidBodyDynamics
using BenchmarkTools

const ScalarType = Float64
# const ScalarType = Float32

function create_floating_atlas()
    url = "https://raw.githubusercontent.com/RobotLocomotion/drake/6e3ca768cbaabf15d0f2bed0fb5bd703fa022aa5/drake/examples/Atlas/urdf/atlas_minimal_contact.urdf"
    urdf = RigidBodyDynamics.cached_download(url, "atlas.urdf")
    atlas = parse_urdf(ScalarType, urdf)
    for joint in out_joints(root_body(atlas), atlas)
        joint.jointType = QuaternionFloating{ScalarType}()
    end
    atlas
end

function create_benchmark_suite()
    suite = BenchmarkGroup()
    mechanism = create_floating_atlas()
    remove_fixed_tree_joints!(mechanism)

    let
        state = MechanismState(ScalarType, mechanism)
        result = DynamicsResult(ScalarType, mechanism)
        suite["mass_matrix"] = @benchmarkable mass_matrix!($(result.massmatrix), $state) setup = rand!($state)
    end

    let
        state = MechanismState(ScalarType, mechanism)
        result = DynamicsResult(ScalarType, mechanism)
        torques = Vector{ScalarType}(num_velocities(mechanism))
        suite["inverse_dynamics"] = @benchmarkable(
            inverse_dynamics!($torques, $(result.jointwrenches), $(result.accelerations), $state, v̇, externalWrenches),
            setup = (
                v̇ = rand(num_velocities($mechanism));
                externalWrenches = [rand(Wrench{ScalarType}, root_frame($mechanism)) for i = 1 : num_bodies($mechanism)];
                rand!($state)
            )
        )
    end

    let
        state = MechanismState(ScalarType, mechanism)
        result = DynamicsResult(ScalarType, mechanism)
        suite["dynamics"] = @benchmarkable(dynamics!($result, $state, τ, externalWrenches),
            setup=(
                rand!($state);
                τ = rand(num_velocities($mechanism));
                externalWrenches = [rand(Wrench{ScalarType}, root_frame($mechanism)) for i = 1 : num_bodies($mechanism)];
            )
        )
    end

    let
        state = MechanismState(ScalarType, mechanism)
        nv = num_velocities(state)
        angular = Matrix{Float64}(3, nv)
        linear = Matrix{Float64}(3, nv)
        out = MomentumMatrix(root_frame(mechanism), angular, linear)
        suite["momentum_matrix"] = @benchmarkable(momentum_matrix!($out, $state), setup = rand!($state))
    end

    let
        state = MechanismState(ScalarType, mechanism)
        suite["momentum"] = @benchmarkable(momentum($state), setup = rand!($state))
    end

    let
        state = MechanismState(ScalarType, mechanism)
        suite["momentum_rate_bias"] = @benchmarkable(momentum_rate_bias($state), setup = rand!($state))
    end

    let
        state = MechanismState(ScalarType, mechanism)
        suite["kinetic_energy"] = @benchmarkable(kinetic_energy($state), setup = rand!($state))
    end

    let
        state = MechanismState(ScalarType, mechanism)
        suite["gravitational_potential_energy"] = @benchmarkable(gravitational_potential_energy($state), setup = rand!($state))
    end

    suite
end

function runbenchmarks()
    suite = create_benchmark_suite()
    tune!(suite)
    Profile.clear_malloc_data()
    results = run(suite, verbose = true)
    for result in results
        println("$(first(result)):")
        display(last(result))
        println()
    end
end

runbenchmarks()
