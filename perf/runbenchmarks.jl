using RigidBodyDynamics
using BenchmarkTools

const ScalarType = Float64
# const ScalarType = Float32

function create_floating_atlas()
    url = "https://raw.githubusercontent.com/RobotLocomotion/drake/6e3ca768cbaabf15d0f2bed0fb5bd703fa022aa5/drake/examples/Atlas/urdf/atlas_minimal_contact.urdf"
    urdf = RigidBodyDynamics.cached_download(url, "atlas.urdf")
    atlas = parse_urdf(ScalarType, urdf)
    for joint in out_joints(root_body(atlas), atlas)
        floatingjoint = Joint(joint.name, frame_before(joint), frame_after(joint), QuaternionFloating{ScalarType}())
        replace_joint!(atlas, joint, floatingjoint)
    end
    atlas
end

function create_benchmark_suite()
    suite = BenchmarkGroup()
    mechanism = create_floating_atlas()
    remove_fixed_tree_joints!(mechanism)

    state = MechanismState{ScalarType}(mechanism)
    result = DynamicsResult{ScalarType}(mechanism)
    nv = num_velocities(state)
    mat = MomentumMatrix(root_frame(mechanism), Matrix{ScalarType}(3, nv), Matrix{ScalarType}(3, nv))
    torques = Vector{ScalarType}(num_velocities(mechanism))
    rfoot = findbody(mechanism, "r_foot")
    lhand = findbody(mechanism, "l_hand")
    p = path(mechanism, rfoot, lhand)
    nvpath = num_velocities(p)
    jac = GeometricJacobian(default_frame(lhand), default_frame(rfoot), root_frame(mechanism), Matrix{ScalarType}(3, nvpath), Matrix{ScalarType}(3, nvpath))

    suite["mass_matrix"] = @benchmarkable(begin
        setdirty!($state)
        mass_matrix!($(result.massmatrix), $state)
    end, setup = rand!($state))

    suite["inverse_dynamics"] = @benchmarkable(begin
        setdirty!($state)
        inverse_dynamics!($torques, $(result.jointwrenches), $(result.accelerations), $state, v̇, externalwrenches)
    end, setup = begin
        v̇ = rand(num_velocities($mechanism))
        externalwrenches = RigidBodyDynamics.BodyDict{ScalarType}(body => rand(Wrench{ScalarType}, root_frame($mechanism)) for body in bodies($mechanism))
        rand!($state)
    end)

    suite["dynamics"] = @benchmarkable(begin
        setdirty!($state)
        dynamics!($result, $state, τ, externalwrenches)
    end, setup = begin
        rand!($state)
        τ = rand(num_velocities($mechanism))
        externalwrenches = RigidBodyDynamics.BodyDict{ScalarType}(body => rand(Wrench{ScalarType}, root_frame($mechanism)) for body in bodies($mechanism))
    end)

    suite["momentum_matrix"] = @benchmarkable(begin
        setdirty!($state)
        momentum_matrix!($mat, $state)
    end, setup = rand!($state))

    suite["geometric_jacobian"] = @benchmarkable(begin
        setdirty!($state)
        geometric_jacobian!($jac, $state, $p)
    end, setup = rand!($state))

    suite["momentum"] = @benchmarkable(begin
        setdirty!($state)
        momentum($state)
    end, setup = rand!($state))

    suite["momentum_rate_bias"] = @benchmarkable(begin
        setdirty!($state)
        momentum_rate_bias($state)
    end, setup = rand!($state))

    suite["kinetic_energy"] = @benchmarkable(begin
        setdirty!($state)
        kinetic_energy($state)
    end, setup = rand!($state))

    suite["gravitational_potential_energy"] = @benchmarkable(begin
        setdirty!($state)
        gravitational_potential_energy($state)
    end, setup = rand!($state))

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
