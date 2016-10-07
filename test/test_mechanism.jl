mechanism = rand_tree_mechanism(Float64, [QuaternionFloating; [Revolute{Float64} for i = 1 : 10]; [Fixed for i = 1 : 5]; [Prismatic{Float64} for i = 1 : 10]]...)
x = MechanismState(Float64, mechanism)
rand!(x)

facts("basic stuff") do
    q = vcat([configuration(x, vertex.edgeToParentData) for vertex in non_root_vertices(mechanism)]...)
    v = vcat([velocity(x, vertex.edgeToParentData) for vertex in non_root_vertices(mechanism)]...)

    @fact q --> configuration_vector(x)
    @fact v --> velocity_vector(x)

    zero_configuration!(x)
    set_configuration!(x, q)
    @fact q --> configuration_vector(x)

    zero_velocity!(x)
    set_velocity!(x, v)
    @fact v --> velocity_vector(x)

    qcopy = copy(configuration_vector(x))
    zero_configuration!(x)
    for joint in joints(mechanism)
        set_configuration!(x, joint, qcopy[mechanism.qRanges[joint]])
    end
    @fact q --> configuration_vector(x)

    vcopy = copy(velocity_vector(x))
    zero_velocity!(x)
    for joint in joints(mechanism)
        set_velocity!(x, joint, vcopy[mechanism.vRanges[joint]])
    end
    @fact v --> velocity_vector(x)

    zero!(x)
    set!(x, [q; v])

    @fact q --> configuration_vector(x)
    @fact v --> velocity_vector(x)
end

facts("q̇ <-> v") do
    q = configuration_vector(x)
    q̇ = configuration_derivative(x)
    v = velocity_vector(x)
    for joint in joints(mechanism)
        qjoint = q[mechanism.qRanges[joint]]
        q̇joint = q̇[mechanism.qRanges[joint]]
        @fact velocity(x, joint) --> roughly(configuration_derivative_to_velocity(joint, qjoint, q̇joint); atol = 1e-12)
    end
end

facts("joint_torque! / geometric_jacobian") do
    for joint in joints(mechanism)
        qjoint = configuration(x, joint)
        wrench = rand(Wrench{Float64}, joint.frameAfter)
        τ = Vector{Float64}(num_velocities(joint))
        joint_torque!(joint, τ, qjoint, wrench)
        S = motion_subspace(joint, qjoint)
        @fact τ --> roughly(joint_torque(S, wrench))
    end
end

facts("geometric_jacobian / relative_twist") do
    bs = Set(bodies(mechanism))
    body = rand([bs...])
    delete!(bs, body)
    base = rand([bs...])
    p = path(mechanism, base, body)
    J = geometric_jacobian(x, p)
    vpath = velocity_vector(x, p)
    T = relative_twist(x, body, base)
    @fact Twist(J, vpath) --> roughly(T; atol = 1e-12)
end

facts("relative_acceleration") do
    bs = Set(bodies(mechanism))
    body = rand([bs...])
    delete!(bs, body)
    base = rand([bs...])
    js = joints(mechanism)
    v̇ = rand(num_velocities(mechanism))
    Ṫ = relative_acceleration(x, body, base, v̇)

    q = configuration_vector(x)
    v = velocity_vector(x)
    q̇ = configuration_derivative(x)
    create_autodiff = (z, dz) -> [ForwardDiff.Dual(z[i]::Float64, dz[i]::Float64) for i in 1 : length(z)]
    q_autodiff = create_autodiff(q, q̇)
    v_autodiff = create_autodiff(v, v̇)
    x_autodiff = MechanismState(eltype(q_autodiff), mechanism)
    set_configuration!(x_autodiff, q_autodiff)
    set_velocity!(x_autodiff, v_autodiff)
    twist_autodiff = relative_twist(x_autodiff, body, base)
    accel_vec = [ForwardDiff.partials(x, 1)::Float64 for x in (Array(twist_autodiff))]

    @fact Array(Ṫ) --> roughly(accel_vec; atol = 1e-12)
end

facts("motion subspace / twist wrt world") do
    for vertex in non_root_vertices(mechanism)
        body = vertex.vertexData
        joint = vertex.edgeToParentData
        parentBody = vertex.parent.vertexData
        @fact relative_twist(x, body, parentBody) --> roughly(Twist(motion_subspace(x, joint), velocity(x, joint)); atol = 1e-12)
    end
end

facts("composite rigid body inertias") do
    for vertex in non_root_vertices(mechanism)
        body = vertex.vertexData
        crb = crb_inertia(x, body)
        subtree = toposort(vertex)
        @fact sum((b::RigidBody) -> spatial_inertia(x, b), [v.vertexData for v in subtree]) --> roughly(crb; atol = 1e-12)
    end
end

facts("momentum_matrix / summing momenta") do
    A = momentum_matrix(x)
    Amat = Array(A)
    for vertex in non_root_vertices(mechanism)
        body = vertex.vertexData
        joint = vertex.edgeToParentData
        Ajoint = Amat[:, mechanism.vRanges[joint]]
        @fact Array(crb_inertia(x, body) * motion_subspace(x, joint)) --> roughly(Ajoint; atol = 1e-12)
    end

    v = velocity_vector(x)
    h = Momentum(A, v)
    hSum = sum(b -> spatial_inertia(x, b) * twist_wrt_world(x, b), non_root_bodies(mechanism))
    @fact h --> roughly(hSum; atol = 1e-12)
end


facts("mass matrix / kinetic energy") do
    Ek = kinetic_energy(x)
    M = mass_matrix(x)
    v = velocity_vector(x)
    @fact 1/2 * dot(v, M * v) --> roughly(Ek; atol = 1e-12)

    q = configuration_vector(x)
    kinetic_energy_fun = v -> begin
        local x = MechanismState(eltype(v), mechanism)
        set_configuration!(x, q)
        set_velocity!(x, v)
        kinetic_energy(x)
    end

    M2 = similar(M.data)
    # FIXME: changed after updating to ForwardDiff 0.2: chunk size 1 necessary because creating a MechanismState with a max size Dual takes forever...
    M2 = ForwardDiff.hessian!(M2, kinetic_energy_fun, velocity_vector(x), ForwardDiff.Chunk{1}())
    @fact M2 --> roughly(M; atol = 1e-12)
end

facts("inverse dynamics / acceleration term") do
    M = mass_matrix(x)

    function v̇_to_τ(v̇)
        inverse_dynamics(x, v̇)
    end
    M2 = similar(M.data)
    ForwardDiff.jacobian!(M2, v̇_to_τ, zeros(Float64, num_velocities(mechanism)))
    @fact M2 --> roughly(M; atol = 1e-12)
end

facts("inverse dynamics / Coriolis term") do
    mechanism = rand_tree_mechanism(Float64, [[Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...) # skew symmetry property tested later on doesn't hold when q̇ ≠ v
    x = MechanismState(Float64, mechanism)
    rand!(x)

    function q_to_M(q)
        local x = MechanismState(eltype(q), mechanism)
        set_configuration!(x, q)
        zero_velocity!(x)
        vec(mass_matrix(x))
    end
    nv = num_velocities(mechanism)
    nq = num_positions(mechanism)
    dMdq = zeros(nv * nv, nq)
    ForwardDiff.jacobian!(dMdq, q_to_M, configuration_vector(x))
    q̇ = velocity_vector(x)
    Ṁ = reshape(dMdq * q̇, num_velocities(mechanism), num_velocities(mechanism))

    q = configuration_vector(x)
    v̇ = zeros(num_velocities(mechanism))
    function v_to_c(v)
        local x = MechanismState(eltype(v), mechanism)
        set_configuration!(x, q)
        set_velocity!(x, v)
        inverse_dynamics(x, v̇)
    end
    C = similar(Ṁ)
    ForwardDiff.jacobian!(C, v_to_c, q̇)
    C *= 1/2

    skew = Ṁ - 2 * C;
    @fact skew + skew' --> roughly(zeros(size(skew)); atol = 1e-12)
end

facts("inverse dynamics / gravity term") do
    mechanism = rand_tree_mechanism(Float64, [[Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...)
    x = MechanismState(Float64, mechanism)
    rand!(x)
    v̇ = zeros(num_velocities(mechanism))
    zero_velocity!(x)
    g = inverse_dynamics(x, v̇)

    function q_to_potential(q)
        x = MechanismState(eltype(q), mechanism)
        set_configuration!(x, q)
        zero_velocity!(x)
        return [potential_energy(x)]
    end

    g2 = similar(g')
    ForwardDiff.jacobian!(g2, q_to_potential, configuration_vector(x))
    @fact g2 --> roughly(g'; atol = 1e-12)
end

facts("inverse dynamics / external wrenches") do
    mechanism = rand_chain_mechanism(Float64, [QuaternionFloating; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...) # what really matters is that there's a floating joint first
    x = MechanismState(Float64, mechanism)
    rand_configuration!(x)
    rand_velocity!(x)

    q = configuration_vector(x)
    q̇ = configuration_derivative(x)
    v = velocity_vector(x)
    v̇ = rand(num_velocities(mechanism))
    externalWrenches = Dict(body => rand(Wrench{Float64}, root_frame(mechanism)) for body in non_root_bodies(mechanism))
    τ = inverse_dynamics(x, v̇, externalWrenches)
    floatingBodyVertex = root_vertex(mechanism).children[1]
    floatingJoint = floatingBodyVertex.edgeToParentData
    floatingJointWrench = Wrench(floatingBodyVertex.edgeToParentData.frameAfter, τ[mechanism.vRanges[floatingJoint]])
    floatingJointWrench = transform(x, floatingJointWrench, root_frame(mechanism))

    create_autodiff = (z, dz) -> [ForwardDiff.Dual(z[i]::Float64, dz[i]::Float64) for i in 1 : length(z)]
    q_autodiff = create_autodiff(q, q̇)
    v_autodiff = create_autodiff(v, v̇)
    x_autodiff = MechanismState(eltype(q_autodiff), mechanism)
    set_configuration!(x_autodiff, q_autodiff)
    set_velocity!(x_autodiff, v_autodiff)
    A_autodiff = Array(momentum_matrix(x_autodiff))
    A = [ForwardDiff.value(A_autodiff[i, j])::Float64 for i = 1 : size(A_autodiff, 1), j = 1 : size(A_autodiff, 2)]
    Ȧ = [ForwardDiff.partials(A_autodiff[i, j], 1)::Float64 for i = 1 : size(A_autodiff, 1), j = 1 : size(A_autodiff, 2)]
    ḣ = A * v̇ + Ȧ * v # rate of change of momentum

    gravitational_force = mass(mechanism) * mechanism.gravitationalAcceleration
    com = center_of_mass(x)
    gravitational_wrench = Wrench(gravitational_force.frame, cross(com, gravitational_force).v, gravitational_force.v)
    total_wrench = floatingJointWrench + gravitational_wrench + sum((w) -> transform(x, w, root_frame(mechanism)), values(externalWrenches))
    @fact Array(total_wrench) --> roughly(ḣ; atol = 1e-12)
end

facts("dynamics / inverse dynamics") do
    mechanism = rand_tree_mechanism(Float64, [QuaternionFloating; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...)
    x = MechanismState(Float64, mechanism)
    rand!(x)

    js = joints(mechanism)
    externalWrenches = Dict(body => rand(Wrench{Float64}, root_frame(mechanism)) for body in non_root_bodies(mechanism))
    stateVector = state_vector(x)

    result = DynamicsResult(Float64, mechanism)
    dynamics!(result, x, stateVector, externalWrenches)
    τ = inverse_dynamics(x, result.v̇, externalWrenches)
    @fact τ --> roughly(zeros(num_velocities(mechanism)); atol = 1e-12)
end

facts("attach mechanism") do
    mechanism = rand_tree_mechanism(Float64, [QuaternionFloating; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...)
    nq = num_positions(mechanism)
    nv = num_velocities(mechanism)
    mechanism2 = rand_tree_mechanism(Float64, [QuaternionFloating; [Revolute{Float64} for i = 1 : 5]; [Prismatic{Float64} for i = 1 : 5]]...)
    connection = Joint("connection", rand(Revolute{Float64}))
    parentBody = rand(bodies(mechanism))
    attach!(mechanism, parentBody, connection, rand(Transform3D{Float64}, connection.frameBefore, parentBody.frame), mechanism2)
    @fact num_positions(mechanism) --> nq + num_positions(mechanism2) + num_positions(connection)
    @fact num_velocities(mechanism) --> nv + num_velocities(mechanism2) + num_velocities(connection)
    vertex = findfirst(tree(mechanism), root_body(mechanism2))
    @fact vertex.edgeToParentData --> connection
    @fact vertex.parent.vertexData --> parentBody
    state = MechanismState(Float64, mechanism)

    # independent acrobots in the same configuration
    # make sure mass matrix is block diagonal, and that blocks on diagonal are the same
    doubleAcrobot = parse_urdf(Float64, "urdf/Acrobot.urdf")
    acrobot2 = parse_urdf(Float64, "urdf/Acrobot.urdf")
    xSingle = MechanismState(Float64, acrobot2)
    rand!(xSingle)
    qSingle = configuration_vector(xSingle)
    nqSingle = length(qSingle)
    connection = Joint("fixed", Fixed())
    parentBody = root_body(doubleAcrobot)
    jointToParent = Transform3D(connection.frameBefore, parentBody.frame, rand(SVector{3}))
    childToJoint = Transform3D(root_body(acrobot2).frame, connection.frameAfter, rand(SVector{3}))
    attach!(doubleAcrobot, parentBody, connection, jointToParent, acrobot2, childToJoint)
    x = MechanismState(Float64, doubleAcrobot)
    set_configuration!(x, [qSingle; qSingle])
    H = mass_matrix(x)
    H11 = H[1 : nqSingle, 1 : nqSingle]
    H12 = H[1 : nqSingle, nqSingle + 1 : end]
    H21 = H[nqSingle + 1 : end, 1 : nqSingle]
    H22 = H[nqSingle + 1 : end, nqSingle + 1 : end]
    @fact H11 --> roughly(H22)
    @fact H12 --> roughly(zeros(H12))
    @fact H21 --> roughly(zeros(H21))
end

facts("simulate") do
    acrobot = parse_urdf(Float64, "urdf/Acrobot.urdf")
    x = MechanismState(Float64, acrobot)
    rand!(x)
    total_energy_before = potential_energy(x) + kinetic_energy(x)
    tspan = [0.; 1.]
    times, states = simulate(x, tspan)
    set!(x, states[end])
    total_energy_after = potential_energy(x) + kinetic_energy(x)
    @fact total_energy_after --> roughly(total_energy_before, 1e-3) # fairly loose tolerance here
end

nothing
