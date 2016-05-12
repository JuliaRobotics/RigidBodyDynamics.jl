mechanism = rand_tree_mechanism(Float64, [QuaternionFloating; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...)
x = MechanismState(Float64, mechanism)
rand!(x)

facts("basic stuff") do
    v = vcat([x.v[vertex.edgeToParentData] for vertex in mechanism.toposortedTree[2 : end]]...)
    q = vcat([x.q[vertex.edgeToParentData] for vertex in mechanism.toposortedTree[2 : end]]...)
    @fact v --> velocity_vector(x)
    @fact q --> configuration_vector(x)
end

facts("q̇ <-> v") do
    q̇ = velocity_to_configuration_derivative(x.q, x.v)
    vCheck = configuration_derivative_to_velocity(x.q, q̇)
    for joint in joints(mechanism)
        @fact vCheck[joint] --> roughly(x.v[joint]; atol = 1e-12)
    end
end

facts("geometric_jacobian / relative_twist") do
    bs = Set(bodies(mechanism))
    body = rand([bs...])
    delete!(bs, body)
    base = rand([bs...])
    p = path(mechanism, base, body)
    J = geometric_jacobian(x, p)
    vpath = vcat([x.v[joint] for joint in p.edgeData]...)
    T = relative_twist(x, body, base)
    @fact J * vpath --> roughly(T; atol = 1e-12)
end

facts("motion subspace / twist wrt world") do
    for vertex in mechanism.toposortedTree[2 : end]
        body = vertex.vertexData
        joint = vertex.edgeToParentData
        parentBody = vertex.parent.vertexData
        @fact relative_twist(x, body, parentBody) --> roughly(motion_subspace(x, joint) * x.v[joint]; atol = 1e-12)
    end
end

facts("composite rigid body inertias") do
    for vertex in mechanism.toposortedTree[2 : end]
        body = vertex.vertexData
        crb = crb_inertia(x, body)
        subtree = toposort(vertex)
        @fact sum((b::RigidBody) -> spatial_inertia(x, b), [v.vertexData for v in subtree]) --> roughly(crb; atol = 1e-12)
    end
end

facts("momentum_matrix / summing momenta") do
    A = momentum_matrix(x)
    for vertex in mechanism.toposortedTree[2 : end]
        body = vertex.vertexData
        joint = vertex.edgeToParentData
        start = x.velocityVectorStartIndices[joint]
        Ajoint = A.mat[:, start : start + num_velocities(joint) - 1]
        @fact (crb_inertia(x, body) * motion_subspace(x, joint)).mat --> roughly(Ajoint; atol = 1e-12)
    end

    v = velocity_vector(x)
    h = A * v
    hSum = sum((b::RigidBody) -> isroot(b) ? zero(Momentum{Float64}, A.frame) : spatial_inertia(x, b) * twist_wrt_world(x, b), bodies(mechanism))
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
        return kinetic_energy(x)
    end
    M2 = ForwardDiff.hessian(kinetic_energy_fun, velocity_vector(x))
    @fact M2 --> roughly(M; atol = 1e-12)
end

facts("inverse dynamics / acceleration term") do
    v̇_to_τ = v̇ -> begin
        v̇Dict = Dict{Joint, Vector{eltype(v̇)}}()
        for joint in keys(x.v)
            v̇Dict[joint] = v̇[x.velocityVectorStartIndices[joint] : x.velocityVectorStartIndices[joint] + num_velocities(joint) - 1]
        end
        return inverse_dynamics(x, v̇Dict)
    end
    M = mass_matrix(x)
    @fact ForwardDiff.jacobian(v̇_to_τ, zeros(Float64, num_velocities(mechanism))) --> roughly(M; atol = 1e-12)
end

facts("inverse dynamics / Coriolis term") do
    mechanism = rand_tree_mechanism(Float64, [[Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...) # skew symmetry property tested later on doesn't hold when q̇ ≠ v
    x = MechanismState(Float64, mechanism)
    rand!(x)
    q_to_M = q -> begin
        local x = MechanismState(eltype(q), mechanism)
        set_configuration!(x, q)
        zero_velocity!(x)
        return vec(mass_matrix(x))
    end
    dMdq = ForwardDiff.jacobian(q_to_M, configuration_vector(x))
    q̇ = velocity_vector(x)
    Ṁ = reshape(dMdq * q̇, num_velocities(mechanism), num_velocities(mechanism))

    q = configuration_vector(x)
    v̇ = Dict([j::Joint => zeros(num_velocities(j))::Vector{Float64} for j in joints(mechanism)])
    v_to_c = v -> begin
        local x = MechanismState(eltype(v), mechanism)
        set_configuration!(x, q)
        set_velocity!(x, v)
        return inverse_dynamics(x, v̇)
    end
    C = 1/2 * ForwardDiff.jacobian(v_to_c, q̇)

    skew = Ṁ - 2 * C;
    @fact skew + skew' --> roughly(zeros(size(skew)); atol = 1e-12)
end

facts("inverse dynamics / gravity term") do
    mechanism = rand_tree_mechanism(Float64, [[Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...) # skew symmetry property tested later on doesn't hold when q̇ ≠ v
    x = MechanismState(Float64, mechanism)
    rand!(x)
    v̇ = Dict([j::Joint => zeros(num_velocities(j))::Vector{Float64} for j in joints(mechanism)])
    zero_velocity!(x)
    g = inverse_dynamics(x, v̇)
    q_to_potential = q -> begin
        local x = MechanismState(eltype(q), mechanism)
        set_configuration!(x, q)
        zero_velocity!(x)
        return [potential_energy(x)]
    end
    @fact -ForwardDiff.jacobian(q_to_potential, configuration_vector(x)) --> roughly(g'; atol = 1e-12)
end

facts("inverse dynamics / external wrenches") do
    mechanism = rand_chain_mechanism(Float64, [QuaternionFloating; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...) # what really matters is that there's a floating joint first
    x = MechanismState(Float64, mechanism)
    rand_configuration!(x)
    rand_velocity!(x)

    v̇ = Dict([joint::Joint => rand(Float64, num_velocities(joint))::Vector{Float64} for joint in joints(mechanism)])
    nonRootBodies = filter(b -> !isroot(b), bodies(mechanism))
    externalWrenches = Dict(([body::RigidBody{Float64} => rand(Wrench{Float64}, root_frame(mechanism))::Wrench{Float64} for body in nonRootBodies]))
    τ = inverse_dynamics(x, v̇, externalWrenches)
    floatingBodyVertex = root_vertex(mechanism).children[1]
    floatingJoint = floatingBodyVertex.edgeToParentData
    start = x.velocityVectorStartIndices[floatingJoint]
    range = start : start + num_velocities(floatingJoint) - 1
    floatingJointWrench = Wrench(floatingBodyVertex.edgeToParentData.frameAfter, τ[range])
    floatingJointWrench = transform(x, floatingJointWrench, root_frame(mechanism))

    A = momentum_matrix(x)
    q_to_A = q -> begin
        local x = MechanismState(eltype(q), mechanism)
        set_configuration!(x, q)
        zero_velocity!(x)
        return vec(momentum_matrix(x).mat)
    end
    dAdq = ForwardDiff.jacobian(q_to_A, configuration_vector(x))
    q̇ = vcat([velocity_to_configuration_derivative(x.q, x.v)[joint] for joint in keys(x.v)]...)
    Ȧ = reshape(dAdq * q̇, size(A.mat))
    v̇ = vcat([v̇[joint] for joint in keys(x.v)]...)
    v = velocity_vector(x)
    ḣ = A.mat * v̇ + Ȧ * v # rate of change of momentum

    gravitational_force = FreeVector3D(root_frame(mechanism), mass(mechanism) * mechanism.gravity)
    com = center_of_mass(x)
    gravitational_wrench = Wrench(gravitational_force.frame, cross(com, gravitational_force).v, gravitational_force.v)
    total_wrench = floatingJointWrench - gravitational_wrench + sum((w) -> transform(x, w, root_frame(mechanism)), values(externalWrenches)) # TODO: minus for gravitational wrench?
    @fact to_array(total_wrench) --> roughly(ḣ; atol = 1e-12)
end
