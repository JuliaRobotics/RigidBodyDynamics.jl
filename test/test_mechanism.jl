mechanism = rand_tree_mechanism(Float64, [QuaternionFloating; [Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...)
x = MechanismState{Float64}(mechanism)
rand!(x)
cache = MechanismStateCache(mechanism, x)

facts("basic stuff") do
    v = vcat([x.v[vertex.edgeToParentData] for vertex in mechanism.toposortedTree[2 : end]]...)
    q = vcat([x.q[vertex.edgeToParentData] for vertex in mechanism.toposortedTree[2 : end]]...)
    @fact v --> velocity_vector(x)
    @fact q --> configuration_vector(x)
end

facts("q̇ <-> v") do
    qdot = velocity_to_configuration_derivative(mechanism, x.q, x.v)
    vCheck = configuration_derivative_to_velocity(mechanism, x.q, qdot)
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
    J = geometric_jacobian(cache, p)
    vpath = vcat([x.v[joint] for joint in p.edgeData]...)
    T = relative_twist(cache, body, base)
    @fact J * vpath --> roughly(T; atol = 1e-12)
end

facts("motion subspace / twist wrt world") do
    for vertex in mechanism.toposortedTree[2 : end]
        body = vertex.vertexData
        joint = vertex.edgeToParentData
        parentBody = vertex.parent.vertexData
        @fact relative_twist(cache, body, parentBody) --> roughly(motion_subspace(cache, joint) * x.v[joint]; atol = 1e-12)
    end
end

facts("composite rigid body inertias") do
    for vertex in mechanism.toposortedTree[2 : end]
        body = vertex.vertexData
        crb = crb_inertia(cache, body)
        subtree = toposort(vertex)
        @fact sum((b::RigidBody) -> spatial_inertia(cache, b), [v.vertexData for v in subtree]) --> roughly(crb; atol = 1e-12)
    end
end

facts("momentum_matrix / summing momenta") do
    A = momentum_matrix(cache)
    for vertex in mechanism.toposortedTree[2 : end]
        body = vertex.vertexData
        joint = vertex.edgeToParentData
        start = cache.velocityVectorStartIndices[joint]
        Ajoint = A.mat[:, start : start + num_velocities(joint) - 1]
        @fact (crb_inertia(cache, body) * motion_subspace(cache, joint)).mat --> roughly(Ajoint; atol = 1e-12)
    end

    v = velocity_vector(x)
    h = A * v
    hSum = sum((b::RigidBody) -> isroot(b) ? zero(Momentum{Float64}, A.frame) : spatial_inertia(cache, b) * twist_wrt_world(cache, b), bodies(mechanism))
    @fact h --> roughly(hSum; atol = 1e-12)
end

facts("mass matrix / kinetic energy") do
    Ek = kinetic_energy(cache)
    M = mass_matrix(cache)
    v = velocity_vector(x)
    @fact 1/2 * dot(v, M * v) --> roughly(Ek; atol = 1e-12)

    q = configuration_vector(x)
    kinetic_energy_fun = v -> begin
        local x = MechanismState{eltype(v)}(mechanism)
        set_configuration!(x, q)
        set_velocity!(x, v)
        local cache = MechanismStateCache(mechanism, x)
        return kinetic_energy(cache)
    end
    M2 = ForwardDiff.hessian(kinetic_energy_fun, velocity_vector(x))
    @fact M2 --> roughly(M; atol = 1e-12)
end

facts("inverse dynamics / acceleration term") do
    v̇_to_τ = v̇ -> begin
        v̇Dict = Dict{Joint, Vector{eltype(v̇)}}()
        for joint in keys(x.v)
            v̇Dict[joint] = v̇[cache.velocityVectorStartIndices[joint] : cache.velocityVectorStartIndices[joint] + num_velocities(joint) - 1]
        end
        return inverse_dynamics(cache, v̇Dict)
    end
    M = mass_matrix(cache)
    @fact ForwardDiff.jacobian(v̇_to_τ, zeros(Float64, num_velocities(mechanism))) --> roughly(M; atol = 1e-12)
end

facts("inverse dynamics / Coriolis term") do
    mechanism = rand_tree_mechanism(Float64, [[Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...) # skew symmetry property tested later on doesn't hold when q̇ ≠ v
    x = MechanismState{Float64}(mechanism)
    rand!(x)
    q_to_M = q -> begin
        local x = MechanismState{eltype(q)}(mechanism)
        set_configuration!(x, q)
        zero_velocity!(x)
        local cache = MechanismStateCache(mechanism, x)
        return vec(mass_matrix(cache))
    end
    dMdq = ForwardDiff.jacobian(q_to_M, configuration_vector(x))
    q̇ = velocity_vector(x)
    Ṁ = reshape(dMdq * q̇, num_velocities(mechanism), num_velocities(mechanism))

    q = configuration_vector(x)
    v̇ = Dict([j::Joint => zeros(num_velocities(j))::Vector{Float64} for j in joints(mechanism)])
    v_to_c = v -> begin
        local x = MechanismState{eltype(v)}(mechanism)
        set_configuration!(x, q)
        set_velocity!(x, v)
        local cache = MechanismStateCache(mechanism, x)
        return inverse_dynamics(cache, v̇)
    end
    C = 1/2 * ForwardDiff.jacobian(v_to_c, q̇)

    skew = Ṁ - 2 * C;
    @fact skew + skew' --> roughly(zeros(size(skew)); atol = 1e-12)
end

facts("inverse dynamics / gravity term") do
    mechanism = rand_tree_mechanism(Float64, [[Revolute{Float64} for i = 1 : 10]; [Prismatic{Float64} for i = 1 : 10]]...) # skew symmetry property tested later on doesn't hold when q̇ ≠ v
    x = MechanismState{Float64}(mechanism)
    rand!(x)
    v̇ = Dict([j::Joint => zeros(num_velocities(j))::Vector{Float64} for j in joints(mechanism)])
    zero_velocity!(x)
    cache = MechanismStateCache(mechanism, x)
    g = inverse_dynamics(cache, v̇)
    q_to_potential = q -> begin
        local x = MechanismState{eltype(q)}(mechanism)
        set_configuration!(x, q)
        zero_velocity!(x)
        local cache = MechanismStateCache(mechanism, x)
        return [potential_energy(cache)]
    end
    @fact -ForwardDiff.jacobian(q_to_potential, configuration_vector(x)) --> roughly(g'; atol = 1e-12)
end


# TODO: test external wrenches
# println(dM)
# Ṁ =

# nonRootBodies = filter(b -> !isroot(b), bodies(mechanism))
# v̇ = Dict([joint::Joint => zeros(num_velocities(joint))::Vector{Float64} for joint in joints(mechanism)])
# externalWrenches = Dict{RigidBody{Float64}, Wrench{Float64}}()
# c = inverse_dynamics(cache, v̇, externalWrenches)

# A = momentum_matrix(cache)
# v = velocity_vector(x)
# h = A * v
# TODO
# v̇ = Dict([joint::Joint => rand(num_velocities(joint))::Vector{Float64} for joint in joints(mechanism)])
# externalWrenches = Dict(([body::RigidBody{Float64} => rand(Wrench{Float64}, body.frame)::Wrench{Float64} for body in nonRootBodies]))
# power = sum(body -> dot(transform(cache, externalWrenches[body], root_frame(mechanism)), twist_wrt_world(cache, body)), nonRootBodies)
# τ = inverse_dynamics(cache, v̇, externalWrenches)
# println(power)
# println(dot(τ, v))
# Δt = 1e-6
# v =
