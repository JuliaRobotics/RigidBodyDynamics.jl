function test_double_pendulum()
    lc1 = rand()
    l1 = rand()
    m1 = rand()
    I1 = rand()
    lc2 = rand()
    l2 = rand()
    m2 = rand()
    I2 = rand()
    g = 9.81

    axis = Vec(0, 1, 0)

    double_pendulum = Mechanism{Float64}("world"; gravity = Vec(0, 0, -g))
    world = root_body(double_pendulum)

    inertia1 = SpatialInertia(CartesianFrame3D("body1"), I1 * (axis * axis'), Vec(0, 0, lc1), m1)
    body1 = RigidBody(inertia1)
    joint1 = Joint("1", Revolute(axis))
    joint1ToWorld = Transform3D{Float64}(joint1.frameBefore, world.frame)
    attach!(double_pendulum, world, joint1, joint1ToWorld, body1)

    inertia2 = SpatialInertia(CartesianFrame3D("body2"), I2 * (axis * axis'), Vec(0, 0, lc2), m2)
    body2 = RigidBody(inertia2)
    joint2 = Joint("2", Revolute(axis))
    joint2ToBody1 = Transform3D(joint2.frameBefore, body1.frame, Vec(0, 0, l1))
    attach!(double_pendulum, body1, joint2, joint2ToBody1, body2)

    x = MechanismState{Float64}(double_pendulum)
    rand!(x)
    cache = MechanismStateCache(double_pendulum, x)

    # from http://underactuated.csail.mit.edu/underactuated.html?chapter=3
    q1 = x.q[joint1][1]
    q2 = x.q[joint2][1]
    v1 = x.v[joint1][1]
    v2 = x.v[joint2][1]
    c1 = cos(q1)
    c2 = cos(q2)
    s1 = sin(q1)
    s2 = sin(q2)
    s12 = sin(q1 + q2)
    T1 = 1/2 * I1 * v1^2
    T2 = 1/2 * (m2 * l1^2 + I2 + 2 * m2 * l1 * lc2 * c2) * v1^2 + 1/2 * I2 * v2^2 + (I2 + m2 * l1 * lc2 * c2) * v1 * v2

    M11 = I1 + I2 + m2 * l1^2 + 2 * m2 * l1 * lc2 * c2
    M12 = I2 + m2 * l1 * lc2 * c2
    M22 = I2
    M = [M11 M12; M12 M22]

    C11 = -2 * m2 * l1 * lc2 * s2 * v2
    C12 = -m2 * l1 * lc2 * s2 * v2
    C21 = m2 * l1 * lc2 * s2 * v1
    C22 = 0
    C = [C11 C12; C21 C22]

    G = [m1 * g * lc1 * s1 + m2 * g * (l1 * s1 + lc2 * s12); m2 * g * lc2 * s12]

    v̇ = Dict([joint::Joint => rand(Float64, num_velocities(joint))::Vector{Float64} for joint in joints(double_pendulum)])
    τ = inverse_dynamics(cache, v̇)
    v̇ = vcat([v̇[joint] for joint in (joint1, joint2)]...)
    v = velocity_vector(x)

    @test isapprox(T1, kinetic_energy(cache, body1); atol = 1e-12)
    @test isapprox(T2, kinetic_energy(cache, body2); atol = 1e-12)
    @test isapprox(M, mass_matrix(cache); atol = 1e-12)
    @test isapprox(M * v̇ + C * v + G, τ; atol = 1e-12)
end
