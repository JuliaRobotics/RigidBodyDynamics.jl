@testset "double pendulum" begin
    lc1 = -0.5
    l1 = -1.
    m1 = 1.
    I1 = 0.333 # about joint instead of CoM in URDF
    lc2 = -1.
    l2 = -2.
    m2 = 1.
    I2 = 1.33 # about joint instead of CoM in URDF
    g = -9.81

    axis = SVector(0., 1., 0.)

    doublePendulum = Mechanism(RigidBody{Float64}("world"); gravity = SVector(0, 0, g))
    world = root_body(doublePendulum)

    inertia1 = SpatialInertia(CartesianFrame3D("upper_link"), I1 * axis * axis', SVector(0, 0, lc1), m1)
    body1 = RigidBody(inertia1)
    joint1 = Joint("shoulder", Revolute(axis))
    joint1ToWorld = Transform3D{Float64}(joint1.frameBefore, world.frame)
    attach!(doublePendulum, world, joint1, joint1ToWorld, body1)

    inertia2 = SpatialInertia(CartesianFrame3D("lower_link"), I2 * axis * axis', SVector(0, 0, lc2), m2)
    body2 = RigidBody(inertia2)
    joint2 = Joint("elbow", Revolute(axis))
    joint2ToBody1 = Transform3D(joint2.frameBefore, body1.frame, SVector(0, 0, l1))
    attach!(doublePendulum, body1, joint2, joint2ToBody1, body2)

    x = MechanismState(Float64, doublePendulum)
    rand!(x)

    # from http://underactuated.csail.mit.edu/underactuated.html?chapter=3
    q1 = configuration(x, joint1)[1]
    q2 = configuration(x, joint2)[1]
    v1 = velocity(x, joint1)[1]
    v2 = velocity(x, joint2)[1]
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

    v̇ = rand(num_velocities(x))
    τ = inverse_dynamics(x, v̇)
    v = velocity_vector(x)

    @test isapprox(T1, kinetic_energy(x, body1), atol = 1e-12)
    @test isapprox(T2, kinetic_energy(x, body2), atol = 1e-12)
    @test isapprox(M, mass_matrix(x), atol = 1e-12)
    @test isapprox(τ, M * v̇ + C * v + G, atol = 1e-12)

    # compare against URDF
    doublePendulumUrdf = parse_urdf(Float64, "urdf/Acrobot.urdf")
    remove_fixed_joints!(doublePendulumUrdf)
    x_urdf = MechanismState(Float64, doublePendulumUrdf)
    for (i, j) in enumerate(joints(doublePendulum))
        urdf_joints = joints(doublePendulumUrdf)
        index = findfirst(joint -> joint.name == j.name, urdf_joints)
        j_urdf = urdf_joints[index]
        set_configuration!(x_urdf, j_urdf, configuration(x, j))
        set_velocity!(x_urdf, j_urdf, velocity(x, j))
    end
    v̇ = rand(num_velocities(x_urdf))
    τ = inverse_dynamics(x_urdf, v̇)
    urdfBodies = bodies(doublePendulumUrdf)
    urdfUpperLink = urdfBodies[findfirst(b -> name(b) == name(body1), urdfBodies)]
    urdfLowerLink = urdfBodies[findfirst(b -> name(b) == name(body2), urdfBodies)]
    @test isapprox(T1, kinetic_energy(x_urdf, urdfUpperLink), atol = 1e-12)
    @test isapprox(T2, kinetic_energy(x_urdf, urdfLowerLink), atol = 1e-12)
    @test isapprox(M, mass_matrix(x_urdf), atol = 1e-12)
    @test isapprox(τ, M * v̇ + C * v + G, atol = 1e-12)
end
