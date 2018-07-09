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

    double_pendulum = Mechanism(RigidBody{Float64}("world"); gravity = SVector(0, 0, g))
    world = root_body(double_pendulum)

    # IMPORTANT for creating the SpatialInertias below:
    # 1) the second argument, `crosspart` is mass *times* center of mass
    # 2) these SpatialInertias are expressed in frames directly after each of the joints,
    # since the moments of inertia are specified in those frames.
    # If the moment of inertia data were given in the frame after a joint,
    # it would be easier to do the following:
    #
    #   joint1 = Joint("joint1", Revolute(axis))
    #   inertia1 = SpatialInertia(CartesianFrame3D("inertia1_centroidal"), I1_about_com * axis * axis', zero(SVector{3, Float64}), m1)
    #   link1 = RigidBody(inertia1)
    #   before_joint_1_to_world = one(Transform3D, frame_before(joint1), default_frame(world))
    #   c1_to_joint = Transform3D(inertia1.frame, frame_after(joint1), SVector(lc1, 0, 0))
    #   attach!(double_pendulum, world, joint1, link1, before_joint_1_to_world, joint_pose = c1_to_joint)

    # create first body and attach it to the world via a revolute joint
    inertia1 = SpatialInertia(CartesianFrame3D("upper_link"), I1 * axis * axis', m1 * SVector(0, 0, lc1), m1)
    body1 = RigidBody(inertia1)
    joint1 = Joint("shoulder", Revolute(axis))
    joint1_to_world = one(Transform3D, joint1.frame_before, default_frame(world))
    attach!(double_pendulum, world, body1, joint1, joint_pose = joint1_to_world)

    inertia2 = SpatialInertia(CartesianFrame3D("lower_link"), I2 * axis * axis', m2 * SVector(0, 0, lc2), m2)
    body2 = RigidBody(inertia2)
    joint2 = Joint("elbow", Revolute(axis))
    joint2_to_body1 = Transform3D(joint2.frame_before, default_frame(body1), SVector(0, 0, l1))
    attach!(double_pendulum, body1, body2, joint2, joint_pose = joint2_to_body1)

    @test findbody(double_pendulum, string(body1)) == body1
    @test_throws ErrorException findbody(double_pendulum, "bla")
    @test findjoint(double_pendulum, string(joint2)) == joint2
    @test_throws ErrorException findjoint(double_pendulum, "bla")

    x = MechanismState(double_pendulum)
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

    v̇ = similar(velocity(x))
    rand!(v̇)
    τ = inverse_dynamics(x, v̇)
    v = velocity(x)

    @test isapprox(T1, kinetic_energy(x, body1), atol = 1e-12)
    @test isapprox(T2, kinetic_energy(x, body2), atol = 1e-12)
    @test isapprox(M, mass_matrix(x), atol = 1e-12)
    @test isapprox(τ, M * v̇ + C * v + G, atol = 1e-12)

    # compare against URDF
    double_pendulum_urdf = parse_urdf(Float64, joinpath(@__DIR__, "urdf", "Acrobot.urdf"))
    remove_fixed_tree_joints!(double_pendulum_urdf)
    x_urdf = MechanismState(double_pendulum_urdf)
    for (i, j) in enumerate(joints(double_pendulum))
        urdf_joints = collect(joints(double_pendulum_urdf))
        index = findfirst(joint -> string(joint) == string(j), urdf_joints)
        j_urdf = urdf_joints[index]
        set_configuration!(x_urdf, j_urdf, configuration(x, j))
        set_velocity!(x_urdf, j_urdf, velocity(x, j))
    end
    v̇ = similar(velocity(x_urdf))
    rand!(v̇)
    τ = inverse_dynamics(x_urdf, v̇)
    urdf_bodies = collect(bodies(double_pendulum_urdf))
    urdf_upper_link = urdf_bodies[findfirst(b -> string(b) == string(body1), urdf_bodies)]
    urdf_lower_link = urdf_bodies[findfirst(b -> string(b) == string(body2), urdf_bodies)]
    @test isapprox(T1, kinetic_energy(x_urdf, urdf_upper_link), atol = 1e-12)
    @test isapprox(T2, kinetic_energy(x_urdf, urdf_lower_link), atol = 1e-12)
    @test isapprox(M, mass_matrix(x_urdf), atol = 1e-12)
    @test isapprox(τ, M * v̇ + C * v + G, atol = 1e-12)
end
