@testset "double pendulum" begin
    Random.seed!(6)
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

    # create first body and attach it to the world via a revolute joint
    inertia1 = SpatialInertia(CartesianFrame3D("upper_link"), moment=I1 * axis * axis', com=SVector(0, 0, lc1), mass=m1)
    body1 = RigidBody(inertia1)
    joint1 = Joint("shoulder", Revolute(axis))
    joint1_to_world = one(Transform3D, joint1.frame_before, default_frame(world))
    attach!(double_pendulum, world, body1, joint1, joint_pose = joint1_to_world)

    inertia2 = SpatialInertia(CartesianFrame3D("lower_link"), moment=I2 * axis * axis', com=SVector(0, 0, lc2), mass=m2)
    body2 = RigidBody(inertia2)
    joint2 = Joint("elbow", Revolute(axis))
    joint2_to_body1 = Transform3D(joint2.frame_before, default_frame(body1), SVector(0, 0, l1))
    attach!(double_pendulum, body1, body2, joint2, joint_pose = joint2_to_body1)

    @test findbody(double_pendulum, string(body1)) == body1
    @test_throws ErrorException findbody(double_pendulum, "bla")
    @test findbody(double_pendulum, BodyID(body2)) == body2
    @test findjoint(double_pendulum, string(joint2)) == joint2
    @test_throws ErrorException findjoint(double_pendulum, "bla")
    @test findjoint(double_pendulum, JointID(joint2)) == joint2

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
    for revolute_joint_type in [Revolute, SinCosRevolute]
        double_pendulum_urdf = parse_urdf(joinpath(@__DIR__, "urdf", "Acrobot.urdf"),
            remove_fixed_tree_joints=false, joint_types=push!(default_urdf_joint_types(), "revolute" => revolute_joint_type))
        x_urdf = MechanismState(double_pendulum_urdf)
        for (i, j) in enumerate(joints(double_pendulum))
            urdf_joints = collect(joints(double_pendulum_urdf))
            index = findfirst(joint -> string(joint) == string(j), urdf_joints)
            j_urdf = urdf_joints[index]
            set_configuration!(x_urdf, j_urdf, configuration(x, j)[1])
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
end
