using Pkg # hide
Pkg.activate("/home/travis/build/JuliaRobotics/RigidBodyDynamics.jl/docs/../examples/3. Four-bar linkage") # hide
Pkg.instantiate() # hide
using LinearAlgebra
using RigidBodyDynamics
using StaticArrays

# gravitational acceleration
g = -9.81

# link lengths
l_0 = 1.10
l_1 = 0.5
l_2 = 1.20
l_3 = 0.75

# link masses
m_1 = 0.5
m_2 = 1.0
m_3 = 0.75

# link center of mass offsets from the preceding joint axes
c_1 = 0.25
c_2 = 0.60
c_3 = 0.375

# moments of inertia about the center of mass of each link
I_1 = 0.333
I_2 = 0.537
I_3 = 0.4

# Rotation axis: negative y-axis
axis = SVector(0., -1., 0.);

world = RigidBody{Float64}("world")
fourbar = Mechanism(world; gravity = SVector(0., 0., g))

joint1 = Joint("joint1", Revolute(axis))
inertia1 = SpatialInertia(frame_after(joint1),
    com=SVector(c_1, 0, 0),
    moment_about_com=I_1*axis*transpose(axis),
    mass=m_1)
link1 = RigidBody(inertia1)
before_joint1_to_world = one(Transform3D,
    frame_before(joint1), default_frame(world))
attach!(fourbar, world, link1, joint1,
    joint_pose = before_joint1_to_world)

joint2 = Joint("joint2", Revolute(axis))
inertia2 = SpatialInertia(frame_after(joint2),
    com=SVector(c_2, 0, 0),
    moment_about_com=I_2*axis*transpose(axis),
    mass=m_2)
link2 = RigidBody(inertia2)
before_joint2_to_after_joint1 = Transform3D(
    frame_before(joint2), frame_after(joint1), SVector(l_1, 0., 0.))
attach!(fourbar, link1, link2, joint2,
    joint_pose = before_joint2_to_after_joint1)

joint3 = Joint("joint3", Revolute(axis))
inertia3 = SpatialInertia(frame_after(joint3),
    com=SVector(l_0, 0., 0.),
    moment_about_com=I_3*axis*transpose(axis),
    mass=m_3)
link3 = RigidBody(inertia3)
before_joint3_to_world = Transform3D(frame_before(joint3),
    default_frame(world), SVector(l_0, 0., 0.))
attach!(fourbar, world, link3, joint3, joint_pose = before_joint3_to_world)

# joint between link2 and link3
joint4 = Joint("joint4", Revolute(axis))
before_joint4_to_joint2 = Transform3D(
    frame_before(joint4), frame_after(joint2), SVector(l_2, 0., 0.))
joint3_to_after_joint4 = Transform3D(
    frame_after(joint3), frame_after(joint4), SVector(-l_3, 0., 0.))
attach!(fourbar, link2, link3, joint4,
    joint_pose = before_joint4_to_joint2, successor_pose = joint3_to_after_joint4)

state = MechanismState(fourbar)
result = DynamicsResult(fourbar);

set_configuration!(state, joint1, 1.6707963267948966) # θ
set_configuration!(state, joint2, -1.4591054166649482) # γ
set_configuration!(state, joint3, 1.5397303602625536) # ϕ

set_velocity!(state, joint1, 0.5)
set_velocity!(state, joint2, -0.47295)
set_velocity!(state, joint3, 0.341)

# Invalidate the cache variables
setdirty!(state)

ts, qs, vs = simulate(state, 3., Δt = 1e-2);

using MeshCatMechanisms

mvis = MechanismVisualizer(fourbar, Skeleton(inertias=false))
OPEN_VISUALIZER = false
OPEN_VISUALIZER && open(mvis);

MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 1.);

# This file was generated using Literate.jl, https://github.com/fredrikekre/Literate.jl

