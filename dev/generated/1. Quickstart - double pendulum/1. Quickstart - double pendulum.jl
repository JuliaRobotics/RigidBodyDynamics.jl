using Pkg # hide
Pkg.activate("/home/travis/build/JuliaRobotics/RigidBodyDynamics.jl/docs/../examples/1. Quickstart - double pendulum") # hide
Pkg.instantiate() # hide
using RigidBodyDynamics
using LinearAlgebra
using StaticArrays

g = -9.81 # gravitational acceleration in z-direction
world = RigidBody{Float64}("world")
doublependulum = Mechanism(world; gravity = SVector(0, 0, g))

axis = SVector(0., 1., 0.) # joint axis
I_1 = 0.333 # moment of inertia about joint axis
c_1 = -0.5 # center of mass location with respect to joint axis
m_1 = 1. # mass
frame1 = CartesianFrame3D("upper_link") # the reference frame in which the spatial inertia will be expressed
inertia1 = SpatialInertia(frame1, moment=I_1 * axis * axis', com=SVector(0, 0, c_1), mass=m_1)

upperlink = RigidBody(inertia1)

shoulder = Joint("shoulder", Revolute(axis))

before_shoulder_to_world = one(Transform3D, frame_before(shoulder), default_frame(world))

attach!(doublependulum, world, upperlink, shoulder, joint_pose = before_shoulder_to_world)

l_1 = -1. # length of the upper link
I_2 = 0.333 # moment of inertia about joint axis
c_2 = -0.5 # center of mass location with respect to joint axis
m_2 = 1. # mass
inertia2 = SpatialInertia(CartesianFrame3D("lower_link"), moment=I_2 * axis * axis', com=SVector(0, 0, c_2), mass=m_2)
lowerlink = RigidBody(inertia2)
elbow = Joint("elbow", Revolute(axis))
before_elbow_to_after_shoulder = Transform3D(frame_before(elbow), frame_after(shoulder), SVector(0, 0, l_1))
attach!(doublependulum, upperlink, lowerlink, elbow, joint_pose = before_elbow_to_after_shoulder)

srcdir = dirname(pathof(RigidBodyDynamics))
urdf = joinpath(srcdir, "..", "test", "urdf", "Acrobot.urdf")
parse_urdf(urdf)

state = MechanismState(doublependulum)

set_configuration!(state, shoulder, 0.3)
set_configuration!(state, elbow, 0.4)
set_velocity!(state, shoulder, 1.)
set_velocity!(state, elbow, 2.);

setdirty!(state)

q = configuration(state)
v = velocity(state)

transform(state, Point3D(frame_after(elbow), zero(SVector{3})), default_frame(world))

transform_to_root(state, frame_after(elbow))

relative_transform(state, frame_after(elbow), frame_after(shoulder))

center_of_mass(state)

mass_matrix(state)

v̇ = similar(velocity(state)) # the joint acceleration vector, i.e., the time derivative of the joint velocity vector v
v̇[shoulder][1] = 1
v̇[elbow][1] = 2
inverse_dynamics(state, v̇)

ts, qs, vs = simulate(state, 5., Δt = 1e-3);

# This file was generated using Literate.jl, https://github.com/fredrikekre/Literate.jl

