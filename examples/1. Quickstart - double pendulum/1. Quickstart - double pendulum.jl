# # @__NAME__

# ## Setup

# In addition to `RigidBodyDynamics`, we'll be using the `StaticArrays` package, used throughout `RigidBodyDynamics`, which provides stack-allocated, fixed-size arrays:

using Pkg # hide
Pkg.activate(@__DIR__) # hide
Pkg.instantiate() # hide
using RigidBodyDynamics
using LinearAlgebra
using StaticArrays

# ## Creating a double pendulum `Mechanism`

# We're going to create a simple `Mechanism` that represents a [double pendulum](https://en.wikipedia.org/wiki/Double_pendulum). The `Mechanism` type can be thought of as an interconnection of rigid bodies and joints.
#
# We'll start by creating a 'root' rigid body, representing the fixed world, and using it to create a new `Mechanism`:

g = -9.81 # gravitational acceleration in z-direction
world = RigidBody{Float64}("world")
doublependulum = Mechanism(world; gravity = SVector(0, 0, g))


# Note that the `RigidBody` type is parameterized on the 'scalar type', here `Float64`.
#
# We'll now add a second body, called 'upper link', to the `Mechanism`. We'll attach it to the world with a revolute joint, with the $y$-axis as the axis of rotation. We'll start by creating a `SpatialInertia`, which stores the inertial properties of the new body:

axis = SVector(0., 1., 0.) # joint axis
I_1 = 0.333 # moment of inertia about joint axis
c_1 = -0.5 # center of mass location with respect to joint axis
m_1 = 1. # mass
frame1 = CartesianFrame3D("upper_link") # the reference frame in which the spatial inertia will be expressed
inertia1 = SpatialInertia(frame1, moment=I_1 * axis * axis', com=SVector(0, 0, c_1), mass=m_1)


# Note that the created `SpatialInertia` is annotated with the frame in which it is expressed (in the form of a `CartesianFrame3D`). This is a common theme among `RigidBodyDynamics` objects. Storing frame information with the data obviates the need for the complicated variable naming conventions that are used in some other libraries to disambiguate the frame in which quantities are expressed. It also enables automated reference frame checks.

# We'll now create the second body:

upperlink = RigidBody(inertia1)


# and a new revolute joint called 'shoulder':

shoulder = Joint("shoulder", Revolute(axis))


# Creating a `Joint` automatically constructs two new `CartesianFrame3D` objects: a frame directly before the joint, and one directly after. To attach the new body to the world by this joint, we'll have to specify where the frame before the joint is located on the parent body (here, the world):

before_shoulder_to_world = one(Transform3D, frame_before(shoulder), default_frame(world))


# Now we can attach the upper link to the world:

attach!(doublependulum, world, upperlink, shoulder, joint_pose = before_shoulder_to_world)


# which changes the tree representation of the `Mechanism`.

# We can attach the lower link in similar fashion:

l_1 = -1. # length of the upper link
I_2 = 0.333 # moment of inertia about joint axis
c_2 = -0.5 # center of mass location with respect to joint axis
m_2 = 1. # mass
inertia2 = SpatialInertia(CartesianFrame3D("lower_link"), moment=I_2 * axis * axis', com=SVector(0, 0, c_2), mass=m_2)
lowerlink = RigidBody(inertia2)
elbow = Joint("elbow", Revolute(axis))
before_elbow_to_after_shoulder = Transform3D(frame_before(elbow), frame_after(shoulder), SVector(0, 0, l_1))
attach!(doublependulum, upperlink, lowerlink, elbow, joint_pose = before_elbow_to_after_shoulder)


# Now our double pendulum `Mechanism` is complete.

# **Note**: instead of defining the `Mechanism` in this way, it is also possible to load in a [URDF](http://wiki.ros.org/urdf) file (an XML file format used in ROS), using the `parse_urdf` function, e.g.:

srcdir = dirname(pathof(RigidBodyDynamics))
urdf = joinpath(srcdir, "..", "test", "urdf", "Acrobot.urdf")
parse_urdf(urdf)


# ## The state of a `Mechanism`

# A `Mechanism` stores the joint/rigid body layout, but no state information. State information is separated out into a `MechanismState` object:

state = MechanismState(doublependulum)


# Let's first set the configurations and velocities of the joints:

set_configuration!(state, shoulder, 0.3)
set_configuration!(state, elbow, 0.4)
set_velocity!(state, shoulder, 1.)
set_velocity!(state, elbow, 2.);


# **Important**: a `MechanismState` contains cache variables that depend on the configurations and velocities of the joints. These need to be invalidated when the configurations and velocities are changed. To do this, call

setdirty!(state)


# The joint configurations and velocities are stored as `Vector`s (denoted $q$ and $v$ respectively in this package) inside the `MechanismState` object:

q = configuration(state)
v = velocity(state)


# ## Kinematics

# We are now ready to do kinematics. Here's how you transform a point at the origin of the frame after the elbow joint to world frame:

transform(state, Point3D(frame_after(elbow), zero(SVector{3})), default_frame(world))


# Other objects like `Wrench`es, `Twist`s, and `SpatialInertia`s can be transformed in similar fashion.

# You can also ask for the homogeneous transform to world:

transform_to_root(state, frame_after(elbow))


# Or a relative transform:

relative_transform(state, frame_after(elbow), frame_after(shoulder))


# and here's the center of mass of the double pendulum:

center_of_mass(state)


# ## Dynamics

# A `MechanismState` can also be used to compute quantities related to the dynamics of the `Mechanism`. Here we compute the mass matrix:

mass_matrix(state)


# Note that there is also a zero-allocation version, `mass_matrix!` (the `!` at the end of a method is a Julia convention signifying that the function is 'in-place', i.e. modifies its input data).

# We can do inverse dynamics as follows (note again that there is a non-allocating version of this method as well):

v̇ = similar(velocity(state)) # the joint acceleration vector, i.e., the time derivative of the joint velocity vector v
v̇[shoulder][1] = 1
v̇[elbow][1] = 2
inverse_dynamics(state, v̇)


# ## Simulation

# Let's simulate the double pendulum for 5 seconds, starting from the state we defined earlier. For this, we can use the basic `simulate` function:

ts, qs, vs = simulate(state, 5., Δt = 1e-3);


# `simulate` returns a vector of times (`ts`) and associated joint configurations (`qs`) and velocities (`vs`). You can of course plot the trajectories using your favorite plotting package (see e.g. [Plots.jl](https://github.com/JuliaPlots/Plots.jl)). The [MeshCatMechanisms](https://github.com/JuliaRobotics/MeshCatMechanisms.jl) or [RigidBodyTreeInspector](https://github.com/rdeits/RigidBodyTreeInspector.jl) packages can also be used for 3D animation of the double pendulum in action. See also [RigidBodySim.jl](https://github.com/JuliaRobotics/RigidBodySim.jl) for a more full-fledge simulation environment.

# A lower level interface for simulation/ODE integration with more options is also available.
# Consult the documentation for more information.
# In addition, [RigidBodySim.jl](https://github.com/JuliaRobotics/RigidBodySim.jl) offers a more full-featured simulation environment.
