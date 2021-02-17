# # @__NAME__

# PREAMBLE

# PKG_SETUP

# Please note that [RigidBodySim.jl](https://github.com/JuliaRobotics/RigidBodySim.jl) now provides a more capable simulation environment.

# ## Setup

using RigidBodyDynamics

# ## Model definition

# We'll just use the double pendulum model, loaded from a URDF:

srcdir = dirname(pathof(RigidBodyDynamics))
urdf = joinpath(srcdir, "..", "test", "urdf", "Acrobot.urdf")
mechanism = parse_urdf(urdf)

# ## Controller

# Let's write a simple controller that just applies $10 \sin(t)$ at the elbow joint and adds some damping at the shoulder joint:

shoulder, elbow = joints(mechanism)
function simple_control!(torques::AbstractVector, t, state::MechanismState)
    torques[velocity_range(state, shoulder)] .= -1 .* velocity(state, shoulder)
    torques[velocity_range(state, elbow)] .= 10 * sin(t)
end;


# ## Simulation

# Basic simulation can be done using the `simulate` function. We'll first create a `MechanismState` object, and set the initial joint configurations and velocities:

state = MechanismState(mechanism)
zero_velocity!(state)
set_configuration!(state, shoulder, 0.7)
set_configuration!(state, elbow, -0.8);


# Now we can simply call `simulate`, which will return a tuple consisting of:
# * simulation times (a `Vector` of numbers)
# * joint configuration vectors (a `Vector` of `Vector`s)
# * joint velocity vectors (a `Vector` of `Vector`s)

final_time = 10.
ts, qs, vs = simulate(state, final_time, simple_control!; Δt = 1e-3);


# For access to lower-level functionality, such as different ways of storing or visualizing the data generated during the simulation, it is advised to simply pattern match the basic `simulate` function.

# ## Visualization

# For visualization, we'll use [`MeshCatMechanisms`](https://github.com/JuliaRobotics/MeshCatMechanisms.jl), an external package based on RigidBodyDynamics.jl.

using MeshCatMechanisms

# Create a `MechanismVisualizer` and open it in a new browser tab
# (see [`MeshCat.jl`](https://github.com/rdeits/MeshCat.jl) for other options):

mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf));

#-

#nb ##NBSKIP
#nb open(mvis)
#md ## open(mvis)


# And animate:
MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 1.);
