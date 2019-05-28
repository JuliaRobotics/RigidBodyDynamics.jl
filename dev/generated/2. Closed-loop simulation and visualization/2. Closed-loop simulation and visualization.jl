using Pkg # hide
Pkg.activate("/home/travis/build/JuliaRobotics/RigidBodyDynamics.jl/docs/../examples/2. Closed-loop simulation and visualization") # hide
Pkg.instantiate() # hide
using RigidBodyDynamics

srcdir = dirname(pathof(RigidBodyDynamics))
urdf = joinpath(srcdir, "..", "test", "urdf", "Acrobot.urdf")
mechanism = parse_urdf(urdf)

shoulder, elbow = joints(mechanism)
function simple_control!(torques::AbstractVector, t, state::MechanismState)
    torques[velocity_range(state, shoulder)] .= -1 .* velocity(state, shoulder)
    torques[velocity_range(state, elbow)] .= 10 * sin(t)
end;

state = MechanismState(mechanism)
zero_velocity!(state)
set_configuration!(state, shoulder, 0.7)
set_configuration!(state, elbow, -0.8);

final_time = 10.
ts, qs, vs = simulate(state, final_time, simple_control!; Δt = 1e-3);

using MeshCatMechanisms

mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf))
OPEN_VISUALIZER = false
OPEN_VISUALIZER && open(mvis);

MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 1.);

# This file was generated using Literate.jl, https://github.com/fredrikekre/Literate.jl

