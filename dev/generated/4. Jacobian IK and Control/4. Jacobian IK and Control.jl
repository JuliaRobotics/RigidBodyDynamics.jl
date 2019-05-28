using Pkg # hide
Pkg.activate("/home/travis/build/JuliaRobotics/RigidBodyDynamics.jl/docs/../examples/4. Jacobian IK and Control") # hide
Pkg.instantiate() # hide
using RigidBodyDynamics
using StaticArrays
using MeshCatMechanisms, Blink

using Random
Random.seed!(42);

srcdir = dirname(pathof(RigidBodyDynamics))
urdf = joinpath(srcdir, "..", "test", "urdf", "Acrobot.urdf")
mechanism = parse_urdf(urdf)
state = MechanismState(mechanism)
mechanism

body = findbody(mechanism, "lower_link")
point = Point3D(default_frame(body), 0., 0, -2)

# Create the visualizer
vis = MechanismVisualizer(mechanism, URDFVisuals(urdf))

# Render our target point attached to the robot as a sphere with radius 0.07
setelement!(vis, point, 0.07)

# Open the visualizer in a new Blink window
OPEN_VISUALIZER = false
OPEN_VISUALIZER && open(vis, Window());

function jacobian_transpose_ik!(state::MechanismState,
                               body::RigidBody,
                               point::Point3D,
                               desired::Point3D;
                               α=0.1,
                               iterations=100)
    mechanism = state.mechanism
    world = root_frame(mechanism)

    # Compute the joint path from world to our target body
    p = path(mechanism, root_body(mechanism), body)
    # Allocate the point jacobian (we'll update this in-place later)
    Jp = point_jacobian(state, p, transform(state, point, world))

    q = copy(configuration(state))

    for i in 1:iterations
        # Update the position of the point
        point_in_world = transform(state, point, world)
        # Update the point's jacobian
        point_jacobian!(Jp, state, p, point_in_world)
        # Compute an update in joint coordinates using the jacobian transpose
        Δq = α * Array(Jp)' * (transform(state, desired, world) - point_in_world).v
        # Apply the update
        q .= configuration(state) .+ Δq
        set_configuration!(state, q)
    end
    state
end

rand!(state)
set_configuration!(vis, configuration(state))

desired_tip_location = Point3D(root_frame(mechanism), 0.5, 0, 2)

jacobian_transpose_ik!(state, body, point, desired_tip_location)
set_configuration!(vis, configuration(state))

transform(state, point, root_frame(mechanism))

qs = typeof(configuration(state))[]

for x in range(-1, stop=1, length=100)
    desired = Point3D(root_frame(mechanism), x, 0, 2)
    jacobian_transpose_ik!(state, body, point, desired)
    push!(qs, copy(configuration(state)))
end
ts = collect(range(0, stop=1, length=length(qs)))
setanimation!(vis, ts, qs)

circle_origin = SVector(0., 0.25, 2)
radius = 0.5
ω = 1.0  # radians per second at which the point should move in its circle

using MeshCat
using GeometryTypes: Point

# Draw the circle in the viewer
θ = repeat(range(0, stop=2π, length=100), inner=(2,))[2:end]
cx, cy, cz = circle_origin
geometry = PointCloud(Point.(cx .+ radius .* sin.(θ), cy, cz .+ 0.5 .* cos.(θ)))
setobject!(vis[:circle], LineSegments(geometry, LineBasicMaterial()))

function make_circle_controller(state::MechanismState,
                                body::RigidBody,
                                point::Point3D,
                                circle_origin::AbstractVector,
                                radius,
                                ω)
    mechanism = state.mechanism
    world = root_frame(mechanism)
    joint_path = path(mechanism, root_body(mechanism), body)
    Jp = point_jacobian(state, joint_path, transform(state, point, root_frame(mechanism)))
    v̇ = similar(velocity(state))

    function controller!(τ, t, state)
        desired = Point3D(world, circle_origin .+ radius .* SVector(sin(t / ω), 0, cos(t / ω)))
        point_in_world = transform_to_root(state, body) * point
        point_jacobian!(Jp, state, joint_path, point_in_world)
        Kp = 200
        Kd = 20
        Δp = desired - point_in_world
        v̇ .= Kp * Array(Jp)' * Δp.v .- 20 .* velocity(state)
        τ .= inverse_dynamics(state, v̇)
    end
end
controller! = make_circle_controller(state, body, point, circle_origin, radius, ω)
ts, qs, vs = simulate(state, 10, controller!);

setanimation!(vis, ts, qs)

using Plots; gr()

xs = Float64[]
zs = Float64[]

# Downsample by 100 just so the plot doesn't become a huge file:
for q in qs[1:100:end]
    set_configuration!(state, q)
    p = transform(state, point, root_frame(mechanism))
    push!(xs, p.v[1])
    push!(zs, p.v[3])
end

plot(xs, zs, xlim=(-1, 1), ylim=(1, 3), aspect_ratio=:equal, legend=nothing)

# This file was generated using Literate.jl, https://github.com/fredrikekre/Literate.jl

