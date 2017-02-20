using Documenter, RigidBodyDynamics

makedocs(
    # options
    modules = [RigidBodyDynamics],
    format = :html,
    sitename ="RigidBodyDynamics.jl",
    authors = "Twan Koolen and contributors.",
    pages = [
        "Home" => "index.md",
        "Quick start guide" => "quickstart.md",
        "Spatial vector algebra" => "spatial.md",
        "Joints" => "joints.md",
        "Rigid bodies" => "rigidbody.md",
        "Mechanism" => "mechanism.md",
        "MechanismState" => "mechanismstate.md",
        "Kinematics/dynamics algorithms" => "algorithms.md",
        "Simulation" => "simulation.md",
        "Benchmarks" => "benchmarks.md"
      ]
)

deploydocs(
    repo   = "github.com/tkoolen/RigidBodyDynamics.jl.git",
    target = "build",
    deps   = nothing,
    make   = nothing
)
