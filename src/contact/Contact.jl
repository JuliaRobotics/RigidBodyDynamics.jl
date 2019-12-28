module Contact

using LinearAlgebra
using RigidBodyDynamics.Spatial
using StaticArrays

# base types
export
    ContactForceModel,
    HalfSpace3D

# interface functions
export
    num_states,
    contact_dynamics,
    point_inside,
    separation,
    detect_contact

# specific models
export HuntCrossleyModel,
    hunt_crossley_hertz

export ViscoelasticCoulombModel,
    ViscoelasticCoulombState

include("collision_detection.jl")
include("contact_force_model.jl")
include("soft_contact_state.jl")

include("hunt_crossley.jl")
include("viscoelastic_coulomb.jl")

end # module
