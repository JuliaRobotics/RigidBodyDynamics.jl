module Contact

using RigidBodyDynamics

using LinearAlgebra
using RigidBodyDynamics.Spatial
using StaticArrays
using TypeSortedCollections
using EnhancedGJK
using DocStringExtensions

import GeometryTypes
import CoordinateTransformations

using RigidBodyDynamics: update_transforms!, update_twists_wrt_world!
using RigidBodyDynamics: frame_definition
using RigidBodyDynamics.CustomCollections: UnorderedPair
using CoordinateTransformations: AffineMap

# base types
export
    ContactForceModel,
    CollisionElement,
    CollisionGroup,
    ContactModel,
    SoftContactState,
    SoftContactResult

# # geometry
# export
#     HalfSpace3D

# interface functions
export
    set_contact_force_model!,
    contact_dynamics

# specific models
export
    SplitContactForceModel,
    HuntCrossleyModel,
    hunt_crossley_hertz,
    ViscoelasticCoulombModel

include("collision_element.jl")
include("collision_detection.jl")
include("contact_force_model.jl")
include("collidable_pair.jl")
include("contact_model.jl")
include("soft_contact_state.jl")

include("hunt_crossley.jl")
include("viscoelastic_coulomb.jl")

end # module
