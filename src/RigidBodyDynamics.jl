__precompile__()

module RigidBodyDynamics

using Base.Random
using StaticArrays
using Rotations
using LightXML
using DocStringExtensions
using Compat
import Base: *, +, /, -, \
import Compat.Iterators: filter
import Compat.Iterators: flatten

const noalloc_doc = """This method does its computation in place, performing no dynamic memory allocation."""

include("graphs.jl")
include("util.jl")
include("third_party_addendum.jl")
include("cache_element.jl")

include("frames.jl")
include("spatial.jl")
include("contact.jl")

include("joint_types.jl")
include("joint.jl")

using .Contact
include("rigid_body.jl")

using .Graphs
include("mechanism.jl")
include("mechanism_manipulation.jl")
include("mechanism_state.jl")
include("dynamics_result.jl")
include("mechanism_algorithms.jl")
include("parse_urdf.jl")
include("ode_integrators.jl")
include("simulate.jl")

export
    # types
    CartesianFrame3D,
    Transform3D,
    Point3D,
    FreeVector3D,
    SpatialInertia,
    RigidBody,
    Joint,
    JointType,
    QuaternionFloating,
    Revolute,
    Prismatic,
    Fixed,
    Twist,
    GeometricJacobian,
    Wrench,
    WrenchMatrix,
    Momentum,
    MomentumMatrix,
    SpatialAcceleration,
    Mechanism,
    MechanismState,
    DynamicsResult,
    # functions
    name, # TODO: remove?
    has_defined_inertia,
    default_frame,
    frame_before,
    frame_after,
    add_frame!,
    transform,
    invtransform,
    newton_euler,
    torque,
    joint_torque!,
    point_velocity,
    local_coordinates!,
    global_coordinates!,
    root_frame,
    tree,
    root_body,
    non_root_bodies,
    isroot,
    bodies,
    path,
    joints,
    tree_joints,
    successor,
    predecessor,
    in_joints,
    out_joints,
    joints_to_children,
    joint_to_parent,
    configuration_derivative,
    velocity_to_configuration_derivative!,
    configuration_derivative_to_velocity!,
    num_positions,
    num_velocities,
    num_additional_states,
    num_bodies,
    configuration,
    configuration_range,
    velocity,
    velocity_range,
    additional_state,
    num_cols,
    joint_transform,
    motion_subspace,
    motion_subspace_in_world,
    constraint_wrench_subspace,
    has_fixed_subspaces,
    bias_acceleration,
    spatial_inertia,
    spatial_inertia!,
    add_contact_point!,
    contact_points,
    crb_inertia,
    setdirty!,
    add_body_fixed_frame!,
    fixed_transform,
    attach!,
    reattach!, # deprecated
    remove_joint!,
    maximal_coordinates,
    submechanism,
    remove_fixed_joints!, # deprecated
    remove_fixed_tree_joints!,
    rand_mechanism,
    rand_chain_mechanism,
    rand_tree_mechanism,
    rand_floating_tree_mechanism,
    configuration_vector, # deprecated
    velocity_vector, # deprecated
    state_vector, # TODO: Base.convert method to Vector?
    rand_configuration!,
    zero_configuration!,
    rand_velocity!,
    zero_velocity!,
    set_configuration!,
    set_velocity!,
    set_additional_state!,
    set!,
    zero!,
    add_frame!,
    twist_wrt_world,
    relative_twist,
    transform_to_root,
    relative_transform,
    mass,
    center_of_mass,
    geometric_jacobian,
    geometric_jacobian!,
    relative_acceleration,
    kinetic_energy,
    gravitational_potential_energy,
    mass_matrix!,
    mass_matrix,
    momentum,
    momentum_matrix!,
    momentum_matrix,
    momentum_rate_bias,
    inverse_dynamics!,
    inverse_dynamics,
    dynamics!,
    parse_urdf,
    simulate,
    # macros
    @framecheck

end # module
