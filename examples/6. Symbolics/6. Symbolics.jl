# # @__NAME__

# PREAMBLE

# PKG_SETUP

# ## Setup

using Quaternions
using RigidBodyDynamics
using StaticArrays
using Symbolics

#-

## Type piracy needed in order to make Symbolics.jl types interact well with Quaternions.jl.
## This should be avoided — see https://docs.julialang.org/en/v1/manual/style-guide/#avoid-type-piracy.
function Base.:/(q::Quaternions.Quaternion, x::Symbolics.Num)
    return Quaternions.Quaternion(q.s / x.val, q.v1 / x.val, q.v2 / x.val, q.v3 / x.val)
end

# ## Create symbolic parameters
# * Masses: $m_1, m_2$
# * Mass moments of inertia (about center of mass): $I_1, I_2$
# * Link lengths: $l_1, l_2$
# * Center of mass locations (w.r.t. preceding joint axis): $c_1, c_2$
# * Gravitational acceleration: $g$

inertias = @variables m₁ m₂ I₁ I₂
lengths = @variables l₁ l₂ c₁ c₂
gravitational_acceleration = @variables g
params = [inertias..., lengths..., gravitational_acceleration...]
transpose(params)


# ## Create double pendulum `Mechanism`
# A `Mechanism` contains the joint layout and inertia parameters, but no state information.

T = Num  # the 'scalar type' of the Mechanism we'll construct
axis = SVector(zero(T), one(T), zero(T)) # axis of rotation for each of the joints
double_pendulum = Mechanism(RigidBody{T}("world"); gravity=SVector(zero(T), zero(T), g))
world = root_body(double_pendulum) # the fixed 'world' rigid body

# Attach the first (upper) link to the world via a revolute joint named 'shoulder'
inertia1 = SpatialInertia(CartesianFrame3D("upper_link"),
    moment=I₁ * axis * transpose(axis),
    com=SVector(zero(T), zero(T), c₁),
    mass=m₁)
body1 = RigidBody(inertia1)
joint1 = Joint("shoulder", Revolute(axis))
joint1_to_world = one(Transform3D{T}, frame_before(joint1), default_frame(world));
attach!(double_pendulum, world, body1, joint1,
    joint_pose=joint1_to_world);

# Attach the second (lower) link to the world via a revolute joint named 'elbow'
inertia2 = SpatialInertia(CartesianFrame3D("lower_link"),
    moment=I₂ * axis * transpose(axis),
    com=SVector(zero(T), zero(T), c₂),
    mass=m₂)
body2 = RigidBody(inertia2)
joint2 = Joint("elbow", Revolute(axis))
joint2_to_body1 = Transform3D(
    frame_before(joint2), default_frame(body1), SVector(zero(T), zero(T), l₁))
attach!(double_pendulum, body1, body2, joint2,
    joint_pose=joint2_to_body1)


# ## Create `MechanismState` associated with the double pendulum `Mechanism`
# A `MechanismState` stores all state-dependent information associated with a `Mechanism`.

x = MechanismState(double_pendulum);

# Set the joint configuration vector of the MechanismState to a new vector of symbolic variables
q = configuration(x)
q .= Symbolics.variables(:q, 1:length(q))

# Set the joint velocity vector of the MechanismState to a new vector of symbolic variables
v = velocity(x)
v .= Symbolics.variables(:v, 1:length(q))


# ## Compute dynamical quantities in symbolic form

# Mass matrix
simplify.(mass_matrix(x), expand=true)


# Kinetic energy
simplify(kinetic_energy(x), expand=true)


# Potential energy
try
    ## This throws `ERROR: TypeError: non-boolean (Num) used in boolean context` because
    ## of `m > 0 || return zero(cache_eltype(state))` in `gravitational_potential_energy`.
    ## See https://docs.sciml.ai/Symbolics/stable/manual/faq/ for more details.
    simplify(gravitational_potential_energy(x), expand=true)
catch e
    e
end
