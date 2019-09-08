# # @__NAME__

# PREAMBLE

# PKG_SETUP

# ## Setup

using Pkg # hide
Pkg.activate(@__DIR__) # hide
Pkg.instantiate() # hide
using RigidBodyDynamics, StaticArrays, ForwardDiff
using Test, Random
Random.seed!(1); # to get repeatable results

# ## Jacobians with respect to $q$ and $v$ - the naive way

# First, we'll load our trusty double pendulum from a URDF:

srcdir = dirname(pathof(RigidBodyDynamics))
urdf = joinpath(srcdir, "..", "test", "urdf", "Acrobot.urdf")
mechanism = parse_urdf(urdf)

# Of course, we can create a `MechanismState` for the double pendulum, and compute its momentum in some random state:

float64state = MechanismState(mechanism)
rand!(float64state)
momentum(float64state)

# But now suppose we want the Jacobian of momentum with respect to the joint velocity vector $v$.
# We can do this using the `ForwardDiff.Dual` type and the `ForwardDiff.jacobian` function.
# The ForwardDiff package implements forward-mode [automatic differentiation](https://en.wikipedia.org/wiki/Automatic_differentiation).

# To use `ForwardDiff.jacobian` we'll create a function that maps `v` (as a `Vector`) to momentum (as a `Vector`):

q = configuration(float64state)
function momentum_vec(v::AbstractVector{T}) where T
    ## create a `MechanismState` that can handle the element type of `v` (which will be some `ForwardDiff.Dual`):
    state = MechanismState{T}(mechanism)

    ## set the state variables:
    set_configuration!(state, q)
    set_velocity!(state, v)

    ## return momentum converted to an `SVector` (as ForwardDiff expects an `AbstractVector`)
    Vector(SVector(momentum(state)))
end

# Let's first check that the function returns the same thing we got from `float64state`:

v = velocity(float64state)
@test momentum_vec(v) == SVector(momentum(float64state))

# That works, so now let's compute the Jacobian with `ForwardDiff`:

J = ForwardDiff.jacobian(momentum_vec, v)

# At this point we note that the matrix `J` is simply the momentum matrix (in world frame) of the `Mechanism`. In this case, RigidBodyDynamics.jl has a specialized algorithm for computing this matrix, so let's verify the results:

A = momentum_matrix(float64state)
@test J ≈ Array(A) atol = 1e-12

# Gradients with respect to $q$ can be computed in similar fashion.


# ## Improving performance

# Ignoring the fact that we have a specialized method available, let's look at the performance of using `ForwardDiff.jacobian`.

using BenchmarkTools
@benchmark ForwardDiff.jacobian($momentum_vec, $v)

# That's not great. Note all the allocations. We can do better by making the following modifications:
#
# 1. use an in-place version of the `jacobian` function, `ForwardDiff.jacobian!`
# 2. reimplement our `momentum_vec` function to be in-place as well
# 3. don't create a new `MechanismState` every time
#
# The third point is especially important; creating a `MechanismState` is expensive!
#
# Regarding the second point, we could also just stop converting momentum from a `StaticArrays.SVector` to a `Vector` to avoid allocations. However, the solution of making the function in-place also applies when the size of the output vector is not known statically (e.g., for `dynamics_bias!`).

# To facillitate reuse of `MechanismState`s while keeping the code nice and generic, we can use a `StateCache` object.
# `StateCache` is a container that stores `MechanismState`s of various types (associated with one `Mechanism`), and will ease the process of using `ForwardDiff`.
# Creating one is easy:

const statecache = StateCache(mechanism)

# `MechanismState`s of a given type can be accessed as follows (note that if a `MechanismState` of a certain type is already available, it will be reused):

float32state = statecache[Float32]
@test float32state === statecache[Float32]

# Now we'll use the `StateCache` to reimplement `momentum_vec`, making it in-place as well:

function momentum_vec!(out::AbstractVector, v::AbstractVector{T}) where T
    ## retrieve a `MechanismState` that can handle the element type of `v`:
    state = statecache[T]

    ## set the state variables:
    set_configuration!(state, q)
    set_velocity!(state, v)

    ## compute momentum and store it in `out`
    m = momentum(state)
    copyto!(out, SVector(m))
end

# Check that the in-place version works as expected on `Float64` inputs:

const out = zeros(6) # where we'll be storing our results
momentum_vec!(out, v)
@test out == SVector(momentum(float64state))

# And use `ForwardDiff.jacobian!` to compute the Jacobian:

const result = DiffResults.JacobianResult(out, v)
const config = ForwardDiff.JacobianConfig(momentum_vec!, out, v)
ForwardDiff.jacobian!(result, momentum_vec!, out, v, config)
J = DiffResults.jacobian(result)
@test J ≈ Array(A) atol = 1e-12

# Let's check the performance again:

@benchmark ForwardDiff.jacobian!($result, $momentum_vec!, $out, $v, $config)

# That's much better. Do note that the specialized algorithm is still faster:

q = copy(configuration(float64state))
@benchmark begin
    set_configuration!($float64state, $q)
    momentum_matrix!($A, $float64state)
end


# ## Time derivatives

# We can also use ForwardDiff to compute time derivatives. Let's verify that energy is conserved for the double pendulum in the absence of nonconservative forces (like damping). That is, we expect that the time derivative of the pendulum's total energy is zero when its state evolves according to the passive dynamics.

# Let's first compute the joint acceleration vector $\dot{v}$ using the passive dynamics:

dynamicsresult = DynamicsResult(mechanism)
set_configuration!(float64state, q)
set_velocity!(float64state, v)
dynamics!(dynamicsresult, float64state)
v̇ = dynamicsresult.v̇

# Now for the time derivative of total energy. ForwardDiff has a `derivative` function that can be used to take derivatives of functions that map a scalar to a scalar. But in this example, we'll instead use ForwardDiff's `Dual` type directly. `ForwardDiff.Dual` represents a (potentially multidimensional) dual number, i.e., a type that stores both the value of a function evaluated at a certain point, as well as the partial derivatives of the function, again evaluated at the same point. See the [ForwardDiff documentation](http://www.juliadiff.org/ForwardDiff.jl/stable/dev/how_it_works.html) for more information.

# We'll create a vector of `Dual`s representing the value and derivative of $q(t)$:

q̇ = v
q_dual = ForwardDiff.Dual.(q, q̇)

# **Note**: for the double pendulum, $\dot{q} = v$, but this is not the case in general for `Mechanism`s created using RigidBodyDynamics.jl. For example, the `QuaternionSpherical` joint type uses a unit quaternion to represent the joint configuration, but angular velocity (in body frame) to represent velocity. In general $\dot{q}$ can be computed from the velocity vector $v$ stored in a `MechanismState` using
#
# ```julia
# configuration_derivative(::MechanismState)
# ```
#
# or its in-place variant, `configuration_derivative!`.

# We'll do the same thing for $v(t)$:

v_dual = ForwardDiff.Dual.(v, v̇)

# Now we're ready to compute the total energy (kinetic + potential) using these `ForwardDiff.Dual` inputs. We'll use our `StateCache` again:

T = eltype(q_dual)
state = statecache[T]
set_configuration!(state, q_dual)
set_velocity!(state, v_dual)
energy_dual = kinetic_energy(state) + gravitational_potential_energy(state)

# Note that the result type of `energy_dual` is again a `ForwardDiff.Dual`. We can extract the energy and its time derivative (mechanical power) from `energy_dual` as follows:

energy = ForwardDiff.value(energy_dual)
partials = ForwardDiff.partials(energy_dual)
power = partials[1];

# So the total energy in the system is:

energy

# **Note**: the total energy is negative because the origin of the world frame is used as a reference for computing gravitational potential energy, i.e., the center of mass of the double pendulum is somewhere below this origin.

# And we can verify that, indeed, there is no power flow into or out of the system:

@test power ≈ 0 atol = 1e-14

