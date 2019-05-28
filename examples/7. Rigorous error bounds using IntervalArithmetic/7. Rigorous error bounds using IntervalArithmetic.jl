# # @__NAME__

# ## Floating-point error

# In computers, real numbers are commonly approximated using floating-point numbers, such as Julia's `Float64`. Unfortunately, not all real numbers can be exactly represented as a finite-size floating-point number, and the results of operations on floating-point numbers can only approximate the results of applying the operation to a true real number. This results in peculiarities like:

2.6 - 0.7 - 1.9


# IntervalArithmetic.jl can be used to quantify floating point error, by computing _rigorous_ worst-case bounds on floating point error, within which the true result is _guaranteed_ to lie.

using Pkg # hide
Pkg.activate(@__DIR__) # hide
Pkg.instantiate() # hide
using IntervalArithmetic


# IntervalArithmetic.jl provides the `Interval` type, which stores an upper and a lower bound:

i = Interval(1.0, 2.0)

#-

dump(i)


# IntervalArithmetic.jl provides overloads for most common Julia functions that take these bounds into account. For example:

i + i

#-

sin(i)


# Note that the bounds computed by IntervalArithmetic.jl take floating point error into account. Also note that a given real number, once converted to (approximated by) a floating-point number may not be equal to the original real number. To rigorously construct an `Interval` that contains a given real number as an input, IntervalArithmetic.jl provides the `@interval` macro:

i = @interval(2.9)
i.lo === i.hi

#-

dump(i)


# Compare this to

i = Interval(2.9)
i.lo === i.hi

#-

dump(i)


# As an example, consider again the peculiar result from before, now using interval arithmetic:

i = @interval(2.6) - @interval(0.7) - @interval(1.9)


# showing that the true result, `0`, is indeed in the guaranteed interval, and indeed:

using Test
@test (2.6 - 0.7 - 1.9) ∈ i


# ## Accuracy of RigidBodyDynamics.jl's `mass_matrix`

# Let's use IntervalArithmetic.jl to establish rigorous bounds on the accuracy of the accuracy of the `mass_matrix` algorithm for the Acrobot (double pendulum) in a certain configuration. Let's get started.

using RigidBodyDynamics


# We'll create a `Mechanism` by parsing the Acrobot URDF, passing in `Interval{Float64}` as the type used to store the parameters (inertias, link lengths, etc.) of the mechanism. Note that the parameters parsed from the URDF are treated as floating point numbers (i.e., like `Interval(2.9)` instead of `@interval(2.9)` above).

const T = Interval{Float64}
srcdir = dirname(pathof(RigidBodyDynamics))
urdf = joinpath(srcdir, "..", "test", "urdf", "Acrobot.urdf")
const mechanism = parse_urdf(urdf; scalar_type=T)
state = MechanismState(mechanism)


# Let's set the initial joint angle of the shoulder joint to the smallest `Interval{Float64}` containing the real number $1$, and similarly for the elbow joint:

shoulder, elbow = joints(mechanism)
set_configuration!(state, shoulder, @interval(1))
set_configuration!(state, elbow, @interval(2));


# And now we can compute the mass matrix as normal:

M = mass_matrix(state)


# Woah, those bounds look pretty big. RigidBodyDynamics.jl must not be very accurate! Actually, things aren't so bad; the issue is just that IntervalArithmetic.jl isn't kidding when it comes to guaranteed bounds, and that includes printing the numbers in shortened form. Here are the lengths of the intervals:

err = map(x -> x.hi - x.lo, M)

#-

@test maximum(abs, err) ≈ 0 atol = 1e-14


# ## Rigorous (worst-case) uncertainty propagation

# IntervalArithmetic.jl can also be applied to propagate uncertainty in a rigorous way when the inputs themselves are uncertain. Consider for example the case that we only know the joint angles up to $\pm 0.05$ radians:

set_configuration!(state, shoulder, @interval(0.95, 1.05))
set_configuration!(state, elbow, @interval(1.95, 2.05));


# and let's compute bounds on the center of mass position:

center_of_mass(state)


# Note that the bounds on the $y$-coordinate are very tight, since our mechanism only lives in the $x$-$z$ plane.
