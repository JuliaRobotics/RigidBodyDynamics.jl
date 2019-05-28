2.6 - 0.7 - 1.9

using Pkg # hide
Pkg.activate("/home/travis/build/JuliaRobotics/RigidBodyDynamics.jl/docs/../examples/7. Rigorous error bounds using IntervalArithmetic") # hide
Pkg.instantiate() # hide
using IntervalArithmetic

i = Interval(1.0, 2.0)

dump(i)

i + i

sin(i)

i = @interval(2.9)
i.lo === i.hi

dump(i)

i = Interval(2.9)
i.lo === i.hi

dump(i)

i = @interval(2.6) - @interval(0.7) - @interval(1.9)

using Test
@test (2.6 - 0.7 - 1.9) ∈ i

using RigidBodyDynamics

const T = Interval{Float64}
srcdir = dirname(pathof(RigidBodyDynamics))
urdf = joinpath(srcdir, "..", "test", "urdf", "Acrobot.urdf")
const mechanism = parse_urdf(urdf; scalar_type=T)
state = MechanismState(mechanism)

shoulder, elbow = joints(mechanism)
set_configuration!(state, shoulder, @interval(1))
set_configuration!(state, elbow, @interval(2));

M = mass_matrix(state)

err = map(x -> x.hi - x.lo, M)

@test maximum(abs, err) ≈ 0 atol = 1e-14

set_configuration!(state, shoulder, @interval(0.95, 1.05))
set_configuration!(state, elbow, @interval(1.95, 2.05));

center_of_mass(state)

# This file was generated using Literate.jl, https://github.com/fredrikekre/Literate.jl

