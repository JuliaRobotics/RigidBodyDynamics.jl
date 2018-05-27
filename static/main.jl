module StaticRBD

using UnicodePlots
using RigidBodyDynamics

Base.@ccallable function julia_main(ARGS::Vector{String})::Cint
    mechanism = parse_urdf(Float64, "doublependulum.urdf")
    @show mechanism

    state = MechanismState(mechanism)
    rand!(state)
    ts, qs, vs = simulate(state, 5.0)
    println(lineplot(ts, [q[1] for q in qs]))

    return 0
end

end
