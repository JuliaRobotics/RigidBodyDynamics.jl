include("staticrbd.jl")
using .StaticRBD
using RigidBodyDynamics

config = [("doublependulum.urdf", "test.csv", false)]
for (urdf, csv, floating) in config
    c_floating = Cint(floating)
    mechanism = GC.@preserve urdf csv begin
        c_urdf = Base.unsafe_convert(Cstring, urdf)
        c_csv = Base.unsafe_convert(Cstring, csv)
        StaticRBD.create_mechanism(c_urdf, c_floating)
    end
    state = StaticRBD.create_state(mechanism)
    result = StaticRBD.create_dynamics_result(mechanism)

    jointwrenches = result.jointwrenches
    accelerations = result.accelerations
    M = result.massmatrix
    vd = result.v̇

    q = configuration(state)
    v = velocity(state)
    vd_desired = similar(v)
    tau = similar(v)

    StaticRBD.inverse_dynamics(tau, jointwrenches, accelerations, state, vd_desired)
    StaticRBD.mass_matrix(M, state)
    StaticRBD.dynamics(result, state, tau)
end
