include("main.jl")
using .StaticRBD

config = Dict("doublependulum.urdf" => "test.csv")
for (urdf, csv) in config
    args = [urdf, csv, "1"]
    StaticRBD.inverse_dynamics_benchmark(args)
    StaticRBD.mass_matrix_benchmark(args)
    StaticRBD.dynamics_benchmark(args)
end
