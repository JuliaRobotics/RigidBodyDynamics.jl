include("main.jl")
using .StaticRBD

config = Dict("doublependulum.urdf" => "test.csv")
for (urdf, csv) in config
    # args = [urdf, csv, "1"]
    GC.@preserve urdf csv begin
        c_urdf = Base.unsafe_convert(Cstring, urdf)
        c_csv = Base.unsafe_convert(Cstring, csv)
        c_floating = Cint(false) # TODO
        mechanism = StaticRBD.create_mechanism(c_urdf, c_floating)
        # StaticRBD.inverse_dynamics_benchmark(args)
        # StaticRBD.mass_matrix_benchmark(args)
        # StaticRBD.dynamics_benchmark(args)
    end
end
