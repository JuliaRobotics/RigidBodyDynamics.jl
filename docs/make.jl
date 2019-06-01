using Documenter
using Literate
using RigidBodyDynamics, RigidBodyDynamics.OdeIntegrators

gendir = joinpath(@__DIR__, "src", "generated")
tutorialpages = String[]

let
    rm(gendir, recursive=true, force=true)
    mkdir(gendir)
    exampledir = joinpath(@__DIR__, "..", "examples")
    excludedirs = String[]
    excludefiles = String[]
    if VERSION < v"1.1.0"
        push!(excludefiles, "Symbolic double pendulum.jl")
    end
    for subdir in readdir(exampledir)
        subdir in excludedirs && continue
        root = joinpath(exampledir, subdir)
        isdir(root) || continue
        for file in readdir(root)
            file in excludefiles && continue
            name, ext = splitext(file)
            lowercase(ext) == ".jl" || continue
            outputdir = joinpath(gendir, subdir)
            cp(root, outputdir)
            preprocess = function (str)
                str = replace(str, "@__DIR__" => "\"$(relpath(root, outputdir))\"")
                str = replace(str, "# PREAMBLE" =>
                """
                # This example is also available as a Jupyter notebook that can be run locally
                # The notebook can be found in the `examples` directory of the package.
                # If the notebooks are missing, you may need to run `using Pkg; Pkg.build()`.
                """)
                return str
            end
            postprocess = function(str)
                str = replace(str, "PKG_SETUP" =>
                """
                ```@setup $name
                using Pkg
                Pkg.activate("$(relpath(root, outputdir))")
                Pkg.instantiate()
                ```
                """)
                return str
            end

            absfile = joinpath(root, file)
            tutorialpage = Literate.markdown(absfile, outputdir; preprocess=preprocess, postprocess=postprocess)
            push!(tutorialpages, relpath(tutorialpage, joinpath(@__DIR__, "src")))
        end
    end
end

makedocs(
    modules = [RigidBodyDynamics, RigidBodyDynamics.OdeIntegrators],
    root = @__DIR__,
    checkdocs = :exports,
    sitename ="RigidBodyDynamics.jl",
    authors = "Twan Koolen and contributors.",
    pages = [
        "Home" => "index.md",
        "Tutorials" => tutorialpages,
        "Library" => [
            "Spatial vector algebra" => "spatial.md",
            "Joints" => "joints.md",
            "Rigid bodies" => "rigidbody.md",
            "Mechanism" => "mechanism.md",
            "MechanismState" => "mechanismstate.md",
            "Kinematics/dynamics algorithms" => "algorithms.md",
            "Custom collection types" => "customcollections.md",
            "Cache types" => "caches.md",
            "Simulation" => "simulation.md",
            "URDF parsing and writing" => "urdf.md",
        ],
        "Benchmarks" => "benchmarks.md",
      ],
    format = Documenter.HTML(prettyurls = parse(Bool, get(ENV, "CI", "false")))
)

deploydocs(
    repo = "github.com/JuliaRobotics/RigidBodyDynamics.jl.git"
)
