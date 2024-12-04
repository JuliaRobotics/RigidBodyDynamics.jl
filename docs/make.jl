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
    if VERSION >= v"1.6"
        push!(excludefiles, "7. Rigorous error bounds using IntervalArithmetic.jl")
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
            stripped_name = replace(name, r"[\s;]" => "")
            postprocess = function(str)
                str = replace(str, "PKG_SETUP" =>
                """
                ```@setup $stripped_name
                import Pkg
                let
                    original_stdout = stdout
                    out_rd, out_wr = redirect_stdout()
                    @async read(out_rd, String)
                    try
                        Pkg.activate("$(relpath(root, outputdir))")
                        Pkg.instantiate()
                    finally
                        redirect_stdout(original_stdout)
                        close(out_wr)
                    end
                end
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
    format = Documenter.HTML(),
    checkdocs = :exports,
    sitename = "RigidBodyDynamics.jl",
    authors = "Twan Koolen and contributors.",
    pages = [
        "Home" => "index.md",
        "Tutorials" => tutorialpages,
        "Library" => [
            "Spatial Vector Algebra" => "spatial.md",
            "Joints" => "joints.md",
            "Rigid Bodies" => "rigidbody.md",
            "Mechanism" => "mechanism.md",
            "Mechanism State" => "mechanismstate.md",
            "Kinematics/Dynamics Algorithms" => "algorithms.md",
            "Custom Collection Types" => "customcollections.md",
            "Cache Types" => "caches.md",
            "Simulation" => "simulation.md",
            "URDF Parsing and Writing" => "urdf.md",
        ],
        "Benchmarks" => "benchmarks.md",
    ],
    warnonly = Documenter.except(),
    repo = Remotes.GitHub("JuliaRobotics", "RigidBodyDynamics.jl"),
)

deploydocs(
    repo="github.com/JuliaRobotics/RigidBodyDynamics.jl",
    devbranch="master",
    push_preview=true,
)
