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
    for (root, _, files) in walkdir(exampledir)
        basename(root) in excludedirs && continue
        for file in files
            file in excludefiles && continue
            name, ext = splitext(file)
            lowercase(ext) == ".jl" || continue
            outputdir = joinpath(gendir, basename(root))
            cp(root, outputdir)
            preprocess = function (str)
                str = replace(str, "OPEN_VISUALIZER = true" => "OPEN_VISUALIZER = false")
                str = replace(str, "@__DIR__" => "\"$root\"")
                return str
            end
            absfile = joinpath(root, file)
            Literate.script(absfile, outputdir; preprocess=preprocess)
            Literate.notebook(absfile, outputdir; preprocess=preprocess)
            tutorialpage = Literate.markdown(absfile, outputdir; preprocess=preprocess)
            push!(tutorialpages, relpath(tutorialpage, joinpath(@__DIR__, "src")))

            # replace the link in the markdown file
            travis_tag = get(ENV, "TRAVIS_TAG", "")
            folder = isempty(travis_tag) ? "latest" : travis_tag
            url = "https://nbviewer.jupyter.org/github/JuliaRobotics/RigidBodyDynamics.jl/blob/gh-pages/$(folder)/"
            if get(ENV, "CI", "") == "true"
                str = read(tutorialpage, String)
                str = replace(str, "[$name.ipynb](generated/$name.ipynb)." => "[$name.ipynb]($(url)generated/$name.ipynb).")
                write(tutorialpage, str)
            end
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
