let
    num_notebooks_tested = 0
    notebookdir = joinpath(@__DIR__, "..", "examples")
    excludedirs = [".ipynb_checkpoints"]
    excludefiles = String[]
    if VERSION < v"1.1.0"
        push!(excludefiles, "Symbolic double pendulum.ipynb")
    end
    for (root, dir, files) in walkdir(notebookdir)
        basename(root) in excludedirs && continue
        for file in files
            file in excludefiles && continue
            name, ext = splitext(file)
            lowercase(ext) == ".ipynb" || continue
            path = joinpath(root, file)
            @eval module $(gensym()) # Each notebook is run in its own module.
            using Test
            using NBInclude
            @testset "Notebook: $($name)" begin
                # Note: use #NBSKIP in a cell to skip it during tests.
                @nbinclude($path; regex = r"^((?!\#NBSKIP).)*$"s)
            end
            end # module
            num_notebooks_tested += 1
        end
    end
    @test num_notebooks_tested > 0
end
