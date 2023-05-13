let
    num_notebooks_tested = 0
    notebookdir = joinpath(@__DIR__, "..", "examples")
    excludedirs = [".ipynb_checkpoints"]
    excludefiles = String[]
    push!(excludefiles, "6. Symbolics.ipynb")  # Disabled until https://github.com/JuliaGeometry/Quaternions.jl/issues/123 is solved.
    # push!(excludefiles, "7. Rigorous error bounds using IntervalArithmetic.ipynb") # Manifest used for 1.1 doesn't work for 1.0.
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
