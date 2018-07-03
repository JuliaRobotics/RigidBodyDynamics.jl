@testset "example notebooks" begin
    notebookdir = joinpath(@__DIR__, "..", "notebooks")
    excludes = String[]
    for file in readdir(notebookdir)
        path = joinpath(notebookdir, file)
        path in excludes && continue
        name, ext = splitext(file)
        lowercase(ext) == ".ipynb" || continue

        @eval module $(gensym()) # Each notebook is run in its own module.
        using Test
        using NBInclude
        @testset "$($name)" begin
            @nbinclude($path; regex = r"^((?!\#NBSKIP).)*$"s) # Use #NBSKIP in a cell to skip it during tests.
        end
        end # module
    end
end
