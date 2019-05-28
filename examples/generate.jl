using Literate

exampledir = @__DIR__
for subdir in readdir(exampledir)
    root = joinpath(exampledir, subdir)
    isdir(root) || continue
    @show subdir
    for file in readdir(root)
        name, ext = splitext(file)
        lowercase(ext) == ".jl" || continue
        absfile = joinpath(root, file)
        @show absfile
        Literate.notebook(absfile, root, execute=false)
        # Literate.markdown(absfile, root)
        # Literate.script(absfile, root)
    end
end
