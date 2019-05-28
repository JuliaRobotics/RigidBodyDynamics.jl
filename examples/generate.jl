using Literate

for dir in filter(isdir, readdir(@__DIR__))
    for file in readdir(dir)
        name, ext = splitext(file)
        lowercase(ext) == ".jl" || continue
        absfile = joinpath(dir, file)
        Literate.notebook(absfile, dir, execute=false)
        # Literate.markdown(absfile, dir)
        # Literate.script(absfile, dir)
    end
end
