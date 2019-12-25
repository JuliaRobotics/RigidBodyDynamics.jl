using Pkg

for x in readdir()
    if isdir(x) && isfile(joinpath(x, "Project.toml"))
        Pkg.activate(x)
        Pkg.update()
    end
end
