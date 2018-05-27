using PackageCompiler

builddir = "build"
mkpath(builddir)

build_executable(
    "main.jl",
    snoopfile = "snoopfile.jl",
    builddir = builddir,
    check_bounds = "no",
    optimize = 3
)
