using PackageCompiler

builddir = "build"
mkpath(builddir)

build_executable(
    "main.jl",
    snoopfile = "snoopfile.jl",
    startup_file = "no",
    builddir = builddir,
    Release = true,
    check_bounds = "no",
    optimize = 3
)
