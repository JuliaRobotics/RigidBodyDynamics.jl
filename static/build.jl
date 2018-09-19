using PackageCompiler

builddir = "build"
mkpath(builddir)

build_shared_lib(
    "main.jl", "main",
    snoopfile = "snoopfile.jl",
    startup_file = "no",
    builddir = builddir,
    Release = true,
    check_bounds = "no",
    optimize = 3
)
