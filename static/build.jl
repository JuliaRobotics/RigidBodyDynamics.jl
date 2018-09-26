using PackageCompiler

builddir = "build"
mkpath(builddir)

build_shared_lib(
    "staticrbd.jl", "staticrbdjl",
    snoopfile = "snoopfile.jl",
    startup_file = "no",
    builddir = builddir,
    Release = true,
    check_bounds = "no",
    optimize = 3
)
