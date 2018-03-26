# RigidBodyDynamics.jl example notebooks

This directory contains Jupyter notebooks that demonstrate various aspects of RigidBodyDynamics.jl.

You can view (but not run) them [online using nbviewer](http://nbviewer.jupyter.org/github/JuliaRobotics/RigidBodyDynamics.jl/tree/master/notebooks/). Note that the notebooks may not render correctly in Github itself (you may get a message saying "Sorry, something went wrong. Reload?").

You can also run the notebooks locally by performing the following steps:

1. [install RigidBodyDynamics.jl](http://www.juliarobotics.org/RigidBodyDynamics.jl/stable/#Installation-1)
2. [install the test dependencies](https://github.com/JuliaRobotics/RigidBodyDynamics.jl/blob/master/test/REQUIRE) in similar fashion.
3. [install IJulia](https://github.com/JuliaLang/IJulia.jl)
4. in the Julia REPL, run
   ```
   using IJulia; notebook(dir=Pkg.dir("RigidBodyDynamics", "notebooks"))
   ```