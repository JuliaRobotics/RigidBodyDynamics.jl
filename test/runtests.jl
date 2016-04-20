using Base.Test

using Quaternions
using FixedSizeArrays
import RigidBodyDynamics

using IJulia
jupyter = IJulia.jupyter
for f in filter(x -> endswith(x, "ipynb"), readdir("../examples"))
    notebook = "../examples/" * f
    run(`$jupyter nbconvert --to notebook --execute $notebook --output $notebook`)
end
