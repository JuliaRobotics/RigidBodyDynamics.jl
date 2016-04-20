using Base.Test

using Quaternions
using FixedSizeArrays
import RigidBodyDynamics

for f in filter(x -> endswith(x, "ipynb"), readdir("../examples"))
    notebook = "../examples/" * f
    run(`jupyter nbconvert --to notebook --execute $notebook --output $notebook`)
end
