using Base.Test

using Quaternions
using FixedSizeArrays
import RigidBodyDynamics

import Conda
condaDir = Conda.SCRIPTDIR
for f in filter(x -> endswith(x, "ipynb"), readdir("../examples"))
    notebook = "../examples/" * f
    run(`export PATH=$condaDir:\$PATH && jupyter nbconvert --to notebook --execute $notebook --output $notebook`)
end
