type DynamicsResult{M, T}
    massMatrix::Symmetric{T, Matrix{T}}
    dynamicsBias::Vector{T}
    constraintJacobian::Matrix{T}
    constraintRhs::Vector{T}
    v̇::Vector{T}
    λ::Vector{T}
    accelerations::Dict{RigidBody{M}, SpatialAcceleration{T}}
    jointWrenches::Dict{RigidBody{M}, Wrench{T}}
    # see solve_dynamics! for meaning of the following variables:
    L::Matrix{T} # lower triangular
    A::Matrix{T} # symmetric
    z::Vector{T}
    Y::Matrix{T}

    function DynamicsResult(::Type{T}, mechanism::Mechanism{M})
        nq = num_positions(mechanism)
        nv = num_velocities(mechanism)
        nNonTree = num_velocities(edge.joint for edge in mechanism.nonTreeEdges)

        massMatrix = Symmetric(Matrix{T}(nv, nv), :L)
        dynamicsBias = Vector{T}(nv)
        constraintJacobian = Matrix{T}(nNonTree, nv)
        constraintRhs = Vector{T}(nNonTree)
        v̇ = Vector{T}(nv)
        λ = Vector{T}(nNonTree)
        accelerations = Dict{RigidBody{M}, SpatialAcceleration{T}}()
        sizehint!(accelerations, num_bodies(mechanism))
        jointWrenches = Dict{RigidBody{M}, Wrench{T}}()
        sizehint!(jointWrenches, num_bodies(mechanism))
        L = Matrix{T}(nv, nv)
        A = Matrix{T}(nNonTree, nNonTree)
        z = Vector{T}(nv)
        Y = Matrix{T}(nNonTree, nv)

        new(massMatrix, dynamicsBias, constraintJacobian, constraintRhs, v̇, λ, accelerations, jointWrenches, L, A, z, Y)
    end
end

DynamicsResult{M, T}(t::Type{T}, mechanism::Mechanism{M}) = DynamicsResult{M, T}(t, mechanism)
