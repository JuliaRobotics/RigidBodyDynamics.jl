"""
$(TYPEDEF)

Stores variables related to the dynamics of a `Mechanism`, e.g. the
`Mechanism`'s mass matrix and joint acceleration vector.

Type parameters:
* `M`: the scalar type of the `Mechanism`.
* `T`: the scalar type of the dynamics-related variables.
"""
type DynamicsResult{M<:Number, T<:Number}
    massMatrix::Symmetric{T, Matrix{T}}
    dynamicsBias::Vector{T}
    constraintJacobian::Matrix{T}
    constraintBias::Vector{T}
    v̇::Vector{T}
    λ::Vector{T}
    accelerations::Dict{RigidBody{M}, SpatialAcceleration{T}}
    jointWrenches::Dict{RigidBody{M}, Wrench{T}}
    # see solve_dynamics! for meaning of the following variables:
    L::Matrix{T} # lower triangular
    A::Matrix{T} # symmetric
    z::Vector{T}
    Y::Matrix{T}

    function DynamicsResult(mechanism::Mechanism{M})
        nq = num_positions(mechanism)
        nv = num_velocities(mechanism)
        nconstraints = isempty(mechanism.nonTreeEdges)? 0 : sum(num_constraints, edge.joint for edge in mechanism.nonTreeEdges)

        massMatrix = Symmetric(Matrix{T}(nv, nv), :L)
        dynamicsBias = Vector{T}(nv)
        constraintJacobian = Matrix{T}(nconstraints, nv)
        constraintBias = Vector{T}(nconstraints)
        v̇ = Vector{T}(nv)
        λ = Vector{T}(nconstraints)
        accelerations = Dict{RigidBody{M}, SpatialAcceleration{T}}()
        sizehint!(accelerations, num_bodies(mechanism))
        jointWrenches = Dict{RigidBody{M}, Wrench{T}}()
        sizehint!(jointWrenches, num_bodies(mechanism))
        L = Matrix{T}(nv, nv)
        A = Matrix{T}(nconstraints, nconstraints)
        z = Vector{T}(nv)
        Y = Matrix{T}(nconstraints, nv)

        new(massMatrix, dynamicsBias, constraintJacobian, constraintBias, v̇, λ, accelerations, jointWrenches, L, A, z, Y)
    end
end

DynamicsResult{M, T}(::Type{T}, mechanism::Mechanism{M}) = DynamicsResult{M, T}(mechanism)
