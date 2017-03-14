"""
$(TYPEDEF)

Stores variables related to the dynamics of a `Mechanism`, e.g. the
`Mechanism`'s mass matrix and joint acceleration vector.

Type parameters:
* `M`: the scalar type of the `Mechanism`.
* `T`: the scalar type of the dynamics-related variables.
"""
type DynamicsResult{M<:Number, T<:Number}
    massmatrix::Symmetric{T, Matrix{T}}
    dynamicsbias::Vector{T}
    constraintjacobian::Matrix{T}
    constraintbias::Vector{T}
    v̇::Vector{T}
    λ::Vector{T}
    accelerations::Dict{RigidBody{M}, SpatialAcceleration{T}}
    jointwrenches::Dict{RigidBody{M}, Wrench{T}}
    # see solve_dynamics! for meaning of the following variables:
    L::Matrix{T} # lower triangular
    A::Matrix{T} # symmetric
    z::Vector{T}
    Y::Matrix{T}

    function (::Type{DynamicsResult{M, T}}){M<:Number, T<:Number}(mechanism::Mechanism{M})
        nq = num_positions(mechanism)
        nv = num_velocities(mechanism)

        nconstraints = mapreduce(num_constraints, +, 0, non_tree_joints(mechanism))

        massmatrix = Symmetric(Matrix{T}(nv, nv), :L)
        dynamicsbias = Vector{T}(nv)
        constraintjacobian = Matrix{T}(nconstraints, nv)
        constraintbias = Vector{T}(nconstraints)
        v̇ = Vector{T}(nv)
        λ = Vector{T}(nconstraints)
        accelerations = Dict{RigidBody{M}, SpatialAcceleration{T}}()
        sizehint!(accelerations, num_bodies(mechanism))
        jointwrenches = Dict{RigidBody{M}, Wrench{T}}()
        sizehint!(jointwrenches, num_bodies(mechanism))
        L = Matrix{T}(nv, nv)
        A = Matrix{T}(nconstraints, nconstraints)
        z = Vector{T}(nv)
        Y = Matrix{T}(nconstraints, nv)

        new{M, T}(massmatrix, dynamicsbias, constraintjacobian, constraintbias, v̇, λ, accelerations, jointwrenches, L, A, z, Y)
    end
end

DynamicsResult{M, T}(::Type{T}, mechanism::Mechanism{M}) = DynamicsResult{M, T}(mechanism)
