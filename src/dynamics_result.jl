"""
$(TYPEDEF)

Stores variables related to the dynamics of a `Mechanism`, e.g. the
`Mechanism`'s mass matrix and joint acceleration vector.

Type parameters:
* `T`: the scalar type of the dynamics-related variables.
* `M`: the scalar type of the `Mechanism`.
"""
mutable struct DynamicsResult{T, M}
    mechanism::Mechanism{M}

    massmatrix::Symmetric{T, Matrix{T}}
    dynamicsbias::SegmentedVector{JointID, T, Base.OneTo{JointID}, Vector{T}}
    constraintjacobian::Matrix{T}
    constraintbias::SegmentedVector{JointID, T, UnitRange{JointID}, Vector{T}}
    constraintrowranges::IndexDict{JointID, UnitRange{JointID}, UnitRange{Int}}

    q̇::SegmentedVector{JointID, T, Base.OneTo{JointID}, Vector{T}}
    v̇::SegmentedVector{JointID, T, Base.OneTo{JointID}, Vector{T}}
    λ::Vector{T}

    accelerations::BodyDict{SpatialAcceleration{T}}
    jointwrenches::BodyDict{Wrench{T}} # TODO: index by joint tree index?

    # see solve_dynamics! for meaning of the following variables:
    L::Matrix{T} # lower triangular
    A::Matrix{T} # symmetric
    z::Vector{T}
    Y::Matrix{T}

    function DynamicsResult{T}(mechanism::Mechanism{M}) where {T, M}
        nq = num_positions(mechanism)
        nv = num_velocities(mechanism)
        nc = num_constraints(mechanism)

        massmatrix = Symmetric(Matrix{T}(undef, nv, nv), :L)
        dynamicsbias = SegmentedVector(Vector{T}(undef, nv), tree_joints(mechanism), num_velocities)
        constraintjacobian = Matrix{T}(undef, nc, nv)
        constraintbias = SegmentedVector{JointID, T, UnitRange{JointID}}(
            Vector{T}(undef, nc), non_tree_joints(mechanism), num_constraints)
        constraintrowranges = ranges(constraintbias)

        q̇ = SegmentedVector(Vector{T}(undef, nq), tree_joints(mechanism), num_positions)
        v̇ = SegmentedVector(Vector{T}(undef, nv), tree_joints(mechanism), num_velocities)
        λ = Vector{T}(undef, nc)

        rootframe = root_frame(mechanism)
        accelerations = BodyDict{SpatialAcceleration{T}}(
            b => zero(SpatialAcceleration{T}, rootframe, rootframe, rootframe) for b in bodies(mechanism))
        jointwrenches = BodyDict{Wrench{T}}(b => zero(Wrench{T}, rootframe) for b in bodies(mechanism))

        L = Matrix{T}(undef, nv, nv)
        A = Matrix{T}(undef, nc, nc)
        z = Vector{T}(undef, nv)
        Y = Matrix{T}(undef, nc, nv)

        new{T, M}(mechanism, massmatrix, dynamicsbias, constraintjacobian, constraintbias, constraintrowranges,
            q̇, v̇, λ, accelerations, jointwrenches,
            L, A, z, Y)
    end
end

DynamicsResult(mechanism::Mechanism{M}) where {M} = DynamicsResult{M}(mechanism)

function Base.copyto!(ẋ::AbstractVector, result::DynamicsResult)
    nq = length(result.q̇)
    nv = length(result.v̇)
    @boundscheck length(ẋ) == nq + nv || throw(DimensionMismatch())
    @inbounds copyto!(ẋ, 1, result.q̇, 1, nq)
    @inbounds copyto!(ẋ, nq + 1, result.v̇, 1, nv)
    ẋ
end

acceleration(result::DynamicsResult, body::Union{<:RigidBody, BodyID}) =
    result.accelerations[body]
set_acceleration!(result::DynamicsResult, body::Union{<:RigidBody, BodyID}, accel::SpatialAcceleration) =
    (result.accelerations[body] = accel)
joint_wrench(result::DynamicsResult, body::Union{<:RigidBody, BodyID}) =
    result.jointwrenches[body]
set_joint_wrench!(result::DynamicsResult, body::Union{<:RigidBody, BodyID}, wrench::Wrench) =
    (result.jointwrenches[body] = wrench)
