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
    ṡ::Vector{T}
    λ::Vector{T}

    contactwrenches::BodyDict{Wrench{T}}
    totalwrenches::BodyDict{Wrench{T}}
    accelerations::BodyDict{SpatialAcceleration{T}}
    jointwrenches::BodyDict{Wrench{T}} # TODO: index by joint tree index?
    contact_state_derivatives::BodyDict{Vector{Vector{DefaultSoftContactStateDeriv{T}}}}

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
        ṡ = Vector{T}(undef, num_additional_states(mechanism))
        λ = Vector{T}(undef, nc)

        rootframe = root_frame(mechanism)
        contactwrenches = BodyDict{Wrench{T}}(b => zero(Wrench{T}, rootframe) for b in bodies(mechanism))
        totalwrenches = BodyDict{Wrench{T}}(b => zero(Wrench{T}, rootframe) for b in bodies(mechanism))
        accelerations = BodyDict{SpatialAcceleration{T}}(
            b => zero(SpatialAcceleration{T}, rootframe, rootframe, rootframe) for b in bodies(mechanism))
        jointwrenches = BodyDict{Wrench{T}}(b => zero(Wrench{T}, rootframe) for b in bodies(mechanism))
        contact_state_derivs = BodyDict{Vector{Vector{DefaultSoftContactStateDeriv{T}}}}(
            b => Vector{Vector{DefaultSoftContactStateDeriv{T}}}() for b in bodies(mechanism))
        startind = Ref(1)
        for body in bodies(mechanism)
            for point::DefaultContactPoint{M} in contact_points(body)
                model = contact_model(point)
                n = num_states(model)
                push!(contact_state_derivs[body], collect(begin
                    ṡ_part = view(ṡ, startind[] : startind[] + n - 1)
                    contact_state_deriv = SoftContactStateDeriv(model, ṡ_part, root_frame(mechanism))
                    startind[] += n
                    contact_state_deriv
                end for j = 1 : length(mechanism.environment)))
            end
        end

        L = Matrix{T}(undef, nv, nv)
        A = Matrix{T}(undef, nc, nc)
        z = Vector{T}(undef, nv)
        Y = Matrix{T}(undef, nc, nv)

        new{T, M}(mechanism, massmatrix, dynamicsbias, constraintjacobian, constraintbias, constraintrowranges,
            q̇, v̇, ṡ, λ, contactwrenches, totalwrenches, accelerations, jointwrenches, contact_state_derivs,
            L, A, z, Y)
    end
end

DynamicsResult(mechanism::Mechanism{M}) where {M} = DynamicsResult{M}(mechanism)

function Base.copyto!(ẋ::AbstractVector, result::DynamicsResult)
    nq = length(result.q̇)
    nv = length(result.v̇)
    ns = length(result.ṡ)
    @boundscheck length(ẋ) == nq + nv + ns || throw(DimensionMismatch())
    @inbounds copyto!(ẋ, 1, result.q̇, 1, nq)
    @inbounds copyto!(ẋ, nq + 1, result.v̇, 1, nv)
    @inbounds copyto!(ẋ, nq + nv + 1, result.ṡ, 1, ns)
    ẋ
end

contact_state_derivatives(result::DynamicsResult, body::Union{<:RigidBody, BodyID}) =
    result.contact_state_derivatives[body]
contact_wrench(result::DynamicsResult, body::Union{<:RigidBody, BodyID}) =
    result.contactwrenches[body]
set_contact_wrench!(result::DynamicsResult, body::Union{<:RigidBody, BodyID}, wrench::Wrench) =
    (result.contactwrenches[body] = wrench)
acceleration(result::DynamicsResult, body::Union{<:RigidBody, BodyID}) =
    result.accelerations[body]
set_acceleration!(result::DynamicsResult, body::Union{<:RigidBody, BodyID}, accel::SpatialAcceleration) =
    (result.accelerations[body] = accel)
joint_wrench(result::DynamicsResult, body::Union{<:RigidBody, BodyID}) =
    result.jointwrenches[body]
set_joint_wrench!(result::DynamicsResult, body::Union{<:RigidBody, BodyID}, wrench::Wrench) =
    (result.jointwrenches[body] = wrench)
