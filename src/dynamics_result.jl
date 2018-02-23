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
    dynamicsbias::Vector{T}
    constraintjacobian::Matrix{T}
    constraintbias::Vector{T}

    v̇::Vector{T}
    λ::Vector{T}
    ṡ::Vector{T}

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

        nconstraints = mapreduce(num_constraints, +, 0, non_tree_joints(mechanism))

        massmatrix = Symmetric(Matrix{T}(nv, nv), :L)
        dynamicsbias = Vector{T}(nv)
        constraintjacobian = Matrix{T}(nconstraints, nv)
        constraintbias = Vector{T}(nconstraints)

        v̇ = Vector{T}(nv)
        λ = Vector{T}(nconstraints)
        ṡ = Vector{T}(num_additional_states(mechanism))

        rootframe = root_frame(mechanism)
        contactwrenches = BodyDict{Wrench{T}}(b => zero(Wrench{T}, rootframe) for b in bodies(mechanism))
        totalwrenches = BodyDict{Wrench{T}}(b => zero(Wrench{T}, rootframe) for b in bodies(mechanism))
        accelerations = BodyDict{SpatialAcceleration{T}}(b => zero(SpatialAcceleration{T}, rootframe, rootframe, rootframe) for b in bodies(mechanism))
        jointwrenches = BodyDict{Wrench{T}}(b => zero(Wrench{T}, rootframe) for b in bodies(mechanism))
        contact_state_derivs = BodyDict{Vector{Vector{DefaultSoftContactStateDeriv{T}}}}(b => Vector{Vector{DefaultSoftContactStateDeriv{T}}}() for b in bodies(mechanism))
        startind = 1
        for body in bodies(mechanism), point in contact_points(body)
            model = contact_model(point)
            n = num_states(model)
            push!(contact_state_derivs[body], collect(begin
                ṡ_part = view(ṡ, startind : startind + n - 1)
                contact_state_deriv = SoftContactStateDeriv(model, ṡ_part, root_frame(mechanism))
                startind += n
                contact_state_deriv
            end for j = 1 : length(mechanism.environment)))
        end

        L = Matrix{T}(nv, nv)
        A = Matrix{T}(nconstraints, nconstraints)
        z = Vector{T}(nv)
        Y = Matrix{T}(nconstraints, nv)

        new{T, M}(mechanism, massmatrix, dynamicsbias, constraintjacobian, constraintbias,
            v̇, λ, ṡ, contactwrenches, totalwrenches, accelerations, jointwrenches, contact_state_derivs,
            L, A, z, Y)
    end
end

DynamicsResult(mechanism::Mechanism{M}) where {M} = DynamicsResult{M}(mechanism)

contact_state_derivatives(result::DynamicsResult, body::Union{<:RigidBody, BodyID}) = result.contact_state_derivatives[body]
contact_wrench(result::DynamicsResult, body::Union{<:RigidBody, BodyID}) = result.contactwrenches[body]
set_contact_wrench!(result::DynamicsResult, body::Union{<:RigidBody, BodyID}, wrench::Wrench) = (result.contactwrenches[body] = wrench)
acceleration(result::DynamicsResult, body::Union{<:RigidBody, BodyID}) = result.accelerations[body]
set_acceleration!(result::DynamicsResult, body::Union{<:RigidBody, BodyID}, accel::SpatialAcceleration) = (result.accelerations[body] = accel)
joint_wrench(result::DynamicsResult, body::Union{<:RigidBody, BodyID}) = result.jointwrenches[body]
set_joint_wrench!(result::DynamicsResult, body::Union{<:RigidBody, BodyID}, wrench::Wrench) = (result.jointwrenches[body] = wrench)
