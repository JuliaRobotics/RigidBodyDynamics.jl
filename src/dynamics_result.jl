"""
$(TYPEDEF)

Stores variables related to the dynamics of a `Mechanism`, e.g. the
`Mechanism`'s mass matrix and joint acceleration vector.

Type parameters:
* `M`: the scalar type of the `Mechanism`.
* `T`: the scalar type of the dynamics-related variables.
"""
type DynamicsResult{M<:Number, T<:Number}
    mechanism::Mechanism{M}

    massmatrix::Symmetric{T, Matrix{T}}
    dynamicsbias::Vector{T}
    constraintjacobian::Matrix{T}
    constraintbias::Vector{T}

    v̇::Vector{T}
    λ::Vector{T}
    ṡ::Vector{T}

    contactwrenches::BodyDict{M, Wrench{T}}
    totalwrenches::BodyDict{M, Wrench{T}}
    accelerations::BodyDict{M, SpatialAcceleration{T}}
    jointwrenches::BodyDict{M, Wrench{T}} # TODO: index by joint tree index?
    contact_state_derivatives::BodyDict{M, Vector{DefaultSoftContactStateDeriv{T}}}

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
        ṡ = Vector{T}(num_additional_states(mechanism))

        rootframe = root_frame(mechanism)
        contactwrenches = BodyDict(b => zero(Wrench{T}, rootframe) for b in bodies(mechanism))
        totalwrenches = BodyDict(b => zero(Wrench{T}, rootframe) for b in bodies(mechanism))
        accelerations = BodyDict(b => zero(SpatialAcceleration{T}, rootframe, rootframe, rootframe) for b in bodies(mechanism))
        jointwrenches = BodyDict(b => zero(Wrench{T}, rootframe) for b in bodies(mechanism))
        contact_state_derivatives = BodyDict(b => Vector{DefaultSoftContactStateDeriv{T}}() for b in bodies(mechanism))
        start = 1
        for body in bodies(mechanism)
            derivs = DefaultSoftContactStateDeriv{T}[]
            for point in contact_points(body)
                model = contact_model(point)
                ṡ_part = view(ṡ, start : start + num_states(model) - 1)
                push!(derivs, SoftContactStateDeriv(model, ṡ_part, root_frame(mechanism)))
                start += num_states(model)
            end
            contact_state_derivatives[body] = derivs
        end

        L = Matrix{T}(nv, nv)
        A = Matrix{T}(nconstraints, nconstraints)
        z = Vector{T}(nv)
        Y = Matrix{T}(nconstraints, nv)

        new{M, T}(mechanism, massmatrix, dynamicsbias, constraintjacobian, constraintbias,
            v̇, λ, ṡ, contactwrenches, totalwrenches, accelerations, jointwrenches, contact_state_derivatives,
            L, A, z, Y)
    end
end

DynamicsResult{M, T}(::Type{T}, mechanism::Mechanism{M}) = DynamicsResult{M, T}(mechanism)

contact_state_derivatives(result::DynamicsResult, body::RigidBody) = result.contact_state_derivatives[body]
contact_wrench(result::DynamicsResult, body::RigidBody) = result.contactwrenches[body]
set_contact_wrench!(result::DynamicsResult, body::RigidBody, wrench::Wrench) = (result.contactwrenches[body] = wrench)
acceleration(result::DynamicsResult, body::RigidBody) = result.accelerations[body]
set_acceleration!(result::DynamicsResult, body::RigidBody, accel::SpatialAcceleration) = (result.accelerations[body] = accel)
joint_wrench(result::DynamicsResult, body::RigidBody) = result.jointwrenches[body]
set_joint_wrench!(result::DynamicsResult, body::RigidBody, wrench::Wrench) = (result.jointwrenches[body] = wrench)
