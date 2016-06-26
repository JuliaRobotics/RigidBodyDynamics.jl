type DynamicsResults{M, T}
    num_positions::Int64
    num_velocities::Int64
    massMatrix::Symmetric{T, Matrix{T}}
    ẋ::Vector{T}
    q̇::AbstractVector{T}
    v̇::AbstractVector{T}
    τ::Vector{T}
    accelerations::Dict{RigidBody{M}, SpatialAcceleration{T}}
    jointWrenches::Dict{RigidBody{M}, Wrench{T}}

    function DynamicsResults(::Type{T}, mechanism::Mechanism{M})
        nq = num_positions(mechanism)
        nv = num_velocities(mechanism)
        massMatrix = Symmetric(zeros(T, nv, nv))
        ẋ = zeros(T, nq + nv)
        q̇ = view(ẋ, 1 : nq)
        v̇ = view(ẋ, nq + 1 : nq + nv)
        τ =  zeros(T, nv)
        accelerations = Dict{RigidBody{M}, SpatialAcceleration{T}}()
        sizehint!(accelerations, length(bodies(mechanism)))
        jointWrenches = Dict{RigidBody{M}, Wrench{T}}()
        sizehint!(jointWrenches, length(bodies(mechanism)))
        new(nq, nv, massMatrix, ẋ, q̇, v̇, τ, accelerations, jointWrenches)
    end
end

DynamicsResults{M, T}(t::Type{T}, mechanism::Mechanism{M}) = DynamicsResults{M, T}(t, mechanism)
