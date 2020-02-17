struct ViscoelasticCoulombModel{T}
    # See section 11.8 of Featherstone, "Rigid Body Dynamics Algorithms", 2008
    μ::T
    k::T
    b::T
end

 # Use a 3-vector; technically only need one state for each tangential direction, but this is easier to work with.
num_states(::ViscoelasticCoulombModel) = 3
zero_state(::ViscoelasticCoulombModel{T}) where {T} = zero(SVector{3, T})
devectorize(::ViscoelasticCoulombModel, x::AbstractVector) = SVector{3}(x)

function tangential_contact_dynamics(model::ViscoelasticCoulombModel, state::SVector{3}, fnormal::Number, tangential_velocity::SVector{3})
    T = typeof(fnormal)
    μ = model.μ
    k = model.k
    b = model.b
    x = state
    v = tangential_velocity

    # compute friction force that would be needed to avoid slip
    fstick = -k * x - b * v

    # limit friction force to lie within friction cone
    fstick_norm² = dot(fstick, fstick)
    fstick_max_norm² = (μ * fnormal)^2
    ftangential = if fstick_norm² > fstick_max_norm²
        fstick * sqrt(fstick_max_norm² / fstick_norm²)
    else
        fstick
    end

    # compute contact state derivative
    ẋ = (-k * x - ftangential) ./ b

    ftangential, ẋ
end
