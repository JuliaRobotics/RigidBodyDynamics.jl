immutable ButcherTableau{N, T}
    a::SMatrix{N, N, T}
    b::SVector{N, T}
    c::SVector{N, T}
    explicit::Bool

    function ButcherTableau(a::AbstractMatrix{T}, b::AbstractVector{T})
        @assert size(a, 1) == N
        @assert size(a, 2) == N
        @assert length(b) == N
        c = vec(sum(a, 2))
        explicit = all(triu(a) .== 0)
        new(SMatrix{N, N}(a), SVector{N}(b), SVector{N}(c), explicit)
    end
end

ButcherTableau{T}(a::Matrix{T}, b::Vector{T}) = ButcherTableau{length(b), T}(a, b)

num_stages{N, T}(::ButcherTableau{N, T}) = N
isexplicit(tableau::ButcherTableau) = tableau.explicit

function runge_kutta_4{T}(::Type{T})
    a = zeros(T, 4, 4)
    a[2, 1] = 1/2
    a[3, 2] = 1/2
    a[4, 3] = 1/2
    b = T[1/6, 1/3, 1/3, 1/6]
    ButcherTableau{4, Float64}(a, b)
end

immutable MuntheKaasIntegrator{X, M, C, N, E}
    state::MechanismState{X, M, C}
    effortSource::E
    result::DynamicsResult{C}
    tableau::ButcherTableau{N, C}

    # TODO: extract out: integrator state
    q0::Vector{C}
    vstage::Vector{Vector{C}}
    v̇stage::Vector{Vector{C}}
    ϕstage::Vector{Vector{C}}
    ϕ̇stage::Vector{Vector{C}}

    # TODO: extract out: integrator output
    tout::Vector{C}
    qout::Vector{Vector{C}}
    vout::Vector{Vector{C}}

    function MuntheKaasIntegrator(state::MechanismState{X, M, C}, effortSource::E,
            result::DynamicsResult{C}, tableau::ButcherTableau{N, C})
        @assert isexplicit(tableau)
        n = num_stages(tableau)
        @assert n > 0

        q0 = Vector{C}(num_positions(state))
        vstage = [Vector{C}(num_velocities(state)) for i in 1 : n]
        v̇stage = [Vector{C}(num_velocities(state)) for i in 1 : n]
        ϕstage = [Vector{C}(num_velocities(state)) for i in 1 : n]
        ϕ̇stage = [Vector{C}(num_velocities(state)) for i in 1 : n]

        tout = Vector{C}()
        qout = Vector{C}()
        qout = Vector{Vector{C}}()
        vout = Vector{Vector{C}}()
        new(state, effortSource, result, tableau, q0, vstage, v̇stage, ϕstage, ϕ̇stage, tout, qout, vout)
    end
end

function MuntheKaasIntegrator{X, M, C, N, E}(state::MechanismState{X, M, C}, effortSource::E, result::DynamicsResult{C}, tableau::ButcherTableau{N, C})
    MuntheKaasIntegrator{X, M, C, N, E}(state, effortSource, result, tableau)
end

function step(integrator::MuntheKaasIntegrator, t::Real, Δt::Real)
    tableau = integrator.tableau
    q0 = integrator.q0
    n = num_stages(integrator.tableau)
    state = integrator.state
    result = integrator.result

    # Use current configuration as the configuration around which the local coordinates for this step will be centered.
    copy!(integrator.q0, configuration_vector(state))

    # Compute integrator stages.
    for i = 1 : n
        # Update local coordinates and velocities
        ϕ = integrator.ϕstage[i]
        v = integrator.vstage[i]
        fill!(ϕ, 0)
        copy!(v, velocity_vector(state))
        for j = 1 : i - 1
            aij = tableau.a[i, j]
            if aij != zero(aij)
                ϕ̇ = integrator.ϕ̇stage[j]
                v̇ = integrator.vstage[j]
                weight = Δt * aij
                scaleadd!(ϕ, ϕ̇, weight)
                scaleadd!(v, v̇, weight)
            end
        end

        # Convert from local to global coordinates and set state
        global_coordinates!(state, q0, ϕ)
        set_velocity!(state, v)

        # Poll for efforts
        torques, externalWrenches = integrator.effortSource(t + tableau.c[i] * Δt, state)

        # Dynamics in global coordinates
        dynamics!(integrator.result, integrator.state, torques, externalWrenches)

        # Convert back to local coordinates and set v̇
        ϕ̇ = integrator.ϕ̇stage[i]
        v̇ = integrator.v̇stage[i]
        a = rand(length(ϕ)) # FIXME
        local_coordinates!(state, a, ϕ̇, q0)
        copy!(v̇, result.v̇)
    end

    # Combine stages (store in vector for first step)
    ϕ = integrator.ϕstage[1]
    v = integrator.vstage[1]
    scale!(ϕ, tableau.b[1] * Δt)
    scale!(v, tableau.b[1] * Δt)
    for i = 2 : n
        scaleadd!(ϕ, integrator.ϕstage[i], tableau.b[i] * Δt)
        scaleadd!(v, integrator.vstage[i], tableau.b[i] * Δt)
    end

    # Convert from local to global coordinates
    global_coordinates!(state, q0, ϕ)
end
