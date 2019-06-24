struct HuntCrossleyModel{T}
    # (2) in Marhefka, Orin, "A Compliant Contact Model with Nonlinear Damping for Simulation of Robotic Systems"
    k::T
    λ::T
    n::T
end

function hunt_crossley_hertz(; k = 50e3, α = 0.2)
    λ = 3/2 * α * k # (12) in Marhefka, Orin
    HuntCrossleyModel(k, λ, 3/2)
end

num_states(::HuntCrossleyModel) = 0
zero_state(::HuntCrossleyModel, ::CartesianFrame3D) = nothing
zero_state_deriv(::HuntCrossleyModel, ::CartesianFrame3D) = nothing

function contact_dynamics(model::HuntCrossleyModel, ::Nothing, z, ż)
    zn = z^model.n
    f = max(model.λ * zn * ż + model.k * zn, 0) # (2) in Marhefka, Orin (note: z is penetration, returning repelling force)
    f, nothing
end
