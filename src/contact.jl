module Contact

using RigidBodyDynamics
using StaticArrays
using Compat

export NormalForceModel,
    FrictionModel,
    SoftContactModel,
    ContactPoint,
    DefaultContactPoint

export normal_force,
    friction_force,
    contact_model,
    normal_force_model,
    friction_model,
    num_states,
    reset!,
    dynamics!,
    contact_dynamics!

export HuntCrossleyModel,
    hunt_crossley_hertz

export
    ViscoelasticCoulombModel,
    ViscoelasticCoulombState

# Types
@compat abstract type NormalForceModel{T} end
normal_force(::NormalForceModel, z, ż) = error("Subtypes must implement")

@compat abstract type FrictionModel{T} end
# TODO: FrictionModel interface

immutable SoftContactModel{N <: NormalForceModel, F <: FrictionModel}
    normal_force_model::N
    friction_model::F
end
normal_force_model(model::SoftContactModel) = model.normal_force_model
friction_model(model::SoftContactModel) = model.friction_model

type ContactPoint{T, M <: SoftContactModel}
    location::Point3D{SVector{3, T}}
    model::M
end
contact_model(point::ContactPoint) = point.model

# Normal contact models
immutable HuntCrossleyModel{T} <: NormalForceModel{T}
    # (2) in Marhefka, Orin, "A Compliant Contact Model with Nonlinear Damping for Simulation of Robotic Systems"
    k::T
    λ::T
    n::T
end

function hunt_crossley_hertz(; k = 50e3, α = 0.2)
    λ = 3/2 * α * k # (12) in Marhefka, Orin
    HuntCrossleyModel(k, λ, 3/2)
end

function normal_force(model::HuntCrossleyModel, z, ż)
    zn = z^model.n
    f = model.λ * zn * ż + model.k * zn # (2) in Marhefka, Orin (note: z is penetration, returning repelling force)
end

# Friction models
immutable ViscoelasticCoulombModel{T} <: FrictionModel{T}
    # See section 11.8 of Featherstone, "Rigid Body Dynamics Algorithms", 2008
    μ::T
    k::T
    b::T
end

# One state for each direction; technically only need one for each tangential direction, but this is easier to work with:
num_states(::ViscoelasticCoulombModel) = 3

type ViscoelasticCoulombState{T, V}
    model::ViscoelasticCoulombModel{T}
    tangential_displacement::FreeVector3D{V}
end

function reset!(state::ViscoelasticCoulombState)
    fill!(state.tangential_displacement.v, 0)
end

function friction_force{T}(model::ViscoelasticCoulombModel{T}, state::ViscoelasticCoulombState{T},
        fnormal::T, tangential_velocity::FreeVector3D)
    μ = model.μ
    k = model.k
    b = model.b
    x = convert(FreeVector3D{SVector{3, T}}, state.tangential_displacement)
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
end

function dynamics!{T}(ẋ::AbstractVector{T}, model::ViscoelasticCoulombModel{T}, state::ViscoelasticCoulombState{T}, ftangential::FreeVector3D)
    # TODO: type of ẋ?
    k = model.k
    b = model.b
    x = state.tangential_displacement
    ẋ .= (-(k * x + ftangential) / b).v
end

@compat const DefaultContactPoint{T} = ContactPoint{T,SoftContactModel{HuntCrossleyModel{T},ViscoelasticCoulombModel{T}}}

# TODO: generalize:
function contact_dynamics!(contact_state_deriv::AbstractVector, contact_state::ViscoelasticCoulombState,
        model::SoftContactModel, penetration::Number, velocity::FreeVector3D, normal::FreeVector3D)
    z = penetration
    force = if z > 0
        ż = -dot(velocity, normal) # penetration velocity
        fnormal = normal_force(normal_force_model(model), z, ż)
        fnormal = max(fnormal, zero(fnormal))
        tangential_velocity = velocity + ż * normal
        ftangential = friction_force(friction_model(model), contact_state, fnormal, tangential_velocity)
        Contact.dynamics!(contact_state_deriv, friction_model(model), contact_state, ftangential)
        fnormal * normal + ftangential
    else
        reset!(contact_state)
        contact_state_deriv .= 0
        zero(normal)
    end
end

end
