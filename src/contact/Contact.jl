module Contact

using LinearAlgebra
using RigidBodyDynamics.Spatial
using StaticArrays

# base types
export
    SoftContactModel,
    ContactPoint,
    ContactEnvironment,
    HalfSpace3D

# interface functions
export
    num_states,
    contact_dynamics,
    point_inside,
    separation,
    detect_contact

# specific models
export HuntCrossleyModel,
    hunt_crossley_hertz

export ViscoelasticCoulombModel,
    ViscoelasticCoulombState


struct SoftContactModel{N, T}
    normal::N
    tangential::T
end

num_states(model::SoftContactModel) = num_states(model.normal) + num_states(model.tangential)

## ContactPoint
mutable struct ContactPoint{T, M <: SoftContactModel}
    location::Point3D{SVector{3, T}}
    model::M
end

@noinline throw_negative_penetration_error() = error("penetration must be nonnegative")

function contact_dynamics(model::SoftContactModel, (normal_state, tangential_state), penetration::Number, velocity::FreeVector3D, normal::FreeVector3D)
    @boundscheck penetration >= 0 || throw_negative_penetration_error()

    z = penetration
    ż = -dot(velocity, normal) # penetration velocity

    fnormal = max(normal_force(model.normal, normal_state, z, ż), 0)
    normal_state_deriv = contact_dynamics(model.normal, normal_state, fnormal)

    tangential_velocity = velocity + ż * normal
    ftangential = tangential_force(model.tangential, tangential_state, fnormal, tangential_velocity)
    tangential_state_deriv = contact_dynamics(model.tangential, tangential_state, ftangential)

    f = fnormal * normal + ftangential
    state_deriv = (normal_state_deriv, tangential_state_deriv)

    f, state_deriv
end


## Normal contact models
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

function normal_force(model::HuntCrossleyModel, ::Nothing, z, ż)
    zn = z^model.n
    f = model.λ * zn * ż + model.k * zn # (2) in Marhefka, Orin (note: z is penetration, returning repelling force)
end

contact_dynamics(model::HuntCrossleyModel, state::Nothing, fnormal::Number) = nothing


# Friction models
struct ViscoelasticCoulombModel{T}
    # See section 11.8 of Featherstone, "Rigid Body Dynamics Algorithms", 2008
    μ::T
    k::T
    b::T
end

 # Use a 3-vector; technically only need one state for each tangential direction, but this is easier to work with.
num_states(::ViscoelasticCoulombModel) = 3
zero_state(::ViscoelasticCoulombModel{T}, frame::CartesianFrame3D) where {T} = FreeVector3D(frame, zero(SVector{3, T}))
zero_state_deriv(::ViscoelasticCoulombModel{T}, frame::CartesianFrame3D) where {T} = FreeVector3D(frame, zero(SVector{3, T}))

function tangential_force(model::ViscoelasticCoulombModel, state::FreeVector3D, fnormal::Number, tangential_velocity::FreeVector3D)
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
end

function contact_dynamics(model::ViscoelasticCoulombModel, state::FreeVector3D, ftangential::FreeVector3D)
    k = model.k
    b = model.b
    x = state
    ẋ = FreeVector3D(x.frame, (-k .* x - ftangential).v ./ b)
    return ẋ
end


# Contact detection
mutable struct HalfSpace3D{T}
    point::Point3D{SVector{3, T}}
    outward_normal::FreeVector3D{SVector{3, T}}

    function HalfSpace3D(point::Point3D{SVector{3, T}}, outward_normal::FreeVector3D{SVector{3, T}}) where {T}
        @framecheck point.frame outward_normal.frame
        new{T}(point, normalize(outward_normal))
    end
end

frame(halfspace::HalfSpace3D) = halfspace.point.frame

function HalfSpace3D(point::Point3D, outward_normal::FreeVector3D)
    T = promote_type(eltype(point), eltype(outward_normal))
    HalfSpace3D(convert(Point3D{SVector{3, T}}, point), convert(FreeVector3D{SVector{3, T}}, outward_normal))
end

Base.eltype(::Type{HalfSpace3D{T}}) where {T} = T
separation(halfspace::HalfSpace3D, p::Point3D) = dot(p - halfspace.point, halfspace.outward_normal)
point_inside(halfspace::HalfSpace3D, p::Point3D) = separation(halfspace, p) <= 0
detect_contact(halfspace::HalfSpace3D, p::Point3D) = separation(halfspace, p), halfspace.outward_normal


# ContactEnvironment
mutable struct ContactEnvironment{T}
    halfspaces::Vector{HalfSpace3D{T}}
    ContactEnvironment{T}() where {T} = new{T}(HalfSpace3D{T}[])
end

Base.push!(environment::ContactEnvironment, halfspace::HalfSpace3D) = push!(environment.halfspaces, halfspace)
Base.length(environment::ContactEnvironment) = length(environment.halfspaces)

end # module
