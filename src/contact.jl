module Contact

using RigidBodyDynamics # TODO: modularize more
using StaticArrays
using ForwardDiff
using Compat

import RigidBodyDynamics: VectorSegment

# base types
export SoftContactModel,
    SoftContactState,
    SoftContactStateDeriv,
    ContactPoint,
    ContactEnvironment,
    HalfSpace3D,
    Box3D

# interface functions
export num_states,
    location,
    contact_model,
    contact_dynamics!,
    reset!,
    point_inside,
    detect_contact

# specific models
export HuntCrossleyModel,
    hunt_crossley_hertz

export ViscoelasticCoulombModel,
    ViscoelasticCoulombState,
    ViscoelasticCoulombStateDeriv

# defaults
export DefaultContactPoint,
    DefaultSoftContactState,
    DefaultSoftContactStateDeriv

# Normal force model/state/state derivative interface:
# * `normal_force(model, state, z::Number, ż::Number)`: return the normal force given penetration `z` and penetration velocity `ż`
# * `num_states(model)`: return the number of state variables associated with the model
# * `state(model, statevec, frame)`: create a new state associated with the model
# * `state_derivative(model, statederivvec, frame)`: create a new state derivative associated with the model
# * `reset!(state)`: resets the contact state
# * `dynamics!(state_deriv, model, state, fnormal)`: sets state_deriv given the current state and the normal force

# Friction model/state/state derivative interface:
# * `friction_force(model, state, fnormal::Number, tangential_velocity::FreeVector3D)`: return the friction force given normal force `fnormal` and tangential velocity `tangential_velocity`
# * `num_states(model)`: return the number of state variables associated with the model
# * `state(model, statevec, frame)`: create a new state associated with the model
# * `state_derivative(model, statederivvec, frame)`: create a new state derivative associated with the model
# * `reset!(state)`: resets the contact state
# * `dynamics!(state_deriv, model, state, ftangential)`: sets state_deriv given the current state and the tangential force force

## SoftContactModel and related types.
immutable SoftContactModel{N, F}
    normal::N
    friction::F
end

immutable SoftContactState{N, F}
    normal::N
    friction::F
end

immutable SoftContactStateDeriv{N, F}
    normal::N
    friction::F
end

normal_force_model(model::SoftContactModel) = model.normal
friction_model(model::SoftContactModel) = model.friction

normal_force_state(state::SoftContactState) = state.normal
friction_state(state::SoftContactState) = state.friction

normal_force_state_deriv(deriv::SoftContactStateDeriv) = deriv.normal
friction_state_deriv(deriv::SoftContactStateDeriv) = deriv.friction

num_states(model::SoftContactModel) = num_states(normal_force_model(model)) + num_states(friction_model(model))

for (fun, returntype) in [(:state, :SoftContactState), (:state_derivative, :SoftContactStateDeriv)]
    @eval function $returntype(model::SoftContactModel, vec::AbstractVector, frame::CartesianFrame3D)
        nnormal = num_states(normal_force_model(model))
        nfriction = num_states(friction_model(model))
        vecnormal = view(vec, 1 : nnormal - 1)
        vecfriction = view(vec, 1 + nnormal : nnormal + nfriction)
        $returntype($fun(normal_force_model(model), vecnormal, frame), $fun(friction_model(model), vecfriction, frame))
    end
end

reset!(state::SoftContactState) = (reset!(normal_force_state(state)); reset!(friction_state(state)))
zero!(deriv::SoftContactStateDeriv) = (zero!(friction_state_deriv(deriv)); zero!(friction_state_deriv(deriv)))

## ContactPoint
type ContactPoint{T, M <: SoftContactModel}
    location::Point3D{SVector{3, T}}
    model::M
end
location(point::ContactPoint) = point.location
contact_model(point::ContactPoint) = point.model

function contact_dynamics!(state_deriv::SoftContactStateDeriv, state::SoftContactState,
        model::SoftContactModel, penetration::Number, velocity::FreeVector3D, normal::FreeVector3D)
    @boundscheck penetration >= 0 || error("penetration must be nonnegative")

    z = penetration
    ż = -dot(velocity, normal) # penetration velocity
    fnormal = max(normal_force(normal_force_model(model), normal_force_state(state), z, ż), 0)
    dynamics!(normal_force_state_deriv(state_deriv), normal_force_model(model), normal_force_state(state), fnormal)

    tangential_velocity = velocity + ż * normal
    ftangential = friction_force(friction_model(model), friction_state(state), fnormal, tangential_velocity)
    dynamics!(friction_state_deriv(state_deriv), friction_model(model), friction_state(state), ftangential)

    fnormal * normal + ftangential
end


## Models with no state
reset!(::Void) = nothing
zero!(::Void) = nothing


## Normal contact models
immutable HuntCrossleyModel{T}
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
state(::HuntCrossleyModel, ::AbstractVector, ::CartesianFrame3D) = nothing
state_derivative(::HuntCrossleyModel, ::AbstractVector, ::CartesianFrame3D) = nothing

function normal_force(model::HuntCrossleyModel, ::Void, z, ż)
    zn = z^model.n
    f = model.λ * zn * ż + model.k * zn # (2) in Marhefka, Orin (note: z is penetration, returning repelling force)
end

dynamics!(ẋ::Void, model::HuntCrossleyModel, state::Void, fnormal::Number) = nothing


# Friction models
immutable ViscoelasticCoulombModel{T}
    # See section 11.8 of Featherstone, "Rigid Body Dynamics Algorithms", 2008
    μ::T
    k::T
    b::T
end

type ViscoelasticCoulombState{V}
    tangential_displacement::FreeVector3D{V} # Use a 3-vector; technically only need one state for each tangential direction, but this is easier to work with.
end

type ViscoelasticCoulombStateDeriv{V}
    deriv::FreeVector3D{V}
end

num_states(::ViscoelasticCoulombModel) = 3
function state(::ViscoelasticCoulombModel, statevec::AbstractVector, frame::CartesianFrame3D)
    ViscoelasticCoulombState(FreeVector3D(frame, statevec))
end
function state_derivative(::ViscoelasticCoulombModel, statederivvec::AbstractVector, frame::CartesianFrame3D)
    ViscoelasticCoulombStateDeriv(FreeVector3D(frame, statederivvec))
end

reset!(state::ViscoelasticCoulombState) = fill!(state.tangential_displacement.v, 0)
zero!(deriv::ViscoelasticCoulombStateDeriv) = fill!(deriv.deriv.v, 0)

function friction_force(model::ViscoelasticCoulombModel, state::ViscoelasticCoulombState,
        fnormal, tangential_velocity::FreeVector3D)
    T = typeof(fnormal)
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

function dynamics!(ẋ::ViscoelasticCoulombStateDeriv, model::ViscoelasticCoulombModel, state::ViscoelasticCoulombState, ftangential::FreeVector3D)
    k = model.k
    b = model.b
    x = state.tangential_displacement
    @framecheck ẋ.deriv.frame state.tangential_displacement.frame
    ẋ.deriv.v .= (-(k * x + ftangential) / b).v
end

@compat const DefaultContactPoint{T} = ContactPoint{T,SoftContactModel{HuntCrossleyModel{T},ViscoelasticCoulombModel{T}}}
@compat const DefaultSoftContactState{T} = SoftContactState{Void, ViscoelasticCoulombState{VectorSegment{T}}}
@compat const DefaultSoftContactStateDeriv{T} = SoftContactStateDeriv{Void, ViscoelasticCoulombStateDeriv{VectorSegment{T}}}


# Contact detection
# TODO: should probably move this somewhere else

type HalfSpace3D{T}
    point::Point3D{SVector{3, T}}
    outward_normal::FreeVector3D{SVector{3, T}}

    function HalfSpace3D(point::Point3D{SVector{3, T}}, outward_normal::FreeVector3D{SVector{3, T}})
        @framecheck point.frame outward_normal.frame
        new(point, outward_normal)
    end
end

HalfSpace3D{T}(point::Point3D{SVector{3, T}}, outward_normal::FreeVector3D{SVector{3, T}}) = HalfSpace3D{T}(point, outward_normal)
frame(halfspace::HalfSpace3D) = halfspace.point.frame

function HalfSpace3D(point::Point3D, outward_normal::FreeVector3D)
    T = promote_type(eltype(point), eltype(outward_normal))
    HalfSpace3D(convert(Point3D{SVector{3, T}}, point), convert(FreeVector3D{SVector{3, T}}, outward_normal))
end

Base.eltype{T}(::Type{HalfSpace3D{T}}) = T
separation(halfspace::HalfSpace3D, p::Point3D) = dot(p - halfspace.point, halfspace.outward_normal)
point_inside(halfspace::HalfSpace3D, p::Point3D) = separation(halfspace, p) <= 0
detect_contact(halfspace::HalfSpace3D, p::Point3D) = separation(halfspace, p), halfspace.outward_normal


type Box3D{T}
    frame::CartesianFrame3D
    halfdims::SVector{3, T}
end
Base.eltype{T}(::Type{Box3D{T}}) = T

Box3D(frame::CartesianFrame3D, x, y, z) = Box3D(frame, SVector(x, y, z))
center(box::Box3D) = Point3D(box.frame)


function point_inside(box::Box3D, p::Point3D)
    # no allocations
    @framecheck box.frame p.frame
    all(abs.(p.v) .<= box.halfdims)
end

function _closest_point_on_surface_inside(box::Box3D, p::Point3D)
    # no allocations
    @framecheck box.frame p.frame
    v = p.v
    maxdiff = v - box.halfdims
    mindiff = v + box.halfdims

    dmax = abs.(maxdiff)
    dmin = abs.(mindiff)

    maxind = indmin(dmax)
    minind = indmin(dmin)

    coord, ind = ifelse(dmax[maxind] <= dmin[minind], (box.halfdims[maxind], maxind), (-box.halfdims[minind], minind))
    vec = if ind == 1
        SVector(coord, v[2], v[3])
    elseif ind == 2
        SVector(v[1], coord, v[3])
    else
        SVector(v[1], v[2], coord)
    end
    Point3D(p.frame, vec)
end

function _closest_point_on_surface_outside(box::Box3D, p::Point3D)
    # no allocations
    @framecheck box.frame p.frame
    temp = max.(-box.halfdims, p.v)
    vec = min.(temp, box.halfdims)
    Point3D(p.frame, vec)
end

function closest_point_on_surface(box::Box3D, p::Point3D)
    # no allocations
    point_inside(box, p) ? _closest_point_on_surface_inside(box, p) : _closest_point_on_surface_outside(box, p)
end

function detect_contact(box::Box3D, p::Point3D)
    # no allocations
    @framecheck box.frame p.frame

    frame = p.frame
    T = eltype(p)
    x = ForwardDiff.Dual(p.v[1], one(T), zero(T), zero(T))
    y = ForwardDiff.Dual(p.v[2], zero(T), one(T), zero(T))
    z = ForwardDiff.Dual(p.v[3], zero(T), zero(T), one(T))
    pdiff = Point3D(frame, SVector(x, y, z))

    # TODO: consider doing this without computing closest point explicitly
    inside = point_inside(box, p)
    closestdiff = inside ? _closest_point_on_surface_inside(box, p) : _closest_point_on_surface_outside(box, p)
    closest = Point3D(frame, ForwardDiff.value.(closestdiff.v))
    Δdiff = pdiff - closestdiff
    distsquared_diff = dot(Δdiff, Δdiff)

    distsquared = ForwardDiff.value(distsquared_diff)
    dist = sqrt(distsquared)
    separation = ifelse(inside, -dist, dist)

    signed_distsquared_diff = inside ? -distsquared_diff : distsquared_diff
    grad = ForwardDiff.partials(signed_distsquared_diff)
    gradnorm = normalize(SVector(grad[1], grad[2], grad[3]))
    outward_normal = FreeVector3D(frame, gradnorm)

    separation, outward_normal
end


# ContactEnvironment
type ContactEnvironment{T}
    halfspaces::Vector{HalfSpace3D{T}}
    # boxes::Vector{Box3D{T}}
    ContactEnvironment() = new(HalfSpace3D{T}[])
end

Base.push!(environment::ContactEnvironment, halfspace::HalfSpace3D) = push!(environment.halfspaces, halfspace)
# Base.push!(environment::ContactEnvironment, box::Box3D) = push!(environment.boxes, box)

end
