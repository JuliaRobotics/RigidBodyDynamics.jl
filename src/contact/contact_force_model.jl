abstract type ContactForceModel end

vectorize(x::Nothing) = SVector{0, Union{}}()
vectorize(x::AbstractVector) = x
vectorize(x::FreeVector3D) = x.v
vectorize(x::Tuple{}) = SVector{0, Union{}}()
vectorize(x::Tuple) = vcat(vectorize(x[1]), vectorize(Base.tail(x)))

struct SplitContactForceModel{N, T} <: ContactForceModel
    normal::N
    tangential::T
end

num_states(model::SplitContactForceModel) = num_states(model.normal) + num_states(model.tangential)
zero_state(model::SplitContactForceModel) = (zero_state(model.normal), zero_state(model.tangential))

function devectorize(model::SplitContactForceModel, x::AbstractVector)
    nx_normal = num_states(model.normal)
    nx_tangential = num_states(model.tangential)
    # TODO: use uviews?
    normal_state = let x_normal = view(x, 1 : nx_normal)
        devectorize(model.normal, x_normal)
    end
    tangential_state = let x_tangential = view(x, nx_normal + 1 : nx_normal + nx_tangential)
        devectorize(model.tangential, x_tangential)
    end
    normal_state, tangential_state
end

@noinline throw_negative_penetration_error() = error("penetration must be nonnegative")

function soft_contact_dynamics(
        model::SplitContactForceModel,
        (normal_state, tangential_state),
        penetration::Number, # of A in B
        relative_velocity::SVector{3}, # of A w.r.t. B
        normal::SVector{3}) # outward normal from B
    @boundscheck penetration >= 0 || throw_negative_penetration_error()
    z = penetration
    ż = -dot(relative_velocity, normal) # penetration velocity
    fnormal, normal_state_deriv = normal_contact_dynamics(
        model.normal, normal_state, z, ż)
    tangential_velocity = relative_velocity + ż * normal
    ftangential, tangential_state_deriv = tangential_contact_dynamics(
        model.tangential, tangential_state, fnormal, tangential_velocity)
    f = fnormal * normal + ftangential
    state_deriv = (normal_state_deriv, tangential_state_deriv)
    f, state_deriv
end
