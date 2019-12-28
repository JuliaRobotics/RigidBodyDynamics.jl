abstract type ContactForceModel end

struct SplitContactForceModel{N, T} <: ContactForceModel
    normal::N
    tangential::T
end

num_states(model::SplitContactForceModel) = num_states(model.normal) + num_states(model.tangential)

@noinline throw_negative_penetration_error() = error("penetration must be nonnegative")

function soft_contact_dynamics(
        model::SplitContactForceModel,
        (normal_state, tangential_state),
        penetration::Number, # of A in B
        relative_velocity::FreeVector3D, # of A w.r.t. B
        normal::FreeVector3D) # outward normal from B
    @boundscheck penetration >= 0 || throw_negative_penetration_error()
    z = penetration
    ż = -dot(relative_velocity, normal) # penetration relative_velocity
    fnormal, normal_state_deriv = soft_contact_dynamics(model.normal, normal_state, z, ż)
    tangential_velocity = relative_velocity + ż * normal
    ftangential, tangential_state_deriv = soft_contact_dynamics(model.tangential, tangential_state, fnormal, tangential_velocity)
    f = fnormal * normal + ftangential
    state_deriv = (normal_state_deriv, tangential_state_deriv)
    f, state_deriv
end
