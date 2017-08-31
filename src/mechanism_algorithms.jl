"""
$(SIGNATURES)

Return the mass of a subtree of a `Mechanism`, rooted at `base` (including
the mass of `base`).
"""
function subtree_mass(base::RigidBody{T}, mechanism::Mechanism{T}) where {T}
    result = isroot(base, mechanism) ? zero(T) : spatial_inertia(base).mass
    for joint in joints_to_children(base, mechanism)
        result += subtree_mass(successor(joint, mechanism), mechanism)
    end
    result
end

"""
$(SIGNATURES)

Return the total mass of the `Mechanism`.
"""
mass(m::Mechanism) = subtree_mass(root_body(m), m)

"""
$(SIGNATURES)

Compute the center of mass of an iterable subset of a `Mechanism`'s bodies in
the given state. Ignores the root body of the mechanism.
"""
function center_of_mass(state::MechanismState, itr)
    T = cache_eltype(state)
    mechanism = state.mechanism
    frame = root_frame(mechanism)
    com = Point3D(frame, zeros(SVector{3, T}))
    mass = zero(T)
    for body in itr
        if !isroot(body, mechanism)
            inertia = spatial_inertia(body)
            if inertia.mass > 0
                bodycom = transform_to_root(state, body) * center_of_mass(inertia)
                com += inertia.mass * FreeVector3D(bodycom)
                mass += inertia.mass
            end
        end
    end
    com /= mass
    com
end

"""
$(SIGNATURES)

Compute the center of mass of the whole `Mechanism` in the given state.
"""
center_of_mass(state::MechanismState) = center_of_mass(state, bodies(state.mechanism))

const geometric_jacobian_doc = """Compute a geometric Jacobian (also known as a
basic, or spatial Jacobian) associated with the joints that form a path in the
`Mechanism`'s spanning tree, in the given state.

A geometric Jacobian maps the vector of velocities associated with the joint path
to the twist of the body succeeding the last joint in the path with respect to
the body preceding the first joint in the path.

See also [`path`](@ref), [`GeometricJacobian`](@ref), [`velocity(state, path)`](@ref),
[`Twist`](@ref).
"""

"""
$(SIGNATURES)

$geometric_jacobian_doc

`transformfun` is a callable that may be used to transform the individual motion
subspaces of each of the joints to the frame in which `out` is expressed.

$noalloc_doc
"""
function geometric_jacobian!(out::GeometricJacobian, state::MechanismState, path::TreePath, transformfun)
    @boundscheck num_velocities(state) == num_cols(out) || error("size mismatch")
    @framecheck out.body default_frame(target(path))
    @framecheck out.base default_frame(source(path))
    update_motion_subspaces_in_world!(state)
    foreach_with_extra_args(out, state, path, state.type_sorted_tree_joints) do out, state, path, joint # TODO: use closure once it doesn't allocate
        vrange = velocity_range(state, joint)
        if Graphs.edge_index(joint) in path.indices
            part = transformfun(motion_subspace_in_world(state, joint))
            direction(joint, path) == :up && (part = -part)
            set_cols!(out, vrange, part)
        else
            zero_cols!(out, vrange)
        end
    end
    out
end

"""
$(SIGNATURES)

$geometric_jacobian_doc

`root_to_desired` is the transform from the `Mechanism`'s root frame to the frame
in which `out` is expressed.

$noalloc_doc
"""
function geometric_jacobian!(out::GeometricJacobian, state::MechanismState, path::TreePath, root_to_desired::Transform3D)
    geometric_jacobian!(out, state, path, S -> transform(S, root_to_desired))
end

"""
$(SIGNATURES)

$geometric_jacobian_doc

See [`geometric_jacobian!(out, state, path, root_to_desired)`](@ref). Uses `state`
to compute the transform from the `Mechanism`'s root frame to the frame in which
`out` is expressed.

$noalloc_doc
"""
function geometric_jacobian!(out::GeometricJacobian, state::MechanismState, path::TreePath)
    if out.frame == root_frame(state.mechanism)
        geometric_jacobian!(out, state, path, identity)
    else
        geometric_jacobian!(out, state, path, inv(transform_to_root(state, out.frame)))
    end
end

"""
$(SIGNATURES)

$geometric_jacobian_doc

The Jacobian is computed in the `Mechanism`'s root frame.

See [`geometric_jacobian!(out, state, path)`](@ref).
"""
function geometric_jacobian(state::MechanismState{X, M, C}, path::TreePath{RigidBody{M}, GenericJoint{M}}) where {X, M, C}
    nv = num_velocities(state)
    angular = Matrix{C}(3, nv)
    linear = Matrix{C}(3, nv)
    bodyframe = default_frame(target(path))
    baseframe = default_frame(source(path))
    jac = GeometricJacobian(bodyframe, baseframe, root_frame(state.mechanism), angular, linear)
    geometric_jacobian!(jac, state, path, identity)
end

const mass_matrix_doc = """Compute the joint-space mass matrix
(also known as the inertia matrix) of the `Mechanism` in the given state, i.e.,
the matrix ``M(q)`` in the unconstrained joint-space equations of motion
```math
M(q) \\dot{v} + c(q, v, w_\\text{ext}) = \\tau
```

This method implements the composite rigid body algorithm.
"""

"""
$(SIGNATURES)

$mass_matrix_doc

$noalloc_doc

The `out` argument must be an ``n_v \\times n_v`` lower triangular
`Symmetric` matrix, where ``n_v`` is the dimension of the `Mechanism`'s joint
velocity vector ``v``.
"""
function mass_matrix!(out::Symmetric{C, Matrix{C}}, state::MechanismState{X, M, C}) where {X, M, C}
    @boundscheck size(out, 1) == num_velocities(state) || error("mass matrix has wrong size")
    @boundscheck out.uplo == 'L' || error("expected a lower triangular symmetric matrix type as the mass matrix")
    fill!(out.data, 0)
    update_motion_subspaces_in_world!(state)
    update_crb_inertias!(state)
    joints = state.type_sorted_tree_joints
    foreach_with_extra_args(out, state, joints) do out, state, jointi # TODO: use closure once it doesn't allocate
        irange = velocity_range(state, jointi)
        bodyi = successor(jointi, state.mechanism)
        Si = motion_subspace_in_world(state, jointi)
        Ici = crb_inertia(state, bodyi)
        F = Ici * Si
        ancestor_joints = state.type_sorted_ancestor_joints[jointi]
        foreach_with_extra_args(out, state, irange, F, ancestor_joints) do out, state, irange, F, jointj # TODO: use closure once it doesn't allocate
            jrange = velocity_range(state, jointj)
            bodyj = successor(jointj, state.mechanism)
            Sj = motion_subspace_in_world(state, jointj)
            block = F.angular' * Sj.angular + F.linear' * Sj.linear
            set_matrix_block!(out.data, irange, jrange, block)
        end
    end
    out
end

mass_matrix!(result::DynamicsResult, state::MechanismState) = mass_matrix!(result.massmatrix, state)

"""
$(SIGNATURES)

$mass_matrix_doc
"""
function mass_matrix(state::MechanismState{X, M, C}) where {X, M, C}
    nv = num_velocities(state)
    ret = Symmetric(Matrix{C}(nv, nv), :L)
    mass_matrix!(ret, state)
    ret
end

const momentum_matrix_doc = """Compute the momentum matrix ``A(q)`` of the
`Mechanism` in the given state.

The momentum matrix maps the `Mechanism`'s joint velocity vector ``v`` to
its total momentum.

See also [`MomentumMatrix`](@ref).
"""

const momentum_matrix!_doc = """$momentum_matrix_doc
The `out` argument must be a mutable `MomentumMatrix` with as many columns as
the dimension of the `Mechanism`'s joint velocity vector ``v``.
"""

"""
$(SIGNATURES)

$momentum_matrix!_doc

`transformfun` is a callable that may be used to transform the individual
momentum matrix blocks associated with each of the joints to the frame in which
`out` is expressed.

$noalloc_doc
"""
function momentum_matrix!(out::MomentumMatrix, state::MechanismState, transformfun)
    @boundscheck num_velocities(state) == num_cols(out) || error("size mismatch")
    update_motion_subspaces_in_world!(state)
    update_crb_inertias!(state)
    foreach_with_extra_args(out, state, state.type_sorted_tree_joints) do out, state, joint
        vrange = velocity_range(state, joint)
        mechanism = state.mechanism
        body = successor(joint, mechanism)
        inertia = crb_inertia(state, body)
        part = transformfun(inertia * motion_subspace_in_world(state, joint)) # TODO: consider pure body frame implementation
        set_cols!(out, vrange, part)
    end
end

"""
$(SIGNATURES)

$momentum_matrix!_doc

`root_to_desired` is the transform from the `Mechanism`'s root frame to the frame
in which `out` is expressed.

$noalloc_doc
"""
function momentum_matrix!(out::MomentumMatrix, state::MechanismState, root_to_desired::Transform3D)
    momentum_matrix!(out, state, IJ -> transform(IJ, root_to_desired))
end

"""
$(SIGNATURES)

$momentum_matrix!_doc

See [`momentum_matrix!(out, state, root_to_desired)`](@ref). Uses `state`
to compute the transform from the `Mechanism`'s root frame to the frame in which
`out` is expressed.

$noalloc_doc
"""
function momentum_matrix!(out::MomentumMatrix, state::MechanismState)
    if out.frame == root_frame(state.mechanism)
        momentum_matrix!(out, state, identity)
    else
        momentum_matrix!(out, state, inv(transform_to_root(state, out.frame)))
    end
end

"""
$(SIGNATURES)

$momentum_matrix_doc

See [`momentum_matrix!(out, state)`](@ref).
"""
function momentum_matrix(state::MechanismState)
    ncols = num_velocities(state)
    T = cache_eltype(state)
    ret = MomentumMatrix(root_frame(state.mechanism), Matrix{T}(3, ncols), Matrix{T}(3, ncols))
    momentum_matrix!(ret, state, identity)
    ret
end

function bias_accelerations!(out::Associative{RigidBody{M}, SpatialAcceleration{T}}, state::MechanismState{X, M}) where {T, X, M}
    update_bias_accelerations_wrt_world!(state)
    mechanism = state.mechanism
    gravitybias = convert(SpatialAcceleration{T}, -gravitational_spatial_acceleration(mechanism))
    for joint in tree_joints(mechanism)
        body = successor(joint, mechanism)
        out[body] = gravitybias + bias_acceleration(state, body)
    end
    nothing
end

function spatial_accelerations!(out::Associative{RigidBody{M}, SpatialAcceleration{T}},
        state::MechanismState{X, M}, vd::StridedVector) where {T, X, M}
    update_twists_wrt_world!(state)
    mechanism = state.mechanism
    root = root_body(mechanism)
    joints = state.type_sorted_tree_joints
    qs = values(state.qs)
    vs = values(state.vs)

    # Compute joint accelerations
    foreach_with_extra_args(state, out, vd, joints, qs, vs) do state, accels, vd, joint, qjoint, vjoint # TODO: use closure once it doesn't allocate
        body = successor(joint, state.mechanism)
        parentbody = predecessor(joint, state.mechanism)
        vdjoint = fastview(vd, velocity_range(state, joint))
        accels[body] = joint_spatial_acceleration(joint, qjoint, vjoint, vdjoint)
    end

    # Recursive propagation
    out[root] = convert(SpatialAcceleration{T}, -gravitational_spatial_acceleration(mechanism))
    for joint in tree_joints(mechanism)
        body = successor(joint, mechanism)
        parentbody = predecessor(joint, mechanism)
        parentframe = default_frame(parentbody)

            # TODO: awkward way of doing this (consider switching to body frame implementation):
        toroot = transform_to_root(state, body)
        twistwrtworld = transform(twist_wrt_world(state, body), inv(toroot))
        jointtwist = change_base(twist(state, joint), parentframe) # to make frames line up
        jointaccel = change_base(out[body], parentframe) # to make frames line up
        out[body] = out[parentbody] + transform(jointaccel, toroot, twistwrtworld, jointtwist)
    end
    nothing
end

spatial_accelerations!(result::DynamicsResult, state::MechanismState) = spatial_accelerations!(result.accelerations, state, result.v̇)

function relative_acceleration(accels::Associative{RigidBody{M}, SpatialAcceleration{T}}, body::RigidBody{M}, base::RigidBody{M}) where {T, M}
    -accels[base] + accels[body]
end

# TODO: ensure that accelerations are up-to-date
relative_acceleration(result::DynamicsResult, body::RigidBody, base::RigidBody) = relative_acceleration(result.accelerations, body, base)

function relative_acceleration(state::MechanismState, body::RigidBody, base::RigidBody, vd::AbstractVector)
    error("""`relative_acceleration(state, body, base, vd)` has been removed.
    Use `spatial_accelerations!(result, state)` or `spatial_accelerations!(accels, state, vd)` to compute the
    spatial accelerations of all bodies in one go, and then use `relative_acceleration(accels, body, base)` or
    `relative_acceleration(result, body, base)`.""")
end

function newton_euler!(
        out::Associative{RigidBody{M}, Wrench{T}}, state::MechanismState{X, M},
        accelerations::Associative{RigidBody{M}, SpatialAcceleration{T}},
        externalwrenches::Associative{RigidBody{M}, Wrench{W}}) where {T, X, M, W}
    update_twists_wrt_world!(state)
    update_spatial_inertias!(state)
    mechanism = state.mechanism
    for joint in tree_joints(mechanism)
        body = successor(joint, mechanism)
        wrench = newton_euler(state, body, accelerations[body])
        out[body] = haskey(externalwrenches, body) ? wrench - externalwrenches[body] : wrench
    end
end


function joint_wrenches_and_torques!(
        torquesout::StridedVector{T},
        net_wrenches_in_joint_wrenches_out::Associative{RigidBody{M}, Wrench{T}}, # TODO: consider having a separate Associative{Joint{M}, Wrench{T}} for joint wrenches
        state::MechanismState{X, M}) where {T, X, M}
    # Note: pass in net wrenches as wrenches argument. wrenches argument is modified to be joint wrenches
    @boundscheck length(torquesout) == num_velocities(state) || error("length of torque vector is wrong")
    mechanism = state.mechanism
    joints = tree_joints(mechanism)
    for i = length(joints) : -1 : 1
        joint = joints[i]
        body = successor(joint, mechanism)
        parentbody = predecessor(joint, mechanism)
        jointwrench = net_wrenches_in_joint_wrenches_out[body]
        if !isroot(parentbody, mechanism)
            # TODO: consider also doing this for the root:
            net_wrenches_in_joint_wrenches_out[parentbody] += jointwrench # action = -reaction
        end
    end

    foreach_with_extra_args(state, torquesout, net_wrenches_in_joint_wrenches_out, state.type_sorted_tree_joints) do state, τ, wrenches, joint # TODO: use closure once it doesn't allocate
        body = successor(joint, state.mechanism)
        @inbounds τjoint = fastview(τ, velocity_range(state, joint))
        # TODO: awkward to transform back to body frame; consider switching to body-frame implementation
        tf = inv(transform_to_root(state, body))
        joint_torque!(τjoint, joint, configuration(state, joint), transform(wrenches[body], tf))
    end
end

const dynamics_bias_doc = """Compute the 'dynamics bias term', i.e. the term
```math
c(q, v, w_\\text{ext})
```
in the unconstrained joint-space equations of motion
```math
M(q) \\dot{v} + c(q, v, w_\\text{ext}) = \\tau
```
given joint configuration vector ``q``, joint velocity vector ``v``,
joint acceleration vector ``\\dot{v}`` and (optionally) external
wrenches ``w_\\text{ext}``.

The `externalwrenches` argument can be used to specify additional
wrenches that act on the `Mechanism`'s bodies.
"""

"""
$(SIGNATURES)

$dynamics_bias_doc

$noalloc_doc
"""
function dynamics_bias!(
        torques::AbstractVector{T},
        biasaccelerations::Associative{RigidBody{M}, SpatialAcceleration{T}},
        wrenches::Associative{RigidBody{M}, Wrench{T}},
        state::MechanismState{X, M},
        externalwrenches::Associative{RigidBody{M}, Wrench{W}} = NullDict{RigidBody{M}, Wrench{T}}()) where {T, X, M, W}
    bias_accelerations!(biasaccelerations, state)
    newton_euler!(wrenches, state, biasaccelerations, externalwrenches)
    joint_wrenches_and_torques!(torques, wrenches, state)
end

function dynamics_bias!(result::DynamicsResult, state::MechanismState)
    dynamics_bias!(result.dynamicsbias, result.accelerations, result.jointwrenches, state, result.totalwrenches)
end

"""
$(SIGNATURES)

$dynamics_bias_doc
"""
function dynamics_bias(
        state::MechanismState{X, M},
        externalwrenches::Associative{RigidBody{M}, Wrench{W}} = NullDict{RigidBody{M}, Wrench{X}}()) where {X, M, W}
    T = promote_type(X, M, W)
    mechanism = state.mechanism
    torques = Vector{T}(num_velocities(state))
    rootframe = root_frame(mechanism)
    jointwrenches = BodyDict{M, Wrench{T}}(b => zero(Wrench{T}, rootframe) for b in bodies(mechanism))
    accelerations = BodyDict{M, SpatialAcceleration{T}}(b => zero(SpatialAcceleration{T}, rootframe, rootframe, rootframe) for b in bodies(mechanism))
    dynamics_bias!(torques, accelerations, jointwrenches, state, externalwrenches)
    torques
end

const inverse_dynamics_doc = """Do inverse dynamics, i.e. compute ``\\tau``
in the unconstrained joint-space equations of motion
```math
M(q) \\dot{v} + c(q, v, w_\\text{ext}) = \\tau
```
given joint configuration vector ``q``, joint velocity vector ``v``,
joint acceleration vector ``\\dot{v}`` and (optionally) external
wrenches ``w_\\text{ext}``.

The `externalwrenches` argument can be used to specify additional
wrenches that act on the `Mechanism`'s bodies.

This method implements the recursive Newton-Euler algorithm.

Currently doesn't support `Mechanism`s with cycles.
"""

"""
$(SIGNATURES)

$inverse_dynamics_doc

$noalloc_doc
"""
function inverse_dynamics!(
        torquesout::AbstractVector{T},
        jointwrenchesout::Associative{RigidBody{M}, Wrench{T}},
        accelerations::Associative{RigidBody{M}, SpatialAcceleration{T}},
        state::MechanismState{X, M},
        v̇::AbstractVector{V},
        externalwrenches::Associative{RigidBody{M}, Wrench{W}} = NullDict{RigidBody{M}, Wrench{T}}()) where {T, X, M, V, W}
    length(tree_joints(state.mechanism)) == length(joints(state.mechanism)) || error("This method can currently only handle tree Mechanisms.")
    spatial_accelerations!(accelerations, state, v̇)
    newton_euler!(jointwrenchesout, state, accelerations, externalwrenches)
    joint_wrenches_and_torques!(torquesout, jointwrenchesout, state)
end

"""
$(SIGNATURES)

$inverse_dynamics_doc
"""
function inverse_dynamics(
        state::MechanismState{X, M},
        v̇::AbstractVector{V},
        externalwrenches::Associative{RigidBody{M}, Wrench{W}} = NullDict{RigidBody{M}, Wrench{X}}()) where {X, M, V, W}
    T = promote_type(X, M, V, W)
    mechanism = state.mechanism
    torques = Vector{T}(num_velocities(state))
    rootframe = root_frame(mechanism)
    jointwrenches = BodyDict{M, Wrench{T}}(b => zero(Wrench{T}, rootframe) for b in bodies(mechanism))
    accelerations = BodyDict{M, SpatialAcceleration{T}}(b => zero(SpatialAcceleration{T}, rootframe, rootframe, rootframe) for b in bodies(mechanism))
    inverse_dynamics!(torques, jointwrenches, accelerations, state, v̇, externalwrenches)
    torques
end

function constraint_jacobian_and_bias!(state::MechanismState, constraintjacobian::AbstractMatrix, constraintbias::AbstractVector)
    has_loops(state.mechanism) || return # nothing to be done
    update_twists_wrt_world!(state)
    update_bias_accelerations_wrt_world!(state)
    update_motion_subspaces_in_world!(state)
    update_constraint_wrench_subspaces!(state)
    mechanism = state.mechanism
    rowstart = Ref(1) # TODO: allocation
    # note: order of rows of Jacobian and bias term is determined by iteration order of state.type_sorted_non_tree_joints
     # TODO: use closure once it doesn't allocate
    foreach_with_extra_args(constraintjacobian, constraintbias, state, mechanism, rowstart, state.type_sorted_non_tree_joints) do constraintjacobian, constraintbias, state, mechanism, rowstart, nontreejoint
        path = state.constraint_jacobian_structure[nontreejoint]
        nextrowstart = rowstart[] + num_constraints(nontreejoint)
        rowrange = rowstart[] : nextrowstart - 1
        succ = successor(nontreejoint, mechanism)
        pred = predecessor(nontreejoint, mechanism)
        T = constraint_wrench_subspace(state, nontreejoint)

        # Jacobian.
        foreach_with_extra_args(constraintjacobian, state, path, T, rowrange, state.type_sorted_tree_joints) do constraintjacobian, state, path, T, rowrange, treejoint # TODO: use closure once it doesn't allocate
            vrange = velocity_range(state, treejoint)
            if Graphs.edge_index(treejoint) in path.indices
                J = motion_subspace_in_world(state, treejoint)
                part = T.angular' * J.angular + T.linear' * J.linear # TODO: At_mul_B
                direction(treejoint, path) == :up && (part = -part)
                set_matrix_block!(constraintjacobian, rowrange, vrange, part)
            else
                zero_matrix_block!(constraintjacobian, rowrange, vrange)
            end
        end

        # Constraint bias.
        has_fixed_subspaces(nontreejoint) || error("Only joints with fixed motion subspace (Ṡ = 0) supported at this point.") # TODO: call to joint-type-specific function
        kjoint = fastview(constraintbias, rowrange)
        crossterm = cross(twist_wrt_world(state, succ), twist_wrt_world(state, pred))
        biasaccel = crossterm + (-bias_acceleration(state, pred) + bias_acceleration(state, succ)) # 8.47 in Featherstone
        At_mul_B!(kjoint, T, biasaccel)
        rowstart[] = nextrowstart
    end
end

constraint_jacobian_and_bias!(result::DynamicsResult, state::MechanismState) = constraint_jacobian_and_bias!(state, result.constraintjacobian, result.constraintbias)

function contact_dynamics!(result::DynamicsResult{T, M}, state::MechanismState{X, M, C}) where {X, M, C, T}
    update_twists_wrt_world!(state)
    mechanism = state.mechanism
    root = root_body(mechanism)
    frame = default_frame(root)
    for body in bodies(mechanism)
        wrench = zero(Wrench{T}, frame)
        points = contact_points(body)
        if !isempty(points)
            # TODO: AABB
            body_to_root = transform_to_root(state, body)
            twist = twist_wrt_world(state, body)
            states_for_body = contact_states(state, body)
            state_derivs_for_body = contact_state_derivatives(result, body)
            for i = 1 : length(points)
                @inbounds c = points[i]
                point = body_to_root * location(c)
                velocity = point_velocity(twist, point)
                states_for_point = states_for_body[i]
                state_derivs_for_point = state_derivs_for_body[i]
                for j = 1 : length(mechanism.environment.halfspaces)
                    primitive = mechanism.environment.halfspaces[j]
                    contact_state = states_for_point[j]
                    contact_state_deriv = state_derivs_for_point[j]
                    model = contact_model(c)
                    # TODO: would be good to move this to Contact module
                    # arguments: model, state, state_deriv, point, velocity, primitive
                    if point_inside(primitive, point)
                        separation, normal = Contact.detect_contact(primitive, point)
                        force = Contact.contact_dynamics!(contact_state_deriv, contact_state,
                            model, -separation, velocity, normal)
                        wrench += Wrench(point, force)
                    else
                        Contact.reset!(contact_state)
                        Contact.zero!(contact_state_deriv)
                    end
                end
            end
        end
        set_contact_wrench!(result, body, wrench)
    end
end

function dynamics_solve!(result::DynamicsResult, τ::AbstractVector)
    # version for general scalar types
    # TODO: make more efficient
    M = result.massmatrix
    c = result.dynamicsbias
    v̇ = result.v̇

    K = result.constraintjacobian
    k = result.constraintbias
    λ = result.λ

    nv = size(M, 1)
    nl = size(K, 1)
    G = [full(M) K'; # TODO: full because of https://github.com/JuliaLang/julia/issues/21332
         K zeros(nl, nl)]
    r = [τ - c; -k]
    v̇λ = G \ r
    v̇[:] = view(v̇λ, 1 : nv)
    λ[:] = view(v̇λ, nv + 1 : nv + nl)
    nothing
end

function dynamics_solve!(result::DynamicsResult{T, S}, τ::AbstractVector{T}) where {S, T<:LinAlg.BlasReal}
    # optimized version for BLAS floats
    M = result.massmatrix
    c = result.dynamicsbias
    v̇ = result.v̇

    K = result.constraintjacobian
    k = result.constraintbias
    λ = result.λ

    L = result.L
    A = result.A
    z = result.z
    Y = result.Y

    L[:] = M.data
    uplo = M.uplo
    LinAlg.LAPACK.potrf!(uplo, L) # L <- Cholesky decomposition of M; M == L Lᵀ (note: Featherstone, page 151 uses M == Lᵀ L instead)
    τBiased = v̇
    @simd for i in eachindex(τ)
        @inbounds τBiased[i] = τ[i] - c[i]
    end
    # TODO: use τBiased .= τ .- c in 0.6

    if size(K, 1) > 0
        # Loops.
        # Basic math:
        # DAE:
        #   [M Kᵀ] [v̇] = [τ - c]
        #   [K 0 ] [λ]  = [-k]
        # Solve for v̇:
        #   v̇ = M⁻¹ (τ - c - Kᵀ λ)
        # Plug into λ equation:
        #   K M⁻¹ (τ - c - Kᵀ λ) = -k
        #   K M⁻¹ Kᵀ λ = K M⁻¹(τ - c) + k
        # which can be solved for λ, after which λ can be used to solve for v̇.

        # Implementation loosely follows page 151 of Featherstone, Rigid Body Dynamics Algorithms.
        # It is slightly different because Featherstone uses M = Lᵀ L, whereas BLAS uses M = L Lᵀ.
        # M = L Lᵀ (Cholesky decomposition)
        # Y = K L⁻ᵀ, so that Y Yᵀ = K L⁻ᵀ L⁻¹ Kᵀ = K M⁻¹ Kᵀ = A
        # z = L⁻¹ (τ - c)
        # b = Y z + k
        # solve A λ = b for λ
        # solve M v̇  = τ - c for v̇

        # Compute Y = K L⁻ᵀ
        Y[:] = K
        LinAlg.BLAS.trsm!('R', uplo, 'T', 'N', one(T), L, Y)

        # Compute z = L⁻¹ (τ - c)
        z[:] = τBiased
        LinAlg.BLAS.trsv!(uplo, 'N', 'N', L, z) # z <- L⁻¹ (τ - c)

        # Compute A = Y Yᵀ == K * M⁻¹ * Kᵀ
        LinAlg.BLAS.gemm!('N', 'T', one(T), Y, Y, zero(T), A) # A <- K * M⁻¹ * Kᵀ

        # Compute b = Y z + k
        b = λ
        b[:] = k
        LinAlg.BLAS.gemv!('N', one(T), Y, z, one(T), b) # b <- Y z + k

        # Compute λ = A⁻¹ b == (K * M⁻¹ * Kᵀ)⁻¹ * (K * M⁻¹ * (τ - c) + k)
        # LinAlg.LAPACK.posv!(uplo, A, b) # NOTE: doesn't work in general because A is only guaranteed to be positive semidefinite
        singular_value_zero_tolerance = 1e-10 # TODO: more principled choice
        # TODO: https://github.com/JuliaLang/julia/issues/22242
        b[:], _ = LinAlg.LAPACK.gelsy!(A, b, singular_value_zero_tolerance) # b == λ <- (K * M⁻¹ * Kᵀ)⁻¹ * (K * M⁻¹ * (τ - c) + k)

        # Update τBiased: subtract Kᵀ * λ
        LinAlg.BLAS.gemv!('T', -one(T), K, λ, one(T), τBiased) # τBiased <- τ - c - Kᵀ * λ

        # Solve for v̇ = M⁻¹ * (τ - c - Kᵀ * λ)
        LinAlg.LAPACK.potrs!(uplo, L, τBiased) # τBiased ==v̇ <- M⁻¹ * (τ - c - Kᵀ * λ)
    else
        # No loops.
        LinAlg.LAPACK.potrs!(uplo, L, τBiased) # τBiased == v̇ <- M⁻¹ * (τ - c)
    end
    nothing
end

"""
$(SIGNATURES)

Compute the joint acceleration vector ``\\dot{v}`` and Lagrange multipliers
``\\lambda`` that satisfy the joint-space equations of motion
```math
M(q) \\dot{v} + c(q, v, w_\\text{ext}) = \\tau - K(q)^{T} \\lambda
```
and the constraint equations
```math
K(q) \\dot{v} = -k
```
given joint configuration vector ``q``, joint velocity vector ``v``, and
(optionally) joint torques ``\\tau`` and external wrenches ``w_\\text{ext}``.

The `externalwrenches` argument can be used to specify additional
wrenches that act on the `Mechanism`'s bodies.
"""
function dynamics!(result::DynamicsResult{T, M}, state::MechanismState{X, M},
        torques::AbstractVector{Tau} = ConstVector(zero(T), num_velocities(state)),
        externalwrenches::Associative{RigidBody{M}, Wrench{W}} = NullDict{RigidBody{M}, Wrench{T}}()) where {T, X, M, Tau, W}
    contact_dynamics!(result, state)
    for body in bodies(state.mechanism)
        contactwrench = result.contactwrenches[body]
        result.totalwrenches[body] = haskey(externalwrenches, body) ? externalwrenches[body] + contactwrench : contactwrench
    end
    dynamics_bias!(result, state)
    mass_matrix!(result, state)
    constraint_jacobian_and_bias!(result, state)
    dynamics_solve!(result, torques)
    nothing
end


"""
$(SIGNATURES)

Convenience function for use with standard ODE integrators that takes a
`Vector` argument
```math
x = \\left(\\begin{array}{c}
q\\\\
v
\\end{array}\\right)
```
and returns a `Vector` ``\\dot{x}``.
"""
function dynamics!(ẋ::StridedVector{X},
        result::DynamicsResult{T, M}, state::MechanismState{X, M}, stateVec::AbstractVector{X},
        torques::AbstractVector{Tau} = ConstVector(zero(T), num_velocities(state)),
        externalwrenches::Associative{RigidBody{M}, Wrench{W}} = NullDict{RigidBody{M}, Wrench{T}}()) where {T, X, M, Tau, W}
    set!(state, stateVec)
    nq = num_positions(state)
    nv = num_velocities(state)
    q̇ = view(ẋ, 1 : nq) # allocates
    v̇ = view(ẋ, nq + 1 : nq + nv) # allocates
    configuration_derivative!(q̇, state)
    dynamics!(result, state, torques, externalwrenches)
    copy!(v̇, result.v̇)
    ẋ
end
