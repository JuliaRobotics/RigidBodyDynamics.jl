const noalloc_doc = """This method does its computation in place, performing no dynamic memory allocation."""

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
basic, or spatial Jacobian) associated with a directed path in the `Mechanism`'s
spanning tree, (a collection of `Joint`s and traversal directions) in the given state.

A geometric Jacobian maps the `Mechanism`'s joint velocity vector ``v``
to the twist of the target of the joint path (the body succeeding the last joint
in the path) with respect to the source of the joint path (the body preceding the
first joint in the path).

See also [`path`](@ref), [`GeometricJacobian`](@ref), [`Twist`](@ref).
"""

"""
$(SIGNATURES)

$geometric_jacobian_doc

`transformfun` is a callable that may be used to transform the individual motion
subspaces of each of the joints to the frame in which `out` is expressed.

$noalloc_doc
"""
function geometric_jacobian!(jac::GeometricJacobian, state::MechanismState, path::TreePath, transformfun)
    @boundscheck num_velocities(state) == size(jac, 2) || error("size mismatch")
    @framecheck jac.body default_frame(target(path))
    @framecheck jac.base default_frame(source(path))
    update_motion_subspaces!(state)
    joints = state.type_sorted_tree_joints
    motion_subspaces = state.motion_subspaces.data
    discard = DiscardVector(length(joints))
    broadcast!(discard, jac, state, path, joints, motion_subspaces) do jac, state, path, joint, motion_subspace
        vrange = velocity_range(state, joint)
        pathindex = findfirst(joint, path)
        if pathindex > 0
            part = transformfun(motion_subspace)
            directions(path)[pathindex] == :up && (part = -part)
            set_cols!(jac, vrange, part)
        else
            zero_cols!(jac, vrange)
        end
    end
    jac
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
function geometric_jacobian(state::MechanismState{X, M, C}, path::TreePath{RigidBody{M}, Joint{M}}) where {X, M, C}
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
    update_motion_subspaces!(state)
    update_crb_inertias!(state)
    fill!(out.data, 0)
    # TODO: broadcast!
    foreach_with_extra_args(out, state, state.motion_subspaces.data, state.treejointids) do out, state, Si, jointidi # TODO: use closure once it doesn't allocate
        irange = velocity_range(state, jointidi)
        bodyid = successorid(jointidi, state)
        Ici = crb_inertia(state, bodyid)
        F = Ici * Si
        ancestor_joint_mask = state.ancestor_joint_masks[jointidi]
        foreach_with_extra_args(out, state, irange, F, ancestor_joint_mask, state.motion_subspaces.data, state.treejointids) do out, state, irange, F, ancestor_joint_mask, Sj, jointidj # TODO: use closure once it doesn't allocate
            Base.@_inline_meta # currently required; try removing with 1.0
            if ancestor_joint_mask[jointidj]
                jrange = velocity_range(state, jointidj)
                block = angular(F)' * angular(Sj) + linear(F)' * linear(Sj)
                set_matrix_block!(out.data, irange, jrange, block)
            end
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
    @boundscheck num_velocities(state) == size(out, 2) || error("size mismatch")
    update_motion_subspaces!(state)
    update_crb_inertias!(state)
    motion_subspaces = state.motion_subspaces.data
    discard = DiscardVector(length(motion_subspaces))
    broadcast!(discard, out, state, motion_subspaces, state.treejointids) do out, state, motion_subspace, jointid
        vrange = velocity_range(state, jointid)
        bodyid = successorid(jointid, state)
        inertia = crb_inertia(state, bodyid)
        part = transformfun(inertia * motion_subspace)
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
    momentum_matrix!(out, state, F -> transform(F, root_to_desired))
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

function bias_accelerations!(out::Associative{BodyID, SpatialAcceleration{T}}, state::MechanismState{X, M}) where {T, X, M}
    update_bias_accelerations_wrt_world!(state)
    gravitybias = convert(SpatialAcceleration{T}, -gravitational_spatial_acceleration(state.mechanism))
    for jointid in state.treejointids
        bodyid = successorid(jointid, state)
        out[bodyid] = gravitybias + bias_acceleration(state, bodyid)
    end
    nothing
end

function spatial_accelerations!(out::Associative{BodyID, SpatialAcceleration{T}},
        state::MechanismState{X, M}, v̇::SegmentedVector{JointID}) where {T, X, M}
    update_twists_wrt_world!(state)

    # Compute joint accelerations
    joints = state.type_sorted_tree_joints
    qs = values(segments(state.q))
    vs = values(segments(state.v))
    v̇s = values(segments(v̇))
    discard = DiscardVector(length(qs))
    broadcast!(discard, state, out, joints, qs, vs, v̇s) do state, accels, joint, qjoint, vjoint, v̇joint
        bodyid = successorid(id(joint), state)
        accels[bodyid] = joint_spatial_acceleration(joint, qjoint, vjoint, v̇joint)
    end

    # Recursive propagation
    # TODO: manual frame changes. Find a way to not avoid the frame checks here.
    mechanism = state.mechanism
    root = root_body(mechanism)
    out[root] = convert(SpatialAcceleration{T}, -gravitational_spatial_acceleration(mechanism))
    for jointid in state.treejointids
        parentbodyid, bodyid = predsucc(jointid, state)
        toroot = transform_to_root(state, bodyid)
        parenttwist = twist_wrt_world(state, parentbodyid)
        bodytwist = twist_wrt_world(state, bodyid)
        jointaccel = out[bodyid]
        jointaccel = SpatialAcceleration(jointaccel.body, parenttwist.body, toroot.to,
            Spatial.transform_spatial_motion(jointaccel.angular, jointaccel.linear, rotation(toroot), translation(toroot))...)
        out[bodyid] = out[parentbodyid] + (-bodytwist) × parenttwist + jointaccel
    end
    nothing
end

spatial_accelerations!(result::DynamicsResult, state::MechanismState) = spatial_accelerations!(result.accelerations, state, result.v̇)

function relative_acceleration(accels::Associative{BodyID, SpatialAcceleration{T}}, body::RigidBody{M}, base::RigidBody{M}) where {T, M}
    -accels[id(base)] + accels[id(body)]
end

# TODO: ensure that accelerations are up-to-date
relative_acceleration(result::DynamicsResult, body::RigidBody, base::RigidBody) = relative_acceleration(result.accelerations, body, base)

function relative_acceleration(state::MechanismState, body::RigidBody, base::RigidBody, v̇::AbstractVector)
    error("""`relative_acceleration(state, body, base, v̇)` has been removed.
    Use `spatial_accelerations!(result, state)` or `spatial_accelerations!(accels, state, v̇)` to compute the
    spatial accelerations of all bodies in one go, and then use `relative_acceleration(accels, body, base)` or
    `relative_acceleration(result, body, base)`.""")
end

function newton_euler!(
        out::Associative{BodyID, Wrench{T}}, state::MechanismState{X, M},
        accelerations::Associative{BodyID, SpatialAcceleration{T}},
        externalwrenches::Associative{BodyID, Wrench{W}}) where {T, X, M, W}
    update_twists_wrt_world!(state)
    update_spatial_inertias!(state)
    for jointid in state.treejointids
        bodyid = successorid(jointid, state)
        wrench = newton_euler(state, bodyid, accelerations[bodyid])
        out[bodyid] = haskey(externalwrenches, bodyid) ? wrench - externalwrenches[bodyid] : wrench
    end
end


function joint_wrenches_and_torques!(
        torquesout::SegmentedVector{JointID},
        net_wrenches_in_joint_wrenches_out::Associative{BodyID, <:Wrench}, # TODO: consider having a separate Associative{Joint{M}, Wrench{T}} for joint wrenches
        state::MechanismState)
    # Note: pass in net wrenches as wrenches argument. wrenches argument is modified to be joint wrenches
    @boundscheck length(torquesout) == num_velocities(state) || error("length of torque vector is wrong")

    for jointid in reverse(state.treejointids)
        parentbodyid, bodyid = predsucc(jointid, state)
        jointwrench = net_wrenches_in_joint_wrenches_out[bodyid]
        if parentbodyid != BodyID(1) # TODO: ugly
            # TODO: consider also doing this for the root:
            net_wrenches_in_joint_wrenches_out[parentbodyid] += jointwrench # action = -reaction
        end
    end

    joints = state.type_sorted_tree_joints
    qs = values(segments(state.q))
    τs = values(segments(torquesout))
    discard = DiscardVector(length(qs))
    broadcast!(discard, state, net_wrenches_in_joint_wrenches_out, joints, qs, τs) do state, wrenches, joint, qjoint, τjoint
        # TODO: awkward to transform back to body frame; consider switching to body-frame implementation
        bodyid = successorid(id(joint), state)
        tf = inv(transform_to_root(state, bodyid))
        joint_torque!(τjoint, joint, qjoint, transform(wrenches[bodyid], tf)) # TODO: consider using motion subspace
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
        torques::SegmentedVector{JointID},
        biasaccelerations::Associative{BodyID, <:SpatialAcceleration},
        wrenches::Associative{BodyID, <:Wrench},
        state::MechanismState{X},
        externalwrenches::Associative{BodyID, <:Wrench} = NullDict{BodyID, Wrench{X}}()) where X
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
        externalwrenches::Associative{BodyID, Wrench{W}} = NullDict{BodyID, Wrench{X}}()) where {X, M, W}
    T = promote_type(X, M, W)
    mechanism = state.mechanism
    torques = Vector{T}(num_velocities(state))
    rootframe = root_frame(mechanism)
    jointwrenches = BodyDict(id(b) => zero(Wrench{T}, rootframe) for b in bodies(mechanism))
    accelerations = BodyDict(id(b) => zero(SpatialAcceleration{T}, rootframe, rootframe, rootframe) for b in bodies(mechanism))
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
        torquesout::SegmentedVector{JointID},
        jointwrenchesout::Associative{BodyID, Wrench{T}},
        accelerations::Associative{BodyID, SpatialAcceleration{T}},
        state::MechanismState,
        v̇::SegmentedVector{JointID},
        externalwrenches::Associative{BodyID, <:Wrench} = NullDict{BodyID, Wrench{T}}()) where T
    @boundscheck length(tree_joints(state.mechanism)) == length(joints(state.mechanism)) || error("This method can currently only handle tree Mechanisms.")
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
        v̇::SegmentedVector{JointID, V},
        externalwrenches::Associative{BodyID, Wrench{W}} = NullDict{BodyID, Wrench{X}}()) where {X, M, V, W}
    T = promote_type(X, M, V, W)
    mechanism = state.mechanism
    torques = SegmentedVector(Vector{T}(num_velocities(state)), tree_joints(mechanism), num_velocities)
    rootframe = root_frame(mechanism)
    jointwrenches = BodyDict(id(b) => zero(Wrench{T}, rootframe) for b in bodies(mechanism))
    accelerations = BodyDict(id(b) => zero(SpatialAcceleration{T}, rootframe, rootframe, rootframe) for b in bodies(mechanism))
    inverse_dynamics!(torques, jointwrenches, accelerations, state, v̇, externalwrenches)
    torques
end

function constraint_jacobian!(jac::AbstractMatrix, state::MechanismState)
    # note: order of rows of Jacobian is determined by iteration order of state.type_sorted_non_tree_joints
    # TODO: traversing jac in the wrong order
    update_motion_subspaces!(state)
    update_constraint_wrench_subspaces!(state)
    rowstart = Ref(1) # TODO: allocation
    nontreejoints = state.type_sorted_non_tree_joints
    wrenchsubspaces = state.constraint_wrench_subspaces.data
    discard = DiscardVector(length(nontreejoints))
    broadcast!(discard, jac, state, rowstart, nontreejoints, wrenchsubspaces) do jac, state, rowstart, nontreejoint, T
        nontreejointid = id(nontreejoint)
        path = state.constraint_jacobian_structure[nontreejointid]
        nextrowstart = rowstart[] + num_constraints(nontreejoint)
        rowrange = rowstart[] : nextrowstart - 1
        nontreejoints = state.type_sorted_tree_joints
        motionsubspaces = state.motion_subspaces.data
        broadcast!(constraint_jacobian_inner!, DiscardVector(length(nontreejoints)), jac, rowrange, state, path, T, nontreejoints, motionsubspaces)
        rowstart[] = nextrowstart
    end
end

@inline function constraint_jacobian_inner!(jac, rowrange, state, path, T, treejoint, S)
            vrange = velocity_range(state, treejoint)
            pathindex = findfirst(treejoint, path)
            if pathindex > 0
                part = angular(T)' * angular(S) + linear(T)' * linear(S) # TODO: At_mul_B
                directions(path)[pathindex] == :up && (part = -part)
        set_matrix_block!(jac, rowrange, vrange, part)
            else
        zero_matrix_block!(jac, rowrange, vrange)
    end
end

constraint_jacobian!(result::DynamicsResult, state::MechanismState) = constraint_jacobian!(result.constraintjacobian, state)

function constraint_bias!(bias::SegmentedVector, state::MechanismState)
    # note: order of rows of Jacobian and bias term is determined by iteration order of state.type_sorted_non_tree_joints
    update_twists_wrt_world!(state)
    update_bias_accelerations_wrt_world!(state)
    update_constraint_wrench_subspaces!(state)
    rowstart = Ref(1) # TODO: allocation
    nontreejoints = state.type_sorted_non_tree_joints
    wrenchsubspaces = state.constraint_wrench_subspaces.data
    discard = DiscardVector(length(nontreejoints))
    broadcast!(discard, bias, state, rowstart,
            nontreejoints, wrenchsubspaces) do bias, state, rowstart, nontreejoint, T
        has_fixed_subspaces(nontreejoint) || error("Only joints with fixed motion subspace (Ṡ = 0) supported at this point.") # TODO: call to joint-type-specific function
        nontreejointid = id(nontreejoint)
        path = state.constraint_jacobian_structure[nontreejointid]
        nextrowstart = rowstart[] + num_constraints(nontreejoint)
        rowrange = rowstart[] : nextrowstart - 1
        kjoint = fastview(bias, rowrange)
        predid, succid = predsucc(nontreejointid, state)
        crossterm = cross(twist_wrt_world(state, succid), twist_wrt_world(state, predid))
        biasaccel = crossterm + (-bias_acceleration(state, predid) + bias_acceleration(state, succid)) # 8.47 in Featherstone
        At_mul_B!(kjoint, T, biasaccel)
        rowstart[] = nextrowstart
    end
    bias
end
constraint_bias!(result::DynamicsResult, state::MechanismState) = constraint_bias!(result.constraintbias, state)

function contact_dynamics!(result::DynamicsResult{T, M}, state::MechanismState{X, M, C}) where {X, M, C, T}
    update_twists_wrt_world!(state)
    mechanism = state.mechanism
    root = root_body(mechanism)
    frame = default_frame(root)
    for body in bodies(mechanism)
        bodyid = id(body)
        wrench = zero(Wrench{T}, frame)
        points = contact_points(body)
        if !isempty(points)
            # TODO: AABB
            body_to_root = transform_to_root(state, bodyid)
            twist = twist_wrt_world(state, bodyid)
            states_for_body = contact_states(state, bodyid)
            state_derivs_for_body = contact_state_derivatives(result, bodyid)
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
    c = parent(result.dynamicsbias)
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
    c = parent(result.dynamicsbias)
    v̇ = parent(result.v̇)

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
    τbiased = v̇
    τbiased .-= c

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
        z[:] = τbiased
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

        # Update τbiased: subtract Kᵀ * λ
        LinAlg.BLAS.gemv!('T', -one(T), K, λ, one(T), τbiased) # τbiased <- τ - c - Kᵀ * λ

        # Solve for v̇ = M⁻¹ * (τ - c - Kᵀ * λ)
        LinAlg.LAPACK.potrs!(uplo, L, τbiased) # τbiased ==v̇ <- M⁻¹ * (τ - c - Kᵀ * λ)
    else
        # No loops.
        LinAlg.LAPACK.potrs!(uplo, L, τbiased) # τbiased == v̇ <- M⁻¹ * (τ - c)
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
function dynamics!(result::DynamicsResult, state::MechanismState{X},
        torques::SegmentedVector{JointID} = ConstVector(zero(X), num_velocities(state)),
        externalwrenches::Associative{BodyID, <:Wrench} = NullDict{BodyID, Wrench{X}}()) where X
    contact_dynamics!(result, state)
    for jointid in state.treejointids
        bodyid = successorid(jointid, state)
        contactwrench = result.contactwrenches[bodyid]
        result.totalwrenches[bodyid] = haskey(externalwrenches, bodyid) ? externalwrenches[bodyid] + contactwrench : contactwrench
    end
    dynamics_bias!(result, state)
    mass_matrix!(result, state)
    if has_loops(state.mechanism)
        constraint_jacobian!(result, state)
        constraint_bias!(result, state)
    end
    dynamics_solve!(result, parent(torques))
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
        result::DynamicsResult, state::MechanismState{X}, state_vec::AbstractVector{X},
        torques::SegmentedVector{JointID} = ConstVector(zero(X), num_velocities(state)),
        externalwrenches::Associative{BodyID, <:Wrench} = NullDict{BodyID, Wrench{X}}()) where X
    set!(state, state_vec)
    nq = num_positions(state)
    nv = num_velocities(state)
    q̇ = SegmentedVector(view(ẋ, 1 : nq), tree_joints(state.mechanism), num_positions) # allocates
    v̇ = SegmentedVector(view(ẋ, nq + 1 : nq + nv), tree_joints(state.mechanism), num_positions) # allocates
    configuration_derivative!(q̇, state)
    dynamics!(result, state, torques, externalwrenches)
    copy!(v̇, result.v̇)
    ẋ
end
