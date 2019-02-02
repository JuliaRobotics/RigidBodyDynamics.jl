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
    update_transforms!(state)
    T = cache_eltype(state)
    mechanism = state.mechanism
    frame = root_frame(mechanism)
    com = Point3D(frame, zero(SVector{3, T}))
    mass = zero(T)
    for body in itr
        if body.inertia !== nothing
            inertia = spatial_inertia(body)
            if inertia.mass > 0
                bodycom = transform_to_root(state, body, false) * center_of_mass(inertia)
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
    @boundscheck num_velocities(state) == size(jac, 2) || throw(DimensionMismatch())
    @framecheck jac.body default_frame(target(path))
    @framecheck jac.base default_frame(source(path))
    update_motion_subspaces!(state)
    fill!(jac.angular, 0)
    fill!(jac.linear, 0)
    for i in eachindex(path.edges) # TODO: just use @inbounds here; currently messes with frame check in set_col!
        @inbounds joint = path.edges[i]
        vrange = velocity_range(state, joint)
        @inbounds direction = directions(path)[i]
        for col in eachindex(vrange)
            @inbounds vindex = vrange[col]
            @inbounds Scol = transformfun(state.motion_subspaces.data[vindex])
            direction == PathDirections.up && (Scol = -Scol)
            set_col!(jac, vindex, Scol)
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
    angular = Matrix{C}(undef, 3, nv)
    linear = Matrix{C}(undef, 3, nv)
    bodyframe = default_frame(target(path))
    baseframe = default_frame(source(path))
    jac = GeometricJacobian(bodyframe, baseframe, root_frame(state.mechanism), angular, linear)
    geometric_jacobian!(jac, state, path, identity)
end


const point_jacobian_doc = """
Compute the Jacobian mapping the `Mechanism`'s joint velocity vector ``v`` to
the velocity of a point fixed to the target of the joint path (the body
succeeding the last joint in the path) with respect to the source of the joint
path (the body preceding the first joint in the path).
"""

"""
$(SIGNATURES)

$point_jacobian_doc

$noalloc_doc
"""
function _point_jacobian!(Jp::PointJacobian, state::MechanismState, path::TreePath,
                         point::Point3D, transformfun)
    @framecheck Jp.frame point.frame
    update_motion_subspaces!(state)
    fill!(Jp.J, 0)
    p̂ = Spatial.hat(point.v)
    @inbounds for i in eachindex(path.edges)
        joint = path.edges[i]
        vrange = velocity_range(state, joint)
        direction = path.directions[i]
        for col in eachindex(vrange)
            vindex = vrange[col]
            Scol = transformfun(state.motion_subspaces.data[vindex])
            direction == PathDirections.up && (Scol = -Scol)
            newcol =  -p̂ * angular(Scol) + linear(Scol)
            for row in 1:3
                Jp.J[row, vindex] = newcol[row]
            end
        end
    end
    Jp
end

"""
$(SIGNATURES)

$point_jacobian_doc

Uses `state` to compute the transform from the `Mechanism`'s root frame to the
frame in which `out` is expressed if necessary.

$noalloc_doc
"""
function point_jacobian!(out::PointJacobian, state::MechanismState, path::TreePath, point::Point3D)
    if out.frame == root_frame(state.mechanism)
        _point_jacobian!(out, state, path, point, identity)
    else
        root_to_desired = inv(transform_to_root(state, out.frame))
        let root_to_desired = root_to_desired
            _point_jacobian!(out, state, path, point, S -> transform(S, root_to_desired))
        end
    end
end


"""
$(SIGNATURES)

$point_jacobian_doc
"""
function point_jacobian(state::MechanismState{X, M, C},
                        path::TreePath{RigidBody{M}, Joint{M}},
                        point::Point3D) where {X, M, C}
    Jp = PointJacobian(point.frame, Matrix{C}(undef, 3, num_velocities(state)))
    point_jacobian!(Jp, state, path, point)
    Jp
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
function mass_matrix!(M::Symmetric, state::MechanismState)
    nv = num_velocities(state)
    @boundscheck size(M, 1) == nv || throw(DimensionMismatch("mass matrix has wrong size"))
    @boundscheck M.uplo == 'L' || throw(ArgumentError(("expected a lower triangular symmetric matrix")))
    update_motion_subspaces!(state)
    update_crb_inertias!(state)
    motion_subspaces = state.motion_subspaces.data
    @inbounds for i in Base.OneTo(nv)
        jointi = velocity_index_to_joint_id(state, i)
        bodyi = successorid(jointi, state)
        Ici = crb_inertia(state, bodyi, false)
        Si = motion_subspaces[i]
        Fi = Ici * Si
        for j in Base.OneTo(i)
            jointj = velocity_index_to_joint_id(state, j)
            M.data[i, j] = if supports(jointj, bodyi, state)
                Sj = motion_subspaces[j]
                (transpose(Fi) * Sj)[1]
            else
                zero(eltype(M))
            end
        end
    end
    M
end

mass_matrix!(result::DynamicsResult, state::MechanismState) = mass_matrix!(result.massmatrix, state)

"""
$(SIGNATURES)

$mass_matrix_doc
"""
function mass_matrix(state::MechanismState{X, M, C}) where {X, M, C}
    nv = num_velocities(state)
    ret = Symmetric(Matrix{C}(undef, nv, nv), :L)
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
function momentum_matrix!(mat::MomentumMatrix, state::MechanismState, transformfun)
    nv = num_velocities(state)
    @boundscheck size(mat, 2) == nv || throw(DimensionMismatch())
    update_motion_subspaces!(state)
    update_crb_inertias!(state)
    @inbounds for i in 1 : nv
        jointi = velocity_index_to_joint_id(state, i)
        bodyi = successorid(jointi, state)
        Ici = crb_inertia(state, bodyi)
        Si = state.motion_subspaces.data[i]
        Fi = transformfun(Ici * Si)
        set_col!(mat, i, Fi)
    end
    mat
end

"""
$(SIGNATURES)

$momentum_matrix!_doc

`root_to_desired` is the transform from the `Mechanism`'s root frame to the frame
in which `out` is expressed.

$noalloc_doc
"""
function momentum_matrix!(mat::MomentumMatrix, state::MechanismState, root_to_desired::Transform3D)
    momentum_matrix!(mat, state, F -> transform(F, root_to_desired))
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
    ret = MomentumMatrix(root_frame(state.mechanism), Matrix{T}(undef, 3, ncols), Matrix{T}(undef, 3, ncols))
    momentum_matrix!(ret, state, identity)
    ret
end

function bias_accelerations!(out::AbstractDict{BodyID, SpatialAcceleration{T}}, state::MechanismState{X, M}) where {T, X, M}
    update_bias_accelerations_wrt_world!(state)
    gravitybias = convert(SpatialAcceleration{T}, -gravitational_spatial_acceleration(state.mechanism))
    for jointid in state.treejointids
        bodyid = successorid(jointid, state)
        out[bodyid] = gravitybias + bias_acceleration(state, bodyid, false)
    end
    nothing
end

function spatial_accelerations!(out::AbstractDict{BodyID, SpatialAcceleration{T}}, state::MechanismState, v̇::SegmentedVector{JointID}) where T
    update_transforms!(state)
    update_twists_wrt_world!(state)

    # Compute joint accelerations
    joints = state.treejoints
    qs = values(segments(state.q))
    vs = values(segments(state.v))
    v̇s = values(segments(v̇))
    foreach_with_extra_args(state, out, joints, qs, vs, v̇s) do state, accels, joint, qjoint, vjoint, v̇joint
        bodyid = successorid(JointID(joint), state)
        accels[bodyid] = joint_spatial_acceleration(joint, qjoint, vjoint, v̇joint)
    end

    # Recursive propagation
    # TODO: manual frame changes. Find a way to not avoid the frame checks here.
    mechanism = state.mechanism
    root = root_body(mechanism)
    out[root] = convert(SpatialAcceleration{T}, -gravitational_spatial_acceleration(mechanism))
    @inbounds for jointid in state.treejointids
        parentbodyid, bodyid = predsucc(jointid, state)
        toroot = transform_to_root(state, bodyid, false)
        parenttwist = twist_wrt_world(state, parentbodyid, false)
        bodytwist = twist_wrt_world(state, bodyid, false)
        jointaccel = out[bodyid]
        jointaccel = SpatialAcceleration(jointaccel.body, parenttwist.body, toroot.to,
            Spatial.transform_spatial_motion(jointaccel.angular, jointaccel.linear, rotation(toroot), translation(toroot))...)
        out[bodyid] = out[parentbodyid] + (-bodytwist) × parenttwist + jointaccel
    end
    nothing
end

spatial_accelerations!(result::DynamicsResult, state::MechanismState) = spatial_accelerations!(result.accelerations, state, result.v̇)

function relative_acceleration(accels::AbstractDict{BodyID, SpatialAcceleration{T}}, body::RigidBody{M}, base::RigidBody{M}) where {T, M}
    -accels[BodyID(base)] + accels[BodyID(body)]
end

# TODO: ensure that accelerations are up-to-date
relative_acceleration(result::DynamicsResult, body::RigidBody, base::RigidBody) = relative_acceleration(result.accelerations, body, base)

function newton_euler!(
        out::AbstractDict{BodyID, Wrench{T}}, state::MechanismState{X, M},
        accelerations::AbstractDict{BodyID, SpatialAcceleration{T}},
        externalwrenches::AbstractDict{BodyID, Wrench{W}}) where {T, X, M, W}
    update_twists_wrt_world!(state)
    update_spatial_inertias!(state)
    for jointid in state.treejointids
        bodyid = successorid(jointid, state)
        wrench = newton_euler(state, bodyid, accelerations[bodyid], false)
        out[bodyid] = haskey(externalwrenches, bodyid) ? wrench - externalwrenches[bodyid] : wrench
    end
end


function joint_wrenches_and_torques!(
        torquesout::SegmentedVector{JointID},
        net_wrenches_in_joint_wrenches_out::AbstractDict{BodyID, <:Wrench}, # TODO: consider having a separate Associative{Joint{M}, Wrench{T}} for joint wrenches
        state::MechanismState)
    update_motion_subspaces!(state)
    # Note: pass in net wrenches as wrenches argument. wrenches argument is modified to be joint wrenches
    @boundscheck length(torquesout) == num_velocities(state) || error("length of torque vector is wrong")
    wrenches = net_wrenches_in_joint_wrenches_out
    for jointid in reverse(state.treejointids)
        parentbodyid, bodyid = predsucc(jointid, state)
        jointwrench = wrenches[bodyid]
        wrenches[parentbodyid] += jointwrench # action = -reaction
        for vindex in velocity_range(state, jointid)
            Scol = state.motion_subspaces.data[vindex]
            torquesout[vindex] = (transpose(Scol) * jointwrench)[1]
        end
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
        biasaccelerations::AbstractDict{BodyID, <:SpatialAcceleration},
        wrenches::AbstractDict{BodyID, <:Wrench},
        state::MechanismState{X},
        externalwrenches::AbstractDict{BodyID, <:Wrench} = NullDict{BodyID, Wrench{X}}()) where X
    bias_accelerations!(biasaccelerations, state)
    newton_euler!(wrenches, state, biasaccelerations, externalwrenches)
    joint_wrenches_and_torques!(torques, wrenches, state)
    torques
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
        externalwrenches::AbstractDict{BodyID, Wrench{W}} = NullDict{BodyID, Wrench{X}}()) where {X, M, W}
    T = promote_type(X, M, W)
    mechanism = state.mechanism
    torques = similar(velocity(state), T)
    rootframe = root_frame(mechanism)
    jointwrenches = BodyDict(BodyID(b) => zero(Wrench{T}, rootframe) for b in bodies(mechanism))
    accelerations = BodyDict(BodyID(b) => zero(SpatialAcceleration{T}, rootframe, rootframe, rootframe) for b in bodies(mechanism))
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
        jointwrenchesout::AbstractDict{BodyID, Wrench{T}},
        accelerations::AbstractDict{BodyID, SpatialAcceleration{T}},
        state::MechanismState,
        v̇::SegmentedVector{JointID},
        externalwrenches::AbstractDict{BodyID, <:Wrench} = NullDict{BodyID, Wrench{T}}()) where T
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
        externalwrenches::AbstractDict{BodyID, Wrench{W}} = NullDict{BodyID, Wrench{X}}()) where {X, M, V, W}
    T = promote_type(X, M, V, W)
    mechanism = state.mechanism
    torques = similar(velocity(state), T)
    rootframe = root_frame(mechanism)
    jointwrenches = BodyDict(BodyID(b) => zero(Wrench{T}, rootframe) for b in bodies(mechanism))
    accelerations = BodyDict(BodyID(b) => zero(SpatialAcceleration{T}, rootframe, rootframe, rootframe) for b in bodies(mechanism))
    inverse_dynamics!(torques, jointwrenches, accelerations, state, v̇, externalwrenches)
    torques
end

function constraint_jacobian!(jac::AbstractMatrix, rowranges, state::MechanismState)
    # TODO: traversing jac in the wrong order
    update_motion_subspaces!(state)
    update_constraint_wrench_subspaces!(state)
    fill!(jac, 0)
    @inbounds for nontreejointid in state.nontreejointids
        path = state.constraint_jacobian_structure[nontreejointid]
        for cindex in constraint_range(state, nontreejointid)
            Tcol = state.constraint_wrench_subspaces.data[cindex]
            for i in eachindex(path.edges)
                treejoint = path.edges[i]
                vrange = velocity_range(state, treejoint)
                direction = directions(path)[i]
                sign = ifelse(direction == PathDirections.up, -1, 1)
                for col in eachindex(vrange)
                    vindex = vrange[col]
                    Scol = state.motion_subspaces.data[vindex]
                    jacelement = flipsign((transpose(Tcol) * Scol)[1], sign)
                    jac[cindex, vindex] = jacelement
                end
            end
        end
    end
    jac
end

function constraint_jacobian!(result::DynamicsResult, state::MechanismState)
    constraint_jacobian!(result.constraintjacobian, result.constraintrowranges, state)
end

"""
$(SIGNATURES)

Return the default Baumgarte constraint stabilization gains. These gains result in
critical damping, and correspond to ``T_{stab} = 0.1`` in Featherstone (2008), section 8.3.
"""
function default_constraint_stabilization_gains(scalar_type::Type{T}) where T
    ConstDict{JointID}(SE3PDGains(PDGains(T(100), T(20)), PDGains(T(100), T(20))))
end

const stabilization_gains_doc = """
The `stabilization_gains` keyword argument can be used to set PD gains for Baumgarte
stabilization, which can be used to prevent separation of non-tree (loop) joints.
See Featherstone (2008), section 8.3 for more information. There are several options
for specifying gains:

* `nothing` can be used to completely disable Baumgarte stabilization.
* Gains can be specifed on a per-joint basis using any `AbstractDict{JointID, <:RigidBodyDynamics.PDControl.SE3PDGains}`,
  which maps the `JointID` for the non-tree joints of the mechanism to the gains for that joint.
* As a special case of the second option, the same gains can be used for all joints
  by passing in a `RigidBodyDynamics.CustomCollections.ConstDict{JointID}`.

The [`default_constraint_stabilization_gains`](@ref) function is called to produce the default gains,
which use the last option.
"""

function constraint_bias!(bias::SegmentedVector, state::MechanismState{X};
        stabilization_gains::Union{AbstractDict{JointID, <:SE3PDGains}, Nothing}=default_constraint_stabilization_gains(X)) where X
    update_transforms!(state)
    update_twists_wrt_world!(state)
    update_bias_accelerations_wrt_world!(state)
    update_constraint_wrench_subspaces!(state)
    constraint_wrench_subspaces = state.constraint_wrench_subspaces.data
    for nontreejointid in state.nontreejointids
        predid, succid = predsucc(nontreejointid, state)

        predtwist = twist_wrt_world(state, predid, false)
        succtwist = twist_wrt_world(state, succid, false)
        crossterm = succtwist × predtwist

        succbias = bias_acceleration(state, succid, false)
        predbias = bias_acceleration(state, predid, false)
        jointbias = -predbias + succbias

        biasaccel = crossterm + jointbias # what's inside parentheses in 8.47 in Featherstone

        if stabilization_gains !== nothing
            # TODO: make this nicer (less manual frame juggling and no manual transformations)
            jointtransform = joint_transform(state, nontreejointid, false)
            jointtwist = -predtwist + succtwist
            jointtwist = Twist(jointtransform.from, jointtransform.to, jointtwist.frame, jointtwist.angular, jointtwist.linear) # make frames line up
            successor_to_root = transform_to_root(state, jointtransform.from) # TODO: expensive
            jointtwist = transform(jointtwist, inv(successor_to_root)) # twist needs to be in successor frame for pd method
            stabaccel = pd(stabilization_gains[nontreejointid], jointtransform, jointtwist, SE3PDMethod{:Linearized}()) # in successor frame
            stabaccel = SpatialAcceleration(stabaccel.body, stabaccel.base, biasaccel.frame,
                Spatial.transform_spatial_motion(stabaccel.angular, stabaccel.linear,
                rotation(successor_to_root), translation(successor_to_root))...) # back to world frame. TODO: ugly way to do this
            stabaccel = SpatialAcceleration(biasaccel.body, biasaccel.body, stabaccel.frame, stabaccel.angular, stabaccel.linear) # make frames line up
            @inbounds biasaccel = biasaccel + -stabaccel
        end

        for cindex in constraint_range(state, nontreejointid)
            Tcol = constraint_wrench_subspaces[cindex]
            # TODO: make nicer:
            @framecheck Tcol.frame biasaccel.frame
            bias[cindex] = (transpose(Tcol.angular) * biasaccel.angular + transpose(Tcol.linear) * biasaccel.linear)[1]
        end
    end
    bias
end

function constraint_bias!(result::DynamicsResult, state::MechanismState{X};
        stabilization_gains::Union{AbstractDict{JointID, <:SE3PDGains}, Nothing}=default_constraint_stabilization_gains(X)) where X
    constraint_bias!(result.constraintbias, state; stabilization_gains=stabilization_gains)
end

function contact_dynamics!(result::DynamicsResult{T, M}, state::MechanismState{X, M, C}) where {X, M, C, T}
    update_transforms!(state)
    update_twists_wrt_world!(state)
    mechanism = state.mechanism
    root = root_body(mechanism)
    frame = default_frame(root)
    for body in bodies(mechanism)
        bodyid = BodyID(body)
        wrench = zero(Wrench{T}, frame)
        points = contact_points(body)
        if !isempty(points)
            # TODO: AABB
            body_to_root = transform_to_root(state, bodyid, false)
            twist = twist_wrt_world(state, bodyid, false)
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
    v̇ = parent(result.v̇)

    K = result.constraintjacobian
    k = result.constraintbias
    λ = result.λ

    nv = size(M, 1)
    nl = size(K, 1)
    G = [Matrix(M) K'; # TODO: Matrix because of https://github.com/JuliaLang/julia/issues/21332
         K zeros(nl, nl)]
    r = [τ - c; -k]
    v̇λ = G \ r
    v̇ .= view(v̇λ, 1 : nv)
    λ .= view(v̇λ, nv + 1 : nv + nl)
    nothing
end

function dynamics_solve!(result::DynamicsResult{T, S}, τ::AbstractVector{T}) where {S, T<:LinearAlgebra.BlasReal}
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

    L .= M.data
    uplo = M.uplo
    LinearAlgebra.LAPACK.potrf!(uplo, L) # L <- Cholesky decomposition of M; M == L Lᵀ (note: Featherstone, page 151 uses M == Lᵀ L instead)
    τbiased = v̇
    τbiased .= τ .- c

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
        Y .= K
        LinearAlgebra.BLAS.trsm!('R', uplo, 'T', 'N', one(T), L, Y)

        # Compute z = L⁻¹ (τ - c)
        z .= τbiased
        LinearAlgebra.BLAS.trsv!(uplo, 'N', 'N', L, z) # z <- L⁻¹ (τ - c)

        # Compute A = Y Yᵀ == K * M⁻¹ * Kᵀ
        LinearAlgebra.BLAS.gemm!('N', 'T', one(T), Y, Y, zero(T), A) # A <- K * M⁻¹ * Kᵀ

        # Compute b = Y z + k
        b = λ
        b .= k
        LinearAlgebra.BLAS.gemv!('N', one(T), Y, z, one(T), b) # b <- Y z + k

        # Compute λ = A⁻¹ b == (K * M⁻¹ * Kᵀ)⁻¹ * (K * M⁻¹ * (τ - c) + k)
        # LinearAlgebra.LAPACK.posv!(uplo, A, b) # NOTE: doesn't work in general because A is only guaranteed to be positive semidefinite
        singular_value_zero_tolerance = 1e-10 # TODO: more principled choice
        # TODO: https://github.com/JuliaLang/julia/issues/22242
        b[:], _ = LinearAlgebra.LAPACK.gelsy!(A, b, singular_value_zero_tolerance) # b == λ <- (K * M⁻¹ * Kᵀ)⁻¹ * (K * M⁻¹ * (τ - c) + k)

        # Update τbiased: subtract Kᵀ * λ
        LinearAlgebra.BLAS.gemv!('T', -one(T), K, λ, one(T), τbiased) # τbiased <- τ - c - Kᵀ * λ

        # Solve for v̇ = M⁻¹ * (τ - c - Kᵀ * λ)
        LinearAlgebra.LAPACK.potrs!(uplo, L, τbiased) # τbiased ==v̇ <- M⁻¹ * (τ - c - Kᵀ * λ)
    else
        # No loops.
        LinearAlgebra.LAPACK.potrs!(uplo, L, τbiased) # τbiased == v̇ <- M⁻¹ * (τ - c)
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

$stabilization_gains_doc
"""
function dynamics!(result::DynamicsResult, state::MechanismState{X},
        torques::AbstractVector = ConstVector(zero(X), num_velocities(state)),
        externalwrenches::AbstractDict{BodyID, <:Wrench} = NullDict{BodyID, Wrench{X}}();
        stabilization_gains=default_constraint_stabilization_gains(X)) where X
    configuration_derivative!(result.q̇, state)
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
        constraint_bias!(result, state; stabilization_gains=stabilization_gains)
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
        result::DynamicsResult, state::MechanismState{X}, x::AbstractVector{X},
        torques::AbstractVector = ConstVector(zero(X), num_velocities(state)),
        externalwrenches::AbstractDict{BodyID, <:Wrench} = NullDict{BodyID, Wrench{X}}();
        stabilization_gains=default_constraint_stabilization_gains(X)) where X
    copyto!(state, x)
    dynamics!(result, state, torques, externalwrenches; stabilization_gains=stabilization_gains)
    copyto!(ẋ, result)
    ẋ
end
