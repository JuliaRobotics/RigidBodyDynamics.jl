function transform(state::MechanismState, point::Point3D, to::CartesianFrame3D)
    point.frame == to && return point # nothing to be done
    relative_transform(state, point.frame, to) * point
end

function transform(state::MechanismState, vector::FreeVector3D, to::CartesianFrame3D)
    vector.frame == to && return vector # nothing to be done
    relative_transform(state, vector.frame, to) * vector
end

function transform(state::MechanismState, twist::Twist, to::CartesianFrame3D)
    twist.frame == to && return twist # nothing to be done
    transform(twist, relative_transform(state, twist.frame, to))
end

function transform(state::MechanismState, wrench::Wrench, to::CartesianFrame3D)
    wrench.frame == to && return wrench # nothing to be done
    transform(wrench, relative_transform(state, wrench.frame, to))
end

function transform(state::MechanismState, accel::SpatialAcceleration, to::CartesianFrame3D)
    accel.frame == to && return accel # nothing to be done
    oldToRoot = transform_to_root(state, accel.frame)
    rootToOld = inv(oldToRoot)
    twistOfBodyWrtBase = transform(relative_twist(state, accel.body, accel.base), rootToOld)
    twistOfOldWrtNew = transform(relative_twist(state, accel.frame, to), rootToOld)
    oldToNew = inv(transform_to_root(state, to)) * oldToRoot
    transform(accel, oldToNew, twistOfOldWrtNew, twistOfBodyWrtBase)
end


function subtree_mass{T}(base::Tree{RigidBody{T}, Joint{T}})
    result = isroot(base) ? zero(T) : spatial_inertia(vertex_data(base)).mass
    for child in children(base)
        result += subtree_mass(child)
    end
    result
end
mass(m::Mechanism) = subtree_mass(tree(m))
mass(state::MechanismState) = mass(state.mechanism)

function center_of_mass{X, M, C}(state::MechanismState{X, M, C}, itr)
    frame = root_body(state.mechanism).frame
    com = Point3D(frame, zeros(SVector{3, C}))
    mass = zero(C)
    for body in itr
        inertia = spatial_inertia(body)
        if inertia.mass > zero(C)
            bodyCom = center_of_mass(inertia)
            com += inertia.mass * FreeVector3D(transform(state, bodyCom, frame))
            mass += inertia.mass
        end
    end
    com /= mass
    com
end

center_of_mass(state::MechanismState) = center_of_mass(state, non_root_bodies(state.mechanism))

function geometric_jacobian{X, M, C}(state::MechanismState{X, M, C}, path::Path{RigidBody{M}, Joint{M}})
    copysign = (motionSubspace::GeometricJacobian, sign::Int64) -> sign < 0 ? -motionSubspace : motionSubspace
    motionSubspaces = [copysign(motion_subspace(state, joint), sign)::GeometricJacobian for (joint, sign) in zip(path.edgeData, path.directions)]
    hcat(motionSubspaces...)
end

function acceleration_wrt_ancestor{X, M, C, V}(state::MechanismState{X, M, C},
        descendant::TreeVertex{RigidBody{M}, Joint{M}},
        ancestor::TreeVertex{RigidBody{M}, Joint{M}},
        v̇::AbstractVector{V})
    mechanism = state.mechanism
    T = promote_type(C, V)
    descendantFrame = default_frame(mechanism, vertex_data(descendant))
    accel = zero(SpatialAcceleration{T}, descendantFrame, descendantFrame, root_frame(mechanism))
    descendant == ancestor && return accel

    current = descendant
    while current != ancestor
        joint = edge_to_parent_data(current)
        v̇joint = view(v̇, mechanism.vRanges[joint]) # TODO: allocates
        jointAccel = SpatialAcceleration(motion_subspace(state, joint), v̇joint)
        accel = jointAccel + accel
        current = parent(current)
    end
    -bias_acceleration(state, vertex_data(ancestor)) + bias_acceleration(state, vertex_data(descendant)) + accel
end

function relative_acceleration(state::MechanismState, body::RigidBody, base::RigidBody, v̇::AbstractVector)
    bodyVertex = findfirst(tree(state.mechanism), body)
    baseVertex = findfirst(tree(state.mechanism), base)
    lca = lowest_common_ancestor(baseVertex, bodyVertex)
    -acceleration_wrt_ancestor(state, baseVertex, lca, v̇) + acceleration_wrt_ancestor(state, bodyVertex, lca, v̇)
end

kinetic_energy{X, M}(state::MechanismState{X, M}, body::RigidBody{M}) = kinetic_energy(spatial_inertia(state, body), twist_wrt_world(state, body))
kinetic_energy{X, M}(state::MechanismState{X, M}, itr) = sum(body::RigidBody -> kinetic_energy(state, body), itr)
kinetic_energy(state::MechanismState) = kinetic_energy(state, non_root_bodies(state.mechanism))

function potential_energy{X, M, C}(state::MechanismState{X, M, C})
    m = mass(state.mechanism)
    gravitationalForce = m * state.mechanism.gravitationalAcceleration
    centerOfMass = transform(state, center_of_mass(state), gravitationalForce.frame)
    -dot(gravitationalForce, FreeVector3D(centerOfMass))
 end

 function _mass_matrix_part!(out::Symmetric, rowstart::Int64, colstart::Int64, jac::GeometricJacobian, mat::MomentumMatrix)
    # more efficient version of
    # @view out[rowstart : rowstart + n - 1, colstart : colstart + n - 1] = jac.angular' * mat.angular + jac.linear' * mat.linear
    n = num_cols(jac)
    m = num_cols(mat)
    @boundscheck (rowstart > 0 && rowstart + n - 1 <= size(out, 1)) || error("size mismatch")
    @boundscheck (colstart > 0 && colstart + m - 1 <= size(out, 2)) || error("size mismatch")
    framecheck(jac.frame, mat.frame)

    for row = 1 : n
        @simd for col = 1 : m
            outrow = rowstart + row - 1
            outcol = colstart + col - 1
            @inbounds begin
                out.data[outrow, outcol] =
                    jac.angular[1, row] * mat.angular[1, col] +
                    jac.angular[2, row] * mat.angular[2, col] +
                    jac.angular[3, row] * mat.angular[3, col] +
                    jac.linear[1, row] * mat.linear[1, col] +
                    jac.linear[2, row] * mat.linear[2, col] +
                    jac.linear[3, row] * mat.linear[3, col]
            end
        end
    end
 end

function mass_matrix!{X, M, C}(out::Symmetric{C, Matrix{C}}, state::MechanismState{X, M, C})
    @boundscheck size(out, 1) == num_velocities(state) || error("mass matrix has wrong size")
    @assert out.uplo == 'U'
    fill!(out.data, zero(C))
    mechanism = state.mechanism

    for vi in filter(v -> !isroot(v), state.toposortedStateVertices)
        # Hii
        jointi = edge_to_parent_data(vi)
        irange = velocity_range(jointi)
        if length(irange) > 0
            Si = motion_subspace(vi)
            Ii = crb_inertia(vi)
            F = Ii * Si
            istart = first(irange)
            _mass_matrix_part!(out, istart, istart, Si, F)

            # Hji, Hij
            vj = parent(vi)
            while (!isroot(vj))
                jointj = edge_to_parent_data(vj)
                jrange = velocity_range(jointj)
                if length(jrange) > 0
                    Sj = motion_subspace(vj)
                    jstart = first(jrange)
                    _mass_matrix_part!(out, jstart, istart, Sj, F)
                end
                vj = parent(vj)
            end
        end
    end
end

function mass_matrix{X, M, C}(state::MechanismState{X, M, C})
    nv = num_velocities(state)
    ret = Symmetric(Matrix{C}(nv, nv))
    mass_matrix!(ret, state)
    ret
end

# TODO: make more efficient:
momentum(state::MechanismState, body::RigidBody) = momentum(state_vertex(state, body))
momentum(state::MechanismState, body_itr) = sum(momentum(state, body) for body in body_itr)
momentum(state::MechanismState) = sum(momentum, non_root_vertices(state))

# TODO: make more efficient:
function momentum_matrix(state::MechanismState)
    hcat([crb_inertia(state, vertex_data(vertex)) * motion_subspace(state, edge_to_parent_data(vertex)) for vertex in non_root_vertices(state.mechanism)]...)
end

momentum_rate_bias(state::MechanismState, body::RigidBody) = newton_euler(state_vertex(state, body))
momentum_rate_bias(state, body_itr) = sum(momentum_rate_bias(state, body) for body in body_itr)
momentum_rate_bias(state::MechanismState) = sum(momentum_rate_bias, non_root_vertices(state))

# FIXME: remove:
newton_euler(state::MechanismState, body::RigidBody, accel::SpatialAcceleration) = newton_euler(state_vertex(state, body), accel)

function bias_accelerations!{T, X, M}(out::Associative{RigidBody{M}, SpatialAcceleration{T}}, state::MechanismState{X, M})
    gravityBias = convert(SpatialAcceleration{T}, -gravitational_spatial_acceleration(state.mechanism))
    for vertex in non_root_vertices(state)
        body = vertex_data(vertex).body
        out[body] = gravityBias + bias_acceleration(vertex)
    end
    nothing
end

function spatial_accelerations!{T, X, M}(out::Associative{RigidBody{M}, SpatialAcceleration{T}}, state::MechanismState{X, M}, v̇::AbstractVector)
    mechanism = state.mechanism
    vertices = mechanism.toposortedTree

    # unbiased joint accelerations + gravity
    rootBody = vertex_data(vertices[1])
    out[rootBody] = convert(SpatialAcceleration{T}, -gravitational_spatial_acceleration(mechanism))
    for vertex in non_root_vertices(mechanism)
        body = vertex_data(vertex)
        joint = edge_to_parent_data(vertex)
        S = motion_subspace(state, joint)
        @inbounds v̇joint = view(v̇, mechanism.vRanges[joint]) # TODO: allocates
        jointAccel = SpatialAcceleration(S, v̇joint)
        out[body] = out[vertex_data(parent(vertex))] + jointAccel
    end

    # add bias acceleration - gravity
    for vertex in non_root_vertices(mechanism)
        body = vertex_data(vertex)
        out[body] += bias_acceleration(state, body)
    end
    nothing
end

function newton_euler!{T, X, M, W}(
        out::Associative{RigidBody{M}, Wrench{T}}, state::MechanismState{X, M},
        accelerations::Associative{RigidBody{M}, SpatialAcceleration{T}},
        externalWrenches::Associative{RigidBody{M}, Wrench{W}} = NullDict{RigidBody{M}, Wrench{T}}())

    mechanism = state.mechanism
    vertices = mechanism.toposortedTree
    for body in non_root_bodies(mechanism)
        wrench = newton_euler(state, body, accelerations[body])
        if haskey(externalWrenches, body)
            wrench -= transform(state, externalWrenches[body], wrench.frame)
        end
        out[body] = wrench
    end
end

"""
Note: pass in net wrenches as wrenches argument. wrenches argument is modified to be joint wrenches
"""
function joint_wrenches_and_torques!{T, X, M}(
        torquesOut::AbstractVector{T},
        netWrenchesInJointWrenchesOut::Associative{RigidBody{M}, Wrench{T}},
        state::MechanismState{X, M})
    @boundscheck length(torquesOut) == num_velocities(state) || error("torquesOut size is wrong")
    mechanism = state.mechanism
    vertices = mechanism.toposortedTree
    for i = length(vertices) : -1 : 2
        vertex = vertices[i]
        joint = edge_to_parent_data(vertex)
        body = vertex_data(vertex)
        jointWrench = netWrenchesInJointWrenchesOut[body]
        if !isroot(parent(vertex))
            parentBody = vertex_data(parent(vertex))
            netWrenchesInJointWrenchesOut[parentBody] = netWrenchesInJointWrenchesOut[parentBody] + jointWrench # action = -reaction
        end
        jointWrench = transform(state, jointWrench, joint.frameAfter)
        @inbounds τjoint = view(torquesOut, mechanism.vRanges[joint]) # TODO: allocates
        joint_torque!(joint, τjoint, configuration(state, joint), jointWrench)
    end
end

function dynamics_bias!{T, X, M, W}(
        torques::AbstractVector{T},
        biasAccelerations::Associative{RigidBody{M}, SpatialAcceleration{T}},
        wrenches::Associative{RigidBody{M}, Wrench{T}},
        state::MechanismState{X, M},
        externalWrenches::Associative{RigidBody{M}, Wrench{W}} = NullDict{RigidBody{M}, Wrench{T}}())

    bias_accelerations!(biasAccelerations, state)
    newton_euler!(wrenches, state, biasAccelerations, externalWrenches)
    joint_wrenches_and_torques!(torques, wrenches, state)
end

function inverse_dynamics!{T, X, M, V, W}(
        torquesOut::AbstractVector{T},
        jointWrenchesOut::Associative{RigidBody{M}, Wrench{T}},
        accelerations::Associative{RigidBody{M}, SpatialAcceleration{T}},
        state::MechanismState{X, M},
        v̇::AbstractVector{V},
        externalWrenches::Associative{RigidBody{M}, Wrench{W}} = NullDict{RigidBody{M}, Wrench{T}}())
    spatial_accelerations!(accelerations, state, v̇)
    newton_euler!(jointWrenchesOut, state, accelerations, externalWrenches)
    joint_wrenches_and_torques!(torquesOut, jointWrenchesOut, state)
end

# note: lots of allocations, preallocate stuff and use inverse_dynamics! for performance
function inverse_dynamics{X, M, V, W}(
        state::MechanismState{X, M},
        v̇::AbstractVector{V},
        externalWrenches::Associative{RigidBody{M}, Wrench{W}} = NullDict{RigidBody{M}, Wrench{X}}())

    T = promote_type(X, M, V, W)
    torques = Vector{T}(num_velocities(state))
    jointWrenches = Dict{RigidBody{M}, Wrench{T}}()
    accelerations = Dict{RigidBody{M}, SpatialAcceleration{T}}()
    inverse_dynamics!(torques, jointWrenches, accelerations, state, v̇, externalWrenches)
    torques
end

type DynamicsResult{M, T}
    massMatrix::Symmetric{T, Matrix{T}}
    massMatrixInversionCache::Symmetric{T, Matrix{T}}
    dynamicsBias::Vector{T}
    biasedTorques::Vector{T}
    ẋ::Vector{T}
    q̇::AbstractVector{T}
    v̇::AbstractVector{T}
    accelerations::Dict{RigidBody{M}, SpatialAcceleration{T}}
    jointWrenches::Dict{RigidBody{M}, Wrench{T}}

    function DynamicsResult(::Type{T}, mechanism::Mechanism{M})
        nq = num_positions(mechanism)
        nv = num_velocities(mechanism)
        massMatrix = Symmetric(zeros(T, nv, nv))
        massMatrixInversionCache = Symmetric(zeros(T, nv, nv))
        ẋ = zeros(T, nq + nv)
        q̇ = view(ẋ, 1 : nq)
        v̇ = view(ẋ, nq + 1 : nq + nv)
        dynamicsBias = zeros(T, nv)
        biasedTorques = zeros(T, nv)
        accelerations = Dict{RigidBody{M}, SpatialAcceleration{T}}()
        sizehint!(accelerations, num_bodies(mechanism))
        jointWrenches = Dict{RigidBody{M}, Wrench{T}}()
        sizehint!(jointWrenches, num_bodies(mechanism))
        new(massMatrix, massMatrixInversionCache, dynamicsBias, biasedTorques, ẋ, q̇, v̇, accelerations, jointWrenches)
    end
end

DynamicsResult{M, T}(t::Type{T}, mechanism::Mechanism{M}) = DynamicsResult{M, T}(t, mechanism)

function joint_accelerations!(out::AbstractVector, massMatrixInversionCache::Symmetric,
        massMatrix::Symmetric, biasedTorques::Vector)
    out[:] = massMatrix \ biasedTorques # TODO: make more efficient
    nothing
end

function joint_accelerations!{T<:Union{Float32, Float64}}(out::AbstractVector{T}, massMatrixInversionCache::Symmetric{T, Matrix{T}},
        massMatrix::Symmetric{T, Matrix{T}}, biasedTorques::Vector{T})
    @inbounds copy!(out, biasedTorques)
    @inbounds copy!(massMatrixInversionCache.data, massMatrix.data)
    Base.LinAlg.LAPACK.posv!(massMatrixInversionCache.uplo, massMatrixInversionCache.data, out)
    nothing
end

function dynamics!{T, X, M, Tau, W}(out::DynamicsResult{T}, state::MechanismState{X, M},
        torques::AbstractVector{Tau} = NullVector{T}(num_velocities(state)),
        externalWrenches::Associative{RigidBody{M}, Wrench{W}} = NullDict{RigidBody{M}, Wrench{T}}())
    configuration_derivative!(out.q̇, state)
    dynamics_bias!(out.dynamicsBias, out.accelerations, out.jointWrenches, state, externalWrenches)
    @inbounds copy!(out.biasedTorques, out.dynamicsBias) # TODO: handle input torques again
    sub!(out.biasedTorques, torques, out.dynamicsBias)
    mass_matrix!(out.massMatrix, state)
    joint_accelerations!(out.v̇, out.massMatrixInversionCache, out.massMatrix, out.biasedTorques)
    nothing
end

# Convenience function that takes a Vector argument for the state and returns a Vector,
# e.g. for use with standard ODE integrators
# Note that preallocatedState is required so that we don't need to allocate a new
# MechanismState object every time this function is called
function dynamics!{T, X, M, W}(ẋ::AbstractVector{X},
        result::DynamicsResult{T}, state::MechanismState{X, M}, stateVec::AbstractVector{X},
        externalWrenches::Associative{RigidBody{M}, Wrench{W}} = NullDict{RigidBody{M}, Wrench{T}}())
    set!(state, stateVec)
    dynamics!(result, state, NullVector{T}(num_velocities(state)), externalWrenches)
    copy!(ẋ, result.ẋ)
    ẋ
end
