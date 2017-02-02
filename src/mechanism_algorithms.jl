function center_of_mass{X, M, C}(state::MechanismState{X, M, C}, itr)
    frame = root_frame(state.mechanism)
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
        v̇::StridedVector{V})
    mechanism = state.mechanism
    T = promote_type(C, V)
    descendantFrame = default_frame(vertex_data(descendant))
    accel = zero(SpatialAcceleration{T}, descendantFrame, descendantFrame, root_frame(mechanism))
    descendant == ancestor && return accel

    current = descendant
    while current != ancestor
        joint = edge_to_parent_data(current)
        v̇joint = UnsafeVectorView(v̇, velocity_range(state, joint)) # TODO: inefficient lookup of velocity range
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

function potential_energy{X, M, C}(state::MechanismState{X, M, C})
    m = mass(state.mechanism)
    gravitationalForce = m * state.mechanism.gravitationalAcceleration
    centerOfMass = transform(state, center_of_mass(state), gravitationalForce.frame)
    -dot(gravitationalForce, FreeVector3D(centerOfMass))
 end

 function force_space_matrix_transpose_mul_jacobian!(out::Matrix, rowstart::Int64, colstart::Int64,
        mat::Union{MomentumMatrix, WrenchMatrix}, jac::GeometricJacobian, sign::Int64)
     @framecheck(jac.frame, mat.frame)
     n = num_cols(mat)
     m = num_cols(jac)
     @boundscheck (rowstart > 0 && rowstart + n - 1 <= size(out, 1)) || error("size mismatch")
     @boundscheck (colstart > 0 && colstart + m - 1 <= size(out, 2)) || error("size mismatch")

     # more efficient version of
     # view(out, rowstart : rowstart + n - 1, colstart : colstart + n - 1)[:] = mat.angular' * jac.angular + mat.linear' * jac.linear

     @inbounds begin
         for col = 1 : m
             outcol = colstart + col - 1
             for row = 1 : n
                 outrow = rowstart + row - 1
                 out[outrow, outcol] = zero(eltype(out))
                 for i = 1 : 3
                     out[outrow, outcol] += flipsign(mat.angular[i, row] * jac.angular[i, col], sign)
                     out[outrow, outcol] += flipsign(mat.linear[i, row] * jac.linear[i, col], sign)
                 end
             end
         end
     end
 end

function mass_matrix!{X, M, C}(out::Symmetric{C, Matrix{C}}, state::MechanismState{X, M, C})
    @boundscheck size(out, 1) == num_velocities(state) || error("mass matrix has wrong size")
    @boundscheck out.uplo == 'L' || error("expected a lower triangular symmetric matrix type as the mass matrix")
    fill!(out.data, zero(C))
    mechanism = state.mechanism

    for vi in non_root_vertices(state)
        # Hii
        jointi = edge_to_parent_data(vi)
        irange = velocity_range(jointi)
        if length(irange) > 0
            Si = motion_subspace(vi)
            Ii = crb_inertia(vi)
            F = Ii * Si
            istart = first(irange)
            force_space_matrix_transpose_mul_jacobian!(out.data, istart, istart, F, Si, 1)

            # Hji, Hij
            vj = parent(vi)
            while (!isroot(vj))
                jointj = edge_to_parent_data(vj)
                jrange = velocity_range(jointj)
                if length(jrange) > 0
                    Sj = motion_subspace(vj)
                    jstart = first(jrange)
                    force_space_matrix_transpose_mul_jacobian!(out.data, istart, jstart, F, Sj, 1)
                end
                vj = parent(vj)
            end
        end
    end
end

function mass_matrix{X, M, C}(state::MechanismState{X, M, C})
    nv = num_velocities(state)
    ret = Symmetric(Matrix{C}(nv, nv), :L)
    mass_matrix!(ret, state)
    ret
end

function momentum_matrix!(out::MomentumMatrix, state::MechanismState)
    @boundscheck num_velocities(state) == num_cols(out) || error("size mismatch")
    pos = 1
    for vertex in non_root_vertices(state)
        part = crb_inertia(vertex) * motion_subspace(vertex)
        @framecheck out.frame part.frame # TODO: transform part to out.frame instead?
        n = 3 * num_cols(part)
        copy!(out.angular, pos, part.angular, 1, n)
        copy!(out.linear, pos, part.linear, 1, n)
        pos += n
    end
end

function momentum_matrix(state::MechanismState)
    ncols = num_velocities(state)
    T = cache_eltype(state)
    angular = Matrix{T}(3, ncols)
    linear = Matrix{T}(3, ncols)
    ret = MomentumMatrix(root_frame(state.mechanism), angular, linear)
    momentum_matrix!(ret, state)
    ret
end

function bias_accelerations!{T, X, M}(out::Associative{RigidBody{M}, SpatialAcceleration{T}}, state::MechanismState{X, M})
    gravityBias = convert(SpatialAcceleration{T}, -gravitational_spatial_acceleration(state.mechanism))
    for vertex in non_root_vertices(state)
        body = vertex_data(vertex).body
        out[body] = gravityBias + bias_acceleration(vertex)
    end
    nothing
end

function spatial_accelerations!{T, X, M}(out::Associative{RigidBody{M}, SpatialAcceleration{T}}, state::MechanismState{X, M}, v̇::StridedVector)
    mechanism = state.mechanism

    # TODO: consider merging back into one loop
    # unbiased joint accelerations + gravity
    out[root_body(mechanism)] = convert(SpatialAcceleration{T}, -gravitational_spatial_acceleration(mechanism))
    for vertex in non_root_vertices(state)
        body = vertex_data(vertex).body
        S = motion_subspace(vertex)
        v̇joint = UnsafeVectorView(v̇, velocity_range(edge_to_parent_data(vertex)))
        jointAccel = SpatialAcceleration(S, v̇joint)
        out[body] = out[vertex_data(parent(vertex)).body] + jointAccel
    end

    # add bias acceleration - gravity
    for vertex in non_root_vertices(state)
        body = vertex_data(vertex).body
        out[body] += bias_acceleration(vertex)
    end
    nothing
end

function newton_euler!{T, X, M, W}(
        out::Associative{RigidBody{M}, Wrench{T}}, state::MechanismState{X, M},
        accelerations::Associative{RigidBody{M}, SpatialAcceleration{T}},
        externalWrenches::Associative{RigidBody{M}, Wrench{W}} = NullDict{RigidBody{M}, Wrench{T}}())
    mechanism = state.mechanism
    for vertex in non_root_vertices(state)
        body = vertex_data(vertex).body
        wrench = newton_euler(vertex, accelerations[body])
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
        torquesOut::StridedVector{T},
        netWrenchesInJointWrenchesOut::Associative{RigidBody{M}, Wrench{T}},
        state::MechanismState{X, M})
    @boundscheck length(torquesOut) == num_velocities(state) || error("torquesOut size is wrong")
    vertices = state.toposortedStateVertices
    for i = length(vertices) : -1 : 2
        vertex = vertices[i]
        body = vertex_data(vertex).body
        jointWrench = netWrenchesInJointWrenchesOut[body]
        if !isroot(parent(vertex))
            parentBody = vertex_data(parent(vertex)).body
            netWrenchesInJointWrenchesOut[parentBody] = netWrenchesInJointWrenchesOut[parentBody] + jointWrench # action = -reaction
        end
        jointState = edge_to_parent_data(vertex)
        jointWrench = transform(jointWrench, inv(transform_to_root(vertex))) # TODO: stay in world frame?
        @inbounds τjoint = UnsafeVectorView(torquesOut, velocity_range(jointState))
        joint_torque!(jointState.joint, τjoint, configuration(jointState), jointWrench)
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

function constraint_jacobian_and_bias!(state::MechanismState, constraintJacobian::AbstractMatrix, constraintBias::AbstractVector)
    # TODO: allocations
    structure = state.constraintJacobianStructure
    rows = rowvals(structure)
    signs = nonzeros(structure)
    nNonTreeEdges = size(structure, 2)
    rowstart = 1
    constraintJacobian[:] = 0
    for i = 1 : nNonTreeEdges
        edge = state.mechanism.nonTreeEdges[i]
        joint = edge.joint
        nextrowstart = rowstart + num_constraints(joint)
        range = rowstart : nextrowstart - 1

        # Constraint wrench subspace.
        jointTransform = relative_transform(state, joint.frameAfter, joint.frameBefore) # TODO: expensive
        T = constraint_wrench_subspace(joint, jointTransform)
        T = transform(T, transform_to_root(state, T.frame)) # TODO: expensive

        # Jacobian rows.
        for j in nzrange(structure, i)
            treevertex = state.toposortedStateVertices[rows[j]]
            J = motion_subspace(treevertex)
            sign = signs[j]
            colstart = first(velocity_range(edge_to_parent_data(treevertex)))
            force_space_matrix_transpose_mul_jacobian!(constraintJacobian, rowstart, colstart, T, J, sign)
        end

        # Constraint bias.
        has_fixed_subspaces(joint) || error("Only joints with fixed motion subspace (Ṡ = 0) supported at this point.")
        kjoint = UnsafeVectorView(constraintBias, range)
        predecessorTwist = twist_wrt_world(state, edge.predecessor)
        successorTwist = twist_wrt_world(state, edge.successor)
        biasAccel = cross(successorTwist, predecessorTwist) + (bias_acceleration(state, edge.successor) + -bias_acceleration(state, edge.predecessor)) # 8.47 in Featherstone
        At_mul_B!(kjoint, T, biasAccel)
        rowstart = nextrowstart
    end
end

function dynamics_solve!(result::DynamicsResult, τ::AbstractVector)
    # version for general scalar types
    # TODO: make more efficient
    M = result.massMatrix
    c = result.dynamicsBias
    v̇ = result.v̇

    K = result.constraintJacobian
    k = result.constraintBias
    λ = result.λ

    nv = size(M, 1)
    nl = size(K, 1)
    A = [M K';
         K zeros(nl, nl)]
    b = [τ - c; -k]
    v̇λ = A \ b
    result.v̇ = view(v̇λ, 1 : nv)
    result.λ = view(v̇λ, nv + 1 : nv + nl)
    nothing
end

function dynamics_solve!{S, T<:LinAlg.BlasReal}(result::DynamicsResult{S, T}, τ::AbstractVector{T})
    # optimized version for BLAS floats
    M = result.massMatrix
    c = result.dynamicsBias
    v̇ = result.v̇

    K = result.constraintJacobian
    k = result.constraintBias
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

        # Compute A = Y Yᵀ == K * M⁻¹ * K'
        LinAlg.BLAS.gemm!('N', 'T', one(T), Y, Y, zero(T), A) # A <- K * M⁻¹ * K'

        # Compute b = Y z + k
        b = λ
        b[:] = k
        LinAlg.BLAS.gemv!('N', one(T), Y, z, one(T), b) # b <- Y z + k

        # Compute λ = A⁻¹ b == (K * M⁻¹ * K')⁻¹ * (K * M⁻¹ * (τ - c) + k)
        LinAlg.LAPACK.posv!(uplo, A, b) # b == λ <- (K * M⁻¹ * K')⁻¹ * (K * M⁻¹ * (τ - c) + k)

        # Update τBiased: subtract K' * λ
        LinAlg.BLAS.gemv!('T', -one(T), K, λ, one(T), τBiased) # τBiased <- τ - c - K' * λ

        # Solve for v̇ = M⁻¹ * (τ - c - K' * λ)
        LinAlg.LAPACK.potrs!(uplo, L, τBiased) # τBiased ==v̇ <- M⁻¹ * (τ - c - K' * λ)
    else
        # No loops.
        LinAlg.LAPACK.potrs!(uplo, L, τBiased) # τBiased == v̇ <- M⁻¹ * (τ - c)
    end
    nothing
end

function dynamics!{T, X, M, Tau, W}(out::DynamicsResult{T}, state::MechanismState{X, M},
        torques::AbstractVector{Tau} = NullVector{T}(num_velocities(state)),
        externalWrenches::Associative{RigidBody{M}, Wrench{W}} = NullDict{RigidBody{M}, Wrench{T}}())
    dynamics_bias!(out.dynamicsBias, out.accelerations, out.jointWrenches, state, externalWrenches)
    mass_matrix!(out.massMatrix, state)
    constraint_jacobian_and_bias!(state, out.constraintJacobian, out.constraintBias)
    dynamics_solve!(out, torques)
    nothing
end

# Convenience function that takes a Vector argument for the state stacked as [q; v]
# and returns a Vector, for use with standard ODE integrators.
function dynamics!{T, X, M, Tau, W}(ẋ::StridedVector{X},
        result::DynamicsResult{T}, state::MechanismState{X, M}, stateVec::AbstractVector{X},
        torques::AbstractVector{Tau} = NullVector{T}(num_velocities(state)),
        externalWrenches::Associative{RigidBody{M}, Wrench{W}} = NullDict{RigidBody{M}, Wrench{T}}())
    set!(state, stateVec)
    nq = num_positions(state)
    nv = num_velocities(state)
    q̇ = view(ẋ, 1 : nq) # allocates
    v̇ = view(ẋ, nq + 1 : nq + nv) # allocates
    configuration_derivative!(q̇, state)
    dynamics!(result, state, torques, externalWrenches)
    copy!(v̇, result.v̇)
    ẋ
end
