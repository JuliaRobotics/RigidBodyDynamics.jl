function subtree_mass{M}(base::Tree{RigidBody{M}, Joint})
    if isroot(base)
        result = 0
    else
        result = base.vertexData.inertia.mass
    end
    for child in base.children
        result += subtree_mass(child)
    end
    return result
end
mass{M}(m::Mechanism{M}) = subtree_mass(m.tree)

function center_of_mass{C}(cache::MechanismStateCache{C}, itr)
    frame = cache.rootFrame
    com = Point3D(frame, zero(Vec{3, C}))
    mass = zero(C)
    for body in itr
        if !isroot(body)
            inertia = body.inertia
            com += inertia.mass * transform(cache, Point3D(inertia.frame, inertia.centerOfMass), frame)
            mass += inertia.mass
        end
    end
    com /= mass
    return com
end

center_of_mass(m::Mechanism, cache::MechanismStateCache) = center_of_mass(cache, bodies(m))

function geometric_jacobian{C, M}(cache::MechanismStateCache{C}, path::Path{RigidBody{M}, Joint})
    flipIfNecessary = (sign::Int64, motionSubspace::GeometricJacobian{C}) -> sign == -1 ? -motionSubspace : motionSubspace
    motionSubspaces = [flipIfNecessary(sign, motion_subspace(cache, joint))::GeometricJacobian{C} for (joint, sign) in zip(path.edgeData, path.directions)]
    return hcat(motionSubspaces...)
end

function mass_matrix{C}(cache::MechanismStateCache{C})
    nv = num_velocities(keys(cache.motionSubspaces))
    H = Array(C, nv, nv)

    for i = 2 : length(cache.toposortedTree)
        vertex_i = cache.toposortedTree[i]
        
        # Hii
        body_i = vertex_i.vertexData
        joint_i = vertex_i.edgeToParentData
        v_start_i = cache.velocityVectorStartIndices[joint_i]
        i_range = v_start_i : v_start_i + num_velocities(joint_i) - 1
        S_i = motion_subspace(cache, joint_i)
        F = crb_inertia(cache, body_i) * S_i
        H[i_range, i_range] = S_i.mat' * F.mat

        # Hji, Hij
        vertex_j = vertex_i.parent
        while (!isroot(vertex_j))
            joint_j = vertex_j.edgeToParentData
            v_start_j = cache.velocityVectorStartIndices[joint_j]
            j_range = v_start_j : v_start_j + num_velocities(joint_j) - 1
            S_j = motion_subspace(cache, joint_j)
            @assert F.frame == S_j.frame
            Hji = At_mul_B(S_j.mat, F.mat)
            H[j_range, i_range] = Hji
            H[i_range, j_range] = Hji'
            vertex_j = vertex_j.parent
        end
    end
    return H
end

function momentum_matrix(cache::MechanismStateCache)
    bodiesAndJoints = [(vertex.vertexData::RigidBody, vertex.edgeToParentData::Joint) for vertex in cache.toposortedTree[2 : end]]
    return hcat([spatial_inertia(cache, body) * motion_subspace(cache, joint) for (body, joint) in bodiesAndJoints]...)
end
