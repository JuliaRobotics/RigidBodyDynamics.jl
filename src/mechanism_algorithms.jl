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
    joints = path.edgeData
    signs = path.directions
    cols = num_velocities(joints)
    mat = Array(C, 6, cols)
    col_start = 1
    for i = 1 : length(joints)
        joint = joints[i]
        motionSubspace = motion_subspace(cache, joint)
        nv = num_velocities(joint)
        mat[:, col_start : col_start + nv - 1] = copysign(motionSubspace.mat, signs[i])
        col_start += nv
    end
    body = signs[end] == 1 ? joints[end].frameAfter : joints[end].frameBefore
    base = signs[1] == 1 ? joints[1].frameBefore : joints[1].frameAfter
    ret = GeometricJacobian(body, base, cache.rootFrame, mat)
end

function mass_matrix{C}(cache::MechanismStateCache{C})
    nv = num_velocities(keys(cache.motionSubspaces))
    H = Array(C, nv, nv)

    for vertex_i in cache.toposortedTree
        if !isroot(vertex_i)
            # Hii
            body_i = vertex_i.vertexData
            joint_i = vertex_i.edgeToParentData
            v_start_i = cache.velocityVectorStartIndices[joint_i]
            i_range = v_start_i : v_start_i + num_velocities(joint_i) - 1
            I_i = crb_inertia(cache, body_i)
            S_i = motion_subspace(cache, joint_i)
            @assert I_i.frame == S_i.frame
            F = to_matrix(I_i) * S_i.mat
            H[i_range, i_range] = S_i.mat' * F

            # Hji, Hij
            vertex_j = vertex_i.parent
            while (!isroot(vertex_j))
                body_j = vertex_j.vertexData
                joint_j = vertex_j.edgeToParentData
                v_start_j = cache.velocityVectorStartIndices[joint_j]
                j_range = v_start_j : v_start_j + num_velocities(joint_j) - 1
                S_j = motion_subspace(cache, joint_j)
                @assert I_i.frame == S_j.frame
                Hji = S_j.mat' * F
                H[j_range, i_range] = Hji
                H[i_range, j_range] = Hji'
                vertex_j = vertex_j.parent
            end
        end
    end
    return H
end
